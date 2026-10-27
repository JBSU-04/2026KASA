#ifndef MPC_PATH_TRACKING_STEERING_DELAY_H
#define MPC_PATH_TRACKING_STEERING_DELAY_H

// Steering delay compensation ported from src_0929 (민성 카사) control_model_utils.h.
// A newly solved command cannot affect the vehicle until the commands already in
// the actuator pipeline have been applied. Integrate those past commands over the
// delay and start the MPC from that future pose.

#include <array>
#include <cmath>
#include <deque>

namespace mpc_tracking {

struct Pose2 { double x; double y; double yaw; };
// Steering uses the MPC sign convention: yaw_rate = -v * delta / Lf.
struct TimedSteering { double time_s; double delta_rad; };

// Integrate commands already in the actuator pipeline. Hold measured speed because
// this delay is identified for steering, not for the pedal controller.
inline Pose2 predictSteeringDelay(double speed, double lf, double now,
                                  double delay, const std::deque<TimedSteering>& history,
                                  double reference_offset = 0.0)
{
    Pose2 pose{0.0, 0.0, 0.0};
    if (delay <= 0.0 || lf <= 0.0) return pose;
    double source_time = now - delay;
    double delta = 0.0;
    size_t next = 0;
    while (next < history.size() && history[next].time_s <= source_time) {
        delta = history[next++].delta_rad;
    }
    while (source_time < now) {
        const double end = next < history.size()
            ? std::min(now, history[next].time_s) : now;
        const double duration = end - source_time;
        const double rate = -speed * delta / lf;
        const double yaw_end = pose.yaw + rate * duration;
        if (std::fabs(rate) < 1e-9) {
            pose.x += speed * std::cos(pose.yaw) * duration;
            pose.y += speed * std::sin(pose.yaw) * duration;
        } else {
            pose.x += speed / rate * (std::sin(yaw_end) - std::sin(pose.yaw));
            pose.y += speed / rate * (std::cos(pose.yaw) - std::cos(yaw_end));
        }
        pose.x += reference_offset * (std::cos(yaw_end) - std::cos(pose.yaw));
        pose.y += reference_offset * (std::sin(yaw_end) - std::sin(pose.yaw));
        pose.yaw = yaw_end;
        source_time = end;
        if (next < history.size() && history[next].time_s <= source_time) {
            delta = history[next++].delta_rad;
        }
    }
    return pose;
}

// Motion over `seconds` from measured speed and yaw rate (vehicle frame, CCW yaw).
// Used for the state age: the pose in /odom already lags the vehicle by this time.
inline Pose2 measuredMotion(double speed, double yaw_rate, double seconds)
{
    const double angle = yaw_rate * seconds;
    if (std::fabs(yaw_rate) < 1e-6) return {speed * seconds, 0.0, angle};
    return {speed / yaw_rate * std::sin(angle), speed / yaw_rate * (1.0 - std::cos(angle)), angle};
}

// Pose b expressed in the frame of pose a.
inline Pose2 compose(const Pose2& a, const Pose2& b)
{
    return {a.x + std::cos(a.yaw) * b.x - std::sin(a.yaw) * b.y,
            a.y + std::sin(a.yaw) * b.x + std::cos(a.yaw) * b.y, a.yaw + b.yaw};
}

// cte/epsi at a vehicle-frame pose, matching mpc.cpp: cte = f(x) - y,
// epsi = psi - atan(f'(x)).
inline std::array<double, 2> trackingErrorsAt(const Pose2& pose, const std::array<double, 4>& c)
{
    const double path_y = c[0] + pose.x * (c[1] + pose.x * (c[2] + pose.x * c[3]));
    const double slope = c[1] + pose.x * (2.0 * c[2] + pose.x * 3.0 * c[3]);
    return {{path_y - pose.y, pose.yaw - std::atan(slope)}};
}

}  // namespace mpc_tracking

#endif
