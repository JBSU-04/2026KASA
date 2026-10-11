#!/usr/bin/env python3
"""Generate reproducible MORAI GPS/IMU EKF report graphs.

The tool supports a live ROS capture and an offline rosbag. It writes aligned
CSV data, a machine-readable JSON summary and four report-ready PNG figures.
No ground-truth pose from Competition_topic is used for localization metrics;
that topic contributes vehicle speed only.
"""

import argparse
import csv
import json
import math
import os
import threading
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Proj


RAW_COLOR = '#D84A4A'
EKF_COLOR = '#198F8A'
NAVY = '#153B5B'
GRID = '#DDE4E8'


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def message_time(message, fallback):
    if hasattr(message, 'header'):
        value = message.header.stamp.to_sec()
        if value > 0.0:
            return value
    return float(fallback)


class Samples:
    def __init__(self, zone=52, east_offset=302595.0, north_offset=4124145.0):
        self.lock = threading.RLock()
        self.proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
        self.east_offset = float(east_offset)
        self.north_offset = float(north_offset)
        self.gps = []
        self.imu = []
        self.fused = []
        self.speed = []
        self.valid = []
        self.reference = []

    def add_gps(self, message, fallback):
        east, north = self.proj(message.longitude, message.latitude)
        eo = float(message.eastOffset) if message.eastOffset != 0.0 else self.east_offset
        no = float(message.northOffset) if message.northOffset != 0.0 else self.north_offset
        with self.lock:
            self.gps.append((message_time(message, fallback), east - eo, north - no))

    def add_imu(self, message, fallback):
        with self.lock:
            self.imu.append((message_time(message, fallback),
                             yaw_from_quaternion(message.orientation)))

    def add_fused(self, message, fallback):
        pose = message.pose.pose
        with self.lock:
            self.fused.append((message_time(message, fallback),
                               pose.position.x, pose.position.y,
                               yaw_from_quaternion(pose.orientation)))

    def add_speed(self, message, fallback):
        with self.lock:
            self.speed.append((message_time(message, fallback), float(message.velocity.x)))

    def add_valid(self, message, fallback):
        with self.lock:
            self.valid.append((float(fallback), 1.0 if message.data else 0.0))

    def set_reference(self, message):
        points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
        if len(points) >= 2:
            with self.lock:
                self.reference = points


def capture_live(args):
    import rospy
    from morai_msgs.msg import EgoVehicleStatus, GPSMessage
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool

    data = Samples(args.utm_zone, args.east_offset, args.north_offset)
    rospy.init_node('ekf_report_graphs', anonymous=True)
    now = lambda: rospy.Time.now().to_sec()
    rospy.Subscriber(args.gps_topic, GPSMessage,
                     lambda m: data.add_gps(m, now()), queue_size=200)
    rospy.Subscriber(args.imu_topic, Imu,
                     lambda m: data.add_imu(m, now()), queue_size=500)
    rospy.Subscriber(args.fused_topic, Odometry,
                     lambda m: data.add_fused(m, now()), queue_size=500)
    rospy.Subscriber(args.speed_topic, EgoVehicleStatus,
                     lambda m: data.add_speed(m, now()), queue_size=200)
    rospy.Subscriber(args.valid_topic, Bool,
                     lambda m: data.add_valid(m, now()), queue_size=100)
    rospy.Subscriber(args.path_topic, Path, data.set_reference, queue_size=1)
    rospy.loginfo('[ekf_report_graphs] collecting %.1f seconds', args.live_seconds)
    end = rospy.Time.now() + rospy.Duration(args.live_seconds)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        rospy.sleep(0.05)
    return data


def load_bag(args):
    import rosbag

    data = Samples(args.utm_zone, args.east_offset, args.north_offset)
    topics = [args.gps_topic, args.imu_topic, args.fused_topic, args.speed_topic,
              args.valid_topic, args.path_topic]
    with rosbag.Bag(args.bag, 'r') as bag:
        for topic, message, stamp in bag.read_messages(topics=topics):
            fallback = stamp.to_sec()
            if topic == args.gps_topic:
                data.add_gps(message, fallback)
            elif topic == args.imu_topic:
                data.add_imu(message, fallback)
            elif topic == args.fused_topic:
                data.add_fused(message, fallback)
            elif topic == args.speed_topic:
                data.add_speed(message, fallback)
            elif topic == args.valid_topic:
                data.add_valid(message, fallback)
            elif topic == args.path_topic:
                data.set_reference(message)
    return data


def load_reference_csv(path):
    points = []
    with open(path, newline='') as handle:
        for row in csv.reader(handle):
            numeric = []
            for value in row:
                try:
                    numeric.append(float(value))
                except ValueError:
                    pass
            if len(numeric) >= 2:
                points.append((numeric[0], numeric[1]))
    return points


def sorted_array(values, columns):
    if not values:
        return np.empty((0, columns), dtype=float)
    output = np.asarray(sorted(values, key=lambda item: item[0]), dtype=float)
    _, unique = np.unique(output[:, 0], return_index=True)
    return output[np.sort(unique)]


def interpolate(source, target_times, column):
    return np.interp(target_times, source[:, 0], source[:, column])


def moving_average(values, window):
    window = max(3, int(window))
    if window % 2 == 0:
        window += 1
    if len(values) <= window:
        return np.repeat(np.median(values, axis=0)[None, :], len(values), axis=0)
    kernel = np.ones(window, dtype=float) / window
    result = np.column_stack([
        np.convolve(np.pad(values[:, i], (window // 2,), mode='edge'),
                    kernel, mode='valid') for i in range(values.shape[1])])
    return result


def polyline_distance(points, reference):
    points = np.asarray(points, dtype=float)
    reference = np.asarray(reference, dtype=float)
    if len(reference) < 2:
        return np.full(len(points), np.nan)
    starts = reference[:-1]
    vectors = reference[1:] - starts
    lengths = np.sum(vectors * vectors, axis=1)
    lengths[lengths < 1.0e-12] = 1.0e-12
    result = np.empty(len(points), dtype=float)
    for begin in range(0, len(points), 128):
        chunk = points[begin:begin + 128]
        delta = chunk[:, None, :] - starts[None, :, :]
        ratio = np.sum(delta * vectors[None, :, :], axis=2) / lengths[None, :]
        ratio = np.clip(ratio, 0.0, 1.0)
        projected = starts[None, :, :] + ratio[:, :, None] * vectors[None, :, :]
        dist2 = np.sum((chunk[:, None, :] - projected) ** 2, axis=2)
        result[begin:begin + len(chunk)] = np.sqrt(np.min(dist2, axis=1))
    return result


def error_metrics(error):
    error = np.asarray(error, dtype=float)
    error = error[np.isfinite(error)]
    return {
        'rmse_m': float(np.sqrt(np.mean(error ** 2))),
        'mae_m': float(np.mean(np.abs(error))),
        'p95_m': float(np.percentile(np.abs(error), 95)),
        'max_m': float(np.max(np.abs(error))),
    }


def configure_plot_style():
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Noto Sans CJK KR', 'NanumGothic', 'DejaVu Sans'],
        'axes.edgecolor': '#90A4AE',
        'axes.labelcolor': '#263238',
        'xtick.color': '#52646D',
        'ytick.color': '#52646D',
        'grid.color': GRID,
        'grid.linewidth': 0.7,
        'axes.unicode_minus': False,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
        'savefig.facecolor': 'white',
    })


def analyze_and_plot(data, args):
    gps = sorted_array(data.gps, 3)
    imu = sorted_array(data.imu, 2)
    fused = sorted_array(data.fused, 4)
    speed = sorted_array(data.speed, 2)
    valid = sorted_array(data.valid, 2)
    reference = data.reference
    if args.reference_csv:
        reference = load_reference_csv(args.reference_csv)
    if len(gps) < 10 or len(fused) < 10:
        raise RuntimeError('at least 10 /gps and /odom_fused samples are required')

    start = max(gps[0, 0], fused[0, 0]) + max(0.0, args.settle_seconds)
    end = min(gps[-1, 0], fused[-1, 0])
    mask = (gps[:, 0] >= start) & (gps[:, 0] <= end)
    gps = gps[mask]
    if len(gps) < 10:
        raise RuntimeError('not enough aligned samples after settling interval')
    times = gps[:, 0]
    t = times - times[0]
    raw_xy = gps[:, 1:3]
    fused_xy = np.column_stack([interpolate(fused, times, 1),
                                interpolate(fused, times, 2)])
    fused_yaw = np.interp(times, fused[:, 0], np.unwrap(fused[:, 3]))
    raw_yaw = (np.interp(times, imu[:, 0], np.unwrap(imu[:, 1]))
               if len(imu) >= 2 else np.full(len(times), np.nan))
    aligned_speed = (np.interp(times, speed[:, 0], speed[:, 1])
                     if len(speed) >= 2 else np.zeros(len(times)))
    stationary = float(np.percentile(np.abs(aligned_speed), 95)) < args.stationary_speed
    reference_distance = (polyline_distance(raw_xy, reference)
                          if len(reference) >= 2 else np.empty(0))
    reference_median_m = (float(np.median(reference_distance))
                          if len(reference_distance) else None)
    reference_usable = (reference_median_m is not None and
                        reference_median_m <= args.reference_max_distance)

    if stationary:
        raw_center = np.median(raw_xy, axis=0)
        fused_center = np.median(fused_xy, axis=0)
        raw_error_xy = raw_xy - raw_center
        fused_error_xy = fused_xy - fused_center
        raw_error = np.linalg.norm(raw_error_xy, axis=1)
        fused_error = np.linalg.norm(fused_error_xy, axis=1)
        metric_name = '정지 위치 편차'
        evaluation = 'stationary_stability'
    elif reference_usable:
        raw_error = reference_distance
        fused_error = polyline_distance(fused_xy, reference)
        raw_error_xy = raw_xy - moving_average(raw_xy, max(3, int(len(raw_xy) / max(t[-1], 1.0))))
        fused_error_xy = fused_xy - moving_average(fused_xy, max(3, int(len(fused_xy) / max(t[-1], 1.0))))
        raw_center = fused_center = np.median(fused_xy, axis=0)
        metric_name = '기준경로 CTE'
        evaluation = 'reference_cte'
    else:
        hz = len(times) / max(t[-1], 1.0)
        raw_trend = moving_average(raw_xy, max(3, int(hz)))
        fused_trend = moving_average(fused_xy, max(3, int(hz)))
        raw_error_xy = raw_xy - raw_trend
        fused_error_xy = fused_xy - fused_trend
        raw_error = np.linalg.norm(raw_error_xy, axis=1)
        fused_error = np.linalg.norm(fused_error_xy, axis=1)
        raw_center = fused_center = np.median(fused_xy, axis=0)
        metric_name = '1초 추세 대비 위치 잔차'
        evaluation = 'high_frequency_residual'

    raw_metrics = error_metrics(raw_error)
    fused_metrics = error_metrics(fused_error)
    reduction = 100.0 * (1.0 - fused_metrics['rmse_m'] / max(raw_metrics['rmse_m'], 1.0e-12))
    valid_ratio = (float(np.mean(valid[:, 1])) if len(valid) else None)
    yaw_window = max(3, int(len(times) / max(t[-1], 1.0)))
    if stationary:
        raw_yaw_plot = np.unwrap(raw_yaw) - np.nanmedian(np.unwrap(raw_yaw))
        fused_yaw_plot = fused_yaw - np.median(fused_yaw)
        yaw_metric_label = '중앙값 기준 Yaw'
    else:
        raw_unwrapped = np.unwrap(raw_yaw)
        raw_yaw_plot = raw_unwrapped - moving_average(raw_unwrapped[:, None], yaw_window)[:, 0]
        fused_yaw_plot = fused_yaw - moving_average(fused_yaw[:, None], yaw_window)[:, 0]
        yaw_metric_label = '1초 추세 대비 Yaw 잔차'
    raw_yaw_std = float(np.nanstd(raw_yaw_plot, ddof=1)) if np.any(np.isfinite(raw_yaw_plot)) else None
    fused_yaw_std = float(np.std(fused_yaw_plot, ddof=1))
    summary = {
        'source': os.path.abspath(args.bag) if args.bag else 'live_ros',
        'evaluation': evaluation,
        'metric_label': metric_name,
        'duration_s': float(t[-1]),
        'aligned_samples': int(len(times)),
        'gps_samples_total': int(len(data.gps)),
        'fused_samples_total': int(len(data.fused)),
        'speed_p95_mps': float(np.percentile(np.abs(aligned_speed), 95)),
        'reference_path_used': bool(reference_usable),
        'reference_path_median_distance_m': reference_median_m,
        'raw': raw_metrics,
        'ekf': fused_metrics,
        'rmse_reduction_percent': float(reduction),
        'raw_std_x_m': float(np.std(raw_error_xy[:, 0], ddof=1)),
        'raw_std_y_m': float(np.std(raw_error_xy[:, 1], ddof=1)),
        'ekf_std_x_m': float(np.std(fused_error_xy[:, 0], ddof=1)),
        'ekf_std_y_m': float(np.std(fused_error_xy[:, 1], ddof=1)),
        'raw_yaw_std_rad': raw_yaw_std,
        'ekf_yaw_std_rad': fused_yaw_std,
        'localization_valid_ratio': valid_ratio,
    }

    os.makedirs(args.output_dir, exist_ok=True)
    with open(os.path.join(args.output_dir, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(args.output_dir, 'aligned_samples.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['time_s', 'raw_x_m', 'raw_y_m', 'ekf_x_m', 'ekf_y_m',
                         'raw_error_m', 'ekf_error_m', 'speed_mps'])
        for row in zip(t, raw_xy[:, 0], raw_xy[:, 1], fused_xy[:, 0], fused_xy[:, 1],
                       raw_error, fused_error, aligned_speed):
            writer.writerow(['{:.9f}'.format(value) for value in row])

    configure_plot_style()
    dpi = args.dpi
    # 1) XY trajectory / stationary cloud
    fig, ax = plt.subplots(figsize=(8.4, 4.4))
    if stationary:
        ax.plot(raw_xy[:, 0] - raw_center[0], raw_xy[:, 1] - raw_center[1], '.',
                ms=4, alpha=.45, color=RAW_COLOR, label='Raw GPS')
        ax.plot(fused_xy[:, 0] - fused_center[0], fused_xy[:, 1] - fused_center[1], '-',
                lw=1.7, alpha=.9, color=EKF_COLOR, label='GPS–IMU EKF')
        ax.set_xlabel('중심 기준 East 편차 [m]')
        ax.set_ylabel('중심 기준 North 편차 [m]')
        ax.set_title('정지 상태 위치 분포')
    else:
        ax.plot(raw_xy[:, 0], raw_xy[:, 1], '.', ms=3, alpha=.35,
                color=RAW_COLOR, label='Raw GPS')
        ax.plot(fused_xy[:, 0], fused_xy[:, 1], '-', lw=1.7,
                color=EKF_COLOR, label='GPS–IMU EKF')
        if reference_usable:
            ref = np.asarray(reference)
            margin = 20.0
            local = ((ref[:, 0] >= np.min(raw_xy[:, 0]) - margin) &
                     (ref[:, 0] <= np.max(raw_xy[:, 0]) + margin) &
                     (ref[:, 1] >= np.min(raw_xy[:, 1]) - margin) &
                     (ref[:, 1] <= np.max(raw_xy[:, 1]) + margin))
            local_ref = ref[local]
            if len(local_ref) >= 2:
                ax.plot(local_ref[:, 0], local_ref[:, 1], '--', lw=1.0,
                        color='#607D8B', label='Reference')
        ax.set_xlabel('Map X [m]')
        ax.set_ylabel('Map Y [m]')
        ax.set_title('Raw GPS와 EKF 주행 궤적')
    ax.axis('equal'); ax.grid(True); ax.legend(frameon=False, loc='best')
    fig.tight_layout(); fig.savefig(os.path.join(args.output_dir, 'trajectory_comparison.png'), dpi=dpi)
    plt.close(fig)

    # 2) X/Y residual time series
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 4.8), sharex=True)
    labels = ('East / X', 'North / Y')
    for index, ax in enumerate(axes):
        ax.plot(t, raw_error_xy[:, index], color=RAW_COLOR, lw=.8, alpha=.65, label='Raw GPS')
        ax.plot(t, fused_error_xy[:, index], color=EKF_COLOR, lw=1.2, label='GPS–IMU EKF')
        ax.set_ylabel(labels[index] + ' [m]'); ax.grid(True)
    axes[0].legend(frameon=False, ncol=2, loc='upper right')
    axes[-1].set_xlabel('시간 [s]')
    fig.suptitle('위치 변동 시계열', color=NAVY, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, .95]); fig.savefig(os.path.join(args.output_dir, 'position_time_series.png'), dpi=dpi)
    plt.close(fig)

    # 3) Metrics
    keys = ['rmse_m', 'mae_m', 'p95_m', 'max_m']
    names = ['RMSE', 'MAE', 'P95', '최대오차']
    x = np.arange(len(keys)); width = .34
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    raw_values = [raw_metrics[k] for k in keys]
    ekf_values = [fused_metrics[k] for k in keys]
    bars1 = ax.bar(x - width / 2, raw_values, width, color=RAW_COLOR, label='Raw GPS')
    bars2 = ax.bar(x + width / 2, ekf_values, width, color=EKF_COLOR, label='GPS–IMU EKF')
    for bars in (bars1, bars2):
        for bar in bars:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), '{:.3f}'.format(bar.get_height()),
                    ha='center', va='bottom', fontsize=8, color='#263238')
    ax.set_xticks(x); ax.set_xticklabels(names); ax.set_ylabel('오차 [m]'); ax.grid(True, axis='y')
    ax.set_title('{} · RMSE {:.1f}% 감소'.format(metric_name, reduction))
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout(); fig.savefig(os.path.join(args.output_dir, 'stability_metrics.png'), dpi=dpi)
    plt.close(fig)

    # 4) Yaw series
    fig, ax = plt.subplots(figsize=(8.4, 3.5))
    if np.any(np.isfinite(raw_yaw_plot)):
        ax.plot(t, np.degrees(raw_yaw_plot), color=RAW_COLOR, lw=.8, alpha=.6, label='IMU quaternion yaw')
    ax.plot(t, np.degrees(fused_yaw_plot), color=EKF_COLOR, lw=1.2, label='EKF yaw')
    ax.set_xlabel('시간 [s]'); ax.set_ylabel(yaw_metric_label + ' [deg]'); ax.grid(True)
    ax.set_title('Yaw 안정성'); ax.legend(frameon=False, ncol=2)
    fig.tight_layout(); fig.savefig(os.path.join(args.output_dir, 'yaw_stability.png'), dpi=dpi)
    plt.close(fig)

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--live-seconds', type=float, metavar='SECONDS')
    source.add_argument('--bag', help='input rosbag path')
    parser.add_argument('--output-dir', default='ekf_report_graphs')
    parser.add_argument('--reference-csv', default='')
    parser.add_argument('--reference-max-distance', type=float, default=10.0,
                        help='reject a reference path when its median GPS distance exceeds this value [m]')
    parser.add_argument('--settle-seconds', type=float, default=3.0)
    parser.add_argument('--stationary-speed', type=float, default=0.5)
    parser.add_argument('--utm-zone', type=int, default=52)
    parser.add_argument('--east-offset', type=float, default=302595.0)
    parser.add_argument('--north-offset', type=float, default=4124145.0)
    parser.add_argument('--gps-topic', default='/gps')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--fused-topic', default='/odom_fused')
    parser.add_argument('--speed-topic', default='/Competition_topic')
    parser.add_argument('--valid-topic', default='/localization/valid')
    parser.add_argument('--path-topic', default='/global_path')
    parser.add_argument('--dpi', type=int, default=180)
    return parser.parse_args()


def main():
    args = parse_args()
    data = capture_live(args) if args.live_seconds is not None else load_bag(args)
    analyze_and_plot(data, args)


if __name__ == '__main__':
    main()
