#!/usr/bin/env python3
"""Create full-run speed and steering plots from the latest monitor CSV."""

import csv
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def main():
    def read_rows(path):
        with path.open(newline="") as handle:
            return list(csv.DictReader(handle))

    if len(sys.argv) == 2:
        csv_path = Path(sys.argv[1])
        rows = read_rows(csv_path)
        if not rows:
            raise RuntimeError(f"No samples in {csv_path}")
    else:
        csv_files = sorted(DATA_DIR.glob("control_feedback_*.csv"),
                           key=lambda p: p.stat().st_mtime, reverse=True)
        if not csv_files:
            raise RuntimeError(f"No control_feedback_*.csv files in {DATA_DIR}")
        csv_path = None
        rows = []
        for candidate in csv_files:
            rows = read_rows(candidate)
            if rows:
                csv_path = candidate
                break
        if csv_path is None:
            raise RuntimeError(f"No samples in any control_feedback_*.csv files in {DATA_DIR}")

    def has_column(name):
        return name in rows[0]

    def values(name, default=float("nan")):
        if not has_column(name):
            return [default for _ in rows]
        output = []
        for row in rows:
            raw_value = row.get(name, "")
            try:
                output.append(float(raw_value))
            except (TypeError, ValueError):
                output.append(default)
        return output

    def degrees(series):
        return [math.degrees(value) if math.isfinite(value) else value
                for value in series]

    def any_finite(series):
        return any(math.isfinite(value) for value in series)

    def is_command_type(series, command_type):
        return any(math.isfinite(value) and int(round(value)) == command_type
                   for value in series)

    time_s = values("elapsed_s")
    command_types = values("command_type")
    uses_pedal_cmd = is_command_type(command_types, 1)
    uses_velocity_cmd = is_command_type(command_types, 2)
    cmd_speed_mps = (
        values("cmd_speed_mps")
        if has_column("cmd_speed_mps")
        else [value / 3.6 for value in values("ctrl_velocity_kph")]
    )
    target_speed_mps = values("target_speed_mps")
    has_target_speed = has_column("target_speed_mps") and any_finite(target_speed_mps)
    mission_speed_target_mps = values("mission_speed_target_mps")
    has_mission_speed_target = (
        has_column("mission_speed_target_mps")
        and any_finite(mission_speed_target_mps)
    )
    curve_speed_limit_mps = values("curve_speed_limit_mps")
    has_curve_speed_limit = (
        has_column("curve_speed_limit_mps")
        and any_finite(curve_speed_limit_mps)
    )
    actual_speed_mps = values("actual_speed_mps")
    speed_error_mps = (
        values("speed_error_mps")
        if has_column("speed_error_mps")
        else [cmd - actual for cmd, actual in zip(cmd_speed_mps, actual_speed_mps)]
    )

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(14, 11))

    if uses_velocity_cmd:
        axes[0].plot(time_s, cmd_speed_mps, label="CtrlCmd velocity [m/s]")
    if has_target_speed:
        axes[0].plot(time_s, target_speed_mps,
                     label="MPC target speed [m/s]")
    if has_mission_speed_target:
        axes[0].plot(time_s, mission_speed_target_mps, "-.",
                     label="Mission-state speed target [m/s]")
    if has_curve_speed_limit:
        axes[0].plot(time_s, curve_speed_limit_mps, ":",
                     label="Curvature speed limit [m/s]")
    axes[0].plot(time_s, actual_speed_mps,
                 label="Competition feedback speed [m/s]")
    if uses_velocity_cmd or has_target_speed:
        axes[0].plot(time_s, speed_error_mps, "--", alpha=0.75,
                     label="speed error target/cmd - feedback [m/s]")
    if uses_pedal_cmd and not has_target_speed:
        axes[0].text(
            0.5, 0.9,
            "longlCmdType=1: this CSV has no MPC target speed column\n"
            "record a new CSV with the updated monitor node",
            ha="center", va="top", transform=axes[0].transAxes,
        )
    axes[0].set_ylabel("speed [m/s]")
    axes[0].grid(True)
    axes[0].legend()

    ctrl_steering_deg = degrees(values("ctrl_steering"))
    competition_wheel_angle_deg = values("wheel_angle")
    competition_front_steer_deg = values("front_steer")
    # MORAI EgoVehicleStatus wheel_angle uses the opposite sign convention from
    # CtrlCmd steering in this setup.  Plot the converted value prominently so
    # command-vs-feedback comparison follows the same steering convention.
    competition_wheel_angle_cmd_sign_deg = [
        -value if math.isfinite(value) else value
        for value in competition_wheel_angle_deg
    ]
    axes[1].plot(time_s, ctrl_steering_deg,
                 label="CtrlCmd steering [deg]")
    # MORAI EgoVehicleStatus reports wheel_angle/front_steer_angle in degrees.
    # /ctrl_cmd.steering is radians, so only the command is converted here.
    axes[1].plot(time_s, competition_wheel_angle_cmd_sign_deg,
                 label="-Competition wheel angle [deg]")
    axes[1].plot(time_s, competition_wheel_angle_deg,
                 ":", alpha=0.35, label="Competition wheel angle raw [deg]")
    axes[1].plot(time_s, competition_front_steer_deg,
                 label="Competition front steer raw [deg]", alpha=0.7)
    axes[1].set_ylabel("steering [deg]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(time_s, values("ctrl_accel"), label="CtrlCmd accel")
    axes[2].plot(time_s, values("ctrl_brake"), label="CtrlCmd brake")
    axes[2].plot(time_s, values("feedback_accel"),
                 label="Competition accel", alpha=0.7)
    axes[2].plot(time_s, values("feedback_brake"),
                 label="Competition brake", alpha=0.7)
    axes[2].set_ylabel("pedal [0..1]")
    axes[2].grid(True)
    axes[2].legend()

    if has_column("cmd_age_s") and has_column("feedback_age_s"):
        axes[3].plot(time_s, values("cmd_age_s"), label="CtrlCmd age [s]")
        axes[3].plot(time_s, values("feedback_age_s"),
                     label="Competition age [s]")
        if has_column("target_speed_age_s"):
            axes[3].plot(time_s, values("target_speed_age_s"),
                         label="MPC target speed age [s]", alpha=0.8)
        if has_column("mission_speed_age_s"):
            axes[3].plot(time_s, values("mission_speed_age_s"),
                         label="Mission speed age [s]", alpha=0.8)
        if has_column("curve_speed_limit_age_s"):
            axes[3].plot(time_s, values("curve_speed_limit_age_s"),
                         label="Curvature speed limit age [s]", alpha=0.8)
        axes[3].set_ylabel("topic age [s]")
        axes[3].grid(True)
        axes[3].legend()
    else:
        axes[3].axis("off")
        axes[3].text(
            0.5, 0.5,
            "topic age columns are not available in this CSV\n"
            "record a new CSV with the updated monitor node",
            ha="center", va="center", transform=axes[3].transAxes,
        )

    axes[3].set_xlabel("elapsed time [s]")
    figure.suptitle("MPC command vs MORAI Competition feedback")
    figure.tight_layout()
    output = csv_path.with_suffix(".png")
    figure.savefig(output, dpi=180)
    print(output)


if __name__ == "__main__":
    main()
