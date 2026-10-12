import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import roundabout_launch_assist as la  # noqa: E402

P = dict(circulate_window_s=3.0, launch_speed_max=5.0, min_margin=0.5, accel_max=1.0, base=0.6, gain=0.2)


def test_commit_launch_raises_pedal():
    # 승인 직후 정지 상태, 상한 8.5: 수동 시연처럼 풀 페달
    assert la.assist_accel(0.2, 0.0, la.COMMIT, None, 0.0, 8.5, P) == 1.0


def test_small_margin_keeps_mpc():
    assert la.assist_accel(0.3, 0.0, la.COMMIT, None, 2.0, 2.4, P) is None


def test_brake_keeps_mpc():
    assert la.assist_accel(0.0, 0.3, la.COMMIT, None, 1.0, 8.5, P) is None


def test_yield_keeps_mpc():
    assert la.assist_accel(0.1, 0.0, 2, None, 0.0, 8.5, P) is None


def test_circulate_window_and_speed_limit():
    assert la.assist_accel(0.3, 0.0, la.CIRCULATE, 1.0, 3.0, 8.5, P) is not None
    assert la.assist_accel(0.3, 0.0, la.CIRCULATE, 4.0, 3.0, 8.5, P) is None
    assert la.assist_accel(0.3, 0.0, la.COMMIT, None, 5.2, 8.5, P) is None


def test_never_lowers_mpc_pedal():
    assert la.assist_accel(0.9, 0.0, la.COMMIT, None, 1.0, 2.0, P) == 0.9


ACC_P = dict(P, acc_min_accel=0.2, acc_min_margin=0.2, acc_base=0.15, acc_gain=0.25)


def test_acc_gap_open_accelerates():
    # 2026-10-12: 순환 중 앞차가 멀어지면(a_des 2.0) 페달을 보탠다.
    assert abs(la.acc_accel(0.1, 0.0, la.CIRCULATE, (True, 2.0, 7.0), 5.0, 8.5, ACC_P) - 0.65) < 1e-9


def test_acc_close_gap_or_brake_defers_to_mpc():
    assert la.acc_accel(0.1, 0.0, la.CIRCULATE, (True, -1.0, 4.0), 5.0, 4.0, ACC_P) is None
    assert la.acc_accel(0.0, 0.2, la.CIRCULATE, (True, 2.0, 7.0), 5.0, 8.5, ACC_P) is None
    assert la.acc_accel(0.1, 0.0, la.CIRCULATE, (False, 2.0, 7.0), 5.0, 8.5, ACC_P) is None
    assert la.acc_accel(0.1, 0.0, 2, (True, 2.0, 7.0), 5.0, 8.5, ACC_P) is None
