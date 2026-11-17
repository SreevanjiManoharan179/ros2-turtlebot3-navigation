import math

from tb3_point_to_point import metrics as M


def test_wrap_angle():
    assert math.isclose(M.wrap_angle(3 * math.pi), math.pi, abs_tol=1e-9) or \
        math.isclose(M.wrap_angle(3 * math.pi), -math.pi, abs_tol=1e-9)
    assert math.isclose(M.wrap_angle(-0.5), -0.5)
    assert math.isclose(M.wrap_angle(2 * math.pi + 0.25), 0.25)


def test_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 2.5):
        q = M.quaternion_from_yaw(yaw)
        assert math.isclose(M.yaw_from_quaternion(*q), yaw, abs_tol=1e-9)


def test_errors_and_lengths():
    assert math.isclose(M.position_error(0, 0, 3, 4), 5.0)
    assert math.isclose(M.yaw_error(math.radians(179), math.radians(-179)), math.radians(2), abs_tol=1e-9)
    assert math.isclose(M.path_length([(0, 0), (3, 0), (3, 4)]), 7.0)
    assert M.path_length([]) == 0.0
    assert math.isclose(M.straight_line_distance((1, 1), (4, 5)), 5.0)
    assert math.isclose(M.path_efficiency(5.0, 6.25), 0.8)
    assert M.path_efficiency(5.0, 0.0) is None


def test_summary_uses_successful_trials_only():
    trials = [
        {'success': True, 'final_position_error_m': 0.10, 'navigation_time_s': 20.0},
        {'success': 'True', 'final_position_error_m': 0.20, 'navigation_time_s': 30.0},
        {'success': False, 'final_position_error_m': 4.0, 'navigation_time_s': 240.0},
    ]
    s = M.summarize(trials)
    assert s['trials'] == 3 and s['successful'] == 2
    assert math.isclose(s['success_rate_pct'], 200.0 / 3.0)
    assert math.isclose(s['final_position_error_m']['mean'], 0.15)
    assert math.isclose(s['navigation_time_s']['mean'], 25.0)
    assert math.isclose(s['navigation_time_s']['std'], math.sqrt(50.0))
