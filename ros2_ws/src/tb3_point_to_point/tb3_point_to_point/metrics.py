"""Performance metrics for point-to-point navigation (pure Python, no ROS imports).

Definitions used throughout the project (see docs/04_EVALUATION.md):

* position error   E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2)
* yaw error        E_yaw = |wrap(yaw_g - yaw_f)|
* path length      L = sum_i sqrt((x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2)
* path efficiency  eta = L_reference / L_actual   (1.0 = no wasted motion)
* success rate     SR = successful goals / total goals * 100
"""
import math


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rotation about z) of a quaternion, in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """(x, y, z, w) quaternion for a pure rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def position_error(x, y, goal_x, goal_y):
    return math.hypot(goal_x - x, goal_y - y)


def yaw_error(yaw, goal_yaw):
    return abs(wrap_angle(goal_yaw - yaw))


def path_length(points):
    """Length of a polyline given as an iterable of (x, y) tuples."""
    total = 0.0
    prev = None
    for p in points:
        if prev is not None:
            total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


def straight_line_distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_efficiency(reference_length, actual_length):
    """Reference length divided by driven length; None if undefined."""
    if reference_length is None or actual_length is None or actual_length <= 1e-6:
        return None
    return reference_length / actual_length


def mean_std(values):
    """Mean and sample standard deviation of the non-None values."""
    vals = [float(v) for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return None, None
    mean = sum(vals) / len(vals)
    if len(vals) < 2:
        return mean, 0.0
    var = sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)
    return mean, math.sqrt(var)


def success_rate(trials):
    trials = list(trials)
    if not trials:
        return 0.0
    ok = sum(1 for t in trials if _truthy(t.get('success')))
    return 100.0 * ok / len(trials)


def _truthy(value):
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'succeeded')
    return bool(value)


SUMMARY_FIELDS = (
    'final_position_error_m',
    'final_yaw_error_deg',
    'navigation_time_s',
    'path_length_m',
    'planned_path_length_m',
    'straight_line_m',
    'path_efficiency',
    'average_speed_mps',
    'recoveries',
    'min_clearance_m',
    'localization_error_m',
)


def _to_float(value):
    if value is None or value == '':
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(out) else out


def summarize(trials):
    """Aggregate a list of trial dicts (as written by the mission runner).

    Success rate uses every trial; the other statistics use successful trials only,
    because a failed trial has no meaningful final error or path efficiency.
    """
    trials = list(trials)
    successful = [t for t in trials if _truthy(t.get('success'))]
    summary = {
        'trials': len(trials),
        'successful': len(successful),
        'success_rate_pct': success_rate(trials),
    }
    for field in SUMMARY_FIELDS:
        mean, std = mean_std(_to_float(t.get(field)) for t in successful)
        summary[field] = {'mean': mean, 'std': std}
    return summary
