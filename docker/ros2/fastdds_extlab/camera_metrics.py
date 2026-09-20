"""Dependency-free statistics for bounded camera checks.

Header age is receiver wall time minus the publisher's header timestamp. It
includes cross-machine clock offset and capture/publish delay, so it is not a
measurement of network latency alone. Negative ages are deliberately retained.
"""

import math


HEADER_AGE_NOTE = (
    'header_age_s = received_unix_s - publisher_header_unix_s; includes '
    'cross-machine clock offset and capture/publish delay, not network latency alone'
)


def percentile(values, percent):
    """Return a linearly interpolated percentile, or None for no observations."""
    if not 0 <= percent <= 100:
        raise ValueError('percent must be between 0 and 100')
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * percent / 100
    lower = math.floor(position)
    upper = math.ceil(position)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def summarize_camera_metrics(frames, seconds):
    """Summarize successfully decoded frames without importing ROS or NumPy."""
    gaps = [later['t'] - earlier['t'] for earlier, later in zip(frames, frames[1:])]
    ages = [frame['header_age_s'] for frame in frames if 'header_age_s' in frame]
    return {
        'total_hz': len(frames) / seconds if seconds > 0 else None,
        'gap_p50_s': percentile(gaps, 50),
        'gap_p95_s': percentile(gaps, 95),
        'gap_p99_s': percentile(gaps, 99),
        'header_age_min_s': min(ages) if ages else None,
        'header_age_median_s': percentile(ages, 50),
        'header_age_p95_s': percentile(ages, 95),
        'header_age_max_s': max(ages) if ages else None,
        'header_age_note': HEADER_AGE_NOTE,
    }
