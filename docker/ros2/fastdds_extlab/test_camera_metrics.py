"""Run with: python -m unittest discover -s docker/ros2/fastdds_extlab -p test_camera_metrics.py"""

import unittest

from camera_metrics import percentile, summarize_camera_metrics


class PercentileTests(unittest.TestCase):
    def test_empty_and_single_observation(self):
        self.assertIsNone(percentile([], 50))
        self.assertEqual(percentile([3.5], 99), 3.5)

    def test_linear_interpolation_and_unsorted_values(self):
        self.assertEqual(percentile([4, 1, 3, 2], 0), 1)
        self.assertEqual(percentile([4, 1, 3, 2], 50), 2.5)
        self.assertAlmostEqual(percentile([4, 1, 3, 2], 95), 3.85)
        self.assertAlmostEqual(percentile([4, 1, 3, 2], 99), 3.97)
        self.assertEqual(percentile([4, 1, 3, 2], 100), 4)

    def test_invalid_percentile(self):
        for value in (-1, 101):
            with self.assertRaises(ValueError):
                percentile([1], value)


class CameraMetricsTests(unittest.TestCase):
    def test_empty_run(self):
        result = summarize_camera_metrics([], 10)
        self.assertEqual(result['total_hz'], 0)
        for key, value in result.items():
            if key.endswith('_s'):
                self.assertIsNone(value, key)

    def test_single_frame_has_age_but_no_gap(self):
        result = summarize_camera_metrics([{'t': 3, 'header_age_s': 0.2}], 5)
        self.assertEqual(result['total_hz'], 0.2)
        self.assertIsNone(result['gap_p50_s'])
        self.assertEqual(result['header_age_min_s'], 0.2)
        self.assertEqual(result['header_age_p95_s'], 0.2)

    def test_gaps_ages_and_full_run_rate(self):
        frames = [
            {'t': 2, 'header_age_s': -0.1},
            {'t': 3, 'header_age_s': 0.1},
            {'t': 5, 'header_age_s': 0.3},
            {'t': 8, 'header_age_s': 0.5},
        ]
        result = summarize_camera_metrics(frames, 10)
        self.assertEqual(result['total_hz'], 0.4)
        self.assertEqual(result['gap_p50_s'], 2)
        self.assertAlmostEqual(result['gap_p95_s'], 2.9)
        self.assertAlmostEqual(result['gap_p99_s'], 2.98)
        self.assertEqual(result['header_age_min_s'], -0.1)
        self.assertAlmostEqual(result['header_age_median_s'], 0.2)
        self.assertAlmostEqual(result['header_age_p95_s'], 0.47)
        self.assertEqual(result['header_age_max_s'], 0.5)
        self.assertIn('cross-machine clock offset', result['header_age_note'])

    def test_older_records_without_age_and_zero_duration(self):
        result = summarize_camera_metrics([{'t': 1}, {'t': 2}], 0)
        self.assertIsNone(result['total_hz'])
        self.assertEqual(result['gap_p95_s'], 1)
        self.assertIsNone(result['header_age_median_s'])


if __name__ == '__main__':
    unittest.main()
