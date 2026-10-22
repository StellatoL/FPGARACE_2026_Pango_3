#!/usr/bin/env python3
"""Guard the comparison's denominator and position/type joint criterion."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    "compare_lane_results", Path(__file__).resolve().parents[1] / "tools/compare_lane_results.py")
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


class ComparisonTest(unittest.TestCase):
    def test_unknown_and_wrong_position_remain_in_denominator(self):
        fields = ["frame", "right_valid", "right_type", "top_y", "bottom_y",
                  "right_top_x", "right_bottom_x"]
        cases = [[0, 1, 2, 0, 100, 10, 10], [1, 1, 1, 0, 100, 10, 10],
                 [2, 0, 0, 0, 100, 0, 0], [3, 1, 2, 0, 100, 100, 100]]
        labels = {"pixel_tolerance": 32, "samples": [
            {"frame": n, "split": "development", "right_type": 2,
             "points": [[10, 20], [10, 80]]} for n in range(4)]}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "samples.csv"
            with path.open("w", newline="") as handle:
                writer = csv.writer(handle);writer.writerow(fields);writer.writerows(cases)
            summary = comparison.evaluate(path, labels, "all")["summary"]
        self.assertEqual(summary["samples"], 4)
        self.assertEqual(summary["correct"], 1)
        self.assertEqual(summary["correct_type"], 2)
        self.assertEqual(summary["missed_unknown"], 1)
        self.assertEqual(summary["false_solid"], 1)
        self.assertEqual(summary["position_mismatch"], 1)

    def test_duplicate_sample_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "samples.csv"
            path.write_text("frame\n1\n1\n")
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                comparison.evaluate(path, {"pixel_tolerance": 32, "samples": []}, "all")


if __name__ == "__main__":
    unittest.main()
