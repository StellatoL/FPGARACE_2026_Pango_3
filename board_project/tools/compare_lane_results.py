#!/usr/bin/env python3
"""Compare identical geometry samples against frozen, sparse manual marks.

Standard library only. No inference, safety-policy changes or frame selection.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def evaluate(csv_path, annotations, split):
    with Path(csv_path).open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    frames = {int(row["frame"]): row for row in rows}
    if len(frames) != len(rows):
        raise ValueError("Duplicate sampled frame")
    summary = dict(samples=0, correct=0, missed_unknown=0, false_solid=0,
                   false_dashed=0, position_mismatch=0, correct_type=0)
    details = []
    errors = []
    tolerance = annotations["pixel_tolerance"]
    for mark in annotations["samples"]:
        if split != "all" and mark["split"] != split:
            continue
        if len(mark["points"]) < 2:
            raise ValueError("At least two visible marks required")
        row = frames[mark["frame"]]
        valid = int(row["right_valid"]) == 1
        kind = int(row["right_type"])
        y0, y1 = float(row["top_y"]), float(row["bottom_y"])
        x0, x1 = float(row["right_top_x"]), float(row["right_bottom_x"])
        if valid and (not all(map(math.isfinite, [x0, x1, y0, y1])) or y1 <= y0):
            raise ValueError("Invalid geometry coordinates")
        residuals = [abs(x0 + (y - y0) * (x1 - x0) / (y1 - y0) - x)
                     for x, y in mark["points"]] if valid else []
        position_ok = valid and max(residuals) <= tolerance
        type_ok = valid and kind == mark["right_type"]
        summary["samples"] += 1
        summary["correct"] += bool(position_ok and type_ok)
        summary["missed_unknown"] += bool(not valid or kind == 0)
        summary["false_solid"] += bool(valid and kind == 1 and mark["right_type"] == 2)
        summary["false_dashed"] += bool(valid and kind == 2 and mark["right_type"] == 1)
        summary["position_mismatch"] += bool(valid and not position_ok)
        summary["correct_type"] += bool(type_ok)
        errors.extend(residuals)
        details.append(dict(frame=mark["frame"], split=mark["split"], right_valid=valid,
                            right_type=kind, correct=bool(position_ok and type_ok),
                            x_errors_px=residuals, position_ok=position_ok))
    summary["valid_point_mean_error_px"] = sum(errors) / len(errors) if errors else None
    summary["valid_point_mean_note"] = "conditional diagnostic; misses stay in samples/correct denominator"
    return dict(summary=summary, frames=details)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("annotations", type=Path)
    parser.add_argument("before_csv", type=Path)
    parser.add_argument("after_csv", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    annotations = json.loads(args.annotations.read_text())
    with args.before_csv.open(newline="") as handle:
        before = [r["frame"] for r in csv.DictReader(handle)]
    with args.after_csv.open(newline="") as handle:
        after = [r["frame"] for r in csv.DictReader(handle)]
    if before != after:
        raise ValueError("Before/after must contain exactly the same ordered sampled frames")
    report = {"scope": annotations["scope"], "limitations": annotations["limitations"],
              "sampled_frames_compared": len(before), "pixel_tolerance": annotations["pixel_tolerance"],
              "counting": "type AND position required; Unknown/invalid included in denominator; no authorization-rate metric",
              "splits": {}}
    for split in ("development", "holdout", "all"):
        report["splits"][split] = {
            "before": evaluate(args.before_csv, annotations, split),
            "after": evaluate(args.after_csv, annotations, split),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report["splits"]["all"]["before"]["summary"], ensure_ascii=False))
    print(json.dumps(report["splits"]["all"]["after"]["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main()
