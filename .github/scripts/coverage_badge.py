#!/usr/bin/env python3
"""Turn a gcovr JSON summary into a shields.io endpoint badge + a gate.

First-party CI tooling — no third-party coverage service involved. Reads
the gcovr `--json-summary` file, writes a shields endpoint JSON, prints the
(floored) line-coverage percent, and exits non-zero if coverage is below
the configured floor.

Usage:
  coverage_badge.py --summary cov.json --min 1 \
      --badge-out badges/coverage-badge.json [--github-output]
"""
import argparse
import json
import math
import os
import sys


def color_for(pct: int) -> str:
    # Tiers per the coverage-badge spec.
    if pct >= 90:
        return "brightgreen"
    if pct >= 75:
        return "green"
    if pct >= 60:
        return "yellowgreen"
    if pct >= 40:
        return "yellow"
    if pct >= 20:
        return "orange"
    return "red"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", required=True, help="gcovr --json-summary file")
    ap.add_argument("--min", type=float, default=1.0, help="minimum line %% (gate)")
    ap.add_argument("--badge-out", help="path to write the shields endpoint JSON")
    ap.add_argument("--github-output", action="store_true",
                    help="also append coverage=NN to $GITHUB_OUTPUT")
    args = ap.parse_args()

    with open(args.summary) as f:
        data = json.load(f)

    line_percent = float(data.get("line_percent", 0.0))
    # Floor so the badge never overstates (99.5% reads as 99%, not 100%).
    pct = math.floor(line_percent)

    if args.badge_out:
        os.makedirs(os.path.dirname(args.badge_out) or ".", exist_ok=True)
        badge = {
            "schemaVersion": 1,
            "label": "coverage",
            "message": f"{pct}%",
            "color": color_for(pct),
        }
        with open(args.badge_out, "w") as f:
            json.dump(badge, f)
        print(f"wrote {args.badge_out}: {badge['message']} ({badge['color']})")

    if args.github_output and os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as f:
            f.write(f"coverage={pct}\n")

    print(f"line coverage: {line_percent:.1f}% (floor {pct}%), floor gate {args.min}%")
    if line_percent < args.min:
        print(f"::error::coverage {line_percent:.1f}% is below the {args.min}% floor")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
