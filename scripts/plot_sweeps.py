#!/usr/bin/env python3
"""Plot Monte-Carlo sweep results from run_sweeps.py CSVs.

    python3 scripts/plot_sweeps.py dropout.csv --x p_detect_scale

Produces <csv-stem>.png: success rate vs the swept parameter (top) and the
dev_max distribution per point (bottom). The top plot read backwards is the
perception requirements spec: where success collapses is the minimum the
perception stack must deliver.
"""

import argparse
import csv
import os
import statistics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_file")
    ap.add_argument("--x", required=True, help="swept parameter (CSV column)")
    ap.add_argument("--out", default="", help="output PNG (default: csv stem)")
    args = ap.parse_args()

    groups = {}
    with open(args.csv_file, newline="") as f:
        for row in csv.DictReader(f):
            try:
                x = float(row[args.x])
            except (KeyError, ValueError):
                continue
            g = groups.setdefault(x, {"n": 0, "pass": 0, "dev": []})
            g["n"] += 1
            g["pass"] += int(row["success"])
            if row.get("dev_max"):
                g["dev"].append(float(row["dev_max"]))

    if not groups:
        raise SystemExit("no rows with column %r in %s" % (args.x, args.csv_file))

    xs = sorted(groups)
    rate = [100.0 * groups[x]["pass"] / groups[x]["n"] for x in xs]
    dev_med = [statistics.median(groups[x]["dev"]) if groups[x]["dev"] else float("nan")
               for x in xs]
    dev_hi = [max(groups[x]["dev"]) if groups[x]["dev"] else float("nan") for x in xs]

    for x in xs:
        g = groups[x]
        print("%s=%-8g  %d/%d pass  dev_max median=%.2f worst=%.2f"
              % (args.x, x, g["pass"], g["n"],
                 statistics.median(g["dev"]) if g["dev"] else -1,
                 max(g["dev"]) if g["dev"] else -1))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.5, 6), sharex=True)
    ax1.plot(xs, rate, marker="o")
    ax1.set_ylabel("mission success (%)")
    ax1.set_ylim(-5, 105)
    ax1.grid(alpha=0.3)
    ax1.set_title("Monte-Carlo sweep: %s" % args.x)

    ax2.plot(xs, dev_med, marker="o", label="median dev_max")
    ax2.plot(xs, dev_hi, marker="x", linestyle="--", label="worst dev_max")
    ax2.axhline(1.75, color="red", linewidth=1, label="half track width")
    ax2.set_xlabel(args.x)
    ax2.set_ylabel("path deviation (m)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    out = args.out or os.path.splitext(args.csv_file)[0] + ".png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
