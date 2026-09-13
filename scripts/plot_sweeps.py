#!/usr/bin/env python3
"""Plot Monte-Carlo sweep results from run_sweeps.py CSVs, with statistics.

    python3 scripts/plot_sweeps.py dropout.csv --x p_detect_scale

Produces <csv-stem>.png: success rate + 95% Wilson score confidence interval
vs the swept parameter (top), and the dev_max distribution per point
(bottom). The top plot read backwards is the requirements spec: where
success collapses (accounting for the CI, not just the point estimate) is
the minimum the upstream subsystem (perception, state estimation, ...) must
deliver.

Each episode is one independent Bernoulli trial (pass/fail) at a fixed
parameter value, from an independent random seed. The Wilson score interval
(Wilson, 1927) is the standard confidence interval for a binomial proportion
-- unlike the naive normal approximation (p +/- 1.96*sqrt(p(1-p)/n)), it
stays inside [0,1] and is well-behaved even at small n or p near 0/1, which
is exactly this experiment's regime. Also prints a plain-text summary table
(episode counts, pass rate, 95% CI, margin of error) -- paste it straight
into a report's results table or appendix.
"""

import argparse
import csv
import math
import os
import statistics


def wilson_ci(k, n, z=1.96):
    """95% (z=1.96) Wilson score confidence interval for a binomial
    proportion: k successes out of n independent trials. Returns
    (point_estimate, lo, hi), all in [0, 1]. Wilson, E.B. (1927),
    "Probable Inference, the Law of Succession, and Statistical Inference"."""
    if n == 0:
        return (float("nan"),) * 3
    p = k / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    margin = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return p, max(0.0, center - margin), min(1.0, center + margin)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_file")
    ap.add_argument("--x", required=True, help="swept parameter (CSV column)")
    ap.add_argument("--out", default="", help="output PNG (default: csv stem)")
    ap.add_argument("--dev-col", default="",
                    help="deviation column for the bottom plot (default: "
                         "auto-detect dev_max [trackdrive] or circ_err_max [skidpad])")
    ap.add_argument("--ref-line", type=float, default=None,
                    help="safety-margin reference value for the bottom plot "
                         "(default: 1.75 for dev_max, 1.5 for circ_err_max)")
    args = ap.parse_args()

    with open(args.csv_file, newline="") as f:
        header = next(csv.reader(f))
    dev_col = args.dev_col or next(
        (c for c in ("dev_max", "circ_err_max") if c in header), None)
    ref_line = args.ref_line
    if ref_line is None:
        ref_line = {"dev_max": 1.75, "circ_err_max": 1.5}.get(dev_col)

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
            if dev_col and row.get(dev_col):
                g["dev"].append(float(row[dev_col]))

    if not groups:
        raise SystemExit("no rows with column %r in %s" % (args.x, args.csv_file))

    xs = sorted(groups)
    stats = [wilson_ci(groups[x]["pass"], groups[x]["n"]) for x in xs]
    rate = [100.0 * p for p, _, _ in stats]
    lo = [100.0 * l for _, l, _ in stats]
    hi = [100.0 * h for _, _, h in stats]
    dev_med = [statistics.median(groups[x]["dev"]) if groups[x]["dev"] else float("nan")
               for x in xs]
    dev_hi = [max(groups[x]["dev"]) if groups[x]["dev"] else float("nan") for x in xs]

    print("%-28s %8s %10s %20s %10s" % ("param", "n", "pass rate", "95% CI (Wilson)", "+/- (pp)"))
    for x, (p, l, h), g in zip(xs, stats, (groups[x] for x in xs)):
        moe = 100.0 * (h - l) / 2.0
        print("%-28s %8d %9.1f%% %10.1f%%-%5.1f%% %9.1f%%"
              % ("%s=%g" % (args.x, x), g["n"], 100 * p, 100 * l, 100 * h, moe))
    n_min = min(groups[x]["n"] for x in xs)
    print("\nAt n=%d episodes/point, worst-case 95%% CI half-width is +/-%.0f"
          "pp (at p=50%%, where uncertainty is largest). Tighter near 0%% or"
          " 100%%; halving the margin needs ~4x the episodes (CI width"
          " shrinks as 1/sqrt(n))." % (n_min, 100 * 1.96 * math.sqrt(0.25 / n_min)))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.5, 6), sharex=True)
    err = [[r - l for r, l in zip(rate, lo)], [h - r for r, h in zip(rate, hi)]]
    ax1.errorbar(xs, rate, yerr=err, marker="o", capsize=4, linewidth=1.5,
                 label="pass rate (95% Wilson CI)")
    ax1.set_ylabel("mission success (%)")
    ax1.set_ylim(-5, 105)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc="lower left")
    n_str = "/".join(str(groups[x]["n"]) for x in xs)
    ax1.set_title("Monte-Carlo sweep: %s  (n=%s episodes/point)" % (args.x, n_str))

    ax2.plot(xs, dev_med, marker="o", label="median %s" % (dev_col or "deviation"))
    ax2.plot(xs, dev_hi, marker="x", linestyle="--", label="worst %s" % (dev_col or "deviation"))
    if ref_line is not None:
        ax2.axhline(ref_line, color="red", linewidth=1,
                    label="half track/lane width (%.2g m)" % ref_line)
    ax2.set_xlabel(args.x)
    ax2.set_ylabel("path/circle deviation (m)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    out = args.out or os.path.splitext(args.csv_file)[0] + ".png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print("\nwrote", out)


if __name__ == "__main__":
    main()
