"""Fig 9a analog: context switches per second for one robot controller
vs number of concurrent robot controllers, with CPU pinning.

Paper's Fig 9a shows context switches/ms for one vBS with CPU pinning,
across 1-5 vBS instances. We use context switches/sec (as collected by
`perf stat -I 1s`), show the mean with error bars for p5-p95 range, and
plot one robot (`panther`) across runs of 1-4 robots.
"""

from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

import metrics


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    plots_dir = os.path.join(here, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    runs = metrics.load_all_runs()
    df = metrics.perf_summary_df(runs)

    sns.set_theme(style="whitegrid", context="talk")
    fig, ax = plt.subplots(figsize=(8, 5.5))

    # Mean with p5-p95 error bars (asymmetric)
    ax.errorbar(df["n_robots"], df["ctxsw_mean"],
                yerr=[df["ctxsw_mean"] - df["ctxsw_p05"],
                      df["ctxsw_p95"] - df["ctxsw_mean"]],
                fmt="o-", linewidth=2.5, markersize=9,
                color="#1f77b4", capsize=6, capthick=2,
                label="Mean (p5-p95 range)")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("Context switches / s (one robot)")
    ax.set_title("Context switches of one robot controller vs #robots\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper left", frameon=True)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = os.path.join(plots_dir, "fig9a_context_switches.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()