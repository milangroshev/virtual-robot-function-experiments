"""Fig 1 & Fig 8 analog: aggregated CPU usage vs number of robot
controllers, measured vs ideal isolation.

Paper's Fig 1 plots per-core CPU usage vs #vBS without CPU pinning; Fig 8
repeats the experiment with CPU pinning. Our setup already pins every
container to CPUs 0-1, so this single figure corresponds to the paper's
Fig 8 (pinned case). We overlay the 'ideal isolation' baseline
(N * single-robot usage) the same way the paper does.
"""

from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import seaborn as sns

import metrics


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    plots_dir = os.path.join(here, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    runs = metrics.load_all_runs()
    df = metrics.cpu_usage_df(runs)

    sns.set_theme(style="whitegrid", context="talk")
    fig, ax = plt.subplots(figsize=(8, 5.5))

    # Measured (95th percentile aggregated CPU)
    ax.plot(df["n_robots"], df["cpu_p95"], "o-", linewidth=2.5,
            markersize=9, color="#1f77b4", label="Measured (p95)")
    # Ideal isolation baseline
    ax.plot(df["n_robots"], df["cpu_ideal"], "k--", linewidth=2.0,
            label="Ideal isolation")

    # Annotate overhead % above measured points
    for _, r in df.iterrows():
        n = int(r["n_robots"])
        if n >= 2 and r["cpu_ideal"] > 0:
            overhead = (r["cpu_p95"] / r["cpu_ideal"] - 1) * 100
            ax.annotate(f"+{overhead:.1f}%",
                        xy=(n, r["cpu_p95"]),
                        xytext=(0, 10), textcoords="offset points",
                        ha="center", fontsize=11, color="#d62728",
                        fontweight="bold")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("Aggregated CPU usage (cores)")
    ax.set_title("Per-core CPU usage vs number of robot controllers\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper left", frameon=True)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = os.path.join(plots_dir, "fig1_8_cpu_vs_robots.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()