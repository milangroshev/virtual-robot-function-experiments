"""New plot (not in paper): RAPL package power vs number of robot
controllers.

The paper's Fig 20 reports power *savings* of the AIRIC RL agent vs
baselines, which requires an RL controller we do not have. As a simpler
substitute, this plot shows how the CPU package power draw grows as we
add robot controllers sharing the same CPU pool (CPUs 0-1). Useful as a
raw input for any future energy-aware controller.
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
    df = metrics.power_summary_df(runs)

    sns.set_theme(style="whitegrid", context="talk")
    fig, ax = plt.subplots(figsize=(8, 5.5))

    # Clamp error bars to >=0 so matplotlib never rejects them.
    lo = (df["power_pkg_mean"] - df["power_pkg_p50"]).clip(lower=0)
    hi = (df["power_pkg_p95"] - df["power_pkg_mean"]).clip(lower=0)
    ax.errorbar(df["n_robots"], df["power_pkg_mean"],
                yerr=[lo, hi],
                fmt="o-", linewidth=2.5, markersize=9,
                color="#2ca02c", capsize=6, capthick=2,
                label="Package power mean (p50-p95)")
    ax.plot(df["n_robots"], df["power_cores_mean"], "s--", linewidth=1.5,
            markersize=8, color="#ff7f0e",
            label="Cores power mean")

    # Annotate per-robot marginal power
    for i in range(1, len(df)):
        delta = df["power_pkg_mean"].iloc[i] - df["power_pkg_mean"].iloc[i-1]
        n = int(df["n_robots"].iloc[i])
        ax.annotate(f"+{delta:.2f} W",
                    xy=(n, df["power_pkg_mean"].iloc[i]),
                    xytext=(0, 12), textcoords="offset points",
                    ha="center", fontsize=11, color="#d62728")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("Power (W)")
    ax.set_title("CPU package power vs number of robot controllers\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper left", frameon=True)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = os.path.join(plots_dir, "power_vs_robots.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()