"""Fig 2 analog (throughput proxy): cmd_vel_nav publish rate vs number
of robot controllers.

The paper's Fig 2 plots normalized throughput vs CPU allocation, with a
measured curve and an ideal isolation line. We have only one CPU
allocation (CPUs 0-1), so we cannot reproduce the x-axis. Instead we use
the throughput vs #robots view: the cmd_vel_nav topic rate is the
robot's control-loop output rate and serves as the throughput proxy. We
plot mean and 5th-percentile across all robot instances in each run.
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
    df = metrics.ros2_cmd_vel_summary_df(runs)
    if df.empty:
        print("No ros2_rates data available; skipping.")
        return

    sns.set_theme(style="whitegrid", context="talk")
    fig, ax = plt.subplots(figsize=(8, 5.5))

    ax.plot(df["n_robots"], df["hz_mean"], "o-", linewidth=2.5,
            markersize=9, color="#1f77b4", label="Mean rate")
    ax.plot(df["n_robots"], df["hz_p05"], "v--", linewidth=1.5,
            markersize=8, color="#d62728", label="5th percentile")
    ax.fill_between(df["n_robots"], df["hz_p05"], df["hz_mean"],
                    alpha=0.15, color="#1f77b4")

    # Reference line at the nominal 10 Hz setpoint
    ax.axhline(10.0, color="gray", linestyle=":", linewidth=1.5,
               label="Setpoint (10 Hz)")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("cmd_vel_nav publish rate (Hz)")
    ax.set_title("cmd_vel_nav rate vs number of robot controllers\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper right", frameon=True)

    fig.tight_layout()
    out = os.path.join(plots_dir, "fig2_cmd_vel_nav_rate.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()