"""New plot (ROS2-specific, beyond paper): navigation goal completion
time vs number of robot controllers.

The paper has no ROS2 equivalent; this adds value by showing how the
robot function (navigation) degrades under CPU contention. We plot the
per-run distribution of goal durations as a box/strip plot plus the
per-run mean, and annotate goals/hour.
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
    goals = metrics.ros2_goals_df(runs)
    summary = metrics.ros2_goals_summary_df(runs)
    if goals.empty:
        print("No ros2_goals data available; skipping.")
        return

    sns.set_theme(style="whitegrid", context="talk")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Left: boxplot of goal durations per n_robots
    sns.boxplot(data=goals, x="n_robots", y="duration_s",
                color="#1f77b4", ax=ax1,
                showmeans=True,
                meanprops={"marker": "D", "markerfacecolor": "white",
                           "markeredgecolor": "black", "markersize": 8})
    sns.stripplot(data=goals, x="n_robots", y="duration_s",
                  color="black", alpha=0.4, size=4, ax=ax1)
    ax1.set_xlabel("Number of robot controllers")
    ax1.set_ylabel("Goal completion time (s)")
    ax1.set_title("Navigation goal duration distribution")

    # Right: goals/hour and mean duration vs #robots
    ax2_twin = ax2.twinx()
    ax2.plot(summary["n_robots"], summary["duration_mean"], "o-",
             linewidth=2.5, markersize=9, color="#1f77b4",
             label="Mean duration (s)")
    ax2_twin.plot(summary["n_robots"], summary["goals_per_hour"], "s--",
                  linewidth=2.0, markersize=9, color="#d62728",
                  label="Goals / hour")
    ax2.set_xlabel("Number of robot controllers")
    ax2.set_ylabel("Mean goal duration (s)", color="#1f77b4")
    ax2_twin.set_ylabel("Goals / hour", color="#d62728")
    ax2.set_title("Throughput vs number of robot controllers")
    ax2.set_xticks(summary["n_robots"])

    # Combined legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper center",
               frameon=True)

    fig.suptitle("Navigation goal completion vs number of robot controllers",
                 fontsize=16, y=1.02)
    fig.tight_layout()
    out = os.path.join(plots_dir, "ros2_goals_vs_robots.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()