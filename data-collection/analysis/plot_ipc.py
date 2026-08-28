"""Fig 10 analog: Instructions Per Cycle (IPC) of one robot controller
vs number of concurrent robot controllers.

Paper's Fig 10 plots IPC vs #vBS, with a red horizontal line at IPC=1
marking the instruction/memory-bounded boundary. We replicate this for
the `panther` robot across runs of 1-4 robots, plotting mean with p5-p95
error bars.
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

    ax.errorbar(df["n_robots"], df["ipc_mean"],
                yerr=[df["ipc_mean"] - df["ipc_p05"],
                      df["ipc_p95"] - df["ipc_mean"]],
                fmt="o-", linewidth=2.5, markersize=9,
                color="#1f77b4", capsize=6, capthick=2,
                label="Mean (p5-p95 range)")

    # Boundary at IPC=1 (paper's red line): <1 means memory/cache-bounded.
    ax.axhline(1.0, color="red", linestyle="--", linewidth=2,
               label="IPC = 1 (boundary)")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("Instructions per cycle (IPC)")
    ax.set_title("IPC of one robot controller vs #robots\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper right", frameon=True)

    fig.tight_layout()
    out = os.path.join(plots_dir, "fig10_ipc.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()