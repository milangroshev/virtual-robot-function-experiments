"""Fig 11 analog: Misses Per 1000 Instructions (MPKI) of one robot
controller vs number of concurrent robot controllers.

Paper's Fig 11 shows MPKI growth driven by cache contention, and
identifies cache memory as the root cause of CPU overhead. We plot both
the overall MPKI (cache-misses/instructions) and the LLC MPKI
(longest_lat_cache.miss/instructions), since LLC contention is the
relevant noisy-neighbor effect.
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

    ax.errorbar(df["n_robots"], df["mpki_mean"],
                yerr=[df["mpki_mean"] - df["mpki_p05"],
                      df["mpki_p95"] - df["mpki_mean"]],
                fmt="o-", linewidth=2.5, markersize=9,
                color="#1f77b4", capsize=6, capthick=2,
                label="MPKI (cache-misses) mean")
    ax.plot(df["n_robots"], df["llc_mpki_mean"], "s--", linewidth=1.5,
            markersize=8, color="#d62728",
            label="LLC MPKI mean")

    ax.set_xlabel("Number of robot controllers")
    ax.set_ylabel("Misses per 1000 instructions (MPKI)")
    ax.set_title("Cache misses of one robot controller vs #robots\n"
                 "with CPU pinning (CPUs 0-1)")
    ax.set_xticks(df["n_robots"])
    ax.legend(loc="upper left", frameon=True)

    fig.tight_layout()
    out = os.path.join(plots_dir, "fig11_mpki.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()