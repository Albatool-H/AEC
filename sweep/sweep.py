#!/usr/bin/env python3
import argparse
import csv
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

PROJECT = Path(__file__).resolve().parent.parent
RUN_DIR = PROJECT / "runs"
OUT_DIR = PROJECT / "results" / "comparison"
LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH", "/opt/ndnsim/root/ndnSIM/lib")

ALPHAS    = [1.2, 1.4, 1.6, 1.8, 2.0]
DEAD_NS   = [0, 1, 5, 10, 25, 50]
CACHE_SIZES = [10, 30, 50, 100, 200]
SYSTEMS   = ["pool", "baseline"]

SIM_TIME = 400.0
DOWN_TIME = 200.0
FAIL_PERIOD = 2000.0
FAIL_START = 100.0
CONS_RATE = 30.0

CONSUMER_MODEL          = "zipf"
TOTAL_REQUESTS          = 10000
CONSUMER_START_OFFSET   = 0.0

PARSE_WINDOW_LO         = None
CONSUMER_END_TIME       = 0.0
USE_CONSUMER_SIDE_WRAP  = False
USE_STRATEGY_WRAP       = False

USE_RL_BANDIT           = False
RL_EPSILON              = 0.05

RL_W_PRODUCER           = None
RL_W_HOP                = None
RL_W_DELAY              = None
RL_W_EVICTION           = None
RL_W_COST               = None
KEEP_TRACES             = False

RNG_SEED                = 1
TOPO_POOL               = "topology/iot-topology.txt"
TOPO_BASELINE           = "topology/iot-topology-noctrl.txt"

CACHE_STRATEGY          = "lce"
BTW_FILE                = "topology/iot-topology-noctrl.btw.tsv"
META_POOL               = "topology/iot-meta.json"

PROG_FOR = {
    "pool":     "sdndn",
    "baseline": "baseline-ndn",
}

def run_one(system: str, alpha: float, dn: int, cs: int) -> Path:
    """Run one cell and return the run directory."""
    label = f"{system}_a{alpha}_dn{dn}_cs{cs}"
    if system == "baseline" and CACHE_STRATEGY != "lce":
        label += f"_{CACHE_STRATEGY}"
    if RNG_SEED != 1:
        label += f"_seed{RNG_SEED}"
    rdir = RUN_DIR / f"matrix-{label}"
    rdir.mkdir(parents=True, exist_ok=True)
    for f in ("app-delays-trace.txt", "l3-rate-trace.txt", "control-trace.txt",
              "cs-trace.txt", "d_pool_stats.csv", "producer_hits.csv",
              "baseline_cs_stats.csv"):
        p = PROJECT / f
        if p.exists():
            p.unlink()

    parts = [PROG_FOR[system],
             f"--simTime={SIM_TIME}",
             f"--cacheSize={cs}",
             f"--zipfAlpha={alpha}",
             f"--deadNodes={dn}",
             f"--consumerModel={CONSUMER_MODEL}"]
    if dn > 0:
        parts += [f"--downTime={DOWN_TIME}",
                  f"--failPeriod={FAIL_PERIOD}",
                  f"--failStart={FAIL_START}"]
    if CONSUMER_MODEL == "iot-oneshot":
        parts.append(f"--totalRequests={TOTAL_REQUESTS}")
    else:
        parts.append(f"--consRate={CONS_RATE}")
    if CONSUMER_START_OFFSET > 0:
        parts.append(f"--consumerStartOffset={CONSUMER_START_OFFSET}")
    if CONSUMER_END_TIME > 0:
        parts.append(f"--consumerEndTime={CONSUMER_END_TIME}")

    parts.append(f"--seed={RNG_SEED}")
    bridge_dir = None
    if system == "pool":
        parts.append(f"--topoFile={TOPO_POOL}")
        parts.append(f"--metaFile={META_POOL}")
        if USE_CONSUMER_SIDE_WRAP and CONSUMER_MODEL == "iot-oneshot":
            parts.append("--useConsumerSideWrap=true")
        if USE_STRATEGY_WRAP and CONSUMER_MODEL == "iot-oneshot":
            parts.append("--useStrategyWrap=true")

        if USE_RL_BANDIT:
            bridge_dir = PROJECT / "runs" / f"rl-bridge-{label}"
            if bridge_dir.exists():
                import shutil; shutil.rmtree(bridge_dir, ignore_errors=True)
            bridge_dir.mkdir(parents=True, exist_ok=True)
            parts.append("--useRlBandit=true")
            parts.append(f"--rlBridgeDir={bridge_dir}")
            for flag, val in (("rlWProducer", RL_W_PRODUCER), ("rlWHop", RL_W_HOP),
                              ("rlWDelay", RL_W_DELAY), ("rlWEviction", RL_W_EVICTION),
                              ("rlWCost", RL_W_COST)):
                if val is not None:
                    parts.append(f"--{flag}={val}")
    else:
        parts.append(f"--topoFile={TOPO_BASELINE}")
        if CACHE_STRATEGY != "lce":
            parts.append(f"--cacheStrategy={CACHE_STRATEGY}")
        if CACHE_STRATEGY == "btw":
            parts.append(f"--btwFile={BTW_FILE}")
    args = " ".join(parts)
    cmd = ["python2", "./waf", "--run", args]
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = LD_LIBRARY_PATH

    print(f"[run] {label}", flush=True)
    log = (rdir / "console.log").open("w")
    agent_proc = None
    try:
        if bridge_dir is not None:

            agent_log = (rdir / "vw-agent.log").open("w")
            agent_proc = subprocess.Popen(
                ["python3", "-u", "scripts/vw-bandit-agent.py",
                 "--bridge-dir", str(bridge_dir),
                 "--epsilon", str(RL_EPSILON),
                 "--poll-seconds", "0.005"],
                cwd=PROJECT, env=env,
                stdout=agent_log, stderr=subprocess.STDOUT)

            time.sleep(1.5)
        subprocess.run(cmd, cwd=PROJECT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    finally:
        log.close()
        if agent_proc is not None:
            agent_proc.terminate()
            try: agent_proc.wait(timeout=10)
            except subprocess.TimeoutExpired: agent_proc.kill()

            if not KEEP_TRACES and bridge_dir is not None and bridge_dir.exists():
                import shutil; shutil.rmtree(bridge_dir, ignore_errors=True)

    for f in ("app-delays-trace.txt", "l3-rate-trace.txt", "control-trace.txt",
              "cs-trace.txt", "d_pool_stats.csv", "producer_hits.csv",
              "baseline_cs_stats.csv"):
        src = PROJECT / f
        if src.exists():
            src.rename(rdir / f)
    return rdir

def parse_cell(system: str, rdir: Path) -> dict:
    """Compute per-cell metrics over the consumer-firing window
    [CONSUMER_START_OFFSET, CONSUMER_END_TIME). All window-bounded
    metrics use the SAME window regardless of failure injection so
    dn=0 cells are directly comparable to dn>0 cells."""
    out = {
        "satisfied":         0,
        "avg_delay_s":       None,
        "avg_hop":           None,
        "avg_retx":          None,
        "producer_rate":     None,
        "served_by_cache_rate": None,
        "eviction_rate":     None,
    }

    win_lo = PARSE_WINDOW_LO if PARSE_WINDOW_LO is not None else CONSUMER_START_OFFSET
    win_hi = CONSUMER_END_TIME if CONSUMER_END_TIME > 0 else SIM_TIME
    win_len = max(1.0, win_hi - win_lo)

    n = d_sum = h_sum = r_sum = 0
    n_in_window = 0
    ad_path = rdir / "app-delays-trace.txt"
    if ad_path.exists():
        with ad_path.open() as f:
            header = next(f, None)

            for line in f:
                parts = line.split("\t")
                if len(parts) < 9: continue
                if parts[4] != "FullDelay": continue
                try:
                    t = float(parts[0]); d = float(parts[5])
                    r = float(parts[7]); h = float(parts[8])
                except ValueError:
                    continue
                n += 1; d_sum += d; h_sum += h; r_sum += r
                if win_lo <= t < win_hi:
                    n_in_window += 1
    if n > 0:
        out["satisfied"]   = n
        out["avg_delay_s"] = d_sum / n
        out["avg_hop"]     = h_sum / n
        out["avg_retx"]    = r_sum / n

    prod_total = 0.0
    if system == "baseline":
        l3_path = rdir / "l3-rate-trace.txt"
        if l3_path.exists():
            with l3_path.open() as f:
                next(f, None)
                for line in f:
                    parts = line.split("\t")
                    if len(parts) < 6: continue
                    if not parts[1].startswith("P"): continue
                    if parts[4] != "InInterests": continue
                    try:
                        t = float(parts[0]); v = float(parts[5])
                    except ValueError:
                        continue
                    if win_lo <= t < win_hi:
                        prod_total += v
    else:
        ph_path = rdir / "producer_hits.csv"
        if ph_path.exists():
            with ph_path.open() as f:
                next(f, None)
                for line in f:
                    parts = line.strip().split("\t")
                    if len(parts) < 3: continue
                    try:
                        t = float(parts[0]); v = float(parts[2])
                    except ValueError:
                        continue
                    if win_lo <= t < win_hi:
                        prod_total += v
    out["producer_rate"] = prod_total / win_len

    if n_in_window > 0:
        rate = 1.0 - prod_total / n_in_window
        out["served_by_cache_rate"] = max(0.0, min(1.0, rate))

    if system == "pool":
        ev_path = rdir / "d_pool_stats.csv"
        ev_col = 4
    else:
        ev_path = rdir / "baseline_cs_stats.csv"
        ev_col = 2
    if ev_path.exists():
        evict_total = 0.0
        with ev_path.open() as f:
            next(f, None)
            for line in f:
                parts = line.strip().split("\t")
                if len(parts) <= ev_col: continue
                try:
                    t  = float(parts[0])
                    ev = float(parts[ev_col])
                except ValueError:
                    continue
                if win_lo <= t < win_hi:
                    evict_total += ev
        out["eviction_rate"] = evict_total / win_len

    return out

METRICS = [
    ("satisfied",            "Satisfied Interests (total)"),
    ("avg_delay_s",          "Avg Full Delay (s)"),
    ("avg_hop",              "Avg Hop Count"),
    ("avg_retx",             "Avg Retx Count"),
    ("producer_rate",        "Producer Interests/s (consumer window)"),
    ("served_by_cache_rate", "Served-by-Cache Rate (consumer window)"),
    ("eviction_rate",        "Network-wide Eviction Rate /s"),
]

def emit_per_cell_csv(cells, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["system", "alpha", "dead_nodes", "cache_size"] + [m for m, _ in METRICS])
        for (system, alpha, dn, cs), m in sorted(cells.items()):
            w.writerow([system, alpha, dn, cs] + [m.get(k) for k, _ in METRICS])

def emit_per_metric(cells, metric, label, alphas, dead_ns, cache_sizes):
    """Option A: per-metric figure with one subplot per cacheSize stacked
    vertically. Each subplot is the grouped bar layout pool-cache vs
    baseline over the alpha x deadNodes conditions."""
    pool_get = lambda a, k, cs: cells.get(("pool",     a, k, cs), {}).get(metric)
    base_get = lambda a, k, cs: cells.get(("baseline", a, k, cs), {}).get(metric)

    csv_path = OUT_DIR / f"{metric}.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        header = ["cache_size", "dead_nodes"]
        for a in alphas:
            header.append(f"a={a}_pool")
            header.append(f"a={a}_base")
        w.writerow(header)
        for cs in cache_sizes:
            for k in dead_ns:
                row = [cs, k]
                for a in alphas:
                    row.append(pool_get(a, k, cs))
                    row.append(base_get(a, k, cs))
                w.writerow(row)

    conds = [(a, k) for a in alphas for k in dead_ns]
    xs = np.arange(len(conds))
    width = 0.4
    nrows = len(cache_sizes)
    fig_w = max(8, len(conds) * 0.4)
    fig_h = 3.0 * nrows
    fig, axes = plt.subplots(nrows=nrows, ncols=1, figsize=(fig_w, fig_h),
                             sharex=True, squeeze=False)
    for i, cs in enumerate(cache_sizes):
        ax = axes[i, 0]
        pool_y = [pool_get(a, k, cs) or 0.0 for a, k in conds]
        base_y = [base_get(a, k, cs) or 0.0 for a, k in conds]
        ax.bar(xs - width / 2, pool_y, width, label="pool-cache", color="#1f77b4")
        ax.bar(xs + width / 2, base_y, width, label="baseline-NDN", color="#ff7f0e")
        ax.set_title(f"cacheSize = {cs}", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.3, axis="y")
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xticks(xs)

    if len(dead_ns) == 1:
        axes[-1, 0].set_xticklabels([f"α={a}" for a, _ in conds], fontsize=8)
        fig.suptitle(f"{label} -- pool-cache vs baseline NDN -- dead nodes={dead_ns[0]}",
                     fontsize=12)
    else:

        axes[-1, 0].set_xticklabels([f"α={a}\ndn={k}" for a, k in conds], fontsize=7)
        fig.suptitle(f"{label} -- pool-cache vs baseline NDN", fontsize=12)

    fig.supylabel(label, fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"{metric}.png", dpi=160)
    plt.close(fig)

def emit_markdown_summary(cells, path, alphas, dead_ns, cache_sizes):
    with open(path, "w") as f:
        f.write("# Pool-cache vs baseline-NDN comparison matrix\n\n")
        win_lo = CONSUMER_START_OFFSET
        win_hi = CONSUMER_END_TIME if CONSUMER_END_TIME > 0 else SIM_TIME
        f.write(f"alpha ∈ {alphas}, deadNodes ∈ {dead_ns}, cacheSize ∈ {cache_sizes}, "
                f"simTime={SIM_TIME}s. Failure: downTime={DOWN_TIME}s "
                f"starting at {FAIL_START}s (only used when dn>0). "
                f"Window-bounded metrics measured over the consumer-firing "
                f"window [{win_lo}, {win_hi}). Single run per cell.\n\n")
        for metric, label in METRICS:
            f.write(f"\n## {label}\n\n")
            for cs in cache_sizes:
                f.write(f"\n### cacheSize = {cs}\n\n")
                cols = ["dead_nodes"]
                for a in alphas:
                    cols.append(f"α={a} pool")
                    cols.append(f"α={a} base")
                f.write("| " + " | ".join(cols) + " |\n")
                f.write("|" + "---|" * len(cols) + "\n")
                for k in dead_ns:
                    row = [str(k)]
                    for a in alphas:
                        p = cells.get(("pool",     a, k, cs), {}).get(metric)
                        b = cells.get(("baseline", a, k, cs), {}).get(metric)
                        row.append(f"{p:.4g}" if p is not None else "-")
                        row.append(f"{b:.4g}" if b is not None else "-")
                    f.write("| " + " | ".join(row) + " |\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alphas",       default=",".join(str(a) for a in ALPHAS))
    ap.add_argument("--dead-nodes",        default=",".join(str(k) for k in DEAD_NS))
    ap.add_argument("--cache-sizes",  default=",".join(str(c) for c in CACHE_SIZES))
    ap.add_argument("--systems",      default=",".join(SYSTEMS))
    ap.add_argument("--skip-run",     action="store_true",
                    help="Skip simulation; only re-parse and re-plot from existing run dirs")
    ap.add_argument("--keep-traces", action="store_true",
                    help="Don't delete l3-rate-trace.txt / cs-trace.txt after parse (allows later re-parse with a changed formula)")
    ap.add_argument("--use-rl-bandit", action="store_true",
                    help="pool cells run with --useRlBandit=true; matrix script launches a VW agent per cell.")
    ap.add_argument("--rl-epsilon", type=float, default=0.05,
                    help="epsilon for the VW agent's --epsilon flag (default 0.05)")
    ap.add_argument("--rl-w-producer", type=float, default=None)
    ap.add_argument("--rl-w-hop",      type=float, default=None)
    ap.add_argument("--rl-w-delay",    type=float, default=None)
    ap.add_argument("--rl-w-eviction", type=float, default=None)
    ap.add_argument("--rl-w-cost",     type=float, default=None)
    ap.add_argument("--seed", type=int, default=1,
                    help="RNG seed passed to scenarios via --seed=N. Per-cell run "
                         "dirs get a _seed{N} suffix when N != 1.")
    ap.add_argument("--prog-pool", default=None,
                    help="Pool binary name (default 'sdndn'). Use this to run an isolated copy, e.g. 'aec'.")
    ap.add_argument("--prog-baseline", default=None,
                    help="Baseline binary name (default 'baseline-ndn').")

    ap.add_argument("--sim-time", type=float, default=None)
    ap.add_argument("--fail-start", type=float, default=None)
    ap.add_argument("--down-window", type=float, default=None,
                    help="Analysis window length in seconds (also passed as --downTime when dn>0)")
    ap.add_argument("--cons-rate", type=float, default=None)
    ap.add_argument("--consumer-model", choices=["zipf", "iot-oneshot"], default=None)
    ap.add_argument("--total-requests", type=int, default=None)
    ap.add_argument("--consumer-start-offset", type=float, default=None)
    ap.add_argument("--consumer-end-time", type=float, default=None)
    ap.add_argument("--no-cold-warm", action="store_true",
                    help="Disable the t=CONSUMER_START_OFFSET cold-warm exclusion in parse_cell; "
                         "metrics will be computed over [0, CONSUMER_END_TIME or SIM_TIME) instead")
    ap.add_argument("--use-consumer-side-wrap", action="store_true")
    ap.add_argument("--use-strategy-wrap",      action="store_true",
                    help="Install PoolCacheGatewayStrategy on /u/ at every switch (3c spike)")
    ap.add_argument("--topo-pool", default=None)
    ap.add_argument("--topo-baseline", default=None)
    ap.add_argument("--meta-pool", default=None)
    ap.add_argument("--cache-strategy", default=None,
                    choices=["lce", "lcd", "probcache", "btw", "poolcache"],
                    help="baseline caching strategy (baseline system only).")
    ap.add_argument("--btw-file", default=None,
                    help="betweenness TSV for --cache-strategy=btw.")
    args = ap.parse_args()

    global SIM_TIME, DOWN_TIME, FAIL_START, CONS_RATE
    global CONSUMER_MODEL, TOTAL_REQUESTS, CONSUMER_START_OFFSET, CONSUMER_END_TIME
    global USE_CONSUMER_SIDE_WRAP, USE_STRATEGY_WRAP, TOPO_POOL, TOPO_BASELINE, META_POOL
    if args.sim_time   is not None: SIM_TIME   = args.sim_time
    if args.fail_start is not None: FAIL_START = args.fail_start
    if args.down_window is not None: DOWN_TIME = args.down_window
    if args.cons_rate  is not None: CONS_RATE  = args.cons_rate
    if args.consumer_model        is not None: CONSUMER_MODEL = args.consumer_model
    if args.total_requests        is not None: TOTAL_REQUESTS = args.total_requests
    if args.consumer_start_offset is not None: CONSUMER_START_OFFSET = args.consumer_start_offset
    if args.consumer_end_time     is not None: CONSUMER_END_TIME = args.consumer_end_time

    if args.no_cold_warm:
        global PARSE_WINDOW_LO
        PARSE_WINDOW_LO = 0.0
    if args.use_consumer_side_wrap: USE_CONSUMER_SIDE_WRAP = True
    if args.use_strategy_wrap:      USE_STRATEGY_WRAP = True
    global USE_RL_BANDIT, RL_EPSILON, KEEP_TRACES, RNG_SEED
    global RL_W_PRODUCER, RL_W_HOP, RL_W_DELAY, RL_W_EVICTION, RL_W_COST
    if args.use_rl_bandit:           USE_RL_BANDIT = True
    if args.rl_epsilon is not None:  RL_EPSILON    = args.rl_epsilon
    if args.rl_w_producer is not None: RL_W_PRODUCER = args.rl_w_producer
    if args.rl_w_hop      is not None: RL_W_HOP      = args.rl_w_hop
    if args.rl_w_delay    is not None: RL_W_DELAY    = args.rl_w_delay
    if args.rl_w_eviction is not None: RL_W_EVICTION = args.rl_w_eviction
    if args.rl_w_cost     is not None: RL_W_COST     = args.rl_w_cost
    if args.keep_traces:             KEEP_TRACES   = True
    if args.seed is not None:        RNG_SEED      = args.seed
    if args.topo_pool     is not None: TOPO_POOL     = args.topo_pool
    if args.topo_baseline is not None: TOPO_BASELINE = args.topo_baseline
    if args.meta_pool     is not None: META_POOL     = args.meta_pool
    if args.prog_pool     is not None: PROG_FOR["pool"]     = args.prog_pool
    if args.prog_baseline is not None: PROG_FOR["baseline"] = args.prog_baseline
    global CACHE_STRATEGY, BTW_FILE
    if args.cache_strategy is not None: CACHE_STRATEGY = args.cache_strategy
    if args.btw_file       is not None: BTW_FILE       = args.btw_file

    alphas      = [float(x) for x in args.alphas.split(",") if x]
    dead_ns       = [int(x)   for x in args.dead_nodes.split(",")  if x]
    cache_sizes = [int(x)   for x in args.cache_sizes.split(",") if x]
    systems     = [s for s in args.systems.split(",") if s]

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cells = {}
    total = len(systems) * len(alphas) * len(dead_ns) * len(cache_sizes)
    done = 0
    for system in systems:
        for a in alphas:
            for k in dead_ns:
                for cs in cache_sizes:
                    rdir = RUN_DIR / f"matrix-{system}_a{a}_k{k}_cs{cs}"
                    if not args.skip_run:
                        rdir = run_one(system, a, k, cs)
                    if not rdir.exists():
                        print(f"[warn] missing run dir {rdir}", file=sys.stderr)
                        continue
                    m = parse_cell(system, rdir)
                    cells[(system, a, k, cs)] = m
                    done += 1
                    print(f"[{done}/{total}] {system} a={a} k={k} cs={cs}: {m}", flush=True)

                    if not args.skip_run and not args.keep_traces:
                        for fname in ("l3-rate-trace.txt", "cs-trace.txt"):
                            p = rdir / fname
                            if p.exists():
                                p.unlink()

    emit_per_cell_csv(cells, OUT_DIR / "per-cell.csv")
    for metric, label in METRICS:
        emit_per_metric(cells, metric, label, alphas, dead_ns, cache_sizes)
    emit_markdown_summary(cells, OUT_DIR / "summary.md", alphas, dead_ns, cache_sizes)
    print(f"[done] tables + plots in {OUT_DIR}")

if __name__ == "__main__":
    main()
