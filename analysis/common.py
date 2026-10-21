import csv
import glob
import math
import os
import statistics as st

DATA = os.environ.get("AEC_DATA",
                      os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data"))
RESULTS = os.environ.get("AEC_RESULTS",
                        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"))

DNS = [0, 30, 40, 50]
CACHES = [5, 10, 50, 100]
ALPHAS = [1.2, 1.6, 2.0]

STRAT = [("lce", "LCE"), ("lcd", "LCD"), ("probcache", "ProbCache"),
         ("btw", "Btw"), ("poolcache", "PoolCache"), ("aec", "AEC")]

COLOURS = {"LCE": "#7f8c8d", "LCD": "#3498db", "ProbCache": "#16a085",
           "Btw": "#8e44ad", "PoolCache": "#e67e22", "AEC": "#c0392b"}

METRICS = [
    ("served_by_cache_rate", "CHR", "{:.3f}"),
    ("avg_delay_s", "Delay (s)", "{:.2f}"),
    ("avg_hop", "Hops", "{:.2f}"),
    ("avg_retx", "Retx", "{:.2f}"),
    ("producer_rate", "PHR", "{:.2f}"),
    ("eviction_rate", "Evic", "{:.2f}"),
]

T95_4 = 2.776
NSEEDS = 5


def seed_values(strat, dn, metric, cs, alpha):
    vals = []
    pattern = os.path.join(DATA, "comparison-%s-dn%d" % (strat, dn),
                           "seed*", "per-cell.csv")
    for f in sorted(glob.glob(pattern)):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                if abs(float(r["alpha"]) - alpha) < 1e-9 and int(r["cache_size"]) == cs:
                    vals.append(float(r[metric]))
    return vals


def cell_mean(strat, dn, metric, cs, alpha):
    vals = seed_values(strat, dn, metric, cs, alpha)
    return st.mean(vals) if vals else None


def ci95(vals):
    if len(vals) < 2:
        return 0.0
    return T95_4 * st.stdev(vals) / math.sqrt(len(vals))


def ensure_results(*parts):
    path = os.path.join(RESULTS, *parts) if parts else RESULTS
    os.makedirs(path, exist_ok=True)
    return path
