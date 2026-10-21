import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import (COLOURS, NSEEDS, STRAT, ci95, ensure_results, seed_values)

ALPHA, CS = 1.2, 5
DPI = 600
WIDTH_IN = 2.24
HEIGHT_IN = 1.3800000000000001

PANELS = [("a", "avg_delay_s", "Delay (s)"),
          ("b", "avg_hop", "Hops"),
          ("c", "avg_retx", "Retx"),
          ("d", "eviction_rate", "Evictions/s"),
          ("e", "producer_rate", "PHR (Interests/s)"),
          ("f", "served_by_cache_rate", "CHR")]

FIG_FOR_DN = {0: "16", 30: "17"}

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["TeX Gyre Termes", "Times New Roman", "Nimbus Roman",
                   "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "axes.labelsize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "xtick.major.size": 2.5,
    "ytick.major.size": 2.5,
})


def exact_inches(px, dpi):
    v = px / dpi
    for _ in range(8):
        if int(v * dpi) == px:
            return v
        v = float(np.nextafter(v, v + 1.0))
    raise RuntimeError("cannot hit %d px at %d dpi" % (px, dpi))


def panel(outdir, dn, letter, col, ylabel, stats):
    labels = [lab for _, lab in STRAT]
    means = [stats[lab][col][0] for lab in labels]
    errs = [stats[lab][col][1] for lab in labels]

    fig, ax = plt.subplots(figsize=(exact_inches(1344, DPI),
                                    exact_inches(828, DPI)), dpi=DPI)
    ax.set_axisbelow(True)
    ax.yaxis.grid(True, linestyle=":", linewidth=0.3, color="0.6")
    ax.xaxis.grid(False)

    x = range(len(labels))
    ax.bar(x, means, width=0.72, color=[COLOURS[l] for l in labels],
           edgecolor="black", linewidth=0.4, yerr=errs,
           error_kw=dict(ecolor="black", elinewidth=0.6, capsize=1.8,
                         capthick=0.6))

    ax.set_ylabel(ylabel)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=35, ha="right", rotation_mode="anchor")
    ax.set_xlim(-0.6, len(labels) - 0.4)
    top = max(m + e for m, e in zip(means, errs))
    ax.set_ylim(0, top * 1.08 if top > 0 else 1.0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.15)

    base = os.path.join(outdir, "Alhaw.%s.%s" % (FIG_FOR_DN[dn], letter))
    fig.savefig(base + ".png", dpi=DPI)
    fig.savefig(base + ".pdf")
    plt.close(fig)
    return base + ".png"


def main():
    outdir = ensure_results("figures")
    report = {}
    written = []

    for dn in (0, 30):
        stats = {}
        for key, lab in STRAT:
            per_col = {}
            for _, col, _ in PANELS:
                v = seed_values(key, dn, col, CS, ALPHA)
                if len(v) != NSEEDS:
                    raise SystemExit("dn=%d %s %s: %d seeds, expected %d"
                                     % (dn, lab, col, len(v), NSEEDS))
                per_col[col] = (sum(v) / len(v), ci95(v))
            stats[lab] = per_col
        report[dn] = stats
        for letter, col, ylabel in PANELS:
            written.append(panel(outdir, dn, letter, col, ylabel, stats))

    print("%3s %-18s %-10s %12s %10s" % ("dn", "metric", "strategy", "mean", "CI half"))
    for dn in (0, 30):
        for _, col, ylabel in PANELS:
            for _, lab in STRAT:
                m, e = report[dn][lab][col]
                print("%3d %-18s %-10s %12.4f %10.4f" % (dn, ylabel, lab, m, e))

    wide = [(dn, y, lab) for dn in (0, 30) for _, c, y in PANELS
            for _, lab in STRAT if report[dn][lab][c][1] > report[dn][lab][c][0]]
    print("\nCI half-width exceeding its bar: %s"
          % ("none" if not wide else wide))

    for p in written:
        from PIL import Image
        try:
            w, h = Image.open(p).size
        except Exception:
            continue
        if (w, h) != (1344, 828):
            raise SystemExit("%s is %dx%d, expected 1344x828" % (p, w, h))
    print("wrote %d panels (png + pdf) to %s" % (len(written), outdir))


if __name__ == "__main__":
    main()
