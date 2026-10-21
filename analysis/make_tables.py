import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from common import (ALPHAS, CACHES, DNS, METRICS, STRAT, cell_mean,
                    ensure_results)

COLS = [(cs, a) for cs in CACHES for a in ALPHAS]
BLOCK_BG = ["#eef4fb", "#ffffff"]
RULE = "#9aa0a6"
LEGEND = ("CHR = served_by_cache_rate    Delay = avg_delay_s    Hops = avg_hop"
          "    Retx = avg_retx    PHR = producer_rate    Evic = eviction_rate"
          "    AEC = Adaptive Edge Cache")


def fmt(strat, metric, vfmt, dn, cs, alpha):
    v = cell_mean(strat, dn, metric, cs, alpha)
    return vfmt.format(v) if v is not None else "--"


def draw_page(pdf, dn):
    ncol = 2 + len(COLS)
    fig, ax = plt.subplots(figsize=(17, 10.5))
    ax.axis("off")
    ax.set_title("Dead-node level  $d_n$ = %d%%   "
                 "(columns: cache size $C$  x  Zipf $\\alpha$; 5-seed means; "
                 "satisfied = 100%% for all, omitted)" % dn,
                 fontsize=12, weight="bold", pad=16)

    head_cs = ["dn = %d%%" % dn, ""] + ["C=%d" % cs if a == ALPHAS[0] else ""
                                        for cs, a in COLS]
    head_a = ["Metric", "Strategy"] + ["$\\alpha$=%s" % a for cs, a in COLS]

    mid = (len(STRAT) - 1) // 2
    body, cellcols, rowmeta = [], [], []
    for mi, (metric, label, vfmt) in enumerate(METRICS):
        for si, (s, slab) in enumerate(STRAT):
            row = [label if si == mid else "", slab]
            row += [fmt(s, metric, vfmt, dn, cs, a) for cs, a in COLS]
            body.append(row)
            cellcols.append(BLOCK_BG[mi % 2])
            rowmeta.append((mi, si))

    table = ax.table(cellText=[head_cs, head_a] + body,
                     cellLoc="center", loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(7.2)

    for (r, c), cell in table.get_celld().items():
        cell.set_linewidth(0.4)
        cell.set_edgecolor("#cccccc")
        if r == 0:
            cell.set_facecolor("#34618a")
            cell.set_text_props(color="white", weight="bold")
            cell.set_height(0.045)
        elif r == 1:
            cell.set_facecolor("#5b85ad")
            cell.set_text_props(color="white", weight="bold")
            cell.set_height(0.045)
        else:
            mi, si = rowmeta[r - 2]
            cell.set_facecolor(cellcols[r - 2])
            if c == 0 and si == mid:
                cell.set_text_props(weight="bold")
            if c == 1:
                cell.set_text_props(weight="bold")
            cell.set_height(0.026)
        if c == 0:
            cell.set_width(0.060)
        elif c == 1:
            cell.set_width(0.070)
        else:
            cell.set_width((1 - 0.130) / len(COLS))

    fig.canvas.draw()
    celld = table.get_celld()
    nbody = len(body)
    top = celld[(0, 0)].get_y() + celld[(0, 0)].get_height()
    bottom = celld[(nbody + 1, 0)].get_y()
    xl = celld[(0, 0)].get_x()
    xr = celld[(0, ncol - 1)].get_x() + celld[(0, ncol - 1)].get_width()

    for col in [2] + [2 + j * len(ALPHAS) for j in range(1, len(CACHES))]:
        x = celld[(0, col)].get_x()
        ax.plot([x, x], [bottom, top], color=RULE, lw=0.9 if col != 2 else 0.7,
                transform=ax.transAxes, clip_on=False, zorder=6)
    ax.plot([xr, xr], [bottom, top], color=RULE, lw=0.7,
            transform=ax.transAxes, clip_on=False, zorder=6)
    for j in range(1, len(METRICS)):
        rr = 2 + j * len(STRAT)
        y = celld[(rr, 0)].get_y() + celld[(rr, 0)].get_height()
        ax.plot([xl, xr], [y, y], color=RULE, lw=0.8,
                transform=ax.transAxes, clip_on=False, zorder=6)
    for yy in (celld[(1, 0)].get_y() + celld[(1, 0)].get_height(),
               celld[(1, 0)].get_y()):
        ax.plot([xl, xr], [yy, yy], color=RULE, lw=0.8,
                transform=ax.transAxes, clip_on=False, zorder=6)
    ax.plot([xl, xl], [bottom, top], color=RULE, lw=0.9,
            transform=ax.transAxes, clip_on=False, zorder=6)
    ax.plot([xl, xr], [top, top], color=RULE, lw=0.9,
            transform=ax.transAxes, clip_on=False, zorder=6)
    ax.plot([xl, xr], [bottom, bottom], color=RULE, lw=0.9,
            transform=ax.transAxes, clip_on=False, zorder=6)

    ax.text(0.5, 0.02, LEGEND, transform=ax.transAxes, ha="center",
            va="bottom", fontsize=8, color="#444444")
    pdf.savefig(fig)
    plt.close(fig)


def latex_table(dn):
    out = []
    out.append("\\begin{table*}[t]")
    out.append("\\centering")
    out.append("\\setlength{\\arrayrulewidth}{0.25pt}")
    out.append("\\caption{Performance of the six caching strategies at "
               "dead-node level $d_n=%d\\%%$. Columns: cache size $C$ "
               "$\\times$ Zipf skew $\\alpha$. Five-seed means. %s.}"
               % (dn, LEGEND.replace("    ", ", ")))
    out.append("\\label{tab:results-dn%d}" % dn)
    out.append("\\resizebox{\\textwidth}{!}{%")
    out.append("\\begin{tabular}{|ll|" + "rrr|" * len(CACHES) + "}")
    out.append("\\hline")
    out.append("\\multicolumn{%d}{|c|}{Dead-node level $d_n=%d\\%%$} \\\\"
               % (2 + len(COLS), dn))
    out.append("\\hline")
    out.append("Cache size $\\rightarrow$ & & "
               + " & ".join("\\multicolumn{3}{c|}{$C=%d$}" % cs for cs in CACHES)
               + " \\\\")
    out.append("\\hline")
    out.append("$\\alpha \\rightarrow$ & & "
               + " & ".join("$%s$" % a for cs, a in COLS) + " \\\\")
    out.append("\\hline")
    mid = (len(STRAT) - 1) // 2
    for metric, label, vfmt in METRICS:
        for si, (s, slab) in enumerate(STRAT):
            lab = label if si == mid else ""
            cells = " & ".join(fmt(s, metric, vfmt, dn, cs, a) for cs, a in COLS)
            out.append("%s & %s & %s \\\\" % (lab, slab, cells))
        out.append("\\hline")
    out.append("\\end{tabular}}")
    out.append("\\end{table*}")
    return "\n".join(out)


def main():
    tables = ensure_results("tables")
    pdf_path = os.path.join(tables, "results-tables.pdf")
    with PdfPages(pdf_path) as pdf:
        for dn in DNS:
            draw_page(pdf, dn)
            print("rendered table for dn=%d%%" % dn)
    tex_path = os.path.join(tables, "results-tables.tex")
    with open(tex_path, "w") as fh:
        fh.write("\n\n".join(latex_table(dn) for dn in DNS) + "\n")
    print("wrote %s (%d pages)" % (pdf_path, len(DNS)))
    print("wrote %s" % tex_path)


if __name__ == "__main__":
    main()
