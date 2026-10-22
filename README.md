# Adaptive Edge Cache (AEC)

ndnSIM implementation of AEC and five baseline caching schemes (LCE, LCD,
ProbCache, Btw, PoolCache) from:

A. A. Alhawas, S. Al-Ahmadi and A. Gazdar, "Adaptive Edge Caching and Routing in Named Data Networking of Things Using Contextual Bandits and SDN Controllers," in IEEE Access, doi: 10.1109/ACCESS.2026.3745640 

Runnable Code Ocean capsule: https://doi.org/10.24433/CO.0701423.v1

Requires Ubuntu 20.04 and ndnSIM 2.1.

## Install

```sh
sudo ./install.sh
```

## Run

```sh
./run.sh     # complete sweep, 1440 runs (~87 h), then tables and figures
```

Output goes to `results/`.

## Tables and figures results

`paper-results.zip` holds the measurements behind the paper: 120 CSV files,
six schemes x four dead-node levels x five seeds. Unzip it, then render the
tables and figures without running the simulator:

```sh
unzip paper-results.zip     # creates data/
python3 analysis/make_tables.py      # four result tables
python3 analysis/make_figures.py     # Figures 16 and 17
```


