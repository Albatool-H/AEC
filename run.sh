#!/usr/bin/env bash
set -euo pipefail

CODE="$(cd "$(dirname "$0")" && pwd)"
NDNSIM_DIR="${NDNSIM_DIR:-/opt/ndnsim}"
BASE="$NDNSIM_DIR/root/ndnSIM"
WORK="${AEC_WORK:-/tmp/aec-build}"
export AEC_RESULTS="${AEC_RESULTS:-$CODE/results}"

export LD_LIBRARY_PATH="$BASE/lib:${LD_LIBRARY_PATH:-}"
export PKG_CONFIG_PATH="$BASE/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

SWEEP="$AEC_RESULTS/sweep"
mkdir -p "$AEC_RESULTS" "$SWEEP"

echo "=============================================================="
echo " 1/3  Building the simulator"
echo "=============================================================="
rm -rf "$WORK"
cp -r "$CODE/sim" "$WORK"
mkdir -p "$WORK/scripts"
cp "$CODE/sweep/sweep.py" "$CODE/sweep/vw-bandit-agent.py" "$WORK/scripts/"
cd "$WORK"
chmod +x waf
python2 ./waf configure --prefix="$BASE"
python2 ./waf build
ls -la build/aec build/iot-baseline-ndn | awk '{print "   " $NF, $5, "bytes"}'

COMMON="--alphas=1.2,1.6,2.0 --cache-sizes=5,10,50,100 \
  --consumer-model=iot-oneshot --total-requests=10000 \
  --consumer-start-offset=30 --consumer-end-time=400 --sim-time=600"

WEIGHTS="--rl-w-producer=1 --rl-w-hop=1 --rl-w-delay=2 --rl-w-eviction=1 \
  --rl-w-cost=0.5 --rl-epsilon=0.05"

echo
echo "=============================================================="
echo " 2/3  Full sweep: six schemes x four dead-node levels x"
echo "      twelve cells x five seeds = 1440 runs, about 87 hours"
echo "=============================================================="

run_aec () {
    python3 scripts/sweep.py $COMMON $WEIGHTS --systems=pool --use-rl-bandit \
        --prog-pool=aec --topo-pool=topology/iot-topology.txt \
        --meta-pool=topology/iot-meta.json \
        --dead-nodes="$1" --seed="$2"
}

run_baseline () {
    local PROG=iot-baseline-ndn
    if [ "$1" = "lce" ]; then PROG=lce-baseline-ndn; fi
    python3 scripts/sweep.py $COMMON --systems=baseline \
        --cache-strategy="$1" --prog-baseline="$PROG" \
        --topo-baseline=topology/iot-topology-noctrl.txt \
        --dead-nodes="$2" --seed="$3"
}

collect () {
    local OUT="$SWEEP/comparison-$1-dn$2/seed$3"
    mkdir -p "$OUT"
    cp results/comparison/per-cell.csv "$OUT/per-cell.csv"
}

for DN in 0 30 40 50; do
    for SEED in 1 2 3 4 5; do
        run_aec "$DN" "$SEED"
        collect aec "$DN" "$SEED"
        for STRAT in lce lcd probcache btw poolcache; do
            run_baseline "$STRAT" "$DN" "$SEED"
            collect "$STRAT" "$DN" "$SEED"
        done
    done
done

echo
echo "=============================================================="
echo " 3/3  Rendering tables and figures from the sweep"
echo "=============================================================="
cd "$CODE"
export AEC_DATA="$SWEEP"
python3 analysis/make_tables.py
python3 analysis/make_figures.py

echo
find "$AEC_RESULTS" -type f -not -path "*/sweep/*" | sort
