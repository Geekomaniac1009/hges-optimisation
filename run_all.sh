#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  run_all.sh — Compile, run experiments, generate graphs
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${CYAN}[RUN]${NC} $*"; }
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }

RESULTS_DIR="results"
GRAPHS_DIR="graphs"

# ─── Step 1: Compile ───
log "Compiling hges.cpp..."
g++ -O2 -std=c++17 -o hges hges.cpp 2>/dev/null
ok "Compiled → ./hges"

mkdir -p "$RESULTS_DIR" "$GRAPHS_DIR"

# ─── Step 2: Battery capacity sweep (no error, h_l=0.99) ───
log "Experiment 1: Battery capacity sweep"
./hges \
    --battery 0 2000 4000 8000 10000 15000 20000 \
    --solar_dev 0 --task_dev 0 \
    --spike 25 --leakage 0.99 \
    --output "$RESULTS_DIR/battery_sweep.csv"
ok "→ $RESULTS_DIR/battery_sweep.csv"

# ─── Step 3: Error rate sweep (fixed battery=10000) ───
log "Experiment 2: Prediction error sweep"
./hges \
    --battery 10000 \
    --solar_dev 0 5 10 20 \
    --task_dev  0 5 10 20 \
    --spike 25 --leakage 0.99 \
    --output "$RESULTS_DIR/error_sweep.csv"
ok "→ $RESULTS_DIR/error_sweep.csv"

# ─── Step 4: Leakage sweep (fixed battery=10000, no error) ───
log "Experiment 3: Leakage sensitivity"
for hl in 0.50 0.75 0.85 0.90 0.95 0.99; do
    ./hges \
        --battery 10000 \
        --solar_dev 0 --task_dev 0 \
        --spike 25 --leakage "$hl" \
        --output "$RESULTS_DIR/leakage_${hl}.csv"
done
# Merge leakage CSVs
head -1 "$RESULTS_DIR/leakage_0.50.csv" > "$RESULTS_DIR/leakage_sweep.csv"
for hl in 0.50 0.75 0.85 0.90 0.95 0.99; do
    tail -n +2 "$RESULTS_DIR/leakage_${hl}.csv" >> "$RESULTS_DIR/leakage_sweep.csv"
done
ok "→ $RESULTS_DIR/leakage_sweep.csv"

# ─── Step 5: Generate graphs ───
log "Generating graphs..."
python3 plot_results.py \
    --csv "$RESULTS_DIR/battery_sweep.csv" \
          "$RESULTS_DIR/error_sweep.csv" \
          "$RESULTS_DIR/leakage_sweep.csv" \
    --outdir "$GRAPHS_DIR"
ok "→ $GRAPHS_DIR/"

# ─── Summary ───
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  All done. Results:"
echo "    CSV:    $RESULTS_DIR/"
ls -1 "$RESULTS_DIR"/*.csv 2>/dev/null | sed 's/^/      /'
echo "    Graphs: $GRAPHS_DIR/"
ls -1 "$GRAPHS_DIR"/*.svg 2>/dev/null | sed 's/^/      /'
echo "═══════════════════════════════════════════════════════════════"
