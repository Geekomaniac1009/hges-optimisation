#!/usr/bin/env python3
"""
plot_results.py — Generate publication-quality graphs from H-GES results CSV.

Reads the output CSV from hges and produces:
  1. Grid cost vs battery capacity (per scenario)
  2. Grid cost vs error rate (at fixed battery)
  3. Grid cost vs leakage (at fixed battery)

Usage:
  python3 plot_results.py --csv results/battery_sweep.csv --outdir graphs/
  python3 plot_results.py --csv results/leakage_sweep.csv --outdir graphs/
  python3 plot_results.py  # uses defaults
"""

import argparse, os, sys
import csv
from collections import defaultdict

def read_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            r['BatteryCap'] = float(r['BatteryCap'])
            r['Leakage'] = float(r['Leakage'])
            r['GridCost'] = float(r['GridCost'])
            r['GridEnergy'] = float(r['GridEnergy'])
            rows.append(r)
    return rows

# ─── SVG Chart Generator (no dependencies) ───

COLORS = {
    'NaiveGrid': '#888888',
    'GG':        '#e74c3c',
    'SDC':       '#3498db',
    'SP+SDC':    '#2ecc71',
    'Lyapunov':  '#9b59b6',
    'SF-Only':   '#f39c12',
    'GF-Only':   '#1abc9c',
    'H-GES':     '#e91e63',
}
MARKERS = {
    'NaiveGrid': 'square', 'GG': 'circle', 'SDC': 'diamond',
    'SP+SDC': 'triangle', 'Lyapunov': 'star', 'SF-Only': 'cross',
    'GF-Only': 'plus', 'H-GES': 'circle',
}
ALGO_ORDER = ['NaiveGrid','SDC','GG','SP+SDC','Lyapunov','SF-Only','GF-Only','H-GES']
DASHES = {
    'NaiveGrid': '8,4', 'GG': '4,2', 'SDC': '6,3', 'SP+SDC': '2,2',
    'Lyapunov': '10,3', 'SF-Only': '4,4', 'GF-Only': '6,2,2,2', 'H-GES': '',
}

def svg_line_chart(title, xlabel, ylabel, series_data, x_values, outpath, width=700, height=420):
    """
    series_data: dict of algo_name → list of y-values (same length as x_values)
    """
    margin = {'top': 50, 'right': 180, 'bottom': 60, 'left': 90}
    pw = width - margin['left'] - margin['right']
    ph = height - margin['top'] - margin['bottom']

    all_y = [v for s in series_data.values() for v in s if v is not None]
    if not all_y: return
    ymin = min(all_y) * 0.92
    ymax = max(all_y) * 1.05

    def sx(i): return margin['left'] + (i / max(1, len(x_values)-1)) * pw
    def sy(v): return margin['top'] + ph - ((v - ymin) / max(1, ymax - ymin)) * ph

    lines = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
                 f'font-family="Arial,sans-serif" font-size="12">')
    lines.append(f'<rect width="{width}" height="{height}" fill="white"/>')

    # Title
    lines.append(f'<text x="{width//2}" y="25" text-anchor="middle" font-size="15" font-weight="bold">{title}</text>')

    # Grid lines
    n_yticks = 6
    for i in range(n_yticks + 1):
        yv = ymin + (ymax - ymin) * i / n_yticks
        y = sy(yv)
        lines.append(f'<line x1="{margin["left"]}" y1="{y:.1f}" x2="{margin["left"]+pw}" y2="{y:.1f}" '
                     f'stroke="#e0e0e0" stroke-width="0.5"/>')
        label = f'{yv/1000:.0f}K' if yv >= 1000 else f'{yv:.0f}'
        lines.append(f'<text x="{margin["left"]-8}" y="{y+4:.1f}" text-anchor="end" font-size="10">{label}</text>')

    # X-axis labels
    for i, xv in enumerate(x_values):
        x = sx(i)
        lines.append(f'<line x1="{x:.1f}" y1="{margin["top"]+ph}" x2="{x:.1f}" y2="{margin["top"]+ph+5}" stroke="#333"/>')
        label = str(int(xv)) if xv == int(xv) else f'{xv}'
        lines.append(f'<text x="{x:.1f}" y="{margin["top"]+ph+20}" text-anchor="middle" font-size="10">{label}</text>')

    # Axes
    lines.append(f'<line x1="{margin["left"]}" y1="{margin["top"]}" x2="{margin["left"]}" '
                 f'y2="{margin["top"]+ph}" stroke="#333" stroke-width="1.5"/>')
    lines.append(f'<line x1="{margin["left"]}" y1="{margin["top"]+ph}" x2="{margin["left"]+pw}" '
                 f'y2="{margin["top"]+ph}" stroke="#333" stroke-width="1.5"/>')

    # Axis labels
    lines.append(f'<text x="{margin["left"]+pw//2}" y="{height-10}" text-anchor="middle" font-size="12">{xlabel}</text>')
    lines.append(f'<text x="15" y="{margin["top"]+ph//2}" text-anchor="middle" font-size="12" '
                 f'transform="rotate(-90,15,{margin["top"]+ph//2})">{ylabel}</text>')

    # Data lines
    for algo in ALGO_ORDER:
        if algo not in series_data: continue
        vals = series_data[algo]
        color = COLORS.get(algo, '#333')
        dash = DASHES.get(algo, '')
        sw = '2.5' if algo == 'H-GES' else '1.5'
        points = [(sx(i), sy(v)) for i, v in enumerate(vals) if v is not None]
        if not points: continue
        path = ' '.join(f'{"M" if j==0 else "L"}{x:.1f},{y:.1f}' for j,(x,y) in enumerate(points))
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ''
        lines.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{sw}"{dash_attr}/>')
        # Dots
        for x, y in points:
            r = '4' if algo == 'H-GES' else '3'
            lines.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{color}"/>')

    # Legend
    lx = margin['left'] + pw + 15
    ly = margin['top'] + 10
    for algo in ALGO_ORDER:
        if algo not in series_data: continue
        color = COLORS.get(algo, '#333')
        fw = ' font-weight="bold"' if algo == 'H-GES' else ''
        lines.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+20}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
        lines.append(f'<circle cx="{lx+10}" cy="{ly}" r="3" fill="{color}"/>')
        lines.append(f'<text x="{lx+25}" y="{ly+4}" font-size="11"{fw}>{algo}</text>')
        ly += 18

    lines.append('</svg>')

    with open(outpath, 'w') as f:
        f.write('\n'.join(lines))
    print(f'  [GRAPH] {outpath}')


def plot_battery_sweep(rows, outdir, scenario='S0_T0'):
    """Fig: Grid cost vs battery capacity for a given scenario."""
    filtered = [r for r in rows if r['Scenario'] == scenario]
    if not filtered: return

    algos = sorted(set(r['Algorithm'] for r in filtered), key=lambda a: ALGO_ORDER.index(a) if a in ALGO_ORDER else 99)
    caps = sorted(set(r['BatteryCap'] for r in filtered))

    series = {}
    for algo in algos:
        vals = []
        for cap in caps:
            matches = [r for r in filtered if r['Algorithm'] == algo and r['BatteryCap'] == cap]
            vals.append(matches[0]['GridCost'] if matches else None)
        series[algo] = vals

    svg_line_chart(
        f'Grid Cost vs Battery Capacity ({scenario})',
        'Battery Capacity (W·slot)', 'Grid Cost ($)',
        series, caps,
        os.path.join(outdir, f'battery_sweep_{scenario}.svg')
    )


def plot_error_rate(rows, outdir, battery=10000):
    """Fig: Grid cost vs error rate at fixed battery."""
    filtered = [r for r in rows if abs(r['BatteryCap'] - battery) < 1]
    if not filtered: return

    # Extract scenarios and sort by error %
    scenarios = sorted(set(r['Scenario'] for r in filtered))
    # Parse error % from scenario name like S5_T5 → 5
    def err_pct(s):
        parts = s.replace('S','').replace('T','').split('_')
        return int(parts[0]) if parts[0].isdigit() else 0
    scenarios = sorted(scenarios, key=err_pct)
    x_vals = [err_pct(s) for s in scenarios]

    algos = sorted(set(r['Algorithm'] for r in filtered), key=lambda a: ALGO_ORDER.index(a) if a in ALGO_ORDER else 99)

    series = {}
    for algo in algos:
        vals = []
        for sc in scenarios:
            matches = [r for r in filtered if r['Algorithm'] == algo and r['Scenario'] == sc]
            vals.append(matches[0]['GridCost'] if matches else None)
        series[algo] = vals

    if len(x_vals) > 1:
        svg_line_chart(
            f'Grid Cost vs Prediction Error (Battery={int(battery)})',
            'Deviation (%)', 'Grid Cost ($)',
            series, x_vals,
            os.path.join(outdir, f'error_rate_bat{int(battery)}.svg')
        )


def plot_leakage(rows, outdir, battery=10000):
    """Fig: Grid cost vs leakage at fixed battery."""
    filtered = [r for r in rows if abs(r['BatteryCap'] - battery) < 1]
    if not filtered: return

    leakages = sorted(set(r['Leakage'] for r in filtered))
    if len(leakages) <= 1: return

    algos = sorted(set(r['Algorithm'] for r in filtered), key=lambda a: ALGO_ORDER.index(a) if a in ALGO_ORDER else 99)

    series = {}
    for algo in algos:
        vals = []
        for lk in leakages:
            matches = [r for r in filtered if r['Algorithm'] == algo and abs(r['Leakage'] - lk) < 0.001]
            vals.append(matches[0]['GridCost'] if matches else None)
        series[algo] = vals

    svg_line_chart(
        f'Grid Cost vs Battery Leakage (Battery={int(battery)})',
        'Leakage Factor (h_l)', 'Grid Cost ($)',
        series, leakages,
        os.path.join(outdir, f'leakage_bat{int(battery)}.svg')
    )


def main():
    parser = argparse.ArgumentParser(description='Plot H-GES results')
    parser.add_argument('--csv', nargs='+', default=['results/battery_sweep.csv'],
                        help='Input CSV file(s)')
    parser.add_argument('--outdir', default='graphs', help='Output directory for graphs')
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # Merge all CSV files
    all_rows = []
    for path in args.csv:
        if os.path.exists(path):
            all_rows.extend(read_csv(path))
            print(f'  [READ] {path} ({len(read_csv(path))} rows)')
        else:
            print(f'  [WARN] {path} not found, skipping')

    if not all_rows:
        print('No data loaded. Run hges first.')
        sys.exit(1)

    # Detect scenarios
    scenarios = sorted(set(r['Scenario'] for r in all_rows))
    batteries = sorted(set(r['BatteryCap'] for r in all_rows))
    leakages = sorted(set(r['Leakage'] for r in all_rows))

    print(f'  Scenarios: {scenarios}')
    print(f'  Batteries: {[int(b) for b in batteries]}')
    print(f'  Leakages:  {leakages}')
    print()

    # Battery sweep graph for each scenario (if multiple batteries)
    if len(batteries) > 1:
        for sc in scenarios:
            plot_battery_sweep(all_rows, args.outdir, sc)

    # Error rate graph (if multiple scenarios at same battery)
    if len(scenarios) > 1:
        for bat in batteries:
            plot_error_rate(all_rows, args.outdir, bat)

    # Leakage graph (if multiple leakage values)
    if len(leakages) > 1:
        for bat in batteries:
            plot_leakage(all_rows, args.outdir, bat)

    print(f'\n  All graphs written to {args.outdir}/')


if __name__ == '__main__':
    main()
