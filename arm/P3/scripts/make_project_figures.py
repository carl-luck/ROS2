import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description='Reproduce project figures from results/source; offline only.')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    root = args.root
    src = root/'results/source'; out = root/'results/figures'; out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'svg.fonttype': 'none', 'pdf.fonttype': 42})
    blue, orange, red = '#0072B2', '#D55E00', '#CC6677'
    metadata = {'purpose': 'Exploratory project results; no significance or reliability claim',
                'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in src.iterdir() if p.is_file()},
                'profile': {}, 'figures': []}

    def save(fig, name, caption):
        fig.canvas.draw()
        # All axes occupy a positive area inside the fixed-size canvas.
        for ax in fig.axes:
            box = ax.get_position()
            assert box.width > 0 and box.height > 0 and box.x0 >= 0 and box.y0 >= 0 and box.x1 <= 1.01 and box.y1 <= 1.01
        for fmt in ('png', 'svg', 'pdf'):
            fig.savefig(out/(name+'.'+fmt), dpi=300)
        metadata['figures'].append({'name': name, 'caption': caption, 'size_inches': fig.get_size_inches().tolist()})
        plt.close(fig)

    planning = list(csv.DictReader((src/'planning_trials.csv').open()))
    assert len(planning) == 20
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    for i, (label, color) in enumerate([('baseline', blue), ('dense', orange)]):
        group = [r for r in planning if r['condition'] == label]
        count = sum(r['validated'] == 'True' for r in group)
        metadata['profile'][label] = {'n': len(group), 'validated': count,
                                     'missing_planning_time': sum(not r['planning_time_s'] for r in group)}
        axes[0].bar(i, count, color=color, width=.55)
        axes[0].text(i, count+.25, str(count)+'/10', ha='center')
        for r in group:
            x, y = int(r['trial']), float(r['planning_time_s'])*1000
            valid = r['validated'] == 'True'
            axes[1].scatter(x, y, color=color if valid else red, marker='o' if label == 'baseline' else '^',
                            facecolors=color if valid else 'none', s=45)
    axes[0].set(xticks=[0, 1], xticklabels=['Baseline\n0.01', 'Dense\n0.001'], ylim=(0, 11.5),
                ylabel='Validated paths (out of 10)', title='A  External path validation')
    axes[1].plot([], [], 'o', color=blue, label='Baseline')
    axes[1].plot([], [], '^', color=orange, label='Dense')
    axes[1].plot([], [], 'o', markerfacecolor='none', color=red, label='Rejected path')
    axes[1].set(xlabel='Trial within condition', ylabel='Planning interface time (ms)',
                title='B  All 20 requests', xticks=[1, 3, 5, 7, 10], ylim=(0, None))
    axes[1].legend(frameon=False, fontsize=8)
    save(fig, 'planning_resolution', '10 baseline then 10 dense trials; identical scene and external validation. All requests shown, including failures. No confidence intervals or paired seeds.')

    summary = json.loads((src/'gazebo_comparison_report.json').read_text())
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for mode, color, style in [('p', blue, '-'), ('mpc', orange, '--')]:
        rows = list(csv.DictReader((src/(mode+'_tracking_log.csv')).open()))
        assert len(rows) > 9000
        times = np.array([float(r['elapsed_sim_s']) for r in rows])
        names = [k[2:] for k in rows[0] if k.startswith('q_')]
        e = np.array([[float(r['q_'+n])-float(r['ref_'+n]) for n in names] for r in rows])
        cost = np.array([float(r['compute_wall_s'])*1000 for r in rows])
        assert np.all(np.isfinite(e)) and np.all(np.isfinite(cost)) and np.all(np.diff(times) > 0)
        rms = np.sqrt(np.mean(e**2, axis=1))
        axes[0].plot(times, np.maximum(rms, 1e-12), color=color, linestyle=style, linewidth=1, label=mode.upper())
        per_joint = np.array(summary['runs'][mode]['per_joint_rmse_rad'])*1e6
        axes[1].plot(np.arange(1, 7), per_joint, 'o' if mode == 'p' else '^', color=color, label=mode.upper())
        sorted_cost = np.sort(cost)
        axes[2].plot(sorted_cost, np.arange(1, len(cost)+1)/len(cost), color=color, linestyle=style, label=mode.upper())
        metadata['profile'][mode] = {'samples': len(rows), 'missing_numeric': 0, 'duration_log_s': float(times[-1]),
                                      'compute_over_20ms': int(sum(cost > 20))}
    axes[0].set(xlabel='Elapsed simulation time (s)', ylabel='Across-joint RMS error (rad)', yscale='log',
                title='A  Online error history')
    axes[0].legend(frameon=False)
    axes[1].set(xlabel='Joint index (published order)', ylabel='Joint RMSE (micro-rad)', title='B  Motion-period error',
                xticks=np.arange(1, 7), ylim=(0, None))
    axes[1].legend(frameon=False)
    axes[2].axvline(20, color='#777777', linestyle=':', linewidth=1)
    axes[2].set(xlabel='Controller compute time (ms)', ylabel='Empirical cumulative fraction',
                title='C  Compute-time distribution', xscale='log', ylim=(0, 1.03))
    axes[2].legend(frameon=False)
    save(fig, 'gazebo_tracking', 'One full run per controller, same reference and limits, slightly different initial states. A displays all logged samples including settling; plot floor 1e-12 rad. B uses motion-period samples. C includes all control cycles; vertical line is 20 ms. No hardware accuracy or statistical superiority claim.')
    (out/'figure_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    print('FIGURES:', out)


if __name__ == '__main__':
    main()
