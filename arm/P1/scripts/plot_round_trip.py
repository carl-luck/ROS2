import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 2:
        print('用法：python3 plot_round_trip.py CSV文件路径')
        return 1

    path = Path(sys.argv[1]).expanduser().resolve()

    phases = ['A_start', 'B', 'A_return']
    groups = {phase: [] for phase in phases}

    required = {
        'phase',
        'joint',
        'sim_time_s',
        'desired_rad',
        'actual_rad',
        'controller_error_rad',
    }

    with path.open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)

        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV 缺少所需列，请使用原始运动记录。')

        for line, row in enumerate(reader, start=2):
            if row['joint'] != 'shoulder_pan_joint':
                continue

            phase = row['phase']

            if phase not in groups:
                raise ValueError(f'第 {line} 行运动阶段未知：{phase}')

            values = [
                float(row['sim_time_s']),
                float(row['desired_rad']),
                float(row['actual_rad']),
                float(row['controller_error_rad']),
            ]

            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'第 {line} 行含无效数值。')

            groups[phase].append(values)

    for phase in phases:
        samples = groups[phase]

        if len(samples) < 2:
            raise ValueError(f'{phase} 样本不足。')

        times = [sample[0] for sample in samples]

        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f'{phase} 时间戳重复或倒退。')

        print(
            f'{phase}: {len(samples)} 帧，'
            f'仿真时间覆盖 {times[-1] - times[0]:.3f} s'
        )

    plt.rcParams.update({
        'font.size': 10,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'svg.fonttype': 'none',
    })

    fig, axes = plt.subplots(
        2,
        3,
        figsize=(12, 6),
        sharex='col',
        sharey='row',
        constrained_layout=True,
    )

    peak_mrad = max(
        abs(sample[3]) * 1000
        for samples in groups.values()
        for sample in samples
    )

    error_limit = max(peak_mrad * 1.2, 0.001)

    for column, phase in enumerate(phases):
        samples = groups[phase]
        first_time = samples[0][0]

        times = [
            sample[0] - first_time
            for sample in samples
        ]

        desired = [sample[1] for sample in samples]
        actual = [sample[2] for sample in samples]
        errors = [sample[3] * 1000 for sample in samples]

        position_ax = axes[0, column]
        error_ax = axes[1, column]

        position_ax.plot(
            times,
            desired,
            color='#0072B2',
            linewidth=2,
            label='Desired',
        )

        position_ax.plot(
            times,
            actual,
            color='#D55E00',
            linestyle='--',
            linewidth=1.5,
            label='Actual',
        )

        position_ax.set_title(
            f'{phase} (n={len(samples)})'
        )

        position_ax.legend(
            loc='upper left',
            frameon=False,
            fontsize=9,
        )

        error_ax.plot(
            times,
            errors,
            color='#0072B2',
            linewidth=1.5,
        )

        error_ax.axhline(
            0,
            color='0.4',
            linestyle='--',
            linewidth=0.8,
        )

        error_ax.set_ylim(-error_limit, error_limit)
        error_ax.set_xlabel('Time from first feedback (sim s)')

        for ax in [position_ax, error_ax]:
            ax.grid(alpha=0.25)
            ax.set_xlim(left=0)

    axes[0, 0].set_ylabel('Joint position (rad)')
    axes[1, 0].set_ylabel('Tracking error (mrad)')

    fig.suptitle(
        'UR5e shoulder pan: fixed-position round trip\n'
        'Error = desired - actual; recorded feedback samples',
        fontsize=13,
    )

    folder = path.parent.parent / 'figures'
    folder.mkdir(parents=True, exist_ok=True)

    for extension in ['png', 'svg']:
        output = folder / f'{path.stem}_tracking.{extension}'

        fig.savefig(
            output,
            dpi=300,
        )

        print(f'已保存：{output}')

    plt.close(fig)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())