import csv
import math
import sys
from pathlib import Path


def main():
    if len(sys.argv) != 2:
        print('用法：python3 analyze_round_trip.py CSV文件路径')
        return 1

    path = Path(sys.argv[1]).expanduser()

    phases = ['A_start', 'B', 'A_return']
    groups = {phase: [] for phase in phases}

    required = {
        'phase',
        'sim_time_s',
        'joint',
        'desired_rad',
        'actual_rad',
        'controller_error_rad',
    }

    with path.open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)

        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV 缺少所需列，请确认选择的是原始运动记录。')

        for line, row in enumerate(reader, start=2):
            if row['joint'] != 'shoulder_pan_joint':
                continue

            phase = row['phase']

            if phase not in groups:
                raise ValueError(f'第 {line} 行出现未知运动阶段：{phase}')

            values = [
                float(row['sim_time_s']),
                float(row['desired_rad']),
                float(row['actual_rad']),
                float(row['controller_error_rad']),
            ]

            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'第 {line} 行含有无效数值。')

            groups[phase].append(values)

    results = []

    print('只分析底座关节 shoulder_pan_joint')
    print('统计范围：每段全部已记录反馈，包括开始时的保持阶段。')
    print()

    for phase in phases:
        samples = groups[phase]

        if len(samples) < 2:
            raise ValueError(f'{phase} 有效样本不足，不能完成分析。')

        times = [sample[0] for sample in samples]
        errors = [sample[3] for sample in samples]

        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f'{phase} 时间戳重复或倒退，请先检查原始数据。')

        rmse = math.sqrt(
            sum(error * error for error in errors) / len(errors)
        )

        peak_index = max(
            range(len(errors)),
            key=lambda i: abs(errors[i]),
        )

        max_error = abs(errors[peak_index])

        consistency = max(
            abs(error - (desired - actual))
            for _, desired, actual, error in samples
        )

        span = times[-1] - times[0]
        peak_time = times[peak_index] - times[0]

        print(f'[{phase}]')
        print(f'  样本数：{len(samples)}')
        print(f'  反馈时间覆盖：{span:.3f} s（仿真时间）')
        print(
            f'  RMSE：{rmse:.8f} rad'
            f' = {math.degrees(rmse):.5f}°'
        )
        print(
            f'  最大绝对误差：{max_error:.8f} rad'
            f' = {math.degrees(max_error):.5f}°'
        )
        print(f'  最大误差发生在首帧后：{peak_time:.3f} s')
        print(f'  error 与 desired-actual 最大差异：{consistency:.3e} rad')

        if consistency > 1e-6:
            print('  注意：两种误差的差异需要进一步检查。')

        print()

        results.append([
            phase,
            len(samples),
            span,
            rmse,
            math.degrees(rmse),
            max_error,
            math.degrees(max_error),
            peak_time,
            consistency,
        ])

    output = path.with_name(path.stem + '_summary.csv')

    with output.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)

        writer.writerow([
            'phase',
            'samples',
            'feedback_span_sim_s',
            'rmse_rad',
            'rmse_deg',
            'max_abs_error_rad',
            'max_abs_error_deg',
            'peak_time_from_first_feedback_s',
            'error_consistency_max_rad',
        ])

        writer.writerows(results)

    print(f'统计结果已保存：{output}')
    print('反馈时间覆盖不等于完整轨迹执行耗时，也不是墙钟耗时。')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())