import argparse
import csv
import json
import math
from pathlib import Path


PHASES = ['A_start', 'B', 'A_return']


def calculate(samples):
    if len(samples) < 2:
        raise ValueError('统计区间内不足两个样本。')

    times = [sample['sim_time_s'] for sample in samples]
    errors = [sample['controller_error_rad'] for sample in samples]

    span = times[-1] - times[0]

    square_integral = 0.0
    absolute_integral = 0.0

    for i in range(1, len(samples)):
        dt = times[i] - times[i - 1]

        square_integral += (
            errors[i - 1] ** 2 + errors[i] ** 2
        ) * dt / 2.0

        absolute_integral += (
            abs(errors[i - 1]) + abs(errors[i])
        ) * dt / 2.0

    peak_index = max(
        range(len(errors)),
        key=lambda i: abs(errors[i]),
    )

    consistency = max(
        abs(
            sample['controller_error_rad']
            - (
                sample['desired_rad']
                - sample['actual_rad']
            )
        )
        for sample in samples
    )

    return {
        'samples': len(samples),
        'trajectory_time_first_s': (
            samples[0]['desired_time_from_start_s']
        ),
        'trajectory_time_last_s': (
            samples[-1]['desired_time_from_start_s']
        ),
        'observed_span_sim_s': span,
        'sample_rmse_rad': math.sqrt(
            sum(error ** 2 for error in errors) / len(errors)
        ),
        'time_weighted_rmse_rad': math.sqrt(
            square_integral / span
        ),
        'max_abs_error_rad': abs(errors[peak_index]),
        'iae_rad_s': absolute_integral,
        'peak_trajectory_time_s': (
            samples[peak_index]['desired_time_from_start_s']
        ),
        'error_consistency_max_rad': consistency,
    }


def main():
    parser = argparse.ArgumentParser(
        description='六关节轨迹误差分析与执行结果关联'
    )

    parser.add_argument('csv_path')
    args = parser.parse_args()

    path = Path(args.csv_path).expanduser().resolve()
    report_path = path.with_name(path.stem + '_run.json')

    if not report_path.is_file():
        raise ValueError(
            f'缺少配套执行记录：{report_path}\n'
            '请使用本次已生成 JSON 的原始 CSV。'
        )

    with report_path.open(encoding='utf-8') as file:
        report = json.load(file)

    if report['feedback_csv'] != path.name:
        raise ValueError('JSON 记录的 CSV 文件名不匹配。')

    joints = report['joint_names']

    if len(joints) != 6 or len(set(joints)) != 6:
        raise ValueError('JSON 中的六关节名称不完整或重复。')

    hold_end = float(report['first_point_time_sim_s'])
    trajectory_end = float(report['segment_duration_sim_s'])

    if not (
        math.isfinite(hold_end)
        and math.isfinite(trajectory_end)
        and 0 <= hold_end < trajectory_end
    ):
        raise ValueError('JSON 中的轨迹时间配置无效。')

    segments = {}

    for segment in report['segments']:
        phase = segment['phase']

        if phase in segments:
            raise ValueError(f'JSON 中阶段重复：{phase}')

        segments[phase] = segment

    if set(segments) != set(PHASES):
        raise ValueError('JSON 缺少完整的三段执行记录。')

    groups = {
        (phase, joint): []
        for phase in PHASES
        for joint in joints
    }

    numeric_columns = [
        'sim_time_s',
        'desired_time_from_start_s',
        'desired_rad',
        'actual_rad',
        'controller_error_rad',
    ]

    required = set(numeric_columns) | {'phase', 'joint'}

    with path.open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)

        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV 缺少所需列，请使用原始反馈 CSV。')

        for line, row in enumerate(reader, start=2):
            key = (row['phase'], row['joint'])

            if key not in groups:
                raise ValueError(
                    f'第 {line} 行阶段或关节名称未知：{key}'
                )

            sample = {
                name: float(row[name])
                for name in numeric_columns
            }

            if not all(
                math.isfinite(value)
                for value in sample.values()
            ):
                raise ValueError(f'第 {line} 行含无效数值。')

            groups[key].append(sample)

    for (phase, joint), samples in groups.items():
        if len(samples) < 2:
            raise ValueError(f'{phase}/{joint} 样本不足。')

        for previous, current in zip(samples, samples[1:]):
            dt_sim = (
                current['sim_time_s']
                - previous['sim_time_s']
            )

            dt_trajectory = (
                current['desired_time_from_start_s']
                - previous['desired_time_from_start_s']
            )

            if dt_sim <= 0 or dt_trajectory <= 0:
                raise ValueError(
                    f'{phase}/{joint} 时间戳重复或倒退。'
                )

            if abs(dt_sim - dt_trajectory) > 1e-5:
                raise ValueError(
                    f'{phase}/{joint} 仿真时间与轨迹时间推进不一致。'
                )

    for phase in PHASES:
        reference = groups[(phase, joints[0])]
        expected_frames = segments[phase]['feedback_frames']

        if len(reference) != expected_frames:
            raise ValueError(
                f'{phase} CSV 帧数与 JSON 记录不一致。'
            )

        for joint in joints[1:]:
            samples = groups[(phase, joint)]

            if len(samples) != len(reference):
                raise ValueError(f'{phase} 六关节样本数量不一致。')

            for a, b in zip(reference, samples):
                if (
                    a['sim_time_s'] != b['sim_time_s']
                    or a['desired_time_from_start_s']
                    != b['desired_time_from_start_s']
                ):
                    raise ValueError(
                        f'{phase} 六关节反馈时间未对齐。'
                    )

    results = []

    for phase in PHASES:
        segment = segments[phase]

        for joint in joints:
            samples = groups[(phase, joint)]

            motion_samples = [
                sample
                for sample in samples
                if hold_end
                <= sample['desired_time_from_start_s']
                <= trajectory_end
            ]

            for scope, selected in [
                ('full', samples),
                ('planned_motion', motion_samples),
            ]:
                metrics = calculate(selected)

                result = {
                    'phase': phase,
                    'joint': joint,
                    'scope': scope,
                    'segment_duration_sim_s': trajectory_end,
                    **metrics,
                    'action_status': segment['action_status'],
                    'controller_error_code': (
                        segment['controller_error_code']
                    ),
                    'segment_checks_passed': (
                        segment['checks_passed']
                    ),
                    'experiment_passed': (
                        report['experiment_passed']
                    ),
                    'wall_send_to_result_s': (
                        segment['wall_send_to_result_s']
                    ),
                    'sim_send_to_result_s': (
                        segment['sim_send_to_result_s']
                    ),
                    'max_endpoint_deviation_rad': (
                        segment['max_endpoint_deviation_rad']
                    ),
                }

                results.append(result)

                if metrics['error_consistency_max_rad'] > 1e-6:
                    print(
                        f'注意：{phase}/{joint}/{scope} '
                        '控制器误差与 desired-actual 存在差异。'
                    )

    output = path.with_name(path.stem + '_metrics.csv')

    with output.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(results[0]),
        )

        writer.writeheader()
        writer.writerows(results)

    print(f'关联执行记录：{report_path.name}')
    print(f'整次实验检查通过：{report["experiment_passed"]}')
    print(f'统计区间：全段，以及轨迹时间 {hold_end}～{trajectory_end}s')
    print('积分使用仿真时间与梯形法，仅覆盖所选样本范围。')
    print()

    for result in results:
        if (
            result['joint'] == 'shoulder_pan_joint'
            and result['scope'] == 'planned_motion'
        ):
            print(f'[{result["phase"]} 底座计划运动阶段]')
            print(
                f'  轨迹时间覆盖：'
                f'{result["trajectory_time_first_s"]:.3f}'
                f'～{result["trajectory_time_last_s"]:.3f} s'
            )
            print(f'  样本数：{result["samples"]}')
            print(
                f'  样本 RMSE：'
                f'{result["sample_rmse_rad"]:.8f} rad'
            )
            print(
                f'  时间加权 RMSE：'
                f'{result["time_weighted_rmse_rad"]:.8f} rad'
            )
            print(
                f'  最大绝对误差：'
                f'{result["max_abs_error_rad"]:.8f} rad'
            )
            print(f'  IAE：{result["iae_rad_s"]:.8f} rad·s')
            print()

    print(f'完整六关节结果：{output}')
    print(f'结果行数：{len(results)}，不含表头。')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())