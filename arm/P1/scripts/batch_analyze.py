import subprocess
import sys
from pathlib import Path


def main():
    project = Path(__file__).resolve().parent.parent
    data_folder = project / 'data'
    analyzer = project / 'scripts' / 'analyze_round_trip.py'

    if not analyzer.is_file():
        print(f'没有找到分析脚本：{analyzer}')
        return 1

    reports = sorted(
        data_folder.glob('round_trip_T*_run.json')
    )

    if not reports:
        print('没有找到配套的运行记录，未执行分析。')
        return 1

    succeeded = []
    failed = []

    for index, report in enumerate(reports, start=1):
        csv_path = report.with_name(
            report.name.removesuffix('_run.json') + '.csv'
        )

        print(
            f'\n[{index}/{len(reports)}] {csv_path.name}',
            flush=True,
        )

        if not csv_path.is_file():
            print('缺少原始 CSV，记录为失败。')
            failed.append(csv_path.name)
            continue

        result = subprocess.run([
            sys.executable,
            str(analyzer),
            str(csv_path),
        ])

        if result.returncode == 0:
            succeeded.append(csv_path.name)
        else:
            failed.append(csv_path.name)

    print('\n批量分析结束')
    print(f'候选数据组：{len(reports)}')
    print(f'分析成功：{len(succeeded)}')
    print(f'分析失败：{len(failed)}')

    if failed:
        print('\n失败清单：')
        for name in failed:
            print(f'  {name}')

    return 0 if not failed else 1


if __name__ == '__main__':
    raise SystemExit(main())