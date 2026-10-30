# P2 复现与版本依据

## 环境准备

先按[P1说明](../../P1/README.md)和[依赖源码清单](../../P1/dependencies.repos)获取、构建官方仿真依赖。P2依赖ur_description、ur_moveit_config、ur_simulation_gz、ros2_control及本包package.xml中声明的ROS组件。检查这些包已可由ament找到。

系统需Python 3.10虚拟环境支持。若`python3 -m venv`提示ensurepip缺失，先安装Ubuntu的`python3.10-venv`。然后：

```bash
cd ~/ROS2/arm/P2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r scripts/requirements_p2_six.txt
python -c "import numpy, scipy, osqp, matplotlib, fcl, trimesh; print('dependencies import OK')"
```

system-site-packages用于使用ROS环境及已安装的NumPy/SciPy/Matplotlib。正式实验版本记录在README和各run.json；requirements只固定额外pip依赖，不锁定系统包。构建和运行命令见[P2入口](../README.md)。

## 两份实际运行源码

1. `experiments/six_joint_small_before_demo_20261031_021856/`：对应小幅三轮对比和异常验证，原始备份在增加demo前由用户保存。
2. `experiments/six_joint_demo_20261031/`：对应大幅demo，与当前包中核心文件一致。

两者的p2_six_core.py、p2_six_control.py和run_p2_six_module.py分别匹配对应批次记录的SHA256，离线复核会实际检查。支持模块zero_hold.py、joint_interface_test.py、启动/控制器配置和package.xml在整理时一起快照；六关节日志未单独记录这些支持文件的哈希，不能宣称已逐一证明其历史字节完全一致。

不要在正在运行的工作空间中同时加载两套同名模块。历史小幅版如需精确重跑，应先在隔离副本中恢复对应三个文件到包/脚本位置，并依赖配套支持文件；当前最新版的standard模式仍可做小幅测试，但不能冒充历史源码逐字一致。所有重跑应产生新批次，保留原始结果。

历史Git状态为dirty，P2当时主要未跟踪。旧code_commit字段不足以定位实际源码，本次提交只是首次将P2整理纳入版本管理，不是追溯修改历史日志。后续实验建议在运行前提交代码，并记录新的提交号和dirty状态。

## 离线复核

```bash
cd ~/ROS2/arm/P2
python3 scripts/analyze_portfolio.py
```

仅需Python标准库，不启动ROS2、不发布命令、不读取当前仿真。默认读取evidence压缩包，重算并覆盖results内的派生CSV/JSON，不修改原始证据。另存结果：

```bash
python3 scripts/analyze_portfolio.py --output /tmp/p2_recomputed_results
```

复核包括：10项正式案例的状态和停止判据、实际/输出采样速度、控制请求变化率、三类注入及恢复、核心/控制/编排源码哈希、每次RMSE和IAE与原记录一致性。异常SIGKILL请求进程不能执行finally，raw run.json保持running是预期，需结合case.json和guard日志。

每次先按各自记录窗口做梯形积分，再汇总三次均值和样本标准差；不同运行端点有毫秒级差异，没有重新裁剪成共同窗口。相对参考从各次实际初态生成，不宣称相同绝对初态。没有做显著性检验。

## 文件策略

data_v1～v5和非代表性图片保留在原机器但不直接加入Git。evidence收录选定正式小幅及demo两个批次的压缩原件，附SHA256；其余开发失败不删除，但未全部发布。构建目录、虚拟环境、日志缓存不发布。实验快照和图只作为复现证据，不是额外运行入口。
