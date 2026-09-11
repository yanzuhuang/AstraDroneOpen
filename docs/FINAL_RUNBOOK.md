# 最终用户运行手册

以下命令均从仓库根目录执行，使用现有已构建环境：

```bash
cd /home/yanzu/AstraDroneOpen
```

运行前结束上一轮仿真。带 `--control` 的巡检命令会自动解锁、起飞并执行任务。正常结束时先等任务落地、解除武装，再停止仿真；下述停止操作均不是返航或降落指令。结果路径相对于仓库根目录，以启动终端打印的当次路径为准。

## 1. 三机多高度巡塔

用途：三机两层巡塔，当前高度为 UAV1 26→22 m、UAV2 20→16 m、UAV3 14→10 m。

```bash
./scripts/run_sh/three_uav_multi_height_inspection.sh --multi-layer --control --gui --rviz --record light
```

需要各机只绕一层时，将 `--multi-layer` 换成 `--single-layer`。两种方式均保持多高度 profile 禁用 Learning Speed。

停止：启动终端按 `Ctrl+C`，等待录包、进程和绘图收尾。

结果：`runtime_artifacts/three_uav_inspection_control_<时间戳>/`，包含 `summary.json`、`uav1.csv`～`uav3.csv`、`swarm.csv`、`candidates.jsonl`、`three_uav_inspection.bag`、日志和 metadata；有有效轨迹数据时生成 `trajectory_xy.png`、`trajectory_3d.png`。

## 2. 三机同高度／普通三机巡塔

用途：三机在 3.0 m 同高度、单层绕塔。

```bash
./scripts/run_sh/three_uav_inspection.sh --control --gui --rviz --record light
```

停止和结果目录与多高度入口相同，每轮使用新目录。两个命令均打开三机 RViz，局部膨胀地图显示默认开启。

两种三机入口的 `light` 录制不含完整点云和 Gazebo model states；需要这些数据时改为 `--record full`。默认不设时限；`--duration SEC` 是按墙钟时间中断运行，不表示完成任务。

## 3. 单机 EGO 巡检

八扇区巡塔：

```bash
./scripts/run_sh/sector_inspection.sh --control --sector-limit 8 --gui --rviz --attach
```

当前默认是 34→30 m 两层、1 cycle。停止时在另一个终端切到仓库根目录执行：

```bash
./scripts/run_sh/sector_inspection.sh --stop
```

结果：`runtime_artifacts/sector_inspection_control_<时间戳>/control.csv` 和 `ros_logs/`；`--stop` 在有 CSV 时尝试生成轨迹／高度图。默认不录 bag；需要录制时，在启动命令中追加 `--bag` 和本仓库 `runtime_artifacts/` 下一个新的绝对 `.bag` 路径，停止时等待 recorder 完成。

单机 waypoint 巡检是另一种保留任务模式：

```bash
./scripts/run_sh/ego_waypoint_inspection.sh --control --scenario tower --gui --rviz --attach
```

停止：

```bash
./scripts/run_sh/ego_waypoint_inspection.sh --stop
```

结果：`runtime_artifacts/ego_waypoint_inspection_tower_<时间戳>/mission.csv` 和 `ros_logs/`。本入口没有 bag 选项。`--scenario single|dual|tower` 是单机任务场景选择，不是无人机数量。

两个单机入口通过 tmux 运行，`Ctrl+b d` 只脱离界面，任务继续执行；使用各自的 `--stop` 收尾。

## 4. Forest SAC

用途：Forest 地图池上的 SAC 训练，以及独立切图检查、短程 smoke 和 evaluation。

**本手册提供入口，不解除既有训练资格限制。尚未获准的正式训练／evaluation 不应因文档列出命令而启动；本轮没有产生新的训练放行证据。** Forest 和 Worksite SAC 都没有 `--control` 开关，非帮助命令可能准备／构建 Hector overlay 并启动仿真。

获准执行相应阶段后，选用以下模式之一：

```bash
./scripts/run_sh/learning_speed_forest_sac.sh --mode preflight --gui
```

`preflight` 启动 Gazebo 并运行三地图 reset 生命周期，不执行 Episode、Replay 或 learner；它不是只读环境检查。

```bash
./scripts/run_sh/learning_speed_forest_sac.sh --mode smoke --gui
```

`smoke` 实际执行 31 个训练 Episodes，每 10 个 completed Episodes 换图，共三次切换。

```bash
./scripts/run_sh/learning_speed_forest_sac.sh --mode training --gui
```

`training` 目标为 10000 个 completed Episodes。默认自动生成 RUN_ID；可用 `--run-id UNIQUE_RUN_ID` 指定新的简单名称，已有输出目录会被拒绝。

独立 evaluation 使用明确的 checkpoint；下方占位符必须替换为实际文件和对应 Episode 编号：

```bash
./scripts/run_sh/learning_speed_forest_sac.sh --mode evaluation --gui \
  --seed 8 --checkpoint /absolute/path/to/checkpoint --checkpoint-episode N
```

evaluation 固定 100 Episodes，seed 可选 8 或 9。当前入口只接受上述四种 mode，没有 `demo` 模式。

停止：启动终端 `Ctrl+C`，等待收尾；人工中断不计作正常完成。

| 模式 | 结果目录 | 主要结果文件 |
|---|---|---|
| training | `runtime_artifacts/rl_training/<RUN_ID>/` | `sac_runtime_summary.json`、checkpoint、`logs/console.log`、`logs/ros/` |
| evaluation | `runtime_artifacts/rl_evaluation/<RUN_ID>/` | `sac_runtime_summary.json`、`logs/console.log`、`logs/ros/` |
| preflight | `runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/<RUN_ID>/` | `qualification_summary.json`、`logs/console.log` |
| smoke | 同上，独立 RUN_ID | `sac_runtime_summary.json`、`logs/console.log` |

## 5. Worksite SAC

用途：Worksite training-only Hector 环境中的 10000-Episode SAC 训练。保持上一节所述资格限制，获准后执行：

```bash
./scripts/run_sh/learning_speed_sac_training.sh --gui
```

默认自动生成 `sac_training_10000ep_YYYYMMDD_HHMMSS` 格式的 RUN_ID。若指定 `--run-id`，必须使用该格式和新目录。不加任何参数也会启动训练，只是 GUI 关闭；此入口不支持 `--mode`、evaluation 或 resume。

停止：启动终端 `Ctrl+C`，记录为主动中断，等待收尾。

结果：`runtime_artifacts/rl_training/<RUN_ID>/`，查看 `sac_runtime_summary.json`、checkpoint、`logs/training_console.log` 和 `logs/ros/`。退出码 0 不能单独证明训练完成，需核对 summary 与实际 Episode 结果。

各入口完整选项使用原脚本 `--help` 查看。全部脚本的分类见 [入口索引](../scripts/run_sh/README.md)；历史实验入口不作为本手册的日常操作方式。
