# 用户启动入口

目录按用途分为 `three_uav/`（4）、`reinforcement_learning/`（5）、`tools/`（2）。根目录仅保留本 README 和三个目录；旧平铺路径已移除，无兼容脚本、软链接或 alias。入口分类仍沿用下表。

## OFFICIAL：最终用户入口

从仓库根目录执行。日常操作以 [项目整理文档·运行说明](../../项目整理文档.md#runbook) 为准；它记录启动、停止和结果位置。带 `--control` 的巡检命令会启动自动飞行。

| 用途 | 正式入口 | 启动命令 |
|---|---|---|
| 三机多高度巡塔（两层） | [three_uav_multi_height_inspection.sh](three_uav/three_uav_multi_height_inspection.sh) | `./scripts/run_sh/three_uav/three_uav_multi_height_inspection.sh --multi-layer --control --gui --rviz --record light` |
| 三机同高度／普通三机巡塔 | [three_uav_inspection.sh](three_uav/three_uav_inspection.sh) | `./scripts/run_sh/three_uav/three_uav_inspection.sh --control --gui --rviz --record light` |
| Forest SAC | [learning_speed_forest_sac.sh](reinforcement_learning/learning_speed_forest_sac.sh) | `./scripts/run_sh/reinforcement_learning/learning_speed_forest_sac.sh --mode training --gui`，仅在相应训练资格确认并获准后执行 |

**OFFICIAL 表示保留的统一用户接口，不表示所有模式都已通过运行验收，也不解除 SAC 既有 NO-GO／qualification 限制。** 本轮只核对源码、帮助和文档，没有重新验收飞行或训练。Forest SAC 脚本没有 `--control` 开关；执行 training 命令就会启动训练。Worksite shell 已退役，底层 launch、节点、配置与历史数据保留；Forest 不替代其环境合同。

### 参数和默认行为

所有正式入口均支持 `--help`；该路径在环境准备和启动之前退出。无控制／dry-run 仍会启动仿真和节点，light/full 模式写入结果，不等于只读检查。

| 入口 | 默认行为 | 实际支持的主要参数 | 停止方式 |
|---|---|---|---|
| 三机多高度 | 单层 26/20/14 m；两层为 26→22、20→16、14→10 m；控制/GUI/RViz 关闭，录制 none（不录 bag） | 普通三机参数，加 `--single-layer`、`--multi-layer`；拒绝 `--learning-speed` | 启动终端 `Ctrl+C` |
| 普通三机 | 3/3/3 m、单层、worksite；控制/GUI/RViz/Learning Speed 关闭，D435 开启，lidar-downsample=1，录制 none（不录 bag），无时限 | `--control`、`--gui`、`--rviz`、`--world FILE`、`--record none\|light\|full`、`--duration SEC`、`--results-dir DIR`、`--learning-speed`、`--disable-d435`、`--lidar-downsample N` | 启动终端 `Ctrl+C`；没有 `--stop` |
| Forest SAC | 必须指定 mode，GUI 关闭，自动生成含时间戳的 RUN_ID | `--mode preflight\|training\|smoke\|evaluation`、`--run-id ID`、`--gui`；evaluation 使用 `--seed 8\|9`、`--checkpoint PATH`、`--checkpoint-episode N` | 启动终端 `Ctrl+C` |

参数保持各脚本原有合同，未增加通用别名：SAC 不支持 `--control`、`--rviz`、`--record`、`--stop`。Forest 当前没有 `demo` mode。`--print-plan`、通用 `--check-only` 等审计建议尚未实现，不应当作可用参数。

Forest `preflight` 会准备／可能构建 Hector overlay，启动 Gazebo 并执行三地图 reset 生命周期，不执行 Episode/Replay/learner；`smoke` 会实际执行 31 个训练 Episodes，每 10 个 completed Episodes 换图，共三次切换。二者均不属于只读检查。Forest SAC 入口可能调用 overlay 准备工具。

### 三机巡检高度 profile

统一入口 `three_uav/three_uav_inspection.sh --profile NAME`：

| NAME | UAV1 / UAV2 / UAV3 首层 | 模式 |
|---|---|---|
| `same_3m`（默认） | 3 / 3 / 3 m | 同层单圈 |
| `same_30m` | 30 / 30 / 30 m | 同层单圈 |
| `multi_height_low_3m` | 15 / 9 / 3 m | 异层各单圈 |

仅展开参数和检查高度合同，不启动任何节点：

```bash
./scripts/run_sh/three_uav/three_uav_inspection.sh --profile same_30m --check-height-profile
```

人工飞行命令（新高度尚待飞行验证）：

```bash
./scripts/run_sh/three_uav/three_uav_inspection.sh --profile same_3m --control --gui --rviz --record light
./scripts/run_sh/three_uav/three_uav_inspection.sh --profile same_30m --control --gui --rviz --record light
./scripts/run_sh/three_uav/three_uav_inspection.sh --profile multi_height_low_3m --control --gui --rviz --record light
```

入口每次启动前校验最终展开参数，light/full 会保存 `height_contract.txt`。不再读取 `ASTRA_THREE_UAV_LAUNCH` 和 `ASTRA_MULTI_LAYER_ENABLED`；高度切换只用显式 profile，不需要编辑源码。三个新 profile 均拒绝 `--multi-layer`，防止最低 3 m 再下降到 -1 m。原异层脚本仍默认 26/20/14 m，保留 `--single-layer` / `--multi-layer`，内部显式选择 `multi_height_legacy`。详细参数和验证边界见 [高度参数化改造报告](../../三机巡检高度参数化改造报告.md)。

### 结果和收尾

下列路径均相对于仓库根目录；以脚本当次打印的路径为准。

| 入口／模式 | 默认结果位置 |
|---|---|
| 普通三机、多高度三机（显式 light/full） | `runtime_artifacts/three_uav_inspection_control_<时间戳>/`；无控制为 `three_uav_inspection_dry_run_<时间戳>/` |
| Forest training | `runtime_artifacts/rl_training/<RUN_ID>/` |
| Forest evaluation | `runtime_artifacts/rl_evaluation/<RUN_ID>/` |
| Forest preflight／smoke | `runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/<RUN_ID>/` |

不写 `--record` 或显式 `--record none` 均不录 bag；显式 `--record light` / `--record full` 使用原有录制逻辑和 topic 集合。

三机 `light` 保存状态、轨迹及任务证据，省略完整点云和 Gazebo model states；需要这两类证据时使用 `full`。`none` 不保留脚本管理的任务证据，不能用于依赖这些证据的验收，且不能与 `--results-dir` 同用。三机自定义目录须使用本仓库内匹配 `runtime_artifacts/three_uav_inspection_*` 的绝对路径；脚本没有训练入口那样的独占目录拒绝复用机制，操作者应选新目录。

停止操作结束进程，不是返航或降落指令。正常飞行先等任务完成、落地并解除武装，再收尾；提前中断保留为中断。三机收尾会结束录包和启动进程，并在存在 swarm.csv 时尝试绘图。tmux 的 `Ctrl+b d` 只是脱离界面。训练是否完成以 `sac_runtime_summary.json` 和实际 Episode 结果为准。

## 分类规则

本目录三个子目录内共 11 个 `.sh` 各归入以下五类之一。第五阶段已删除六个退役实验编排外壳；第七阶段新增只读 MAVROS 诊断入口；第八阶段删除已迁移的旧 `echo.sh`，此前已正式退役七个单机 shell；底层 ROS 组件保持；本轮另退役四个指定 shell。依据：[项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。

| 类别 | 定义 | 脚本数 |
|---|---|---:|
| OFFICIAL | 上述最终用户任务接口；SAC 仍受既有训练资格限制 | 3 |
| RESEARCH | 独立研究模式、共享实验执行器，不作为日常最终任务推荐 | 1 |
| DIAGNOSTIC | 初始化、附加采集和故障诊断；区分只读与主动注入 | 4 |
| LEGACY | 保留的旧演示、诊断与单次实验执行器，退出日常推荐 | 2 |
| INTERNAL | 被现有入口调用的辅助实现／环境准备工具 | 1 |

## RESEARCH

| 入口 | 用途／保留原因 |
|---|---|
| [learning_speed_manual_run.sh](reinforcement_learning/learning_speed_manual_run.sh) | A/B 单次标定及资格实验共享执行器；历史编排外壳已删除，执行器及其依赖保留 |

## DIAGNOSTIC

三架无人机第一视角：仿真启动后，在另一个桌面终端运行
`./scripts/run_sh/three_uav/three_uav_camera_view.sh --gazebo` 打开三个 Gazebo 原生图像窗口；
不带 `--gazebo` 则打开三个 ROS 彩色图像窗口。`Ctrl+C` 只关闭本次查看器。
详细说明与验证边界见 [项目整理文档·三机第一视角](../../项目整理文档.md#camera-view)。

| 入口 | 用途／行为 |
|---|---|
| [mavros_monitor.sh](tools/mavros_monitor.sh) | 四路 MAVROS 只读实时监视，默认根 namespace，可选 `--namespace /uav1`、`/uav2`、`/uav3`；不启动系统、不发布、不创建结果目录 |
| [three_uav_camera_view.sh](three_uav/three_uav_camera_view.sh) | 附加三路 D435 彩色第一视角；仅订阅图像，不启动仿真或飞行 |
| [three_uav_outdoor_village.sh](three_uav/three_uav_outdoor_village.sh) | 三机 outdoor 初始化；启动仿真，明确拒绝 `--control` |
| [learning_speed_calibration.sh](reinforcement_learning/learning_speed_calibration.sh) | 对已运行系统附加 Observation C／标定采集 |

MAVROS 监视：在已 source ROS 环境、已有 ROS master 的独立终端（tmux 外）执行 `./scripts/run_sh/tools/mavros_monitor.sh`。用 `--help` 查看四路 topic；`Ctrl+b d` 退出并清理本次监视，pane 内 `Ctrl+C` 仅停止该路。独立 socket 和唯一 session 隔离现有 tmux；没有 `--stop` 或后台保持模式。

## LEGACY

以下两个 LEGACY 文件继续保留，不删除、不重构。旧 `echo.sh` 已在第八阶段单独删除，四路监视使用上方 DIAGNOSTIC 入口 `mavros_monitor.sh`。六个已退役编排外壳已在第五阶段删除，其整理结论与协议差异汇总于 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)，原文见固定 Git 版本。不要为了查看用法而试跑这些旧脚本；部分入口会直接开启控制或重建 tmux 会话。

| 入口 | 保留内容 |
|---|---|
| [record.sh](tools/record.sh) | 旧 topic 录包，历史输出为 `~/bag` |
| [high_speed_exploration.sh](reinforcement_learning/high_speed_exploration.sh) | 旧高速档位与 run-ID 配对协议 |

## INTERNAL

| 入口 | 被调用关系／保留原因 |
|---|---|
| [prepare_hector_training_overlay.sh](reinforcement_learning/prepare_hector_training_overlay.sh) | Forest SAC 入口依赖的 Hector overlay 准备工具，可能复制、移动和构建；不并入只读检查 |

目录外的双机入口、场景生成器、YOLO、安装器、分析器及包内节点继续留在原位置；本轮没有新建包装器或将其重定向。

## 保留边界（不自动退役）

| 候选 | 进入归档评审前需证明的替代关系 |
|---|---|
| `record.sh` | 新录制 profile 覆盖原 topic、压缩／命名、停止及输出目录迁移 |


第三阶段未证明OFFICIAL完全替代任何旧入口。第四阶段将六份历史编排退役为Git与文档协议，按单独的退役标准评审删除资格；这不表示OFFICIAL新增了相同实验能力。第五阶段已按批准集合删除六个编排外壳，见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。没有新增其他删除候选许可。其余共享执行器与底层 launch 保持受保护；本轮不抽取公共 shell library、不重构进程管理。

## 本轮验证边界（2026-09-11）

基于 `43dd4bf` 工作区完成 27 个脚本的 Bash 语法检查、6 个 OFFICIAL 脚本的 `--help` 执行检查，以及分类完整性、文档链接和正式命令一致性检查。6 个脚本移除帮助正文后与 HEAD 逐字一致，其余 21 个脚本完整内容与 HEAD 一致。未启动 ROS/Gazebo、飞行、preflight、smoke、training 或 evaluation；本轮不产生新的运行 PASS。

## 第六阶段：剩余旧入口定位

详见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。第六阶段当时 echo.sh 为 MIGRATE_THEN_DELETE（仅设计四路监视的最小迁移，当时保留）；record.sh 与 pc_example.sh 为 KEEP_AS_IS。该阶段三者均未改变行为，NEXT_DELETE_SET 当时为空。当前任务使用顶部三机正式命令；已退役 shell 的历史命令不再运行。

第六阶段当时 OFFICIAL 六项沿用“受维护用户接口”口径；按第六阶段“最终成果正常使用 / 研究训练”功能层级，两个 SAC 入口属于 RESEARCH，报告给出22项功能复核表。当前导航已按单机 shell 退役更新；分类不构成训练放行或额外删除许可。

## 第七阶段：四路 MAVROS 诊断迁移

`mavros_monitor.sh` 归类为 DIAGNOSTIC，四路 topic 与旧入口逐项一致。静态及隔离验证通过，真实 ROS 数据显示待人工验证。第七阶段当时 `echo.sh` 为 READY_TO_DELETE，`NEXT_DELETE_SET=["scripts/run_sh/echo.sh"]` 是当时的后续候选，第七阶段未删除。第六阶段的空集合是历史结论。详见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。其他旧入口继续保留。

## 第八阶段：旧 echo 单文件删除

已按授权仅物理删除 `scripts/run_sh/echo.sh`；四路监视继续使用 DIAGNOSTIC 的 `mavros_monitor.sh`。第八阶段结束时为 22 个 shell：OFFICIAL 6、RESEARCH 3、DIAGNOSTIC 7、LEGACY 4、INTERNAL 2。历史整理报告现已合并后删除，结论见模块一；当前 `NEXT_DELETE_SET=[]`，不继续自动删除其他文件。低风险回归与保护范围见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。

## 单机 shell 正式退役

七个单机独立演示／实验 shell 已按审计集合删除，旧 pc_example alias 已移除。该阶段结束时为 15 个 shell；当前为 11 个，分类数量见上表；`NEXT_DELETE_SET=[]`。三机仍直接使用 `sector_inspection_mission_node`、`sector_inspection.yaml`、`occupancy_stamp_adapter_node` 和 `ego_gazebo_bridge`，这些组件、所有 launch／节点／测试、RL 入口和运行工件完整保留。详见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。

## 最终三目录布局

物理路径与调用修复、验证边界见 [项目整理文档·模块一](../../项目整理文档.md#模块一启动脚本与-run_sh-整理)。正式三机入口为 `./scripts/run_sh/three_uav/three_uav_multi_height_inspection.sh --multi-layer --control --gui --rviz --record light`。
