# 用户启动入口

## OFFICIAL：最终用户入口

从仓库根目录执行。日常操作以 [FINAL_RUNBOOK](../../docs/FINAL_RUNBOOK.md) 为准；它记录启动、停止和结果位置。带 `--control` 的巡检命令会启动自动飞行。

| 用途 | 正式入口 | 启动命令 |
|---|---|---|
| 三机多高度巡塔（两层） | [three_uav_multi_height_inspection.sh](three_uav_multi_height_inspection.sh) | `./scripts/run_sh/three_uav_multi_height_inspection.sh --multi-layer --control --gui --rviz --record light` |
| 三机同高度／普通三机巡塔 | [three_uav_inspection.sh](three_uav_inspection.sh) | `./scripts/run_sh/three_uav_inspection.sh --control --gui --rviz --record light` |
| 单机 EGO 八扇区巡检 | [sector_inspection.sh](sector_inspection.sh) | `./scripts/run_sh/sector_inspection.sh --control --sector-limit 8 --gui --rviz --attach` |
| 单机 EGO waypoint 巡检 | [ego_waypoint_inspection.sh](ego_waypoint_inspection.sh) | `./scripts/run_sh/ego_waypoint_inspection.sh --control --scenario tower --gui --rviz --attach` |
| Forest SAC | [learning_speed_forest_sac.sh](learning_speed_forest_sac.sh) | `./scripts/run_sh/learning_speed_forest_sac.sh --mode training --gui`，仅在相应训练资格确认并获准后执行 |
| Worksite SAC | [learning_speed_sac_training.sh](learning_speed_sac_training.sh) | `./scripts/run_sh/learning_speed_sac_training.sh --gui`，仅在相应训练资格确认并获准后执行 |

**OFFICIAL 表示保留的统一用户接口，不表示所有模式都已通过运行验收，也不解除 SAC 既有 NO-GO／qualification 限制。** 本轮只核对源码、帮助和文档，没有重新验收飞行或训练。两个 SAC 脚本没有 `--control` 开关；执行 training 命令就会启动训练。Forest 与 Worksite 保留各自环境和 Episode 合同，不能只换 world 来互相替代；单机八扇区与 waypoint 也是不同任务模式。

### 参数和默认行为

所有正式入口均支持 `--help`；该路径在环境准备和启动之前退出。无控制／dry-run 仍会启动仿真和节点、写入结果，不等于只读检查。

| 入口 | 默认行为 | 实际支持的主要参数 | 停止方式 |
|---|---|---|---|
| 三机多高度 | 单层 26/20/14 m；两层为 26→22、20→16、14→10 m；控制/GUI/RViz 关闭，录制 light | 普通三机参数，加 `--single-layer`、`--multi-layer`；拒绝 `--learning-speed` | 启动终端 `Ctrl+C` |
| 普通三机 | 3/3/3 m、单层、worksite；控制/GUI/RViz/Learning Speed 关闭，D435 开启，lidar-downsample=1，录制 light，无时限 | `--control`、`--gui`、`--rviz`、`--world FILE`、`--record none\|light\|full`、`--duration SEC`、`--results-dir DIR`、`--learning-speed`、`--disable-d435`、`--lidar-downsample N` | 启动终端 `Ctrl+C`；没有 `--stop` |
| 单机八扇区 | 无控制，8 扇区、1 cycle、34→30 m 两层；GUI/RViz/attach 关闭；默认不录 bag | `--control`、`--gui`、`--rviz`、`--attach`、`--world FILE`、`--report FILE`、`--bag FILE`、`--sector-limit`、`--cycles`；完整高度参数见 `--help` | 新终端执行原脚本 `--stop` |
| 单机 waypoint | 无控制 dry-run，waypoints=8，GUI/RViz/attach 关闭；mission.csv，无 bag 开关 | `--scenario dry-run\|single\|dual\|tower`、`--control`、`--waypoints N`、`--gui`、`--rviz`、`--world FILE`、`--report FILE`、`--attach` | 新终端执行原脚本 `--stop` |
| Forest SAC | 必须指定 mode，GUI 关闭，自动生成含时间戳的 RUN_ID | `--mode preflight\|training\|smoke\|evaluation`、`--run-id ID`、`--gui`；evaluation 使用 `--seed 8\|9`、`--checkpoint PATH`、`--checkpoint-episode N` | 启动终端 `Ctrl+C` |
| Worksite SAC | 不带参数也启动 10000-Episode training；GUI 关闭，自动 RUN_ID | `--gui`、`--run-id sac_training_10000ep_YYYYMMDD_HHMMSS` | 启动终端 `Ctrl+C` |

参数保持各脚本原有合同，未增加通用别名：单机不支持三机的 `--record`；SAC 不支持 `--control`、`--rviz`、`--record`、`--stop`。Worksite 没有 `--mode`、evaluation 或 resume 用户选项；Forest 当前没有 `demo` mode。`--print-plan`、通用 `--check-only` 等审计建议尚未实现，不应当作可用参数。

Forest `preflight` 会准备／可能构建 Hector overlay，启动 Gazebo 并执行三地图 reset 生命周期，不执行 Episode/Replay/learner；`smoke` 会实际执行 31 个训练 Episodes，每 10 个 completed Episodes 换图，共三次切换。二者均不属于只读检查。两个 SAC 入口都可能调用 overlay 准备工具。

### 结果和收尾

下列路径均相对于仓库根目录；以脚本当次打印的路径为准。

| 入口／模式 | 默认结果位置 |
|---|---|
| 普通三机、多高度三机 | `runtime_artifacts/three_uav_inspection_control_<时间戳>/`；无控制为 `three_uav_inspection_dry_run_<时间戳>/` |
| 单机八扇区 | `runtime_artifacts/sector_inspection_control_<时间戳>/control.csv`；无控制为 `sector_inspection_dry_run_<时间戳>/dry_run.csv` |
| 单机 waypoint | `runtime_artifacts/ego_waypoint_inspection_<scenario>_<时间戳>/mission.csv` |
| Forest training、Worksite training | `runtime_artifacts/rl_training/<RUN_ID>/` |
| Forest evaluation | `runtime_artifacts/rl_evaluation/<RUN_ID>/` |
| Forest preflight／smoke | `runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/<RUN_ID>/` |

三机 `light` 保存状态、轨迹及任务证据，省略完整点云和 Gazebo model states；需要这两类证据时使用 `full`。`none` 不保留脚本管理的任务证据，不能用于依赖这些证据的验收，且不能与 `--results-dir` 同用。三机自定义目录须使用本仓库内匹配 `runtime_artifacts/three_uav_inspection_*` 的绝对路径；脚本没有训练入口那样的独占目录拒绝复用机制，操作者应选新目录。单机 `--report`／八扇区 `--bag` 必须位于本仓库 `runtime_artifacts/` 下；八扇区仅在显式 `--bag FILE` 时录包。

停止操作结束进程，不是返航或降落指令。正常飞行先等任务完成、落地并解除武装，再收尾；提前中断保留为中断。三机收尾会结束录包和启动进程，并在存在 swarm.csv 时尝试绘图；八扇区 `--stop` 会等待录包完成，再结束会话并尝试绘图。tmux 的 `Ctrl+b d` 只是脱离界面。训练是否完成以 `sac_runtime_summary.json` 和实际 Episode 结果为准；Worksite 的进程退出码不能单独证明训练成功。

## 分类规则

本目录当前 27 个 `.sh` 各归入以下五类之一。分类仅改变用户导航，原文件路径、调用关系和执行权限全部保留。依据：[项目启动入口收尾审计报告](../../项目启动入口收尾审计报告.md)。

| 类别 | 定义 | 脚本数 |
|---|---|---:|
| OFFICIAL | 上述最终用户任务接口；SAC 仍受既有训练资格限制 | 6 |
| RESEARCH | 独立研究模式、共享实验执行器，不作为日常最终任务推荐 | 3 |
| DIAGNOSTIC | 初始化、附加采集和故障诊断；区分只读与主动注入 | 5 |
| LEGACY | 保留原位置的旧演示与历史实验协议，退出日常推荐 | 11 |
| INTERNAL | 被现有入口调用的辅助实现／环境准备工具 | 2 |

## RESEARCH

| 入口 | 用途／保留原因 |
|---|---|
| [fixed_orbit_inspection.sh](fixed_orbit_inspection.sh) | 固定圆周路线预览及独立任务模式；默认预览，不能视为已被八扇区替代 |
| [ego_planner_stack.sh](ego_planner_stack.sh) | PX4/FAST-LIO/EGO/bridge 集成与规划开发，默认无控制 |
| [learning_speed_manual_run.sh](learning_speed_manual_run.sh) | A/B 单次标定及资格实验共享执行器，仍被历史 batch/qualification 调用 |

## DIAGNOSTIC

| 入口 | 用途／行为 |
|---|---|
| [three_uav_outdoor_village.sh](three_uav_outdoor_village.sh) | 三机 outdoor 初始化；启动仿真，明确拒绝 `--control` |
| [learning_speed_calibration.sh](learning_speed_calibration.sh) | 对已运行系统附加 Observation C／标定采集 |
| [three_uav_self_filter_record_bag.sh](three_uav_self_filter_record_bag.sh) | 三机点云过滤专项证据，保留独立 topic 集合 |
| [uav2_occupancy_record_bag.sh](uav2_occupancy_record_bag.sh) | UAV2 占据图、定位和模型状态专项证据 |
| [sector_inspection_failure_injection.sh](sector_inspection_failure_injection.sh) | **主动向 planner/status 发布故障状态**，仅供受控测试，不是只读监视 |

## LEGACY

本轮只标记，不删除、不移动、不改变帮助或执行行为。不要为了查看用法而试跑这些旧脚本；部分入口会直接开启控制或重建 tmux 会话。

| 入口 | 保留内容 |
|---|---|
| [pc_example.sh](pc_example.sh) | 旧 tmux 演示、base-only、旧 Offboard、FAST-LIO、QGC 组合；默认含旧自动控制 |
| [echo.sh](echo.sh) | 旧单机四路 MAVROS topic 监视 |
| [record.sh](record.sh) | 旧 topic 录包，历史输出为 `~/bag` |
| [fixed_speed_baseline.sh](fixed_speed_baseline.sh) | 旧固定速度基线协议与证据采集 |
| [fixed_speed_baseline_batch.sh](fixed_speed_baseline_batch.sh) | 旧固定 ID 的 12 次基线编排，内部传入控制参数 |
| [high_speed_exploration.sh](high_speed_exploration.sh) | 旧高速档位与 run-ID 配对协议 |
| [high_speed_exploration_batch.sh](high_speed_exploration_batch.sh) | 旧高速顺序编排 |
| [high_speed_boundary_refinement.sh](high_speed_boundary_refinement.sh) | 旧高速边界补充实验 |
| [learning_speed_manual_batch.sh](learning_speed_manual_batch.sh) | 历史 A/B 六档标定矩阵及基础设施重试协议 |
| [learning_speed_postfix_batch.sh](learning_speed_postfix_batch.sh) | 历史 post-fix A/B 七档矩阵及固定目录／命名 |
| [learning_speed_progressive_qualification.sh](learning_speed_progressive_qualification.sh) | 历史 Environment B 逐档高速资格验证协议，保留真实失败即停语义 |

## INTERNAL

| 入口 | 被调用关系／保留原因 |
|---|---|
| [prepare_hector_training_overlay.sh](prepare_hector_training_overlay.sh) | 两个 SAC 入口依赖的 Hector overlay 准备工具，可能复制、移动和构建；不并入只读检查 |
| [sector_inspection_record_bag.sh](sector_inspection_record_bag.sh) | 八扇区主入口通过内部组件调用的专项录包实现；用户用主入口 `--bag FILE` |

各主脚本的 `--component` 分支同样属于内部实现，不是新的用户任务入口。目录外的双机入口、场景生成器、YOLO、安装器、分析器及包内节点继续留在原位置；本轮没有新建包装器或将其重定向。

## 下一阶段：替代验证候选

| 候选 | 进入归档评审前需证明的替代关系 |
|---|---|
| `echo.sh` | 新诊断入口覆盖原四路 topic、命名空间及停止行为 |
| `record.sh` | 新录制 profile 覆盖原 topic、压缩／命名、停止及输出目录迁移 |
| `pc_example.sh` | base-only、旧 Offboard 演示、FAST-LIO、QGC 分别有明确入口，教程引用完成迁移 |
| 旧 fixed-speed/high-speed/manual/postfix batch 和 progressive qualification | 逐份保留速度与代际、固定 ID、顺序、重试边界、失败即停、输出 schema 和后处理依赖；不能只比较脚本名称 |

当前没有已证明可删除的对象。下一阶段应先完成静态依赖、参数默认值、无控制行为、原场景运行证据和退出／结果等价验证，再单独评审归档。共享执行器、专项 recorder 和底层 launch 保持受保护；本轮不抽取公共 shell library、不重构进程管理。

## 本轮验证边界（2026-09-11）

基于 `43dd4bf` 工作区完成 27 个脚本的 Bash 语法检查、6 个 OFFICIAL 脚本的 `--help` 执行检查，以及分类完整性、文档链接和正式命令一致性检查。6 个脚本移除帮助正文后与 HEAD 逐字一致，其余 21 个脚本完整内容与 HEAD 一致。未启动 ROS/Gazebo、飞行、preflight、smoke、training 或 evaluation；本轮不产生新的运行 PASS。
