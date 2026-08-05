# 临时缓存与运行产物清理审计（待讨论）

日期：2026-08-05
分支：`ego-swarm`
审计时 HEAD：`e95e044ef74226d785eac7c500cf987fec498bb8`

## 1. 目的与范围

本文件仅记录“Git 源代码管理区被仿真运行产物干扰”的只读审计结论，供后续讨论。它**不是删除清单**，也不授权删除、`git rm --cached`、历史重写、提交或推送。

审计范围为当前工作树中已被 Git 跟踪的典型运行产物：ROS 日志、rosbag、构建目录、缓存目录和临时 world。没有修改源码、配置、忽略规则或数据文件。

## 2. 当前发现

1. 当前 HEAD 中共跟踪 **282 个**疑似运行产物，其中 **267 个为 `.log`**；工作树中未发现被 Git 跟踪的 rosbag、`build/`、`devel/`、`artifacts/` 或 `temp_worlds/`。
2. 这批文件在当前工作树约占 **155 MiB**。它们绝大多数是 Stage 5 三机验证期间生成的 ROS 原始日志，而不是绕塔源码、launch、参数、世界文件或 RViz 配置。
3. 其中 **263 个文件**首次随提交 `c8cfadf`（2026-08-03，`清除虚拟障碍物，三机绕塔正常`）被加入版本管理。这解释了之后每次运行为什么会在“源代码管理”中出现大量运行相关变化。
4. [scripts/run_sh/three_uav_inspection.sh](scripts/run_sh/three_uav_inspection.sh) 默认将每次运行结果写入 `test_evidence/stage5_<mode>_<时间戳>/`，并设置 `ROS_LOG_DIR=<结果目录>/ros_logs`。因此新运行会产生新的未跟踪目录；若复用已跟踪结果目录，还会修改已跟踪的 `roslaunch.log`、`master.log`、`rosout.log` 等文件。

## 3. 当前工作区中可见的运行产物

审计时工作区已有以下用户资产，均未改动：

- 已修改：`AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt`；
- 已修改：`test_evidence/stage5_false_obstacle_fix_20260803/triple_validation/` 下的 `master.log`、`rosout.log`、`roslaunch.log`；
- 未跟踪：`temp_worlds/`；
- 未跟踪：6 个 `test_evidence/stage5_control_20260803_*/` 运行目录。

上述文件不是本次审计创建的。特别是 `mat_pre.txt` 目前已有用户修改，不应在未经逐内容确认的情况下还原、删除或取消跟踪。

## 4. 已跟踪运行日志的主要分布

| 证据组 | 文件数 | 当前体积 | 初步判定 |
|---|---:|---:|---|
| `stage5_false_obstacle_fix_20260803` | 68 | 33.26 MiB | 专项完整证据，暂不批量处理 |
| `stage5_octagon_optimization_validation_20260803_024800` | 48 | 34.24 MiB | 含根因分析输入，需保留最小输入 |
| `stage5_octagon_optimization_validation_20260803_040000` | 48 | 30.61 MiB | 含成功样本分析输入，需保留最小输入 |
| `stage5_first_point_tier_fix_validation_20260803_012118` | 48 | 28.64 MiB | 验证记录，需单独判断 |
| `stage5_control_20260803_143131` | 48 | 15.48 MiB | 历史控制记录，需单独判断 |
| 其余较早 Stage 5 / 三机记录 | 18 | 约 9.66 MiB | 不构成当前反复变更主因 |

## 5. 不能直接全部取消跟踪的原因

这些日志不参与三机仿真的运行时启动；移出 Git 跟踪本身不会改变 Gazebo、PX4、MAVROS、FAST-LIO、EGO 或三机任务逻辑。

但少数离线分析脚本直接读取它们：

- `scripts/tool/analyze_three_uav_orbit.py` 读取目标目录的 `roslaunch.log`；
- `test_evidence/stage5_uav2_trajectory_root_cause_audit_20260803/analyze_audit.py` 从 `ros_logs/*/rosout.log` 读取失败与成功样本；
- `stage5_cleanup_and_rviz_display_report.md` 将 `stage5_false_obstacle_fix_20260803` 表述为完整保留的专项证据组。

因此“所有 `.log` 一律删除/取消跟踪”会降低离线根因分析与完整证据复核的可重现性，即使不会破坏仿真运行。

## 6. 建议的后续处理方案（未执行）

### A. 保留在 Git 的正式证据

建议保留：`summary.json`、`run_metadata.txt`、三机 CSV、`swarm.csv`、根因 JSON、最终图表，以及上述分析脚本实际需要的 `roslaunch.log` / `rosout.log`。是否保留完整的 `stage5_false_obstacle_fix_20260803` 原始日志，需由项目负责人确认其作为完整证据的要求。

### B. 候选：从 Git 取消跟踪、但保留本地文件

候选范围是每个 `test_evidence/**/ros_logs/` 内不被上述分析脚本读取的节点日志、stdout 日志、`master.log` 等原始 ROS 运行日志；以及确认不再有分析用途的 `rosbag.log`。

此操作应使用逐路径清单执行 `git rm --cached -- <路径>`，这样文件仍留在本机，但后续对它们的运行改写不会显示为 Git 修改。执行前必须确认每个保留目录及分析输入，不能使用广泛通配符或整仓库清理。

### C. 防止未来运行继续污染工作区

建议在确认“哪些结果目录必须被版本化”后再修改 `.gitignore`。可能的候选规则包括：

```gitignore
temp_worlds/
test_evidence/stage5_control_*/
```

更稳妥的工程方案是让脚本默认写入单独的、被忽略的 `runtime_artifacts/` 目录；只有人工确认有价值的一次运行，才将其轻量摘要和必要分析输入归档到 `test_evidence/`。这会改变现有脚本“默认永久证据目录”的行为，需另行评审后实施。

## 7. 明确不建议做的操作

- 不要使用 VS Code 的“全部暂存”或 `git add .` 来提交三机源码；应逐文件暂存源码、配置和文档。
- 不要对 `FAST_LIO/Log/` 做整目录清理；其中混有说明、绘图/分析脚本和文本记录。
- 不要对当前已有修改执行全局“丢弃更改”、`git restore .`、`git clean`、`git reset --hard`。
- 不要为解决日常工作区噪声而重写 Git 历史。若将来要从全部历史移除大文件，必须先确认远端共享情况、备份和协作者迁移方案。

## 8. 建议的讨论决策

在执行任何清理前，需要负责人确认以下三点：

1. `stage5_false_obstacle_fix_20260803` 是否必须作为“完整原始日志证据”留在 Git；
2. 两个八边形样本的 `roslaunch.log` 与 `rosout.log` 是否改为外部归档，还是继续随仓库保存以支持脚本复跑；
3. 后续默认运行结果是写入 Git 忽略的 `runtime_artifacts/`，还是继续写入 `test_evidence/` 并只在每次运行后手动筛选。

在这三点明确前，建议仅使用“逐文件暂存”的提交方式，不做删除或取消跟踪。
