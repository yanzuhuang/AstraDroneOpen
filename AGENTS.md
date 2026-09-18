# 强化学习训练整理分支协作入口

本分支从 `三机巡检最小可运行整理` 的 `9b5b2049899e421b2c728e2c7547615ddc0501f7` 创建。
当前系统说明、人工命令、验证边界见根目录 `强化学习训练代码说明.md`；保留包清单见 `docs/rl_dependency_audit.json`。
原分支的三机巡检、PX4、FAST-LIO 和历史报告保留在 Git 历史，不代表本分支包含这些功能。

开始任务先检查当前 Git 分支、HEAD 和工作区。源码、launch、配置与本次运行工件优先于历史记忆。
保护用户未提交修改，不擅自恢复、清理、暂存或提交用户资产；不修改外部 PX4、Hector canonical checkout 或升级依赖。
当前入口为 `scripts/run_sh/reinforcement_learning/learning_speed_forest_sac.sh`，默认构建使用 `/tmp/astra_rl_minimal_overlay` 和 `/tmp/astra_hector_training_overlay`。
不得把已有工作空间 devel 中的残留包视为本分支依赖已经满足的证据。
`runtime_artifacts/sac_python_packages` 是当前 Python 运行依赖；其余历史产物只被 Git 排除，未经授权不删除。

保持 SAC、Observation C、奖励公式、动作范围、EGO 核心和控制器参数不变，除非用户明确要求修改。
真实失败必须保留，不调参、跳 seed 或放宽时序与安全门制造 PASS。
静态、单测、启动、Gazebo 闭环和正式训练是不同证据等级。最小启动通过不代表 10000-Episode training 已获资格验证，也不代表策略收敛。
没有明确运行授权，不自行启动 Gazebo 或训练。正式训练与 evaluation 必须使用新的唯一输出目录；不恢复旧 pilot。
