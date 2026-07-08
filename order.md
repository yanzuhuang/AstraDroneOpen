🚀 阶段一：开机进入开发状态
每次启动进入 Ubuntu 系统后，按照以下三步唤醒你的开发环境：

唤醒地面站： 打开 QGroundControl，检查左上角设置中是否勾选了虚拟游戏手柄 (Virtual Joystick)，确保底层飞控解锁安全机制通过。

进入主控台： 打开 VSCode，直接加载 ~/AstraDroneOpen 文件夹。

呼出终端： 使用快捷键 Ctrl + `（反引号）在底部呼出终端面板。建议点击 + 号开启两个终端：一个专门用来跑仿真脚本，一个用来编译 C++ 代码。

💻 阶段二：修改业务代码与初次启动
当你需要调整任务逻辑（例如将起飞高度参数修改为 2.0，编写位置指令发布器）时，修改和生效的流程如下：

定位核心代码：
在 VSCode 左侧目录树进入 ~/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/。在此处的源文件中编写你的控制节点代码。

编译代码生效（仅限 C++）：
如果你修改的是 Python 脚本或 .launch 配置文件，直接保存即可。如果你修改的是 C++ 源码，请在备用终端中执行编译：

Bash
cd ~/AstraDroneOpen/AstraDrone_ros1_ws/
catkin_make
一键拉起世界与飞机：
在主终端中，运行开源项目的集成启动脚本，这会同步拉起 ROS、Gazebo 仿真环境以及飞控：

Bash
cd ~/AstraDroneOpen/
./scripts/run_sh/pc_example.sh
(此时你应该能看到 Gazebo 窗口弹出，无人机按照你代码中的逻辑起飞执行任务。)

🔄 阶段三：调试修改与重新仿真循环
在算法开发过程中，你一定会频繁修改参数并重新验证效果。不需要关掉终端，只需在运行仿真脚本的主终端中执行以下“重启三连”：

优雅退出当前任务：
在终端中按下 Ctrl + C，等待几秒钟，让所有节点有序退出，直到绿色输入提示符重新出现。

彻底清理幽灵进程：
为了防止端口被占用或 Gazebo 渲染卡死，强制清理后台残余：

Bash
killall -9 roscore rosmaster gzserver gzclient px4
按需重新编译并再次起飞：

如果是 C++，切回备用终端再跑一次 catkin_make。

回到主终端，按键盘的 ↑ (上方向键) 调出历史命令 killall -9 ...，再按一次 ↑ 调出 ./scripts/run_sh/pc_example.sh，按下回车直接重飞。