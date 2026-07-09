标准开发工作流
在目前的开发阶段，你每次修改代码的标准闭环操作应该是这样的：

清场： 终端输入 

tmux kill-server 

彻底关闭上一次的仿真。

写代码： 在 VSCode 里修改 .cpp 源码，并按 Ctrl + S 确保保存。

编译代码： 在工作空间根目录下 

cd ~/AstraDroneOpen/AstraDrone_ros1_ws 

运行：

catkin_make

刷新并运行：

./scripts/run_sh/pc_example.sh


🚀 极速迭代工作流（标准推荐）
以后每次你想换一种飞行模式测试，只需要在 VSCode 和 Gazebo 之间循环以下四个步骤：

第一步：强制中断当前飞行 (VSCode)
在你运行 roslaunch 的那个 VSCode 终端里，直接按下 Ctrl + C。

现象： 你的控制节点被杀死。飞控收不到位置指令，会触发失控保护，无人机会在 Gazebo 里就地缓慢降落。

第二步：等待锁定 (Gazebo)
切记：这一步是关键！ 不要急着马上跑新代码。看一眼 QGroundControl 地面站或者 Gazebo 画面，等无人机完全降落到地面，并且螺旋桨完全停转（系统进入 Disarmed 锁定状态）。通常这只需要几秒钟。

第三步：切代码与编译 (VSCode)
无人机落地后，你就可以放心地去改 C++ 代码，或者用 Git Graph 切换分支。
修改完后，在 VSCode 终端里直接编译：

Bash
catkin_make
第四步：一键“热启动” (VSCode)
编译完成后，不用管飞机现在停在 Gazebo 的哪里，直接重新运行：

Bash
source devel/setup.bash
roslaunch offboard autoarming_control.launch
现象： 飞机会原地解锁，起飞，然后自动斜线飞向 (0, 0) 原点的上空，到达指定高度后，完美开始执行你刚写的新轨迹！

强迫症专属：如何把飞机物理“瞬移”回原点？
如果你觉得重新启动时，飞机从远处飞回原点的过程太慢，或者你就是有强迫症，必须让它在每次起飞前都物理上端端正正地停在 Gazebo 坐标轴的正中央 (0,0,0)，你可以用 Gazebo 提供的“重置”功能：

先等飞机落地并停转（和上面的第二步一样，必须等螺旋桨停下，否则飞控的卡尔曼滤波器 EKF 会因为传感器数据突变而彻底崩溃报错）。

点击 Gazebo 仿真窗口顶部的菜单栏：Edit -> Reset Model Poses（或者使用快捷键 Ctrl + Shift + R）。

飞机就会瞬间“瞬移”回世界的最中心，姿态也会回正。

然后再去 VSCode 里 catkin_make 和 roslaunch。

⚠️ 避坑警告：在 Gazebo 的 Edit 菜单里，绝对不要点 Reset World。它会把仿真的时间也清零重置，这会导致 ROS 和飞控的时间戳彻底错乱，接下来的所有起飞指令都会失效，你只能痛苦地 tmux kill-server 重启所有东西了。只点 Reset Model Poses！