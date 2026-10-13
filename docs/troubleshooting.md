# 常见问题

下列 ROS 命令在 Ubuntu 20.04 / ROS Noetic 环境中运行。先按 [README](../README.md) 构建；每个新终端都要加载 ROS 和工作空间环境。

## 找不到 package 或节点

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack find turtlesim_circle
```

预期找到 `~/catkin_ws/src/turtlesim_circle`。若找不到，检查 `package.xml` 是否直接位于该文件夹，重新运行 `catkin_make`，再加载 `devel/setup.bash`。ZIP 解压可能多出一层目录，也可能丢失脚本的可执行权限：

```bash
chmod +x ~/catkin_ws/src/turtlesim_circle/scripts/*.py
```

## 小乌龟没有运动或运行中停止

控制器需要收到有效、未超时的 `/turtle1/pose`，并确认速度订阅者和可选录包器已就绪。启动和恢复阶段还会等待 `startup_delay`。先检查：

```bash
rosnode list
rosparam get /circle_controller
rostopic info /turtle1/cmd_vel
rostopic info /turtle1/pose
```

在独立终端观察位姿发布频率，按 Ctrl+C 结束观察：

```bash
rostopic hz /turtle1/pose
```

重点区分以下情况：

- **已到 duration**：这是预定停车，节点继续发布零速度。重新运行实验，或启动时使用 `duration:=0`。
- **Invalid circle parameter**：检查 `period > 0`、`amplitude >= 0`、`amplitude_mode` 为 `speed` 或 `radius`、`direction` 为 `1` 或 `-1`。四个运动参数可在线修正。
- **Stopped: waiting...**：检查位姿、速度订阅关系及录包器。位姿超时默认 1 秒；应先排除仿真器卡顿、节点退出或连接问题。
- **录包器退出**：查看 launch 终端的具体错误。录包目录需存在、可写且有足够空间；使用标准 launch 时，必需节点退出会使整组节点关闭。

正常实验至少需要 turtlesim 订阅速度；开启录包时还要有 recorder。rqt_plot 也是订阅者，因此只看订阅者数量不能判断每个节点的身份，应同时查看节点名称和录包日志。

## 录包目录或文件有问题

显式指定目录可避免不清楚 `ROS_HOME` 的情况：

```bash
mkdir -p ~/catkin_ws/bags
roslaunch turtlesim_circle circle.launch \
  bag_dir:="$HOME/catkin_ws/bags" duration:=24
```

等待小乌龟停止后，使用 Ctrl+C 正常结束 launch。`.bag.active` 表示文件仍在写入或上次未正常关闭；保留原文件，先确认录制进程已退出。完整文件可检查：

```bash
BAG="$HOME/catkin_ws/bags/实际文件名.bag"
rosbag info "$BAG"
```

应包含 `/turtle1/cmd_vel`（`geometry_msgs/Twist`）和 `/turtle1/pose`（`turtlesim/Pose`）。缺少其中一个话题时，先检查录制命令和实际话题名；分析脚本不会编造缺失的数据。

## rqt_plot 空白或 rqt_graph 缺少连接

图形工具需要可用的桌面显示环境。没有界面的远程终端不能直接提供现场 rqt 截图。

确认控制器正在运行，并在 rqt_plot 中添加完整字段：

```text
/turtle1/cmd_vel/linear/x
/turtle1/cmd_vel/angular/z
```

匀速转圈时水平线是正确结果；两条曲线分别是线速度和角速度，单位不同。首次打开工具只能看到开始订阅之后的数据，不能自动显示整个已录制 bag 的历史。

在 rqt_graph 选择 **Nodes/Topics (all)**、取消隐藏过滤项、点击刷新。查看录包器和 rqt_plot 的连接时，保持它们运行。`rostopic info` 可以对照发布者/订阅者名单。

## 分析环境缺少 rosbag 或 rosbags

在 ROS Noetic 终端中，加载 `/opt/ros/noetic/setup.bash` 后用系统 Python 3 和 `--backend rosbag`。在不安装 ROS 的电脑上，从项目根目录用 Python 3.10+：

```bash
python -m pip install -r requirements-analysis.txt
python scripts/analyze_bag.py --bag "/path/to/circle.bag" \
  --output-dir "/path/to/new_report" --backend rosbags
```

ROS Noetic 的 Python 3.8 环境与独立分析环境的依赖要求不同，不要在同一个环境中强行混用。

## 提示 Output artifacts already exist

分析器默认保护原有的三张 PNG、两份 CSV 和 `summary.json`。推荐按实验批次使用不同目录；确实要重新生成这六个文件时加 `--overwrite`：

```bash
python scripts/analyze_bag.py --bag "/path/to/circle.bag" \
  --output-dir "/path/to/report" --backend rosbags --overwrite
```

这个开关仅针对分析输出，不会覆盖原始 bag，也不会清理目录中的其他实验记录。合成 bag 生成器始终要求新文件名。

## 轨迹不圆、角度突然跳变

先使用默认参数从 turtlesim 初始位置运行。过大的半径可能碰壁，运行中改变参数会产生多个圆弧。theta 原始值在 ±π 附近回绕是正常角度表示；连续展开角度依赖相邻采样的实际转角小于 π，采样缺失或过快转动会造成歧义。

## 私有 GitHub 仓库访问失败

发布前，README 中列出的目标地址可能尚不存在；以已确认的发布状态为准。发布后，私有仓库需要登录有访问权的账号，Git 操作也需要该账号的认证。未登录或无权限时，GitHub 可能显示 404；这不等同于仓库已删除。不要把访问令牌写入代码、文档或提交历史。
