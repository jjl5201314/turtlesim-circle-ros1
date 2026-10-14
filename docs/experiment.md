# 小乌龟圆周运动实验说明

## 实验目标与实现

本实验通过 Python ROS 节点向 `/turtle1/cmd_vel` 连续发送线速度与角速度，使 turtlesim 做圆周运动。参数服务器提供周期和幅值配置；`circle.launch` 启动仿真、控制器、录包器和可选的图形工具；`analyze_bag.py` 从 bag 中读取速度指令和位姿并绘图。

| 作业要求 | 对应实现 |
| --- | --- |
| ROS package，C++ 或 Python | Python 3 catkin package `turtlesim_circle` |
| 配置周期、幅值 | `period`、`amplitude`、`amplitude_mode`，支持 speed / radius 两种定义 |
| roslaunch | `launch/circle.launch` |
| 记录速度和位置 | `/turtle1/cmd_vel`、`/turtle1/pose` 同时写入 bag |
| MATLAB 或 Python 分析 | Python 分析脚本，生成 PNG、CSV 和 JSON |
| rqt_plot | 查看 `linear/x`、`angular/z` |
| rqt_graph | 查看节点与话题的实际发布/订阅关系 |
| Git 托管 | Git 仓库与 GitHub 工作流；私有访问需授予账号权限 |

## 理论依据

对于前向线速度 v 和角速度 ω，平面运动学为：

```text
dx/dt = v cos(theta)
dy/dt = v sin(theta)
dtheta/dt = omega
```

当 v、ω 恒定且 ω 不为 0 时，轨迹为圆，半径 `R=|v/ω|`，一圈周期 `T=2π/|ω|`。若从 `(x0,y0)` 出发、初始朝向 `theta0=0`，以运动开始为 `τ=0`：

```text
x(τ) = x0 + (v/omega) sin(omega τ)
y(τ) = y0 + (v/omega) [1 − cos(omega τ)]
theta_unwrapped(τ) = omega τ
```

这里的运动时间 τ 与图中的 bag 时间不同：bag 可能包含启动前和停止后的静止段。两者之间差一个运动起始时间偏移。初始方向不为零时，需要在通用解中保留 `theta0`；不能直接把以上特例用于任意启动状态。

两种幅值定义统一使用 `ω=d·2π/T`：

| 模式 | 输入的幅值 A | 线速度 | 半径 | 改变周期、保持 A 不变时 |
| --- | --- | --- | --- | --- |
| speed | m/s | `v=A` | `R=A·T/(2π)` | 周期越长，半径越大 |
| radius | m | `v=2πA/T` | `R=A` | 半径不变，周期越长速度越小 |

方向 `d=1` 为逆时针，`d=-1` 为顺时针。幅值为 0 时线速度为 0，配置的角速度仍可产生原地旋转；这属于退化的零半径轨迹。

## 建议实验组合

每组分别启动 launch、完成录制、正常停止，再运行下一组。全部实验使用 `duration:=24`；其他参数按下表覆盖。以下数值是理论预期，不是已测量结果。

| 实验 | mode | T / s | A | direction | 预期 v / (m/s) | 预期 ω / (rad/s) | 预期 R / m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | speed | 8 | 1 | 1 | 1.000000 | 0.785398 | 1.273240 |
| B | radius | 8 | 1 | 1 | 0.785398 | 0.785398 | 1.000000 |
| C | radius | 4 | 1 | 1 | 1.570796 | 1.570796 | 1.000000 |
| D | radius | 8 | 1 | -1 | 0.785398 | -0.785398 | 1.000000 |

实验 A 与 B 对比幅值定义；B 与 C 对比相同半径下的周期变化；B 与 D 对比旋转方向。初始位置靠近边界或增大半径可能导致碰壁，碰壁数据不能用于验证理想圆周模型。

## 操作与证据保存

1. 按 README 安装依赖、构建 package 并加载工作空间环境。
2. 使用 `record:=true plot:=true graph:=true` 启动实验；确认录包开始和小乌龟运动。
3. 在 rqt_plot 检查两条 `cmd_vel` 曲线，保存窗口截图。速度指令稳定阶段应为水平线。
4. 在 rqt_graph 选择 `Nodes/Topics (all)`、取消隐藏过滤项并刷新，保存实际关系图。录包器和 rqt_plot 必须仍在运行。
5. 等待设定运动时间结束，确认小乌龟停止；再按 Ctrl+C 正常结束 launch，等待 `.bag.active` 变为完整 `.bag`。
6. 使用 `rosbag info` 核对两类话题均有消息；保存输出与实际参数。
7. 运行分析脚本，将三张 PNG、两份 CSV 和 `summary.json` 保存在本组实验专用目录。

例如实验 B：

```bash
mkdir -p ~/catkin_ws/bags
roslaunch turtlesim_circle circle.launch \
  amplitude_mode:=radius period:=8 amplitude:=1 direction:=1 \
  duration:=24 record:=true plot:=true graph:=true \
  bag_dir:="$HOME/catkin_ws/bags" bag_prefix:=experiment_B
```

另一终端记录运行参数和图结构信息：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
mkdir -p ~/catkin_ws/reports/experiment_B
rosparam get /circle_controller > ~/catkin_ws/reports/experiment_B/parameters.yaml
rosnode list > ~/catkin_ws/reports/experiment_B/nodes.txt
rostopic list > ~/catkin_ws/reports/experiment_B/topics.txt
rostopic info /turtle1/cmd_vel > ~/catkin_ws/reports/experiment_B/cmd_vel_connections.txt
rostopic info /turtle1/pose > ~/catkin_ws/reports/experiment_B/pose_connections.txt
```

正常停止后，查看 bag 文件名，并将 `BAG` 改为本次实验的文件路径：

```bash
ls -lh ~/catkin_ws/bags/*.bag
BAG="$HOME/catkin_ws/bags/实际文件名.bag"
rosbag info "$BAG" \
  > ~/catkin_ws/reports/experiment_B/bag_info.txt
rosrun turtlesim_circle analyze_bag.py \
  --bag "$BAG" \
  --output-dir ~/catkin_ws/reports/experiment_B
```

已有分析图表和数据表默认不会被覆盖。重新分析时使用新目录，或核对路径后增加 `--overwrite`；原始 bag 不会被分析器修改。

## 曲线应如何解释

**速度—时间曲线。** 控制指令只设置 `linear.x=v`、`angular.z=ω`，其余分量为零。稳态时两条指令曲线保持常数；运动结束后应回到零。`Pose` 中的速度来自仿真器报告，可与指令变化对照，但不是通过位移独立估算的测速结果。

**位置—时间曲线。** 理想圆周运动的 x、y 是同周期、相差四分之一周期的正弦型变化。绝对位置包含初始坐标和圆心偏移。theta 在 ±π 附近回绕是角度表示方式；展开后的 theta 在稳态应随时间近似线性变化，斜率为 ω。停止后位置应保持稳定。

**XY 轨迹。** 应接近圆，绘图采用相同的 x/y 比例，避免视觉上把圆拉成椭圆。数值积分、消息采样和 ROS 调度会使轨迹与理想解析解存在小差异。若出现沿窗口边缘运动的段落，优先检查半径是否过大或是否发生碰壁。

**时间对齐。** 两个话题都没有 Header，使用 bag 接收时间，并从两类有效消息共同的最早时间减去零点。分析保留原始采样点，没有强制配对、插值或平滑。不同话题的采样率、启动时间和通信延迟可能不同；不能直接将相同行号当成同时发生的数据。

**运行中修改参数。** 修改后产生分段的速度常量和不同半径的轨迹，参数变化时刻附近会出现跳变。这类数据适合演示在线参数生效，但不能用一个固定圆模型拟合全程。

**位姿或连接中断。** 控制器会在有效 pose 超过 `pose_timeout`（默认 1 秒）未更新或就绪条件丢失时输出零速度。恢复后重新等待 `startup_delay`；`duration` 的计时不重置，也不会扣除暂停时间。因此有中断的 24 秒运动窗口可能不足 3 圈，应在实验记录中说明。

## rqt 图中的关系

- `/circle_controller` 发布 `/turtle1/cmd_vel`。
- `/turtlesim` 订阅 `/turtle1/cmd_vel`，发布 `/turtle1/pose`，还会提供颜色话题及相关服务。
- `/circle_controller` 也订阅 `/turtle1/pose`，持续检查位姿是否有效、是否超时。
- `/circle_recorder` 订阅两类实验数据；启动录制时发布 `/circle_recorder/begin_write`，控制器用它确认录包已就绪。
- rqt_plot 订阅 `/turtle1/cmd_vel` 并提取 `linear.x`、`angular.z`。
- rqt_graph 通过 ROS 图信息显示关系；它本身并不是速度数据的必经节点。
- `/rosout` 等日志连接和工具内部节点也属于实际 ROS 图。查看“全部关系”时保留它们；报告中若只截核心部分，应明确说明已裁剪或过滤。

README 内的 Mermaid 表示设计预期；另外提供真实 rqt_graph 插件生成的完整图，原始 DOT 和 ROS master 状态可在录包归档中核对。

## 验证状态与填写记录

已在 GitHub Actions 的 ROS Noetic 容器中运行真实 turtlesim，并完成两种模式的录包和分析，结果见 [验证记录](verification.md) 与 [真实录包归档](../examples/README.md)。容器采用 Qt offscreen 模式，已运行真实 rqt_plot / rqt_graph 插件并保存界面图、完整 DOT 和 ROS master 状态。用户电脑桌面未实测，现场演示仍按本文命令打开标准窗口。合成 bag 另外用于检验文件读取、数据导出和数学模型。

实测后补全以下记录：

| 检查项 | 记录内容 |
| --- | --- |
| 运行环境 | Ubuntu / ROS / Python 版本，package 提交号 |
| 实验参数 | mode、T、A、direction、duration、rate |
| 真实 bag | 文件名、录制时长、两个话题的消息数 |
| 速度 | 稳态指令值、启动和停止变化、pose 报告值 |
| 位置 | 观察到的周期、轨迹半径、是否碰壁 |
| 图形工具 | rqt_plot 截图、rqt_graph 截图 |
| 异常与解释 | 丢失话题、非法参数、边界限制、采样延迟等 |

实验记录应附带真实 bag 和截图。合成示例用于演示分析程序，其来源标记应随文件保留，不能用作现场录制证据。自动测试的实际结果和运行环境见 [验证记录](verification.md)。
