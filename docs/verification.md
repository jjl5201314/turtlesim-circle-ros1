# 交付验证记录

日期：2026-10-14（北京时间）。本地验证环境：Windows，Python 3.12.14；远端 ROS 验证环境：GitHub Actions 的 ROS Noetic / Ubuntu 20.04 容器。

## 已实际执行

1. `python -m pytest -q tests`：**38 passed，36 subtests passed**。
   - 10 项运动参数测试：两种幅值模式、周期关系、旋转方向、零幅值、非法值、数值溢出。
   - 13 项控制器测试：模拟 ROS 接口与时钟，检查启动等待、参数热更新、非法参数停车及恢复、位姿超时及无效值、连接丢失和恢复、定时停止、异常退出及停止过程。这些是单元测试，不是 ROS 实测。
   - 15 项分析测试：ROS1 二进制 bag 写入与读取、异步时间轴、纳秒精度、消息校验、CSV、图像导出、合成模型闭环、防覆盖、导出失败保留旧结果、内嵌来源声明、SHA-256 校验和可复现生成。
2. `catkin_pkg` 成功解析 `package.xml`；Python 源文件通过 Python 3.8 语法检查；两个 launch 文件通过 XML 解析。本地未执行 catkin 构建，远端构建结果见下节。
3. 用更新后的生成器分别生成并分析 speed / radius 两种模式的标准 ROS1 `.bag` 文件。每份包含 541 条速度指令、1620 条位姿消息及 1 条合成来源声明，公共时间轴覆盖 27 秒，包含 2 秒静止、24 秒运动和 1 秒静止。单独移动 bag 后仍可识别合成来源；分析报告保存文件名与 SHA-256，不包含本机绝对路径。
4. 检查本地合成数据的速度、位置及轨迹图。这些合成示例图均明确标注 `SYNTHETIC DEMO - no live ROS run`，与下方远端实际录包分别保存。

离线依赖版本：NumPy 2.3.5、Matplotlib 3.11.2、Rosbags 0.11.5、pytest 9.1.1。

## 远端 ROS 实际运行

代码提交 `0638387d0696f88041ee1cdade17daaccc3b8640` 已通过：

- [Python checks](https://github.com/jjl5201314/turtlesim-circle-ros1/actions/runs/38447877069)：38 项测试、36 项子测试和语法编译通过。
- [ROS1 integration](https://github.com/jjl5201314/turtlesim-circle-ros1/actions/runs/38447877114)：`catkin_make` 构建成功，实际启动 turtlesim、控制节点与 rosbag，分别完成两种幅值模式的一圈录制和分析。
- 两种模式的指令速度、报告速度、运动时长、最终停止和轨迹闭合检查通过。speed 录得 357 条指令及 748 条 pose，起终点距离约 0.0000；radius 录得 353 条指令及 741 条 pose，起终点距离约 0.0000。

原始 bag、参数、节点/话题清单、日志、图与 CSV 已归档到 [examples/ros1-recordings.zip](../examples/ros1-recordings.zip)，文件来源与 SHA-256 见 [examples/README.md](../examples/README.md)。README 首页曲线来自这次实际录包；rqt 界面图由同次工作流单独运行的真实插件生成，并保存了 ROS master 状态与 DOT 关系图。

第一次集成运行已完成运动，但日志被缓冲，等待完成标记的步骤超时。随后为测试环境设置 `PYTHONUNBUFFERED=1` 并完善退出等待；后续录包运行全部成功。随后调整旧版 Matplotlib 的标题留白，并补充真实 rqt 插件检查。最终上述运行全部通过；这些修正不改变控制算法。

## 合成示例结果

两组均使用 `period=8`、`amplitude=1`，理想运动三圈。

| 指标 | speed | radius |
| --- | --- | --- |
| 指令线速度 | 1.000000 | 0.785398 |
| 指令角速度 / rad·s⁻¹ | 0.785398 | 0.785398 |
| 理论圆半径 | 1.273240 | 1.000000 |
| 根据采样点累计的路径长 | 23.999829 | 18.849421 |
| 首末位置距离 | 0 | 0 |
| 展开角度总变化 / rad | 18.849556 | 18.849556 |

这些数值来自数学模型写入的合成 bag，不是 turtlesim 录制结果。坐标采用 turtlesim 坐标单位。位姿用 float32 存储，因此与双精度指令可能有微小数值差异；折线累计路径长度也略短于解析弧长。

## rqt 界面验证与范围

- 实际运行 Noetic rqt_plot / rqt_graph 插件，每个工具使用独立 ROS 节点；rqt_plot 收到两个 cmd_vel 字段，rqt_graph 显示完整节点/话题关系。
- 已保存并查看两个插件控件的 PNG、完整关系图 DOT 与 ROS master 的发布/订阅 JSON；来源见 [归档说明](../examples/README.md)。
- 使用 Qt offscreen 宿主获取真实插件控件图像，不是 Windows 桌面截图。用户电脑上的标准桌面窗口尚未实测。

## 代码托管

GitHub 私有仓库：[jjl5201314/turtlesim-circle-ros1](https://github.com/jjl5201314/turtlesim-circle-ros1)。代码已上传，远端文件内容哈希与脚本可执行权限已核对；本地 Git 保留开发历史备份，并与远端 main 同步。

真实实验请依照 [experiment.md](experiment.md) 运行，保存 `.bag`、参数、节点话题清单和 rqt 截图。
