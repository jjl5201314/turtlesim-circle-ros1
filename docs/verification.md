# 交付验证记录

日期：2026-10-14。验证环境：Windows，Python 3.12.14。

## 已实际执行

1. `python -m pytest -q tests`：**38 passed，36 subtests passed**。
   - 10 项运动参数测试：两种幅值模式、周期关系、旋转方向、零幅值、非法值、数值溢出。
   - 13 项控制器测试：模拟 ROS 接口与时钟，检查启动等待、参数热更新、非法参数停车及恢复、位姿超时及无效值、连接丢失和恢复、定时停止、异常退出及停止过程。这些是单元测试，不是 ROS 实测。
   - 15 项分析测试：ROS1 二进制 bag 写入与读取、异步时间轴、纳秒精度、消息校验、CSV、图像导出、合成模型闭环、防覆盖、导出失败保留旧结果、内嵌来源声明、SHA-256 校验和可复现生成。
2. `catkin_pkg` 成功解析 `package.xml`；Python 源文件通过 Python 3.8 语法检查；两个 launch 文件通过 XML 解析。未执行 catkin 构建。
3. 用更新后的生成器分别生成并分析 speed / radius 两种模式的标准 ROS1 `.bag` 文件。每份包含 541 条速度指令、1620 条位姿消息及 1 条合成来源声明，公共时间轴覆盖 27 秒，包含 2 秒静止、24 秒运动和 1 秒静止。单独移动 bag 后仍可识别合成来源；分析报告保存文件名与 SHA-256，不包含本机绝对路径。
4. 检查生成的速度、位置及轨迹图。所有示例图均明确标注 `SYNTHETIC DEMO - no live ROS run`。

离线依赖版本：NumPy 2.3.5、Matplotlib 3.11.2、Rosbags 0.11.5、pytest 9.1.1。

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

## 尚未执行

- ROS Noetic 中的 `catkin_make`、真实 turtlesim 运行和 `rosbag record`。
- rqt_plot、rqt_graph 的现场窗口验证和截图。
- GitHub Actions 的结果尚待确认。已配置 Python 检查和 ROS1 容器录制分析，最新运行见 [Actions](https://github.com/jjl5201314/turtlesim-circle-ros1/actions)。

## 代码托管

GitHub 私有仓库已创建：[jjl5201314/turtlesim-circle-ros1](https://github.com/jjl5201314/turtlesim-circle-ros1)。仓库访问权限和本地测试均已核对；代码上传与远端运行结果分别验证，不以“已配置工作流”代替运行成功。

真实实验请依照 [experiment.md](experiment.md) 运行，保存 `.bag`、参数、节点话题清单和 rqt 截图。
