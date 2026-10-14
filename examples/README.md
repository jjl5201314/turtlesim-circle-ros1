# 真实 ROS 录包与分析结果

[`ros1-recordings.zip`](ros1-recordings.zip) 是成功的 [ROS1 integration 运行](https://github.com/jjl5201314/turtlesim-circle-ros1/actions/runs/38447877114) 原始产物副本。它来自 GitHub Actions 的 ROS Noetic / Ubuntu 20.04 容器，运行真实 turtlesim 节点与 rosbag record；不是数学模型生成的合成 bag。

- 被验证的代码提交：`0638387d0696f88041ee1cdade17daaccc3b8640`。
- 运行时间：2026-10-14 北京时间；bag 文件名使用运行器 UTC 日期 2026-10-13。
- ZIP SHA-256：`8d4c5b672fa50d711ade33139b0d89ccda0b902901abfc92bc9ee436f632abde`。
- 两种分析模式均为 `period=8`、`amplitude=1`、`duration=8`，每次约一圈。

解压后包含 `speed/`、`radius/` 和 `rqt/`。前两个目录各有原始 `.bag`、`bag-info.txt`、`launch.log`、`parameters.yaml`、`nodes.txt`、`topics.txt`，以及 `plots/` 内的三张图、两份 CSV 和 `summary.json`。

| 模式 | cmd_vel 样本 | pose 样本 | 起终点距离（turtlesim 单位） |
| --- | ---: | ---: | ---: |
| speed | 357 | 748 | 约 0.0000 |
| radius | 353 | 741 | 约 0.0000 |

`rqt/` 包含真实 rqt_plot / rqt_graph 插件的界面 PNG、完整关系图 DOT、ROS master 发布/订阅状态 JSON 及日志。Noetic 插件运行在 Qt offscreen 宿主中，每个工具有独立 ROS 节点；图像来自真实插件控件渲染，不是手工绘制的示意图，也不是用户电脑桌面截图。rqt_plot 接收实时 cmd_vel，rqt_graph 选择 Nodes/Topics (all) 并关闭隐藏过滤。

rqt 演示单独运行，设置 8 秒启动等待、24 秒运动窗口，在截图完成后提前关闭；其中额外的 bag 用于界面演示，不作为完整三圈的分析样本。完整的一圈分析请使用 speed/ 和 radius/。

示例离线重分析（先将 BAG 改成解压后的实际文件路径）：

```bash
BAG="/path/to/extracted/speed/实际文件名.bag"
python scripts/analyze_bag.py --bag "$BAG" --output-dir reports/recorded_speed
```

分析器不会仅凭任意输入 bag 就宣称来源经过验证，因此 `summary.json` 的来源字段仍保留该限制。这里的真实录包来源由上面的 Actions 运行、归档校验值与日志共同佐证。

用户电脑上的桌面环境未实测；现场运行可按 README 的 roslaunch 命令打开标准 rqt 窗口。
