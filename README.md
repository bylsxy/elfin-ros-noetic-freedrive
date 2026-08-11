# Elfin ROS Noetic 零力拖拽与真机控制套件

<p align="center">
  <img src="docs/images/elfin.png" alt="大族 Elfin 机械臂" width="520">
</p>

面向 **大族机器人 Han's Robot Elfin E05 / Elfin5** 的 ROS 1 社区增强项目。在厂商开源驱动基础上，补齐了 EtherCAT 真机安全接管、六轴重力补偿零力拖拽、中文新手控制面板、实体 FREE/POINT 按钮、MoveIt/Gazebo 联动、末端 I/O 审计和可复现诊断流程。

> 本项目基于 [hans-robot/elfin_robot](https://github.com/hans-robot/elfin_robot) 的 `noetic_ethercat` 分支继续开发，保留原 BSD 3-Clause 许可证和 Han's Robot 版权声明。它是社区维护版本，不代表厂商官方发布，也不是经过安全认证的工业控制器。

## 项目状态

| 项目 | 当前状态 |
| --- | --- |
| 主要平台 | NVIDIA Jetson Orin NX 16GB、Ubuntu 20.04、ROS Noetic |
| 实机型号 | Elfin E05 / Elfin5，3 个双轴关节从站 + 1 个末端 I/O 从站 |
| EtherCAT | SOEM 枚举、自动识别机器人网卡、1 kHz `ros_control` 硬件接口 |
| 常规控制 | Servo On/Off、清 Fault、六轴/笛卡尔点动、MoveIt 轨迹执行 |
| 零力拖拽 | 空载多姿态重力标定和人工拖动已完成；最新新末端样本为 `1.241 kg`、重心半径 `0.461 m`、路径容量峰值 `82.5%`，待最多 1 秒带载保持与首次小范围实机验收 |
| 实体按钮 | `DI bit 4 = POINT`，`DI bit 5 = FREE`，映射固定 |
| 末端 I/O | `INPUT_0..2`、`OUTPUT_0..2` 已对齐；AI0/AI1、RS-485 暂无可用 ROS 协议节点 |
| 灯环 | 发布厂家状态语义；现有驱动没有主动设置实体灯色的接口 |
| 许可证 | BSD 3-Clause |

## 核心能力

- **零力拖拽 / 手动示教**：自研 `elfin_freedrive_controller`，每周期按六轴角度计算 KDL 重力力矩，并叠加可调阻尼、关节限位墙、速度保护和力矩余量。
- **无松闸切换**：位置控制器与力矩控制器通过 `controller_manager` 严格互斥切换；FREE 路径不释放机械抱闸。
- **基础模型 + 自动负载辨识**：保留 17 个空载样本得到的机械臂基础模型；新夹爪、相机和线缆随动部分作为独立的总质量与法兰三维重心自动拟合，不再把按键前的人手预载误学成重力。
- **中文控制面板**：关节、笛卡尔、Servo/Fault、3 DI/3 DO、POINT/FREE、拖拽阻尼、速度保护和维护诊断集中显示，窗口可缩放和滚动。
- **一键启动与诊断**：自动检测 EtherCAT 网卡、清理死亡 ROS 注册、核验模型和 `/joint_states`，并提供离线全栈检查和碰撞后只停机诊断。
- **MoveIt 与 Gazebo**：支持规划、自碰撞检查、RViz 交互目标、轨迹执行和隔离端口的无真机仿真。
- **完整日志**：每次拖拽记录位置、速度、反馈力矩、模型力矩、实际命令、交接进度和状态事件，便于复盘退出原因。

## 系统结构

```text
实体 POINT/FREE ─┐
中文 Panel ──────┼─> elfin_freedrive_manager
ROS 服务 ────────┘          │
                             ├─ 严格切换位置/力矩控制器
MoveIt / RViz ─> 轨迹控制器 ┤
                             v
                   elfin_ros_control
                             │
                   SOEM / EtherCAT 主站
                             │
              Elfin 三组双轴关节 + 末端 I/O
```

日常路径仍是标准 ROS 控制栈；零力拖拽只是一个由管理器按需接管六个 effort 资源的独立控制器，不会替换 MoveIt，也不会绕开驱动 Fault、Servo 状态和关节硬限位。

## 文档导航

| 文档 | 适合什么时候看 |
| --- | --- |
| [E05 新手手动操作手册](docs/e05_manual_operation.md) | 第一次编译、仿真、启动真机、使用 Panel、RViz 和停机 |
| [零力拖拽控制器说明](elfin_freedrive_controller/README.md) | 理解状态机、重力预检、参数、日志和 FREE/POINT 行为 |
| [未知末端负载一键标定](docs/e05_automatic_payload_calibration.md) | 更换夹爪、相机、剪刀或线缆后，理解高位自动辨识、回滚和安全边界 |
| [空载标定与真机验收记录](docs/e05_freedrive_calibration_2026-07-24.md) | 查看标定数据、误退出根因、验证边界和待复验项 |
| [末端 I/O 与接口总表](docs/e05_interface_inventory.md) | 查询 DI/DO、AI、RS-485、灯环、EtherCAT 模块和 ROS 接口 |
| [Jetson 首次恢复记录](docs/e05_jetson_bringup_2026-07-20.md) | 复核设备、从站身份、本机标定、实时调度和首动证据 |
| [Basic API 中文说明](docs/API_description.md) | 查询关节点动、笛卡尔控制和基础服务 |
| [MoveIt RViz 中文教程](docs/moveit_plugin_tutorial.md) | 用鼠标设目标、规划、预览和执行轨迹 |
| [Elfin 模块说明](docs/elfin_module_tutorial.md) | 理解底层双轴模块接口；不作为日常运动入口 |

## 支持环境

本分支以以下组合为基线，其他组合需要重新验证：

- Ubuntu 20.04
- ROS Noetic、catkin、`ros_control`
- MoveIt 1、Gazebo 11、RViz
- SOEM EtherCAT
- Python 3.8 与 wxPython
- Elfin E05 / Elfin5 的 EtherCAT 总线版本

安装常用二进制依赖：

```bash
sudo apt update
sudo apt install \
  ros-noetic-soem \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-moveit \
  ros-noetic-trac-ik \
  python3-wxgtk4.0
```

不要为了安装本项目执行整机 `apt upgrade`、CUDA 重装或全局 Python 升级。ROS 节点和 catkin 工具使用系统 `/usr/bin/python3`。

## 编译

将仓库放在 catkin 工作区的 `src` 目录，然后编译：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/bylsxy/elfin-ros-noetic-freedrive.git elfin_robot
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

这个仓库包含设备专属的 E05 空载标定。换机器人、末端工具或负载后，不能直接把本机标定当作已验证结果；先核对 `elfin_drivers.yaml` 的编码器零位，再重新完成多姿态重力辨识。

## 离线验证

先运行不会连接 EtherCAT、不会使能电机的完整离线检查：

```bash
rosrun elfin_robot_bringup elfin_offline_smoke_test.sh
```

它会完成：

1. Shell 语法检查；
2. 全 catkin 工作区编译；
3. freedrive 的按钮、重力模型、负载回归与安全数学单元测试；
4. E05 Xacro 展开和 URDF 解析；
5. Python 语法检查；
6. 关键 Gazebo、MoveIt、Panel、硬件和 freedrive launch 解析。

只有末尾出现 `PASS` 才说明离线软件链完整。该结果不证明 EtherCAT、电机、抱闸、末端 I/O 或真实运动正常。

## 无真机仿真

```bash
rosrun elfin_robot_bringup start_elfin5_freedrive_sim.sh
```

默认使用独立回环 ROS/Gazebo 端口，同时启动 Gazebo、MoveIt、Basic API 和中文 Panel，不包含 EtherCAT 节点。可选图形界面：

```bash
rosrun elfin_robot_bringup start_elfin5_freedrive_sim.sh --rviz
rosrun elfin_robot_bringup start_elfin5_freedrive_sim.sh --rviz-egl
rosrun elfin_robot_bringup start_elfin5_freedrive_sim.sh --gazebo-gui
```

Jetson 上默认不打开 3D 窗口，以规避已确认的 NVIDIA GLX/RViz 稳定性问题。按 `Ctrl+C` 关闭整套仿真。

## 真机启动

真机操作前必须先读 [E05 新手手动操作手册](docs/e05_manual_operation.md)。至少确认：机器人型号和 EtherCAT 接口正确、底座与负载固定、PE 正常、扫掠区和夹点清空、独立停机手段可用，并从低速空载开始。

终端 A 启动 Servo Off 的真机栈：

```bash
rosrun elfin_robot_bringup start_elfin5_hardware.sh
```

终端 B 启动 MoveIt 与中文 Panel：

```bash
rosrun elfin_robot_bringup start_elfin5_panel.sh --rviz
```

> **事故锁已解除：** 2026-07-27 08:27:45，按用户明确指令移走活动事故锁；原文件已只读归档为 `/home/catas/.ros/ELFIN_FREEDRIVE_LOCKOUT.archived-20260727-082745`。下面的命令不再因事故锁直接退出，但这不代表新末端通过验证：当前正式负载仍为 `0 kg`，六轴模型误差和力矩容量等硬门禁继续生效。详见 [事故记录](docs/e05_freedrive_incident_2026-07-26.md)。

仅在空载或负载模型受支持、并满足现场安全条件的受监督零力拖拽试验中，终端 A 才增加 `--freedrive`：

```bash
rosrun elfin_robot_bringup start_elfin5_hardware.sh --freedrive
```

此参数只开放管理器门禁，不会自动 Servo On、进入 CST 或移动。实体 FREE 必须从第一次观测到高电平起保持至少 `0.70 秒`，并取得至少 8 个 10 Hz 高电平样本；允许过滤一次孤立的低电平毛刺，持续低电平会清空本次候选。`read_di` 通信失败或输入 PDO 不完整时，管理器会立即把末端输入标记为不可用、清空未确认候选，并要求重新观察到稳定低电平后才能再次确认 FREE；不会把读取失败当成 `0`。Panel 请求也必须逐项通过状态新鲜度、静止、Fault、Servo、控制器和六轴模型误差等硬门禁。

## FREE 与 POINT

| 输入 | 固定映射 | 行为 |
| --- | --- | --- |
| POINT | 原始 DI 字的 bit 4 | 每个按下沿持久记录一次时间与六轴关节位置，不发送轨迹 |
| FREE | 原始 DI 字的 bit 5 | 连续按住约 `0.70--0.80 秒`才会满足软件的 0.70 秒/8 样本确认；保持期间可手动拖动，松开后在新位置恢复位置保持 |

FREE 不会打开机械抱闸。管理器在切入前被动观察位置模式下的反馈力矩；该数据用于预检和 0.5 秒无跳变交接，不再用于在线改变固定重力倍率。更换夹爪、相机或负载后，必须重新标定，不能靠按住 FREE 前施力“调重力”。

Panel 的“拖拽高级”提供“高位一键标定未知末端”：主体辨识始终在普通位置控制下完成，6 个双向姿态拟合总质量和三维重心、2 个独立姿态验算，最后才做默认 0.8 秒且硬上限 1 秒的高阻尼零力保持。完整前置条件和物理边界见 [未知末端负载一键标定](docs/e05_automatic_payload_calibration.md)。

运行数据默认保存到 `$ROS_HOME`，未设置时使用 `~/.ros`：

```text
elfin_freedrive_points.yaml
elfin_freedrive_trials/
elfin_freedrive_gravity_samples.csv
elfin_freedrive_gravity_candidate.yaml
elfin_freedrive_payload.yaml
elfin_payload_calibration_runs/
```

## 已知边界

- 当前重力标定只覆盖这台 E05 的空载、无末端工具状态。
- 自动负载辨识只能把法兰之后的物体近似成一个牢固的刚性总质量与固定重心；未知外形碰撞、松动部件和姿态相关线缆拉力不能被保证，留出姿态不一致时会拒绝。
- 2026-07-25 最新误退出来自旧入口自适应：人手预载使倍率估计降到 `0.544`，钳位后仍欠补偿，J3 最终超过 `0.4 rad/s`。该自适应现已在配置和源码缺省值中关闭，但修复后仍需重新做受监督真机复验。
- 速度、关节、Fault、通信和控制器丢失保护仍然保留；不应为了连续拖动而删除这些退出条件。
- 末端 `AI0/AI1` 和 `485_A/B` 只有物理端子信息，当前驱动没有完整解码或协议节点。
- 灯环颜色目前只是期望状态话题，不代表软件已经向实体灯环写色。
- MoveIt 只会避开已加入 planning scene 的物体；没有深度感知节点时，它不知道现实中的桌子、树枝和人员位置。
- 软件 Stop 和 Servo Off 不是安全认证急停，不能替代独立硬件停机链路。

## 仓库结构

```text
elfin_freedrive_controller/  六轴重力补偿控制器、管理器、消息、服务与测试
elfin_robot_bringup/         真机/仿真/Panel 启动、网卡检测、停机与离线检查
elfin_ros_control/           Elfin ros_control 硬件接口
elfin_ethercat_driver/       SOEM EtherCAT 与 PDO/SDO 驱动
elfin_basic_api/             Basic API 与中文控制面板
elfin5_moveit_config/        E05/Elfin5 MoveIt 与低负载 RViz 配置
elfin_gazebo/                Gazebo 仿真
elfin_description/           URDF、传动和模型资源
docs/                        中文操作、接口、标定和恢复文档
```

## 参与开发

提交改动前至少执行 `git diff --check`、目标包编译和 freedrive 单元测试。涉及控制器共享资源、EtherCAT 写周期、力矩、抱闸、Servo 或 Fault 的改动，还必须按“静态配置 → 编译 → 仿真 → 传感器/状态只读 → 低速空载真机 → 集成任务”的顺序验证，并在文档中记录设备、负载、姿态、启动命令和观察结果。

问题报告请附上：机器人型号与末端版本、ROS/系统版本、启动命令、`/joint_states`、Servo/Fault 状态、控制器列表，以及对应的 freedrive trial CSV 事件行。不要上传密钥、内网凭据或无法公开再分发的厂商资料。

## 检索关键词

Elfin、Elfin5、E05、Han's Robot、大族机器人、ROS Noetic、ROS 1、EtherCAT、SOEM、`ros_control`、MoveIt、Gazebo、Jetson Orin NX、freedrive、hand guiding、gravity compensation、zero-force teaching、零力拖拽、手动示教、重力补偿、拖拽示教、机械臂力控。

## 许可证与致谢

项目沿用 [BSD 3-Clause](LICENSE.txt)。原始 ROS 驱动版权归 Han's Robot Co., Ltd. 所有；社区增强代码在相同许可证下发布。感谢厂商开源基础驱动和 ROS、MoveIt、SOEM、Orocos KDL 等项目。
