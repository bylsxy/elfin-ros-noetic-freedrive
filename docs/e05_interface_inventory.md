# E05 实体、电气、ROS 与 Panel 接口清单

本文记录 2026-07-22 对本机 Han's Robot / 华沿 E05 的实物、厂家电气说明书和当前 ROS Noetic 源码的交叉审计。目的不是把每个底层入口都做成按钮，而是明确：实体上有什么、ROS 已经实现什么、哪些仍需验证。

## 1. 证据等级

按可靠性从高到低区分四类证据：

1. `实物`：本机铭牌、端子标签和照片直接可见。
2. `厂家手册`：`docs/机器人电气说明书（中文版）归一化20240731(1)-1.pdf` 的表格或示意图。
3. `活动源码`：当前 Noetic 启动路径真正调用的 C++/Python 代码。
4. `历史代码`：未被活动服务调用的旧 Modbus、LED、End Button 等实现，只能作为线索，不能当作本机接口定义。

遇到冲突时必须保留冲突说明，不能为了让 Panel 看起来“齐全”而猜通道。

## 2. 机械臂底座和当前控制路径

| 实体接口 | 当前用途 | 状态 |
| --- | --- | --- |
| `48V+`、`48V-` | 机械臂主电源 | 已用于现有 E05 系统 |
| `PE` | 保护接地 | 现场已做连续性检查；它不是信号 GND |
| EtherCAT 网口 | Jetson 到 E05 四个从站 | 启动脚本按从站身份自动选择网卡 |

当前项目走“Jetson + ROS Noetic + SOEM EtherCAT”路径。厂家手册中的完整控制箱、手持示教器和 Mini control box 接口不等于当前 ROS 已接管它们。

## 3. 末端 12 芯工具接口

厂家手册第 29/34 页与实物标签给出以下定义。手册把 Pin 9 印成 `AIO`，实物标签和信号说明均表明它是模拟输入 0，本文统一写成 `AI0`。

| Pin | 实体标签 | 作用 | 当前 ROS 覆盖 |
| --- | --- | --- | --- |
| 1 | `INPUT_0` | 数字输入 0 | `read_di`，Panel 只读 DI0 |
| 2 | `INPUT_1` | 数字输入 1 | `read_di`，Panel 只读 DI1 |
| 3 | `INPUT_2` | 数字输入 2 | `read_di`，Panel 只读 DI2 |
| 4 | `OUTPUT_0` | 数字输出 0 | `read_do` / `write_do`，Panel DO0 |
| 5 | `OUTPUT_1` | 数字输出 1 | `read_do` / `write_do`，Panel DO1 |
| 6 | `OUTPUT_2` | 数字输出 2 | `read_do` / `write_do`，Panel DO2 |
| 7 | `485_A` | RS-485 A | 物理存在；当前没有 ROS 协议节点 |
| 8 | `485_B` | RS-485 B | 物理存在；当前没有 ROS 协议节点 |
| 9 | `AI0` / 手册 `AIO` | 模拟输入 0 | 驱动未解码、未发布 |
| 10 | `AI1` | 模拟输入 1 | 驱动未解码、未发布 |
| 11 | `24V` | 工具电源 | 物理电源，不是 ROS 数据接口 |
| 12 | `GND` | 工具 0 V | 工具信号参考地，不是 PE |

### 3.1 电气规格

- 工具电源：24 V，典型允许电流 1 A，最大 1.5 A。手册注明相关端口共用同一路内部电源，过流会保护关断并写机器人日志。
- 数字输入：PNP；`-3..5 V` 为 OFF，`11..30 V` 为 ON，浮空由弱下拉保持 OFF。
- 模拟输入：`0..10 V`、约 `15 kOhm` 输入电阻、非差分、共用 GND。
- 数字输出：最大 0.4 A，手册表格写“PNP”，同页文字和接线图却写“激活后导通到 GND、禁用后开路（开集/开漏）”。这两种描述在电流方向上矛盾。
- 在台架用限流负载和万用表确认输出拓扑以前，不得按“PNP 高侧输出”或“低侧开漏”任一假设直接接夹爪、电磁阀。感性负载还需要保护二极管。

## 4. 灯环不是四路用户 LED

厂家手册第 32/34 页把末端灯环定义为机器人状态指示：

| 颜色 | 厂家定义 |
| --- | --- |
| 绿色 | 上电使能状态、自由驱动状态 |
| 红色 | 异常去使能状态 |
| 白色 | 负载识别状态 |
| 蓝色 | 开启零力示教状态 |
| 黄色 | 上电去使能状态 |
| 紫色 | 程序自动运行状态 |

当前 ROS 驱动没有“设置灯环颜色”服务。旧 Panel 的 `LED0..LED3` 来自历史输出寄存器解释，不能证明能控制本机可见灯环，现已从 E05 Panel 删除。freedrive 管理器会按当前状态发布 `/elfin_freedrive_manager/ring_state` 和 `/elfin_freedrive_manager/ring_color`，Panel 也会显示绿/黄/红/蓝的**预期语义**；这两个话题用于界面和将来对接厂家接口，不会写实体灯环。

## 5. 末端实体 POINT / FREE 按键

实物照片 `Desktop/图片传输/bf9d14573e00bf4342f95e9ae43866de.jpg` 显示 `POINT` 和 `FREE` 两个按钮。本机已经固定确认它们在 `/elfin_ros_control/elfin/io_port1/read_di` 原始 16 位字中的映射：

| 实体按键 | 固定输入位 | Panel 行为 |
| --- | --- | --- |
| `POINT` | DI bit 4 | 按下沿把时间与 J1..J6 弧度持久写入 `~/.ros/elfin_freedrive_points.yaml`；不发送轨迹 |
| `FREE` | DI bit 5 | 从首次高电平观测起至少持续 `0.70 秒`且至少收到 8 个 10 Hz 高电平样本后，才请求切换到有界重力补偿；一次孤立低电平毛刺会被过滤，持续低电平会清空候选；确认松开后按保护路径恢复当前位置保持 |

这两个内部按键位不等于末端接线端子的 `INPUT_0..2`（DI bit 0..2），Panel 不再运行时猜测或临时学习映射。管理器启动时若发现 FREE 已经按下，必须先看到一次释放，再允许下一次连续按压确认，避免启动时误进入。2026-07-26 事故锁已在 2026-07-27 按用户指令解除；按钮长按、负载模型和其他运行门禁保持不变，详见 [事故记录](e05_freedrive_incident_2026-07-26.md)。

当前新增的 `elfin_freedrive_controller` 使用驱动原有 CST/力矩模式和 KDL 动力学模型实现重力补偿。它与“全部松抱闸”完全不同：Servo 保持使能、每轴持续受力矩上限/变化率/速度/关节限位约束，且 FREE 行为永远不会调用 `open_brake_slaveX` 或 `close_brake_slaveX`。受保护的单模块松闸仍只在维护窗口中提供。

## 6. 当前 EtherCAT I/O 驱动的真实行为

活动实现位于：

```text
elfin_ethercat_driver/src/elfin_ethercat_io_client.cpp
```

| ROS 服务 | 活动源码行为 | 结论 |
| --- | --- | --- |
| `/elfin_ros_control/elfin/io_port1/read_di` | 优先从 slave 4 的周期输入 PDO 读取 `0x6001:01` 对应的前 16 位；仅在旧 ESI 未映射输入 PDO 时回退到 SDO | 可读数字输入原始字；PDO/SDO 传输或长度校验失败时 ROS 服务失败，不再返回伪造的 `0` |
| `/elfin_ros_control/elfin/io_port1/read_do` | 读 slave 4 的 `0x7001:01`，左移 12 位返回 | 可回读数字输出寄存器 |
| `/elfin_ros_control/elfin/io_port1/write_do` | 请求值右移 12 位后写 `0x7001:01` | Panel 仅开放 DO0..2，并采用“读原值、改单个位、写后回读” |
| `/elfin_ros_control/elfin/io_port1/get_txpdo` | 返回 20 字节全零字符串 | 占位实现，不是真实 PDO |
| `/elfin_ros_control/elfin/io_port1/get_rxpdo` | 返回 4 字节全零字符串 | 占位实现，不是真实 PDO |

源码还存在这些限制：

- 构造函数的状态门禁使用 `slave_no_`，但实际寄存器读写仍固定为厂家 E05 I/O slave 4；当前 bringup 也固定把该客户端绑定到 slave 4。
- `readInput_unit(int n)` 仍只实现数字输入；参数 `n` 仅用于拒绝未实现的 AI/相机通道，不会再为这些通道返回伪造数据。
- RS-485 只有物理 A/B 端子，没有帧格式、波特率、设备协议或 ROS 节点。
- 旧 Modbus 函数和 `L_4/H_4 is button DI` 注释没有被当前活动服务调用，只能作为历史线索。

## 7. 运动、驱动和维护接口

### 7.1 高层 Basic API

| 接口 | Panel 入口 | 作用 |
| --- | --- | --- |
| `/elfin_basic_api/enable_robot` | 伺服上电 | 位置对齐检查、全轴使能、启动轨迹控制器；失败时回滚 |
| `/elfin_basic_api/disable_robot` | 伺服关闭 | 取消轨迹、全轴 Servo Off、停止控制器 |
| `/elfin_basic_api/joint_teleop` | J1..J6 `+/-` | 持续关节点动，松开调用 Stop |
| `/elfin_basic_api/cart_teleop` | X/Y/Z/Rx/Ry/Rz `+/-` | 持续笛卡尔点动，松开调用 Stop |
| `/elfin_basic_api/home_teleop` | 回 ROS 零位 | 按住向固定六轴零位运动，松开停止 |
| `/elfin_basic_api/stop_teleop` | 停止轨迹 | 取消 Panel/轨迹 action，不是物理断电 |
| `/elfin_basic_api/set_reference_link` | 设置坐标系 | 改变笛卡尔点动参考系 |
| `/elfin_basic_api/set_end_link` | 设置坐标系 | 改变笛卡尔点动末端连杆 |

### 7.2 零力拖拽控制器与管理器

| 接口 | 作用 |
| --- | --- |
| `/elfin_freedrive_manager/set_freedrive` | `SetBool(true)` 请求进入，`false` 请求退出；真机默认由启动门禁锁定 |
| `/elfin_freedrive_manager/record_point` | 立即持久记录当前六轴姿态，不运动 |
| `/elfin_freedrive_manager/list_recorded_points` | 返回实际 YAML 路径、序号、时间、来源和 J1--J6；供姿态管理窗口逐条展示 |
| `/elfin_freedrive_manager/delete_recorded_point` | 删除指定序号，原子重写 YAML 并连续重排其余序号 |
| `/elfin_freedrive_manager/state`、`state_detail`、`active` | Panel 使用的状态、完整原因和活动标志 |
| `/elfin_freedrive_manager/recorded_point`、`point_count` | 最近记录姿态与累计数量 |
| `/elfin_freedrive_controller/command_state` | 控制器的六轴位置、速度和实际下发力矩 |
| `/elfin_freedrive_controller/status` | 正常、软限制、求解失败或硬安全停止状态 |
| `/elfin_freedrive_controller/telemetry` | 六轴反馈/模型/命令力矩、接管进度、模型一致性和减速状态 |
| `/elfin_freedrive_controller/request_settle` | 管理器内部使用的单向增强阻尼请求，不是日常人工按钮 |
| `/elfin_freedrive_manager/model_validation` | Panel 显示的位置模式被动重力预检结果 |
| `/elfin_freedrive_manager/trial_log_path` | 本次 CSV 的绝对路径 |
| `/elfin_freedrive_manager/record_gravity_sample` | 仅在位置控制、静止、Servo On、无 Fault 时记录一组被动标定样本 |
| `/elfin_freedrive_manager/fit_gravity_calibration` | 对双向成对姿态抵消静摩擦后拟合，并生成候选 YAML；不会热改控制器 |
| `/elfin_freedrive_manager/get_payload_model` | 读取当前末端总质量、法兰三维重心、验证指标、同步状态和 YAML 路径 |
| `/elfin_freedrive_manager/set_payload_model` | 仅在 FREE 停止时暂存/持久化或回滚经过验证的末端负载模型 |
| `/elfin_freedrive_manager/evaluate_payload_model` | 返回指定六轴姿态的空臂重力、末端线性回归矩阵和力矩容量，供自动标定只读计算 |

进入条件包括：六轴状态新鲜完整、速度低于 `0.02 rad/s`、Servo On、无 Fault、位置控制器在线、本次真机启动显式允许 freedrive，并且位置模式下的反馈保持力矩通过 KDL 方向/比例/残差预检。管理器用 `STRICT` 原子切换保证位置控制与力矩控制不会同时占用六轴。FREE 松开或保护触发后先请求增强阻尼；正常恢复状态为 `EXITING -> READY`，保护恢复为 `RECOVERING -> READY`，真机直接切回失败时可短暂显示 `HOLDING`，表示驱动器已用当前编码器位置进入 CSP 且管理器正在恢复 ROS 位置控制器。只有位置保持完全无法建立时才请求 Servo Off。

控制器每周期按当前六轴角度重新计算 URDF/KDL 重力力矩，并叠加阻尼、软关节限位和超速阻尼；活动拖拽没有位置弹簧。CST 入口先沿用位置模式实测保持力矩，再用 0.5 秒平滑过渡到固定的多姿态标定模型。FREE 输出直接采用 E05 URDF 六轴额定力矩 `[420, 420, 200, 200, 69, 69] Nm`，不再叠加历史 20% 或 `[15, 84, 36, 15, 10, 8] Nm` 人工帽；未验证模型重力达到额定值 90% 时拒绝/退出，已验证负载使用 92% 迟滞线。每次试验 CSV 默认保存在 `$ROS_HOME/elfin_freedrive_trials/`，未设置 `ROS_HOME` 时为 `~/.ros/elfin_freedrive_trials/`。2026-08-11 候选 J5 `9.411 Nm` 的反复拒绝已确认来自旧 `10 Nm` 人工帽，而非测量失败。

旧硬件接口曾把反馈 `axis.effort` 转为目标力矩，导致 effort 控制器命令没有真正写入 EtherCAT。现已改为 `axis.effort_cmd`，并在 CST 入口预置实测保持力矩、CSP 入口同步当前编码器位置、饱和目标力矩 PDO。2026-07-24 空载真机静态试验已验证完整交接和恢复，但带载与人工手感仍须分开验收。

Panel“拖拽高级”已接入自动末端负载标定。它在普通位置控制下用 6 个双向姿态拟合总质量和法兰三维重心，再用 2 个独立姿态、实际短时保持姿态 88% 容量余量和最长 1 秒高阻尼保持做验证；完整激励路径容量保留为诊断。成功后保存到 `~/.ros/elfin_freedrive_payload.yaml`。普通 FREE 在入口和运行中逐周期仍以 90% 容量硬门禁拒绝/退出。它不能识别未知末端外形，也不能把姿态相关的柔性线缆拉力精确化为固定重心，完整边界见 `docs/e05_automatic_payload_calibration.md`。

### 7.3 三个双轴模块

| EtherCAT 模块 | 关节 |
| --- | --- |
| slave1 | J2、J1 |
| slave2 | J3、J4 |
| slave3 | J5、J6 |

维护窗口保留每组模块清故障、立即抱闸和受保护松闸。松闸必须 Servo Off、无 Fault、编码器静止、勾选额定支撑确认；一次只允许一组，最多 5 秒后自动请求抱闸。单模块直接 enable、自动位置识别、无关节限制命令和力矩主题不做成人工按钮。

## 8. 控制箱 / Mini box 接口边界

厂家 S 系列说明书列出了 AC 电源、保险丝/电源开关、示教器连接器、机器人连接器，以及箱内 VGA、USB、HMI、两个外部以太网口；控制器 I/O 又分为可配置 I/O、电源、通用 I/O、远程信号、模拟 I/O 和安全信号。

本机现有 ROS 路径没有接入或发布这些控制箱接口。它们不能与末端 12 芯工具 I/O 混为一谈，也不能仅凭说明书截图在 Panel 中伪造按钮。将来启用 Mini box 时需要单独记录实际型号、端子丝印、线束和协议，再建立第二份映射。

## 9. Panel、MoveIt、RViz 各负责什么

| 组件 | 负责 | 不负责 |
| --- | --- | --- |
| Panel | Servo/Fault、低速人工点动、3 DI/3 DO、POINT 姿态管理、FREE 零力拖拽、会话日志、维护诊断 | 自动生成采摘逻辑、感知未知障碍、直接指定实体灯色 |
| MoveIt `move_group` | 运动学、规划、自碰撞和已加入场景物体的碰撞检查、轨迹执行接口 | 自动知道现实桌子/相机位置、直接采集深度图 |
| RViz | 显示模型与规划场景、拖动交互标记、Plan 预览、Execute 请求 | 机械臂底层驱动、急停、轨迹长期存储 |
| RealSense / 感知节点 | 未来提供点云、目标位姿、深度信息 | 未配置时不会自动进入 MoveIt OctoMap |

因此视觉采摘的正确后续顺序是：先标定相机与机器人坐标系，再把深度障碍加入 planning scene，最后由 MoveIt 规划并通过标准轨迹控制器执行。Panel 是人工接管和诊断入口，不应承载整套自动采摘算法。

Panel 主界面按功能内聚为四个紧凑矩形模块：六关节点动、六维末端点动、接管与 `OUTPUT_0..2`、机器人状态与 `INPUT_0..2`/POINT/FREE；底部会话日志持续追加。姿态文件管理、完整拖拽阈值与逐轴阻尼、危险抱闸操作分别放在“姿态管理”“拖拽高级”“维护”窗口，避免把所有接口横向堆进主操作区。
