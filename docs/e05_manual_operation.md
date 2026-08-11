# E05 / Elfin5 手动操作手册

这份手册用于不依赖 Codex 时的日常编译、仿真、真机操作和停机。

## 最短正确启动路径

真机日常操作只需要记住两个启动入口，分别放在两个终端中：

```text
终端 A：/home/catas/START_ELFIN_HARDWARE.sh
             ↓ 等待找到 4 个 EtherCAT 从站且发布 /joint_states
终端 B：/home/catas/START_ELFIN_PANEL.sh
             ↓ 自动启动 MoveIt、Basic API 和 Control Panel
```

只有事故记录中的复验条件全部完成并正式解除锁定后，受监督的零力拖拽验证才可把终端 A 改为 `/home/catas/START_ELFIN_HARDWARE.sh --freedrive`。这个参数只开放控制器门禁，不会自动 Servo On 或进入力矩模式。

需要 RViz 时，终端 B 改用：

```bash
/home/catas/START_ELFIN_PANEL.sh --rviz
```

不要单独运行 `roslaunch elfin_basic_api elfin_basic_api.launch`。Basic API 不是独立程序，它必须先拿到 MoveIt 的语义模型、`elfin_arm` 规划组和 planning scene 服务；单独启动正是 `Robot semantic description not found`、`Group 'elfin_arm' was not found` 和 `/get_planning_scene has not been advertised` 三类报错的原因。

## 1. 当前安全边界

> **2026-07-26 事故状态：** 新末端仍使用 `0 kg` 模型时，DI bit 5 的一次非预期高电平触发了 FREE 和下坠。从首次读高到最终读低的日志间隔为 `0.59168 秒`，但旧单线程切换阻塞了中间采样，不能把它误称为精确电气脉宽。活动事故锁已于 2026-07-27 08:27:45 按用户明确指令解除并只读归档；`--freedrive` 不再因锁文件退出，但当前 `0 kg` 正式负载仍不匹配新末端。详见 [事故记录](e05_freedrive_incident_2026-07-26.md)。

- 机械臂是 Han's Robot E05，ROS 版本为 Noetic。当前机器人接在 `eth1`，但启动脚本会按从站身份自动检测，不依赖固定网卡名。
- 电闸可以作为最终物理断电手段，但首次运动时必须由一人专门守在电闸旁，且无需进入机械臂运动范围就能断电。
- 2026-07-20 已由现场确认 PE 连续性合格、独立拉闸演练完成并清场；首次真机 Servo On 和运动成功。
- 当前内核不是 PREEMPT_RT。硬件入口已用有限 cgroup 预算恢复 RR 10；10 秒、1 ms 周期测试为最小 `1 us`、平均 `12 us`、最大 `138 us`。这只是首次低速空载测试依据，不代表高负载实时认证。
- 每次实验仍需先确认 PE、断电人员和清场状态，结束时先 Servo Off 再关闭上游电源。

## 2. 六个关节的名字

关节是两个连杆之间的旋转轴，不是整段白色外壳。

| ROS 名称 | 常用物理名称 | 所在位置与作用 |
| --- | --- | --- |
| `elfin_joint1` | J1 / 基座 / 腰部轴 | 最底部大关节，绕竖直轴旋转整条机械臂 |
| `elfin_joint2` | J2 / 肩关节 | 基座上方的俯仰轴，抬起或放下大臂 |
| `elfin_joint3` | J3 / 肘关节 | 中部折叠轴，改变机械臂伸展程度 |
| `elfin_joint4` | J4 / 腕部第一轴 / 前臂旋转 | 肘后方的旋转轴，主要改变腕部滚转方向 |
| `elfin_joint5` | J5 / 腕部俯仰轴 | 靠近末端的弯腕轴，改变法兰朝向 |
| `elfin_joint6` | J6 / 工具法兰旋转轴 | 最末端旋转轴，绕工具中心轴转动法兰 |

本机标定首次只读验证时为 J1 约 `-20.27°`、J2--J6 接近 `0°`。2026-07-20 吊装恢复后曾读到 J1 约 `10.49°`、J2 约 `-0.46°`。这些只是历史验收记录，不是每次开机的目标；当前姿态始终以实时 `/joint_states` 和现场目视为准。

## 3. 每个新终端的 ROS 环境

按 `Ctrl+Alt+T` 打开终端。每个新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/catas/ros_ws/devel/setup.bash
```

命令运行后终端需要保持打开。`Ctrl+C` 用于关闭该终端中启动的 ROS 程序。

## 4. 软件停机怎么运行

不要依赖文件管理器双击。首次真机实验前，在独立终端中先输入但不要按回车：

```bash
/home/catas/STOP_ELFIN.sh
```

需要停机时按回车。也可以显式使用 Bash：

```bash
bash /home/catas/STOP_ELFIN.sh
```

正常结果应包含：

```text
Servo Off confirmed by the driver.
Elfin hardware driver stopped.
```

如果显示硬件驱动仍在运行，立即拉电闸。若机械臂本来就没有启动，出现 `Unable to communicate with master` 后又显示驱动已停止是正常的重复停机结果。

## 5. 一键离线自检

下面的脚本不会打开 EtherCAT，也不会控制真机：

```bash
/home/catas/TEST_ELFIN_OFFLINE.sh
```

它会编译完整 catkin 工作区，运行 freedrive 的按钮、重力模型、负载回归与安全数学单元测试，解析 E05 URDF，检查 Python 语法，并解析模型、硬件、Gazebo、MoveIt、Basic API 和 freedrive 的关键 launch 文件。最后出现 `PASS` 才算通过。

它不能证明电机、EtherCAT、末端 I/O 或真实运动正常。

## 6. 手动运行完整仿真

仿真不会连接 EtherCAT 网口或真机。现在只需一个终端：

```bash
/home/catas/START_ELFIN_FREEDRIVE_SIM.sh
```

它会在回环地址的隔离 ROS/Gazebo 端口一次启动力矩仿真、MoveIt、Basic API 和 Panel；默认不打开 Gazebo/RViz 3D 窗口，以避开 Jetson 之前的 NVIDIA 图形崩溃。需要图形时选择：

```bash
/home/catas/START_ELFIN_FREEDRIVE_SIM.sh --gazebo-gui
/home/catas/START_ELFIN_FREEDRIVE_SIM.sh --rviz
/home/catas/START_ELFIN_FREEDRIVE_SIM.sh --rviz-egl
```

关闭时在这个终端按一次 `Ctrl+C`，等待所有节点显示 `done`。不要使用 `gazebo --version` 检查版本；这台机器上的该命令会意外留下 Gazebo 后台进程。

在 Panel 中点“进入零力拖拽”后状态应变成 `ACTIVE`；点“退出并保持当前位置”后应回到 `READY`。专用仿真已经配置 effort 位置控制 PID，不再使用普通位置接口仿真那套“缺少 PID”结论。

无界面模式下 `gzserver` 会与桌面 `DISPLAY` 隔离并使用软件渲染，避免 Jetson 上已经复现的 `GLXBadDrawable`。Gazebo 在非零重力姿态恢复 stock effort 位置控制器时可能出现有限 PID 静态偏差；它不等于真机 CSP 保持误差，也不能用来调真机重力倍率。

## 7. 手动进行真机只读诊断

本节不会 Servo On，但仍会打开 EtherCAT 并将 1--3 号从站切入 OP。只在机械臂供电、底座固定、人员远离且电闸有人值守时执行。

终端 A：自动识别机器人网口，以 RR 10 启动模型和 Servo-Off 硬件驱动。

```bash
/home/catas/START_ELFIN_HARDWARE.sh
```

终端 B：读取六轴、使能和故障状态。

```bash
source /opt/ros/noetic/setup.bash
source /home/catas/ros_ws/devel/setup.bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /elfin_ros_control/elfin/enable_state
rostopic echo -n 1 /elfin_ros_control/elfin/fault_state
```

结束时先运行：

```bash
/home/catas/STOP_ELFIN.sh
```

然后在终端 A 中按 `Ctrl+C`。

## 8. MoveIt 和 Control Panel 如何手动控制真机

### 8.1 每次都按这个顺序启动

开始前确认：机械臂空载或负载已固定、底座固定、工作范围和夹点清空、PE 正常，另有一人只负责上游电闸。先在停机终端输入下面这行但暂不回车：

```bash
/home/catas/STOP_ELFIN.sh
```

终端 A 启动 Servo-Off 真机栈：

```bash
/home/catas/START_ELFIN_HARDWARE.sh
```

日常点动保持上面命令不变；只有按 8.3 节做零力拖拽验证时改成：

```bash
/home/catas/START_ELFIN_HARDWARE.sh --freedrive
```

必须看到以下关键结果，且这个终端保持打开：

```text
Verified E05 EtherCAT interface: eth1
SOEM found and configured 4 slaves
Started ['joint_state_controller'] successfully
```

终端 B 启动完整控制界面：

```bash
/home/catas/START_ELFIN_PANEL.sh
```

它会先自动清除 rosmaster 中所有已无法响应的死亡节点注册，再检查机器人模型、新鲜的 `/joint_states`、仍真实存活的重复节点和真机 Servo 状态，然后一次启动 MoveIt、Basic API 和 Panel。硬件入口也会在已有 ROS master 时做同样清理。只有同名节点仍能响应健康检查时才会拒绝重复启动。必须等到：

```text
Ready to take commands for planning group elfin_arm.
```

如果还要 RViz，关闭终端 B 后重新使用：

```bash
/home/catas/START_ELFIN_PANEL.sh --rviz
```

首次 Servo On 前在第三个终端核对：

```bash
source /opt/ros/noetic/setup.bash
source /home/catas/ros_ws/devel/setup.bash
rostopic echo -n 1 /elfin_ros_control/elfin/enable_state
rostopic echo -n 1 /elfin_ros_control/elfin/fault_state
rostopic echo -n 1 /joint_states
```

正常起点是 `enable_state=False`、`fault_state=False`，并且六轴读数与实体姿态一致。Panel 速度先保持 `1%`，Servo On 后先短按单个关节的正确退离方向；不要用 Home 作为首次试动。

### 8.2 Panel 按键

Panel 速度默认 1%、可调到 100%。100% 对应关节点动基准速度约 `0.78 rad/s`（约 `44.7 deg/s`）。关节和笛卡尔按钮是持续点动：按下开始，松开调用 Stop。

| 控件 | 实际作用 |
| --- | --- |
| `伺服上电 / Servo On` | 先检查静止和命令位置对齐，再使能六轴并启动 `elfin_arm_controller` |
| `伺服关闭 / Servo Off` | 取消当前轨迹、关闭六轴伺服并停止运动控制器 |
| `清除故障 / Clear Fault` | 在 Panel 显示 Fault 且机械接触/供电原因已处理后，清一次驱动故障；不会自动解决碰撞 |
| `回 ROS 零位 / Home` | 六轴按比例同时运动到固定的 ROS 零位；松开会停止 |
| `停止轨迹 / Stop` | 取消当前 Panel/轨迹 action，但不关闭伺服，也不是物理急停 |
| `J1..J6 +/-` | 对应关节朝正/负限位持续运动，松开停止 |
| `X/Y/Z +/-` | 末端在 `Ref. link` 坐标系中沿三轴平移 |
| `Rx/Ry/Rz +/-` | 末端绕 `Ref. link` 坐标系三轴持续旋转；按住继续、松开停止，直到 IK、碰撞检查或真实关节限位不再允许该方向；显示栏的 `R/P/Y` 是当前姿态角 |
| `点动速度倍率` | 调整 Panel/Basic API 产生的轨迹速度倍率，不控制 RViz 内单独的 MoveIt 速度倍率 |
| `DO0..DO2` | 分别对应实体 `OUTPUT_0..2`；读取原寄存器、只切换选中位、写后回读 |
| `DI0..DI2` | 分别只读显示实体 `INPUT_0..2`；浮空输入应为 OFF；首次读取只建立基线，之后 ON/OFF 变化会在会话日志追加一条“结果：完成”记录 |
| `POINT（DI bit 4）` | 按下沿把时间与 J1..J6 弧度持久写入 `~/.ros/elfin_freedrive_points.yaml`；不发送轨迹 |
| `FREE（DI bit 5）` | 从首次读高起至少 0.70 秒且至少 8 个 10 Hz 高电平样本后才请求切换；一次孤立低电平毛刺会被过滤，持续低电平会清空候选；确认松开后按 1 秒保护退出；绝不松抱闸 |
| `进入零力拖拽` | 与实体 FREE 共用同一管理器和所有门禁；事故锁已解除，但当前负载模型、六轴误差、容量、Servo/Fault 等检查仍可拒绝请求 |
| `退出并保持当前位置` | 与松开实体 FREE 相同；无法恢复位置控制时执行保护回退，真机最终请求 Servo Off |
| `FREE 上限` | 稳定拖拽时可在 50%--300% 间缩放软/硬速度阈值；接管和退出阶段大于 100% 的部分不生效 |
| `拖拽高级` | 查看六轴实际速度阈值、重力预检、CSV 和当前负载；J1--J6 阻尼可调，并提供高位一键未知末端标定 |
| `姿态管理` | 逐条查看、记录、删除 POINT 姿态，并直接显示管理器实际使用的 YAML 路径 |
| `设置坐标系` | 修改笛卡尔点动的参考坐标系和末端坐标系；普通使用保持默认值 |
| `维护与接口诊断` | 打开受保护抱闸控制和只读 ROS/驱动诊断；不是普通运动区 |
| `会话事件日志` | 按时间持续追加动作、结果、状态、Fault 和 FREE 退出原因；新内容不会覆盖旧内容 |

主界面沿用原始 GitHub Panel 的固定密度：六关节、六维末端、接管与输出、状态与输入分别形成紧凑矩形模块，常用操作在 `1366 x 768` 内无需滚动。可见文字只保留动作和状态，完整用途放在鼠标悬停提示中；危险抱闸、姿态管理和高级诊断分别进入独立窗口。

POINT 默认保存在 `/home/catas/.ros/elfin_freedrive_points.yaml`；若 launch 覆盖了路径，“姿态管理”顶部会显示真实文件位置。它通过 `/elfin_freedrive_manager/list_recorded_points` 读取全部记录，通过 `/elfin_freedrive_manager/delete_recorded_point` 原子删除选中记录并连续重排序号。

本机固定映射为 `DI bit 4 = POINT`、`DI bit 5 = FREE`，它们与外接端子 `INPUT_0..2` 分开。POINT 每次按下持久记录一次；FREE 必须从首次读高起同时满足 0.70 秒和 8 次连续采样才请求进入，任一条件不足都不会切换，松开请求保护退出。管理器启动时若 FREE 已按住也不会误触发，必须先释放再重新按下；已确认 FREE 后若 I/O 读取丢失，也会退出并等待低电平重新武装。拖拽始终保持 Servo On 和抱闸正常工作，不会调用维护区的松抱闸服务。完整设计见 `elfin_freedrive_controller/README.md`。

`维护与接口诊断` 会打开可缩放的维护窗口：

- 自动列出当前所有在线 Elfin 服务和主题。
- 只读显示运动状态、位置对齐、编码器计数、关节模块 TxPDO/RxPDO 和末端 I/O TxPDO/RxPDO。
- 提供 slave1（J2/J1）、slave2（J3/J4）、slave3（J5/J6）各自的模块清错、抱闸闭合和受保护释放。
- 释放前必须勾选额定支撑/清场确认，并同时通过 Servo Off、无 Fault、编码器静止检查；每次只允许一个双轴模块释放，5 秒后自动请求闭合，`Close now` 可提前闭合。
- 原始单模块 enable、自动位置识别、无关节限位调试主题和力矩接口明确不做成按钮，因为它们会绕过高层一致性检查或可能重新标定/驱动机器人。
- AI0/AI1 与 Smart Camera 字段虽出现在末端源码名称中，但当前驱动既未解码也未发布它们；RS-485 也没有协议节点。原始 slave4 PDO 查询还是全零占位实现，维护区不会显示伪造读数。
- 若在某组抱闸的 5 秒释放周期内关闭主窗口，Panel 会先暂停退出、要求该后台周期发送闭合请求，完成后才继续关闭。

通知窗口和“设置坐标系”窗口由内容布局计算高度；主界面和维护窗口均可缩放，长文字会换行，结果框可滚动。完整接口审计见 `docs/e05_interface_inventory.md`。

`Home` 不会把每次开机姿态重新定义为零。源码明确把目标设为六个 `0 rad`，零位来自 `elfin_drivers.yaml` 的固定 `count_zeros`，且本机 `automatic_recognition: false`。重启时刷新的是“当前位置保持命令”，目的是 Servo On 时不跳动；它不是 Home。MoveIt 的 Start State 默认取开机后的当前位置，也容易让人误以为 Home 被刷新。

当前 MoveIt 场景尚未包含实验桌、墙面、相机支架等真实环境几何体。它能检查机器人自身和已加入 planning scene 的物体，但不会凭空知道桌子在哪里；Panel 的关节点动也不能替代现场观察。获得桌面尺寸和相对机器人基座的位姿后，应把桌子加入碰撞场景，再依赖 MoveIt 做环境避碰。

### 8.3 真机零力拖拽复验（事故锁已解除，仍需受支持负载）

2026-07-24 的空载静态 CST 和 2026-07-25 的人工拖动记录只证明旧负载、旧时序下的历史结果。活动事故锁已按用户指令解除，但当前正式负载仍为 `0 kg`。本次新末端候选已通过离线测量和容量复算，仍须先在 Panel 完成最多 1 秒的受监督保持验证并看到正式配置持久化，之后才执行日常人工拖拽。

某一新负载第一次人手试拖时，姿态必须远离桌面、自碰撞和六轴限位；使用能承重但不主动牵拉的额定防坠支撑，清空扫掠区和夹点，另有一人只守上游电闸。

1. 终端 A 用 `/home/catas/START_ELFIN_HARDWARE.sh --freedrive` 启动，确认 4 个从站和 `Freedrive manager gate: UNLOCKED`。
2. 终端 B 启动 `/home/catas/START_ELFIN_PANEL.sh`，保持点动速度 1%，正常 Servo On。
3. 保持机械臂静止且手先离开，查看 Panel“重力模型预检”。绿色为严格通过；黄色表示已标定模型在当前单姿态存在残差，但方向、比例、稳定性和力矩容量仍可接受；红色未通过不能绕过。
4. 一只手在远离夹点、容易控制的连杆位置轻扶，点击“进入零力拖拽”或持续按住实体 FREE。实体按钮从首次读高起连续确认至少 0.70 秒并取得至少 8 个样本后才请求，实际按住通常约 0.70--0.80 秒；切换后的 0.5 秒重力接管期间不要预先蓄力猛拉。
5. 状态为 `ACTIVE` 后只轻推几毫米/几度，松手确认没有持续自行加速；随后退出并确认 `READY` 和当前位置保持。
6. 读取 `/elfin_freedrive_controller/telemetry`、`/joint_states`、Fault、Panel 详情和最新 CSV。任何主动下坠、抬升、抖动、Fault、Servo Off 或通信过期都立即拉闸并停止验证。
7. 小范围手推通过后，才逐步扩大拖动范围，最后验证实体 FREE 松开和 POINT 记录。

当前 FREE 直接采用 E05 URDF 六轴额定力矩 `[420, 420, 200, 200, 69, 69] Nm` 作为输出上限，不再叠加 20% 或 `[15, 84, 36, 15, 10, 8] Nm` 人工逐轴帽。未验证负载达到额定值 90% 时拒绝/退出，已验证负载使用 92% 迟滞线；力矩变化率、速度、关节限位、Fault 和退出恢复保护保持不变。安装夹爪、相机、转接板或其他负载后，空载标定仍不足以证明重力模型正确。

Panel 的“FREE 上限”可在 `50%--300%` 调整稳定拖拽阶段的软减速和硬速度阈值；接管和退出阶段始终最多采用 100% 保护。“拖拽高级”可把 J1--J6 阻尼分别设为 `5%--500%`。两者都只能在退出零力拖拽后应用，下一次进入生效。降低阻尼时一次只改一个轴，每次最多下降 10%--20%，先做几毫米试拖；阻尼不负责承重，重力错误应重新标定工具负载。

### 8.4 更换末端后自动标定负载

更换夹爪、剪刀、相机、转接件、末端电机或线缆布置后，不能继续把空载模型当成正确重力。事故锁已经解除，但不会把不合格候选变成正式模型；只有负载质量、刚性模型残差、实际短时保持姿态 88% 力矩余量和最多 1 秒保持满足要求时，才可使用 `--freedrive`。完整标定前应先用普通位置点动把法兰移到 `elfin_base` 的 `z >= 0.65 m`，再进入 Panel“拖拽高级”执行标定。

2026-08-11 最新 `7/7` 样本 `auto-payload-20260811-071430` 得到 `1.563 kg`、重心 `[0.125, -0.012, 0.409] m`、半径 `0.428 m`，拟合/留出 RMSE 为 `1.43/1.69 Nm`，测量质量拒绝为空。旧 J5 `10 Nm` 人工帽把保持姿态 `9.411 Nm` 和路径峰值 `9.468 Nm` 错误显示为 `94.1%/94.7%`；按 J5 URDF `69 Nm` 额定值重新解释后仅为 `13.64%/13.72%`。此前的反复失败是历史软件门禁导致的控制拒绝，不是测量、Home 起点或负载额定能力失败。

Panel 会先检查 Servo/Fault 状态是否在 0.75 秒内更新、FREE 是否为 `READY` 和当前法兰高度，再显示清场确认。完整标定会用普通位置控制完成高位双向采样；已有完整且末端未改变的样本可点击“复用最近样本，仅做短时验证”，避免重复 32 段运动。复用时若当前低于 `0.65 m`，脚本会先以 3% 经 MoveIt 抬到高位 H，轨迹不得比当前高度再下降超过 `15 mm`，然后按实际保持姿态重新检查容量。两条路径都只有在拟合、留出姿态、保持姿态容量和默认 0.8 秒且绝不超过 1 秒的零力保持全部通过后才持久化；中止或失败会请求恢复位置保持、旧负载和原阻尼。

总负载必须不超过 E05 额定 5 kg，且未知实体在整段路径至少有 0.40 m 现场余量。MoveIt 看不到未建模夹爪、线缆和人员，柔性线缆也不一定能等效为固定重心，因此不能把“一键”理解成“任意未知物体绝对安全”。第一次操作步骤、输出文件和全部拒绝条件见 `docs/e05_automatic_payload_calibration.md`。

## 9. 碰撞或 Servo On 立即 Fault 时怎么恢复

不要连续点击 Servo On，也不要自动/徒手释放抱闸。驱动 Fault 时 Servo On 会失败是保护结果；自由落体后再锁闸会产生不可控方向和冲击，并且本机一个 `open_brake_slaveX` 会同时释放该模块的两个关节。

只要 ROS 和 EtherCAT 仍在线，立即运行：

```bash
/home/catas/RECOVER_ELFIN_COLLISION.sh
```

脚本会按顺序执行：

1. 保存碰撞前的 Enable、Fault、关节位置、运动/对齐状态、原始 TxPDO 和控制器状态。
2. 取消轨迹与 Panel 点动。
3. 请求全轴 Servo Off，并停止 `elfin_arm_controller`。
4. 再次请求 Servo Off，确认 `enable_state=False`。
5. 保持所有抱闸闭合，不 Servo On、不清 Fault、不发送回退轨迹。

诊断记录保存在：

```text
/home/catas/elfin_recovery_reports/
```

如果脚本不能确认 Servo Off，立即拉上游电闸。如果机械臂仍压着桌子，先移走可移动的障碍；无法移走时必须像本次一样用额定吊具承重、约束可能扫动的关节并清空夹点，不能把“自动开全抱闸”写成日常恢复程序。

机械接触已经解除或机械臂已被额定支撑后，可以只清一次 Fault：

```bash
/home/catas/RECOVER_ELFIN_COLLISION.sh --clear-fault
```

这个选项仍然不会 Servo On 或运动。只有报告末尾同时满足 `enable_state=False`、`fault_state=False`、`get_motion_state success=False`（表示没有运动）、`get_pos_align_state success=True`，才回到 Panel，以 `1%` 短按正确的单关节退离方向。再次出现 Fault 就停止重试，检查 48 V 母线、公共安全/使能条件、抱闸供电和报告中的驱动状态。

2026-07-20 的一次实测故障为三个双轴模块在 Servo On 的 phase 3 同时失败，六轴状态字均为 `0x0028`、错误码均为 `0x0002`。这证明是底层驱动 Fault，不是 MoveIt 的软件锁；本地资料尚未给出 `0x0002` 的厂商定义，因此文档不把它武断解释成某一种故障。

## 10. 常见启动信息与截图报错

| 屏幕信息 | 含义与处理 |
| --- | --- |
| `Robot semantic description not found` | 没有加载 MoveIt 的 SRDF。按 `Ctrl+C` 关闭错误启动，改用 `START_ELFIN_PANEL.sh`。 |
| `Group 'elfin_arm' was not found` | 同一缺依赖问题，不是机械臂型号消失。不要单独启动 Basic API。 |
| `/get_planning_scene has not been advertised` | `move_group` 没运行；新 Panel 启动脚本会一并启动它。 |
| `No ROS master is running` | 真机先启动 `START_ELFIN_HARDWARE.sh`；仿真先启动 Gazebo。 |
| `No fresh /joint_states` | 状态源尚未就绪。查看终端 A 是否找到 4 个从站，不能直接 Servo On。 |
| `Another Elfin hardware launch is already running` | 启动脚本已自动清理死亡 ROS 注册；此信息表示仍有真实进程持有 EtherCAT 独占锁。回到原终端结束它，不要启动第二个主站或运行 `slaveinfo`。 |
| `commands aren't aligned with actual positions` | 不得绕过。先 Servo Off，等待当前位置跟随，再运行碰撞恢复脚本保存状态。 |
| `setEnable phase3 ... failed` / `0x0028` | 驱动未进入 Operation Enabled；停止重试，按第 9 节处理机械负载、供电和 Fault。 |
| `ABORTED: CONTROL_FAILED` | 轨迹控制器、Servo 或底层 Fault 导致执行失败；不是“规划成功就一定能动”。 |
| `IK plugin ... deprecated API` | 兼容性警告，当前 IKFast 仍可用，不是本次启动失败。 |
| `No 3D sensor plugin(s) defined for octomap updates` | 尚未接入深度相机到 MoveIt OctoMap；不妨碍无传感器规划，但也不会自动感知桌子。 |
| RViz 在 `Constructing new MoveGroup connection` 后 `SIGSEGV`（常见为 `-11`，随后可能被记录为 `-9`） | 这是旧 `MotionPlanning` 插件在当前 Noetic/图形环境中的兼容性崩溃，不是 `solve_type` 或 IK 警告。默认 `--rviz` 和柑橘视觉配置已移除该插件；不要切换回 `--rviz-original`。只有纯图形初始化失败且日志没有 MoveGroup 连接时，才尝试 `--rviz-egl`。 |

## 11. 灯环与末端 I/O

本机实体末端只有 `INPUT_0..2`、`OUTPUT_0..2`、`485_A/B`、`AI0/AI1`、`24V` 和 `GND`。新版 Panel 因而只显示三路 DI 和三路 DO，不再显示来源不明的第四路和 `LED0..3`。

| 实体端子 | 电气规格 | 当前 ROS 状态 |
| --- | --- | --- |
| `INPUT_0..2` | PNP；11-30 V 为 ON，浮空为 OFF | `read_di` 可读，Panel 只读显示 DI0..2 |
| `OUTPUT_0..2` | 最大 0.4 A；手册 PNP 表格与“导通 GND”文字矛盾 | `read_do/write_do` 可用；Panel 写单个位并回读 |
| `AI0/AI1` | 0-10 V、约 15 kOhm、非差分共地 | 驱动未解码/发布，不在 Panel 伪造数值 |
| `485_A/B` | RS-485 物理线 | 当前没有波特率、帧格式或 ROS 协议节点 |
| `24V/GND` | 24 V，典型 1 A、最大 1.5 A | 工具电源，不是 ROS 数据接口 |

驱动运行时，可以只读当前数字输入/输出原始值：

```bash
rosservice call /elfin_ros_control/elfin/io_port1/read_di "data: true"
rosservice call /elfin_ros_control/elfin/io_port1/read_do "data: true"
```

`read_di` 优先从 slave4 的周期输入 PDO 读取 16 位输入字，从而避免 10 Hz 末端按键轮询持续抢占 mailbox；仅在旧 ESI 没有输入 PDO 映射时才回退读取 `0x6001:01` SDO。PDO/SDO 传输失败或返回长度不足时，ROS 服务直接失败，不会把失败伪装成 `0`；FREE 管理器会退出已确认拖拽、清空未确认候选，并要求稳定低电平重新解锁。`read_do/write_do` 在 ROS 消息中使用 bit 12..14 表示三路实体输出。

灯环由机器人状态自动控制，不是三路 DO，也没有 ROS 颜色设置服务。厂家定义为：绿=上电使能/自由驱动、红=异常去使能、白=负载识别、蓝=零力示教、黄=上电去使能、紫=程序自动运行。灯不亮应先根据这些状态和厂商控制链排查，不能再点击虚构的 LED 位。

数字输出手册存在电流方向矛盾。在限流台架验证前，不要直接接夹爪、电磁阀或感性负载。完整 Pin 表和源码缺口见 `docs/e05_interface_inventory.md`。

## 12. MoveIt/RViz 鼠标规划与姿态记录

`START_ELFIN_PANEL.sh --rviz` 现在默认加载低负载只读配置：帧率 10 FPS，关闭未使用的 `/joy_pose` 显示，并保留机器人模型、规划轨迹、语义 MarkerArray 和环境点云。它不加载 `moveit_rviz_plugin/MotionPlanning`，因此不会在 RViz 内部再次创建 MoveGroup 客户端；当前 Noetic 环境中，该插件在建立 `elfin_arm` 连接后可能触发 `SIGSEGV`。MoveIt 的规划器、轨迹执行节点和 Panel/柑橘视觉终端不受影响，规划与执行请通过这些终端完成。

如果终端 B 没有使用 `--rviz`，硬件和 `move_group` 已运行时可以单独打开同一低负载 RViz：

```bash
source /opt/ros/noetic/setup.bash
source /home/catas/ros_ws/devel/setup.bash
roslaunch elfin5_moveit_config moveit_rviz.launch config:=true
```

如果再次出现 NVIDIA GLX 路径崩溃，先关闭原 Panel 启动终端，再使用 Qt EGL 备用路径：

```bash
/home/catas/START_ELFIN_PANEL.sh --rviz-egl
```

原始较重配置仍仅用于定位兼容性问题，不作为日常或实机默认；它保留 `MotionPlanning`，所以可能重新触发上述崩溃：

```bash
/home/catas/START_ELFIN_PANEL.sh --rviz-original
```

只有在明确启动 `--rviz-original` 且确认该插件稳定时，右侧才会出现 `MotionPlanning -> Planning`。此时可将 Planning Group 选为 `elfin_arm`，先把 Start State 设为 `current` 并点 `Update`；拖动末端标记或使用 `Joints` 页可生成目标。`Plan` 只计算和动画预览，`Execute` 才会请求真机执行，`Plan & Execute` 会连续完成两步。日常使用优先通过 Panel/柑橘视觉终端的规划服务；任何真实执行仍须满足现有执行门禁和现场确认。

系统自带 MoveIt Commander，可记录关键姿态并在重启后重新规划播放：

```bash
source /opt/ros/noetic/setup.bash
source /home/catas/ros_ws/devel/setup.bash
rosrun moveit_commander moveit_commander_cmdline.py elfin_arm
```

在出现 `elfin_arm>` 后使用：

```text
rec pose1
rec pose2
show
save /home/catas/elfin_positions.cmd
load /home/catas/elfin_positions.cmd
plan pose1
execute
go pose2
stop
```

先用 Panel 或 RViz 移到一个姿态，再输入 `rec pose1`；移动到下一姿态后输入 `rec pose2`。`plan + execute` 分两步预览和执行，`go` 是直接规划并执行。保存的是关键姿态，不是旧时间戳下的原始电机数据；回放时 MoveIt 会从当前状态重新规划，更适合环境会变化的视觉抓取实验。

`rosbag record /joint_states` 可以记录实际关节时间序列用于分析，但直接 `rosbag play` 只会重发状态消息，不能也不应直接驱动电机。

## 13. 已完成的源码验收

- 完整 catkin 工作区全量编译通过。
- E05 Xacro 展开和 URDF 结构检查通过。
- 所有 Python 文件语法检查通过。
- 模型、硬件、Gazebo、MoveIt、Demo 和 Basic API launch 解析通过。
- Gazebo 模型、两个控制器、MoveIt、Basic API 和 Control Panel 启动通过。
- 仿真 J1 `0 -> 0.10 rad -> 0`，两次 MoveIt 执行均返回 `SUCCEEDED`。
- 真机 EtherCAT 找到 4 个从站；Servo Off、无故障、静止、位置对齐和停机退出均已验证。
- freedrive 新包测试覆盖按钮状态机、平滑接管、非有限输入、反向力矩、比例/残差拒绝、双向摩擦中心拟合、末端负载回归和重力容量余量；其余旧包仍缺少系统化单元测试，不能把编译和冒烟通过理解成所有边界行为都已证明。
- 真机 Servo On、六轴 Panel 点动和 Servo Off 已成功。2026-07-20 曾对历史 `LED0` 位做寄存器写入/回读且未见灯光；本次电气审计证明它不能当作 E05 可控灯环接口，已从 Panel 删除。
- MoveIt RViz 已连接真机，MoveIt Commander 的当前姿态记录、显示和文件保存已通过。
- freedrive 旧版仿真曾验证 `READY -> ACTIVE -> READY`、六轴控制器互斥、3 秒平滑接管、J2 外力推动约 `0.18 rad`、退出后位置控制器恢复，以及 8 秒超时分支的原子回切。当前 0.5 秒接管、固定标定模型和 Panel 参数服务已经编译及单元测试通过，仍需补充修复后的真机小范围复验。
- 旧真机失败日志已定位到硬件写周期使用反馈 `axis.effort` 而非命令 `axis.effort_cmd`，以及运动中切回位置控制失败。写命令、CST/CSP 交接同步、恢复状态机和切换回调超时窗口均已修复。
- 2026-07-24 空载真机已完成 17 样本/6 双向姿态标定；Home 1 秒、中负载 1 秒和中负载 4 秒静态 CST 均 `ACTIVE -> READY`、无 Fault。4 秒完整交接最大位移 `1.1e-5 rad`。2026-07-25 已完成人工拖动并定位入口预载误学问题；关闭该自适应后的版本仍待小范围复验，带载始终未验收。
- 灯环只实现厂家状态语义的 Panel 显示与 ROS 话题；没有证据表明当前驱动能主动指定实体颜色，真机进入 CST 后是否由固件自动变蓝也尚未验证。
- `Rx/Ry/Rz` 原有的 `100 * 0.02 rad = 2 rad` 固定轨迹上限已移除。2026-07-20 在 Servo Off、运动控制器 stopped 时验证 `Rx+` 生成 `126` 个轨迹点（约 `2.52 rad`）后由当前姿态下的 IK/关节连续性约束截停；调用前后关节位置一致、Servo 仍为 Off。
- 新增的 `START_ELFIN_PANEL.sh` 会阻止重复节点和缺少模型/关节状态时的错误启动；`RECOVER_ELFIN_COLLISION.sh` 的静态检查明确不包含任何开抱闸、Servo On 或运动命令。
