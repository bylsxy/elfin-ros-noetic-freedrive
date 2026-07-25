# E05 与 Jetson 首次恢复记录（2026-07-20）

脱离 Codex 的日常命令见 `docs/e05_manual_operation.md`。

## 设备与连接

- 机械臂：Han's Robot E05（设备资产标签与序列号不进入公开仓库）
- 生产日期：2024-12-23
- 铭牌参数：DC 48 V / 3.8 A（满载）、额定负载 5 kg、工作范围 800 mm
- 主机：NVIDIA Jetson Orin NX 16GB，Ubuntu 20.04 / ROS Noetic
- 连接：机械臂底座 RJ45 当前直接连接 Jetson `eth1`；软件按 EtherCAT 从站身份自动检测接口
- 链路：100 Mb/s、全双工、carrier up；EtherCAT 使用 100BASE-TX，此速率正常

## EtherCAT 被动枚举结果

执行：

```bash
sudo /opt/ros/noetic/bin/slaveinfo eth1
```

结果：找到 4 个从站，Calculated workcounter 为 12，全部处于 SAFE-OP（State 4）。

1. `Hans Robot`，Vendor `0x0000001a`，Product `0x50440200`，Revision `0x05132016`
2. `Hans Robot`，Vendor `0x0000001a`，Product `0x50440200`，Revision `0x05132016`
3. `Hans Robot`，Vendor `0x0000001a`，Product `0x50440200`，Revision `0x05132016`
4. `F2838x CPU1 EtherCAT Slave`，Vendor `0x00201911`，Product `0x10003201`，Revision `0x00000001`

这与当前 `noetic_ethercat` 分支默认的 3 个双轴关节从站和第 4 个末端 I/O 从站相符。本次没有发送 Servo On、控制字或轨迹。

## 本机标定与配置

配置文件：`elfin_robot_bringup/config/elfin_drivers.yaml`

- 网卡：配置默认值为空；`START_ELFIN_HARDWARE.sh` 启动时实测自动选择 `eth1`
- 从站：`[1, 2, 3]`
- 六轴零位（按驱动顺序 J2、J1、J3、J4、J5、J6）：
  `[238934, 83244, 223713, -436541, -261579, 486066]`
- E05/Elfin5 力矩换算参数：
  `[2536.224, 2536.224, 5251.283, 5251.283, 15975.05, 15975.05]`
- `automatic_recognition: false`，首次接管时禁止自动位置识别

零位参数属于该设备的重要本地资产，未经可靠来源核对不得修改。

## 已验证的软件部分

以下纯模型启动成功，12 秒后由 `timeout` 正常终止：

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/ros_ws/devel/setup.bash
roslaunch elfin_robot_bringup elfin5_bringup.launch
```

成功加载 `robot_description` 和 `robot_state_publisher`。没有启动 `elfin_ros_control.launch`。

安全初始化补丁已使用以下命令单包编译通过：

```bash
cd /home/jetson/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg elfin_ros_control
```

## 真机使能条件与验证结果

1. 2026-07-20 现场确认 PE 连续性合格、独立拉闸演练完成并清场。
2. 当前 Jetson 内核不是 PREEMPT_RT。现已通过专用、有限预算的 cgroup 获得 RR 10；10 秒 `cyclictest` 在 1 ms 周期下测得最小 `1 us`、平均 `12 us`、最大 `138 us`。该短测只支持首次低速空载验证，不批准连续轨迹或负载测试。
3. 本机六轴零位来自已有未提交配置；2026-07-20 已由现场确认只读姿态与 J1 约 `-20.27°`、J2--J6 接近 `0°` 一致。
4. 首次运动前必须准备并验证一个独立于运动程序的可靠停机手段。ROS 的取消轨迹、停止控制器或 Servo Off 服务属于软件控制，不等同于独立安全回路。
5. 首次接管按“状态读取 → 姿态核对 → 故障/使能核对 → 1% 空载运动 → Servo Off”完成，真实运动成功。

`chrt` 原先失败的根因已经定位：cgroup v1 的 `user.slice` 和 `system.slice` 实时预算均为 0。硬件启动脚本现在只为本次 Elfin 进程树临时建立 50% 上限的实时 cgroup，以 RR 10 运行，退出后清理；没有改动全局调度预算。

## 当前停机/安全状态

- 末端未接设备、未接末端供电、无负载。
- EtherCAT 硬件驱动做过两次短时 Servo Off 状态读取，目前已退出；停机后四个从站为 SAFE-OP。
- 已成功完成 Servo On、Panel 六轴运动、Stop/松开停止和 Servo Off；当前每次实验仍从低速度开始。

## 无运动硬件接管实测

按正确顺序先启动模型、再启动硬件：

```bash
# 终端 A；自动检测接口，加载模型并以 Servo Off 启动 RR 10 硬件栈
/home/jetson/START_ELFIN_HARDWARE.sh
```

实测结果：

- SOEM 找到 4 个从站，IOMap 为 458 bytes，并完成配置。
- `/elfin_ros_control/elfin/enable_state`：`False`。
- `/elfin_ros_control/elfin/fault_state`：`False`。
- `/elfin_ros_control/elfin/get_motion_state`：`False`，机械臂静止。
- `/elfin_ros_control/elfin/get_pos_align_state`：`True`，命令位置与实际位置对齐。
- `joint_state_controller`：`running`，只发布状态。
- `elfin_arm_controller`：`initialized`，未启动，不能执行轨迹。
- 六轴读数（rad）：`[-0.3538066, 0.0000413, 0.0000323, -0.0001552, 0.0000323, 0.0000873]`。
- 换算后约为：J1 `-20.27°`，J2--J6 接近 `0°`；现场已确认与实体姿态一致。

若只启动 `elfin_ros_control.launch` 而没有先加载 E05 模型，驱动和状态读取仍能运行，但 `elfin_arm_controller` 会因缺少 `robot_description` 加载失败。这种启动顺序不能用于后续运动。

## 当前软件停机手段

在独立终端中预先输入下面的命令但先不要按回车；需要停机时立即按回车：

```bash
/home/jetson/STOP_ELFIN.sh
```

脚本按以下顺序处理：

1. 调用 `/elfin_ros_control/elfin/disable_robot`，向三个双轴模块请求 Servo Off。
2. 关闭 `/elfin_ros_control` 硬件节点，使 EtherCAT 管理器请求所有从站退出 OP 并回到 INIT。
3. ROS 通信失效时，直接向硬件驱动进程发送 `SIGINT`，走相同的正常退出路径。

这不是安全认证急停。电脑死机、进程卡死、网线断开后的驱动行为异常时，脚本可能无效。首次运动时必须另有一人只负责守住可立即断开的上游电源，且此人不得进入机械臂运动范围。

该脚本已在真实驱动上测试两次：两次均返回 `robot is disabled`，硬件节点随后干净退出。第一次退出后的 `slaveinfo` 重新枚举显示四个从站全部为 `SAFE-OP (State 4)`。测试过程中从未调用 Servo On、清故障、启动轨迹控制器或发送轨迹，机械臂没有运动。

## 2026-07-20 接触故障与吊装恢复记录

- 接触桌面后，Servo On 在三个双轴模块的 phase 3 同时失败；六轴状态字均为 `0x0028`，错误码均为 `0x0002`，Basic API 随即全轴 Servo Off 回滚。
- 本地源码只能证明这是 CiA-402 Fault，尚未找到 Han's 对 `0x0002` 的可靠定义，不能把它直接等同于某个单一碰撞码。
- 现场使用额定吊带张紧承重、机械约束 J1、清空扫掠区和夹点并由专人守电闸。清 Fault 后只调用 `elfin_module_open_brake_slave1`，其源码会同时释放 slave1 上的 J2/J1；J3--J6 未释放。
- 人员只用牵引绳调整。随后调用 `elfin_module_close_brake_slave1` 成功，三秒复测无位移，Servo=False、Fault=False；当时读数约为 J1 `10.49°`、J2 `-0.46°`。
- 此流程依赖临时起重和现场约束，不能自动化为“碰撞后自由坠落”。新增 `RECOVER_ELFIN_COLLISION.sh` 仅执行安全停止、诊断和可选的一次清 Fault，永远不开抱闸、不 Servo On、不发轨迹。

## 2026-07-20 Basic API 错误启动记录

单独运行 `roslaunch elfin_basic_api elfin_basic_api.launch` 时出现：

```text
Robot semantic description not found
Group 'elfin_arm' was not found
/get_planning_scene has not been advertised
```

原因是 Basic API 依赖 MoveIt 的 SRDF、`elfin_arm` 规划组和 planning scene 服务；不是 EtherCAT 故障。新增 `START_ELFIN_PANEL.sh` 和 `elfin5_control_panel.launch`，统一按 MoveIt → Basic API/Panel 的依赖顺序启动，并在启动前自动清理无法响应的死亡 ROS 注册，再检查 ROS master、模型、`/joint_states`、仍存活的重复节点和真机 Servo 状态。硬件启动入口在连接已有 rosmaster 时也执行相同清理。

## 源码启动层次与用途

这不是单纯的“简易 API”，也不是一套已经完成的视觉抓取业务逻辑，而是一套从 EtherCAT 到上层规划接口的完整 ROS 控制基础设施：

```text
E05 三个双轴 EtherCAT 从站
  -> SOEM EtherCAT 主站与 Elfin PDO/SDO 驱动
  -> ros_control 六关节硬件接口
  -> JointTrajectoryController 轨迹执行
  -> Basic API / MoveIt / RViz / 用户视觉抓取节点
```

- `elfin_ethercat_driver`：枚举从站、收发 PDO/SDO、Servo On/Off、清故障和状态读取。
- `elfin_ros_control`：把编码器/速度/力矩换算成 ROS 关节状态，把控制器命令写回驱动器。
- `elfin_arm_controller`：执行 ROS 标准 `FollowJointTrajectory` 轨迹。
- `elfin_basic_api`：在标准控制栈之上包装关节点动、笛卡尔点动、目标位姿、回零、速度倍率和图形面板。
- `elfin5_moveit_config`：运动学、路径规划、碰撞检查和 RViz 交互。
- `elfin_gazebo`：不连接真机的仿真。

项目负责“如何接管并执行关节运动”，但没有针对本实验的相机识别、目标选择、抓取策略和任务状态机；这些需要后续作为用户程序接在 MoveIt 或标准轨迹接口之上。

## 启动硬件驱动时实际发生的事情

执行 `roslaunch elfin_robot_bringup elfin_ros_control.launch` 会立即：

1. 自动检测唯一匹配 E05 从站身份的无 IP 有线接口，并用 root/raw-socket 权限独占它；当前为 `eth1`。
2. 找到四个从站并映射 PDO；把 1--3 号关节从站切到 OP，第 4 号末端 I/O 保持 SAFE-OP。
3. 启动 1 kHz EtherCAT 与 `ros_control` 循环。
4. 发布 `/joint_states`、使能状态和故障状态。
5. 启动 `joint_state_controller`，但只加载、不启动 `elfin_arm_controller`。

它不会因为启动而自动 Servo On；本机配置也已设置 `automatic_recognition: false`。为防止以后 Servo On 时使用未初始化目标，硬件接口已改为在首次编码器读取后将目标位置同步为实际位置，并显式清零速度与力矩命令。

## 让机械臂运动的入口

1. Gazebo：只动仿真模型，必须作为首选功能验证。
2. 短 `FollowJointTrajectory`：直接向 `elfin_arm_controller` 发小角度轨迹，边界清楚，最适合首次真机单关节试动。
3. MoveIt/RViz：先规划、检查碰撞，再点击 Execute，适合常规实验。
4. Basic API/Control Panel：提供 Servo On/Off、关节点动、笛卡尔点动和目标位置等直观入口。
5. Python/C++ 用户节点：通过 MoveIt 或轨迹 action 编写视觉抓取与任务逻辑。
6. 直接 EtherCAT/模块服务：绕过规划与碰撞检查，风险最高，不作为日常控制方法。

Control Panel 已按现场需求恢复原始持续点动：关节或笛卡尔按钮按下后朝目标/限位运动，松开调用 Stop；速度默认 1%，上限 100%。需要固定关键姿态和可重放任务时，优先使用 MoveIt 的命名姿态与重新规划，而不是依赖按键持续时间。

另有两条源码审计限制：不得发布 `elfin_teleop_joint_cmd_no_limit`；不得使用 Effort、PosTrq 或 PosVelTrq 等力矩相关接口。当前 `write_update()` 的力矩写回读取的是实测 `effort` 而不是 `effort_cmd`，其行为与接口定义不一致，必须单独审计和测试后才能启用。
