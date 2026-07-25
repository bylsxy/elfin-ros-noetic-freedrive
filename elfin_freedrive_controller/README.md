# Elfin E05 零力拖拽控制器

这个包在现有 ROS Noetic / `ros_control` 架构中增加六轴重力补偿拖拽，不释放机械抱闸，也不绕过驱动 Fault、关节限位或 Servo 状态。

## 当前结论

- Gazebo 中已验证 `位置保持 -> 零力拖拽 -> 新位置保持` 的完整切换。
- 六轴控制器资源互斥、外力可推动、松手后阻尼减速、POINT 文件记录均已验证。
- 真机硬件写周期已经改为下发 `axis.effort_cmd`；空载 E05 已用 17 个静态样本、6 组双向姿态完成 J2/J3 重力标定。
- Home 和中等承重姿态的 1 秒静态 CST，以及覆盖旧版完整 3 秒交接的 4 秒静态 CST 均已通过；最大位移为 `1.1e-5 rad`，无 Fault 或 Servo Off。当前接管已改为 0.5 秒，必须另做空载复验。
- 2026-07-25 已完成空载人工拖动；旧入口自适应会把按键前的人手预载误学成重力倍率，已依据 trial 日志默认关闭。修复后的固定模型仍需再次做受监督小范围复验。
- 真机入口日常默认锁定，只有显式 `--freedrive` 才开放；多姿态标定未验证时还有第二道配置门禁。
- 现有驱动没有实体灯环颜色写接口。管理器只发布厂家定义的预期颜色，不能保证真机灯环跟随。

## 数据流

```text
POINT (DI bit 4) -> 管理器 -> 持久记录当前六轴姿态

FREE (DI bit 5，新的按下沿立即请求)
  -> 在位置控制仍在线时采集静止姿态和反馈力矩
  -> 检查模型方向、比例、残差、Servo/Fault/joint_states/静止/真机门禁
  -> STRICT 停止 elfin_arm_controller
  -> STRICT 启动 elfin_freedrive_controller
  -> 硬件接口切入 CST 力矩模式
  -> 从实测保持力矩平滑过渡到角度相关的 KDL 重力力矩
  -> KDL 重力补偿 + 阻尼 + 力矩/速度/关节限位

FREE 松开
  -> 等待速度连续降低
  -> STRICT 恢复 elfin_arm_controller
  -> 在松手后的新位置保持
```

任何切换或运行监控失败都会先进入增强阻尼的 `RECOVERING`，连续确认静止后原子恢复位置控制。真机若严格切换暂时失败，会先停止 CST，让驱动器用同步过的当前编码器位置进入 CSP 保持，再重试 ROS 位置控制器；只有当前位置保持仍完全无法建立时才请求 Servo Off。FREE 路径不会调用任何开闸/合闸服务，也不能绕过驱动器底层 Fault。

## 真机重力预检

管理器在位置控制器仍持有机械臂时持续被动采集至少 12 个静止样本，持续至少 0.25 秒，然后比较反馈保持力矩和同一姿态下的 URDF/KDL 重力力矩。默认分成两级：

- 严格通过：至少两个有效承重轴、方向一致度不低于 `0.90`、实测/模型比例在 `0.50..2.00`、归一化残差不高于 `0.30`，且反馈力矩标准差不高于 `2 Nm`。
- 黄色警告但允许：多姿态标定已经验证、反馈稳定、没有反向轴、比例仍在 `0.50..2.00` 且方向一致度不低于 `0.50`；这覆盖单轴承重和静摩擦造成的单姿态残差。
- 硬拒绝：力矩方向相反、比例越界、反馈波动、状态过期、机械臂仍在运动、Fault 或 Servo Off。

单个姿态不能可靠区分负载、摩擦和人手预加载，因此真机默认不再用入口反馈改写重力倍率。管理器在正常位置保持期间仍持续给出预检结果，切入瞬间的反馈只用于 0.5 秒无跳变交接，随后回到多姿态标定的固定模型。Gazebo 明确显示“跳过真机反馈力矩一致性检查”。

## 一键离线仿真

```bash
/home/jetson/START_ELFIN_FREEDRIVE_SIM.sh
```

这个入口同时启动 Gazebo 力矩模型、MoveIt、Basic API 和中文 Panel，固定使用：

```text
ROS_MASTER_URI=http://127.0.0.1:11312
GAZEBO_MASTER_URI=http://127.0.0.1:11346
```

因此它不连接默认 ROS master，也不包含 EtherCAT 节点。需要 3D 窗口时可用 `--gazebo-gui`、`--rviz` 或 `--rviz-egl`；Jetson 日常验证建议保持默认无 3D 窗口。关闭时在同一终端按 `Ctrl+C`。

`gzserver` 已固定使用无显示的软件渲染环境，避免继承 Jetson 桌面的 GLX 上下文后触发 `GLXBadDrawable`。Gazebo 使用的是 stock effort 轨迹控制器：在非零重力姿态恢复位置控制时会有 PID 静态偏差，因此仿真主要验证重力控制、资源互斥和状态机，不能用这个偏差推断真机 CSP 的保持误差。仿真退出速度阈值和阻尼也有独立覆盖，不会改写真机配置。

## 真机入口

普通硬件启动会加载控制器但保持门禁锁定：

```bash
/home/jetson/START_ELFIN_HARDWARE.sh
```

只有受监督的 freedrive 验证才使用：

```bash
/home/jetson/START_ELFIN_HARDWARE.sh --freedrive
```

`--freedrive` 只开放管理器门禁，不会自动 Servo On、切力矩模式或运动。随后仍需启动 Panel、正常 Servo On，并由 Panel 或实体 FREE 明确请求进入。

空载静态真机验证已于 2026-07-24 完成，记录见 `docs/e05_freedrive_calibration_2026-07-24.md`。第一次人手试拖仍必须保持同样清场、防坠和守闸条件，先轻推几毫米/几度并松手观察，再逐步扩大范围；安装任何末端或负载后不能直接沿用空载结论。

## 固定按钮和记录文件

- POINT：DI 原始字 bit 4，上升沿记录一次。
- FREE：DI 原始字 bit 5，检测到新的按下沿立即请求进入，松开退出。
- 管理器启动时 FREE 已按住不会触发，必须先释放再重新按下。
- 默认记录文件：`/home/jetson/.ros/elfin_freedrive_points.yaml`。
- 仿真记录文件：`/tmp/elfin_freedrive_sim_points.yaml`。

每条记录包含时间、触发来源和六个关节弧度，可直接用作 eye-to-hand 标定时的机器人姿态采样索引；它不是连续轨迹录像。

## 主要安全参数

配置位于 `config/elfin_freedrive_controller.yaml`：

- `effort_limit_scale`：按 URDF 额定力矩设置第一层上限，当前为 20%。
- `effort_limits`：真机显式每轴上限，当前为 `[15, 84, 30, 15, 8, 8] Nm`。J2 的 `84 Nm` 等于 E05 URDF `420 Nm` 额定值的 20%，用于覆盖 2026-07-25 空载高重力姿态实测的约 `67.3 Nm` 保持力矩；它没有突破全局 20% 上限。
- `maximum_gravity_effort_fraction`：模型重力达到显式上限 90% 时拒绝/退出，防止静默饱和。

管理器在位置控制仍保持机械臂时，已经使用同一套固定重力模型、力矩上限和 90% 余量做后台容量预检。容量不足会明确报告“Jx 需求/可用力矩”，不会再先显示 `ACTIVE` 后以“模型不一致”或“超速”的错误理由退出。

真机默认关闭单姿态入口重力倍率自适应。2026-07-25 的日志证明，持续人手预载会把同一空载机械臂的入口倍率从 `0.544` 改到 `1.201`；电机反馈力矩无法区分重力、摩擦和人手外力。当前只用入口反馈完成 0.5 秒无跳变交接，随后使用经过多姿态、双向到达标定的固定重力模型。更换末端或负载后应重新做多姿态标定，不能靠按 FREE 时施力“调重力”。
- `torque_rate_limits`：每轴力矩变化率限制。
- `velocity_soft_limits` / `velocity_hard_limits`：软阻尼和硬停止阈值。
- `velocity_limit_scale`：Panel 可在 `50%--200%` 间调整的运行时倍率；只能在零力控制器未运行时修改，默认 100%。它同时缩放软减速和硬停止阈值，不改变关节角、力矩或 Fault 保护。
- `damping_scales`：Panel 中 J1--J6 独立的速度阻尼倍率，范围 `25%--200%`；降低可减轻该轴拖拽阻力，但更容易快速运动。它不改写重力模型。
- `adaptive_entry_scale`：默认关闭，避免把人手预载、静摩擦或外部接触误学成重力。
- `handoff_duration`：从位置保持力矩过渡到固定多姿态重力模型的时间，当前为 `0.5 s`。
- `limit_margin` / `hard_limit_margin`：接近关节限位时的软/硬保护范围。
- `damping` / `friction`：手感和真实机械摩擦补偿，真机验证后才能调。
- `gravity_joint_scales` / `gravity_bias`：本机空载双向姿态标定结果；负载改变后必须重新验证。
- `minimum_model_*` / `maximum_model_*`：真机入口的一致性门禁，不要为“先跑起来”而放宽。

不要为了“更轻”直接提高力矩上限或摩擦补偿。先保存 `/elfin_freedrive_controller/command_state`、`/joint_states` 和驱动力矩反馈，再一次只调整一个参数。

每次进入会在 `/home/jetson/.ros/elfin_freedrive_trials/` 新建 CSV，记录六轴位置、速度、实测力矩、模型重力力矩、实际命令、接管进度和状态事件。多姿态标定由 `collect_elfin_gravity_calibration.py` 在正常位置控制下低速往返并被动读取力矩；不允许通过松抱闸或观察自由下坠来辨识重力。

## 灯环状态话题

管理器发布：

- `GREEN_SERVO_ON_EXPECTED`
- `YELLOW_SERVO_OFF_EXPECTED`
- `RED_FAULT_EXPECTED`
- `BLUE_ZERO_FORCE_EXPECTED`

对应话题为 `/elfin_freedrive_manager/ring_state` 和 `/elfin_freedrive_manager/ring_color`。`EXPECTED` 表示厂家手册期望语义，不表示驱动已经向实体灯环写色。
