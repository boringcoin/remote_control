# RDK 手套蓝牙接收与 AuraOS 遥操

该目录用于把弯曲度手套的 JDY-18 蓝牙串口数据接到 RDK，再经 AuraOS 已运行的本地运动 API 控制 ESP 底盘。

它**不会直接打开** `/dev/ttyUSB0`：该串口仍由 AuraOS 守护进程独占。RDK 上已验证的接口为 `http://127.0.0.1:8765/api/motion`。

## 数据格式

手套原始 Arduino 程序以约 10 Hz 发送一行 CSV：

```text
finger_1,finger_2,finger_3,finger_4,finger_5,angle_x,angle_y,angle_z\r\n
```

前五项是弯曲角（约 0–90）。最后三项是 JY901 的 `stcAngle.Angle[0..2]` 换算的姿态角（度）。这份手套固件**没有发送原始陀螺仪角速度或加速度**；脚本会记录它实际收到的三路姿态角，字段名为 `angle_x_deg`、`angle_y_deg`、`angle_z_deg`。

每次运行都会在 `logs/` 下生成 CSV，保存手指、姿态、映射的速度、死手开关状态和原始帧。
手套通过 JDY-18 蓝牙发送姿态数据，当前遥操程序主要使用 roll 和 pitch 进行控制：

roll：控制机器人前进 / 后退；
pitch：控制机器人左转 / 右转。

程序启动后会记录静止零位：

neutral_roll
neutral_pitch

并计算相对偏移：

linear_delta = roll - neutral_roll
turn_delta   = pitch - neutral_pitch

当前日志示例：

roll=-49.00
neutral_roll=-12.47
linear_delta=-36.53
pitch=58.00
neutral_pitch=0.00
turn_delta=58.00

对应控制状态：

BACKWARD_LEFT
left_speed=-150.0
right_speed=-70.0

直线后退时：

BACKWARD
left_speed=-150.0
right_speed=-150.0

程序会记录当前姿态、零位、运动状态、左右轮速度和原始数据。若蓝牙数据超过约 1 秒未更新，会自动切换到 STOP。## 首次安装与扫描

```bash
cd /home/sunrise/remote_control
/home/sunrise/auraos/.venv-linux/bin/python -m pip install -r requirements.txt
/home/sunrise/auraos/.venv-linux/bin/python glove_teleop.py --scan
```

手套上电后，记下输出中的 MAC 地址。例如：

```bash
/home/sunrise/auraos/.venv-linux/bin/python glove_teleop.py --address AA:BB:CC:DD:EE:FF
```

这一步是**采集模式**：只显示并记录数据，机器人不会动。

## 遥操前的安全试验

在机器人悬空或留出足够安全距离时先用 dry-run 验证方向：

```bash
/home/sunrise/auraos/.venv-linux/bin/python glove_teleop.py \
  --address AA:BB:CC:DD:EE:FF --enable-motion --dry-run
```

默认映射为：

- `angle_y` 相对上电静止零位：前后速度；
- `angle_x` 相对零位：转向角速度；
- 第一列手指弯曲值达到 `60` 才是 deadman 使能；松开即发 AuraOS `/stop`；
- 前 30 个完整帧用于静止零位校准，未校准完成不会运动；
- 数据超过 250 ms 未到达、BLE 断开、Ctrl-C 或 API 错误都会停车。

若方向相反，在实际运行时分别加入 `--invert-linear` / `--invert-angular`；若拇指不是 CSV 的第一列，调整 `--deadman-finger 0..4`。确认方向后才移除 `--dry-run`：

```bash
/home/sunrise/auraos/.venv-linux/bin/python glove_teleop.py \
  --address AA:BB:CC:DD:EE:FF --enable-motion
```

默认最大速度为 `0.12 m/s`、`0.16 rad/s`，低于 AuraOS 当前配置的 `0.15 m/s`、`0.20 rad/s`。这不是紧急停机的替代品；现场仍应保留实体断电/急停手段。

## 蓝牙特性不匹配时

脚本优先订阅 JDY 常见的 `FFE1` 通知特征；若该模块刷了其它固件，指定实际通知 UUID：

```bash
/home/sunrise/auraos/.venv-linux/bin/python glove_teleop.py \
  --address AA:BB:CC:DD:EE:FF --characteristic UUID --enable-motion
```

如果扫描不到手套，先确认手套供电且未被手机占用；必要时用 `bluetoothctl` 先完成配对，再重新运行扫描。

