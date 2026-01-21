# Tools/mavlink_shell.py 输入输出分析

## 功能概述
通过 MAVLink 协议打开一个远程 shell，用于与飞行器进行命令行交互。

---

## 输入

### 1. 命令行参数
| 参数 | 说明 | 默认值 |
|------|------|--------|
| `port` | MAVLink 端口：serial(设备[,波特率]), udp(IP:端口), tcp(IP:端口) | 自动检测 |
| `--baudrate, -b` | MAVLink 端口波特率 | 57600 |

### 2. 标准输入 (stdin)
- 用户键盘输入的 shell 命令
- 支持的特殊按键：
  - `Enter` - 发送命令
  - `Backspace` - 删除字符
  - `↑/↓` - 浏览命令历史（最多50条）

---

## 输出

### 1. 标准输出 (stdout)
- 从飞行器接收到的 shell 响应数据
- 调试信息（debug模式）
- 用户输入的回显

### 2. MAVLink 消息输出
通过 `SERIAL_CONTROL` 消息发送到飞行器：
- 消息ID: `SERIAL_CONTROL`
- 设备号: `devnum=10`（shell端口）
- 标志: `EXCLUSIVE | RESPOND`
- 数据块: 每次最多70字节

---

## 数据流

```
┌─────────────┐                    ┌──────────────────┐
│   用户键盘   │ ──stdin──>         │  mavlink_shell   │
│             │ <──stdout──         │                  │
└─────────────┘                    └────────┬─────────┘
                                             │
                    MAVLink SERIAL_CONTROL   │
                    <======================> │
                                             │
                                    ┌───────▼───────┐
                                    │   飞行器       │
                                    │   (PX4)       │
                                    └───────────────┘
```

---

## 核心类

### MavlinkSerialPort
| 方法 | 功能 |
|------|------|
| `write(b)` | 通过 MAVLink 发送字节到飞行器 |
| `read(n)` | 从飞行器读取最多 n 字节 |
| `close()` | 关闭连接 |
| `_recv()` | 内部方法，接收 SERIAL_CONTROL 消息 |

---

## 依赖
- `pymavlink` - MAVLink 协议实现
- `pyserial` - 串口通信
- `termios` - 终端控制（禁用回显、非缓冲输入）

---

# 雷达位姿估计与IMU融合集成方案

## 概述

将10Hz雷达位姿估计与高频IMU数据集成到PX4中，用于室内导航。PX4已有完整的microXRCE-DDS客户端和EKF2外部视觉融合基础设施。

---

## Aiding（辅助）原理

```
IMU (200-1000Hz) ──────────────→ 持续状态预测 (Dead Reckoning)
                                        │
                                        │ 漂移修正
                                        ↓
雷达 (10Hz) ──────→ ROS2桥接 ────→ EKF2融合 (100Hz)
```

### 核心概念

| 组件 | 频率 | 作用 |
|------|------|------|
| IMU | 200-1000Hz | 提供高频状态传播，但会累积漂移 |
| 雷达 | 10Hz | 提供绝对位姿测量，修正IMU漂移 |
| EKF2 | 100Hz | 24状态扩展卡尔曼滤波器，融合所有传感器 |

---

## 完整数据流

```
┌──────────────────┐     ROS2 Topics      ┌─────────────────────┐
│                  │ <────────────────    │                     │
│  雷达传感器        │    /radar/pose       │  ROS2 雷达桥接节点    │
│  (10Hz)          │    /radar/twist      │  (坐标转换)           │
│                  │                      │                     │
└──────────────────┘                      └──────────┬──────────┘
                                                     │
                                                     │ VehicleOdometry
                                                     │ (ENU→NED转换)
                                                     ↓
                                           ┌─────────────────────┐
                                           │  microXRCE-DDS      │
                                           │  Agent (UDP:8888)   │
                                           └──────────┬──────────┘
                                                      │
                                                      │ microXRCE-DDS
                                                      │ protocol
                                                      ↓
┌──────────────────┐    uORB     ┌─────────────────────┐
│                  │ <────────── │  PX4                │
│  EKF2            │    vehicle_ │  uxrce_dds_client   │
│  (100Hz)         │    visual_  │  模块                │
│                  │    odometry │                     │
└────────┬─────────┘             └─────────────────────┘
         │
         │ fused estimate
         ↓
┌──────────────────┐
│  位姿估计输出      │
│  - position      │
│  - velocity      │
│  - attitude      │
└──────────────────┘
```

---

## 坐标系转换

### ROS2 (ENU/FLU) → PX4 (NED/FRD)

```
ROS2标准                    PX4标准
─────────────────          ─────────────────
位置: ENU (东-北-上)   →    位置: NED (北-东-下)
机身: FLU (前-左-上)   →    机身: FRD (前-右-下)

         Z(Up)                      Z(Down)
            ↑                          ↓
            │                          │
     Y ←    │                    Y →   │
  (North)   │                (East)    │
            │                          │
            └────────→ X         └────────→ X
          (East)                   (North)
```

### 转换公式

```python
# 位置转换: ENU → NED
px4_pos[0] = ros_pos[1]   # North = East(ENU)
px4_pos[1] = ros_pos[0]   # East = North(ENU)
px4_pos[2] = -ros_pos[2]  # Down = -Up(ENU)

# 速度转换: ENU → NED
px4_vel[0] = ros_vel[1]
px4_vel[1] = ros_vel[0]
px4_vel[2] = -ros_vel[2]

# 四元数转换: 需要绕East轴旋转180°
# 从ENU到NED的四元数: q_enu_to_ned = [√0.5, √0.5, 0, 0]
```

---

## VehicleOdometry 消息格式

```cpp
# 定义在 msg/VehicleOdometry.msg

uint64 timestamp           # 发布时间戳 (微秒)
uint64 timestamp_sample    # 测量时间戳 (微秒)

# 姿态帧
uint8 pose_frame           # 1 = NED (PX4标准)
float32[3] position        # 位置 [x, y, z] 单位:米
float32[4] q               # 四元数 [w, x, y, z]

# 速度帧
uint8 velocity_frame       # 1 = NED, 3 = BODY_FRD
float32[3] velocity        # 速度 [vx, vy, vz] 单位:m/s
float32[3] angular_velocity # 角速度 [wx, wy, wz] 单位:rad/s

# 协方差 (不确定性)
float32[3] position_variance      # 位置方差 [m²]
float32[3] orientation_variance   # 姿态方差 [rad²]
float32[3] velocity_variance      # 速度方差 [(m/s)²]

# 质量和状态
int8 quality               # 质量 0-100 (越高越可靠)
uint8 reset_counter        # 重置计数器 (检测跳跃)
```

---

## 实施步骤

### 第一步：ROS2雷达桥接节点（主要工作）

# see mocap_to_visual_odometry 

### 第二步：PX4配置（验证现有配置）

**文件**: `src/modules/uxrce_dds_client/dds_topics.yaml`

已配置（第88-89行）:
```yaml
subscriptions:
  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry
```

### 第三步：启动uxrce_dds_client模块

```bash
# 在板级启动脚本中添加
uxrce_dds_client start -t udp -p 8888
```

### 第四步：EKF2参数配置

```bash
# 外部视觉融合控制 (位掩码: 1111 = 融合位置+速度+偏航)
EKF2_EV_CTRL = 15

# 测量噪声 (根据雷达规格调整)
EKF2_EVP_NOISE = 0.05      # 位置噪声 [m]
EKF2_EVV_NOISE = 0.15      # 速度噪声 [m/s]
EKF2_EVA_NOISE = 0.05      # 姿态噪声 [rad]

# 延迟和同步
EKF2_EV_DELAY = 50         # 测量延迟 [ms]

# 质量阈值
EKF2_EV_QMIN = 50          # 最低质量要求 (0-100)

# 室内导航设置
EKF2_GPS_CTRL = 0          # 禁用GPS
EKF2_HGT_REF = 4           # 使用外部视觉作为高度源
```

---

## 关键文件清单

### PX4端（极少修改）

| 文件 | 作用 | 需要修改 |
|------|------|----------|
| `src/modules/uxrce_dds_client/dds_topics.yaml` | DDS话题配置 | 验证已有配置 |
| `src/modules/ekf2/EKF2.cpp` | EKF2外部视觉融合 | 无需修改 |
| `msg/VehicleOdometry.msg` | 消息格式定义 | 无需修改 |

### ROS2端（主要工作）



---

## EKF2融合实现

**文件**: `src/modules/ekf2/EKF2.cpp:2016-2168`

`UpdateExtVisionSample()` 函数实现:
- 速度帧验证 (NED, FRD, BODY_FRD)
- 位置帧验证 (NED, FRD)
- 姿态四元数验证和归一化
- 协方差处理（优先消息方差，否则参数）
- 质量过滤 (`EKF2_EV_QMIN`)
- 时间戳同步

**最大间隔**: 200ms - 超时后自动切换到IMU模式

---

## ROS2/uORB话题频率限制

### 重要：各数据源的频率要求和限制

| 话题/传感器 | 硬性限制 | 推荐频率 | 说明 |
|------------|----------|----------|------|
| **vehicle_visual_odometry** | 最小 **5Hz** | **10-30Hz** | 最大间隔200ms，低于5Hz会超时 |
| **sensor_accel (IMU)** | 无硬限制 | **200-1000Hz** | EKF2用IMU预测，越高越好 |
| **sensor_gyro (IMU)** | 无硬限制 | **200-1000Hz** | 同上 |
| **vehicle_gps_position** | 最小 **2Hz** | **5-10Hz** | 最大间隔500ms |
| **sensor_baro** | 最小 **5Hz** | **10-50Hz** | 最大间隔200ms |
| **vehicle_magnetometer** | 最小 **2Hz** | **10-50Hz** | 最大间隔500ms |

### 频率限制详解

#### 1. EKF2滤波器频率
```cpp
// EKF2预测频率：100Hz (默认)
EKF2_PREDICT_US = 10000  // 10ms = 100Hz
```
- EKF2以100Hz运行
- 每次预测使用IMU数据
- 传感器数据在缓冲区中与IMU同步

#### 2. 外部视觉(雷达)频率
```cpp
// 来自 src/modules/ekf2/EKF/common.h
EV_MAX_INTERVAL = 200e3  // 200ms = 5Hz 最小
```

**频率限制说明**:
- **硬性下限**: 5Hz (200ms间隔) - 低于此会触发超时
- **推荐范围**: 10-30Hz
  - 10Hz: 可用，但有较大延迟
  - 20-30Hz: 推荐，平衡性能和负载
  - 50Hz以上: 收益递减，增加CPU负载

**如果雷达只有10Hz**:
- 完全可用，EKF2会自动处理
- 设置 `EKF2_EV_DELAY = 100` (100ms) 来补偿延迟
- 系统会使用IMU填补雷达数据之间的空隙

#### 3. 动态最小观测间隔
```cpp
// 来自 estimator_interface.cpp
_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
```

EKF2会动态计算传感器最小允许间隔，防止数据丢失。如果数据到达太快，会发出警告：
```
ECL_WARN("EV data too fast %" PRIi64 " < %" PRIu64 " + %d", ...)
```

#### 4. microXRCE-DDS限制
- **传输速率**: UDP可达数百Hz无压力
- **带宽**: VehicleOdometry约200字节/消息
  - 10Hz = 2KB/s = 16kbps (可忽略)
  - 50Hz = 10KB/s = 80kbps (仍然很低)

### 您的雷达配置建议

**雷达输出: 10Hz**
```cpp
// mocap_to_visual_odometry.cpp 中的限流配置
time_unit_ = 0.1  // 100ms = 10Hz
```

**如果雷达输出频率可调**:
- **10Hz**: 可用，设置 `EKF2_EV_DELAY = 100`
- **20Hz**: 推荐，`EKF2_EV_DELAY = 50`
- **30Hz**: 最佳，`EKF2_EV_DELAY = 33`

**关键参数**:
```bash
# 根据实际频率调整延迟参数
EKF2_EV_DELAY = 100  # [ms] = 1000 / 雷达频率(Hz)
```

---

## 测试验证

### 1. 静态测试
```bash
# 检查数据接收
px4 listener vehicle_visual_odometry

# 检查融合状态
px4 listener estimator_aid_src_ev_pos
px4 listener estimator_aid_src_ev_vel
```

### 2. 动态测试
- 缓慢移动飞行器
- 监控 `estimator_local_position`
- 检查位置漂移

### 3. 鲁棒性测试
- 暂时遮挡雷达
- 验证IMU切换
- 检查平滑恢复

---




# PX4-ROS2/DDS BRIDGE 配置指南

## mavlink_shell.py 在此场景中的作用

**作用有限**。`mavlink_shell.py` 主要用于：
- 调试和诊断
- 临时修改参数
- 检查 DDS 桥接状态

**不用于**实际的 ROS2/DDS 数据桥接（那需要 `micrortps_agent` 或 `px4-ros2` bridge）。

---

## ARM64 载板端配置步骤

### 1. PX4 固件配置

在 `boards/hkust/nxt-dual/default.px4board` 或对应板配置中启用：

```make
# 启用 uORB DDS
CONFIG_UORB_ENABLE_DSPY=y

# 启用 MicroRTPS_agent
CONFIG_MICRORTPS_AGENT=y
CONFIG_MICRORTPS_TRANSPORT=uart  # 或 udp

# DDS 配置
CONFIG_DDS=y
```

### 2. 固件运行时参数设置

使用 mavlink_shell.py 或 QGroundControl 设置以下参数：

```bash
# 通过 mavlink_shell.py 执行：
param set UX_RNG_START_MIN 0
param set UX_RNG_END_MAX 100
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0

# DDS 相关
param set ORB_USE_MAVLINK 1          # 允许 uORB 通过 MAVLink
param set ORB_IDLC_NUTTX 1           # IDL 支持
param set MAV_PROTO_VER 2            # MAVLink v2
param set MAV_COMP_ID_ALL 1          # 组件ID
```

### 3. ARM64 载板安装依赖

```bash
# 安装 ROS2 Humble（ARM64）
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# 安装 PX4-ROS2 bridge
sudo apt install ros-humble-px4-ros2 -y

# 安装 FastDDS
sudo apt install fastdds -y
```

### 4. 启动 MicroRTPS Agent

```bash
# 在 ARM64 载板上运行
micrortps_agent -d /dev/ttyACM0 -b 921600 -t serial

# 或 UDP 模式
micrortps_agent -t UDP -r 2019 -s 2020
```

### 5. 启动 ROS2 Bridge

```bash
source /opt/ros/humble/setup.bash
source /usr/share/px4-ros2/ros2_ws/install/setup.bash

# 启动 bridge
ros2 launch px4_ros2 bridge_agent.launch.py
```

### 6. 使用 mavlink_shell.py 验证

```bash
# 连接到载板上的 PX4
python3 Tools/mavlink_shell.py /dev/ttyACM0 -b 57600

# 进入 shell 后检查 DDS 状态
top                    # 查看 micrortps_agent 是否运行
uorb top               # 查看 uORB 消息流
dds status             # 检查 DDS 连接（如果有命令）
```

---

## 完整数据流

```
┌──────────────┐      uORB      ┌─────────────┐      MicroRTPS      ┌──────────────┐
│              │ <────────────> │  PX4        │ <─────────────────> │  ARM64载板   │
│  传感器/执行器│                │  (NuttX)    │   (Serial/UDP)      │  (ROS2)      │
└──────────────┘                └─────────────┘                     └──────────────┘
                                                                          │
                                                                          │ DDS
                                                                          ▼
                                                                   ┌──────────────┐
                                                                   │  ROS2 Topics │
                                                                   │  /fmu/...    │
                                                                   └──────────────┘

                              ┌──────────────────────────────────────────────┐
                              │         mavlink_shell.py (仅调试用)           │
                              │  <────── MAVLink Serial Control ──────>      │
                              └──────────────────────────────────────────────┘
```

---

## 关键端口/设备

| 用途 | 设备/端口 |
|------|----------|
| MAVLink Shell | `/dev/ttyACM0` @ 57600 |
| MicroRTPS Bridge | `/dev/ttyACM1` 或 UDP @ 921600 |
| ROS2 DDS | UDP 2019/2020 或默认 7400/7411 |

---

## 故障排查

```bash
# 检查串口设备
ls -l /dev/ttyACM*

# 检查 ROS2 节点
ros2 node list

# 检查话题
ros2 topic list | grep fmu

# 检查 DDS 域
ros2 daemon stop && ros2 daemon start
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```
