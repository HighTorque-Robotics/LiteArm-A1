<<<<<<< HEAD
# Hightorque Robot SDK

高擎机器人电机控制SDK，提供C++库和Python接口，用于控制高擎关节电机。

## 项目简介

本项目是一个用于控制高擎机器人电机的SDK，支持通过CAN总线和串口与电机控制器通信。SDK提供了完整的C++库以及Python绑定，可以方便地集成到各种机器人控制系统中。

### 主要特性

- 支持多种电机型号（4438、5046、5047、6056等系列）
- 多种控制模式：位置控制、速度控制、力矩控制、混合控制
- 支持多个CAN总线板卡和多个电机
- 提供LCM（Lightweight Communications and Marshalling）消息发布
- 完整的Python接口支持
- 实时电机状态反馈
- CRC校验确保通信可靠性

## 系统要求

### 依赖库

- CMake >= 3.0.2
- C++11 编译器
- libserialport（串口通信）
- LCM（轻量级通信库）
- yaml-cpp（YAML配置文件解析）
- pybind11（Python绑定，可选）

### 安装依赖

Ubuntu/Debian系统：

```bash
# 安装系统依赖
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    libserialport-dev \
    libyaml-cpp-dev \

# 安装Python依赖
pip install numpy
```

### 配置文件

SDK使用YAML格式的配置文件来设置CAN总线板卡和电机参数。配置文件示例：

```yaml
can_bus:
  - device: /dev/ttyACM
    baudrate: 4000000
    board_id: 1
    motor_ids:

```

## 编译

### 编译C++库

```bash
# 克隆代码（如果还没有）
cd /path/to/hightorque_robot

# 创建构建目录
mkdir -p build
cd build

# 配置和编译
cmake ..
make -j4
```

编译完成后，会生成以下文件：
- `libhightorque_robot.so` - 主要的共享库
- `motor_run` - 电机控制示例
- `motor_msg_subscriber` - LCM消息订阅示例

Python模块会生成在 `python/` 目录下：`hightorque_robot_py.cpython-*.so`

## 配置

### 机器人配置文件

主配置文件位于 `robot_param/robot_config.yaml`：

```yaml
robot:
  name: "HI"
  param_file: "../robot_param/1dof_STM32H730_model_test_Orin_params.yaml"
```

### 电机参数配置

电机参数文件（如 `1dof_STM32H730_model_test_Orin_params.yaml`）定义了：
- 机器人名称和类型
- CAN总线配置
- 电机数量和参数
- 通信设置

## 运行示例

### C++示例

1. **电机控制示例**
```bash
cd build
./motor_run
```
运行电机控制演示，电机会进行往复运动并实时显示状态。

2. **LCM消息订阅**
```bash
cd build
./motor_msg_subscriber
```
订阅并显示电机状态的LCM消息。

### Python示例

1. **基本功能测试**
```bash
cd python/
python quick_test.py
```
测试Python绑定的基本功能（不需要硬件）。

2. **电机控制示例**
```bash
cd python/
python example_motor_control.py
```
交互式电机控制程序，提供多种控制模式的演示。

3. **简单使用示例**
```python
import hightorque_robot_py as hr

# 创建机器人实例
robot = hr.Robot()
robot.lcm_enable()  # 启用LCM通信

# 获取电机列表
print(f"发现 {len(robot.Motors)} 个电机")

# 控制电机
for motor in robot.Motors:
    # 位置-速度-最大力矩控制
    motor.pos_vel_MAXtqe(position=1.0, velocity=0.5, max_torque=10.0)

# 发送控制命令
robot.motor_send_2()

# 读取电机状态
for motor in robot.Motors:
    state = motor.get_current_motor_state()
    print(f"电机 {state.ID}: 位置={state.position:.3f}, 速度={state.velocity:.3f}")

# 停止所有电机
robot.set_stop()
robot.motor_send_2()
```

## API参考

### C++ API

主要类：
- `hightorque_robot::robot` - 机器人主控制类
- `motor` - 单个电机控制类
- `canboard` - CAN总线板卡类
- `canport` - CAN端口类
- `serial_driver` - 串口驱动类

### Python API

#### Robot类（对所有电机有效）
- `lcm_enable()` - 启用LCM通信
- `motor_send_2()` - 发送电机控制命令
- `set_stop()` - 停止所有电机
- `set_reset()` - 重置所有电机
- `set_reset_zero()` - 设置电机零位
- `send_get_motor_state_cmd()` - 发送获取电机状态的命令
- `Motors` - 电机对象列表（只读）

#### Motor类（对单个电机有效）
- `position(pos)` - 位置控制
- `velocity(vel)` - 速度控制
- `torque(tor)` - 力矩控制
- `pos_vel_MAXtqe(pos, vel, max_tor)` - 位置-速度-最大力矩混合控制
- `pos_vel_tqe_kp_kd(pos, vel, tor, kp, kd)` - 带PD参数的混合控制
- `get_current_motor_state()` - 获取当前电机状态
- `get_motor_id()` - 获取电机ID

#### 控制模式常量
- `MODE_POSITION` (0x80) - 位置控制模式
- `MODE_VELOCITY` (0x81) - 速度控制模式
- `MODE_TORQUE` (0x82) - 力矩控制模式
- `MODE_POS_VEL_TQE` (0x90) - 混合控制模式

#### 注意事项
所有控制函数和查询函数都只是把命令放在缓存区，需要调用 `motor_send_2()` 才能发送。
比如控制电机运动：
```python
import hightorque_robot_py as hr
robot = hr.Robot()
# robot.lcm_enable() # 可选择开启LCM通信
robot.Motors[0].position(1.0) # 把控制命令放在缓存区
robot.motor_send_2()          # 发送命令
```

获取电机状态：
```python
import hightorque_robot_py as hr
robot = hr.Robot()
# robot.lcm_enable() # 可选择开启LCM通信
robot.send_get_motor_state_cmd() # 把控制命令放在缓存区
while True:
   robot.motor_send_2()          # 发送命令
   for motor in robot.Motors:
      state = motor.get_current_motor_state()   # 仅当robot.motor_send_2()执行之后，状态才会更新
      print(f"电机 {state.ID}: 状态={state.state}, 位置={state.position:.3f}, 速度={state.velocity:.3f}, 力矩={state.torque:.3f}")

```

## 故障排除

### 常见问题

1. **找不到串口设备**
   - 检查USB连接是否正常
   - 确认有权限访问串口：`sudo usermod -a -G dialout $USER`
   - 重新登录后生效

2. **Python导入失败**
   - 确保已正确编译Python模块
   - 检查Python路径：`export PYTHONPATH=$PYTHONPATH:/path/to/hightorque_robot/python`

3. **电机不响应**
   - 检查配置文件中的电机参数
   - 确认电机电源已开启
   - 检查CAN总线连接

4. **程序退出缓慢**
   - 这是已知问题，可以在Python程序末尾添加：
   ```python
   import gc
   del robot
   gc.collect()
   ```

### 调试技巧

1. 使用 `parse_demo` 检查配置是否正确加载
2. 查看控制台输出的错误信息
3. 检查 `/dev/ttyACM*` 设备是否存在
4. 使用 `dmesg` 查看系统日志

## 开发说明

### 目录结构

```
hightorque_robot/
├── CMakeLists.txt         # CMake构建配置
├── include/               # C++头文件
│   ├── hardware/          # 硬件相关类
│   ├── crc/               # CRC校验
│   └── serial_*.hpp       # 串口相关
├── src/                   # C++源文件
├── example/               # C++示例程序
├── python/                # Python绑定和示例
│   ├── pybind_wrapper.cpp # pybind11包装器
│   ├── setup.py           # Python安装脚本
│   └── *.py               # Python示例
├── robot_param/           # 机器人配置文件
└── msg/                   # LCM消息定义
```

### 添加新功能

1. 在相应的C++类中添加功能
2. 如需Python支持，在 `pybind_wrapper.cpp` 中添加绑定
3. 重新编译项目
4. 添加相应的测试和文档

## 许可证

请查看项目根目录下的LICENSE文件。

## 联系方式

如有问题或建议，请提交Issue或联系项目维护者。
=======
# LiteArm-A1
>>>>>>> 448257d35a87d1ba57d89df82eab28a02f0acd54
