# LiteArm-A1 部分 demo 例程说明


### 安装说明

#### 安装依赖

1. 创建 conda 环境
```bash
conda create -n hightorque_test python=3.10
```
2. 激活 conda 环境
```bash
conda activate hightorque_test
```
3. 安装numpy库
```bash
conda install numpy
```
## 🚀 控制脚本说明

| 脚本文件 | 功能描述 | 类别 |
|----------|----------|------|
| `0_robot_get_state.py` | 查看机械臂关节角信息 | 状态监测 |
| `0_example_motor_set_zero.py` | 重置机械臂零位 | 初始化 |
| `1_PosVel_control.py` | 简单速度位置模式 | 基础控制 |
| `3_interpolation_control_nozeroVel.py` | 插值非零速度控制 | 高级控制 |
| `3_interpolation_control_zeroVel.py` | 插值零速度控制 | 高级控制 |
| `3_sin_trajectory_control.py` | sin轨迹控制 | 轨迹规划 |
