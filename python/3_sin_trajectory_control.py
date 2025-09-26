#!/usr/bin/env python3
import time
import sys
import os
import numpy as np

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from LiteArm.LiteArm import LiteArm  # 从LiteArm子目录导入LiteArm类

"""
正弦轨迹跟踪控制程序
机器人关节沿着正弦函数轨迹运动
"""

def main():
    # 控制参数
    frequency = 0.3  # Hz，正弦波频率（可调节：0.1-2.0 Hz）(频率越高速度越快)
    duration = 10.0  # 运动持续时间（秒）
    control_rate = 300  # 控制频率 Hz
    dt = 1.0 / control_rate
    
    # 定义各关节角度限制（弧度）
    # 1关节：±180° = ±π, 2-3关节：0到-180° = 0到-π, 4-5关节：±90° = ±π/2, 6关节：±180° = ±π
    joint_limits = [
        [-np.pi, np.pi],        # 关节1：±180度
        [0, np.pi],            # 关节2：-180到0度
        [0, np.pi],            # 关节3：-180到0度
        [0, np.pi/2],    # 关节4：±90度
        [-np.pi/2, np.pi/2],    # 关节5：±90度
        [-np.pi, np.pi]         # 关节6：±180度
    ]
    
    # 获取初始位置作为中心位置
    print("获取初始位置...")
    center_pos = robot.get_current_pos()
    print(f"中心位置: {[f'{p:.3f}' for p in center_pos]}")
    
    # 检查初始位置是否在限制范围内
    for i, pos in enumerate(center_pos):
        if pos < joint_limits[i][0] or pos > joint_limits[i][1]:
            print(f"警告: 关节{i+1}初始位置 {pos:.3f} 超出限制范围 [{joint_limits[i][0]:.3f}, {joint_limits[i][1]:.3f}]")
    
    # 设置各关节的振幅（弧度）- 自动调整以避免超限
    amplitudes = []
    for i in range(len(center_pos)):
        # 计算到上下限的距离
        dist_to_upper = joint_limits[i][1] - center_pos[i]
        dist_to_lower = center_pos[i] - joint_limits[i][0]
        # 取较小值作为安全振幅，再乘以0.8作为安全系数
        safe_amplitude = min(dist_to_upper, dist_to_lower) * 0.8
        # 与预设振幅比较，取较小值
        preset_amplitude = [0.4, 0.6, 0.6, 0.5, 0.4, 0.0][i]
        amplitudes.append(min(safe_amplitude, preset_amplitude))
    
    print(f"调整后的振幅: {[f'{a:.3f}' for a in amplitudes]} rad")
    
    # 计算并显示最大速度
    max_velocities = [amp * 2 * np.pi * frequency for amp in amplitudes]
    print(f"各关节最大速度: {[f'{v:.2f}' for v in max_velocities]} rad/s")
    
    # 设置各关节的相位偏移（可以让各关节运动不同步）
    # phase_offsets = [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 0]  # 相位偏移
    phase_offsets = [0, 0, 0, 0, 0, 0]  # 零相位偏移
    
    print("\n开始正弦轨迹运动...")
    print(f"频率: {frequency} Hz, 持续时间: {duration} 秒")
    print(f"振幅: {amplitudes}")
    
    start_time = time.time()
    step = 0
    
    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            current_time = time.time() - start_time
            
            # 计算正弦轨迹
            omega = 2 * np.pi * frequency
            
            # 位置：x = x0 + A * sin(ωt + φ)
            pos = []
            vel = []
            
            for i in range(robot.motor_count):
                # 正弦位置
                p = center_pos[i] + amplitudes[i] * np.sin(omega * current_time + phase_offsets[i])
                pos.append(p)
                
                # 速度（位置的导数）：v = A * ω * cos(ωt + φ)
                v = amplitudes[i] * omega * np.cos(omega * current_time + phase_offsets[i])
                vel.append(v)
            
            # 角度限幅
            for i in range(robot.motor_count):
                # 限制位置在关节限制范围内
                if pos[i] < joint_limits[i][0]:
                    pos[i] = joint_limits[i][0]
                    vel[i] = 0  # 到达限位时速度置零
                elif pos[i] > joint_limits[i][1]:
                    pos[i] = joint_limits[i][1]
                    vel[i] = 0  # 到达限位时速度置零
            
            # 发送控制命令
            for i in range(robot.motor_count):
                robot.Motors[i].pos_vel_MAXtqe(pos[i], vel[i], 10.0)
            robot.motor_send_2()
            
            # 定期打印状态
            if step % 50 == 0:  # 每0.5秒打印一次
                print(f"\r时间: {current_time:.2f}s | "
                      f"关节1位置: {pos[0]:.3f} | "
                      f"关节2位置: {pos[1]:.3f} | "
                      f"关节3位置: {pos[2]:.3f}", end="")
            
            step += 1
            
            # 控制循环频率
            loop_time = time.time() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)
                
    except KeyboardInterrupt:
        print("\n\n轨迹被中断")
    
    # 返回中心位置
    print("\n\n返回中心位置...")
    robot.pos_vel_MAXtqe(center_pos, [0.5] * robot.motor_count, [10.0] * robot.motor_count, iswait=True)
    
    print("运动完成")
    time.sleep(1)

if __name__ == "__main__":
    robot = LiteArm()
    
    # 先移动到安全的初始位置
    print("移动到初始位置...")
    zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # init_pos = [0.0, -1.0, -1.0, 0.0, 0.0, 0.0]
    init_pos = [-0.26, 1.2, 1.4, 0.8, -0.3, 0.0]
    vel = [0.5] * robot.motor_count
    max_torque = [10.0] * robot.motor_count
    
    success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)
    time.sleep(3)

    success = robot.pos_vel_MAXtqe(init_pos, vel, max_torque, iswait=True)
    if success:
        print("到达初始位置")
        time.sleep(1)
    
    try:
        main()
        while(1):
            success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
        robot.set_stop()