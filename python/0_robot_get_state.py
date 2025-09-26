#!/usr/bin/env python3
import time
import sys
import os

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

from LiteArm.LiteArm import LiteArm

"""
获取并打印机械臂关节角度信息
实时显示6个关节和夹爪的当前状态
"""

def print_robot_state(robot):
    """打印机器人状态信息"""
    # 获取关节角度
    positions = robot.get_current_pos()
    velocities = robot.get_current_vel()
    
    # 获取夹爪状态
    robot.send_get_motor_state_cmd()
    robot.motor_send_2()
    gripper_state = robot.Motors[6].get_current_motor_state()
    
    print("\n" + "="*50)
    print("机械臂状态信息")
    print("="*50)
    
    # 打印6个关节信息
    for i in range(robot.motor_count):
        print(f"关节{i+1}: 位置={positions[i]:7.3f} rad, 速度={velocities[i]:7.3f} rad/s")
    
    # 打印夹爪信息
    print(f"夹爪:   位置={gripper_state.position:7.3f} rad, 速度={gripper_state.velocity:7.3f} rad/s")
    
    # 正运动学
    # fk = robot.forward_kinematics()
    # if fk is not None:
    #     pos, rot = fk
    #     print("\n末端位置 (米):")
    #     print(f"  X: {pos[0]:7.4f}, Y: {pos[1]:7.4f}, Z: {pos[2]:7.4f}")

def main():
    robot = LiteArm()
    
    try:
        while True:
            print_robot_state(robot)
            time.sleep(0.5)  # 每0.5秒更新一次
            
    except KeyboardInterrupt:
        print("\n\n程序被中断")

if __name__ == "__main__":
    main()