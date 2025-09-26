#!/usr/bin/env python3  # 指定使用python3解释器
import time  # 导入时间模块，用于延时
import sys  # 导入sys模块，用于操作系统相关功能
import os  # 导入os模块，用于路径操作

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 获取当前文件的上上级目录
sys.path.append(parent_dir)  # 将上上级目录添加到模块搜索路径

from LiteArm.LiteArm import LiteArm  # 从LiteArm子目录导入LiteArm类，实现机械臂控制

"""
简单的六关节机器人位置速度控制程序
直接在代码中修改目标位置数组来控制机器人
"""  # 程序说明：通过修改目标位置数组控制六关节机械臂

def main():  # 主函数，执行机械臂控制流程
    # 发送位置控制命令（使用阻塞模式）
    print("\n发送控制命令...")  # 打印提示信息
    zero_success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)  # 让机械臂回到零位
    print(f"执行状态0：{zero_success}")  # 打印零位执行结果
    time.sleep(1)  # 延时1秒
    robot.pos_vel_MAXtqe(pos2, vel, max_torque, gripper=gripper_close, iswait=True)  # 机械臂移动到pos2并夹爪闭合
    time.sleep(2)  # 延时2秒
    robot.pos_vel_MAXtqe(pos1, vel, max_torque, gripper=gripper_open, iswait=True)  # 机械臂移动到pos1并夹爪打开
    time.sleep(2)  # 延时2秒
    robot.pos_vel_MAXtqe(pos2, vel, max_torque, gripper=gripper_close, iswait=True)  # 再次移动到pos2并夹爪闭合
    robot.pos_vel_MAXtqe(pos1, vel, max_torque, gripper=gripper_open, iswait=True)  # 再次移动到pos1并夹爪打开
    time.sleep(2)  # 延时2秒
    robot.pos_vel_MAXtqe(pos2, vel, max_torque, gripper=gripper_close, iswait=True)  # 再次移动到pos2并夹爪闭合
    time.sleep(2)  # 延时2秒
    zero_success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)  # 机械臂回到零位
    print(f"执行状态0：{zero_success}")  # 打印零位执行结果
    time.sleep(2)  # 延时2秒

    # 保持位置2秒
    print("\n保持位置2秒...")  # 打印提示信息
    time.sleep(2)  # 保持当前位置2秒
    # 结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":  # 判断是否为主程序入口
    robot = LiteArm()  # 实例化LiteArm对象，初始化机械臂
    zero_pos = [0.0] * robot.motor_count  # 定义零位位置数组，所有关节为0
    pos1 = [0.0, 0.8, 0.8, -0.30, 0.0, 0.0]  # 定义第一个目标位置
    pos2 = [0.0, 1.7, 1.7, -0.4, 0.0, 0.0]  # 定义第二个目标位置
    vel = [0.3] * robot.motor_count  # 定义各关节运动速度
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]  # 定义各关节最大力矩
    gripper_close = {
        'pos':0.0,  # 夹爪闭合位置
        'vel':1.5,  # 夹爪闭合速度
        'max_tqu':0.2  # 夹爪闭合最大力矩
    }
    gripper_open = {
        'pos':4.0,  # 夹爪打开位置
        'vel':1.5,  # 夹爪打开速度
        'max_tqu':0.2  # 夹爪打开最大力矩
    }
    try:
        main()  # 执行主控制流程
    except KeyboardInterrupt:  # 捕获键盘中断
        # 不加这行电机在程序停止后也会掉电
        robot.set_stop()  # 停止所有电机，保证安全
        print("\n\n程序被中断")  # 打印中断提示
        print("\n\n所有电机已停止")  # 打印所有电机停止提示
    except Exception as e:  # 捕获其他异常
        print(f"\n错误: {e}")  # 打印错误信息