#!/usr/bin/env python3
"""
演示hightorque_robot库用法的Python脚本示例。
此脚本展示如何使用Python绑定控制电机。
"""

import time
import sys
import os
import signal
import threading
import numpy as np

# 将python目录添加到路径中以导入模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import hightorque_robot_py as hr
except ImportError as e:
    print(f"Error importing hightorque_robot_py: {e}")
    print("Make sure you have built the Python bindings using CMake")
    sys.exit(1)


# 干净关闭的全局标志
shutdown_flag = threading.Event()

def signal_handler(signum, frame):
    """处理Ctrl+C信号"""
    print("\n\n接收到中断信号。正在优雅关闭...")
    shutdown_flag.set()


def main():
    print("<<< motor_set_zero >>>")
    
    # 重置关闭标志
    shutdown_flag.clear()
    
    # 创建机器人实例
    robot = hr.Robot()
    
    # 启用LCM通信
    robot.lcm_enable()
    
    # 获取电机数量
    motor_count = len(robot.Motors)
    print(f"发现 {motor_count} 个电机")
    
    if motor_count == 0:
        print("未发现电机。请检查您的配置和连接。")
        return
    
    # 打印电机信息
    for i, motor in enumerate(robot.Motors):
        print(f"电机 {i}: ID={motor.get_motor_id()}, "
              f"类型={motor.get_motor_enum_type()}, "
              f"名称={motor.get_motor_name()}")
    
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 电机控制循环
    print("开始电机控制循环...")
    print("按Ctrl+C停止...")
    robot.set_reset_zero()
    time.sleep(1.5)
    
    try:
        while not shutdown_flag.is_set():

            robot.send_get_motor_state_cmd()
            
            for i, motor in enumerate(robot.Motors):
                state = motor.get_current_motor_state()
                print(f"Motor {state.ID}: mode={state.mode}, "
                      f"fault=0x{state.fault:02X}, "
                      f"pos={state.position:.6f}, "
                      f"vel={state.velocity:.6f}, "
                      f"tor={state.torque:.6f}")
            
            
            # 使用更短的睡眠时间并检查关闭标志
            for _ in range(50):  # 50 * 10ms = 500ms 总计
                if shutdown_flag.is_set():
                    break
                time.sleep(0.001)
                      
    except Exception as e:
        print(f"\n发生错误: {e}")
    
    finally:
        print("\n正在停止电机...")
        robot.set_stop()
        robot.motor_send_2()  # 发送停止命令
        time.sleep(0.1)  # 给停止命令一些发送时间
        print("电机已停止。")
        
        # 短暂延迟后强制退出以确保清理
        print("强制退出...")
        os._exit(0)

if __name__ == "__main__":
    print("高力矩机器人 Python 接口演示")
    print("=" * 40)
    
    try:

        main()

            
    except KeyboardInterrupt:
        print("\n退出中...")
    except Exception as e:
        print(f"错误: {e}")
        print(f"错误: {e}")