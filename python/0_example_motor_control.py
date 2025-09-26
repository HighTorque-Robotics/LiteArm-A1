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
    print(f"导入hightorque_robot_py时出错: {e}")
    print("请确保您已使用CMake构建了Python绑定")
    sys.exit(1)


# 干净关闭的全局标志
shutdown_flag = threading.Event()

def signal_handler(signum, frame):
    """处理Ctrl+C信号"""
    print("\n\n接收到中断信号。正在优雅关闭...")
    shutdown_flag.set()

def main():
    print("初始化高力矩机器人...")
    
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
        print(f"Motor {i}: ID={motor.get_motor_id()}, "
              f"Type={motor.get_motor_enum_type()}, "
              f"Name={motor.get_motor_name()}")
    
    # 设置信号处理程序
    signal.signal(signal.SIGINT, signal_handler)
    
    # 电机控制循环
    print("启动电机控制循环...")
    print("按Ctrl+C停止...")
    angle = np.pi
    count = 0
    
    try:
        while not shutdown_flag.is_set():
            # 控制电机
            for i, motor in enumerate(robot.Motors):
                if i < motor_count:
                    # 前半部分电机向一个方向移动
                    motor.pos_vel_MAXtqe(angle, 0.5, 10.0)
                else:
                    # 后半部分电机向相反方向移动
                    motor.pos_vel_MAXtqe(-angle, 0.5, 10.0)
            
            # 向电机发送命令
            robot.motor_send_2()
            
            # 读取电机状态
            for i, motor in enumerate(robot.Motors):
                state = motor.get_current_motor_state()
                print(f"Motor {state.ID}: mode={state.mode}, "
                      f"fault=0x{state.fault:02X}, "
                      f"pos={state.position:.6f}, "
                      f"vel={state.velocity:.6f}, "
                      f"tor={state.torque:.6f}")
            
            # 每250个周期反转方向
            count += 1
            if count >= 250:
                count = 0
                angle *= -1
                print(f"反转方向，新角度: {angle}")
            
            # 使用更短的睡眠时间并检查关闭标志
            for _ in range(50):  # 50 * 10毫秒 = 总共500毫秒
                if shutdown_flag.is_set():
                    break
                time.sleep(0.001)
                      
    except Exception as e:
        print(f"\n发生错误: {e}")
    
    finally:
        print("\n停止电机...")
        robot.set_stop()
        robot.motor_send_2()  # 发送停止命令
        time.sleep(0.1)  # 给停止命令发送留出时间
        print("电机已停止。")
        
        # 短暂延迟后强制退出以确保清理
        print("强制退出...")
        os._exit(0)


def demo_motor_types():
    """演示电机类型枚举"""
    print("\n可用的电机类型:")
    for name, motor_type in hr.get_motor_type_map().items():
        print(f"  {name}: {motor_type}")
    
    print(f"\n控制模式常量:")
    print(f"  MODE_POSITION: 0x{hr.MODE_POSITION:02X}")
    print(f"  MODE_VELOCITY: 0x{hr.MODE_VELOCITY:02X}")
    print(f"  MODE_TORQUE: 0x{hr.MODE_TORQUE:02X}")
    print(f"  MODE_POS_VEL_TQE: 0x{hr.MODE_POS_VEL_TQE:02X}")


def simple_position_control():
    """简单位置控制示例"""
    print("\n简单位置控制示例...")
    
    # 重置关闭标志
    shutdown_flag.clear()
    
    # 设置信号处理程序
    signal.signal(signal.SIGINT, signal_handler)
    
    robot = hr.Robot()
    robot.lcm_enable()
    
    if len(robot.Motors) == 0:
        print("未发现电机。")
        return
    
    motor = robot.Motors[0]  # 使用第一个电机
    print(f"使用电机ID: {motor.get_motor_id()}")
    print("按Ctrl+C停止...")
    
    # 移动到不同位置
    positions = [0.0, 0.5, -0.5, 0.0]
    
    try:
        for pos in positions:
            if shutdown_flag.is_set():
                break
                
            print(f"移动到位置: {pos}")
            motor.position(pos)
            robot.motor_send_2()
            
            # 等待并检查位置
            for i in range(1000):
                if shutdown_flag.is_set():
                    break
                
                state = motor.get_current_motor_state()
                print(f"当前位置: {state.position:.3f}, 目标: {pos}")
                time.sleep(0.001)
                robot.send_get_motor_state_cmd()
                robot.motor_send_2()
                # 检查是否足够接近目标
                if abs(state.position - pos) < 0.01:
                    break
        
        if not shutdown_flag.is_set():
            print("位置控制演示完成。")
            
    except Exception as e:
        print(f"\n发生错误: {e}")
        
    finally:
        print("\n停止电机...")
        # robot.set_stop()
        # robot.motor_send_2()
        time.sleep(0.1)
        print("电机已停止。")
        
        # 强制退出
        print("强制退出...")
        os._exit(0)

def get_motor_state():
    print("获取电机状态...")
    
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
        print(f"Motor {i}: ID={motor.get_motor_id()}, "
              f"Type={motor.get_motor_enum_type()}, "
              f"Name={motor.get_motor_name()}")
    
    # 设置信号处理程序
    signal.signal(signal.SIGINT, signal_handler)
    
    # 电机控制循环
    print("启动电机控制循环...")
    print("按Ctrl+C停止...")
    angle = np.pi
    count = 0
    robot.set_stop()
    robot.motor_send_2()
    robot.send_get_motor_state_cmd()
    try:
        while not shutdown_flag.is_set():
            # 控制电机

            # 向电机发送命令
            robot.motor_send_2()
            
            # 读取电机状态
            for i, motor in enumerate(robot.Motors):
                state = motor.get_current_motor_state()
                print(f"Motor {state.ID}: mode={state.mode}, "
                      f"fault=0x{state.fault:02X}, "
                      f"pos={state.position:.6f}, "
                      f"vel={state.velocity:.6f}, "
                      f"tor={state.torque:.6f}")
            
            # 每250个周期反转方向
            count += 1
            if count >= 250:
                count = 0
                angle *= -1
                print(f"反转方向，新角度: {angle}")
            
            # 使用更短的睡眠时间并检查关闭标志
            for _ in range(50):  # 50 * 10毫秒 = 总共500毫秒
                if shutdown_flag.is_set():
                    break
                time.sleep(0.001)
                      
    except Exception as e:
        print(f"\n发生错误: {e}")
    
    finally:
        print("\n停止电机...")
        robot.set_stop()
        robot.motor_send_2()  # 发送停止命令
        time.sleep(0.1)  # 给停止命令发送留出时间
        print("电机已停止。")
        
        # 短暂延迟后强制退出以确保清理
        print("强制退出...")
        os._exit(0)

if __name__ == "__main__":
    print("高力矩机器人Python接口演示")
    print("=" * 40)
    
    # 显示电机类型和常量
    demo_motor_types()
    
    print("\n选择演示:")
    print("1. 完整电机控制循环")
    print("2. 简单位置控制")
    print("3. 获取电机状态")
    print("4. 退出")
    
    try:
        choice = input("请输入选择 (1-4): ").strip()
        
        if choice == "1":
            main()
        elif choice == "2":
            simple_position_control()
        elif choice == "3":
            get_motor_state()
        elif choice == "4":
            print("正在退出...")
        else:
            print("无效选择。")
            
    except KeyboardInterrupt:
        print("\n正在退出...")
    except Exception as e:
        print(f"错误: {e}")