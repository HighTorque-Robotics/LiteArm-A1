请遵守以下要求：
1.保持和以下代码一样简洁的风格：
#!/usr/bin/env python3
import time
import sys
import os

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

from LiteArm.LiteArm import LiteArm  # 从LiteArm子目录导入LiteArm类

"""
简单的六关节机器人位置速度控制程序
直接在代码中修改目标位置数组来控制机器人
"""

def main():
    # 发送位置控制命令（使用阻塞模式）
    print("\n发送控制命令...")
    issuccess1 = robot.pos_vel_MAXtqe(pos1, vel, max_torque, iswait=True)
    print(f"执行状态1：{issuccess1}")
    time.sleep(3)
    issuccess2 = robot.pos_vel_MAXtqe(pos2, vel, max_torque, iswait=True)
    print(f"执行状态2：{issuccess2}")
    time.sleep(1)
    issuccess3 = robot.pos_vel_MAXtqe(pos3, vel, max_torque, iswait=True)
    print(f"执行状态3：{issuccess3}")

    # 保持位置2秒
    print("\n保持位置2秒...")
    time.sleep(2)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = LiteArm()
    pos1 = [-0.26, -1.5, -0.90, -0.5, -1.0, 0.0] 
    pos2 = [-0.26, -0.5, -0.50, -0.5, -1.0, 0.0] 
    pos3 = [0.0] * robot.motor_count
    vel = [0.8] * robot.motor_count      
    max_torque = [10.0] * robot.motor_count   
    try:
        main()
    except KeyboardInterrupt:
        # 不加这行电机在程序停止后也会掉电
        robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
2.不需要做过多的错误处理和打印输出
3.代码简洁易读是第一位