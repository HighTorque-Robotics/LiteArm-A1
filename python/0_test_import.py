#!/usr/bin/env python3
"""
验证Python绑定功能的简单测试脚本。
"""

import hightorque_robot_py as hr

def test_basic_functionality():
    print("测试hightorque_robot_py Python绑定...")
    
    # 测试模块导入
    print("✓ 模块导入成功")
    
    # 测试可用的类
    print(f"✓ Robot类可用: {hasattr(hr, 'Robot')}")
    print(f"✓ Motor类可用: {hasattr(hr, 'Motor')}")
    print(f"✓ MotorBackData类可用: {hasattr(hr, 'MotorBackData')}")
    
    # 测试枚举
    motor_types = hr.get_motor_type_map()
    print(f"✓ 电机类型可用: {len(motor_types)} 种类型")
    print(f"  示例: {list(motor_types.keys())[:3]}")
    
    # 测试常量
    print("✓ 控制模式常量:")
    print(f"  MODE_POSITION: 0x{hr.MODE_POSITION:02X}")
    print(f"  MODE_VELOCITY: 0x{hr.MODE_VELOCITY:02X}")
    print(f"  MODE_TORQUE: 0x{hr.MODE_TORQUE:02X}")
    print(f"  MODE_POS_VEL_TQE: 0x{hr.MODE_POS_VEL_TQE:02X}")
    
    # 测试机器人创建（没有硬件可能会失败）
    try:
        robot = hr.Robot()
        print(f"✓ 机器人对象创建成功")
        print(f"✓ 机器人已配置 {len(robot.Motors)} 个电机")
        
        # 测试机器人方法（不需要硬件的基本方法）
        print("✓ 机器人方法可用:")
        methods = ['lcm_enable', 'motor_send_2', 'set_stop', 'set_reset']
        for method in methods:
            available = hasattr(robot, method)
            print(f"  {method}: {'✓' if available else '✗'}")
            
    except Exception as e:
        print(f"⚠ 机器人创建失败（没有硬件时预期会失败）: {e}")
    
    # 测试电机反馈数据结构
    try:
        motor_data = hr.MotorBackData()
        motor_data.position = 1.5
        motor_data.velocity = 0.5
        motor_data.torque = 2.0
        print(f"✓ MotorBackData结构体正常工作")
        print(f"  设置位置: {motor_data.position}")
        print(f"  设置速度: {motor_data.velocity}")
        print(f"  设置扭矩: {motor_data.torque}")
    except Exception as e:
        print(f"✗ MotorBackData测试失败: {e}")
    
    print("\nPython绑定测试完成！")

if __name__ == "__main__":
    test_basic_functionality()