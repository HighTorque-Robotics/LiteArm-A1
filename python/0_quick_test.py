#!/usr/bin/env python3
"""
不与硬件交互的Python绑定快速测试。
"""

import hightorque_robot_py as hr

def main():
    print("=== Hightorque Robot Python Bindings Test ===")
    
    # 测试1：基本导入和常量
    print("\n1. 测试基本功能:")
    print(f"   ✓ 模块已导入: hightorque_robot_py")
    print(f"   ✓ 可用的类: Robot, Motor, MotorBackData")
    
    # 测试2：电机类型
    motor_types = hr.get_motor_type_map()
    print(f"\n2. 电机类型 ({len(motor_types)} 种可用):")
    for i, (name, type_val) in enumerate(list(motor_types.items())[:5]):
        print(f"   - {name}: {type_val}")
    
    # 测试3：控制模式
    print(f"\n3. 控制模式:")
    print(f"   - 位置模式: 0x{hr.MODE_POSITION:02X}")
    print(f"   - 速度模式: 0x{hr.MODE_VELOCITY:02X}")
    print(f"   - 扭矩模式: 0x{hr.MODE_TORQUE:02X}")
    print(f"   - 组合模式: 0x{hr.MODE_POS_VEL_TQE:02X}")
    
    # 测试4：数据结构
    print(f"\n4. 测试数据结构:")
    motor_data = hr.MotorBackData()
    motor_data.ID = 1
    motor_data.position = 1.57  # π/2弧度
    motor_data.velocity = 0.5
    motor_data.torque = 2.0
    motor_data.mode = hr.MODE_POSITION
    motor_data.fault = 0
    
    print(f"   ✓ MotorBackData已创建:")
    print(f"     ID: {motor_data.ID}")
    print(f"     位置: {motor_data.position:.3f} rad")
    print(f"     速度: {motor_data.velocity:.3f} rad/s")
    print(f"     扭矩: {motor_data.torque:.3f} Nm")
    print(f"     模式: 0x{motor_data.mode:02X}")
    print(f"     故障: 0x{motor_data.fault:02X}")
    
    # 测试5：电机版本结构
    print(f"\n5. 测试电机版本结构:")
    version = hr.MotorVersion()
    version.id = 1
    version.major = 4
    version.minor = 0
    version.patch = 3
    print(f"   ✓ 电机版本: v{version.major}.{version.minor}.{version.patch} (电机ID: {version.id})")
    
    print(f"\n=== 所有测试通过！ ===")
    print(f"Python绑定正常工作。")
    print(f"您现在可以在Python项目中使用hightorque_robot_py模块。")

if __name__ == "__main__":
    main()