#!/usr/bin/env python3
"""
带有改进信号处理和清理功能的hightorque_robot包装类。
即使底层C++线程在I/O操作上被阻塞，此包装器也能确保正确清理。
"""

import sys
import os
import signal
import threading
import atexit
import time

# 将python目录添加到路径中以导入模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import hightorque_robot_py as hr
except ImportError as e:
    print(f"Error importing hightorque_robot_py: {e}")
    sys.exit(1)


class RobotWrapper:
    """具有更好清理处理的机器人包装类"""
    
    def __init__(self):
        self._robot = None
        self._shutdown_flag = threading.Event()
        self._cleanup_done = False
        
        # 在退出时注册清理函数
        atexit.register(self._cleanup)
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """处理信号以实现干净关闭"""
        print(f"\n\n接收到信号 {signum}。开始关闭...")
        self._shutdown_flag.set()
        
        # 给一点时间进行优雅关闭
        time.sleep(0.5)
        
        # 如果尚未清理则强制退出
        if not self._cleanup_done:
            print("强制退出...")
            os._exit(0)
    
    def _cleanup(self):
        """清理资源"""
        if self._cleanup_done:
            return
            
        self._cleanup_done = True
        
        if self._robot is not None:
            print("\n正在清理机器人资源...")
            try:
                self._robot.set_stop()
                self._robot.motor_send_2()
                time.sleep(0.1)
            except:
                pass
            
            # 删除机器人对象以触发其析构函数
            self._robot = None
            
    def __enter__(self):
        """上下文管理器进入"""
        self._robot = hr.Robot()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self._cleanup()
        
    def __getattr__(self, name):
        """将属性访问委托给包装的机器人"""
        if self._robot is None:
            raise RuntimeError("机器人未初始化。请使用 'with RobotWrapper() as robot:' 语法")
        return getattr(self._robot, name)
    
    @property
    def shutdown_requested(self):
        """检查是否请求关闭"""
        return self._shutdown_flag.is_set()
    
    @property
    def robot(self):
        """获取底层机器人对象"""
        return self._robot
    
    def initialize(self):
        """如果未使用上下文管理器则初始化机器人"""
        if self._robot is None:
            self._robot = hr.Robot()
        return self._robot


def example_with_wrapper():
    """使用包装类的示例"""
    print("机器人包装器示例")
    print("=" * 40)
    
    with RobotWrapper() as robot:
        robot.lcm_enable()
        
        motor_count = len(robot.Motors)
        print(f"发现 {motor_count} 个电机")
        
        if motor_count == 0:
            print("未发现电机。")
            return
            
        print("按Ctrl+C停止...")
        
        angle = 0.5
        count = 0
        
        while not robot.shutdown_requested:
            # 控制电机
            for i, motor in enumerate(robot.Motors):
                if i < motor_count // 2:
                    motor.pos_vel_MAXtqe(angle, 0.1, 10.0)
                else:
                    motor.pos_vel_MAXtqe(-angle, 0.1, 10.0)
            
            robot.motor_send_2()
            
            # 偶尔打印状态
            if count % 10 == 0:
                for motor in robot.Motors:
                    state = motor.get_current_motor_state()
                    print(f"电机 {state.ID}: pos={state.position:.3f}")
            
            count += 1
            if count >= 250:
                count = 0
                angle *= -1
                print(f"反转方向，角度: {angle}")
            
            # 更频繁地检查关闭请求
            for _ in range(10):
                if robot.shutdown_requested:
                    break
                time.sleep(0.05)
    
    print("示例成功完成！")


if __name__ == "__main__":
    example_with_wrapper()