import time
import sys
import os
import numpy as np

# 将python目录添加到路径中以导入模块
# 需要添加上一级的python目录，而不是当前的LiteArm目录
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

try:
    import hightorque_robot_py as hr
except ImportError as e:
    print(f"导入hightorque_robot_py时出错: {e}")
    print("请确保您已使用CMake构建了Python绑定")
    sys.exit(1)

class LiteArm(hr.Robot):  # 继承自hr.Robot
    def __init__(self, config_path=None):
        # 调用父类的初始化方法
        super().__init__()
        
        #夹爪电机id
        self.gripper_id = len(self.Motors)
        print("初始化LiteArm...")
        # 获取电机数量(不包含夹爪电机)
        self.motor_count = len(self.Motors) - 1
        print(f"发现 {self.motor_count} 个电机")
        if self.motor_count == 0:
            print("未发现电机。请检查您的配置和连接。")
            return
        # 打印电机信息
        for i, motor in enumerate(self.Motors):
            print(f"Motor {i}: ID={motor.get_motor_id()}, "
                f"Type={motor.get_motor_enum_type()}, "
                f"Name={motor.get_motor_name()}")

    def check_position_reached(self, target_positions, gripper=None, tolerance=0.1):
        """检查前关节位置是否到达"""
        self.send_get_motor_state_cmd()
        self.motor_send_2()
        all_reached = True
        position_errors = []
        
        # 检查前6个关节
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            error = abs(state.position - target_positions[i])
            position_errors.append(error)
            if error > tolerance:
                all_reached = False
        
        # 检查夹爪（如果提供参数）
        if gripper is not None:
            gripper_state = self.Motors[6].get_current_motor_state()
            gripper_error = abs(gripper_state.position - gripper['pos'])
            position_errors.append(gripper_error)
            if gripper_error > tolerance:
                all_reached = False
        
        return all_reached, position_errors
    
    def wait_for_position(self, target_positions, gripper=None, tolerance=0.01, timeout=15.0):
        """等待位置到达"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            reached, _ = self.check_position_reached(target_positions, gripper, tolerance)
            if reached:
                return True
            time.sleep(0.02)
        return False

    def get_current_state(self):
        """获取当前关节状态"""
        self.send_get_motor_state_cmd()
        self.motor_send_2()
        state = []
        for i in range(self.motor_count):
            motor_state = self.Motors[i].get_current_motor_state()
            state.append(motor_state)
        return state

    def get_current_pos(self):
        """获取当前关节角度"""
        self.send_get_motor_state_cmd()
        self.motor_send_2()
        joint_angles = []
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            joint_angles.append(state.position)
        return joint_angles
    
    def get_current_vel(self):
        """获取当前关节速度"""
        self.send_get_motor_state_cmd()
        self.motor_send_2()
        joint_velocities = []
        for i in range(self.motor_count):
            state = self.Motors[i].get_current_motor_state()
            joint_velocities.append(state.velocity)
        return joint_velocities

    def pos_vel_MAXtqe(self, pos, vel, max_tqu=10.0, gripper=None, iswait=False, tolerance=0.1, timeout=15.0):
        # 检查关节数量（除了夹爪电机）
        if not (len(pos) == len(vel) == len(max_tqu) == self.motor_count):
            raise ValueError(f"关节参数长度必须为{self.motor_count}")
        
        # 控制关节（除了夹爪电机）
        for i in range(self.motor_count):
            motor = self.Motors[i]
            motor.pos_vel_MAXtqe(pos[i], vel[i], max_tqu[i])
        
        # 控制夹爪（如果提供参数）
        if gripper is not None:
            if not all(key in gripper for key in ['pos', 'vel', 'max_tqu']):
                raise ValueError("夹爪参数必须包含 'pos', 'vel', 'max_tqu'")
            gripper_motor = self.Motors[self.gripper_id-1]
            gripper_motor.pos_vel_MAXtqe(gripper['pos'], gripper['vel'], gripper['max_tqu'])
        
        self.motor_send_2()
        if iswait:
            return self.wait_for_position(pos, gripper, tolerance, timeout)
        return True

    def pos_vel_tqe_kp_kd(self, pos, vel, tqe, kp, kd, gripper=None, iswait=False, tolerance=0.1, timeout=10.0):
        # 检查关节数量（除了夹爪电机）
        params = [pos, vel, tqe, kp, kd]
        if not all(len(p) == self.motor_count for p in params):
            raise ValueError(f"前6个关节参数长度必须为{self.motor_count}")
        
        # 控制关节（除了夹爪电机）
        for i in range(self.motor_count):
            motor = self.Motors[i]
            motor.pos_vel_tqe_kp_kd(pos[i], vel[i], tqe[i], kp[i], kd[i])
        
        # 控制夹爪（如果提供参数）
        if gripper is not None:
            required_keys = ['pos', 'vel', 'tqe', 'kp', 'kd']
            if not all(key in gripper for key in required_keys):
                raise ValueError(f"夹爪参数必须包含 {required_keys}")
            gripper_motor = self.Motors[self.gripper_id-1]
            gripper_motor.pos_vel_tqe_kp_kd(
                gripper['pos'], 
                gripper['vel'], 
                gripper['tqe'],
                gripper['kp'],
                gripper['kd']
            )
        
        self.motor_send_2()
        if iswait:
            return self.wait_for_position(pos, gripper, tolerance, timeout)
        return True
    
    def septic_interpolation(self, start_pos, end_pos, duration, current_time):
        """七次多项式插值轨迹生成（速度、加速度、加加速度连续）(起始终止的速度和加速度都为0)"""
        if current_time <= 0:
            return start_pos, [0.0] * len(start_pos), [0.0] * len(start_pos)
        if current_time >= duration:
            return end_pos, [0.0] * len(end_pos), [0.0] * len(end_pos)
        
        # 归一化时间
        t = current_time / duration
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        t6 = t5 * t
        t7 = t6 * t
        
        # 七次多项式系数 (位置)
        a0 = 1 - 35*t4 + 84*t5 - 70*t6 + 20*t7
        a1 = 35*t4 - 84*t5 + 70*t6 - 20*t7
        
        # 一阶导数系数 (速度)
        da0 = -140*t3 + 420*t4 - 420*t5 + 140*t6
        da1 = 140*t3 - 420*t4 + 420*t5 - 140*t6
        
        # 二阶导数系数 (加速度)
        dda0 = -420*t2 + 1680*t3 - 2100*t4 + 840*t5
        dda1 = 420*t2 - 1680*t3 + 2100*t4 - 840*t5
        
        # 计算位置、速度、加速度
        pos = []
        vel = []
        acc = []
        for i in range(len(start_pos)):
            # 位置
            p = a0 * start_pos[i] + a1 * end_pos[i]
            pos.append(p)
            # 速度
            v = (da0 * start_pos[i] + da1 * end_pos[i]) / duration
            vel.append(v)
            # 加速度
            a = (dda0 * start_pos[i] + dda1 * end_pos[i]) / (duration * duration)
            acc.append(a)
        
        return pos, vel, acc
    
    def septic_interpolation_with_velocity(self, start_pos, end_pos, start_vel, end_vel, duration, current_time):
        """
        七次多项式插值轨迹生成（指定起始和终止速度）
        可以实现非零速度的平滑过渡
        """
        if current_time <= 0:
            return start_pos, start_vel, [0.0] * len(start_pos)
        if current_time >= duration:
            return end_pos, end_vel, [0.0] * len(end_pos)
        
        # 归一化时间
        t = current_time / duration
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        t6 = t5 * t
        t7 = t6 * t
        
        # 七次多项式系数（考虑速度边界条件）
        # p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
        # 边界条件：p(0)=p0, p(1)=p1, v(0)=v0, v(1)=v1, a(0)=0, a(1)=0, j(0)=0, j(1)=0
        
        pos = []
        vel = []
        acc = []
        
        for i in range(len(start_pos)):
            p0 = start_pos[i]
            p1 = end_pos[i]
            v0 = start_vel[i] * duration  # 转换为归一化速度
            v1 = end_vel[i] * duration
            
            # 系数计算（满足8个边界条件）
            a0 = p0
            a1 = v0
            a2 = 0  # 起始加速度为0
            a3 = 0  # 起始加加速度为0
            
            # 通过矩阵求解得到的系数
            a4 = 35*(p1 - p0) - 20*v0 - 15*v1
            a5 = -84*(p1 - p0) + 45*v0 + 39*v1
            a6 = 70*(p1 - p0) - 36*v0 - 34*v1
            a7 = -20*(p1 - p0) + 10*v0 + 10*v1
            
            # 计算位置
            p = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5 + a6*t6 + a7*t7
            pos.append(p)
            
            # 计算速度（一阶导数）
            v = (a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4 + 6*a6*t5 + 7*a7*t6) / duration
            vel.append(v)
            
            # 计算加速度（二阶导数）
            a = (2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3 + 30*a6*t4 + 42*a7*t5) / (duration * duration)
            acc.append(a)
        
        return pos, vel, acc
    
            
if __name__ == "__main__":
    LiteArm = LiteArm()