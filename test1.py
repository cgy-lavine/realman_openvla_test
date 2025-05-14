#!/usr/bin/env python3
"""
OpenVLA-RealMan Gen3控制测试脚本
此脚本用于接收OpenVLA传来的7DOF关节数据，并使用这些数据控制RealMan Gen3机械臂

作者: Claude
日期: 2025-05-14
"""

import time
import numpy as np
from rm_python_wrapper import RoboticArm, rm_thread_mode_e, rm_force_type_e, rm_robot_arm_model_e

class OpenVLAControl:
    def __init__(self, robot_ip="192.168.1.10", robot_port=8080):
        """初始化机械臂控制器"""
        # 初始化机械臂通信 - 使用三线程模式以实现最佳性能
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        
        # 创建机械臂连接
        print(f"正在连接机械臂 {robot_ip}:{robot_port}...")
        self.handle = self.robot.rm_create_robot_arm(robot_ip, robot_port, level=1)
        print(f"机械臂已连接，句柄ID: {self.handle.id}")
        
        # 获取机械臂信息
        ret, robot_info = self.robot.rm_get_robot_info()
        if ret == 0:
            print(f"机械臂型号: {robot_info['robot_type']}")
            print(f"自由度: {robot_info['arm_dof']}")
            print(f"控制器版本: {robot_info['robot_controller_version']}")
        else:
            print(f"获取机械臂信息失败，错误码: {ret}")
            
        # 设置机械臂为真实模式
        ret = self.robot.rm_set_arm_run_mode(1)
        if ret == 0:
            print("机械臂已设置为真实模式")
        else:
            print(f"设置机械臂模式失败，错误码: {ret}")
            
        # 上电
        ret = self.robot.rm_set_arm_power(1)
        if ret == 0:
            print("机械臂已上电")
        else:
            print(f"机械臂上电失败，错误码: {ret}")
        
        time.sleep(1)  # 等待上电完成
        
        # 设置碰撞检测等级
        ret = self.robot.rm_set_collision_state(5)  # 中等碰撞检测灵敏度
        if ret == 0:
            print("碰撞检测已设置")
        else:
            print(f"设置碰撞检测失败，错误码: {ret}")
            
        # 获取当前关节角度作为初始值
        ret, joints = self.robot.rm_get_joint_degree()
        if ret == 0:
            self.current_joints = joints
            print(f"当前关节角度: {joints}")
        else:
            print(f"获取关节角度失败，错误码: {ret}")
            self.current_joints = [0.0] * 7  # 默认值
        
        # OpenVLA相关参数初始化
        self.vla_joints = None
        self.last_received_time = time.time()
        self.timeout_threshold = 1.0  # 1秒超时
        self.control_active = False
        self.max_joint_speed = 20.0  # 最大关节速度 (度/秒)
        
    def update_from_vla(self, vla_joints):
        """从OpenVLA更新7DOF关节数据"""
        # 检查输入数据有效性
        if len(vla_joints) != 7:
            print(f"错误：收到的关节数据长度不正确 ({len(vla_joints)})")
            return False
            
        # 更新接收时间
        self.last_received_time = time.time()
        
        # 限制关节移动速度，确保安全
        limited_joints = self.limit_joint_speed(vla_joints)
        
        # 保存当前关节值
        self.vla_joints = limited_joints
        self.control_active = True
        
        return True
        
    def limit_joint_speed(self, target_joints):
        """限制关节移动速度，确保安全"""
        limited_joints = []
        for i, (current, target) in enumerate(zip(self.current_joints, target_joints)):
            # 计算关节差值
            diff = target - current
            
            # 限制最大运动幅度
            if abs(diff) > self.max_joint_speed:
                limited = current + np.sign(diff) * self.max_joint_speed
            else:
                limited = target
                
            limited_joints.append(limited)
            
        return limited_joints
    
    def move_arm(self):
        """根据最新的VLA数据移动机械臂"""
        if not self.control_active:
            return False
            
        # 检查数据是否超时
        if time.time() - self.last_received_time > self.timeout_threshold:
            print("VLA数据超时，停止控制")
            self.control_active = False
            return False
            
        # 使用关节空间运动指令移动机械臂
        ret = self.robot.rm_movej(
            joint=self.vla_joints,
            v=30,          # 速度百分比 (30%)
            r=0,           # 交融半径
            connect=0,     # 不与后续轨迹连接
            block=0        # 非阻塞模式
        )
        
        if ret == 0:
            # 更新当前关节位置
            self.current_joints = self.vla_joints
            return True
        else:
            print(f"机械臂移动失败，错误码: {ret}")
            return False
    
    def move_arm_canfd(self):
        """使用CANFD透传方式控制机械臂（实时性更好）"""
        if not self.control_active:
            return False
            
        # 检查数据是否超时
        if time.time() - self.last_received_time > self.timeout_threshold:
            print("VLA数据超时，停止控制")
            self.control_active = False
            return False
            
        # 使用CANFD透传控制机械臂
        ret = self.robot.rm_movej_canfd(
            joint=self.vla_joints,
            follow=True,           # 高跟随模式
            trajectory_mode=1,     # 曲线拟合模式
            radio=50               # 中等平滑度
        )
        
        if ret == 0:
            # 更新当前关节位置
            self.current_joints = self.vla_joints
            return True
        else:
            print(f"CANFD透传控制失败，错误码: {ret}")
            return False
    
    def shutdown(self):
        """关闭机械臂连接"""
        # 停止机械臂运动
        self.robot.rm_set_arm_slow_stop()
        time.sleep(0.5)
        
        # 断电
        self.robot.rm_set_arm_power(0)
        print("机械臂已断电")
        
        # 删除机械臂连接
        self.robot.rm_delete_robot_arm()
        self.robot.rm_destory()
        print("机械臂连接已关闭")


def simulate_vla_data():
    """生成模拟的OpenVLA数据，用于测试"""
    # 生成一个简单的正弦波模式关节角
    t = time.time()
    joints = [
        30 * np.sin(0.2 * t),              # 关节1
        20 * np.sin(0.3 * t) + 45,         # 关节2
        15 * np.sin(0.4 * t) - 45,         # 关节3
        25 * np.sin(0.25 * t),             # 关节4
        20 * np.sin(0.5 * t),              # 关节5
        15 * np.sin(0.6 * t),              # 关节6
        10 * np.sin(0.7 * t),              # 关节7
    ]
    return joints


def main():
    """主函数"""
    # 创建控制器
    controller = OpenVLAControl(robot_ip="192.168.1.10", robot_port=8080)
    
    try:
        print("启动VLA-Gen3控制测试...")
        print("按Ctrl+C退出")
        
        # 主循环
        while True:
            # 在实际应用中，这里应该从OpenVLA接收数据
            # 这里使用模拟数据进行测试
            vla_joints = simulate_vla_data()
            
            # 更新关节数据
            controller.update_from_vla(vla_joints)
            
            # 移动机械臂（可选择常规movej或canfd透传方式）
            # controller.move_arm()        # 常规方式
            controller.move_arm_canfd()    # CANFD透传方式（实时性更好）
            
            # 短暂延时
            time.sleep(0.01)  # 10ms循环周期
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 确保正确关闭机械臂连接
        controller.shutdown()


if __name__ == "__main__":
    main()