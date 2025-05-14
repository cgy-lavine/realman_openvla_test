#!/usr/bin/env python3
"""
RealMan Gen3控制脚本
用于从文件加载DOF数据并控制RealMan Gen3机械臂执行相应动作
"""

import os
import sys
import time
import threading
import numpy as np
import math
from rm_python_wrapper import RoboticArm, rm_thread_mode_e, rm_robot_arm_model_e

# 最大允许等待时间（秒）
TIMEOUT_DURATION = 20

class RealManGen3Controller:
    def __init__(self, robot_ip="192.168.1.10", robot_port=8080):
        """初始化RealMan Gen3控制器"""
        # 初始化机械臂通信，使用双线程模式
        self.robot = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        
        # 创建与机械臂的连接
        print(f"正在连接到机械臂 {robot_ip}:{robot_port}...")
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
        
        time.sleep(2)  # 等待上电完成
        
        # 设置碰撞检测等级
        ret = self.robot.rm_set_collision_state(5)  # 中等碰撞检测灵敏度
        if ret == 0:
            print("碰撞检测已设置")
        else:
            print(f"设置碰撞检测失败，错误码: {ret}")
            
        # 设置最大速度和加速度
        self.robot.rm_set_arm_max_line_speed(0.3)      # 0.3 m/s
        self.robot.rm_set_arm_max_line_acc(1.0)        # 1.0 m/s²
        self.robot.rm_set_arm_max_angular_speed(0.5)   # 0.5 rad/s
        self.robot.rm_set_arm_max_angular_acc(1.0)     # 1.0 rad/s²
        
        # 获取当前末端位姿作为初始值
        ret, arm_state = self.robot.rm_get_current_arm_state()
        if ret == 0:
            self.initial_pose = [
                arm_state['position']['x'],
                arm_state['position']['y'],
                arm_state['position']['z'],
                arm_state['euler']['rx'],
                arm_state['euler']['ry'], 
                arm_state['euler']['rz']
            ]
            print(f"当前末端位姿: {self.initial_pose}")
        else:
            print(f"获取末端位姿失败，错误码: {ret}")
            self.initial_pose = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # 默认值
            
    def move_to_home_position(self):
        """移动机械臂到初始位置"""
        print("正在将机械臂移动到初始位置...")
        
        # 使用机械臂的初始位置
        joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 7DOF机械臂的零位
        
        # 使用关节空间运动指令移动机械臂
        ret = self.robot.rm_movej(
            joint=joints,
            v=20,          # 速度百分比 (20%)
            r=0,           # 交融半径
            connect=0,     # 不与后续轨迹连接
            block=1        # 阻塞模式，等待运动完成
        )
        
        if ret == 0:
            print("已到达初始位置")
            return True
        else:
            print(f"移动到初始位置失败，错误码: {ret}")
            return False
            
    def load_dof_data(self, file_path):
        """从文件加载DOF数据"""
        dof_data = []
        
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    try:
                        # 创建一个字典来存储解析后的数据
                        data = {}
                        
                        # 提取world_vector数据
                        world_start = line.find("'world_vector': array([") + len("'world_vector': array([")
                        world_end = line.find("])", world_start)
                        world_str = line[world_start:world_end]
                        world_vector = [float(x.strip()) for x in world_str.split(',')]
                        data['world_vector'] = world_vector
                        
                        # 提取rotation_delta数据
                        rot_start = line.find("'rotation_delta': array([") + len("'rotation_delta': array([")
                        rot_end = line.find("])", rot_start)
                        rot_str = line[rot_start:rot_end]
                        rotation_delta = [float(x.strip()) for x in rot_str.split(',')]
                        data['rotation_delta'] = rotation_delta
                        
                        # 提取open_gripper数据
                        grip_start = line.find("'open_gripper': array([") + len("'open_gripper': array([")
                        grip_end = line.find("])", grip_start)
                        grip_str = line[grip_start:grip_end]
                        open_gripper = float(grip_str.strip())
                        data['open_gripper'] = open_gripper
                        
                        dof_data.append(data)
                    except Exception as e:
                        print(f"解析行时出错: {e}")
                        continue
            
            print(f"成功从文件加载了 {len(dof_data)} 个动作")
            return dof_data
            
        except Exception as e:
            print(f"加载DOF数据失败: {e}")
            return []
            
    def execute_dof_action(self, data, type="arr"):
        """执行单个DOF动作"""
        print("执行DOF动作...")
        
        # 提取向量
        if type == "dict":
            world_vector = data['world_vector']
            rotation_delta = data['rotation_delta']
            gripper_state = data['open_gripper']
        elif type == "arr":
            world_vector = data[:3]
            rotation_delta = data[3:6]
            gripper_state = data[6]
            
        # 计算目标位姿
        target_pose = [
            self.initial_pose[0] + world_vector[0] * 0.01,  # 转换为米
            self.initial_pose[1] + world_vector[1] * 0.01,  # 转换为米
            self.initial_pose[2] + world_vector[2] * 0.01,  # 转换为米
            self.initial_pose[3] + rotation_delta[0] * -math.pi,  # 转换为弧度
            self.initial_pose[4] + rotation_delta[1] * -math.pi,  # 转换为弧度
            self.initial_pose[5] + rotation_delta[2] * -math.pi   # 转换为弧度
        ]
        
        # 使用笛卡尔空间直线运动
        ret = self.robot.rm_movel(
            pose=target_pose,
            v=30,          # 速度百分比 (30%)
            r=0,           # 交融半径
            connect=0,     # 不与后续轨迹连接
            block=1        # 阻塞模式，等待运动完成
        )
        
        if ret == 0:
            print("位姿移动成功")
        else:
            print(f"位姿移动失败，错误码: {ret}")
            return False
            
        # 设置夹爪状态
        if gripper_state > 0.9:  # 如果值大于0.9，认为是打开
            print("打开夹爪")
            ret = self.robot.rm_set_gripper_release(
                speed=500,      # 夹爪速度
                block=True,     # 阻塞式
                timeout=3       # 超时时间（秒）
            )
        else:
            gripper_position = int((1.0 - gripper_state) * 1000)  # 映射到0-1000范围
            print(f"设置夹爪位置为 {gripper_position}")
            ret = self.robot.rm_set_gripper_position(
                position=gripper_position,  # 夹爪位置
                block=True,     # 阻塞式
                timeout=3       # 超时时间（秒）
            )
            
        if ret == 0:
            print("夹爪操作成功")
            return True
        else:
            print(f"夹爪操作失败，错误码: {ret}")
            return False
    
    def execute_dof_actions(self, dof_data):
        """执行一系列DOF动作"""
        print(f"开始执行 {len(dof_data)} 个DOF动作...")
        
        success_count = 0
        
        # 执行每个动作序列
        for idx, data in enumerate(dof_data):
            print(f"执行动作 {idx+1}/{len(dof_data)}...")
            
            if self.execute_dof_action(data, "dict"):
                success_count += 1
            
            # 短暂延时，让机械臂稳定
            time.sleep(0.5)
            
        print(f"成功执行了 {success_count}/{len(dof_data)} 个动作")
        return success_count == len(dof_data)
    
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
        print("机械臂连接已关闭")
        
        # 销毁所有线程
        self.robot.rm_destory()


def main():
    """主函数"""
    # 解析命令行参数
    robot_ip = "192.168.31.10"  # 默认IP地址
    robot_port = 8080           # 默认端口
    
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    if len(sys.argv) > 2:
        robot_port = int(sys.argv[2])
    
    # 创建控制器
    controller = RealManGen3Controller(robot_ip, robot_port)
    
    try:
        # 移动到初始位置
        success = controller.move_to_home_position()
        
        if success:
            # 加载DOF数据
            dof_file_path = "dof_example.txt"  # 确保文件路径正确
            if not os.path.exists(dof_file_path):
                print(f"错误: 找不到文件 {dof_file_path}")
                return 1
                
            dof_data = controller.load_dof_data(dof_file_path)
            
            if len(dof_data) > 0:
                # 执行动作
                success = controller.execute_dof_actions(dof_data)
                
                # 返回初始位置
                controller.move_to_home_position()
            else:
                print("没有加载到有效的DOF数据")
                success = False
        
        return 0 if success else 1
        
    except Exception as e:
        print(f"程序执行过程中出错: {e}")
        return 1
        
    finally:
        # 确保正确关闭机械臂连接
        controller.shutdown()


if __name__ == "__main__":
    exit(main())