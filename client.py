#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用RealMan API将摄像头画面以20FPS的速率传输至OpenVLA，阻塞CLI提示。
"""
import sys
import os
import requests
import json_numpy
json_numpy.patch()
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
import threading
from rm_python_wrapper import RoboticArm, rm_thread_mode_e

# 设置帧率
hz = 20
interval = 1/hz

class RealManController:
    def __init__(self, robot_ip="192.168.1.10", robot_port=8080):
        """初始化RealMan控制器"""
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
        
        # 获取当前末端位姿
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
    
    def execute_dof_action(self, dof_data):
        """执行DOF动作
        
        Args:
            dof_data: DOF数据，可以是数组或字典格式
        """
        print("执行DOF动作...")
        
        # 获取当前位姿作为参考点
        ret, arm_state = self.robot.rm_get_current_arm_state()
        if ret == 0:
            initial_x = arm_state['position']['x']
            initial_y = arm_state['position']['y']
            initial_z = arm_state['position']['z']
            initial_theta_x = arm_state['euler']['rx']
            initial_theta_y = arm_state['euler']['ry']
            initial_theta_z = arm_state['euler']['rz']
        else:
            print(f"获取当前位姿失败，使用初始位姿，错误码: {ret}")
            initial_x = self.initial_pose[0]
            initial_y = self.initial_pose[1]
            initial_z = self.initial_pose[2]
            initial_theta_x = self.initial_pose[3]
            initial_theta_y = self.initial_pose[4]
            initial_theta_z = self.initial_pose[5]
        
        # 提取向量
        # 根据数据类型判断如何提取数据
        if isinstance(dof_data, dict) or hasattr(dof_data, 'keys'):
            # 字典类型数据
            world_vector = dof_data['world_vector']
            rotation_delta = dof_data['rotation_delta']
            gripper_state = dof_data['open_gripper']
        else:
            # 数组类型数据
            world_vector = dof_data[:3]
            rotation_delta = dof_data[3:6]
            gripper_state = dof_data[6]
        
        # 设置目标位姿
        target_pose = [
            initial_x + world_vector[0] * 0.01,  # 转换为米
            initial_y + world_vector[1] * 0.01,  # 转换为米
            initial_z + world_vector[2] * 0.01,  # 转换为米
            initial_theta_x + rotation_delta[0] * -np.pi,  # 转换为弧度
            initial_theta_y + rotation_delta[1] * -np.pi,  # 转换为弧度
            initial_theta_z + rotation_delta[2] * -np.pi   # 转换为弧度
        ]
        
        # 执行笛卡尔空间直线运动
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
        # 判断夹爪状态并执行相应操作
        try:
            if isinstance(gripper_state, np.ndarray):
                gripper_state = float(gripper_state)
                
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
        except Exception as e:
            print(f"夹爪操作出错: {e}")
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
        print("机械臂连接已关闭")
        
        # 销毁所有线程
        self.robot.rm_destory()


def send_log(msg: str):
    """向服务器发送日志"""
    url = "http://localhost:8000/log" 
    try:
        resp = requests.post(url, json={"text": msg})
        print("服务器返回:", resp.json())
    except Exception as e:
        print(f"发送日志失败: {e}")

def send_act(image: np.array, instruction: str):
    """向服务器发送图像和指令，返回动作数据"""
    url = "http://localhost:8000/act" 
    try:
        resp = requests.post(
            url,
            json={"image": image,
                "instruction": instruction,
                "unnorm_key":"roboturk"}
        )
        
        print("服务器返回:", resp.json())
        return resp.json()
    except Exception as e:
        print(f"发送动作请求失败: {e}")
        return None

def crop_center_square(image):
    """裁剪图像中心正方形区域"""
    h, w = image.shape[:2]
    min_dim = min(h, w)
    start_x = w//2 - min_dim//2
    start_y = h//2 - min_dim//2
    return image[start_y: start_y+min_dim, start_x: start_x + min_dim]

def capture_and_process_image():
    """捕获并处理摄像头图像"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头")
    
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("捕获图像失败")
    
    frame = crop_center_square(frame)
    frame_resized = cv2.resize(frame, (256, 256))
    return frame_resized

def main():
    """主函数"""
    print("输入指令；按 Ctrl-C 结束当前指令动作流；Ctrl-D / Ctrl-Z 退出程序。\n")
    
    # 检查摄像头
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            sys.exit("摄像头无法打开")
        cap.release()
    except Exception as e:
        sys.exit(f"摄像头检查失败: {e}")
    
    # 创建RealMan控制器实例
    robot_ip = "192.168.1.10"  # 请修改为您的机械臂IP地址
    robot_port = 8080          # 请修改为您的机械臂端口
    
    # 处理命令行参数
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    if len(sys.argv) > 2:
        robot_port = int(sys.argv[2])
    
    controller = RealManController(robot_ip, robot_port)
    
    try:
        # 移动到初始位置
        controller.move_to_home_position()
        
        # 请求指令
        try:
            instruction = input("👉 instruction> ").strip()
        except (EOFError, KeyboardInterrupt):  # Ctrl-D / Ctrl-Z / Ctrl-C at prompt
            print("\nexit")
            return
        
        print(f"执行: "{instruction}" —— Ctrl-C 退出。\n")
        
        # 持续根据帧率更新第三人称视图
        while True:
            try:
                # 如果需要，可以在这里重新设置指令
                # instruction = "do something"
                
                # 捕获并处理图像
                img = capture_and_process_image()
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # 发送图像和指令到服务器，获取动作数据
                resp = send_act(img_rgb, instruction)
                
                if resp:
                    # 执行动作
                    controller.execute_dof_action(resp)
                
                # 等待一段时间，保持帧率
                time.sleep(3)  # 可以调整为更短的时间，如 interval
                
            except KeyboardInterrupt:
                print("\n动作流已中断")
                break
            except Exception as e:
                print(f"执行过程中发生错误: {e}")
                time.sleep(1)  # 发生错误时暂停一下
                
    except Exception as e:
        print(f"程序执行错误: {e}")
    finally:
        # 确保关闭机械臂连接
        controller.shutdown()

if __name__ == "__main__":
    main()