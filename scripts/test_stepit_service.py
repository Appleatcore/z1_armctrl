#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys

# 导入 StepIt 的服务定义
try:
    from stepit_ros_msgs.srv import Control, ControlRequest
except ImportError:
    print("✗ 错误: 找不到 stepit_ros_msgs 模块。")
    print("请确保已执行: source ~/workspaces/stepit_ws/install/setup.bash")
    sys.exit(1)


def call_service(service_name, service_type, request=None):
    """调用 ROS 服务并返回结果 (你提供的通用函数)"""
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        service = rospy.ServiceProxy(service_name, service_type)

        if request is None:
            response = service()
        else:
            response = service(request)

        return True, response
    except rospy.ROSException as e:
        print(f"✗ 服务调用超时: {e}")
        return False, None
    except Exception as e:
        print(f"✗ 服务调用失败: {e}")
        return False, None


def send_stepit_command(cmd_str):
    """
    基于 call_service 封装的业务逻辑函数
    """
    service_name = '/control'
    print(f"正在发送请求: '{cmd_str}'")

    # 构造请求对象
    req = ControlRequest()
    req.request = cmd_str

    # 使用你新写的 call_service
    success, response = call_service(service_name, Control, req)

    if success and response:
        if response.status == 0:
            print(f"  ✓ 执行成功! 响应信息: {response.message}")
        else:
            print(f"  ⚠ 执行失败! 状态码: {response.status}, 信息: {response.message}")
    else:
        print(f"  ✗ 无法完成服务调用。")


def test_control_service():
    # 1. 初始化 ROS 节点
    rospy.init_node('test_stepit_control_custom', anonymous=True)

    print("\n--- 开始执行测试序列 ---")

    # 步骤 0: 禁用摇杆
    print("\n[步骤 0] 禁用摇杆控制")
    rospy.sleep(1.0)
    send_stepit_command("Policy/CmdPitch/DisableJoystick")

    # ==========================
    # 步骤 1
    # ==========================
    print("\n[步骤 1] 设置高度 1.0 && 俯仰角 -0.3")
    send_stepit_command("Policy/CmdHeight/SetHeight:1.0")

    print("  ... 等待 1.0 秒 ...")
    rospy.sleep(1.0)

    send_stepit_command("Policy/CmdPitch/SetPitch:-0.3")

    # 步骤间间隔
    print("\n  ... 步骤间间隔 1.0 秒 ...")
    rospy.sleep(1.0)
    input("按回车键继续...")

    # ==========================
    # 步骤 2
    # ==========================
    print("\n[步骤 2] 设置高度 0.6 && 俯仰角 0.3")
    send_stepit_command("Policy/CmdHeight/SetHeight:0.6")

    print("  ... 等待 1.0 秒 ...")
    rospy.sleep(1.0)

    send_stepit_command("Policy/CmdPitch/SetPitch:0.3")

    print("\n--- 测试完成 ---")
    input("按回车键退出...")


if __name__ == "__main__":
    try:
        test_control_service()
    except KeyboardInterrupt:
        print("\n用户中断测试")
