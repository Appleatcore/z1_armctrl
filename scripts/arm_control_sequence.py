#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 假设包名叫 foundationpose_srvs，srv 文件名 Calibrate.srv
from iros_ros_foundationpose.srv import Reset   # 改成你的实际包名/类型名
"""
机械臂控制序列脚本
功能：回到home位置 -> 移动到默认位置 -> 标定 -> 获取坐标 -> 执行五点规划
每个步骤之间停顿适当时间
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from arm_controller_srvs.srv import PlanTofivepoint, PlanToDefault
import time
import sys
import signal

def signal_handler(sig, frame):
    """处理 Ctrl+C 信号"""
    print("\n\n接收到中断信号，退出程序...")
    sys.exit(0)

def print_header(text):
    """打印带边框的标题"""
    print("\n" + "=" * 50)
    print(text)
    print("=" * 50)

def print_step(step_num, total_steps, description):
    """打印步骤信息"""
    print(f"\n[步骤 {step_num}/{total_steps}] {description}...")

def call_service(service_name, service_type, request=None):
    """调用 ROS 服务并返回结果"""
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

def get_tf_transform(tf_buffer, source_frame, target_frame, timeout=5.0):
    """获取 TF 变换"""
    print(f"正在监听 tf 变换 ({source_frame} -> {target_frame})，持续{timeout}秒...")
    
    start_time = rospy.Time.now()
    last_transform = None
    
    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            # 获取最新的变换
            transform = tf_buffer.lookup_transform(
                source_frame, 
                target_frame, 
                rospy.Time(0),  # 获取最新的变换
                rospy.Duration(0.5)
            )
            
            last_transform = transform
            
            # 打印当前变换
            trans = transform.transform.translation
            rot = transform.transform.rotation
            print(f"At time {transform.header.stamp.to_sec():.3f}")
            print(f"- Translation: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
            print(f"- Rotation: in Quaternion [{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]")
            
            rospy.sleep(0.5)  # 每0.5秒更新一次
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            print(f"警告: {e}")
            rospy.sleep(0.1)
            continue
    
    return last_transform

def main():
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 初始化 ROS 节点
    rospy.init_node('arm_control_sequence', anonymous=True)
    
    print_header("机械臂控制序列脚本")
    
    # 创建 TF 监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 等待 TF 树建立
    rospy.sleep(1.0)
    
    # # 步骤 1: 回到 home 位置（可选）
    # print_step(1, 5, "回到home位置")
    # success, response = call_service(
    #     '/arm_controller_node/back_to_home',
    #     Trigger
    # )
    # if success and response.success:
    #     print("✓ 回到home位置成功")
    # else:
    #     print("✗ 回到home位置失败")
    #     return 1
    # rospy.sleep(5.0)
    
    # 步骤 2: 移动到默认位置
    print_step(2, 5, "移动到默认位置")
    
    # 构建 PlanToDefault 请求
    default_request = PlanToDefault._request_class()
    default_request.plan_to_default = False  # 或者 True，根据需要
    
    success, response = call_service(
        '/arm_controller_node/plan_to_default',
        PlanToDefault,
        default_request
    )
    if success and response.call_success:
        print("✓ 移动到默认位置成功")
    else:
        print("✗ 移动到默认位置失败")
        return 1
    # rospy.sleep(7.0)
    
    # 步骤 3: 标定（可选）
    print_step(3, 5, "执行标定")
    print("\n" + "=" * 50)
    print("是否执行标定？")
    print("  [y] - 执行标定")
    print("  [n] - 跳过标定，直接获取坐标")
    print("  [q] - 退出程序")
    print("=" * 50)
    


    try:
        user_input = input("请选择 [y/n/q] (或 Ctrl+C 退出): ").strip().lower()
        
        if user_input == 'q':
            print("用户退出")
            return 0
        elif user_input == 'n':
            print("⚠ 跳过标定步骤")
        else:  # 默认执行标定
            cmd_request = Reset._request_class()  # 创建请求对象
            cmd_request.cmd = 1                   # 传入数值 1
            success, response = call_service(
                '/foundationpose/service',
                Reset,
                cmd_request
            )
            if success and response.success:
                print("✓ 标定成功")
                print(f"  消息: {response.message}")
                rospy.sleep(5.0)
            else:
                print("✗ 标定失败")
                print("是否继续？ [y/n]: ", end="")
                continue_input = input().strip().lower()
                if continue_input != 'y':
                    return 1
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)
    
    # 步骤 4: 获取物体坐标
    print_step(4, 5, "获取物体坐标")
    
    transform = get_tf_transform(
        tf_buffer,
        'link00',
        'estimated_object',
        timeout=5.0
    )
    
    if transform is not None:
        print("✓ 坐标获取完成")
        
        # 提取坐标
        pos_x = transform.transform.translation.x
        pos_y = transform.transform.translation.y
        pos_z = transform.transform.translation.z
        ori_x = transform.transform.rotation.x
        ori_y = transform.transform.rotation.y
        ori_z = transform.transform.rotation.z
        ori_w = transform.transform.rotation.w
        
        print("\n" + "=" * 50)
        print("解析到的坐标：")
        print(f"  Position: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")
        print(f"  Orientation: x={ori_x:.3f}, y={ori_y:.3f}, z={ori_z:.3f}, w={ori_w:.3f}")
        print("=" * 50)
    else:
        print("⚠ 警告：未能获取到有效的坐标数据，使用默认值")
        pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
        ori_x, ori_y, ori_z, ori_w = 0.0, 0.0, 0.0, 1.0
    
    # 用户确认
    print("\n" + "=" * 50)
    print("请确认上面的坐标信息是否正确")
    print("按回车键继续执行五点规划，或按 Ctrl+C 取消")
    print("=" * 50)
    
    try:
        input("按回车继续...")
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)
    
    # 步骤 5: 执行五点规划
    print_step(5, 5, "执行五点规划")
    
    print("\n" + "=" * 50)
    print("将使用以下坐标执行五点规划：")
    print(f"  Position: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")
    print(f"  Orientation: x={ori_x:.3f}, y={ori_y:.3f}, z={ori_z:.3f}, w={ori_w:.3f}")
    print("=" * 50 + "\n")
    
    # 构建请求
    request = PlanTofivepoint._request_class()
    request.target_pose.header.frame_id = 'link00'
    request.target_pose.header.stamp = rospy.Time.now()
    request.target_pose.pose.position.x = pos_x
    request.target_pose.pose.position.y = pos_y
    request.target_pose.pose.position.z = pos_z
    request.target_pose.pose.orientation.x = ori_x
    request.target_pose.pose.orientation.y = ori_y
    request.target_pose.pose.orientation.z = ori_z
    request.target_pose.pose.orientation.w = ori_w
    
    success, response = call_service(
        '/arm_controller_node/plan_to_five_point',
        PlanTofivepoint,
        request
    )
    
    if success and response.call_success:
        print("✓ 五点规划执行成功")
    else:
        print("✗ 五点规划执行失败")
        return 1
    
    print_header("所有步骤执行完成！")
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        print("\n程序被中断")
        sys.exit(0)
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)
    except SystemExit:
        # 正常退出，不需要处理
        raise
