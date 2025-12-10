#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from iros_ros_foundationpose.srv import Reset, crossgetgoalandangle, planandgrippercontrol

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from arm_controller_srvs.srv import crossgetgoalandangle, planandgrippercontrol, PlanToDefault, BackToHome
from tf.transformations import quaternion_from_euler
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
    
def wait_for_enter(point_name):
    """等待用户按下回车键"""
    print(f"\n>>> 准备执行点: {point_name}")
    try:
        input(">>> 按下 [Enter] 键开始执行此点，或按 Ctrl+C 取消...")
        return True
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        return False

def execute_point_by_name(point_name, points_dict, service_name):
    """
    按名称查找并执行单个点位。
    
    :param point_name: 要执行的点位名称 (例如 "MID")
    :param points_dict: 包含所有点位数据的字典
    :param service_name: 要调用的 planandgrippercontrol 服务名
    :return: True (成功) 或 False (失败)
    """
    print(f"\n--- 正在尝试执行点: {point_name} ---")

    # 1. 查找点位
    if point_name not in points_dict:
        print(f"✗ 警告: 点位 '{point_name}' 未在 crossgetgoalandangle 的响应中找到。跳过此点。")
        return True # 注意：这里返回 True 以允许序列继续，False 会终止整个序列

    # 2. 提取数据
    point_data = points_dict[point_name]
    pose = point_data["pose"] # geometry_msgs/Pose

    # !! 假设: joint6_pos 对应 roll_angle
    joint6 = point_data["roll"] 
    
    # !! 警告: gripper_pos 未在 crossgetgoalandangle 中定义
    # !! 您需要在这里设置一个有意义的值 (例如 0.8 = 闭合, 0.0 = 打开)
    # 示例: gripper_val = 0.8 if point_name == "MID" else 0.0
    gripper_val = point_data["pitch"] # <--- !! 修改这里

    print(f"  Pose: {pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f},{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}")
    print(f"  Joint6: {joint6:.3f}, Gripper: {gripper_val:.3f}")
    
    if not wait_for_enter(point_name):
        return False

    # 3. 构建请求
    plan_request = planandgrippercontrol._request_class()
    plan_request.target_pose = pose
    plan_request.gripper_pos = gripper_val
    plan_request.joint6_pos = joint6
    
    # 4. 调用服务
    success, plan_response = call_service(
        service_name,
        planandgrippercontrol,
        plan_request
    )
    
    if success and plan_response.call_success:
        print(f"✓ 点 {point_name} 执行成功")
        # print("...等待 2 秒...")
        # rospy.sleep(2.0) # 成功后暂停
        return True
    else:
        print(f"✗ 点 {point_name} 执行失败")
        print("!! 终止序列 !!") 
        return False # 执行失败，终止整个序列
    
def main():
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 初始化 ROS 节点
    rospy.init_node('arm_control_sequence_custom', anonymous=True)
    
    print_header("机械臂控制序列脚本 (自定义流程)")
    
    total_steps = 4 

    # ----------------------------------------------------
    # --- [关键] 在此定义 *默认* 坐标
    # ----------------------------------------------------
    # 这些值将在标定步骤中被覆盖（如果成功）
    # !! 您必须在此处填入您想要的默认值 !!
    pos_x, pos_y, pos_z = 0.75, 0.0, 0.5
    input_roll  = 0.2  # 示例值，请修改
    input_pitch = -1.56   # 示例值，请修改
    input_yaw   = 0.0  # 示例值，请修改
    
    print("\n" + "-" * 30)
    print(f"正在计算 RPY 转四元数...")
    print(f"输入 RPY (rad): [{input_roll:.5f}, {input_pitch:.5f}, {input_yaw:.5f}]")

    # 3. 转化为四元数
    # quaternion_from_euler 返回顺序通常为 [x, y, z, w]
    q = quaternion_from_euler(input_roll, input_pitch, input_yaw)
    
    ori_x, ori_y, ori_z, ori_w = q[0], q[1], q[2], q[3]
    # ori_x, ori_y, ori_z, ori_w = 0.03534, 0.03534, 0.70629, 0.70629 # 默认姿态

    print(f"计算结果 Quaternion: [{ori_x:.5f}, {ori_y:.5f}, {ori_z:.5f}, {ori_w:.5f}]")
    print("-" * 30 + "\n")

    # 步骤 1: 移动到默认位置
    print_step(1, total_steps, "移动到默认位置")
    
    default_request = PlanToDefault._request_class()
    default_request.plan_to_default = False
    
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
    
    # 步骤 2: 标定（可选）
    print_step(2, total_steps, "执行标定 (将更新 *完整姿态*)")
    print("\n" + "=" * 50)
    print("是否执行标定？")
    print("  [y] - 执行标定 (将从 response.message 更新 *完整姿态*)")
    print("  [n] - 跳过标定 (将使用默认姿态)")
    print("  [q] - 退出程序")
    print("=" * 50)
    
    try:
        user_input = input("请选择 [y/n/q] (或 Ctrl+C 退出): ").strip().lower()
        
        if user_input == 'q':
            print("用户退出")
            return 0
        elif user_input == 'n':
            print("⚠ 跳过标定步骤")
            print(f"  将使用默认姿态 (Pos: x={pos_x:.3f})")
        else:  # 默认执行标定
            cmd_request = Reset._request_class()
            cmd_request.cmd = 1
            success, response = call_service(
                '/foundationpose/service',
                Reset,
                cmd_request
            )
            if success and response.success:
                print("✓ 标定成功")
                
                # --- [新] 尝试从 message (PoseStamped) 更新所有坐标 ---
                try:
                    # 假设 response.message 是一个 geometry_msgs/PoseStamped
                    print(f"  收到 PoseStamped 消息，正在解析...")
                    
                    pos_x = response.message.pose.position.x
                    pos_y = response.message.pose.position.y
                    pos_z = response.message.pose.position.z
                    ori_x = response.message.pose.orientation.x
                    ori_y = response.message.pose.orientation.y
                    ori_z = response.message.pose.orientation.z
                    ori_w = response.message.pose.orientation.w
                    
                    print(f"✓ 已从标定结果更新 *完整姿态*")
                    print(f"  New Pos: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")
                    print(f"  New Ori: x={ori_x:.3f}, y={ori_y:.3f}, z={ori_z:.3f}, w={ori_w:.3f}")
                    
                except AttributeError as e:
                    print(f"✗ 警告: 无法解析标定消息。消息类型不是 PoseStamped？ 错误: {e}")
                    print(f"  将继续使用默认姿态。")
                # --- [结束] ---
                
                rospy.sleep(5.0)
            else:
                print("✗ 标定失败")
                print(f"  将使用默认姿态 (Pos: x={pos_x:.3f})")
                print("是否继续？ [y/n]: ", end="")
                continue_input = input().strip().lower()
                if continue_input != 'y':
                    return 1
                    
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)
    
    # ----------------------------------------------------
    # 步骤 3: [新] 调用 crossgetgoalandangle 服务
    # ----------------------------------------------------
    print_step(3, total_steps, "计算五点规划路径 (crossgetgoalandangle)")

    print("\n" + "=" * 50)
    print("将使用以下坐标作为 'crossgetgoalandangle' 的输入：")
    print(f"  Position: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")
    print(f"  Orientation: x={ori_x:.3f}, y={ori_y:.3f}, z={ori_z:.3f}, w={ori_w:.3f}")
    print("=" * 50 + "\n")

    # 构建 crossgetgoalandangle 请求
    goal_request = crossgetgoalandangle._request_class()
    goal_request.target_pose.header.frame_id = 'link00' # 假设 frame_id
    goal_request.target_pose.header.stamp = rospy.Time.now()
    goal_request.target_pose.pose.position.x = pos_x 
    goal_request.target_pose.pose.position.y = pos_y  
    goal_request.target_pose.pose.position.z = pos_z 
    goal_request.target_pose.pose.orientation.x = ori_x 
    goal_request.target_pose.pose.orientation.y = ori_y 
    goal_request.target_pose.pose.orientation.z = ori_z 
    goal_request.target_pose.pose.orientation.w = ori_w 

    # !! 注意: 替换为你的实际服务名 !!
    SERVICE_GETGOAL = '/arm_controller_node/cross_get_goal_and_angle' 
    print(f"正在调用服务: {SERVICE_GETGOAL}")

    success, goal_response = call_service(
        SERVICE_GETGOAL, 
        crossgetgoalandangle,
        goal_request
    )

    if not (success and goal_response.call_success):
        print(f"✗ 'crossgetgoalandangle' 服务调用失败 (Success: {success}, Response Success: {goal_response.call_success if goal_response else 'N/A'})")
        return 1
    
    print(f"✓ 'crossgetgoalandangle' 服务成功, 收到 {len(goal_response.target_poses)} 个点位")

    # --- [新] 将返回的列表转换为字典
    planned_points = {}
    if len(goal_response.pose_names) != len(goal_response.target_poses):
        print("✗ 错误: 'pose_names' 和 'target_poses' 列表长度不匹配! 无法继续。")
        return 1
        
    for i in range(len(goal_response.pose_names)):
        name = goal_response.pose_names[i]
        planned_points[name] = {
            "pose": goal_response.target_poses[i],
            "pitch": goal_response.pitch_angles[i],
            "roll": goal_response.roll_angles[i]
        }
    
    print(f"✓ 已将 {len(planned_points)} 个点位存入字典: {list(planned_points.keys())}")
    print("=" * 50 + "\n")

    # --- [关键] 我们将使用这个列表作为执行路径
    # 列表 (goal_response.pose_names) 用于 *定义顺序*
    execution_path = goal_response.pose_names
    print(f"✓ 将使用服务返回的路径: {execution_path}")
    print("=" * 50 + "\n")

    # ----------------------------------------------------
    # 步骤 4: [新] 遍历服务返回的路径并应用规则
    # ----------------------------------------------------
    print_step(4, total_steps, "遍历服务返回的路径并应用规则")

    # !! 这是您提供的服务名 !!
    SERVICE_PLANCONTROL = '/arm_controller_node/plan_and_gripper_control'
    
    try:
        # 我们遍历服务返回的路径
        for i in range(len(execution_path)):
            current_point = execution_path[i] # 例如: "TOP1"
            
            # 1. 执行当前点
            print(f"\n[执行路径 {i+1}/{len(execution_path)}]: {current_point}")
            
            # 我们使用 execute_point_by_name，它会去 *字典* (planned_points) 中查找数据
            if not execute_point_by_name(current_point, planned_points, SERVICE_PLANCONTROL):
                print(f"!! 点 {current_point} 执行失败，序列终止 !!")
                return 1 # 终止
            
            # 2. [规则检查] 检查是否需要插入 "MID"
            
            # 检查是否还有下一个点
            if i < len(execution_path) - 1:
                # 这就是您要的: 用 goal_response.pose_names (即 execution_path) 来表示下一个点
                next_point = execution_path[i+1] # 例如: "TOP2"
                
                # 规则 1: TOP1 -> MID -> TOP2
                if current_point == "TOP1" and next_point == "TOP2":
                    print("\n!! 规则触发: TOP1 -> TOP2。正在插入 MID... !!")
                    if not execute_point_by_name("MID", planned_points, SERVICE_PLANCONTROL):
                        print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                        return 1 # 终止

                # 规则 2: LEFT1 -> MID -> LEFT2
                elif current_point == "LEFT1" and next_point == "LEFT2":
                    print("\n!! 规则触发: LEFT1 -> LEFT2。正在插入 MID... !!")
                    if not execute_point_by_name("MID", planned_points, SERVICE_PLANCONTROL):
                        print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                        return 1 # 终止
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)
                
    # 步骤 1: 定义服务名和类型
    SERVICE_NAME = "/arm_controller_node/back_to_home"
    SERVICE_TYPE = BackToHome  # <--- 这是您导入的 Python 类型

    # 步骤 2: 创建请求对象 (Request)
    # 因为您的 .srv 文件有请求部分 (bool back_to_home)
    print("正在创建请求...")
    my_request = BackToHome._request_class()
    
    # 步骤 3: 为请求赋值
    # 您想让它回到 home，所以设置为 True
    my_request.back_to_home = True
    print(f"正在调用服务: {SERVICE_NAME}")
    success, response = call_service(
        SERVICE_NAME, 
        SERVICE_TYPE,
        my_request
    )
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
