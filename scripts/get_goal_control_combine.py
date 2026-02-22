#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from iros_ros_foundationpose.srv import Reset

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from arm_controller_srvs.srv import (
    getgoalandangle,
    planandgrippercontrol,
    PlanToDefault,
    BackToHome,
)
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import sys
import signal
import numpy as np


import numpy as np

# 配置参数
CALIBRATION_SAMPLES = 5  # 标定时采集的样本数量（用于均值滤波）
CALIBRATION_SAMPLE_INTERVAL = 0.5  # 采样间隔（秒）


def quaternion_average(quaternions):
    """
    计算多个四元数的平均值
    使用 Markley 等人提出的方法

    参数:
        quaternions: list of tuples (x, y, z, w)

    返回:
        tuple: 平均四元数 (x, y, z, w)
    """
    if not quaternions:
        return (0, 0, 0, 1)

    # 将四元数列表转换为 numpy 数组
    Q = np.array(quaternions)

    # 构建 4x4 矩阵 M
    M = np.zeros((4, 4))
    for q in Q:
        q = np.array([q[3], q[0], q[1], q[2]])  # 转换为 (w, x, y, z) 格式
        M += np.outer(q, q)

    M = M / len(Q)

    # 计算特征值和特征向量
    eigenvalues, eigenvectors = np.linalg.eig(M)

    # 选择最大特征值对应的特征向量
    max_eigenvector = eigenvectors[:, np.argmax(eigenvalues)]

    # 返回格式为 (x, y, z, w)
    return (
        max_eigenvector[1],
        max_eigenvector[2],
        max_eigenvector[3],
        max_eigenvector[0],
    )


def collect_calibration_samples(
    tf_buffer, source_frame, target_frame, num_samples=5, interval=0.5, time_threshold=0.5
):
    """
    采集多个标定样本并进行均值滤波

    参数:
        tf_buffer: TF2 缓冲区
        source_frame: 源坐标系
        target_frame: 目标坐标系
        num_samples: 采集样本数量
        interval: 采样间隔（秒）
        time_threshold: 有效数据时间阈值（秒）

    返回:
        tuple: (pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w) 或 None
    """
    print(f"\n正在采集 {num_samples} 个标定样本...")

    positions = []
    orientations = []

    for i in range(num_samples):
        try:
            # 获取 TF 变换
            transform = tf_buffer.lookup_transform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            # 检查数据的新鲜度
            current_time = rospy.Time.now()
            data_time = transform.header.stamp
            time_diff = (current_time - data_time).to_sec()

            # 只有当数据足够新鲜时才使用
            if abs(time_diff) < time_threshold:
                # 提取位置
                pos = transform.transform.translation
                positions.append([pos.x, pos.y, pos.z])

                # 提取姿态（四元数）
                ori = transform.transform.rotation
                orientations.append((ori.x, ori.y, ori.z, ori.w))

                print(
                    f"  样本 {i+1}/{num_samples}: 位置=[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}], 延迟={time_diff:.4f}s"
                )
            else:
                print(
                    f"  ⚠ 样本 {i+1} 数据过时（延迟: {time_diff:.4f}s > 阈值 {time_threshold}s），跳过"
                )
                continue

            # 等待采样间隔
            if i < num_samples - 1:
                rospy.sleep(interval)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            print(f"  ⚠ 样本 {i+1} 采集失败: {e}")
            continue

    # 检查是否采集到足够的样本
    if len(positions) < num_samples / 2:
        print(f"✗ 采集样本不足（仅 {len(positions)}/{num_samples}），标定失败")
        return None

    # 计算位置均值
    positions_array = np.array(positions)
    pos_mean = np.mean(positions_array, axis=0)
    pos_std = np.std(positions_array, axis=0)

    # 计算四元数平均值
    ori_mean = quaternion_average(orientations)

    print(f"\n✓ 采集完成！共 {len(positions)} 个有效样本")
    print(f"  位置均值: [{pos_mean[0]:.6f}, {pos_mean[1]:.6f}, {pos_mean[2]:.6f}]")
    print(f"  位置标准差: [{pos_std[0]:.6f}, {pos_std[1]:.6f}, {pos_std[2]:.6f}]")

    return (
        pos_mean[0],
        pos_mean[1],
        pos_mean[2],
        ori_mean[0],
        ori_mean[1],
        ori_mean[2],
        ori_mean[3],
    )


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
        # 如果嫌麻烦，可以注释掉下面这行 input，直接 return True
        input(">>> 按下 [Enter] 键开始执行此点，或按 Ctrl+C 取消...")
        return True
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        return False


def get_tf_transform(
    tf_buffer, source_frame, target_frame, timeout=5.0, time_threshold=0.5
):
    """
    获取 TF 变换，并检查数据的新鲜度。
    只有当 TF 数据的时间戳与当前时间相差在 time_threshold 以内时，才视为有效。
    """
    print(f"正在监听 tf 变换 ({source_frame} -> {target_frame})，持续{timeout}秒...")

    start_time = rospy.Time.now()
    last_valid_transform = None  # 改名：存储最后一次【有效】的变换

    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            # 1. 获取最新的变换 (Time(0) 返回缓冲区里最新的一帧，但不保证是现在的)
            transform = tf_buffer.lookup_transform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(0.1)
            )

            # 2. 【核心修改】计算时间差 (当前时间 - 数据时间)
            current_time = rospy.Time.now()
            data_time = transform.header.stamp
            time_diff = (current_time - data_time).to_sec()

            # 3. 检查数据是否“新鲜”
            if abs(time_diff) < time_threshold:
                last_valid_transform = transform  # 数据有效，更新

                # 打印当前变换 (标记为有效)
                trans = transform.transform.translation
                rot = transform.transform.rotation
                print(
                    f"✓ [有效] 延迟: {time_diff:.4f}s | Stamp: {data_time.to_sec():.3f}"
                )
                print(f"  - Trans: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
                # print(f"  - Rot:   [{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]")
            else:
                # 数据过时，不更新 last_valid_transform
                print(
                    f"✗ [丢弃] 数据过时! 延迟: {time_diff:.4f}s > 阈值 {time_threshold}s"
                )

            rospy.sleep(0.1)  # 循环间隔

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            print(f"警告: {e}")
            rospy.sleep(0.1)
            continue

    if last_valid_transform is None:
        print("⚠ 警告: 在超时时间内未获取到任何【新鲜】的 TF 数据！")
        return None

    return last_valid_transform


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
        print(
            f"✗ 警告: 点位 '{point_name}' 未在 getgoalandangle 的响应中找到。跳过此点。"
        )
        return True  # 注意：这里返回 True 以允许序列继续，False 会终止整个序列

    # 2. 提取数据
    point_data = points_dict[point_name]
    pose = point_data["pose"]  # geometry_msgs/Pose

    # !! 假设: joint6_pos 对应 roll_angle
    joint6 = point_data["roll"]

    # !! 警告: gripper_pos 未在 getgoalandangle 中定义
    # !! 您需要在这里设置一个有意义的值 (例如 0.8 = 闭合, 0.0 = 打开)
    # 示例: gripper_val = 0.8 if point_name == "MID" else 0.0
    gripper_val = point_data["pitch"]  # <--- !! 修改这里

    print(
        f"  Pose: {pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f},{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}"
    )
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
        service_name, planandgrippercontrol, plan_request
    )

    if success and plan_response.call_success:
        print(f"✓ 点 {point_name} 执行成功")
        # print("...等待 2 秒...")
        # rospy.sleep(2.0) # 成功后暂停
        return True
    else:
        print(f"✗ 点 {point_name} 执行失败")
        print("!! 终止序列 !!")
        return False  # 执行失败，终止整个序列


def execute_with_recovery(point_name, points_dict, service_name):
    """尝试执行，如果失败则调用default服务并重试一次"""
    # 1. 第一次尝试
    if execute_point_by_name(point_name, points_dict, service_name):
        return True

    # 2. 如果失败，调用default服务
    print(f"\n⚠ 警告: 点 {point_name} 首次执行失败！")
    print(f"➜ 正在尝试: 调用 plan_to_default 服务并重试...")

    SERVICE_NAME = "/arm_controller_node/plan_to_default"
    SERVICE_TYPE = PlanToDefault
    default_req = PlanToDefault._request_class()
    default_req.plan_to_default = True
    h_succ, response = call_service(SERVICE_NAME, SERVICE_TYPE, default_req)

    if not h_succ or not response.call_success:
        print("✗ 严重错误: plan_to_default 服务调用失败，放弃重试。")
        return False

    print("✓ plan_to_default 服务调用成功")
    rospy.sleep(1.0)

    # 3. 第二次尝试
    print(f"➜ 正在重试: 前往 {point_name} ...")
    if execute_point_by_name(point_name, points_dict, service_name):
        print(f"✓ 重试成功: 点 {point_name} 执行完成")
        return True
    else:
        print(f"✗ 重试失败: 点 {point_name} 无法到达")
        return False


def main():
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化 ROS 节点
    rospy.init_node('arm_control_sequence_custom', anonymous=True)

    print_header("机械臂控制序列脚本 (自定义流程)")

    # 创建 TF 监听器（用于坐标转换）
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)  # 等待 TF 树建立

    total_steps = 4

    # ----------------------------------------------------
    # --- [关键] 在此定义 *默认* 坐标
    # ----------------------------------------------------
    # 这些值将在标定步骤中被覆盖（如果成功）
    # !! 您必须在此处填入您想要的默认值 !!
    pos_x, pos_y, pos_z = 0.85, 0.0, 0.3
    input_roll = 0.0  # 示例值，请修改
    input_pitch = 0.0  # 示例值，请修改
    input_yaw = 1.5708  # 示例值，请修改

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
    # ----------------------------------------------------

    # 步骤 1: 移动到默认位置
    print_step(1, total_steps, "移动到默认位置")

    default_request = PlanToDefault._request_class()
    default_request.plan_to_default = False

    success, response = call_service(
        '/arm_controller_node/plan_to_default', PlanToDefault, default_request
    )
    if success and response.call_success:
        print("✓ 移动到默认位置成功")
    else:
        print("✗ 移动到默认位置失败")
        return 1
    rospy.sleep(1.0)

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
            # 标定循环，支持失败后重试
            calibration_done = False
            while not calibration_done:
                # 1. 先调用标定服务
                cmd_request = Reset._request_class()
                cmd_request.cmd = 1
                success, response = call_service(
                    '/foundationpose/service', Reset, cmd_request
                )
                if success and response.success:
                    print("✓ 标定成功")

                    # 2. 标定成功后，等待 TF 树更新
                    print("  等待 TF 树更新...")
                    rospy.sleep(2.0)

                    # 3. 采集多个样本并进行均值滤波
                    calibration_result = collect_calibration_samples(
                        tf_buffer,
                        'link00',
                        'estimated_object_varified',
                        num_samples=CALIBRATION_SAMPLES,
                        interval=CALIBRATION_SAMPLE_INTERVAL,
                    )

                    # 4. 从滤波结果中提取坐标
                    try:
                        if calibration_result is not None:
                            pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w = (
                                calibration_result
                            )

                            print("✓ 坐标滤波完成")

                            # 将四元数转换为欧拉角 (roll, pitch, yaw)
                            quaternion = (ori_x, ori_y, ori_z, ori_w)
                            roll, pitch, yaw = euler_from_quaternion(quaternion)

                            # 显示转换后的坐标
                            print("\n" + "=" * 60)
                            print("✓ 标定成功！获取到的坐标如下：")
                            print("=" * 60)
                            print(f"  位置 (Position):")
                            print(f"    x = {pos_x:.6f} m")
                            print(f"    y = {pos_y:.6f} m")
                            print(f"    z = {pos_z:.6f} m")
                            print(f"\n  姿态 (Orientation - Quaternion):")
                            print(f"    x = {ori_x:.6f}")
                            print(f"    y = {ori_y:.6f}")
                            print(f"    z = {ori_z:.6f}")
                            print(f"    w = {ori_w:.6f}")
                            print(f"\n  姿态 (Orientation - Euler Angles):")
                            print(
                                f"    roll  = {roll:.6f} rad  ({roll*180/3.14159:.2f}°)"
                            )
                            print(
                                f"    pitch = {pitch:.6f} rad  ({pitch*180/3.14159:.2f}°)"
                            )
                            print(
                                f"    yaw   = {yaw:.6f} rad  ({yaw*180/3.14159:.2f}°)"
                            )
                            print("=" * 60)

                            # 等待用户确认
                            confirm_input = (
                                input(
                                    "\n>>> 请确认以上坐标是否正确，按 [Enter] 继续，按 [r] 重新标定，按 [q] 退出: "
                                )
                                .strip()
                                .lower()
                            )
                            if confirm_input == 'q':
                                print("用户选择退出")
                                return 0
                            elif confirm_input == 'r':
                                print("\n正在重新标定...")
                                continue  # 重新标定
                            else:
                                # 用户确认，继续执行
                                print(f"✓ 已从标定结果更新 *完整姿态*")
                                calibration_done = True
                        else:
                            print("✗ 错误：未能获取到有效的坐标数据（超时）")
                            print("程序退出")
                            return 1
                    except AttributeError as e:
                        print(f"✗ 错误: 无法解析 TF 变换。错误: {e}")
                        print("程序退出")
                        return 1
                    # --- [结束] ---
                else:
                    print("✗ 标定失败")
                    print(f"  将使用默认姿态 (Pos: x={pos_x:.3f})")
                    print("请选择操作：")
                    print("  [r] - 重新标定")
                    print("  [y] - 使用默认值继续")
                    print("  [n] - 退出程序")
                    retry_input = input("请选择 [r/y/n]: ").strip().lower()
                    if retry_input == 'r':
                        print("\n正在重新标定...")
                        continue  # 继续循环，重新标定
                    elif retry_input == 'y':
                        calibration_done = True  # 使用默认值继续
                    else:
                        return 1  # 退出程序

    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)

    # ----------------------------------------------------
    # 步骤 3: [新] 调用 getgoalandangle 服务
    # ----------------------------------------------------
    print_step(3, total_steps, "计算五点规划路径 (getgoalandangle)")

    print("\n" + "=" * 50)
    print("将使用以下坐标作为 'getgoalandangle' 的输入：")
    print(f"  Position: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")
    print(f"  Orientation: x={ori_x:.3f}, y={ori_y:.3f}, z={ori_z:.3f}, w={ori_w:.3f}")
    print("=" * 50 + "\n")

    # 构建 getgoalandangle 请求
    goal_request = getgoalandangle._request_class()
    goal_request.target_pose.header.frame_id = 'link00'  # 假设 frame_id
    goal_request.target_pose.header.stamp = rospy.Time.now()
    goal_request.target_pose.pose.position.x = pos_x
    goal_request.target_pose.pose.position.y = pos_y
    goal_request.target_pose.pose.position.z = pos_z
    goal_request.target_pose.pose.orientation.x = ori_x
    goal_request.target_pose.pose.orientation.y = ori_y
    goal_request.target_pose.pose.orientation.z = ori_z
    goal_request.target_pose.pose.orientation.w = ori_w

    SERVICE_GETGOAL = '/arm_controller_node/get_goal_and_angle'
    print(f"正在调用服务: {SERVICE_GETGOAL}")

    success, goal_response = call_service(
        SERVICE_GETGOAL, getgoalandangle, goal_request
    )

    if not (success and goal_response.call_success):
        print(
            f"✗ 'getgoalandangle' 服务调用失败 (Success: {success}, Response Success: {goal_response.call_success if goal_response else 'N/A'})"
        )
        return 1

    print(
        f"✓ 'getgoalandangle' 服务成功, 收到 {len(goal_response.target_poses)} 个点位"
    )

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
            "roll": goal_response.roll_angles[i],
        }

    print(f"✓ 已将 {len(planned_points)} 个点位存入字典: {list(planned_points.keys())}")
    print("=" * 50 + "\n")

    # --- [检查] 检查是否获取到所有 5 个点位
    if len(planned_points) != 5:
        print(f"⚠ 警告: 只检测到 {len(planned_points)} 个点位，未全部看到 (预期 5 个)")
        print(f"  检测到的点位: {list(planned_points.keys())}")
        print("\n是否继续执行？")
        print("  [y] - 继续执行")
        print("  [n] - 退出程序")
        try:
            confirm_input = input("请选择 [y/n]: ").strip().lower()
            if confirm_input != 'y':
                print("用户选择退出程序")
                return 1
        except (KeyboardInterrupt, EOFError):
            print("\n\n用户取消操作")
            sys.exit(0)
    else:
        print(f"✓ 已检测到全部 5 个点位")

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
            current_point = execution_path[i]  # 例如: "TOP1"

            # 1. 执行当前点
            print(f"\n[执行路径 {i+1}/{len(execution_path)}]: {current_point}")

            # 我们使用 execute_with_recovery，它会去 *字典* (planned_points) 中查找数据
            if not execute_with_recovery(
                current_point, planned_points, SERVICE_PLANCONTROL
            ):
                print(f"!! 点 {current_point} 执行失败，序列终止 !!")
                return 1  # 终止

            # 2. [规则检查] 检查是否需要插入 "MID"

            # 检查是否还有下一个点
            if i < len(execution_path) - 1:
                # 这就是您要的: 用 goal_response.pose_names (即 execution_path) 来表示下一个点
                next_point = execution_path[i + 1]  # 例如: "TOP2"

                # 规则 1: OUT1 -> MID -> OUT2
                if current_point == "OUT1" and next_point == "OUT2":
                    print("\n!! 规则触发: OUT1 -> OUT2。正在插入 MID... !!")
                    if not execute_with_recovery(
                        "MID", planned_points, SERVICE_PLANCONTROL
                    ):
                        print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                        return 1  # 终止

                # 规则 2: OUT1 -> MID -> HALF2
                if current_point == "OUT1" and next_point == "HALF2":
                    print("\n!! 规则触发: OUT1 -> HALF2。正在插入 MID... !!")
                    if not execute_with_recovery(
                        "MID", planned_points, SERVICE_PLANCONTROL
                    ):
                        print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                        return 1  # 终止

                # 规则 3: HALF1 -> MID -> HALF2
                if current_point == "HALF1" and next_point == "HALF2":
                    print("\n!! 规则触发: HALF1 -> HALF2。正在插入 MID... !!")
                    if not execute_with_recovery(
                        "MID", planned_points, SERVICE_PLANCONTROL
                    ):
                        print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                        return 1  # 终止
        if not wait_for_enter("结束，回到初始位置"):
            return False
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        sys.exit(0)

    # 步骤 1: 定义服务名和类型
    SERVICE_NAME = "/arm_controller_node/back_to_home"
    SERVICE_TYPE = BackToHome

    # 步骤 2: 创建请求对象 (Request)
    # 因为您的 .srv 文件有请求部分 (bool back_to_home)
    print("正在创建请求...")
    my_request = BackToHome._request_class()

    # 步骤 3: 为请求赋值
    # 您想让它回到 home，所以设置为 True
    my_request.back_to_home = True
    print(f"正在调用服务: {SERVICE_NAME}")
    success, response = call_service(SERVICE_NAME, SERVICE_TYPE, my_request)
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
