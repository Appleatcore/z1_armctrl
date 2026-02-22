#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from iros_ros_foundationpose.srv import Reset

# 导入 StepIt 的服务定义

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from arm_controller_srvs.srv import (
    crossgetgoalandangle,
    planandgrippercontrol,
    PlanToHorizon,
    PlanToHorizonHeight,
    BackToHome,
)
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import sys
import signal
import numpy as np

ENABLE_DOG_CONTROL = False  # True: 控制狗的姿态; False: 仅运行机械臂逻辑
if ENABLE_DOG_CONTROL:
    from stepit_ros_msgs.srv import Control, ControlRequest

ENABLE_AUTO_CONTROL = False  # True: 全自动控制; False: 手动确认
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
    tf_buffer, source_frame, target_frame, num_samples=5, interval=0.5
):
    """
    采集多个标定样本并进行均值滤波

    参数:
        tf_buffer: TF2 缓冲区
        source_frame: 源坐标系
        target_frame: 目标坐标系
        num_samples: 采集样本数量
        interval: 采样间隔（秒）

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

            # 提取位置
            pos = transform.transform.translation
            positions.append([pos.x, pos.y, pos.z])

            # 提取姿态（四元数）
            ori = transform.transform.rotation
            orientations.append((ori.x, ori.y, ori.z, ori.w))

            print(
                f"  样本 {i+1}/{num_samples}: 位置=[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]"
            )

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
    if ENABLE_AUTO_CONTROL:
        print(">>> [自动模式] 自动执行...")
        return True
    try:
        input(">>> 按下 [Enter] 键开始执行此点，或按 Ctrl+C 取消...")
        return True
    except (KeyboardInterrupt, EOFError):
        print("\n\n用户取消操作")
        return False


def get_tf_transform(
    tf_buffer, source_frame, target_frame, timeout=5.0, time_threshold=0.5
):
    """获取 TF 变换，并检查数据的新鲜度"""
    print(f"正在监听 tf 变换 ({source_frame} -> {target_frame})，持续{timeout}秒...")

    start_time = rospy.Time.now()
    last_valid_transform = None

    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            transform = tf_buffer.lookup_transform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(0.1)
            )
            current_time = rospy.Time.now()
            data_time = transform.header.stamp
            time_diff = (current_time - data_time).to_sec()

            if abs(time_diff) < time_threshold:
                last_valid_transform = transform
                trans = transform.transform.translation
                print(
                    f"✓ [有效] 延迟: {time_diff:.4f}s | Stamp: {data_time.to_sec():.3f}"
                )
                print(f"  - Trans: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]")
            else:
                print(
                    f"✗ [丢弃] 数据过时! 延迟: {time_diff:.4f}s > 阈值 {time_threshold}s"
                )
            rospy.sleep(0.1)
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
    """按名称查找并执行单个点位"""
    print(f"\n--- 正在尝试执行点: {point_name} ---")

    if point_name not in points_dict:
        print(
            f"✗ 警告: 点位 '{point_name}' 未在 crossgetgoalandangle 的响应中找到。跳过此点。"
        )
        return True

    point_data = points_dict[point_name]
    pose = point_data["pose"]
    joint6 = point_data["roll"]
    gripper_val = point_data["pitch"]

    print(
        f"  Pose: {pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}"
    )
    print(f"  Joint6: {joint6:.3f}, Gripper: {gripper_val:.3f}")

    if not wait_for_enter(point_name):
        return False

    plan_request = planandgrippercontrol._request_class()
    plan_request.target_pose = pose
    plan_request.gripper_pos = gripper_val
    plan_request.joint6_pos = joint6

    success, plan_response = call_service(
        service_name, planandgrippercontrol, plan_request
    )

    if success and plan_response.call_success:
        print(f"✓ 点 {point_name} 执行成功")
        return True
    else:
        print(f"✗ 点 {point_name} 执行失败")
        print("!! 终止序列 !!")
        return False


def execute_with_recovery(point_name, points_dict, service_name, loop_round):
    """尝试执行，如果失败则根据轮次调用horizon或default服务并重试一次"""
    # 1. 第一次尝试
    if execute_point_by_name(point_name, points_dict, service_name):
        return True

    # 2. 如果失败，根据奇偶轮次选择恢复服务
    print(f"\n⚠ 警告: 点 {point_name} 首次执行失败！")

    if ENABLE_DOG_CONTROL and loop_round % 2 == 1:
        # 奇数轮：调用 plan_to_horizon
        SERVICE_NAME = "/arm_controller_node/plan_to_horizon"
        SERVICE_TYPE = PlanToHorizon
        print(f"➜ 正在尝试: 调用 plan_to_horizon 服务并重试...")

        horizon_req = PlanToHorizon._request_class()
        if hasattr(horizon_req, 'plan_to_horizon'):
            horizon_req.plan_to_horizon = True

        h_succ, response = call_service(SERVICE_NAME, SERVICE_TYPE, horizon_req)
    else:
        # 偶数轮：调用 plan_to_horizon_height
        SERVICE_NAME = "/arm_controller_node/plan_to_horizon_height"
        SERVICE_TYPE = PlanToHorizonHeight
        print(f"➜ 正在尝试: 调用 plan_to_horizon_height 服务并重试...")

        height_req = PlanToHorizonHeight._request_class()
        height_req.height = 0.0
        h_succ, response = call_service(SERVICE_NAME, SERVICE_TYPE, height_req)

    if not h_succ or not response.call_success:
        print(f"✗ 严重错误: 恢复服务调用失败，放弃重试。")
        return False

    print("✓ 恢复服务调用成功")
    rospy.sleep(1.0)

    # 3. 第二次尝试
    print(f"➜ 正在重试: 前往 {point_name} ...")
    if execute_point_by_name(point_name, points_dict, service_name):
        print(f"✓ 重试成功: 点 {point_name} 执行完成")
        return True
    else:
        print(f"✗ 重试失败: 点 {point_name} 无法到达")
        return False


def reorganize_points_simple(original_points, lack_path):
    """
    逻辑：
    1. 必须存在 'MID' 才能计算相对位置。
    2. 计算出点的真实方位（TOP, DOWN...）。
    3. 【核心修改】检查该方位是否在 lack_path 中：
       - 如果在：加入 new_points，并从 lack_path 中移除（标记为已找到）。
       - 如果不在：跳过（不更新，不覆盖已有的）。

    Returns:
        new_points (dict): 本轮新增/捕获的点位
        final_path (list): 本轮需要执行的点位顺序
        lack_path (list): 剩余还未找到的点位
    """
    print("\n" + "*" * 50)
    print(f"正在检查点位 (当前缺失: {lack_path})...")

    # 0. 基础保护
    if not original_points:
        return {}, [], lack_path

    # 1. 必须找到 MID 作为【计算基准】
    # 即使 MID 已经不缺了，我们也需要它来计算 TOP/DOWN
    if "MID" not in original_points:
        print("⚠ 本次扫描未找到 'MID' 点，无法计算相对坐标。")
        return {}, [], lack_path

    new_points = {}

    # 获取基准点坐标
    mid_data = original_points["MID"]
    mid_pose = mid_data["pose"].position

    # 2. 检查 MID 自身是否需要更新
    if "MID" in lack_path:
        new_points["MID"] = mid_data
        # lack_path.remove("MID")
        print("✓ [更新] 成功捕获缺失点: MID")
    else:
        print("  [跳过] MID 已存在，无需更新")

    # 3. 遍历其他点，计算方位并与 lack_path 比对
    for name, data in original_points.items():
        if name == "MID":
            continue  # 跳过 MID 自身

        p = data["pose"].position
        dy = p.y - mid_pose.y
        dz = p.z - mid_pose.z

        # --- 计算空间方位 ---
        new_key = name  # 默认名字

        # 判断是 垂直点(TOP/DOWN) 还是 水平点(LEFT/RIGHT)
        if abs(dz) > abs(dy):
            # 垂直方向 (容差 10cm)
            if dz > 0.1:
                new_key = "TOP"
            elif dz < -0.1:
                new_key = "DOWN"
        else:
            # 水平方向
            if dy > 0.1:
                new_key = "LEFT"
            elif dy < -0.1:
                new_key = "RIGHT"

        # 与 lack_path 比较
        if new_key in lack_path:
            # 只有缺少的点才更新
            new_points[new_key] = data
            # lack_path.remove(new_key)  # 从缺失列表中移除
            print(
                f"✓ [更新] 成功捕获缺失点: '{new_key}' (原名: {name}, dZ={dz:.3f}, dY={dy:.3f})"
            )
        else:
            # 已经有该点了，忽略本次扫描结果
            print(f"  [跳过] '{new_key}' 已存在，不更新。")

    # 4. 生成本轮需要执行的路径 (只包含本次新发现的点)
    final_path = []

    # 按照扫描逻辑排序
    if "LEFT" in new_points:
        final_path.append("LEFT")
    if "MID" in new_points:
        final_path.append("MID")
    if "DOWN" in new_points:
        final_path.append("DOWN")
    if "RIGHT" in new_points:
        final_path.append("RIGHT")
    if "TOP" in new_points:
        final_path.append("TOP")

    print("-" * 30)
    print(f"✓ 本轮筛选完成")
    print(f"  新增执行: {final_path}")
    # print(f"  剩余缺失: {lack_path}")
    print("-" * 30)

    return new_points, final_path, lack_path


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


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('arm_control_sequence_custom', anonymous=True)
    print_header("机械臂控制序列脚本 (循环扫描版)")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    # 默认坐标定义 (在循环外定义，这样循环中可以继承上一轮的标定结果，或者重置)
    # pos_x, pos_y, pos_z = 0.7, 0.0, 0.2
    # input_roll, input_pitch, input_yaw = 0.0, -1.87, 0.0
    pos_x, pos_y, pos_z = 0.5, 0.0, 0.05
    input_roll, input_pitch, input_yaw = 0.0, -0.5, 0.0

    print(f"正在计算初始姿态四元数...")
    q = quaternion_from_euler(input_roll, input_pitch, input_yaw)

    ori_x, ori_y, ori_z, ori_w = q[0], q[1], q[2], q[3]

    # =========================================================================
    # 循环控制变量初始化
    # =========================================================================
    # 初始状态：假设缺少所有点，强制进入循环
    lack_path = ["TOP", "DOWN", "LEFT", "RIGHT", "MID"]
    loop_round = 0

    print_header(f"开始执行！初始缺失点位: {lack_path}")

    # =========================================================================
    # 主循环 (Step 1 -> Step 5)
    # 只要 lack_path 不为空，就一直循环
    # =========================================================================
    while len(lack_path) > 0:
        loop_round += 1
        print("\n" + "#" * 60)
        print(f"   进入第 {loop_round} 轮尝试 (缺少: {lack_path})")
        print("#" * 60)
        # ----------------------------------------------------
        # 步骤 1: 移动到初始/扫描位置 (根据轮次切换服务)
        # ----------------------------------------------------
        print_step(1, 6, "移动到初始观测位置")

        if ENABLE_DOG_CONTROL and loop_round % 2 == 1:
            # 奇数轮：Plan To Horizon
            print(f"-> 第 {loop_round} 轮为奇数，调用: plan_to_horizon")
            horizon_request = PlanToHorizon._request_class()
            if hasattr(horizon_request, 'plan_to_horizon'):
                horizon_request.plan_to_horizon = True

            success, response = call_service(
                '/arm_controller_node/plan_to_horizon', PlanToHorizon, horizon_request
            )
        else:
            # 偶数轮：Plan To Horizon Height
            print(f"-> 第 {loop_round} 轮为偶数，调用: plan_to_horizon_height")
            height_request = PlanToHorizonHeight._request_class()
            height_request.height = 0.0

            success, response = call_service(
                '/arm_controller_node/plan_to_horizon_height',
                PlanToHorizonHeight,
                height_request,
            )

        if success and response.call_success:
            print("✓ 移动到观测位置成功")
        else:
            print("✗ 移动到观测位置失败")
            # 根据需求决定是 return 1 还是继续
            return 1

        rospy.sleep(1.0)
        # ----------------------------------------------------
        # 步骤 2: 标定
        # ----------------------------------------------------
        print_step(2, 6, "执行标定 (更新物体位置)")

        # 根据 ENABLE_AUTO_CONTROL 决定是否需要用户输入
        if ENABLE_AUTO_CONTROL:
            # 自动模式：默认执行标定
            user_input = 'y'
            print("[自动模式] 自动执行标定")
        else:
            # 手动模式：询问用户
            print("\n" + "=" * 50)
            print("是否执行标定？")
            print("  [y] - 执行标定")
            print("  [n] - 跳过标定")
            print("  [q] - 退出程序")
            print("=" * 50)

            try:
                user_input = input("请选择 [y/n/q] (或 Ctrl+C 退出): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                print("\n\n用户取消操作")
                sys.exit(0)

        # 处理用户选择
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
                cmd_request.cmd = 3
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
                        'estimated_object',
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

                            # 根据模式决定是否等待用户确认
                            if ENABLE_AUTO_CONTROL:
                                print("\n[自动模式] 自动使用以上坐标继续...")
                                rospy.sleep(1.0)
                                calibration_done = True
                            else:
                                # 手动模式：等待用户确认
                                try:
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
                                except (KeyboardInterrupt, EOFError):
                                    print("\n\n用户取消操作")
                                    sys.exit(0)
                        else:
                            print("⚠ 警告：未能获取到有效的坐标数据")
                            print("✗ 标定失败")
                            print(f"  将使用默认姿态 (Pos: x={pos_x:.3f})")

                            # 标定失败处理：先回到 Home 点
                            print("\n正在让机械臂回到 Home 点...")
                            home_req = BackToHome._request_class()
                            home_req.back_to_home = True
                            call_service(
                                "/arm_controller_node/back_to_home",
                                BackToHome,
                                home_req,
                            )
                            print("✓ 机械臂已回到 Home 点")

                            if ENABLE_AUTO_CONTROL:
                                print("[自动模式] 标定失败，退出程序")
                                return 1
                            else:
                                print("请选择操作：")
                                print("  [r] - 重新标定")
                                print("  [y] - 使用默认值继续")
                                print("  [n] - 退出程序")
                                try:
                                    retry_input = (
                                        input("请选择 [r/y/n]: ").strip().lower()
                                    )
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
                    except AttributeError as e:
                        print(f"✗ 警告: 无法解析 TF 变换。错误: {e}")
                        print(f"  将继续使用默认姿态。")
                        calibration_done = True
                    # --- [结束] ---
                else:
                    print("✗ 标定失败")
                    # print(f"  将使用默认姿态 (Pos: x={pos_x:.3f})")

                    # # 标定失败处理：先回到 Home 点
                    # print("\n正在让机械臂回到 Home 点...")
                    # home_req = BackToHome._request_class()
                    # home_req.back_to_home = True
                    # call_service("/arm_controller_node/back_to_home", BackToHome, home_req)
                    # print("✓ 机械臂已回到 Home 点")

                    if ENABLE_AUTO_CONTROL:
                        print("[自动模式] 标定失败，退出程序")
                        return 1
                    else:
                        print("请选择操作：")
                        print("  [r] - 重新标定")
                        print("  [y] - 使用默认值继续")
                        print("  [n] - 退出程序")
                        try:
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
        # 步骤 3: 规划 (crossgetgoalandangle) & 更新 lack_path
        # ----------------------------------------------------
        print_step(3, 6, "计算规划路径")

        goal_req = crossgetgoalandangle._request_class()
        goal_req.target_pose.header.frame_id = 'link00'
        goal_req.target_pose.header.stamp = rospy.Time.now()
        goal_req.target_pose.pose.position.x = pos_x
        goal_req.target_pose.pose.position.y = pos_y
        goal_req.target_pose.pose.position.z = pos_z
        goal_req.target_pose.pose.orientation.x = ori_x
        goal_req.target_pose.pose.orientation.y = ori_y
        goal_req.target_pose.pose.orientation.z = ori_z
        goal_req.target_pose.pose.orientation.w = ori_w

        succ, goal_res = call_service(
            '/arm_controller_node/cross_get_goal_and_angle',
            crossgetgoalandangle,
            goal_req,
        )

        if not (succ and goal_res.call_success):
            print("✗ 规划服务调用失败，跳过本轮执行，直接回零重试...")
            # 如果规划失败，不要退出程序，而是进入 Step 5 回零，然后下一轮循环
            # 但需要保持 lack_path 不为空，防止循环退出
            pass
        else:
            # 数据转换
            raw_points = {}
            for i in range(len(goal_res.pose_names)):
                raw_points[goal_res.pose_names[i]] = {
                    "pose": goal_res.target_poses[i],
                    "pitch": goal_res.pitch_angles[i],
                    "roll": goal_res.roll_angles[i],
                }

            # === 调用重组函数，并更新循环条件 lack_path ===
            planned_points, execution_path, lack_path = reorganize_points_simple(
                raw_points, lack_path
            )

            print(f"✓ 本轮规划结果: 找到 {len(planned_points)} 个点")
            # print(f"✓ 剩余缺失点位: {lack_path}")
            if len(lack_path) == 0:
                print("\n🎉 太棒了！所有点位都已集齐！本轮执行后将结束循环。")

            # ----------------------------------------------------
            # 步骤 4: 执行 (Execute)
            # ----------------------------------------------------
            if execution_path:
                print_step(4, 5, f"执行本轮路径: {execution_path}")

                SERVICE_PLANCONTROL = '/arm_controller_node/plan_and_gripper_control'

                for i in range(len(execution_path)):
                    current_point = execution_path[i]

                    # 1. 执行当前点
                    print(f"\n[执行路径 {i+1}/{len(execution_path)}]: {current_point}")

                    # 我们使用 execute_with_recovery，它会去 *字典* (planned_points) 中查找数据
                    if not execute_with_recovery(
                        current_point, planned_points, SERVICE_PLANCONTROL, loop_round
                    ):
                        print(f"!! 点 {current_point} 执行失败，序列终止 !!")
                        continue
                    else:
                        print(f"✓ 点 {current_point} 执行完成")
                        if current_point in lack_path:
                            lack_path.remove(current_point)

                    # 2. [规则检查] 检查是否需要插入 "MID"

                    # 检查是否还有下一个点
                    if i < len(execution_path) - 1:
                        # 这就是您要的: 用 goal_response.pose_names (即 execution_path) 来表示下一个点
                        next_point = execution_path[i + 1]  # 例如: "TOP2"

                        # 规则 1: TOP -> MID -> DOWN
                        if current_point == "TOP" and next_point == "DOWN":
                            print("\n!! 规则触发: TOP -> DOWN。正在插入 MID... !!")
                            if not execute_with_recovery(
                                "MID", planned_points, SERVICE_PLANCONTROL, loop_round
                            ):
                                print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                                return 1  # 终止
                            else:
                                print(f"✓ 点 {current_point} 执行完成")
                                if current_point in lack_path:
                                    lack_path.remove(current_point)
                        # 规则 2: LEFT -> MID -> RIGHT
                        elif current_point == "LEFT" and next_point == "RIGHT":
                            print("\n!! 规则触发: LEFT -> RIGHT。正在插入 MID... !!")
                            if not execute_with_recovery(
                                "MID", planned_points, SERVICE_PLANCONTROL, loop_round
                            ):
                                print(f"!! 过渡点 MID 执行失败，序列终止 !!")
                                return 1  # 终止
                            else:
                                print(f"✓ 点 {current_point} 执行完成")
                                if current_point in lack_path:
                                    lack_path.remove(current_point)
                if not wait_for_enter("结束，回到初始位置"):
                    return False
            else:
                print("⚠ 本轮未规划出有效路径，跳过执行步骤。")

        # ----------------------------------------------------
        # 步骤 5: 回到初始位置 (Back to Home)
        # ----------------------------------------------------
        print_step(5, 6, "本轮结束，回到 Home 点")
        my_request = BackToHome._request_class()
        my_request.back_to_home = True
        call_service("/arm_controller_node/back_to_home", BackToHome, my_request)
        # ----------------------------------------------------
        # 循环判断
        # ----------------------------------------------------
        if len(lack_path) > 0:
            print("\n" + "!" * 50)
            print(f"⚠ 本轮结束后，仍缺失以下点位: {lack_path}")
            print("请调整物体位置或角度，以便相机能看到缺失的部分。")

            if not ENABLE_AUTO_CONTROL:
                try:
                    input("按回车键调整...")
                except (KeyboardInterrupt, EOFError):
                    print("\n\n用户取消操作")
                    sys.exit(0)
            else:
                print("[自动模式] 自动继续...")
                rospy.sleep(1.0)  # 自动模式下稍微等待一下

            # ----------------------------------------------------
            # 步骤 6: [新加入] 改变机器人姿态，准备下一轮
            # ----------------------------------------------------
            if ENABLE_DOG_CONTROL:
                print_step(6, 6, "调整机器人姿态，准备下一轮扫描")
                if loop_round % 2 == 1:
                    print("\n[步骤 1] 设置高度 1.0 && 俯仰角 0.4")
                    send_stepit_command("Policy/CmdHeight/SetHeight:1.0")
                    print("  ... 等待 1.0 秒 ...")
                    rospy.sleep(1.0)
                    send_stepit_command("Policy/CmdPitch/SetPitch:0.4")
                else:
                    print("\n[步骤 2] 设置高度 0.6 && 俯仰角 0.4")
                    send_stepit_command("Policy/CmdHeight/SetHeight:0.6")
                    print("  ... 等待 1.0 秒 ...")
                    rospy.sleep(1.0)
                    send_stepit_command("Policy/CmdPitch/SetPitch:0.4")
            else:
                print("\n[跳过] 步骤 6: 狗姿态控制已禁用。")
                print("按回车键开始下一轮扫描... (或 Ctrl+C 退出)")
                print("!" * 50)
                print("\n--- 姿态调整完成 ---")

                if not ENABLE_AUTO_CONTROL:
                    try:
                        input("按回车键继续...")
                    except (KeyboardInterrupt, EOFError):
                        print("\n\n用户取消操作")
                        sys.exit(0)
                else:
                    print("[自动模式] 自动继续...")
                    rospy.sleep(1.0)
        else:
            print("\n" + "=" * 50)
            print("✅ 任务圆满完成！所有 5 个点位均已成功规划并执行。")
            print("=" * 50)
            break

    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except (KeyboardInterrupt, EOFError):
        pass
    except SystemExit:
        raise
