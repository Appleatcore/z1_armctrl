#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试标定的均值滤波功能
用于独立测试 TF 变换的多样本采集和滤波
"""

import rospy
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion


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
    return (max_eigenvector[1], max_eigenvector[2], max_eigenvector[3], max_eigenvector[0])


def collect_calibration_samples(tf_buffer, source_frame, target_frame, num_samples=5, interval=0.5):
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
    print(f"源坐标系: {source_frame} -> 目标坐标系: {target_frame}")
    
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
            
            print(f"  样本 {i+1}/{num_samples}: 位置=[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}]")
            
            # 等待采样间隔
            if i < num_samples - 1:
                rospy.sleep(interval)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
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
    pos_max_diff = np.max(positions_array, axis=0) - np.min(positions_array, axis=0)
    
    # 计算四元数平均值
    ori_mean = quaternion_average(orientations)
    
    print(f"\n✓ 采集完成！共 {len(positions)} 个有效样本")
    print(f"\n位置统计:")
    print(f"  均值:   [{pos_mean[0]:.6f}, {pos_mean[1]:.6f}, {pos_mean[2]:.6f}]")
    print(f"  标准差: [{pos_std[0]:.6f}, {pos_std[1]:.6f}, {pos_std[2]:.6f}]")
    print(f"  最大差: [{pos_max_diff[0]:.6f}, {pos_max_diff[1]:.6f}, {pos_max_diff[2]:.6f}]")
    
    return (pos_mean[0], pos_mean[1], pos_mean[2], 
            ori_mean[0], ori_mean[1], ori_mean[2], ori_mean[3])


def main():
    rospy.init_node('test_calibration_filter', anonymous=True)
    
    print("=" * 60)
    print("标定均值滤波测试程序")
    print("=" * 60)
    
    # 初始化 TF2
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    
    # 配置参数
    source_frame = 'link00'
    target_frame = 'estimated_object'
    num_samples = 10  # 采集 10 个样本
    interval = 0.3    # 每 0.3 秒采集一个样本
    
    print(f"\n配置:")
    print(f"  源坐标系: {source_frame}")
    print(f"  目标坐标系: {target_frame}")
    print(f"  样本数量: {num_samples}")
    print(f"  采样间隔: {interval} 秒")
    
    try:
        input("\n按 [Enter] 键开始采集样本...")
    except (KeyboardInterrupt, EOFError):
        print("\n用户取消")
        return
    
    # 采集样本并滤波
    result = collect_calibration_samples(
        tf_buffer, source_frame, target_frame, 
        num_samples=num_samples, interval=interval
    )
    
    if result is not None:
        pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w = result
        
        # 转换为欧拉角
        quaternion = (ori_x, ori_y, ori_z, ori_w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        print("\n" + "=" * 60)
        print("✓ 滤波后的标定结果:")
        print("=" * 60)
        print(f"\n位置 (Position):")
        print(f"  x = {pos_x:.6f} m")
        print(f"  y = {pos_y:.6f} m")
        print(f"  z = {pos_z:.6f} m")
        print(f"\n姿态 (Orientation - Quaternion):")
        print(f"  x = {ori_x:.6f}")
        print(f"  y = {ori_y:.6f}")
        print(f"  z = {ori_z:.6f}")
        print(f"  w = {ori_w:.6f}")
        print(f"\n姿态 (Orientation - Euler Angles):")
        print(f"  roll  = {roll:.6f} rad  ({roll*180/3.14159:.2f}°)")
        print(f"  pitch = {pitch:.6f} rad  ({pitch*180/3.14159:.2f}°)")
        print(f"  yaw   = {yaw:.6f} rad  ({yaw*180/3.14159:.2f}°)")
        print("=" * 60)
    else:
        print("\n✗ 标定失败")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n程序被中断")
