#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
对比测试：验证均值滤波对标定精度的改善
用于比较单次采样和多次采样滤波的效果
"""

import rospy
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion
import time


def get_single_sample(tf_buffer, source_frame, target_frame):
    """获取单次采样结果"""
    try:
        transform = tf_buffer.lookup_transform(
            source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0)
        )
        pos = transform.transform.translation
        ori = transform.transform.rotation
        return (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
    except Exception as e:
        print(f"采样失败: {e}")
        return None


def quaternion_average(quaternions):
    """计算四元数平均值"""
    if not quaternions:
        return (0, 0, 0, 1)
    
    Q = np.array(quaternions)
    M = np.zeros((4, 4))
    for q in Q:
        q = np.array([q[3], q[0], q[1], q[2]])
        M += np.outer(q, q)
    M = M / len(Q)
    
    eigenvalues, eigenvectors = np.linalg.eig(M)
    max_eigenvector = eigenvectors[:, np.argmax(eigenvalues)]
    
    return (max_eigenvector[1], max_eigenvector[2], max_eigenvector[3], max_eigenvector[0])


def collect_filtered_samples(tf_buffer, source_frame, target_frame, num_samples=10, interval=0.3):
    """采集多个样本并滤波"""
    positions = []
    orientations = []
    
    for i in range(num_samples):
        sample = get_single_sample(tf_buffer, source_frame, target_frame)
        if sample is not None:
            positions.append([sample[0], sample[1], sample[2]])
            orientations.append((sample[3], sample[4], sample[5], sample[6]))
        rospy.sleep(interval)
    
    if not positions:
        return None
    
    positions_array = np.array(positions)
    pos_mean = np.mean(positions_array, axis=0)
    pos_std = np.std(positions_array, axis=0)
    ori_mean = quaternion_average(orientations)
    
    return (pos_mean[0], pos_mean[1], pos_mean[2], 
            ori_mean[0], ori_mean[1], ori_mean[2], ori_mean[3], pos_std)


def main():
    rospy.init_node('compare_calibration_methods', anonymous=True)
    
    print("=" * 70)
    print("标定方法对比测试：单次采样 vs 均值滤波")
    print("=" * 70)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    
    source_frame = 'link00'
    target_frame = 'estimated_object'
    num_tests = 5  # 重复测试次数
    num_samples = 10  # 滤波采样数
    
    print(f"\n配置:")
    print(f"  源坐标系: {source_frame}")
    print(f"  目标坐标系: {target_frame}")
    print(f"  重复测试次数: {num_tests}")
    print(f"  滤波采样数: {num_samples}")
    
    try:
        input("\n按 [Enter] 键开始测试...")
    except (KeyboardInterrupt, EOFError):
        print("\n用户取消")
        return
    
    # 测试 1: 单次采样（重复 num_tests 次）
    print("\n" + "=" * 70)
    print("【测试 1】单次采样方法")
    print("=" * 70)
    
    single_results = []
    for i in range(num_tests):
        print(f"\n第 {i+1}/{num_tests} 次采样...")
        result = get_single_sample(tf_buffer, source_frame, target_frame)
        if result:
            single_results.append(result)
            print(f"  位置: [{result[0]:.6f}, {result[1]:.6f}, {result[2]:.6f}]")
        rospy.sleep(1.0)
    
    if single_results:
        single_positions = np.array([[r[0], r[1], r[2]] for r in single_results])
        single_mean = np.mean(single_positions, axis=0)
        single_std = np.std(single_positions, axis=0)
        
        print(f"\n单次采样统计 ({len(single_results)} 次):")
        print(f"  位置均值: [{single_mean[0]:.6f}, {single_mean[1]:.6f}, {single_mean[2]:.6f}]")
        print(f"  位置标准差: [{single_std[0]:.6f}, {single_std[1]:.6f}, {single_std[2]:.6f}]")
        print(f"  平均标准差: {np.mean(single_std):.6f} m")
    
    # 测试 2: 均值滤波（重复 num_tests 次）
    print("\n" + "=" * 70)
    print("【测试 2】均值滤波方法")
    print("=" * 70)
    
    filtered_results = []
    filtered_stds = []
    for i in range(num_tests):
        print(f"\n第 {i+1}/{num_tests} 次滤波采样...")
        result = collect_filtered_samples(tf_buffer, source_frame, target_frame, num_samples, 0.2)
        if result:
            filtered_results.append(result[:7])  # 前7个是位置和姿态
            filtered_stds.append(result[7])      # 第8个是标准差
            print(f"  滤波后位置: [{result[0]:.6f}, {result[1]:.6f}, {result[2]:.6f}]")
            print(f"  本轮标准差: [{result[7][0]:.6f}, {result[7][1]:.6f}, {result[7][2]:.6f}]")
        rospy.sleep(1.0)
    
    if filtered_results:
        filtered_positions = np.array([[r[0], r[1], r[2]] for r in filtered_results])
        filtered_mean = np.mean(filtered_positions, axis=0)
        filtered_std = np.std(filtered_positions, axis=0)
        avg_internal_std = np.mean([np.mean(std) for std in filtered_stds])
        
        print(f"\n滤波采样统计 ({len(filtered_results)} 次):")
        print(f"  位置均值: [{filtered_mean[0]:.6f}, {filtered_mean[1]:.6f}, {filtered_mean[2]:.6f}]")
        print(f"  位置标准差: [{filtered_std[0]:.6f}, {filtered_std[1]:.6f}, {filtered_std[2]:.6f}]")
        print(f"  平均标准差: {np.mean(filtered_std):.6f} m")
        print(f"  滤波内部平均标准差: {avg_internal_std:.6f} m")
    
    # 对比结果
    print("\n" + "=" * 70)
    print("【对比结果】")
    print("=" * 70)
    
    if single_results and filtered_results:
        improvement_ratio = np.mean(single_std) / np.mean(filtered_std)
        print(f"\n精度改善:")
        print(f"  单次采样平均标准差: {np.mean(single_std):.6f} m")
        print(f"  滤波采样平均标准差: {np.mean(filtered_std):.6f} m")
        print(f"  精度提升: {improvement_ratio:.2f}x")
        print(f"  标准差降低: {(1 - 1/improvement_ratio) * 100:.1f}%")
        
        position_diff = np.abs(single_mean - filtered_mean)
        print(f"\n位置差异:")
        print(f"  X轴差异: {position_diff[0]:.6f} m")
        print(f"  Y轴差异: {position_diff[1]:.6f} m")
        print(f"  Z轴差异: {position_diff[2]:.6f} m")
        print(f"  总体差异: {np.linalg.norm(position_diff):.6f} m")
        
        print("\n✓ 结论:")
        if improvement_ratio > 1.5:
            print("  均值滤波显著提高了标定精度！建议使用。")
        elif improvement_ratio > 1.2:
            print("  均值滤波适度提高了标定精度，推荐使用。")
        else:
            print("  两种方法精度相近，可根据速度需求选择。")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\n程序被中断")
