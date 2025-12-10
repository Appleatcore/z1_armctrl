#!/usr/bin/env python3
"""
测试 get_goal_and_angle 服务
该服务执行五点规划的前4步，返回目标点位和角度，但不执行运动
"""

import rospy
from arm_controller_srvs.srv import getgoalandangle
from geometry_msgs.msg import PoseStamped
import sys

def test_get_goal_and_angle():
    """测试 get_goal_and_angle 服务"""
    
    rospy.init_node('test_get_goal_and_angle', anonymous=True)
    
    # 等待服务可用
    service_name = '/arm_controller_node/get_goal_and_angle'
    print(f"等待服务 {service_name} 可用...")
    
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        print(f"✓ 服务 {service_name} 已就绪")
    except rospy.ROSException:
        print(f"✗ 服务 {service_name} 超时，请确保 arm_controller_node 正在运行")
        return False
    
    # 创建服务代理
    try:
        get_goal_service = rospy.ServiceProxy(service_name, getgoalandangle)
    except rospy.ServiceException as e:
        print(f"✗ 创建服务代理失败: {e}")
        return False
    
    # 构造测试目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = "link00"
    target_pose.header.stamp = rospy.Time.now()
    
    # 设置位置（可以根据实际情况修改）
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.2
    
    # 设置姿态（单位四元数）
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0
    
    print("\n" + "="*60)
    print("测试目标位姿:")
    print(f"  位置: ({target_pose.pose.position.x:.3f}, "
          f"{target_pose.pose.position.y:.3f}, "
          f"{target_pose.pose.position.z:.3f})")
    print(f"  姿态: ({target_pose.pose.orientation.x:.3f}, "
          f"{target_pose.pose.orientation.y:.3f}, "
          f"{target_pose.pose.orientation.z:.3f}, "
          f"{target_pose.pose.orientation.w:.3f})")
    print("="*60)
    
    # 调用服务
    print("\n正在调用服务...")
    try:
        response = get_goal_service(target_pose)
        
        if response.call_success:
            print(f"\n✓ 服务调用成功！")
            print(f"✓ 找到 {len(response.target_poses)} 个可达目标点位\n")
            
            # 显示每个目标点位的详细信息
            for i, (pose, name, pitch, roll) in enumerate(zip(
                response.target_poses,
                response.pose_names,
                response.pitch_angles,
                response.roll_angles
            )):
                print(f"{'='*60}")
                print(f"目标点 {i+1}: {name}")
                print(f"{'='*60}")
                print(f"  位置:")
                print(f"    X: {pose.position.x:8.4f} m")
                print(f"    Y: {pose.position.y:8.4f} m")
                print(f"    Z: {pose.position.z:8.4f} m")
                print(f"  姿态:")
                print(f"    Qx: {pose.orientation.x:8.4f}")
                print(f"    Qy: {pose.orientation.y:8.4f}")
                print(f"    Qz: {pose.orientation.z:8.4f}")
                print(f"    Qw: {pose.orientation.w:8.4f}")
                print(f"  角度:")
                print(f"    Pitch: {pitch:8.4f} rad ({pitch*180/3.14159:.2f}°)")
                print(f"    Roll:  {roll:8.4f} rad ({roll*180/3.14159:.2f}°)")
                print()
            
            # 显示执行顺序
            print(f"{'='*60}")
            print("执行顺序（如果使用 plan_to_five_point 服务）:")
            print(f"{'='*60}")
            execution_order = ["OUT1", "HALF1", "MID", "HALF2", "OUT2"]
            for i, name in enumerate(execution_order):
                if name in response.pose_names:
                    idx = response.pose_names.index(name)
                    print(f"  {i+1}. {name:6s} - 位置: "
                          f"({response.target_poses[idx].position.x:.3f}, "
                          f"{response.target_poses[idx].position.y:.3f}, "
                          f"{response.target_poses[idx].position.z:.3f})")
                else:
                    print(f"  {i+1}. {name:6s} - 不可达")
            
            return True
            
        else:
            print(f"\n✗ 服务调用失败：未找到可达点位")
            return False
            
    except rospy.ServiceException as e:
        print(f"\n✗ 服务调用异常: {e}")
        return False

def main():
    """主函数"""
    print("\n" + "="*60)
    print("测试 get_goal_and_angle 服务")
    print("="*60)
    
    try:
        success = test_get_goal_and_angle()
        
        if success:
            print("\n" + "="*60)
            print("✓ 测试完成！")
            print("="*60)
            return 0
        else:
            print("\n" + "="*60)
            print("✗ 测试失败")
            print("="*60)
            return 1
            
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        return 1
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main())
