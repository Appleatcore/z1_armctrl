# PoseParamUpdater 节点使用说明

## 功能描述

`pose_param_updater_node` 是一个ROS节点,用于订阅 `geometry_msgs/PoseStamped` 类型的话题,并将接收到的位置数据(x, y, z)自动更新到ROS参数服务器中。

这个节点特别适用于需要动态更新机械臂目标位置参数的场景。

## 文件结构

```
arm_controller/
├── include/
│   └── pose_param_updater.h          # 头文件
├── src/
│   ├── pose_param_updater.cpp        # 实现文件
│   └── pose_param_updater_node.cpp   # 节点主程序
└── launch/
    └── pose_param_updater.launch     # 启动文件
```

## 编译

在你的工作空间中编译:

```bash
cd ~/unitree_ws_demo
catkin_make
# 或者只编译arm_controller包
catkin_make --pkg arm_controller
```

## 使用方法

### 方法1: 使用launch文件启动 (推荐)

```bash
# 使用默认参数启动
roslaunch arm_controller pose_param_updater.launch

# 自定义话题名称
roslaunch arm_controller pose_param_updater.launch pose_topic:=/your/custom/pose/topic

# 自定义所有参数
roslaunch arm_controller pose_param_updater.launch \
    pose_topic:=/your/pose/topic \
    param_x:=/custom/x \
    param_y:=/custom/y \
    param_z:=/custom/z
```

### 方法2: 直接运行节点

```bash
# 使用默认参数
rosrun arm_controller pose_param_updater_node

# 自定义话题名称
rosrun arm_controller pose_param_updater_node _pose_topic:=/your/pose/topic

# 自定义参数名称
rosrun arm_controller pose_param_updater_node \
    _pose_topic:=/your/pose/topic \
    _param_x:=/custom/x \
    _param_y:=/custom/y \
    _param_z:=/custom/z
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pose_topic` | string | `/target_pose` | 要订阅的PoseStamped话题名称 |
| `param_x` | string | `/test/line_point_x` | 存储x坐标的ROS参数名称 |
| `param_y` | string | `/test/line_point_y` | 存储y坐标的ROS参数名称 |
| `param_z` | string | `/test/line_point_z` | 存储z坐标的ROS参数名称 |

## 测试示例

### 1. 启动节点

```bash
roslaunch arm_controller pose_param_updater.launch
```

### 2. 发布测试消息

在另一个终端中,使用 `rostopic pub` 发布测试数据:

```bash
rostopic pub /target_pose geometry_msgs/PoseStamped "
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 0.8
    y: 0.0
    z: 0.15
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### 3. 查看参数是否更新

```bash
# 查看单个参数
rosparam get /test/line_point_x
rosparam get /test/line_point_y
rosparam get /test/line_point_z

# 查看所有test命名空间下的参数
rosparam get /test
```

## 与arm_controller_node集成

如果你想让这个节点与 `arm_controller_node` 一起启动,可以修改 `arm_controller_node.launch`:

```xml
<launch>
    <!-- 原有的arm_controller_node配置 -->
    <include file="$(find arm_controller)/launch/arm_controller_node.launch" />
    
    <!-- 添加pose_param_updater节点 -->
    <include file="$(find arm_controller)/launch/pose_param_updater.launch">
        <arg name="pose_topic" value="/your/pose/topic" />
    </include>
</launch>
```

## 工作原理

1. 节点订阅指定的 `PoseStamped` 话题
2. 当收到新消息时,提取 `pose.position.x`, `pose.position.y`, `pose.position.z`
3. 使用 `ros::NodeHandle::setParam()` 更新ROS参数服务器
4. 其他节点(如 `arm_controller_node`)可以通过 `rosparam get` 读取这些更新后的参数

## 注意事项

- 该节点只更新位置信息(x, y, z),不处理姿态信息(orientation)
- 如果需要更新其他参数(如pitch, yaw, roll),可以扩展代码
- 参数更新是实时的,其他节点需要主动重新读取参数才能获取最新值
- 建议在 `arm_controller_node` 中添加参数监听机制以实现动态更新

## 扩展建议

如果需要更新姿态参数,可以修改 `poseCallback` 函数:

```cpp
void PoseParamUpdater::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 位置
    nh_.setParam(param_x_, msg->pose.position.x);
    nh_.setParam(param_y_, msg->pose.position.y);
    nh_.setParam(param_z_, msg->pose.position.z);
    
    // 姿态 (四元数转欧拉角)
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    nh_.setParam("/test/line_point_roll", roll);
    nh_.setParam("/test/line_point_pitch", pitch);
    nh_.setParam("/test/line_point_yaw", yaw);
}
```

## 故障排查

1. **节点无法启动**: 检查是否已编译 (`catkin_make`)
2. **参数未更新**: 使用 `rostopic echo /target_pose` 确认话题有数据
3. **话题名称错误**: 使用 `rostopic list` 查看可用话题
4. **权限问题**: 确保参数服务器可写

## 联系方式

如有问题,请查看ROS日志: `rosrun rqt_console rqt_console`
