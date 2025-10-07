## 完成的工作

1. **模块化设计**: 在src/unitree_ros-master/robots/z1_description/xacro创建了一个独立的 `camera.xacro` 文件，用于定义相机的所有属性，以便于维护和复用。
2. **相机集成**: 在主机器人模型 `robot.xacro` 中成功引用了相机模块，并将其固定在夹爪的 `gripperMover` 连杆上。
3. **传感器配置**: 在 Gazebo 中为相机配置了传感器插件 (`libgazebo_ros_camera.so`)，使其能够以 30Hz 的频率发布 800x800 分辨率的彩色图像。
4. **模型姿态调整**: 修正了 D435 模型在仿真中默认的垂直姿态，通过在 URDF 中应用 RPY 旋转，使其恢复为正常的水平放置姿态。

## 后续步骤

- 导入操作管道模型
- 逆运动解算末端坐标和角度

## 注意事项

- 相机相对机械臂的坐标不正确
- 如果需要深度信息，可将 `libgazebo_ros_camera.so` 插件更换为 `libgazebo_ros_openni_kinect.so`，以同时获取彩色图、深度图和点云数据