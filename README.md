**2024/05/27**
* 本工作空间基于ROS2实现
* 实现机械臂本体在gazebo的规划运动仿真，夹爪暂不可用
* 由于不知名原因，引用.STL和.dae文件时，不能使用相对路径(package://robot_description/meshes/visual)，只能使用绝对路径，使用时需修改成自己的路径
* 运行时使用的命令行
    ```shell
    # 第一个terminal
    colcon build
    source install/setup.bash
    ros2 launch robot gazebo.launch.py
    ```
    ```
    # 第二个terminal
    source install/setup.bash
    ros2 launch robot usr_moveit.launch.py
    ```
    ```shell
    # 第三个terminal
    source install/setup.bash
    ros2 run kinematics fk_node # 或ik_node
    ```
* 在rviz2中调整末端执行器位姿，点击plan and execute，在gazebo中即可观测到机械臂的运动
* 调整kinematics/ik.cpp中的target_pose参数，可以调整期望的末端执行器位姿
* 调整kinematics/fk.cpp中的targetPose[6]参数，可以调整六个joint的角度
