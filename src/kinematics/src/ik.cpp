#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class IkNode : public rclcpp :: Node {
    public:
        IkNode(const std::string &name) : Node(name) {
            arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("z1_group"));
            std::string end_effector_link = arm->getEndEffectorLink();    // 获取终端link名称
           
            std::string reference_frame = "link00";       // 设置目标位置所使用的参考坐标系
            arm->setPoseReferenceFrame(reference_frame);

            arm->allowReplanning(true);     // 当运动规划失败后，允许重新规划

            arm->setGoalPositionTolerance(0.0005);      // 设置位置(m)的允许误差
            arm->setGoalOrientationTolerance(0.0007);   // 设置姿态(rad)的允许误差

            arm->setMaxVelocityScalingFactor(0.2);      // 设置允许的最大速度和加速度
            arm->setMaxAccelerationScalingFactor(0.2);

            arm->setNamedTarget("home");     // 令机械臂先回到初始位置
            arm->move();
            sleep(1);

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = 0.2;
            target_pose.position.y = 0.2;
            target_pose.position.z = 0.45;
            target_pose.orientation.x = 0.70692;
            target_pose.orientation.y = 0;
            target_pose.orientation.z = 0;
            target_pose.orientation.w = 0.70729;

            // 设置机械臂当前运动状态为初始状态
            arm->setStartStateToCurrentState();

            // 写入目标位姿
            arm->setPoseTarget(target_pose, end_effector_link);

            // 运动规划, 此时机械臂不会运动
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::core::MoveItErrorCode success = arm->plan(plan);

            // 是否规划成功
            RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success? "SUCCEED" : "FAILED");

            // 运动执行
            if(success) {
                arm->execute(plan);
            }
        }
    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IkNode>("arm_ik");
    rclcpp::spin(node);
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);

    // executor.spin();
    rclcpp::shutdown();
    return 0;
}