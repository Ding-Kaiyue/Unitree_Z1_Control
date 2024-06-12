#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmFKNode : public rclcpp::Node
{
public:
    ArmFKNode(const std::string &node_name) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "ArmFKNode started");
        arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("z1_group"));
        // 设置机械臂运动允许的误差
        arm->setGoalJointTolerance(0.0007);
        // 设置机械臂最大速度及加速度缩放因子
        arm->setMaxAccelerationScalingFactor(0.5);
        arm->setMaxVelocityScalingFactor(0.5);

        // 令机械臂先回到初始位置
        arm->setNamedTarget("home");
        arm->move();
        sleep(1);
        std::vector<double> joint_group_positions(6);
        double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
        for (int i = 0; i < 6; i++) {
            joint_group_positions[i] = targetPose[i];
        }
        // 写入关节值
        arm->setJointValueTarget(joint_group_positions);
        // 运动规划
        arm->move();
        sleep(1);
    }
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmFKNode>("arm_fk");

    rclcpp::spin(node);
    // 声明执行器（多线程）
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);

    // executor.spin();
    rclcpp::shutdown();
    return 0;
}