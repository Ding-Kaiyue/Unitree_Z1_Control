#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "robot_interfaces/msg/jointpos.hpp"
#include <vector>
#include <algorithm>

using namespace std;

class RobotControl : public rclcpp::Node {
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

        explicit RobotControl(std::string name);
        ~RobotControl(){}

        void timer_callback();
    private:
        robot_interfaces::msg::Jointpos joint_msg;
        int arm_type = 65;     //机械臂型号
        bool follow = false;    // 低跟随模式
        // 实例化样条
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server;
        // 话题发布者
        rclcpp::Publisher<robot_interfaces::msg::Jointpos>::SharedPtr joint_pos_pub;
        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel();
        void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
};


#endif