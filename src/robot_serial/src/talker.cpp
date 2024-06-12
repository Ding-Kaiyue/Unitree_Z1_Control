#include <functional>
#include <memory>
#include <string>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

 
using namespace std::chrono_literals;
using std::placeholders::_1;
serial::Serial ros_ser;

int ros_ser_init()
{
    ros_ser.setPort("/dev/ttyACM0");
    ros_ser.setBaudrate(115200);
    serial::Timeout to =serial::Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);

    try
    {
        ros_ser.open();
    }
    catch(const std::exception& e)
    {
        std::cout<<"unable to open"<<std::endl;     //若打开串口失败打印"unable to open"到终端
        return -1;
    }
    
    if(ros_ser.isOpen())
    {
        std::cout<<"open"<<std::endl;              //若打开串口打印"open"到终端
    }
    else
    {
        return -1;
    }
}

class JointStates : public rclcpp :: Node 
{
    public:
        JointStates(const std::string &name) : Node(name) {
            subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStates::joint_states_callback, this, _1));
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        float joint_states_[6] = {0.0f};
        unsigned char buffer[24] = {0};
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {    
            for(size_t i = 0; i < msg->name.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "joint %s position %f", msg->name[i].c_str(), msg->position[i]);
                send_position_via_serial(msg->position[i]);
            }
        }
        void send_position_via_serial(double position)
        {
            std::string data = std::to_string(position);
            ros_ser.write(data);
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStates>("joint_states_subscribe");
    ros_ser_init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    ros_ser.close();
    return 0;
}
