/*
    CS22B1090
    Shubh Khandelwal
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointStateNode : public rclcpp::Node
{

    private:

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::JointState joint_state;

    void publish_joint_states()
    {
        joint_state.header.stamp = now();
        joint_state_publisher->publish(joint_state);
    }

    void update_joint_states(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty()) 
        {
            joint_state.position = msg->data;
        }
    }

    public:

    JointStateNode() : Node("joint_state_node")
    {
        joint_state.name = {"joint_01", "joint_23", "joint_45", "joint_67"};
        joint_state.position = {0.0, 0.0, 0.0, 0.0};
        joint_state.velocity = {0.0, 0.0, 0.0, 0.0};
        joint_state.effort = {0.0, 0.0, 0.0, 0.0};
        joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        position_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("/position", 10, std::bind(&JointStateNode::update_joint_states, this, std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JointStateNode::publish_joint_states, this));
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateNode>());
    rclcpp::shutdown();
    return 0;
}