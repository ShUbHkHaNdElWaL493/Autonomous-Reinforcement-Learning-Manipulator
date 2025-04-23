/*
    CS22B1090
    Shubh Khandelwal
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class GzBridgeNode : public rclcpp::Node
{

    private:

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_01_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_23_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_45_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_67_subscriber;

    rclcpp::TimerBase::SharedPtr timer;

    std::vector<double> positions;

    void joint_01_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        positions[0] = msg->data;
    }

    void joint_23_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        positions[1] = msg->data;
    }

    void joint_45_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        positions[2] = msg->data;
    }

    void joint_67_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        positions[3] = msg->data;
    }

    void publish_joint_states()
    {

        trajectory_msgs::msg::JointTrajectory joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.joint_names = {"joint_01", "joint_23", "joint_45", "joint_67"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = this->positions;
        point.time_from_start = rclcpp::Duration::from_seconds(0.01);

        joint_msg.points.push_back(point);
        joint_trajectory_publisher->publish(joint_msg);

    }

    public:

    GzBridgeNode() : Node("gz_bridge_node")
    {

        joint_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/manipulator_controller/joint_trajectory", 10);

        joint_01_subscriber = this->create_subscription<std_msgs::msg::Float64>("/joint_01", 10, std::bind(&GzBridgeNode::joint_01_callback, this, std::placeholders::_1));
        joint_23_subscriber = this->create_subscription<std_msgs::msg::Float64>("/joint_23", 10, std::bind(&GzBridgeNode::joint_23_callback, this, std::placeholders::_1));
        joint_45_subscriber = this->create_subscription<std_msgs::msg::Float64>("/joint_45", 10, std::bind(&GzBridgeNode::joint_45_callback, this, std::placeholders::_1));
        joint_67_subscriber = this->create_subscription<std_msgs::msg::Float64>("/joint_67", 10, std::bind(&GzBridgeNode::joint_67_callback, this, std::placeholders::_1));

        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GzBridgeNode::publish_joint_states, this));

        positions = std::vector<double>(4, 0.0);

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzBridgeNode>());
    rclcpp::shutdown();
    return 0;
}