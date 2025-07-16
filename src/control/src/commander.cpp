/*
    CS22B1090
    Shubh Khandelwal
*/

#include <description/srv/gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class CommanderNode : public rclcpp::Node
{

    private:

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joints_publisher;
    rclcpp::Service<description::srv::Gripper>::SharedPtr gripper_service;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void gripper_callback(const std::shared_ptr<description::srv::Gripper::Request> request, std::shared_ptr<description::srv::Gripper::Response> response)
    {

        trajectory_msgs::msg::JointTrajectory joints_msg;
        joints_msg.joint_names = {"joint_01", "joint_23", "joint_45", "joint_67"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {request->joints[0], request->joints[1], request->joints[2], request->joints[3]};
        point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(10));

        joints_msg.points.push_back(point);
        joints_publisher->publish(joints_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::string target_frame = "base_link";
        std::string source_frame = "link_8";

        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.01)
            );
            response->success = true;
            response->gripper[0] = t.transform.translation.x;
            response->gripper[1] = t.transform.translation.y;
            response->gripper[2] = t.transform.translation.z;
        } catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF Lookup Error: %s", ex.what());
            response->success = false;
            response->gripper[0] = 0;
            response->gripper[1] = 0;
            response->gripper[2] = 0;
        }

    }

    public:

    CommanderNode() : Node("commander_node")
    {

        joints_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
        gripper_service = this->create_service<description::srv::Gripper>(
            "/get_gripper",
            std::bind(&CommanderNode::gripper_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommanderNode>());
    rclcpp::shutdown();
    return 0;
}