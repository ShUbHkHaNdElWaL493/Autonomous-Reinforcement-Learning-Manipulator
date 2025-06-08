/*
    CS22B1090
    Shubh Khandelwal
*/

#include <manipulator/srv/gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ROSBridgeNode : public rclcpp::Node
{

    private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_publisher;
    rclcpp::Service<manipulator::srv::Gripper>::SharedPtr gripper_service;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    void gripper_callback(const std::shared_ptr<manipulator::srv::Gripper::Request> request, std::shared_ptr<manipulator::srv::Gripper::Response> response)
    {

        std_msgs::msg::Float64MultiArray joints_msg;
        joints_msg.layout = std_msgs::msg::MultiArrayLayout();
        joints_msg.data = {request->joints[0], request->joints[1], request->joints[2], request->joints[3]};
        joints_publisher->publish(joints_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

    ROSBridgeNode() : Node("ros_bridge_node")
    {

        joints_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
        gripper_service = this->create_service<manipulator::srv::Gripper>(
            "/get_gripper",
            std::bind(&ROSBridgeNode::gripper_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROSBridgeNode>());
    rclcpp::shutdown();
    return 0;
}