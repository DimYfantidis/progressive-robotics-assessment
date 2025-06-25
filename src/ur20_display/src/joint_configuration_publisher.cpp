#include <memory>
#include <chrono>
#include <algorithm>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;


class JointConfigurationPublisher : public rclcpp::Node
{
public:

    JointConfigurationPublisher()
    :   Node("ur20_configuration_publisher")
    {
        // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

        // param_desc.name = "joint_states";
        // param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;

        this->declare_parameter<std::vector<std::string>>("joint_names", {});
        this->declare_parameter<std::vector<double>>("joint_positions", {});
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&JointConfigurationPublisher::publisher_callback_, this)
        );
    }


private:

    void publisher_callback_()
    {
        auto joint_names = this->get_parameter("joint_names").as_string_array();
        auto joint_positions = this->get_parameter("joint_positions").as_double_array();

        auto msg = sensor_msgs::msg::JointState();

        msg.header.stamp = this->get_clock()->now();
        msg.name = std::move(joint_names);
        msg.position = std::move(joint_positions);

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointConfigurationPublisher>());
    rclcpp::shutdown();
    return 0;
}