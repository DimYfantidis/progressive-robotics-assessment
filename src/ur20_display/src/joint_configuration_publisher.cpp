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

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <rviz_visual_tools/rviz_visual_tools.hpp>


using namespace std::chrono_literals;


class JointConfigurationPublisher : public rclcpp::Node
{
public:

    JointConfigurationPublisher()
    :   Node("ur20_joint_configuration_publisher")
    {
        // Joints configuration, specified through the parameter server.
        this->declare_parameter<std::string>("tf_prefix", "");
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());
        this->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>());
        this->declare_parameter<std::vector<double>>("joint_velocities", std::vector<double>());
        this->declare_parameter<std::vector<double>>("joint_efforts", std::vector<double>());
        this->declare_parameter<double>("frequency", 30.0);
        // Store the parameters
        parameters_.tf_prefix = this->get_parameter("tf_prefix").as_string();
        parameters_.joint_names = this->get_parameter("joint_names").as_string_array();
        parameters_.joint_positions = this->get_parameter("joint_positions").as_double_array();
        parameters_.joint_velocities = this->get_parameter("joint_velocities").as_double_array();
        parameters_.joint_efforts = this->get_parameter("joint_efforts").as_double_array();
        parameters_.frequency = this->get_parameter("frequency").as_double();   // Hz

        // docs: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html#id2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Joint state publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        tf2_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<uint64_t>(1E9 / parameters_.frequency)),
            std::bind(&JointConfigurationPublisher::tf_listener_callback_, this)
        );
        publisher_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<uint64_t>(1E9 / parameters_.frequency)),
            std::bind(&JointConfigurationPublisher::publisher_callback_, this)
        );
        // RVIZ visual tools docs: https://github.com/PickNikRobotics/rviz_visual_tools/blob/ros2/README.md
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers", this));
    }


private:

    /// @brief  Listens to the transform between the robot's `gripper` relative to the robot's `elbow`, 
    ///         the transform between the robot's `elbow` relative to the `world`, and the transform
    ///         between the robot's `gripper` relative to the `world`. Finally, the code's and the transforms' 
    ///         correctness is validated.
    void tf_listener_callback_()
    {
        // The three requested transforms.
        geometry_msgs::msg::TransformStamped tf_elbow_gripper;
        geometry_msgs::msg::TransformStamped tf_world_elbow;
        geometry_msgs::msg::TransformStamped tf_world_gripper;

        // The transforms in Isometry 3D formats.
        Eigen::Isometry3d iso3d_elbow_gripper;
        Eigen::Isometry3d iso3d_world_elbow;
        Eigen::Isometry3d iso3d_world_gripper;

        // Lamda function for the convenient conversion from the first type to the second one.
        // see: https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
        auto get_isometry = [](
            geometry_msgs::msg::TransformStamped& t
        )
        {
            Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();

            Eigen::Quaterniond quaternion{
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z
            };

            Eigen::Vector3d displacement{
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            };

            isometry.linear() = quaternion.toRotationMatrix();
            isometry.translation() = displacement;

            return isometry;
        };

        // Convenient lamda function for more compact code.
        auto lookup_transform = [&](
            // One of the three requested transforms (e.g. tf_world_gripper)
            geometry_msgs::msg::TransformStamped& t,
            // e.g. world
            std::string source,
            // e.g. gripper
            std::string dest,
            // Result by reference (e.g. iso3d_world_gripper)
            Eigen::Isometry3d& iso3d
        )
        {
            // see: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html#id2
            try
            {
                t = tf_buffer_->lookupTransform(
                    parameters_.tf_prefix + dest,
                    parameters_.tf_prefix + source, 
                    tf2::TimePointZero
                );
                
                iso3d = get_isometry(t);

                RCLCPP_INFO(
                    this->get_logger(),
                    "\nTransform %s -> %s:"
                    "\n> translation: (x=%.2lf, y=%.2lf, z=%.2lf) "
                    "\n> rotation:    (x=%.2lf, y=%.2lf, z=%.2lf, w=%.2lf)",
                    source.c_str(),
                    dest.c_str(),
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                );
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_INFO(
                    this->get_logger(), 
                    "Could not transform \"%s\" to \"%s\": %s",
                    source.c_str(), dest.c_str(), e.what()
                );
                return;
            }
        };

        // Listen for all three requested transforms.
        lookup_transform(tf_world_elbow, "world", "upper_arm_link", iso3d_world_elbow);
        lookup_transform(tf_elbow_gripper, "upper_arm_link", "gripper_link", iso3d_elbow_gripper);
        lookup_transform(tf_world_gripper, "world", "gripper_link", iso3d_world_gripper);

        // (world -> elbow) + (elbow -> gripper) = (world -> gripper)
        Eigen::Isometry3d combined = iso3d_elbow_gripper * iso3d_world_elbow;

        if (combined.isApprox(iso3d_world_gripper, 0.0001))
        {
            // This means that the three transforms are correct!
            RCLCPP_INFO(this->get_logger(), "Transforms validated successfully.");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "INVALID TRANSFORMS: examine (world -> elbow), (elbow -> gripper) "
                "and (world -> gripper) transforms for errors"
            );
        }

        // RVIZ visual tools docs: https://github.com/PickNikRobotics/rviz_visual_tools/blob/ros2/README.md
        visual_tools_->publishAxisLabeled(iso3d_world_gripper, "Tf_elbow_gripper", rviz_visual_tools::LARGE, rviz_visual_tools::RED);
        visual_tools_->trigger();
    }

    /// @brief UR20's joint state publisher.
    void publisher_callback_()
    {
        auto msg = sensor_msgs::msg::JointState();

        msg.header.stamp = this->now();
        msg.name = parameters_.joint_names;
        msg.position = parameters_.joint_positions;
        msg.velocity = parameters_.joint_velocities;
        msg.effort = parameters_.joint_efforts;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr tf2_timer_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


    // All variables that refer to the ROS node's parameters are grouped 
    // together within this data structure for semantic purposes.
    struct parameters_t {

        std::string tf_prefix;

        std::vector<std::string> joint_names;

        std::vector<double> joint_positions;

        std::vector<double> joint_velocities;

        std::vector<double> joint_efforts;

        double frequency;

    } parameters_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointConfigurationPublisher>());
    rclcpp::shutdown();
    return 0;
}