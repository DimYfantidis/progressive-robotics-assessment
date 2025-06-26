#include <random>
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/empty.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "rviz_visual_tools/rviz_visual_tools.hpp"

#ifndef M_PI
#   define M_PI 3.141592653589793
#endif


using namespace std::chrono_literals;
using std::placeholders::_1;


class JointConfigurationPublisher : public rclcpp::Node
{
public:

    JointConfigurationPublisher()
        :   
        Node("ur20_joint_configuration_publisher"),

        generator_{std::random_device{}()},

        angle_distribution_(-M_PI, M_PI)
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

        if (
            // Input check
            parameters_.joint_names.size() != parameters_.joint_positions.size() || 
            parameters_.joint_positions.size() != parameters_.joint_velocities.size() || 
            parameters_.joint_velocities.size() != parameters_.joint_efforts.size()
        )
        {
            throw std::runtime_error(
                "INPUT ERROR: Invalid joint configuration. "
                "`joint_names`, `joint_positions`, `joint_velocities` and `joint_efforts` "
                "parameters must be sequences of equal lengths."
            );
        }

        size_t num_joints = parameters_.joint_positions.size();

        initial_joint_positions_.resize(num_joints);
        goal_joint_positions_.resize(num_joints);

        for (size_t i = 0; i < num_joints; ++i)
        {
            // Initial joint positions.
            initial_joint_positions_[i] = parameters_.joint_positions[i];
            // Randomly generated final joint positions.
            goal_joint_positions_[i] = angle_distribution_(generator_);
        }
        current_joint_positions_ = initial_joint_positions_;


        // docs: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html#id2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf2_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<uint64_t>(1E9 / parameters_.frequency)),
            std::bind(&JointConfigurationPublisher::tf_listener_callback_, this)
        );

        rclcpp::SubscriptionOptions options;
        options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Joint state publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        publisher_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<uint64_t>(1E9 / parameters_.frequency)),
            std::bind(&JointConfigurationPublisher::publisher_callback_, this)
        );
        animate_command_listener_ = this->create_subscription<std_msgs::msg::Empty>(
            "/" + parameters_.tf_prefix + "animate_cmd", 
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&JointConfigurationPublisher::animate_command_callback_, this, _1),
            options
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

        // Lamda function for the convenient conversion from Transform format to Isometry3D format.
        // see: https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
        auto get_isometry = [](geometry_msgs::msg::TransformStamped& t)
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
            // This means that the three transhould_animate_sforms are correct!
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
        msg.velocity = parameters_.joint_velocities;
        msg.effort = parameters_.joint_efforts;
        msg.position = std::vector<double>(
            current_joint_positions_.data(),
            current_joint_positions_.data() + current_joint_positions_.size()
        );

        publisher_->publish(msg);
    }

    /// @brief  Callback function that listens for "animate" commands.
    ///         An "animate" command instructs the robot to start animating
    ///         by changing its joint positions (or rather angles) over a time 
    ///         frame of 4.5 seconds
    /// @param msg
    ///     Serves as a placeholder. Essentially, by sending a dummy message to 
    ///     the `/animate_cmd` topic, the user will instruct the robot to animate.
    ///     Simply type `ros2 topic pub --once /animate_cmd std_msgs/msg/Empty '{}'`
    ///     into a separate terminal.
    void animate_command_callback_(const std_msgs::msg::Empty& msg)
    {
        // Suppress "unused variable" compiler warnings.
        std::ignore = msg;

        double t0 = this->now().seconds() + this->now().nanoseconds() * 1E-9;
        double t = t0;

        constexpr double PERIOD = 3.0;
        constexpr double TOTAL_TIME = 1.5 * PERIOD;
        constexpr double TIME_STEP = 0.034;

        while (t - t0 < TOTAL_TIME)
        {
            // These references are simply for shortening the variable names so
            // that the interpolation's formula doesn't give a headache to the reader.
            Eigen::VectorXd& current = current_joint_positions_;
            Eigen::VectorXd& init = initial_joint_positions_;
            Eigen::VectorXd& goal = goal_joint_positions_;
            
            // Interpolation factor
            double f = std::sin(2.0 * M_PI * t / PERIOD);

            // The main idea behind the formula comes from the parameterization of a straight line segment.
            // see: https://www.geeksforgeeks.org/maths/parametrization-of-a-line/
            current = init + (goal - init) * f;

            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<uint64_t>(1000 * TIME_STEP))
            );
            t = this->now().seconds() + this->now().nanoseconds() * 1E-9;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr tf2_timer_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    Eigen::VectorXd initial_joint_positions_;

    Eigen::VectorXd goal_joint_positions_;

    Eigen::VectorXd current_joint_positions_;

    std::mt19937 generator_;

    std::uniform_real_distribution<double> angle_distribution_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr animate_command_listener_;


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

    // It is very important that we use a multithreaded executor in this case 
    // as `JointConfigurationPublisher::animate_command_callback_` has sleep statements.
    // 
    // If the default executor is used (`rclcpp::executors::SingleThreadedExecutor`), then 
    // the TF listener and the publisher will halt their execution until `animate_command_callback_` 
    // has finished; an undesired outcome. 
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<JointConfigurationPublisher> node = std::make_shared<JointConfigurationPublisher>();

    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}