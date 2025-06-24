// Standard C++ compiler headers
#include <memory>
#include <string>
#include <array>
#include <chrono>
#include <cctype>
#include <iomanip>
#include <algorithm>
#include <functional>

// ROS2 client Library
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "linear_algebra_resources/srv/least_squares.hpp"
// ROS2 filesystem management
#include "ament_index_cpp/get_package_share_directory.hpp"

// Secondary dependencies
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"


using std::placeholders::_1;
using std::placeholders::_2;


class LinearAlgebraClient : public rclcpp::Node
{
public:

    LinearAlgebraClient()
        : Node("linear_algebra_client")
    {
        // Parameter handling (1)
        this->declare_parameter("yaml_filename", "client_data.yaml");
        this->declare_parameter("topic", "client");
        this->declare_parameter("frequency", 2.0);
        /// Parameter handling (2)
        parameters_.yaml_filename = this->get_parameter("yaml_filename").as_string();
        parameters_.topic = this->get_parameter("topic").as_string();
        parameters_.frequency = this->get_parameter("frequency").as_double();

        // Get the path of the resources package to fetch the YAML file with the input data.
        std::string pkg_path = ament_index_cpp::get_package_share_directory("linear_algebra_resources");
        std::string yaml_path = pkg_path + "/config/" + parameters_.yaml_filename;

        // Instantiate publisher.
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            parameters_.topic,
            rclcpp::QoS(rclcpp::KeepLast(10))
        );
        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<uint64_t>(1e9 / parameters_.frequency)), 
            std::bind(&LinearAlgebraClient::publisher_callback_, this)
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Client publishing String messages on \"%s\" topic",
            parameters_.topic.c_str()
        );
        
        // Start the client and send attempt to establish comms with service.
        client_ = this->create_client<linear_algebra_resources::srv::LeastSquares>("compute_least_squares");
        send_request_(yaml_path);
    }


private:

    /// @brief  Callback function of the client node's publisher. The publisher sends
    ///         messages over the "client" topic.
    void publisher_callback_()
    {
        auto msg = std_msgs::msg::String();
        
        msg.data = "Hello #" + std::to_string(counter_++) + " from Client node!";

        publisher_->publish(msg);
    }

    /// @brief Send a request for the linear least squares solution of the system ||Ax - b||
    /// @param filepath 
    ///     The absolute path of the YAML file containing the input data, i.e. matrix A: Mx3 and vector b: Mx1.
    ///     It should be present within the `config` directory of the `linear_algebra_resources` package.
    ///     Can be specified as a `LinearAlgebraClient` node parameter. Defaults to "client_data.yaml".
    void send_request_(const std::string& filepath)
    {
        YAML::Node yaml_data = YAML::LoadFile(filepath);

        auto request = std::make_shared<linear_algebra_resources::srv::LeastSquares::Request>();

        // We need to make sure that all three provided columns have the same size
        if (
            yaml_data["column1"].size() != yaml_data["column2"].size() || 
            yaml_data["column2"].size() != yaml_data["column3"].size()
        )
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "INPUT ERROR: Expected an Mx3 matrix A. Make sure the `column*` fields in \"%s\" are YAML sequences of equal length."
                "Cannot send request to \"compute_least_squares\" service.",
                filepath.c_str()
            );
            return;
        }
        // We also need to make sure that the number of rows in the matrix and 
        // the number of the vector's scalars are equal.
        if (yaml_data["vector"].size() != yaml_data["column1"].size())
        {
            RCLCPP_ERROR(
                this->get_logger(), 
                "INPUT ERROR: Expected an Mx3 matrix A and an M-dimensional vector b, but received "\
                "imensions are A: %zux3 and b: %zux1. Cannot send request to \"compute_least_squares\" service. "\
                "Inspect file at: \"%s\"",
                yaml_data["column1"].size(), yaml_data["vector"].size(), filepath.c_str()
            );
            return;
        }

        // Compute the number of rows, referred to as `M` in the problem description
        size_t row_length = yaml_data["vector"].size();


        // Resize vector to match dimensionality
        request->vector.resize(row_length);

        for (size_t i = 0; i < row_length; ++i)
        {
            // Read scalars
            request->vector[i] = yaml_data["vector"][i].as<double>();
        }

        // Resize dynamic array to match the dimensionality
        request->matrix.resize(row_length);
        
        for (size_t i = 0; i < row_length; ++i)
        {
            // Read row-wise matrix data
            request->matrix[i].x = yaml_data["column1"][i].as<double>();
            request->matrix[i].y = yaml_data["column2"][i].as<double>();
            request->matrix[i].z = yaml_data["column3"][i].as<double>();
        }

        // Wait for server to become available.
        while (!client_->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_INFO(this->get_logger(), "Service \"compute_least_squares\" hasn't started yet. Retrying in 1 sec...");

            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Established communication with \"compute_least_squares\" service.");

        std::ignore = client_->async_send_request(
            request,
            std::bind(&LinearAlgebraClient::response_callback_, this, _1)
        );
    }

    /// @brief Starts execution when the server responds with the computed result.
    /// @param future 
    ///     Contains the response message that is expected to be computed by the `LinearAlgebraServer`.
    void response_callback_(rclcpp::Client<linear_algebra_resources::srv::LeastSquares>::SharedFuture future)
    {
        auto response = future.get();

        RCLCPP_INFO(this->get_logger(), "Received response.");

        Eigen::Vector3d solution{
            response->solution.x,
            response->solution.y,
            response->solution.z
        };
        Eigen::Vector3d displacement{
            response->displacement_vector.x,
            response->displacement_vector.y,
            response->displacement_vector.z
        };
        Eigen::Matrix3d rotation_matrix{
            {
                response->rotation_matrix[0].x,
                response->rotation_matrix[0].y,
                response->rotation_matrix[0].z,
            },
            {
                response->rotation_matrix[1].x,
                response->rotation_matrix[1].y,
                response->rotation_matrix[1].z,
            },
            {
                response->rotation_matrix[2].x,
                response->rotation_matrix[2].y,
                response->rotation_matrix[2].z,
            },
        };

        // Current vector is x' = R * x + d
        // To fetch x (solution), we need to translate the rotated vector back to its original position:
        solution = solution - displacement;
        // Then we must reverse the rotation using the rotation matrix's inverse.
        // The inverse of a rotation matrix is always equal to its transpose:
        solution = rotation_matrix.transpose() * solution;

        RCLCPP_INFO(
            this->get_logger(), 
            "Computed solution is: [%lf, %lf, %lf]", 
            solution.x(), solution.y(), solution.z()
        );
    }
    

private:

    rclcpp::Client<linear_algebra_resources::srv::LeastSquares>::SharedPtr client_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t counter_;

    // All variables that refer to the ROS node's parameters are grouped 
    // together within this data structure for semantic purposes.
    struct parameters_t {

        std::string yaml_filename;
        
        std::string topic;

        double frequency;

    } parameters_; 
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearAlgebraClient>());
    rclcpp::shutdown();
    return 0;
}
