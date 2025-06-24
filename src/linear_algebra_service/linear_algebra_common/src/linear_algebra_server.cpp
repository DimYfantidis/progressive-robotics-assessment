// Standard C++ compiler headers
#include <cmath>
#include <mutex>
#include <memory>
#include <string>
#include <limits>
#include <random>
#include <thread>
#include <atomic>
#include <chrono>
#include <cctype>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <condition_variable>

// ROS client Library
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "linear_algebra_resources/srv/least_squares.hpp"

// Secondary dependencies
#include "Eigen/Dense"

#ifndef M_PI
#   define M_PI 3.141592653589793
#endif

using std::placeholders::_1;
using std::placeholders::_2;


class LinearAlgebraServer : public rclcpp::Node
{
public:

    LinearAlgebraServer()
        : 
        Node("linear_algebra_server"),

        generator_{std::random_device{}()},

        angle_distribution_(-M_PI, M_PI),

        unbounded_distribution_(
            std::numeric_limits<double>::min(), 
            std::numeric_limits<double>::max()
        ),

        bounded_distribution_(-1.0, 1.0)
    {
        service_ = this->create_service<linear_algebra_resources::srv::LeastSquares>(
            "compute_least_squares",
            std::bind(&LinearAlgebraServer::service_callback_, this, _1, _2)
        );
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "client",
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&LinearAlgebraServer::subscription_callback_, this, _1)    
        );

        // Initialise thread resources
        worker_resources_.is_thread_running = ATOMIC_VAR_INIT(true);
        worker_resources_.has_received_message = false;
        worker_resources_.logged_output = true;

        // Instantiate thread
        worker_resources_.thread_ptr = std::make_unique<std::thread>(
            std::bind(&LinearAlgebraServer::thread_worker_, this)
        );

        RCLCPP_INFO(this->get_logger(), "Listening for requests...");
    }

    ~LinearAlgebraServer()
    {
        // This is to make the worker thread break out of its loop.
        worker_resources_.is_thread_running = false;

        // This is to make the worker thread release the mutex.
        this->worker_resources_.has_received_message = true;
        worker_resources_.condition.notify_one();

        worker_resources_.thread_ptr->join();
    }


private:
    
    /// @brief Callback function for `LinearAlgebraServer`'s subscriber. The subscriber
    ///        receives messages from the "client" topic. Function structured according 
    ///        to guidelines and sample code provided in the C++ reference docs.
    ///        see: https://en.cppreference.com/w/cpp/thread/condition_variable.html
    /// @param msg
    ///     A message of type `std_msgs::msg::String`. Message type was not specified in
    ///     the problem description, and thus `std_msgs::msg::String` was chosen arbitrarily.
    void subscription_callback_(const std::shared_ptr<std_msgs::msg::String> msg)
    {
        // Wait for the worker.
        {
            std::unique_lock lock(worker_resources_.mutex);
            worker_resources_.condition.wait(
                lock, 
                [this] { return (this->worker_resources_.logged_output ? true : false); }
            );
        }
        worker_resources_.condition.notify_one();
        // Send data to the worker thread.
        {
            std::lock_guard lock(worker_resources_.mutex);

            // Evaluate the worker thread's condition to true.
            worker_resources_.has_received_message = true;

            // Give the new message to the worker thread.
            worker_resources_.received_msg_ptr = msg; 

            // Allow the thread worker to log the message before listening for another message.
            this->worker_resources_.logged_output = false;
        }
        worker_resources_.condition.notify_one();
    }

    /// @brief Function passed to the worker thread. It simply logs the next message received
    ///        by the subscriber. Function structured according to guidelines and sample code 
    ///        provided in the C++ reference docs.
    ///        see: https://en.cppreference.com/w/cpp/thread/condition_variable.html
    void thread_worker_()
    {
        while (rclcpp::ok())
        {
            // Wait until data is received by the publisher.
            std::unique_lock lock(worker_resources_.mutex);
            
            worker_resources_.condition.wait(
                lock, 
                [this] { return (this->worker_resources_.has_received_message ? true : false); }
            );

            if (!worker_resources_.is_thread_running)
            {
                // This is to make sure the 
                this->worker_resources_.logged_output = true;
                worker_resources_.condition.notify_one();
                return;
            }

            RCLCPP_INFO(
                this->get_logger(), 
                "I heard: %s", 
                worker_resources_.received_msg_ptr->data.c_str()
            );
            
            // Send data back to publisher.
            worker_resources_.logged_output = true;
            worker_resources_.has_received_message = false;
            
            // Manual unlocking is done before notifying, to avoid 
            // waking up the waiting thread only to block again.
            lock.unlock();
            worker_resources_.condition.notify_one();
        }
    }

    /// @brief  Callback function for handling incoming requests from a client. 
    ///         The server receives a linear system and returns its linear least squares
    ///         solution. The solution is then rotated by a random 3D rotation and then
    ///         displaced by a random 3D vector.
    /// @param request 
    ///     ROS2 service message request. Contains an Mx3 matrix and a M-dimensional vector.
    /// @param response 
    ///     ROS2 service message response. Contains the rotated-and-displaced solution, the 
    ///     rotation matrix and the displacement vector, along with a `success` variable.
    ///     `success` indicates whether the input received by the client is correctly formatted.
    void service_callback_(
        const std::shared_ptr<linear_algebra_resources::srv::LeastSquares::Request> request,
        std::shared_ptr<linear_algebra_resources::srv::LeastSquares::Response> response
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received request for least squares...");

        if (request->matrix.size() != request->vector.size())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "INPUT ERROR: Service expected an Mx3 matrix and M-dimensional vector "
                "but received dimensions are %zux3 and %zux1. Service could not proceed.",
                request->matrix.size(), request->vector.size()
            );
            response->success = false;
            return;
        }

        // std::vector::size is equivalent to the number of rows
        size_t row_num = request->matrix.size();

        Eigen::MatrixXd A(row_num, 3);
        Eigen::VectorXd b(row_num);

        for (size_t i = 0; i < row_num; ++i)
        {
            // Turn the requested matrix into the appropriate format (Eigen::Matrix)
            A(i, 0) = request->matrix[i].x;
            A(i, 1) = request->matrix[i].y;
            A(i, 2) = request->matrix[i].z;

            // Turn the requested vector into the appropriate format (Eigen::Matrix)
            b(i) = request->vector[i];
        }

        response->displacement_vector.x = 2.0;

        // Linear least squares computed as shown in Eigen's docs.
        // see: https://eigen.tuxfamily.org/dox-devel/group__LeastSquares.html

        // Solution vector
        Eigen::Vector3d x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        double angle = angle_distribution_(generator_);

        // Normally the `LinearAlgebraServer::unbunded_distribution` should be used as
        // the range of the random translation is not stated in the problem description.
        // However to avoid any unecessary confusion caused by floating point errors, the 
        // bounded distribution in the range [-1000.0, 1000.0] was preferred.
        Eigen::Vector3d d{
            1000.0 * bounded_distribution_(generator_),
            1000.0 * bounded_distribution_(generator_),
            1000.0 * bounded_distribution_(generator_)
        };
        // Same for the axis of the random rotation
        Eigen::Vector3d axis{
            bounded_distribution_(generator_),
            bounded_distribution_(generator_),
            bounded_distribution_(generator_)
        };
        // If the axis vector is not normalized, then the angle-axis object represents an invalid rotation
        // see: https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
        axis.normalize();

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

        // Rotate the solution vector
        x = R * x;
        // Translate the vector
        x = x + d;

        // Return solution to the Client
        response->solution.x = x(0);
        response->solution.y = x(1);
        response->solution.z = x(2);

        // Return the displacement vector to the Client
        response->displacement_vector.x = d(0);
        response->displacement_vector.y = d(1);
        response->displacement_vector.z = d(2);

        // Return the rotation matrix to the Client
        response->rotation_matrix[0].x = R(0, 0);
        response->rotation_matrix[0].y = R(0, 1);
        response->rotation_matrix[0].z = R(0, 2);

        response->rotation_matrix[1].x = R(1, 0);
        response->rotation_matrix[1].y = R(1, 1);
        response->rotation_matrix[1].z = R(1, 2);

        response->rotation_matrix[2].x = R(2, 0);
        response->rotation_matrix[2].y = R(2, 1);
        response->rotation_matrix[2].z = R(2, 2);

        
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Finished request for least squares");
    }

    rclcpp::Service<linear_algebra_resources::srv::LeastSquares>::SharedPtr service_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    std::mt19937 generator_;

    std::uniform_real_distribution<double> angle_distribution_;

    std::uniform_real_distribution<double> unbounded_distribution_;

    std::uniform_real_distribution<double> bounded_distribution_;

    // All variables that refer to the separate thread are grouped 
    // together within this data structure for semantic purposes.
    struct worker_resources_t {

        std::unique_ptr<std::thread> thread_ptr;

        std::atomic_bool is_thread_running;

        std::atomic_bool has_received_message;

        std::atomic_bool logged_output;

        std::condition_variable condition;

        std::shared_ptr<std_msgs::msg::String> received_msg_ptr;

        std::mutex mutex;

    } worker_resources_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearAlgebraServer>());
    rclcpp::shutdown();
    return 0;
}
