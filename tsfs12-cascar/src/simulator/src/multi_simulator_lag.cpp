#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <math.h>
#include <memory>
#include <vector>
// #include "sensor_msgs/msg/joint_state.hpp"
#include "cascar_msgs/msg/control_request.hpp"
#include "cascar_msgs/msg/steering_angle.hpp"
#include "cascar_msgs/srv/set_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class CascarSimulator : public rclcpp::Node
{
  public:
    CascarSimulator() : Node("cascar_simulator"), count_(0)
    {
        // Declare the parameter before using it
        this->declare_parameter<std::string>("agent_name", "");

        // Get the parameter from launch file or CLI
        this->get_parameter("agent_name", agent_name_);

        // Prepend agent_name to frame names if specified
        std::string namespace_ = agent_name_.empty() ? "" : agent_name_ + "/";

        // Define frame names dynamically
        odom_link_frame_ = namespace_ + "odom";
        base_link_frame_ = namespace_ + "base_link";

        // Publishers
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        steer_publisher_ =
            this->create_publisher<cascar_msgs::msg::SteeringAngle>("steering_angle", 1);

        // Subscribers
        ctrl_subscription_ = this->create_subscription<cascar_msgs::msg::ControlRequest>(
            "control", 1, std::bind(&CascarSimulator::controlCallback, this, _1));
        lat_ctrl_subscription_ = this->create_subscription<cascar_msgs::msg::ControlRequest>(
            "lat_control", 1, std::bind(&CascarSimulator::latControlCallback, this, _1));
        lon_ctrl_subscription_ = this->create_subscription<cascar_msgs::msg::ControlRequest>(
            "lon_control", 1, std::bind(&CascarSimulator::lonControlCallback, this, _1));

        // Services
        server_ = this->create_service<cascar_msgs::srv::SetState>(
            "state_reset", std::bind(&CascarSimulator::srvCallback, this, _1, _2));
        client_ = this->create_client<std_srvs::srv::Empty>("reset");

        // Timer for state updates
        timer_ = this->create_wall_timer(10ms, std::bind(&CascarSimulator::timerCallback, this));
    }

  private:
    std::string agent_name_;
    std::string odom_link_frame_, base_link_frame_;

    // State variables
    // double x_ = 0, y_ = 0, v_ = 0, th_ = M_PI_2, L_ = 0.275;
    double x_ = 2.5, y_ = 2.0, v_ = 0.0, th_ = 0.0, L_ = 0.275;
    double a_x = 0.0, d_f = 0.0, d_fd = 0.0, f_ = 100;
    double d_f_cmd = 0.0; // Desired steering angle

    // Process model (Rear-axle reference)
    void processModelRear()
    {
        double dt = 1 / f_;
        double tau = 0.315; // Time constant [s] – tune this value!

        // First-order lag steering dynamics
        double d_f_dot = (d_f_cmd - d_f) / tau;
        d_f += d_f_dot * dt;

        x_ += v_ * cos(th_) * dt;
        y_ += v_ * sin(th_) * dt;
        th_ += v_ * tan(d_f) * dt / L_;

        v_ += a_x * dt;
        v_ = std::clamp(v_, -1.0, 1.0);
        // d_f = std::clamp(d_f, -M_PI/4, M_PI/4);
        d_f = std::clamp(d_f, -M_PI / 6, M_PI / 6);
    }

    // Process model (Front-axle reference)
    void processModelFront()
    {
        double dt = 1 / f_;
        double vf = v_ / cos(d_f);
        x_ += vf * cos(th_ + d_f) * dt;
        y_ += vf * sin(th_ + d_f) * dt;
        th_ += v_ * tan(d_f) * dt / L_;

        v_ += a_x * dt;
        v_ = std::clamp(v_, -1.0, 1.0);
        // d_f = std::clamp(d_f, -M_PI/4, M_PI/4);
        d_f = std::clamp(d_f, -M_PI / 6, M_PI / 6);
    }

    // Callback for control messages
    void controlCallback(const cascar_msgs::msg::ControlRequest::SharedPtr msg)
    {
        this->a_x = msg->ax;
        this->d_f_cmd = msg->df;
    }
    void latControlCallback(const cascar_msgs::msg::ControlRequest::SharedPtr msg)
    {
        this->d_f = msg->df;
    }
    void lonControlCallback(const cascar_msgs::msg::ControlRequest::SharedPtr msg)
    {
        this->a_x = msg->ax;
    }

    // Reset service
    void srvCallback(const cascar_msgs::srv::SetState::Request::SharedPtr req,
                     cascar_msgs::srv::SetState::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to reset state.");
        v_ = 0;
        a_x = 0;
        d_f = 0;
        d_fd = 0;
        x_ = req->data[0];
        y_ = req->data[1];
        th_ = req->data[2];
        res->success = true;
    }

    // Timer callback for processing
    void timerCallback()
    {
        processModelRear();
        broadcastOdometry();
        a_x = 0;
    }

    // Publish Steering Angle
    void broadcastSteeringAngle()
    {
        auto now = this->get_clock()->now();
        cascar_msgs::msg::SteeringAngle steer;
        steer.header.stamp = now;
        steer.header.frame_id = this->base_link_frame_;
        steer.df = d_f;
        steer_publisher_->publish(steer);
    }

    // Publish Odometry
    void broadcastOdometry()
    {
        auto now = this->get_clock()->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = this->odom_link_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.child_frame_id = this->base_link_frame_;
        odom.twist.twist.linear.x = v_ * cos(th_);
        odom.twist.twist.linear.y = v_ * sin(th_);
        odom.twist.twist.angular.z = v_ * tan(d_f) / L_;
        odom_publisher_->publish(odom);
    }

    // Class attributes
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<cascar_msgs::msg::ControlRequest>::SharedPtr ctrl_subscription_;
    rclcpp::Subscription<cascar_msgs::msg::ControlRequest>::SharedPtr lat_ctrl_subscription_;
    rclcpp::Subscription<cascar_msgs::msg::ControlRequest>::SharedPtr lon_ctrl_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<cascar_msgs::msg::SteeringAngle>::SharedPtr steer_publisher_;
    rclcpp::Service<cascar_msgs::srv::SetState>::SharedPtr server_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CascarSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
