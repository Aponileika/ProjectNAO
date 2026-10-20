#include "cascar_interfaces/msg/actuator_command.hpp"
#include "cascar_interfaces/srv/reset_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

inline double map_normalized_control(double value, double maximum)
{
  return std::clamp(value, -1.0, 1.0) * maximum;
}

class CascarSimulator : public rclcpp::Node {
public:
  CascarSimulator() : Node("cascar_simulator") {
    this->declare_parameter<std::string>("agent_name", "");
    this->declare_parameter<double>("wheelbase_m", 0.275);
    this->declare_parameter<double>("max_speed_mps", 1.0);
    this->declare_parameter<double>("max_acceleration_mps2", 1.0);
    this->declare_parameter<double>("max_steering_angle_rad", 0.35);
    this->declare_parameter<double>("command_timeout_sec", 0.25);
    this->declare_parameter<double>("odom_position_variance", 0.01);
    this->declare_parameter<double>("odom_yaw_variance", 0.01);
    this->declare_parameter<double>("odom_linear_velocity_variance", 0.01);
    this->declare_parameter<double>("odom_yaw_rate_variance", 0.01);

    agent_name_ = this->get_parameter("agent_name").as_string();
    wheelbase_ = positiveParameter("wheelbase_m");
    max_speed_ = positiveParameter("max_speed_mps");
    max_acceleration_ = positiveParameter("max_acceleration_mps2");
    max_steering_angle_ = positiveParameter("max_steering_angle_rad");
    command_timeout_sec_ = positiveParameter("command_timeout_sec");
    odom_position_variance_ = nonnegativeParameter("odom_position_variance");
    odom_yaw_variance_ = nonnegativeParameter("odom_yaw_variance");
    odom_linear_velocity_variance_ =
      nonnegativeParameter("odom_linear_velocity_variance");
    odom_yaw_rate_variance_ = nonnegativeParameter("odom_yaw_rate_variance");

    // Prepend agent_name to frame names if specified
    std::string namespace_ = agent_name_.empty() ? "" : agent_name_ + "/";

    // Define frame names dynamically
    odom_link_frame_ = namespace_ + "odom";
    base_link_frame_ = namespace_ + "base_link";
    front_left_joint_ = namespace_ + "base_to_front_left_wheel";
    front_right_joint_ = namespace_ + "base_to_front_right_wheel";

    // Publishers
    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    // Subscribers
    control_subscription_ =
        this->create_subscription<cascar_interfaces::msg::ActuatorCommand>(
            "actuator_command", 1,
            std::bind(&CascarSimulator::controlCallback, this, _1));

    // Services
    server_ = this->create_service<cascar_interfaces::srv::ResetPose>(
        "reset_pose", std::bind(&CascarSimulator::srvCallback, this, _1, _2));

    // Timer for state updates
    timer_ = this->create_wall_timer(
        10ms, std::bind(&CascarSimulator::timerCallback, this));
  }

private:
  std::string agent_name_;
  std::string odom_link_frame_, base_link_frame_;
  std::string front_left_joint_, front_right_joint_;
  double wheelbase_;
  double max_speed_;
  double max_acceleration_;
  double max_steering_angle_;
  double command_timeout_sec_;
  double odom_position_variance_;
  double odom_yaw_variance_;
  double odom_linear_velocity_variance_;
  double odom_yaw_rate_variance_;
  std::optional<std::chrono::steady_clock::time_point> last_command_time_;
  bool watchdog_applied_ = true;

  // State variables
  double x_ = 2.5, y_ = 2.0, v_ = 0.0, yaw_ = 0.0;
  double acceleration_ = 0.0, steering_angle_ = 0.0, update_frequency_hz_ = 100;

  double positiveParameter(const std::string & name)
  {
    const double value = this->get_parameter(name).as_double();
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and greater than zero");
    }
    return value;
  }

  double nonnegativeParameter(const std::string & name)
  {
    const double value = this->get_parameter(name).as_double();
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and non-negative");
    }
    return value;
  }

  // Process model (Rear-axle reference)
  void processModelRear() {
    double dt = 1 / update_frequency_hz_;
    x_ += v_ * cos(yaw_) * dt;
    y_ += v_ * sin(yaw_) * dt;
    yaw_ += v_ * tan(steering_angle_) * dt / wheelbase_;

    if (acceleration_ == 0.0) {
      // Neutral throttle brakes to rest without overshooting into reverse.
      const double speed = std::max(0.0, std::abs(v_) - max_acceleration_ * dt);
      v_ = std::copysign(speed, v_);
    } else {
      v_ += acceleration_ * dt;
    }
    v_ = std::clamp(v_, -max_speed_, max_speed_);
    steering_angle_ = std::clamp(steering_angle_, -max_steering_angle_, max_steering_angle_);
  }

  // Update acceleration and steering together from one command source.
  void controlCallback(const cascar_interfaces::msg::ActuatorCommand::SharedPtr msg)
  {
    if (!std::isfinite(msg->throttle) || !std::isfinite(msg->steering)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring actuator command containing a non-finite value");
      return;
    }
    acceleration_ = map_normalized_control(msg->throttle, max_acceleration_);
    steering_angle_ = map_normalized_control(msg->steering, max_steering_angle_);
    last_command_time_ = std::chrono::steady_clock::now();
    watchdog_applied_ = false;
  }

  // Reset service
  void srvCallback(const cascar_interfaces::srv::ResetPose::Request::SharedPtr req,
                   cascar_interfaces::srv::ResetPose::Response::SharedPtr res) {
    if (!std::isfinite(req->pose.x) || !std::isfinite(req->pose.y) ||
      !std::isfinite(req->pose.theta))
    {
      RCLCPP_WARN(this->get_logger(), "Rejected non-finite reset pose");
      res->success = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Received request to reset state.");
    v_ = 0;
    acceleration_ = 0;
    steering_angle_ = 0;
    x_ = req->pose.x;
    y_ = req->pose.y;
    yaw_ = req->pose.theta;
    res->success = true;
  }

  // Timer callback for processing
  void timerCallback() {
    if (!watchdog_applied_ && last_command_time_.has_value() &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *last_command_time_).count() >
      command_timeout_sec_)
    {
      acceleration_ = 0.0;
      steering_angle_ = 0.0;
      watchdog_applied_ = true;
      RCLCPP_WARN(
        this->get_logger(), "Actuator command timed out; braking to a stop");
    }
    processModelRear();
    broadcastOdometry();
    broadcastJointStates();
  }

  void broadcastJointStates() {
    auto now = this->get_clock()->now();
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now;
    joint_state.name = {front_left_joint_, front_right_joint_};
    joint_state.position = {steering_angle_, steering_angle_};
    joint_state_publisher_->publish(joint_state);
  }

  // Publish Odometry
  void broadcastOdometry() {
    auto now = this->get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = this->odom_link_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.covariance[0] = odom_position_variance_;
    odom.pose.covariance[7] = odom_position_variance_;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = odom_yaw_variance_;
    odom.child_frame_id = this->base_link_frame_;
    // Odometry twist is expressed in child_frame_id (base_link).
    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = v_ * tan(steering_angle_) / wheelbase_;
    odom.twist.covariance[0] = odom_linear_velocity_variance_;
    odom.twist.covariance[7] = 1e6;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = odom_yaw_rate_variance_;
    odom_publisher_->publish(odom);
  }

  // Class attributes
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<cascar_interfaces::msg::ActuatorCommand>::SharedPtr
    control_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Service<cascar_interfaces::srv::ResetPose>::SharedPtr server_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CascarSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
