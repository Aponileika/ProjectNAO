#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class CarTracker : public rclcpp::Node {
 public:
  CarTracker() : Node("car_tracker"), count_(0) {
    // Declare the parameter before using it
    this->declare_parameter<std::string>("agent_name", "");

    // Get the parameter from launch file or CLI
    this->get_parameter("agent_name", agent_name_);

    // Prepend agent_name to frame names if specified
    std::string namespace_ = agent_name_.empty() ? "" : agent_name_ + "/";

    tracker_frame_ = namespace_ + "car_tracker";

    // Initialize class attributes
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("car_tracker", 1);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&CarTracker::odomCallback, this, _1));
    click_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 1,
            std::bind(&CarTracker::clickedPointCallback, this, _1));
    server_ = this->create_service<std_srvs::srv::Empty>(
        "reset", std::bind(&CarTracker::srvCallback, this, _1, _2));
    timer_ = this->create_wall_timer(
        40ms, std::bind(&CarTracker::timerCallback, this));
    static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transform
    this->staticTransform();
  }

 private:
  // State variables
  std::string agent_name_;
  std::string tracker_frame_;

  nav_msgs::msg::Path trackedPath_;

  void generateMsg() {
    if (this->trackedPath_.poses.size() > 500) {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Its time to reset data");
      this->trackedPath_.poses.erase(this->trackedPath_.poses.begin(),
                                     this->trackedPath_.poses.begin() + 100);
    }

    auto now = this->get_clock()->now();
    this->trackedPath_.header.stamp = now;
    this->trackedPath_.header.frame_id = this->tracker_frame_;
    publisher_->publish(this->trackedPath_);
  }

  void staticTransform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = this->tracker_frame_;

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    static_broadcaster_->sendTransform(t);
  }

  // Built in timer to update and publish tracked path
  void timerCallback() { generateMsg(); }

  void clickedPointCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr _) {
    this->trackedPath_.poses.clear();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped posPlaceholder;
    posPlaceholder.pose.position.x = msg->pose.pose.position.x;
    posPlaceholder.pose.position.y = msg->pose.pose.position.y;
    this->trackedPath_.poses.push_back(posPlaceholder);
  }

  void srvCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr) {
    this->trackedPath_.poses.clear();
  }

  // Declare class attributes
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      click_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}