#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <functional>
#include <memory>
#include <string>

using std::placeholders::_1;

class OdomTfPublisher : public rclcpp::Node {
public:
  OdomTfPublisher() : Node("odom_tf_publisher") {
    this->declare_parameter<std::string>("agent_name", "");
    const auto agent_name = this->get_parameter("agent_name").as_string();
    const auto prefix = agent_name.empty() ? "" : agent_name + "/";
    odom_frame_ = prefix + "odom";
    base_link_frame_ = prefix + "base_link";

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&OdomTfPublisher::odomCallback, this, _1));

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(map_to_odom);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_link_frame_;
    transform.transform.translation.x = message->pose.pose.position.x;
    transform.transform.translation.y = message->pose.pose.position.y;
    transform.transform.translation.z = message->pose.pose.position.z;
    transform.transform.rotation = message->pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_;
  std::string base_link_frame_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
