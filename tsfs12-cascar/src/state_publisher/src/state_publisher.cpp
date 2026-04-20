#include <chrono>
#include <memory>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "cascar_msgs/msg/control_request.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class StatePublisher : public rclcpp::Node{
public:
    StatePublisher()
            : Node("state_publisher"), count_(0)
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
        b2frw_ = namespace_ + "base_to_front_right_wheel";
        b2flw_ = namespace_ + "base_to_front_left_wheel";

        // Initialize class attributes
        state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
        timer_ = this->create_wall_timer(
                10ms, std::bind(&StatePublisher::timerCallback, this));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 1, std::bind(&StatePublisher::odomCallback, this, _1));
        ctrl_subscription_ = this->create_subscription<cascar_msgs::msg::ControlRequest>(
                "control", 1, std::bind(&StatePublisher::ctrlCallback, this, _1));
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transform
        this->staticTransform();
    }

private:
    // Agent name
    std::string agent_name_;
    // Frame and joint names
    std::string base_link_frame_, odom_link_frame_;
    std::string b2frw_, b2flw_;

    //State variables
    double x_ = 2;
    double y_ = 0;
    double th_ = M_PI_2;
    double d_f = 0;

    // Built in timer to update and publish states
    void timerCallback()
    {
        broadcastTransform();
    }

    void staticTransform(void)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = this->odom_link_frame_;

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

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->x_ = msg->pose.pose.position.x;
        this->y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        this->th_ = yaw;
    }

    void ctrlCallback(const cascar_msgs::msg::ControlRequest::SharedPtr msg)
    {
        this->d_f = msg->df; // + msg->dfd*0.5;
        this->d_f = std::min(M_PI/4,std::max(this->d_f, -M_PI/4));
    }

    // Broadcasting function
    void broadcastTransform(void){
        auto now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = this->odom_link_frame_;
        transformStamped.child_frame_id = this->base_link_frame_;
        transformStamped.transform.translation.x = this->x_;
        transformStamped.transform.translation.y = this->y_;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, this->th_);

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // Joint state
        sensor_msgs::msg::JointState jointState;
        jointState.header.stamp = now;
        std::vector<std::string> names {this->b2frw_, this->b2flw_};
        jointState.name = names;
        std::vector<double> positions = {this->d_f, this->d_f};
        jointState.position = positions;

        //publish the messages
        state_publisher_->publish(jointState);
        broadcaster_->sendTransform(transformStamped);

    }

    // Declare class attributes
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<cascar_msgs::msg::ControlRequest>::SharedPtr ctrl_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    size_t count_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}