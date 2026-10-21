#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cascar_msgs/msg/path.hpp>
#include <cascar_msgs/msg/path_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "guidance/hybridAStar.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <string>
#include <stdexcept>

using Pose = std::array<double, 3>;
using Path = std::vector<std::array<double, 5>>;

class Guidance : public rclcpp::Node
{
public:
    Guidance() : Node("guidance")
    {
        RCLCPP_INFO(this->get_logger(), "Guidance Node Started");

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        path_publisher = this->create_publisher<cascar_msgs::msg::Path>("planned_path", 1);

        pos_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "qualisys_pose", 10,
            std::bind(&Guidance::pos_callback, this, std::placeholders::_1)
        );

        goal_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "goal", 10,
            std::bind(&Guidance::goal_callback, this, std::placeholders::_1)
        );

        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&Guidance::map_callback, this, std::placeholders::_1)
        );

        path_planner = std::make_unique<HybridAStar>();

        Ts = 1.0;
        timer = this->create_wall_timer(
            std::chrono::duration<double>(Ts),
            std::bind(&Guidance::main_loop, this)
        );

        x_goal = 2.0;
        y_goal = 2.0;
        theta_goal = -M_PI / 2.0;
    }

private:
    void pos_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
        pose_received = true;

        if (map_received && goal_received && !path_planned)
        {
            plan_path();
        }
    }

    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        x_goal = msg->x;
        y_goal = msg->y;
        theta_goal = msg->theta;
        goal_received = true;
        plan_path();
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->info.width == 0 || msg->info.height == 0 ||
            !std::isfinite(msg->info.resolution) || msg->info.resolution <= 0.0 ||
            msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid occupancy grid");
            return;
        }

        occupancy_map.assign(msg->info.height, std::vector<double>(msg->info.width));
        for (std::size_t i = 0; i < msg->data.size(); ++i)
        {
            const int8_t cell = msg->data[i];
            occupancy_map[i / msg->info.width][i % msg->info.width] =
                (cell >= occupied_threshold) ? 1.0 : 0.0;
        }

        map_resolution = msg->info.resolution;
        map_origin_x = msg->info.origin.position.x;
        map_origin_y = msg->info.origin.position.y;
        map_origin_yaw = quaternion_yaw(msg->info.origin.orientation);
        map_frame = msg->header.frame_id;
        map_received = true;

        if (goal_received && pose_received && !path_planned)
        {
            plan_path();
        }
    }

    void publish_path()
    {
        cascar_msgs::msg::Path msg;
        msg.points.reserve(path.size());

        for (const auto &point : path)
        {
            cascar_msgs::msg::PathPoint path_point;
            path_point.x = point[0];
            path_point.y = point[1];
            path_point.theta = point[2];
            path_point.dir = point[3];
            path_point.distance = point[4];
            msg.points.push_back(path_point);
        }

        path_publisher->publish(msg);
    }

    void plan_path()
    {
        if (!map_received || !pose_received || !goal_received)
        {
            return;
        }

        std::cout << "Planning path" << std::endl;

        geometry_msgs::msg::TransformStamped map_from_odom;
        try
        {
            map_from_odom = tf_buffer->lookupTransform(
                map_frame, "odom", rclcpp::Time(0));
        }
        catch (const std::runtime_error & ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Cannot plan until TF %s <- odom is available: %s",
                map_frame.c_str(), ex.what());
            return;
        }

        const double tf_yaw = quaternion_yaw(map_from_odom.transform.rotation);
        const double tf_cos = std::cos(tf_yaw);
        const double tf_sin = std::sin(tf_yaw);
        const double map_x = map_from_odom.transform.translation.x;
        const double map_y = map_from_odom.transform.translation.y;

        auto odom_to_grid = [this, tf_cos, tf_sin, tf_yaw, map_x, map_y](const Pose & pose) {
            const double x_map = map_x + tf_cos * pose[0] - tf_sin * pose[1];
            const double y_map = map_y + tf_sin * pose[0] + tf_cos * pose[1];
            const double dx = x_map - map_origin_x;
            const double dy = y_map - map_origin_y;
            const double origin_cos = std::cos(map_origin_yaw);
            const double origin_sin = std::sin(map_origin_yaw);
            return Pose{
                origin_cos * dx + origin_sin * dy,
                -origin_sin * dx + origin_cos * dy,
                wrap_angle(pose[2] + tf_yaw - map_origin_yaw)
            };
        };

        Pose start = odom_to_grid({x, y, theta});
        // Goals use the planner's map-local frame, whose origin is the
        // occupancy grid's lower-left corner.
        Pose goal = {x_goal, y_goal, theta_goal};
        std::array<double, 2> xlim = {
            0.0, occupancy_map.front().size() * map_resolution};
        std::array<double, 2> ylim = {
            0.0, occupancy_map.size() * map_resolution};

        path = path_planner->search(
            start, goal, occupancy_map, xlim, ylim, map_resolution);

        const double origin_cos = std::cos(map_origin_yaw);
        const double origin_sin = std::sin(map_origin_yaw);
        for (auto & point : path)
        {
            const double x_map =
                map_origin_x + origin_cos * point[0] - origin_sin * point[1];
            const double y_map =
                map_origin_y + origin_sin * point[0] + origin_cos * point[1];
            const double dx = x_map - map_x;
            const double dy = y_map - map_y;

            point[0] = tf_cos * dx + tf_sin * dy;
            point[1] = -tf_sin * dx + tf_cos * dy;
            point[2] = wrap_angle(point[2] + map_origin_yaw - tf_yaw);
        }

        path_planned = !path.empty();
        RCLCPP_INFO(this->get_logger(), "PATH PLANNED: %zu points", path.size());
        publish_path();
    }

    static double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static double wrap_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void main_loop()
    {
        publish_path();
    }

    rclcpp::Publisher<cascar_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<HybridAStar> path_planner;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    double Ts;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double x_goal = 2.0;
    double y_goal = 2.0;
    double theta_goal = -M_PI / 2.0;

    Path path = {};
    std::vector<std::vector<double>> occupancy_map;
    double map_resolution = 0.05;
    double map_origin_x = 0.0;
    double map_origin_y = 0.0;
    double map_origin_yaw = 0.0;
    std::string map_frame;
    bool map_received = false;
    bool pose_received = false;
    bool goal_received = false;
    bool path_planned = false;
    static constexpr int8_t occupied_threshold = 50;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Guidance>());
    rclcpp::shutdown();
    return 0;
}