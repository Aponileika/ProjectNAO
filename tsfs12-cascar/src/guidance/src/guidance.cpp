#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cascar_msgs/msg/path.hpp>
#include <cascar_msgs/msg/path_point.hpp>

#include "hybridAStar.hpp"

#include <array>
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <cmath>

using Pose = std::array<double, 3>;
using Path = std::vector<std::array<double, 5>>;

class Guidance : public rclcpp::Node
{
public:
    Guidance() : Node("guidance")
    {
        RCLCPP_INFO(this->get_logger(), "Guidance Node Started");

        path_publisher = this->create_publisher<cascar_msgs::msg::Path>("planned_path", 1);

        pos_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "pose", 10,
            std::bind(&Guidance::pos_callback, this, std::placeholders::_1)
        );

        goal_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "goal", 10,
            std::bind(&Guidance::goal_callback, this, std::placeholders::_1)
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
    }

    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        x_goal = msg->x;
        y_goal = msg->y;
        theta_goal = msg->theta;
        plan_path();
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
        Pose start = {x, y, theta};
        Pose goal = {x_goal, y_goal, theta_goal};

        std::vector<std::vector<double>> map {};
        std::array<double, 2> xlim = {0.0, 0.0};
        std::array<double, 2> ylim = {0.0, 0.0};

        path = path_planner->search(start, goal, map, xlim, ylim);
        RCLCPP_INFO(this->get_logger(), "PATH PLANNED: %zu points", path.size());
        publish_path();
    }

    void main_loop()
    {
        publish_path();
    }

    rclcpp::Publisher<cascar_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<HybridAStar> path_planner;

    double Ts;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double x_goal = 2.0;
    double y_goal = 2.0;
    double theta_goal = -M_PI / 2.0;

    Path path = {};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Guidance>());
    rclcpp::shutdown();
    return 0;
}