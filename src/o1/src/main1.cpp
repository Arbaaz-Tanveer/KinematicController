#include <iostream>
#include <cmath>
#include <vector>

#include "Coordinate.hpp"
#include "PathPlanner.hpp"
#include "Visualize.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

double min(double a, double b) { return a < b ? a : b; }
double max(double a, double b) { return a > b ? a : b; }

double set_angle(double theta)
{
    while (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }
    while (theta < -M_PI)
    {
        theta += 2 * M_PI;
    }
    return theta;
}

Visualize *window = new Visualize(800);
double runTime = 0.01;
double A = 22, B = 14;
optimalPlanner plannerType = PLANNER_RRTSTAR;
planningObjective objectiveType = OBJECTIVE_PATHLENGTH;

long long int iteration_count = 0;
int flag = 0;
double obs_size = 0.7;
double max_angle = 0.5;
double max_acceleration = 1;
double max_speed = 3;

std::vector<Point2D> obs, targetPos; // Changed to dynamic size
Point2D nowPos(0, 0, 0), finalPos(0, 0, 0), ballPos(0, 0, 0);
std::vector<pair<double, double>> points;

int findclosestpoint(std::vector<Point2D> &targetPos, Point2D &nowPos)
{
   =20
    double min_dist = 1000000;
    int idx = 0;
    for (int i = 0; i < targetPos.size(); i++)
    {
        if (sqrtl((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                  (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y)) <
            min_dist)
        {
            min_dist =
                sqrt((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                     (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y));
            idx = i;
        }
    }
    return idx;
}

bool check_path(const Point2D &path_point, const Point2D &obs_point)
{
    double dist = sqrt((obs_point.x - path_point.x) * (obs_point.x - path_point.x) + (obs_point.y - path_point.y) * (obs_point.y - path_point.y));
    return (dist > obs_size);
}

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener_node")
    {
        o1_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("o1_data", 10, std::bind(&ListenerNode::o1_callback, this, std::placeholders::_1));
        obstacles_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("o1_obstacles", 10, std::bind(&ListenerNode::obstacles_callback, this, std::placeholders::_1));
        ball_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("ball_data", 10, std::bind(&ListenerNode::ball_callback, this, std::placeholders::_1));
        decision_target_subscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>("/o1/decision_target_data", 10, std::bind(&ListenerNode::decision_target_callback, this, std::placeholders::_1));
        target_array_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/o1/target_pos", 10);
    }

    void o1_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        nowPos = Point2D(msg->data[0], msg->data[1], msg->data[2]);
        my_pose_received = true;
        listening();
    }

    void obstacles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 1)
        {
            RCLCPP_WARN(this->get_logger(), "Received empty obstacles message");
            return;
        }
        int num_obstacles = static_cast<int>(msg->data[0]);
        if (msg->data.size() != static_cast<size_t>(1 + 2 * num_obstacles))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid obstacles message size");
            return;
        }
        obs.clear();
        for (int i = 0; i < num_obstacles; ++i)
        {
            double x = msg->data[1 + 2 * i];
            double y = msg->data[1 + 2 * i + 1];
            obs.emplace_back(x, y, 0); // Theta set to 0 as it's not provided
        }
        obstacles_received = true;
    }

    void ball_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        ballPos = Point2D(msg->data[0], msg->data[1], 0);
        ball_pose_received = true;
    }

    void decision_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        cout << "New decision target data received" << endl;
        finalPos = Point2D(msg->data[0], msg->data[1], msg->data[2]);
        target_pose_received = true;
    }

    void listening()
    {
        if (!my_pose_received || !obstacles_received || !ball_pose_received || !target_pose_received)
            return;

        for (const auto &obstacle : obs)
        {
            double dist_from_target = sqrt((obstacle.x - finalPos.x) * (obstacle.x - finalPos.x) + (obstacle.y - finalPos.y) * (obstacle.y - finalPos.y));
            if (dist_from_target < obs_size)
            {
                targetPos.clear();
                targetPos.push_back(nowPos);
                iteration_count++;
                window->visualizeGame(targetPos, nowPos, findclosestpoint(targetPos, nowPos), nowPos.theta, obs, ballPos);
                publish_next_point(targetPos, nowPos);
                return;
            }
        }

        for (int i = 0; i < targetPos.size() && flag == 0; i++)
        {
            for (auto curr_obs : obs)
            {
                if (!check_path(targetPos[i], curr_obs))
                {
                    flag = 1;
                    break;
                }
            }
        }

        if (flag || iteration_count == 0 || (targetPos.size() > 0 && (finalPos.x != targetPos.back().x || finalPos.y != targetPos.back().y)))
        {
            try
            {
                targetPos.clear();
                targetPos = plan(runTime, A, B, obs, plannerType, objectiveType, nowPos, finalPos, ballPos);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                std::cout << "Could not find path" << std::endl;
                targetPos.clear();
                targetPos.push_back(nowPos);
            }
            flag = 0;
        }

        if (sqrt((targetPos.back().x - finalPos.x) * (targetPos.back().x - finalPos.x) + (targetPos.back().y - finalPos.y) * (targetPos.back().y - finalPos.y)) > 0.1)
        {
            targetPos.clear();
            targetPos.push_back(nowPos);
        }

        iteration_count++;
        window->visualizeGame(targetPos, nowPos, findclosestpoint(targetPos, nowPos), nowPos.theta, obs, ballPos);
        publish_next_point(targetPos, nowPos);
    }

    void publish_next_point(std::vector<Point2D> &path, Point2D &nowPos)
    {
        if (!path.empty())
        {
            int idx = findclosestpoint(path, nowPos);
            auto prev_point = path[min(idx, path.size() - 2)];
            auto next_point = path[min(idx + 1, path.size() - 1)];
            auto next_to_next_point = path[min(idx + 2, path.size() - 1)];

            double x0 = prev_point.x;
            double y0 = prev_point.y;
            double x1 = next_point.x;
            double y1 = next_point.y;
            double x2 = next_to_next_point.x;
            double y2 = next_to_next_point.y;

            double angle = atan2((y2 - y1), (x2 - x1));

            if (idx + 2 == path.size() - 1 && idx + 1 != path.size() - 1)
            {
                double angle = atan2((y1 - path[idx].y), (x1 - path[idx].x));
            }

            double dist = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
            double speed = 0;
            idx++;
            if (idx > 0 && idx < path.size())
            {
                Point2D p_prev = path[idx - 1];
                Point2D p_curr = path[idx];
                Point2D p_next = path[idx + 1];

                double d1 = sqrt((p_curr.x - p_prev.x) * (p_curr.x - p_prev.x) + (p_curr.y - p_prev.y) * (p_curr.y - p_prev.y));
                double d2 = sqrt((p_next.x - p_curr.x) * (p_next.x - p_curr.x) + (p_next.y - p_curr.y) * (p_next.y - p_curr.y));

                double angle1 = atan2(p_curr.y - p_prev.y, p_curr.x - p_prev.x);
                double angle2 = atan2(p_next.y - p_curr.y, p_next.x - p_curr.x);

                double dtheta = fabs(set_angle(angle2 - angle1));

                if (dtheta > 1e-6)
                {
                    double radius = (d1 + d2) / dtheta;
                    speed = sqrt(max_acceleration * radius);
                    speed = min(max_speed, speed);
                }
                else
                {
                    speed = max_speed;
                }
            }
            double vx = speed * cos(angle);
            double vy = speed * sin(angle);

            double bot_global_angle = set_angle(nowPos.theta);
            double ball_wrto_bot = set_angle(atan2((ballPos.y - nowPos.y), (ballPos.x - nowPos.x)));
            double rel_angle = set_angle(ball_wrto_bot - bot_global_angle);

            rel_angle = min(rel_angle, max_angle);
            rel_angle = max(rel_angle, -max_angle);

            double theta = finalPos.theta;
            double omega = 0;

            cout << "Next Point: " << x1 << " " << y1 << " " << theta << endl;
            cout << "Next velocity: " << vx << " " << vy << " " << omega << " " << speed << endl;

            std_msgs::msg::Float32MultiArray message;
            message.data = {x1, y1, theta, vx, vy, omega, speed, static_cast<float>(idx), static_cast<float>(path.size()), x0, y0};
            target_array_publisher->publish(message);
        }
    }

private:
    bool my_pose_received{false}, obstacles_received{false}, ball_pose_received{false}, target_pose_received{false};

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr o1_subscriber, obstacles_subscriber, ball_subscriber, decision_target_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_array_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}