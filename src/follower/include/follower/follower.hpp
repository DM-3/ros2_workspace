#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"


class Follower : public rclcpp::Node
{
public:
    Follower();
    ~Follower();

private:
    void init_params();
    void get_params();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scannerSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _movePub;

    // transfer state between callbacks    
    int _targetAngle = 0;
    float _targetDist = 0.0f;

    // parameters
    double _minDistance;
    double _linearSpeed;
    double _angularSpeed;
    double _openingAngle;
};
