#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Follower : public rclcpp::Node
{
public:
    Follower();
    ~Follower();

private:
    void init_params();
    void get_params();
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr _msg);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _cameraSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _movePub;

    // transfer state between callbacks
    int _targetAngle = 0;

    // parameters
    double _linearSpeed;
    double _angularSpeed;
};
