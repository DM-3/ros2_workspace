#include "camera_line/follower.hpp"

using namespace std::chrono_literals;

Follower::Follower() :
    Node("Line_Follower")
{
    RCLCPP_INFO(this->get_logger(), "Constructor of \"%s\"", this->get_name());

    init_params();

    // create_timer
    _timer = this->create_wall_timer(100ms, 
        std::bind(&Follower::timer_callback, this));

    // Quality of Service
    auto qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
        qos_profile.history, 1), qos_profile);

    // subscription to camera
    _cameraSub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", qos, std::bind(&Follower::image_callback, 
        this, std::placeholders::_1));
    
    // publisher for movement
    _movePub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

Follower::~Follower()
{
    RCLCPP_INFO(this->get_logger(), "Destructor of \"%s\"", this->get_name());

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    _movePub->publish(msg);
}

void Follower::init_params()
{
    this->declare_parameter("linearSpeed", 0.1);
    this->declare_parameter("angularSpeed", 0.01);
}

void Follower::get_params()
{
    _linearSpeed  = this->get_parameter("linearSpeed").as_double();
    _angularSpeed = this->get_parameter("angularSpeed").as_double();
}

void Follower::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr _msg)
{
    RCLCPP_INFO(this->get_logger(), "Received image %li", uint64_t(_msg.get()));
}

void Follower::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback");
}