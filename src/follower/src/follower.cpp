#include "follower/follower.hpp"

using namespace std::chrono_literals;


Follower::Follower() : 
    Node("Object_Follower")
{
    RCLCPP_INFO(this->get_logger(), "Constructor of \"%s\"", this->get_name());

    init_params();

    // create timer
    _timer = this->create_wall_timer(100ms, 
        std::bind(&Follower::timer_callback, this));
    
    // Quality of Service
    auto qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
        qos_profile.history, 1), qos_profile);

    // subscription to laserscanner
    _scannerSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&Follower::scan_callback, 
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
    this->declare_parameter("minDistance", 0.2);
    this->declare_parameter("linearSpeed", 0.1);
    this->declare_parameter("angularSpeed", 0.01);
    this->declare_parameter("openingAngle", 40.0);
}

void Follower::get_params()
{
    _minDistance  = this->get_parameter("minDistance").as_double();
    _linearSpeed  = this->get_parameter("linearSpeed").as_double();
    _angularSpeed = this->get_parameter("angularSpeed").as_double();
    _openingAngle = this->get_parameter("openingAngle").as_double();
}

void Follower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
    std::vector<float> ranges = _msg.get()->ranges;

    // filter ranges out of laserscanner bounds
    for (auto& r : ranges)
        if(r == 0.0f) r = std::numeric_limits<float>::max();

    // find closest range
    auto  minElem = std::min_element(ranges.begin(), ranges.end());
    _targetAngle = minElem - ranges.begin();
    _targetAngle = _targetAngle - (_targetAngle > 180 ? 360 : 0);    // convert angle to range:      left [180, -180) right
    _targetDist  = *minElem;
}

void Follower::timer_callback()
{
    get_params();

    auto msg = geometry_msgs::msg::Twist();

    msg.angular.z = _targetAngle * _angularSpeed;
    msg.linear.x = abs(_targetAngle) < _openingAngle / 2 ? 
        sqrtf(_targetDist - _minDistance) * _linearSpeed : 0.0;

    _movePub->publish(msg);
}
