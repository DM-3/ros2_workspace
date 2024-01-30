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

    // prepare opencv window
    cv::namedWindow("view", cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();
}

Follower::~Follower()
{
    RCLCPP_INFO(this->get_logger(), "Destructor of \"%s\"", this->get_name());

    // reset motion to zero when destroying Follower
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    _movePub->publish(msg);
}

void Follower::init_params()
{
    this->declare_parameter("linearSpeed", 0.01);
    this->declare_parameter("angularSpeed", 1.0);
    this->declare_parameter("openingRatio", 0.1);
}

void Follower::get_params()
{
    _linearSpeed  = this->get_parameter("linearSpeed").as_double();
    _angularSpeed = this->get_parameter("angularSpeed").as_double();
    _openingRatio = this->get_parameter("openingRatio").as_double();
}

void Follower::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr _msg)
{
    // convert compressed image to grayscale opencv image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*_msg);
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);

    cv::imshow("view", cv_ptr->image);

    // extract bottom row of pixels
    std::vector<uint8_t> bottomRow(cv_ptr->image.cols);
    for (int i = 0; i < (int)bottomRow.size(); i++)
        bottomRow[i] = cv_ptr->image.at<uint8_t>(cv_ptr->image.rows - 1, i);

    // find brightest pixels among bottom row
    int maxCol = std::max_element(bottomRow.begin(), bottomRow.end()) - bottomRow.begin();
    _rotationDiff = maxCol - bottomRow.size() / 2.0;
    _rotationDiff /= double(cv_ptr->image.cols);

    std::cout << "_rotationDiff: " << _rotationDiff << " maxCol: " << maxCol << std::endl;
}

void Follower::timer_callback()
{
    get_params();

    auto msg = geometry_msgs::msg::Twist();

    // object holding new motion outputs
    msg.angular.z = _rotationDiff * -_angularSpeed;
    msg.linear.x = abs(_rotationDiff) < _openingRatio ? 
        _linearSpeed / (abs(_rotationDiff) + _linearSpeed) : 0.0;

    _movePub->publish(msg);
}