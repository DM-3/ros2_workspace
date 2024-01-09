#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class Follower : public rclcpp::Node
{
public:
    Follower() :
        Node("Object_Follower")
    {
        timer_ = this->create_wall_timer(500ms,
            std::bind(&Follower::timer_callback, this));
    }

private:
    void timer_callback()   
    {
        RCLCPP_INFO(this->get_logger(), "Hello Follower");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Follower>());
    rclcpp::shutdown();
    return 0;
}
