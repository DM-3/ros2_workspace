#include "follower/follower.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Follower>());
    rclcpp::shutdown();
    return 0;
}
