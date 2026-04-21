// ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.htm
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "navi_interfaces/msg/coord2d.hpp"
#include <navigation_pkg/NaviSubscriber.hpp>

using std::placeholders::_1;
using namespace ssy263_amd;

NaviSubscriber::NaviSubscriber(): Node("navi_subscriber")
{
  subscription_ = this->create_subscription<navi_interfaces::msg::Coord2d>(
    "navi_topic", 10, std::bind(&NaviSubscriber::topic_callback, this, _1));
}

NaviSubscriber::~NaviSubscriber()
{
}

void NaviSubscriber::topic_callback(const navi_interfaces::msg::Coord2d & message) const
{
  RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << message.x << "," << message.y << "'");
}
