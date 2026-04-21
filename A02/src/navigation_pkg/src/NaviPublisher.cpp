// ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.htm
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "navi_interfaces/msg/coord2d.hpp"
#include <navigation_pkg/NaviPublisher.hpp>

using namespace std::chrono_literals;
using namespace ssy263_amd;

NaviPublisher::NaviPublisher(): Node("navi_publisher"), count_(0)
{
  publisher_ = this->create_publisher<navi_interfaces::msg::Coord2d>("navi_topic", 10);
  timer_ = this->create_wall_timer(
  500ms, std::bind(&NaviPublisher::publish_callback, this));
}


void NaviPublisher::publish_callback()
{
  auto message = navi_interfaces::msg::Coord2d();
  message.y = 0;
  message.x = this->count_++;

  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.x << "," << message.y <<"'");
  publisher_->publish(message);
}

NaviPublisher::~NaviPublisher()
{
}
