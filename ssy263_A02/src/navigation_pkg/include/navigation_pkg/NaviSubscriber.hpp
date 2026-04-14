#ifndef NAVI_SUBSCRIBER_HPP // Header guard for "NaviPublisher.hpp"
#define NAVI_SUBSCRIBER_HPP
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "navi_interfaces/msg/coord2d.hpp"
namespace ssy263_amd
{
class NaviSubscriber : public rclcpp::Node
{
public:
  NaviSubscriber();
  ~NaviSubscriber();

private:
  void topic_callback(const navi_interfaces::msg::Coord2d & message) const;
  rclcpp::Subscription<navi_interfaces::msg::Coord2d>::SharedPtr subscription_;
};
}

#endif
