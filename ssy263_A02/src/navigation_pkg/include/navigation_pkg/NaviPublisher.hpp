#ifndef NAVI_PUBLISHER_HPP // Header guard for "NaviPublisher.hpp"
#define NAVI_PUBLISHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "navi_interfaces/msg/coord2d.hpp"

namespace ssy263_amd
{
class NaviPublisher : public rclcpp::Node
{
  public:
    NaviPublisher();
    ~NaviPublisher();

  private:
    void publish_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<navi_interfaces::msg::Coord2d>::SharedPtr publisher_;
    size_t count_;

};
}

#endif // End of header NAVI_PUBLISHER_HPP
