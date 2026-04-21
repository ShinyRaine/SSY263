#include <navigation_pkg/NaviSubscriber.hpp>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ssy263_amd::NaviSubscriber>());
  rclcpp::shutdown();
  return 0;
}
