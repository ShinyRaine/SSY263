#include <navigation_pkg/NaviPublisher.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ssy263_amd::NaviPublisher>()); 
  rclcpp::shutdown();
  return 0;
}
