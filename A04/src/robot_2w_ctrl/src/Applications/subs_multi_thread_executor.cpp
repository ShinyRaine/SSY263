#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <string>

#include <std_msgs/msg/string.hpp>

// Replace with your message type (e.g., std_msgs::msg::String)
using MyMsgTypeShPt = std_msgs::msg::String::SharedPtr;
using SubMyMsgTypeShPt = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;

class MultithreadedSubscriberNode : public rclcpp::Node
{
public:
  MultithreadedSubscriberNode() : Node("multithreaded_subscriber_node")
  {
    // Create separate executors for each subscriber
    RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);
    executor1_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);
    executor2_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);

    // Create the first subscriber with a callback function
    // auto callback1 = std::bind(&MultithreadedSubscriberNode::subscriber1Callback, this, std::placeholders::_1);
    subscription1 = create_subscription<std_msgs::msg::String>(
        "/topic1", 10, std::bind(&MultithreadedSubscriberNode::subscriber1Callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);
    executor1_->add_node(shared_from_this());

    // Create the second subscriber with a callback function
    subscription2 = create_subscription<std_msgs::msg::String>(
        "/topic2", 10, std::bind(&MultithreadedSubscriberNode::subscriber2Callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(get_logger(), __FILE__ << ":" << __LINE__);
    executor2_->add_node(shared_from_this());
  }

private:
  void subscriber1Callback(const MyMsgTypeShPt msg)
  {
    // Process message received on topic1 (replace with your logic)
    RCLCPP_INFO(this->get_logger(), "Received message on topic1: %s", msg->data.c_str());
  }

  void subscriber2Callback(const MyMsgTypeShPt msg)
  {
    // Process message received on topic2 (replace with your logic)
    RCLCPP_INFO(this->get_logger(), "Received message on topic2: %s", msg->data.c_str());
  }

private:
  SubMyMsgTypeShPt subscription1;
  SubMyMsgTypeShPt subscription2;

public:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor1_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor2_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), __FILE__ << ":" << __LINE__);
  auto node = std::make_shared<MultithreadedSubscriberNode>();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), __FILE__ << ":" << __LINE__);
  // Create separate threads for each executor
  std::thread thread1([&] { node->executor1_.get()->spin(); });
  RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), __FILE__ << ":" << __LINE__);
  std::thread thread2([&] { node->executor2_.get()->spin(); });

  // Wait for threads to finish (optional)
  // thread1.join();
  // thread2.join();

  rclcpp::shutdown();
  return 0;
}