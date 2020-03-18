#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/int32_tracked.hpp"

#include "rosslt/trackingnode.h"

using namespace std::chrono_literals;

class PublisherNode : public TrackingNode {
public:
  PublisherNode()
      : TrackingNode("slt_talker"), count_ {loc(5)}
  {
    publisher_ = create_publisher<rosslt_msgs::msg::Int32Tracked>("foo", 10);

    auto timer_callback =
      [this]() -> void {
        auto message = Tracked<std_msgs::msg::Int32>();

        SET_FIELD(message, data, 2*reevaluate(count_));

        RCLCPP_INFO(get_logger(), "Publishing: '%i'", message.get_data().data);
        publisher_->publish(static_cast<rosslt_msgs::msg::Int32Tracked>(message));
      };
    timer_ = create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rosslt_msgs::msg::Int32Tracked>::SharedPtr publisher_;
  Tracked<int> count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
