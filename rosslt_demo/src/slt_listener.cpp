
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/int32_tracked.hpp"

#include "rosslt/trackingnode.h"

class Subscriber : public TrackingNode {
public:
  Subscriber()
  : TrackingNode("slt_listener")
  {
    subscriber_ = create_subscription<rosslt_msgs::msg::Int32Tracked>("foo", rclcpp::QoS(rclcpp::KeepLast(20)),
    [this](rosslt_msgs::msg::Int32Tracked::UniquePtr msg) {
        Tracked<int> counter {msg->data.data, msg->location};
        RCLCPP_INFO(get_logger(), "Got message: '%i' from %s:%i", counter.get_data(),
                    counter.get_location().source_node.c_str(),
                    counter.get_location().location_id);
        force_value(counter, counter + 2);
    });
    
  }

private:
  rclcpp::Subscription<rosslt_msgs::msg::Int32Tracked>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}

