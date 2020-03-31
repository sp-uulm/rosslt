#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"

#include "rosslt/trackingnode.h"

using namespace std::chrono_literals;

class PublisherNode : public TrackingNode {
public:
  PublisherNode()
      : TrackingNode("vis_talker")
  {
      x = loc(2.0);
      y = loc(1.5);

      publisher_ = create_publisher<rosslt_msgs::msg::MarkerTracked>("foo", 10);
      auto timer_callback =
          [this]() -> void {
              auto message = Tracked<visualization_msgs::msg::Marker>();
              std::cout << "a" << std::endl;
              auto z = loc(0.0);

              auto pose = GET_FIELD(message, pose);
              auto position = GET_FIELD(pose, position);

              SET_FIELD(position, x, x);
              SET_FIELD(position, y, y);
              SET_FIELD(position, z, z);

              SET_FIELD(pose, position, position);
              SET_FIELD(message, pose, pose);

              reevaluate(message);

              publisher_->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(message));
          };
      timer_ = create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rosslt_msgs::msg::MarkerTracked>::SharedPtr publisher_;
  Tracked<double> x, y;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
