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
      x = loc(0.0);
      y = loc(1.5);

      publisher_ = create_publisher<rosslt_msgs::msg::MarkerTracked>("tracked_marker", 10);
      auto timer_callback =
          [this]() -> void {
              auto z = loc(2.0);

              publish_marker(0, x, y, z);
              publish_marker(1, x+2, y, 0.0);
          };
      timer_ = create_wall_timer(500ms, timer_callback);
  }

  void publish_marker(int id, Tracked<double> x, Tracked<double> y, Tracked<double> z) {
      auto message = Tracked<visualization_msgs::msg::Marker>();
      auto pose = GET_FIELD(message, pose);
      auto position = GET_FIELD(pose, position);

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = "world";

      std_msgs::msg::ColorRGBA color;
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;

      geometry_msgs::msg::Vector3 scale;
      scale.x = 0.2;
      scale.y = 0.2;
      scale.z = 0.2;

      SET_FIELD(message, header, header);
      SET_FIELD(message, id, id);
      SET_FIELD(message, type, visualization_msgs::msg::Marker::SPHERE);
      SET_FIELD(message, color, color);
      SET_FIELD(message, scale, scale);
      SET_FIELD(position, x, x);
      SET_FIELD(position, y, y);
      SET_FIELD(position, z, z);

      SET_FIELD(pose, position, position);
      SET_FIELD(message, pose, pose);

      reevaluate(message);

      publisher_->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(message));
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
