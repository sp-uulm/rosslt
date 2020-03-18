
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"

#include "rosslt/trackingnode.h"

std::string to_string(const visualization_msgs::msg::Marker& m) {
    return "Marker {" + std::to_string(m.pose.position.x) + ","
                      + std::to_string(m.pose.position.y) + ","
                      + std::to_string(m.pose.position.z) + "}";
}

class Subscriber : public TrackingNode {
public:
  Subscriber()
  : TrackingNode("vis_listener")
  {
    subscriber_ = create_subscription<rosslt_msgs::msg::MarkerTracked>("foo", rclcpp::QoS(rclcpp::KeepLast(20)),
    [this](rosslt_msgs::msg::MarkerTracked::UniquePtr msg) {
        Tracked<visualization_msgs::msg::Marker> marker {*msg};

        auto pose = GET_FIELD(marker, pose);
        auto position = GET_FIELD(pose, position);

        Tracked<double> x = GET_FIELD(position, x);
        RCLCPP_INFO(get_logger(), "Got message: '%s' from %s:%i", to_string(marker.get_data()).c_str(),
                    position.get_location()["x"].source_node.c_str(),
                    position.get_location()["x"].location_id);
        force_value(x, x + 1.0);
    });

  }

private:
  rclcpp::Subscription<rosslt_msgs::msg::MarkerTracked>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}

