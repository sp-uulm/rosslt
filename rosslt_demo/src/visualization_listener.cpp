
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"
#include <interactive_markers/interactive_marker_server.hpp>

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
    server = std::make_unique<interactive_markers::InteractiveMarkerServer>("simple_marker", this);

    subscriber_ = create_subscription<rosslt_msgs::msg::MarkerTracked>("foo", rclcpp::QoS(rclcpp::KeepLast(20)),
    [this](rosslt_msgs::msg::MarkerTracked::UniquePtr msg) {
        Tracked<visualization_msgs::msg::Marker> marker {*msg};

        auto pose = GET_FIELD(marker, pose);
        auto position = GET_FIELD(pose, position);

        visualization_msgs::msg::InteractiveMarker m;

        m.header.frame_id = "world";
        m.header.stamp = now();
        m.name = "foo_" + marker.get_data().ns + std::to_string(marker.get_data().id);
        m.pose = pose;

        visualization_msgs::msg::InteractiveMarkerControl marker_control;
        marker_control.always_visible = true;

        auto marker_ = marker.get_data();
        marker_.pose = geometry_msgs::msg::Pose();
        marker_control.markers.push_back(marker_);

        m.controls.push_back(marker_control);

        if (marker.get_location()["pose/position/x"].is_valid()) {
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.name = "move_x";
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

            // add the control to the interactive marker
            m.controls.push_back(control);
        }

        if (marker.get_location()["pose/position/y"].is_valid()) {
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.name = "move_y";
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

            // add the control to the interactive marker
            m.controls.push_back(control);
        }

        if (marker.get_location()["pose/position/z"].is_valid()) {
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.name = "move_z";
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

            // add the control to the interactive marker
            m.controls.push_back(control);
        }

        server->insert(m, [this, position](const auto& feedback) {
            if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
                if (feedback->control_name == "move_x") {
                    force_value(GET_FIELD(position, x), feedback->pose.position.x);
                }
                if (feedback->control_name == "move_y") {
                    force_value(GET_FIELD(position, y), feedback->pose.position.y);
                }
                if (feedback->control_name == "move_z") {
                    force_value(GET_FIELD(position, z), feedback->pose.position.z);
                }
            }
        });

        server->applyChanges();
    });

  }

private:
    rclcpp::Subscription<rosslt_msgs::msg::MarkerTracked>::SharedPtr subscriber_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}

