#include <string>
#include <queue>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "rosslt/trackingnode.h"

#include "rosslt_msgs/msg/pose_tracked.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.hpp>

using namespace std::chrono_literals;

class SltQuadNode : public TrackingNode {
public:
    SltQuadNode() : TrackingNode("slt_quad") {
        server = std::make_unique<interactive_markers::InteractiveMarkerServer>("interactive_marker2", this);

        target_subscriber = create_subscription<rosslt_msgs::msg::PoseTracked>("control_target", 200, [this] (const rosslt_msgs::msg::PoseTracked::UniquePtr tgt) {
            control_delay_buffer.push(make_pair(static_cast<Tracked<geometry_msgs::msg::Pose>>(*tgt), now()));
        });

        br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        auto position = GET_FIELD(ps, position);
        SET_FIELD(position, x, 0.0);
        SET_FIELD(position, y, 0.0);
        SET_FIELD(position, z, 0.0);
        SET_FIELD(ps, position, position);

        timer = create_wall_timer(20ms, [this]() -> void {
          while (!control_delay_buffer.empty() &&
                  rclcpp::Time(control_delay_buffer.front().second) < (now() - QUAD_DELAY)) {
            ps = control_delay_buffer.front().first;
            control_delay_buffer.pop();
          }

          geometry_msgs::msg::TransformStamped transform;
          transform.header.stamp = now();
          transform.header.frame_id = "world";
          transform.child_frame_id = "slt_quad";

          auto position = GET_FIELD(ps, position);
          transform.transform.translation.x = GET_FIELD(position, x);
          transform.transform.translation.y = GET_FIELD(position, y);
          transform.transform.translation.z = GET_FIELD(position, z);

          transform.transform.rotation = GET_FIELD(ps, orientation);

          br->sendTransform(transform);
        });

        timer2 = create_wall_timer(200ms, [this]() -> void {
          create_interactive_marker(ps);
        });
    }

    void create_interactive_marker(Tracked<geometry_msgs::msg::Pose> pose) {
        visualization_msgs::msg::InteractiveMarker m;

        m.header.frame_id = "world";
        m.header.stamp = now();
        m.name = "slt_quad";
        m.pose = pose;

        visualization_msgs::msg::InteractiveMarkerControl marker_control;
        marker_control.name = "slt_quad";
        marker_control.always_visible = true;
        marker_control.orientation.w = 1;
        marker_control.orientation.x = 0;
        marker_control.orientation.y = 1;
        marker_control.orientation.z = 0;
        marker_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

        visualization_msgs::msg::Marker marker_;

        marker_.scale.x = 0.001;
        marker_.scale.y = 0.001;
        marker_.scale.z = 0.001;
        marker_.mesh_resource = "package://rosslt_demo/meshes/bebop.dae";
        marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker_.header = m.header;
        marker_.ns = "slt_quad";
        marker_.id = 0;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.color.r = 1.0;
        marker_.color.g = 1.0;
        marker_.color.b = 1.0;
        marker_.color.a = 1.0;
        marker_.pose = pose;


        marker_control.markers.push_back(marker_);

        m.controls.push_back(marker_control);

        auto position = GET_FIELD(pose, position);
        server->insert(m, [this, position](const auto& feedback) mutable {
            if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
                // TODO: force at once to avoid overwriting
                if (position.get_location()["z"].is_valid()) {
                    force_value(GET_FIELD(position, z), feedback->pose.position.z);
                }
                if (position.get_location()["y"].is_valid()) {
                    force_value(GET_FIELD(position, y), feedback->pose.position.y);
                }
                if (position.get_location()["x"].is_valid()) {
                    force_value(GET_FIELD(position, x), feedback->pose.position.x);
                }
            }
        });

        server->applyChanges();
    }

private:
    rclcpp::Duration QUAD_DELAY = 100ms;

    std::queue<std::pair<Tracked<geometry_msgs::msg::Pose>, rclcpp::Time>> control_delay_buffer;

    rclcpp::Subscription<rosslt_msgs::msg::PoseTracked>::SharedPtr target_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer2;

    Tracked<geometry_msgs::msg::Pose> ps;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SltQuadNode>());
    rclcpp::shutdown();
    return 0;
}

