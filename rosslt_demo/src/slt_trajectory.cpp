#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/pose_tracked.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"

#include "rosslt/trackingnode.h"

using namespace std::chrono_literals;

class Bezier {
public:
    Bezier() {
        SET_FIELD(b0, x, 0.0);
        SET_FIELD(b0, y, 0.0);
        SET_FIELD(b0, z, 0.0);

        b2 = b1 = b0;

        tau = 1.0;
    }

    Bezier(const Tracked<geometry_msgs::msg::Pose>& start, const Tracked<geometry_msgs::msg::Pose>& end, double tau) : tau(tau) {
        b0 = GET_FIELD(start, position);
        b2 = GET_FIELD(end, position);

        // create b1 inbetween b0 and b2
        // TODO: use direction in b2
        auto x = GET_FIELD(b0, x) + (GET_FIELD(b2, x) - GET_FIELD(b0, x))/2;
        auto y = GET_FIELD(b0, y) + (GET_FIELD(b2, y) - GET_FIELD(b0, y))/2;
        auto z = GET_FIELD(b0, z) + (GET_FIELD(b2, z) - GET_FIELD(b0, z))/2;

        SET_FIELD(b1, x, x);
        SET_FIELD(b1, y, y);
        SET_FIELD(b1, z, z);
    }

    Tracked<geometry_msgs::msg::Point> get_control_point() const {
        return b1;
    }

    Tracked<geometry_msgs::msg::Point> get_trajectory_point(double t) const {
        if (t <= 0)
            return b0;
        if (t >= tau)
            return b2;

        t /= tau; // scale t to [0,1]

        auto x = (1-t) * (1-t) * GET_FIELD(b0, x) + t * (1-t) * GET_FIELD(b1, x) + t * t * GET_FIELD(b2, x);
        auto y = (1-t) * (1-t) * GET_FIELD(b0, y) + t * (1-t) * GET_FIELD(b1, y) + t * t * GET_FIELD(b2, y);
        auto z = (1-t) * (1-t) * GET_FIELD(b0, z) + t * (1-t) * GET_FIELD(b1, z) + t * t * GET_FIELD(b2, z);

        Tracked<geometry_msgs::msg::Point> result;

        SET_FIELD(result, x, x);
        SET_FIELD(result, y, y);
        SET_FIELD(result, z, z);

        return result;
    }

private:
    Tracked<geometry_msgs::msg::Point> b0, b1, b2;
    double tau;
};

class TrajectoryNode : public TrackingNode {
public:
  TrajectoryNode()
  : TrackingNode("slt_trajectory")
  {
    planning_time = now();

    marker_publisher_ = create_publisher<rosslt_msgs::msg::MarkerTracked>("trajectory_marker", 10);
    control_target_publisher_ = create_publisher<rosslt_msgs::msg::PoseTracked>("control_target", 10);

    target_pose_subscriber_ = create_subscription<rosslt_msgs::msg::PoseTracked>("target_pose", rclcpp::QoS(rclcpp::KeepLast(20)),
    [this](rosslt_msgs::msg::PoseTracked::UniquePtr msg) {
        Tracked<geometry_msgs::msg::Pose> pose_msg {*msg};
        auto target_position = GET_FIELD(pose_msg, position);
        geometry_msgs::msg::Pose current_pose;
        current_pose.position.x = 0.0;
        current_pose.position.y = 0.0;
        current_pose.position.z = 0.0;

        current_trajectory = Bezier(current_pose, pose_msg, 4.0);
        planning_time = now();

        // publish line strip
        marker_publisher_->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(
                                       create_curve_marker(0, current_trajectory)));

        // publish trajectory control point
        auto control_position = current_trajectory.get_control_point();
        marker_publisher_->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(
                                       create_point_marker(1, GET_FIELD(control_position, x), GET_FIELD(control_position, y), GET_FIELD(control_position, z))));

        // publish trajectory endpoint
        marker_publisher_->publish(static_cast<rosslt_msgs::msg::MarkerTracked>(
                                       create_point_marker(2, GET_FIELD(target_position, x), GET_FIELD(target_position, y), GET_FIELD(target_position, z))));
    });

    auto timer_callback =
        [this]() -> void {
            double t = (now() - planning_time).seconds();
            auto point = current_trajectory.get_trajectory_point(t);
            Tracked<geometry_msgs::msg::Pose> pose;
            SET_FIELD(pose, position, point);
            control_target_publisher_->publish(static_cast<rosslt_msgs::msg::PoseTracked>(pose));
        };
    timer_ = create_wall_timer(20ms, timer_callback);
  }

  Tracked<visualization_msgs::msg::Marker> create_point_marker(int id, Tracked<double> x, Tracked<double> y, Tracked<double> z) {
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

      return message;
  }

  Tracked<visualization_msgs::msg::Marker> create_curve_marker(int id, const Bezier& curve) {
      auto message = Tracked<visualization_msgs::msg::Marker>();

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = "world";

      std_msgs::msg::ColorRGBA color;
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;
      color.a = 1.0;

      geometry_msgs::msg::Vector3 scale;
      scale.x = 0.1;
      scale.y = 0.1;
      scale.z = 0.1;

      SET_FIELD(message, header, header);
      SET_FIELD(message, id, id);
      SET_FIELD(message, type, visualization_msgs::msg::Marker::LINE_STRIP);
      SET_FIELD(message, color, color);
      SET_FIELD(message, scale, scale);

      Tracked<std::vector<geometry_msgs::msg::Point>> points;

      for (int i = 0; i <= 5; ++i) {
          points.push_back(curve.get_trajectory_point(i*0.2));
      }

      SET_FIELD(message, points, points);

      return message;
  }

private:
    Bezier current_trajectory;
    rclcpp::Time planning_time;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rosslt_msgs::msg::PoseTracked>::SharedPtr target_pose_subscriber_;
    rclcpp::Publisher<rosslt_msgs::msg::MarkerTracked>::SharedPtr marker_publisher_;
    rclcpp::Publisher<rosslt_msgs::msg::PoseTracked>::SharedPtr control_target_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}

