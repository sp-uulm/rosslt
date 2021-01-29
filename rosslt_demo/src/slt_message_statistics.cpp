#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosslt_msgs/msg/pose_tracked.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"
#include "rosslt_msgs/msg/source_change.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class StatisticsNode : public rclcpp::Node {
public:
    StatisticsNode() : rclcpp::Node("slt_message_statistics") {

        target_pose_subscriber_ = create_subscription<rosslt_msgs::msg::PoseTracked>("target_pose", rclcpp::QoS(rclcpp::KeepLast(20)),
            [this](rosslt_msgs::msg::PoseTracked::UniquePtr msg) {
                rclcpp::SerializedMessage serialized_msg;
                rclcpp::Serialization<rosslt_msgs::msg::PoseTracked> serialization;
                serialization.serialize_message(msg.get(), &serialized_msg);
                size_t tracked_size = serialized_msg.size();

                rclcpp::Serialization<geometry_msgs::msg::Pose> serialization2;
                serialization2.serialize_message(&msg->data, &serialized_msg);
                size_t untracked_size = serialized_msg.size();

                rclcpp::Serialization<rosslt_msgs::msg::LocationHeader> serialization3;
                serialization3.serialize_message(&msg->location, &serialized_msg);
                size_t header_size = serialized_msg.size();

                size_t expression_size = 0;

                for (const auto& l : msg->location.locations)
                    expression_size += l.expression.size();

                size_t path_size = 0;

                for (const auto& l : msg->location.paths)
                    path_size += l.size();

                RCLCPP_INFO_STREAM(get_logger(), "target_pose " << tracked_size << " " << header_size << " " << untracked_size << " " << expression_size << " " << path_size);
            });
        control_target_subscriber_ = create_subscription<rosslt_msgs::msg::PoseTracked>("control_target", rclcpp::QoS(rclcpp::KeepLast(20)),
            [this](rosslt_msgs::msg::PoseTracked::UniquePtr msg) {
                rclcpp::SerializedMessage serialized_msg;
                rclcpp::Serialization<rosslt_msgs::msg::PoseTracked> serialization;
                serialization.serialize_message(msg.get(), &serialized_msg);
                size_t tracked_size = serialized_msg.size();

                rclcpp::Serialization<geometry_msgs::msg::Pose> serialization2;
                serialization2.serialize_message(&msg->data, &serialized_msg);
                size_t untracked_size = serialized_msg.size();

                rclcpp::Serialization<rosslt_msgs::msg::LocationHeader> serialization3;
                serialization3.serialize_message(&msg->location, &serialized_msg);
                size_t header_size = serialized_msg.size();

                size_t expression_size = 0;

                for (const auto& l : msg->location.locations)
                    expression_size += l.expression.size();

                size_t path_size = 0;

                for (const auto& l : msg->location.paths)
                    path_size += l.size();

                RCLCPP_INFO_STREAM(get_logger(), "control_target " << tracked_size << " " << header_size << " " << untracked_size << " " << expression_size << " " << path_size);
            });
        tracked_marker_subscriber_ = create_subscription<rosslt_msgs::msg::MarkerTracked>("tracked_marker", rclcpp::QoS(rclcpp::KeepLast(20)),
            [this](rosslt_msgs::msg::MarkerTracked::UniquePtr msg) {
                rclcpp::SerializedMessage serialized_msg;
                rclcpp::Serialization<rosslt_msgs::msg::MarkerTracked> serialization;
                serialization.serialize_message(msg.get(), &serialized_msg);
                size_t tracked_size = serialized_msg.size();

                rclcpp::Serialization<visualization_msgs::msg::Marker> serialization2;
                serialization2.serialize_message(&msg->data, &serialized_msg);
                size_t untracked_size = serialized_msg.size();

                rclcpp::Serialization<rosslt_msgs::msg::LocationHeader> serialization3;
                serialization3.serialize_message(&msg->location, &serialized_msg);
                size_t header_size = serialized_msg.size();

                size_t expression_size = 0;

                for (const auto& l : msg->location.locations)
                    expression_size += l.expression.size();

                size_t path_size = 0;

                for (const auto& l : msg->location.paths)
                    path_size += l.size();

                RCLCPP_INFO_STREAM(get_logger(), "tracked_marker " << tracked_size << " " << header_size << " " << untracked_size << " " << expression_size << " " << path_size);
            });
    }

private:
    rclcpp::Subscription<rosslt_msgs::msg::PoseTracked>::SharedPtr target_pose_subscriber_;
    rclcpp::Subscription<rosslt_msgs::msg::PoseTracked>::SharedPtr control_target_subscriber_;
    rclcpp::Subscription<rosslt_msgs::msg::MarkerTracked>::SharedPtr tracked_marker_subscriber_;
    rclcpp::Subscription<rosslt_msgs::msg::SourceChange>::SharedPtr source_change_subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatisticsNode>());
    rclcpp::shutdown();
    return 0;
}
