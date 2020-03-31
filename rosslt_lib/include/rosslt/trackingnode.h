#ifndef TRACKINGNODE_H
#define TRACKINGNODE_H

#include "rosslt/tracked.h"
#include "rosslt/location.h"

#include "rosslt_msgs/msg/source_change.hpp"
#include "rosslt_msgs/srv/get_value.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <experimental/source_location>

#include "rclcpp/rclcpp.hpp"

using source_location = std::experimental::source_location;

namespace std {
template <> struct hash<std::experimental::source_location>
{
    size_t operator()(const std::experimental::source_location & x) const
    {
        return std::hash<std::string>()(x.file_name())^std::hash<std::string>()(x.function_name())^x.line()^x.column();
    }
};

template <> struct equal_to<std::experimental::source_location>
{
    size_t operator()(const std::experimental::source_location & x, const std::experimental::source_location & y) const
    {
        return strcmp(x.file_name(), y.file_name()) == 0 && strcmp(x.function_name(), y.function_name()) == 0 && x.line() == y.line() && x.column() == y.column();
    }
};
}

template <typename T, template <class> class Fut, typename Node>
T get_future(Node& node, Fut<T>& fut) {
    if (rclcpp::spin_until_future_complete(node.get_node_base_interface(), fut) ==
            rclcpp::executor::FutureReturnCode::SUCCESS) {
        return fut.get();
    }
    RCLCPP_ERROR(node.get_logger(), "Error getting result from future");
    return nullptr;
}

struct LocationFunc {
    std::function<std::string(int32_t)> get;
    std::function<void(int32_t, const std::string&)> set;
};

class LocationManager {
public:

    LocationManager(rclcpp::Node& node) : node(node) {
        using namespace std::placeholders;
        using namespace rclcpp;
        sc_sub = node.create_subscription<rosslt_msgs::msg::SourceChange>("/sc", QoS(KeepLast(10)), std::bind(&LocationManager::on_source_change, this, _1));
        sc_pub = node.create_publisher<rosslt_msgs::msg::SourceChange>("/sc", QoS(KeepLast(10)));
        get_value_service = node.create_service<rosslt_msgs::srv::GetValue>(std::string(node.get_fully_qualified_name()) + "/get_slt_value", std::bind(&LocationManager::on_get_value, this, _1, _2));
    }

    int32_t create_location(const LocationFunc& f, const source_location& sl = source_location::current()) {
        if (auto it = source_locations.find(sl); it == source_locations.end()) {
            locations.push_back(f);
            source_locations[sl] = locations.size() - 1;
            return locations.size() - 1;
        } else {
            return it->second;
        }
    }

    int32_t get_location_id(const source_location& sl) const {
        if (auto it = source_locations.find(sl); it != source_locations.end()) {
            return it->second;
        } else {
            return -1;
        }
    }

    void change_location(const std::string& source_node, int32_t location_id, const std::string& new_value) {
        auto msg = std::make_unique<rosslt_msgs::msg::SourceChange>();
        msg->set__source_node(source_node);
        msg->set__location_id(location_id);
        msg->set__new_value(new_value);

        if (source_node == node.get_fully_qualified_name()) {
            on_source_change(move(msg));
        } else {
            sc_pub->publish(move(msg));
        }
    }

    std::string current_value(int32_t location_id) const {
        return locations.at(location_id).get(location_id);
    }

private:
    void on_get_value(rosslt_msgs::srv::GetValue_Request::SharedPtr request, rosslt_msgs::srv::GetValue_Response::SharedPtr response) const {
        if (static_cast<int32_t>(locations.size()) > request->location_id && request->location_id >= 0) {
            response->set__current_value(locations.at(request->location_id).get(request->location_id));
            response->set__valid_id(true);
        } else {
            response->set__valid_id(false);
        }
    }

    void on_source_change(rosslt_msgs::msg::SourceChange::UniquePtr msg) {
        if (msg->source_node == node.get_fully_qualified_name()) {
            auto id = msg->location_id;
            if (id < 0 || id >= static_cast<int32_t>(locations.size())) {
                RCLCPP_WARN(node.get_logger(), "source change to unknown location id %d", id);
                return;
            }

            locations[id].set(id, msg->new_value);
        }
    }

    // vector indexed by location_id, function is called with the new value
    std::unordered_map<source_location, int32_t> source_locations;
    std::vector<LocationFunc> locations;
    std::shared_ptr<rclcpp::Subscription<rosslt_msgs::msg::SourceChange>> sc_sub;
    std::shared_ptr<rclcpp::Publisher<rosslt_msgs::msg::SourceChange>> sc_pub;
    std::shared_ptr<rclcpp::Service<rosslt_msgs::srv::GetValue>> get_value_service;

    rclcpp::Node& node;
};

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode(const std::string& name);

protected:
    template<typename T>
    Tracked<T> loc(T data, const source_location& sl = source_location::current()) {
        int32_t id;

        if (id = loc_mgr.get_location_id(sl); id < 0) {
            LocationFunc lf {
                // get
                [this](int32_t id) -> std::string {
                    return get_parameter("loc" + std::to_string(id)).value_to_string();
                },

                // set
                [this](int32_t id, const std::string& new_val) {
                    set_parameter(rclcpp::Parameter("loc" + std::to_string(id), sto<T>(new_val)));
                }
            };

            id = loc_mgr.create_location(lf, sl);

            declare_parameter("loc" + std::to_string(id), data);
        } else {
            data = sto<T>(get_parameter("loc" + std::to_string(id)).value_to_string());
        }

        Location location {get_fully_qualified_name(), id};
        return Tracked<T>(data, location);
    }

    template<typename T, typename U>
    void force_value(const Tracked<T>& val, const U& new_val) {
        if (!val.get_location().at(".").is_valid())
            return;

        // TODO: use inv_plus etc
        auto new_val_rev = applyExpression(static_cast<T>(new_val), reverseExpression(val.get_location().at(".").expression));
        loc_mgr.change_location(val.get_location().at(".").source_node, val.get_location().at(".").location_id, std::to_string(new_val_rev));
    }

    template<typename T>
    Tracked<T>& reevaluate(Tracked<T>& val) {
        if constexpr (rosidl_generator_traits::is_message<T>::value) {
            map_leaves(val, [this](auto& x) {
                reevaluate(x);
            });
            return val;
        } else {
            if (!val.get_location().at(".").is_valid())
                return val;

            if (val.get_location().at(".").source_node == get_fully_qualified_name()) {
                val.get_data() = sto<T>(loc_mgr.current_value(val.get_location().at(".").location_id));
            } else {
                using namespace std::chrono_literals;

                auto client = create_client<rosslt_msgs::srv::GetValue>(val.get_location().at(".").source_node + "/get_slt_value");
                client->wait_for_service(1s);

                auto request = std::make_shared<rosslt_msgs::srv::GetValue::Request>();
                request->set__location_id(val.get_location().at(".").location_id);
                auto response_fut = client->async_send_request(request);
                auto response = get_future(*this, response_fut);

                if (response->valid_id)
                    val.get_data() = sto<T>(response->current_value);
            }
            val.get_data() = applyExpression(val.get_data(), val.get_location()["."].expression);
            return val;
        }
    }

private:
    LocationManager loc_mgr;
};

#endif // TRACKINGNODE_H
