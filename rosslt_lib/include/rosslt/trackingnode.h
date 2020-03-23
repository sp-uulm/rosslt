#ifndef TRACKINGNODE_H
#define TRACKINGNODE_H

#include "rosslt/tracked.h"
#include "rosslt/location.h"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode(const std::string& name);

protected:
    template<typename T>
    Tracked<T> loc(T data, unsigned id = static_cast<unsigned int>(rand())) {
        declare_parameter("loc" + std::to_string(id), data);
        Location location {get_fully_qualified_name(), id};
        return Tracked<T>(data, location);
    }

    template<typename T, typename U>
    void force_value(const Tracked<T>& val, const U& new_val) {
        if (!val.get_location().at(".").is_valid())
            return;

        rclcpp::AsyncParametersClient param_client(shared_from_this(), val.get_location().at(".").source_node);
        param_client.wait_for_service();

        // TODO: use inv_plus etc
        auto new_val_rev = applyExpression(static_cast<T>(new_val), reverseExpression(val.get_location().at(".").expression));
        param_client.set_parameters({rclcpp::Parameter("loc" + std::to_string(val.get_location().at(".").location_id), new_val_rev)});
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
                get_parameter("loc" + std::to_string(val.get_location().at(".").location_id), val.get_data());
            } else {
                rclcpp::SyncParametersClient param_client(shared_from_this(), val.get_location().at(".").source_node);
                param_client.wait_for_service();
                val.get_data() = param_client.get_parameter("loc" + std::to_string(val.get_location().at(".").location_id), val.get_data());
            }
            val.get_data() = applyExpression(val.get_data(), val.get_location()["."].expression);
            return val;
        }
    }

};

#endif // TRACKINGNODE_H
