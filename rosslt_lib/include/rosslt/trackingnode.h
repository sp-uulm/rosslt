#ifndef TRACKINGNODE_H
#define TRACKINGNODE_H

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/location.hpp"

template<typename T>
class Location {
public:
    Location(const std::string& source_node, unsigned int location_id)
        : source_node(source_node), location_id(location_id)
    {
    }

    Location(const rosslt_msgs::msg::Location& loc) {
        source_node = loc.source_node;
        location_id = loc.location_id;
    }

    operator rosslt_msgs::msg::Location () {
        rosslt_msgs::msg::Location loc;
        loc.set__source_node(source_node);
        loc.set__location_id(location_id);
        return loc;
    }

    std::string source_node;
    unsigned int location_id;
private:
    //std::function<T(T)> reverse_func;
};

template<typename T>
class Tracked {
public:
    Tracked(T data, Location<T> location) : data(data), location(location) {

    }

    operator T () const {
        return get_data();
    }

    T& get_data() {
        return data;
    }

    const T& get_data() const {
        return data;
    }

    Location<T>& get_location() {
        return location;
    }

    const Location<T>& get_location() const {
        return location;
    }

    Tracked<T> operator++(int) {
        Tracked<T> copy = *this;
        data++;
        return copy;
    }

private:
    T data;
    Location<T> location;
};

class TrackingNode : public rclcpp::Node {
public:
    TrackingNode(const std::string& name);

protected:
    template<typename T>
    Tracked<T> loc(T data, unsigned id = static_cast<unsigned int>(rand())) {
        declare_parameter("loc" + std::to_string(id), data);
        Location<T> location {get_fully_qualified_name(), id};
        return Tracked<T>(data, location);
    }

    template<typename T>
    void force_value(Tracked<T>& val, const T& new_val) {
        rclcpp::AsyncParametersClient param_client(shared_from_this(), val.get_location().source_node);
        param_client.wait_for_service();
        param_client.set_parameters({rclcpp::Parameter("loc" + std::to_string(val.get_location().location_id), new_val)});
    }

    template<typename T>
    Tracked<T>& reevaluate(Tracked<T>& val) {
        if (val.get_location().source_node == get_fully_qualified_name()) {
            get_parameter("loc" + std::to_string(val.get_location().location_id), val.get_data());
        } else {
            rclcpp::SyncParametersClient param_client(shared_from_this(), val.get_location().source_node);
            param_client.wait_for_service();
            val.get_data() = param_client.get_parameter("loc" + std::to_string(val.get_location().location_id), val.get_data());
        }
        return val;
    }

};

#endif // TRACKINGNODE_H
