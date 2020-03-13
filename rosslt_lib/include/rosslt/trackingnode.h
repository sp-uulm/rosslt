#ifndef TRACKINGNODE_H
#define TRACKINGNODE_H

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rosslt_msgs/msg/location.hpp"
#include "rosslt_msgs/msg/location_header.hpp"

class Location {
public:
    Location()
    {
    }

    Location(const std::string& source_node, unsigned int location_id)
        : source_node(source_node), location_id(location_id)
    {
    }

    Location(const rosslt_msgs::msg::Location& loc) {
        source_node = loc.source_node;
        location_id = loc.location_id;
        expression = loc.expression;
    }

    operator rosslt_msgs::msg::Location () const {
        rosslt_msgs::msg::Location loc;
        loc.set__source_node(source_node);
        loc.set__location_id(location_id);
        loc.set__expression(expression);
        return loc;
    }

    bool is_valid() const {
        return !source_node.empty();
    }

    std::string source_node;
    unsigned int location_id;
    std::string expression; //rpn; value starts on stack
};

template<typename T>
class Tracked {
public:
    using LocationMap = std::unordered_map<std::string, Location>;

    Tracked(T data, Location loc) : data(data) {
        location["."] = loc;
    }

    Tracked(T data, rosslt_msgs::msg::LocationHeader loc) : data(data) {
        for (unsigned i = 0; i < loc.paths.size(); ++i) {
            location[loc.paths[i]] = loc.locations[i];
        }
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

    LocationMap& get_location() {
        return location;
    }

    const LocationMap& get_location() const {
        return location;
    }

    template<typename U>
    Tracked<U> get_field(const U& member, const std::string& name) const {
        assert(&member >= &data && &member < &data + sizeof(T));

        return Tracked<U>(member, Location());
    }

    template<typename U>
    Tracked<T> set_field(const U& member, const std::string& name) const {
        assert(&member >= &data && &member < &data + sizeof(T));

        return *this;
    }

    Tracked<T> operator++(int) {
        Tracked<T> copy = *this;
        data++;
        location["."].expression += "1;+;";
        return copy;
    }

    Tracked<T> operator+(const T& other) const {
        Tracked<T> copy = *this;
        copy.data = data + other;
        copy.location["."].expression += std::to_string(other) + ";+;";
        return copy;
    }

    Tracked<T> operator*(const T& other) const {
        Tracked<T> copy = *this;
        copy.data = data * other;
        copy.location["."].expression += std::to_string(other) + ";*;";
        return copy;
    }

    Tracked<T> operator-(const T& other) const {
        Tracked<T> copy = *this;
        copy.data = data - other;
        copy.location["."].expression += std::to_string(other) + ";-;";
        return copy;
    }

    Tracked<T> operator/(const T& other) const {
        Tracked<T> copy = *this;
        copy.data = data / other;
        copy.location["."].expression += std::to_string(other) + ";/;";
        return copy;
    }

    friend Tracked<T> operator+(const T& lhs, const Tracked<T>& rhs) {
        return rhs + lhs;
    }

    friend Tracked<T> operator*(const T& lhs, const Tracked<T>& rhs) {
        return rhs * lhs;
    }

    friend Tracked<T> operator-(const T& lhs, const Tracked<T>& rhs) {
        return rhs - lhs;
    }

    friend Tracked<T> operator/(const T& lhs, const Tracked<T>& rhs) {
        return rhs / lhs;
    }

private:
    T data;
    LocationMap location;
};

template <typename T>
T sto(const std::string& s) {
    return s;
}

template <>
int sto<int>(const std::string& s) {
    return stoi(s);
}

template <typename T>
T applyExpression(T val, const std::string& exp_string) {
    std::cout << "applyExpression '" << exp_string << "' to " << val << std::endl;
    std::vector<T> stack;
    stack.push_back(val);
    size_t end = exp_string.find(';');
    for (size_t start = 0; end != std::string::npos; end = exp_string.find(';', start)) {
        std::string token = exp_string.substr(start, end-start);
        std::cout << "token: '"<< token << "'" << std::endl;
        start = end+1;

        if (token == "+") {
            *(stack.end()-2) = *(stack.end()-2) + stack.back();
            stack.pop_back();
        } else if (token == "-") {
            *(stack.end()-2) = *(stack.end()-2) - stack.back();
            stack.pop_back();
        } else if (token == "*") {
            *(stack.end()-2) = *(stack.end()-2) * stack.back();
            stack.pop_back();
        } else if (token == "/") {
            *(stack.end()-2) = *(stack.end()-2) / stack.back();
            stack.pop_back();
        } else {
            stack.push_back(sto<T>(token));
        }
    }
    std::cout << "result: " << stack.back() << std::endl;
    return stack.back();
}

std::string reverseExpression(const std::string& exp);

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
        auto new_val_rev = applyExpression(static_cast<T>(new_val), reverseExpression(val.get_location().at(".").expression));
        param_client.set_parameters({rclcpp::Parameter("loc" + std::to_string(val.get_location().at(".").location_id), new_val_rev)});
    }

    template<typename T>
    Tracked<T>& reevaluate(Tracked<T>& val) {
        if (!val.get_location().at(".").is_valid())
            return val;

        if (val.get_location().source_node == get_fully_qualified_name()) {
            get_parameter("loc" + std::to_string(val.get_location().at(".").location_id), val.get_data());
        } else {
            rclcpp::SyncParametersClient param_client(shared_from_this(), val.get_location().at(".").source_node);
            param_client.wait_for_service();
            val.get_data() = param_client.get_parameter("loc" + std::to_string(val.get_location().at(".").location_id), val.get_data());
        }
        val.get_data() = applyExpression(val.get_data(), val.get_location()["."].expression);
        return val;
    }

};

#endif // TRACKINGNODE_H
