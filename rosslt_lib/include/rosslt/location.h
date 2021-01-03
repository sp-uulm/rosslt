#ifndef LOCATION_H
#define LOCATION_H

#include <string>
#include <unordered_map>

#include "rosslt_msgs/msg/location.hpp"
#include "rosslt_msgs/msg/location_header.hpp"


class Location {
public:
    Location() = default;

    Location(std::string source_node, unsigned int location_id);

    Location(const rosslt_msgs::msg::Location& loc);

    operator rosslt_msgs::msg::Location () const;

    bool operator== (const Location& other) const {
        return source_node == other.source_node &&
               location_id == other.location_id;
    }

    bool is_valid() const {
        return !source_node.empty();
    }

    std::string source_node;
    int location_id = 0;
    std::string expression; //rpn; value starts on stack
};

using LocationMap = std::unordered_map<std::string, Location>;

LocationMap add_locations(LocationMap a, const LocationMap& b, std::string prefix = "") {
    for (const auto& [k, v] : b) {
        if (prefix.size() == 0) {
            a[k] = v;
        } else {
            a[k == "." ? prefix : prefix + "/" + k] = v;
        }
    }

    return a;
}

LocationMap remove_locations(const LocationMap& a, std::string prefix = "") {
    LocationMap copy;

    for (const auto& [k, v] : a) {
        if (!k.rfind(prefix, 0) == 0) {// !k.starts_with(prefix)
            copy[k] = v;
        }
    }

    return copy;
}

LocationMap location_slice(const LocationMap& location, const std::string& prefix) {
    LocationMap map;

    for (const auto& [k, v] : location) {
        if (k.rfind(prefix, 0) == 0) {// k.starts_with(prefix)
            auto new_key = k.substr(prefix.size());
            map[new_key.empty() ? "." : new_key.substr(1)] = v;
        }
    }

    return map;
}

#endif // LOCATION_H
