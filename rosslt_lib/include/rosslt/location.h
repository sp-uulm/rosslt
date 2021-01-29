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

LocationMap add_locations(LocationMap a, const LocationMap& b, std::string prefix = "");

LocationMap remove_locations(const LocationMap& a, std::string prefix = "");

LocationMap location_slice(const LocationMap& location, const std::string& prefix);

#endif // LOCATION_H
