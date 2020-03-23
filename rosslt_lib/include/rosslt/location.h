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

    bool is_valid() const {
        return !source_node.empty();
    }

    std::string source_node;
    unsigned int location_id;
    std::string expression; //rpn; value starts on stack
};

using LocationMap = std::unordered_map<std::string, Location>;

#endif // LOCATION_H
