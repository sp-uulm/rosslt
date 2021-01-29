#include "rosslt/location.h"

LocationMap add_locations(LocationMap a, const LocationMap& b, std::string prefix) {
    for (const auto& [k, v] : b) {
        if (prefix.size() == 0) {
            a[k] = v;
        } else {
            a[k == "." ? prefix : prefix + "/" + k] = v;
        }
    }

    return a;
}

LocationMap remove_locations(const LocationMap& a, std::string prefix) {
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
