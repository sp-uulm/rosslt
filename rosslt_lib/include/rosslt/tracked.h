#ifndef TRACKED_H
#define TRACKED_H

#include "rosslt/helper.h"
#include "rosslt/location.h"

#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

#include <string>

template<typename T>
class Tracked {
public:
    Tracked(T data = T(), Location loc = Location()) : data(data) {
        location["."] = loc;
    }

    Tracked(const T& data, LocationMap loc) : data(data), location(std::move(loc)) {
    }

    Tracked(const T& data, const rosslt_msgs::msg::LocationHeader& loc) : data(data) {
        for (unsigned i = 0; i < loc.paths.size(); ++i) {
            location[loc.paths[i]] = loc.locations[i];
        }
    }

    template <typename Msg>
    Tracked(const Msg& msg) : Tracked(msg2data<T>(msg.data), msg.location) {
    }

    operator T () const {
        return get_data();
    }

    template <typename Msg>
    operator Msg () const {
        Msg msg;
        msg.data = data2msg<decltype(msg.data)>(data);

        for (const auto& [path, loc] : location) {
            if (loc.is_valid()) {
                msg.location.paths.push_back(path);
                msg.location.locations.push_back(loc);
            }
        }

        return msg;
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
        //assert(&member >= &data && &member < &data + sizeof(T));

        LocationMap map;

        for (const auto& [k, v] : location) {
            if (k.rfind(name, 0) == 0) {// k.starts_with(name)
                auto new_key = k.substr(name.size());
                map[new_key.empty() ? "." : new_key.substr(1)] = v;
            }
        }

        return Tracked<U>(member, map);
    }

    template<typename U>
    Tracked<T> set_field(const U& member, const std::string& name, const Tracked<U>& value) const {
        //assert(&member >= &data && &member < &data + sizeof(T));

        Tracked<T> copy = *this;
        *reinterpret_cast<U*>(reinterpret_cast<uint8_t*>(&copy) + (reinterpret_cast<const uint8_t*>(&member) - reinterpret_cast<const uint8_t*>(this))) = value.get_data();

        for (const auto& [k, v] : value.get_location()) {
            copy.location[k == "." ? name : name + "/" + k] = v;
        }

        return copy;
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

#define SET_FIELD(Obj, Field, Value) (Obj) = (Obj).set_field((Obj).get_data().Field, #Field, (Value))

#define GET_FIELD(Obj, Field) (Obj).get_field((Obj).get_data().Field, #Field)

#define APPLY_AS(type) {\
    Tracked<type> t {*reinterpret_cast<type*>(reinterpret_cast<uint8_t*>(&tracked_message.get_data()) + member.offset_ + offset), tracked_message.get_location()[prefix + member.name_]}; \
    func(t);\
    *reinterpret_cast<type*>(reinterpret_cast<uint8_t*>(&tracked_message.get_data()) + member.offset_ + offset) = t.get_data();\
    tracked_message.get_location()[prefix + member.name_] = t.get_location()["."];}


template<typename T, class Visitor>
void map_leaves_(Tracked<T>& tracked_message, const rosidl_typesupport_introspection_cpp::MessageMembers* members, Visitor func, const std::string& prefix, size_t offset) {

    for (unsigned i = 0; i < members->member_count_; ++i) {
        const auto& member = members->members_[i];

        switch (member.type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            APPLY_AS(bool);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
            APPLY_AS(int8_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
            APPLY_AS(uint8_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            APPLY_AS(float);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            APPLY_AS(int16_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            APPLY_AS(int32_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            APPLY_AS(int64_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
            APPLY_AS(wchar_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            APPLY_AS(double);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
            APPLY_AS(std::string);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            APPLY_AS(uint16_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            APPLY_AS(uint32_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            APPLY_AS(uint64_t);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
            {
                const auto* child_members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
                map_leaves_(tracked_message, child_members, func, prefix + member.name_ + "/", offset + member.offset_);
            }
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
//            APPLY_AS(std::wstring);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
            APPLY_AS(long double);
            break;
        }
    }
}

template<typename T, class Visitor>
void map_leaves(Tracked<T>& tracked_message, Visitor func) {
    const auto* handle = rosidl_typesupport_introspection_cpp::get_message_type_support_handle<T>();

    const auto* members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(handle->data);

    map_leaves_(tracked_message, members, func, "", 0);
}

#endif // TRACKED_H
