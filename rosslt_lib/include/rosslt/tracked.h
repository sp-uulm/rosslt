#ifndef TRACKED_H
#define TRACKED_H

#include "rosslt/helper.h"
#include "rosslt/location.h"

#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

# include <boost/iterator/iterator_facade.hpp>

#include <string>
#include <cmath>

template<class> class Tracked;

template <typename T>
struct is_tracked {
    static constexpr bool value = false;
};

template <typename T>
struct is_tracked<Tracked<T>> {
    static constexpr bool value = true;
};

template <typename T>
constexpr bool is_tracked_v = is_tracked<T>::value;

template<typename T>
class Tracked {
public:
    Tracked() : data() {
        location["."] = Location();
    }

    Tracked(const T& data, Location loc = Location()) : data(data) {
        location["."] = loc;
    }

    template <typename U>
    Tracked(U data, Location loc = Location(), std::enable_if_t<!rosidl_generator_traits::is_message<U>::value>* = nullptr) : data(data) {
        location["."] = loc;
    }

    Tracked(const T& data, LocationMap loc) : data(data), location(std::move(loc)) {
        location["."]; // make sure . is initialized, as location_slice does not guarantee it.
    }

    Tracked(const T& data, const rosslt_msgs::msg::LocationHeader& loc) : data(data) {
        for (unsigned i = 0; i < loc.paths.size(); ++i) {
            location[loc.paths[i]] = loc.locations[i];
        }
    }

    Tracked(const Tracked<T>& other) = default;

    template <typename Msg>
    Tracked(const Msg& msg, std::enable_if_t<rosidl_generator_traits::is_message<Msg>::value>* = nullptr) : Tracked(msg2data<T>(msg.data), msg.location) {
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

        return Tracked<U>(member, location_slice(location, name));
    }

    template<typename U>
    Tracked<T> set_field(const U& member, const std::string& name, const Tracked<U>& value) const {
        //assert(&member >= &data && &member < &data + sizeof(T));

        Tracked<T> copy = *this;
        *reinterpret_cast<U*>(reinterpret_cast<uint8_t*>(&copy) + (reinterpret_cast<const uint8_t*>(&member) - reinterpret_cast<const uint8_t*>(this))) = value.get_data();

        copy.location = add_locations(location, value.get_location(), name);

        return copy;
    }

    Tracked<T> sin() const {
        static_assert (std::is_arithmetic_v<T>, "sin can only be called on tracked arithmetic values");
        Tracked<T> copy = *this;
        copy.data = std::sin(data);
        copy.location["."].expression += "sin;";
        return copy;
    }

    Tracked<T> cos() const {
        static_assert (std::is_arithmetic_v<T>, "cos can only be called on tracked arithmetic values");
        Tracked<T> copy = *this;
        copy.data = std::cos(data);
        copy.location["."].expression += "cos;";
        return copy;
    }

    Tracked<T> operator++(int) {
        Tracked<T> copy = *this;
        data++;
        location["."].expression += "1;+;";
        return copy;
    }

    template <typename U, typename V>
    friend Tracked<T> operator+(const U& lhs, const V& rhs) {
        if constexpr (std::is_same_v<U, Tracked<T>>) {
            Tracked<T> copy = lhs;
            copy.data = lhs.data + static_cast<T>(rhs);
            copy.location["."].expression += to_literal(static_cast<T>(rhs)) + ";+;";
            return copy;
        } else if constexpr (std::is_same_v<V, Tracked<T>>) {
            Tracked<T> copy = rhs;
            copy.data = static_cast<T>(lhs) + rhs.data;
            copy.location["."].expression += to_literal(static_cast<T>(lhs)) + ";swap;+;";
            return copy;
        } else {
            return static_cast<T>(lhs) + static_cast<T>(rhs);
        }
    }

    template <typename U, typename V>
    friend Tracked<T> operator*(const U& lhs, const V& rhs) {

        // both operands are tracked, so we can choose which side to use -> lhs bias
        if constexpr (std::is_same_v<U, Tracked<T>> && std::is_same_v<V, Tracked<T>>) {
            Tracked<T> copy;
            copy.data = lhs.data * rhs.data;

            // if the lhs has a source and the rhs is not 0 use the lhs as source, otherwise use the rhs as source
            if (lhs.location.at(".").is_valid()) {

                //optimization: delete the location, if the rhs is zero as it cannot be reversed
                if constexpr (std::is_arithmetic_v<T>) {
                    if (static_cast<T>(rhs) == 0) {
                        goto use_rhs;
                    }
                }

                copy.location = lhs.location;
                copy.location["."].expression += to_literal(static_cast<T>(rhs)) + ";*;";
                return copy;
            }

use_rhs:
            copy.location = rhs.location;
            copy.location["."].expression += to_literal(static_cast<T>(lhs)) + ";swap;*;";
            return copy;
        } else if constexpr (std::is_same_v<U, Tracked<T>>) {
            Tracked<T> copy = lhs;
            copy.data = lhs.data * static_cast<T>(rhs);
            copy.location["."].expression += to_literal(static_cast<T>(rhs)) + ";*;";

            //optimization: delete the location, if the rhs is zero as it cannot be reversed
            if constexpr (std::is_arithmetic_v<T>) {
                if (static_cast<T>(rhs) == 0) {
                    copy.location["."] = Location();
                }
            }

            return copy;
        } else if constexpr (std::is_same_v<V, Tracked<T>>) {
            Tracked<T> copy = rhs;
            copy.data = static_cast<T>(lhs) * rhs.data;
            copy.location["."].expression += to_literal(static_cast<T>(lhs)) + ";swap;*;";

            //optimization: delete the location, if the lhs is zero as it cannot be reversed
            if constexpr (std::is_arithmetic_v<T>) {
                if (static_cast<T>(lhs) == 0) {
                    copy.location["."] = Location();
                }
            }

            return copy;
        } else {
            return static_cast<T>(lhs) * static_cast<T>(rhs);
        }
    }

    template <typename U, typename V, std::enable_if_t<std::is_convertible_v<U,T> || std::is_convertible_v<V,T>, bool> = true>
    friend Tracked<T> operator-(const U& lhs, const V& rhs) {
        if constexpr (std::is_same_v<U, Tracked<T>>) {
            Tracked<T> copy = lhs;
            copy.data = lhs.data - static_cast<T>(rhs);
            copy.location["."].expression += to_literal(static_cast<T>(rhs)) + ";-;";
            return copy;
        } else if constexpr (std::is_same_v<V, Tracked<T>>) {
            Tracked<T> copy = rhs;
            copy.data = static_cast<T>(lhs) - rhs.data;
            copy.location["."].expression += to_literal(static_cast<T>(lhs)) + ";swap;-;";
            return copy;
        } else {
            return static_cast<T>(lhs) - static_cast<T>(rhs);
        }
    }

    template <typename U, typename V>
    friend Tracked<T> operator/(const U& lhs, const V& rhs) {
        if constexpr (std::is_same_v<U, Tracked<T>>) {
            Tracked<T> copy = lhs;
            copy.data = lhs.data / static_cast<T>(rhs);
            copy.location["."].expression += to_literal(static_cast<T>(rhs)) + ";/;";
            return copy;
        } else if constexpr (std::is_same_v<V, Tracked<T>>) {
            Tracked<T> copy = rhs;
            copy.data = static_cast<T>(lhs) / rhs.data;
            copy.location["."].expression += to_literal(static_cast<T>(lhs)) + ";swap;/;";
            return copy;
        } else {
            return static_cast<T>(lhs) / static_cast<T>(rhs);
        }
    }

    // methods for Tracked<vector<X>>

    template< class, class = void >
    struct get_value_type_member {
        using type = int;
    };

    template< class U >
    struct get_value_type_member<U, std::void_t<typename U::value_type>> {
        using type = typename U::value_type;
    };

    using value_type = typename get_value_type_member<T>::type;

    class Reference {
    public:
        explicit Reference(Tracked<std::vector<T>>* p, size_t position)
          : vec(p), pos(position) {}

        Reference operator= (const Tracked<T>& other) {
            vec->get_data()[pos] = other.get_data();
            vec->get_location() = add_locations(vec->get_location(), other.get_location(), std::to_string(pos));
            return *this;
        }

        operator Tracked<T> () const {
            return get();
        }

        const Tracked<T> get() const {
            return Tracked<T>(vec->get_data()[pos], location_slice(vec->get_location(), std::to_string(pos)));
        }

        Tracked<T> get() {
            return Tracked<T>(vec->get_data()[pos], location_slice(vec->get_location(), std::to_string(pos)));
        }

        const T get_data() const {
            return get().get_data();
        }

        const LocationMap get_location() const {
            return get().get_location();
        }

    private:
        Tracked<std::vector<T>>* vec;
        size_t pos;
    };

private:

    class Iterator : public boost::iterator_facade<Iterator, value_type, std::random_access_iterator_tag, typename Tracked<value_type>::Reference> {
    public:
        Iterator()
          : vec(nullptr) {}

        explicit Iterator(Tracked<T>* p, size_t position)
          : vec(p), pos(position) {}

     private:
        friend class boost::iterator_core_access;

        void increment() { pos++; }
        void decrement() { pos--; }

        void advance(int n) {
            pos += n;
        }

        ptrdiff_t distance_to(const Iterator& other) const {
            return other.pos - pos;
        }

        bool equal(const Iterator& other) const {
            return pos == other.pos;
        }

        typename Tracked<value_type>::Reference dereference() const {
            return (*vec)[pos];
        }

        Tracked<T>* vec;
        size_t pos;
    };

    class ConstIterator : public boost::iterator_facade<Iterator, value_type, std::random_access_iterator_tag, typename Tracked<value_type>::Reference> {
    public:
        ConstIterator()
          : vec(nullptr) {}

        explicit ConstIterator(const Tracked<T>* p, size_t position)
          : vec(p), pos(position) {}

     private:
        friend class boost::iterator_core_access;

        void increment() { pos++; }
        void decrement() { pos--; }

        void advance(int n) {
            pos += n;
        }

        ptrdiff_t distance_to(const ConstIterator& other) const {
            return other.pos - pos;
        }

        bool equal(const ConstIterator& other) const {
            return pos == other.pos;
        }

        const typename Tracked<value_type>::Reference dereference() const {
            return (*vec)[pos];
        }

        const Tracked<T>* vec;
        size_t pos;
    };

public:
    size_t size() const {
        static_assert(is_vector<T>::value, "size can only be called on a Tracked<vector<T>>");

        return get_data().size();
    }

    template<typename U>
    void push_back(const U& value) {
        static_assert(is_vector<T>::value, "push_back can only be called on a Tracked<vector<T>>");
        static_assert(is_tracked_v<U> || std::is_convertible_v<U, typename T::value_type>, "The argument to push_back must be of type value_type or Tracked<value_type>");

        if constexpr (is_tracked_v<U>) {
            get_data().push_back(value.get_data());
            location = add_locations(location, value.get_location(), std::to_string(get_data().size() - 1));

            return;
        }

        if constexpr (std::is_convertible_v<U, typename T::value_type>) {
            get_data().push_back(value);

            return;
        }
    }

    void pop_back() {
        static_assert(is_vector<T>::value, "pop_back can only be called on a Tracked<vector<T>>");

        get_data().pop_back();

        location = remove_locations(location, std::to_string(get_data().size()));
    }

    void clear() {
        static_assert(is_vector<T>::value, "clear can only be called on a Tracked<vector<T>>");

        get_data().clear();

        location = location_slice(location, ".");
    }

    typename Tracked<value_type>::Reference front() {
        static_assert(is_vector<T>::value, "front can only be called on a Tracked<vector<T>>");

        return (*this)[0];
    }

    const typename Tracked<value_type>::Reference front() const {
        static_assert(is_vector<T>::value, "front can only be called on a Tracked<vector<T>>");

        return (*this)[0];
    }

    typename Tracked<value_type>::Reference back() {
        static_assert(is_vector<T>::value, "back can only be called on a Tracked<vector<T>>");

        return (*this)[get_data().size()-1];
    }

    const typename Tracked<value_type>::Reference back() const {
        static_assert(is_vector<T>::value, "back can only be called on a Tracked<vector<T>>");

        return (*this)[get_data().size()-1];
    }

    typename Tracked<value_type>::Reference operator[] (size_t pos) {
        static_assert(is_vector<T>::value, "operator[] can only be called on a Tracked<vector<T>>");

        return typename Tracked<value_type>::Reference(this, pos);
    }

    const typename Tracked<value_type>::Reference operator[] (size_t pos) const {
        static_assert(is_vector<T>::value, "operator[] can only be called on a Tracked<vector<T>>");

        return typename Tracked<value_type>::Reference(this, pos);
    }

    Iterator begin() {
        static_assert(is_vector<T>::value, "begin can only be called on a Tracked<vector<T>>");

        return Iterator(this, 0);
    }

    Iterator end() {
        static_assert(is_vector<T>::value, "end can only be called on a Tracked<vector<T>>");

        return Iterator(this, get_data().size());
    }

    ConstIterator begin() const {
        static_assert(is_vector<T>::value, "begin can only be called on a Tracked<vector<T>>");

        return ConstIterator(this, 0);
    }

    ConstIterator end() const {
        static_assert(is_vector<T>::value, "end can only be called on a Tracked<vector<T>>");

        return ConstIterator(this, get_data().size());
    }

private:

    T data;
    LocationMap location;
};

#define SET_FIELD(Obj, Field, Value) (Obj) = (Obj).set_field((Obj).get_data().Field, #Field, make_tracked(Value))

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

template <typename T>
Tracked<T> sin(const Tracked<T>& v) {
    return v.sin();
}

template <typename T>
Tracked<T> cos(const Tracked<T>& v) {
    return v.cos();
}

template <typename T, std::enable_if_t<!is_tracked_v<T>, bool> = true>
Tracked<T> make_tracked(T data, Location loc = Location()) {
    return Tracked<T>(data, loc);
}

template <typename T, std::enable_if_t<is_tracked_v<T>, bool> = true>
T make_tracked(T data) {
    return data;
}

#endif // TRACKED_H
