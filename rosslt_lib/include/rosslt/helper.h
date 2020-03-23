#ifndef HELPER_H
#define HELPER_H

#include <type_traits>
#include <string>
#include <cstdlib>
#include <vector>

template<typename To, typename From>
To msg2data(const From& from) {
    return from;
}

template<typename To, typename From>
To data2msg(const From& from) {
    To msg;
    msg = from;
    return msg;
}

template <typename T>
T sto(const std::string& s) {
    if constexpr (std::is_integral_v<T>) {
        return stoi(s);
    } else if constexpr (std::is_floating_point_v<T>) {
        return stod(s);
    } else {
        return s;
    }
}

template <>
bool sto<bool>(const std::string& s) {
    return s == "1" || s == "true" || s == "True";
}

template <typename T>
T inv_plus(const T& lhs, const T& rhs) {
    static_assert (std::is_arithmetic_v<T>, "inv_plus must be specialized for non-arithmetic types");
    return lhs - rhs;
}

template <typename T>
T inv_minus(const T& lhs, const T& rhs) {
    static_assert (std::is_arithmetic_v<T>, "inv_minus must be specialized for non-arithmetic types");
    return lhs + rhs;
}

template <typename T>
T inv_mul(const T& lhs, const T& rhs) {
    static_assert (std::is_arithmetic_v<T>, "inv_mul must be specialized for non-arithmetic types");
    return lhs / rhs;
}

template <typename T>
T inv_div(const T& lhs, const T& rhs) {
    static_assert (std::is_arithmetic_v<T>, "inv_div must be specialized for non-arithmetic types");
    return lhs * rhs;
}


template <typename T>
T applyExpression(T val, const std::string& exp_string) {
    if constexpr (!std::is_arithmetic_v<T>) {
        return val;
    } else {
        std::vector<T> stack;
        stack.push_back(val);
        size_t end = exp_string.find(';');
        for (size_t start = 0; end != std::string::npos; end = exp_string.find(';', start)) {
            std::string token = exp_string.substr(start, end-start);
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
        return stack.back();
    }
}

std::string reverseExpression(const std::string& exp);

#endif // HELPER_H
