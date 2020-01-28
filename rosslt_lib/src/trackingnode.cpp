#include "rosslt/trackingnode.h"

TrackingNode::TrackingNode(const std::string& name) : Node(name) {

}

std::string inverse_op(const std::string& op) {
    if (op == "+") return "-";
    if (op == "-") return "+";
    if (op == "*") return "/";
    if (op == "/") return "*";
    throw std::invalid_argument("no valid operator '" + op + "'");
}

std::string reverseExpression(const std::string& exp) {
    std::cout << "reverse Expression: '" << exp << "'" << std::endl;
    std::vector<std::string> v;
    size_t end = exp.find(';');
    for (size_t start = 0; end != std::string::npos; end = exp.find(';', start)) {
        std::string token = exp.substr(start, end-start);
        start = end+1;

        v.push_back(token);
    }

    std::stringstream result;

    for (int i = static_cast<int>(v.size())-2; i >= 0; i-=2) {
        result << v[i] << ";" << inverse_op(v[i+1]) << ";";
    }

    std::cout << "result: '" << result.str() << "'" << std::endl;
    return result.str();
}
