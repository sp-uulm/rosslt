#include "rosslt/trackingnode.h"

Location::Location(std::string source_node, unsigned int location_id)
    : source_node(std::move(source_node)), location_id(location_id)
{
}

Location::Location(const rosslt_msgs::msg::Location& loc) {
    source_node = loc.source_node;
    location_id = loc.location_id;
    expression = loc.expression;
}

Location::operator rosslt_msgs::msg::Location () const {
    rosslt_msgs::msg::Location loc;
    loc.set__source_node(source_node);
    loc.set__location_id(location_id);
    loc.set__expression(expression);
    return loc;
}

TrackingNode::TrackingNode(const std::string& name) : Node(name), loc_mgr(*this) {

}

std::string inverse_op(const std::string& op) {
    if (op == "+") return "-";
    if (op == "-") return "+";
    if (op == "*") return "/";
    if (op == "/") return "*";
    if (op == "swap") return "swap";
    if (op == "sin") return "asin";
    if (op == "cos") return "acos";
    if (op == "asin") return "sin";
    if (op == "acos") return "cos";
    throw std::invalid_argument("no valid operator '" + op + "'");
}

int is_operator(const std::string& token) {
    if (token == "+") return 2;
    if (token == "-") return 2;
    if (token == "*") return 2;
    if (token == "/") return 2;
    if (token == "sin") return 1;
    if (token == "cos") return 1;
    if (token == "asin") return 1;
    if (token == "acos") return 1;
    if (token == "swap") return 1;

    return 0;
}

std::string reverseExpression(const std::vector<std::string>& v, int pos, bool swapped = false) {
    if (pos >= v.size())
        return "";

    std::stringstream result;

    int n = 0;
    int i = pos;
    for(;;) {
        n -= is_operator(v[i]);
        if (n < 0)
            break;
        n++;
        i++;
    }

    //the operator at i depends on the source and must be reversed, everything before i can be copied to the result

    for (int j = 0; j < i; ++j) {
        result << v[j] << ";";
    }

    // if the operator is non-commutative and rhs (swapped) it must be kept otherwise, the swap is removed and the operator reversed
    // Example:
    // 3 2;swap;- -> -1 and -1 2;swap;- -> 3 but
    // 3 2;swap;+ ->  5 and  5 2;- -> 3
    bool op_is_rhs = swapped || (i-1 >= 0 && v[i-1] == "swap");
    bool op_is_commutative = v[i] == "+" || v[i] == "*";

    if (op_is_commutative) {
        result << (op_is_rhs ? "swap;" + inverse_op(v[i]) : inverse_op(v[i])) << ";";
    } else {
        result << (op_is_rhs ? v[i] : inverse_op(v[i])) << ";";
    }

    return reverseExpression(v, i+1, (v[i] == "swap")) + result.str();
}

std::string reverseExpression(const std::string& exp) {
    std::vector<std::string> v;
    size_t end = exp.find(';');
    for (size_t start = 0; end != std::string::npos; end = exp.find(';', start)) {
        std::string token = exp.substr(start, end-start);
        start = end+1;

        v.push_back(token);
    }

    return reverseExpression(v, 0);
}


