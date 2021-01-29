#include <gtest/gtest.h>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "rosslt/tracked.h"
#include "rosslt_msgs/msg/int32_tracked.hpp"
#include "rosslt_msgs/msg/marker_tracked.hpp"

TEST(TestTracked, ConstructTrackedInt) {
    Tracked<int> test1(5);

    EXPECT_EQ(static_cast<int>(test1), 5);
    EXPECT_EQ(test1.get_data(), 5);
    EXPECT_FALSE(test1.get_location().at(".").is_valid());
}

TEST(TestTracked, ConstructTrackedIntFromMessage) {
    rosslt_msgs::msg::Int32Tracked msg;
    msg.data.data = 7;

    Tracked<std_msgs::msg::Int32> test1(msg);

    EXPECT_EQ(test1.get_data().data, 7);
    EXPECT_FALSE(test1.get_location()["."].is_valid());
}

TEST(TestTracked, Location) {
    Location loc1;

    EXPECT_FALSE(loc1.is_valid());

    Location loc2("mynode", 42);

    EXPECT_TRUE(loc2.is_valid());

    rosslt_msgs::msg::Location msg = loc2;

    EXPECT_EQ(msg.location_id, 42);
    EXPECT_EQ(msg.source_node, "mynode");

    Location loc3(msg);

    EXPECT_EQ(loc3.location_id, 42);
    EXPECT_EQ(loc3.source_node, "mynode");
}

TEST(TestTracked, TrackedWithLocation) {
    Location loc("mynode", 42);

    EXPECT_TRUE(loc.is_valid());

    Tracked<double> a(2.0, loc);

    EXPECT_EQ(a.get_location()["."].location_id, 42);

    auto b = a;

    EXPECT_EQ(b.get_location()["."].location_id, 42);
}

TEST(TestTracked, ArithmeticDoublePlus) {
    Tracked<double> a = 5.0;

    EXPECT_EQ(static_cast<double>(a), 5.0);

    auto b = a + 2.0;

    EXPECT_EQ(static_cast<double>(b), 7.0);
    EXPECT_EQ(static_cast<double>(a), 5.0);

    auto c = 3.0 + a;

    EXPECT_EQ(static_cast<double>(c), 8.0);
    EXPECT_EQ(c.get_location()["."].expression, "3.000000;swap;+;");

    auto d = b + c;

    EXPECT_EQ(static_cast<double>(d), 15.0);
    EXPECT_EQ(d.get_location()["."].expression, "2.000000;+;8.000000;+;");
}

TEST(TestTracked, ArithmeticString) {
    Tracked<std::string> a = "Hallo";

    EXPECT_EQ(a.get_data(), "Hallo");

    auto b = "Oh, " + a + " Welt";

    EXPECT_EQ(b.get_data(), "Oh, Hallo Welt");
}

TEST(TestTracked, ArithmeticIntMinus) {
    Tracked<int> a = 4;

    EXPECT_EQ(a.get_data(), 4);

    auto b = a - 3;

    EXPECT_EQ(b.get_data(), 1);

    auto c = 3 - a;

    EXPECT_EQ(c.get_data(), -1);

    auto d = c - b;

    EXPECT_EQ(d.get_data(), -2);
    EXPECT_EQ(d.get_location()["."].expression, "3;swap;-;1;-;");
}

TEST(TestTracked, ArithmeticIntMultiply) {
    Tracked<int> a (4, Location("foo", 27));

    EXPECT_EQ(a.get_data(), 4);

    auto b = a * 3;

    EXPECT_EQ(b.get_data(), 12);

    auto c = 2 * b;

    EXPECT_EQ(c.get_data(), 24);

    auto d = c * a;

    EXPECT_EQ(d.get_data(), 96);
    EXPECT_EQ(d.get_location()["."].expression, "3;*;2;swap;*;4;*;");
}

TEST(TestTracked, ArithmeticIntMultiplyOptimization) {
    Tracked<int> a (4, Location("foo", 24));
    Tracked<int> x (0, Location("bar", 24));

    EXPECT_EQ(a.get_data(), 4);

    auto b = a * 0;

    EXPECT_EQ(b.get_data(), 0);
    EXPECT_FALSE(b.get_location()["."].is_valid());

    auto c = 0 * a;

    EXPECT_EQ(c.get_data(), 0);
    EXPECT_FALSE(c.get_location()["."].is_valid());

    auto d = x * a;

    EXPECT_EQ(d.get_data(), 0);
    EXPECT_TRUE(d.get_location()["."].is_valid());
    EXPECT_EQ(d.get_location()["."].source_node, "bar");

    auto e = a * x;

    EXPECT_EQ(e.get_data(), 0);
    EXPECT_TRUE(e.get_location()["."].is_valid());
    EXPECT_EQ(e.get_location()["."].source_node, "bar");
}

TEST(TestTracked, ArithmeticIntDiv) {
    Tracked<int> a = 4;

    EXPECT_EQ(a.get_data(), 4);

    auto b = a / 2;

    EXPECT_EQ(b.get_data(), 2);

    auto c = 19 / a;

    EXPECT_EQ(c.get_data(), 4);

    auto d = c / b;

    EXPECT_EQ(d.get_data(), 2);
    EXPECT_EQ(d.get_location()["."].expression, "19;swap;/;2;/;");
}

TEST(TestTracked, Trigonometry) {
    Tracked<double> a = M_PI;

    auto b = sin(a / 6.0);

    EXPECT_DOUBLE_EQ(b.get_data(), 0.5);
    EXPECT_EQ(b.get_location()["."].expression, "6.000000;/;sin;");

    auto c = cos(a / 3.0);

    EXPECT_DOUBLE_EQ(c.get_data(), 0.5);
    EXPECT_EQ(c.get_location()["."].expression, "3.000000;/;cos;");
}

TEST(TestTracked, ApplyExpression) {
    // int
    EXPECT_EQ(applyExpression(5, ""), 5);
    EXPECT_EQ(applyExpression(5, "\"1\";+;"), 6);

    EXPECT_EQ(applyExpression(4, "2;+;"), 6);
    EXPECT_EQ(applyExpression(0, "2;-;"), -2);
    EXPECT_EQ(applyExpression(3, "4;*;"), 12);
    EXPECT_EQ(applyExpression(15, "5;/;"), 3);

    EXPECT_EQ(applyExpression(3, "18;swap;/;"), 6);
    EXPECT_EQ(applyExpression(0, "2;swap;-;"), 2);

    EXPECT_EQ(applyExpression(4, "1;1;1;1;+;+;+;+;"), 8);
    EXPECT_EQ(applyExpression(1, "2;3;*;4;2;/;+;-;"), -7);

    EXPECT_EQ(applyExpression(1, "sin;"), 0);
    EXPECT_EQ(applyExpression(0, "cos;"), 1);

    // double
    EXPECT_DOUBLE_EQ(applyExpression(5.0, ""), 5);
    EXPECT_DOUBLE_EQ(applyExpression(5.0, "\"1.23\";+;"), 6.23);

    EXPECT_DOUBLE_EQ(applyExpression(4.0, "2.5;+;"), 6.5);
    EXPECT_DOUBLE_EQ(applyExpression(0.0, "2;-;"), -2);
    EXPECT_DOUBLE_EQ(applyExpression(3.0, "4;*;"), 12);
    EXPECT_DOUBLE_EQ(applyExpression(15.0, "6;/;"), 15.0/6.0);

    EXPECT_DOUBLE_EQ(applyExpression(3.0, "18;swap;/;"), 6);
    EXPECT_DOUBLE_EQ(applyExpression(0.0, "2;swap;-;"), 2);
    EXPECT_DOUBLE_EQ(applyExpression(6.0, "4;swap;*;"), 24.0);
    EXPECT_DOUBLE_EQ(applyExpression(2.0, "3;swap;+;"), 5.0);

    EXPECT_DOUBLE_EQ(applyExpression(4.0, "1;1;1;1;+;+;+;+;"), 8);
    EXPECT_DOUBLE_EQ(applyExpression(1.0, "2;3;*;4;2;/;+;-;"), -7);

    EXPECT_DOUBLE_EQ(applyExpression(1.0, "sin;"), sin(1.0));
    EXPECT_DOUBLE_EQ(applyExpression(4.0, "cos;"), cos(4.0));

    EXPECT_DOUBLE_EQ(applyExpression(0.4, "asin;"), asin(0.4));
    EXPECT_DOUBLE_EQ(applyExpression(0.3, "acos;"), acos(0.3));

    // string
    EXPECT_EQ(applyExpression<std::string>("Hallo", ""), "Hallo");
    EXPECT_EQ(applyExpression<std::string>("", "\"Hallo;Hallo;Hallo\";+;"), "Hallo;Hallo;Hallo");
    EXPECT_EQ(applyExpression<std::string>("Hallo", "\" Welt\";+;"), "Hallo Welt");
    EXPECT_EQ(applyExpression<std::string>("Hallo", "\" Welt\";swap;+;"), " WeltHallo");
    EXPECT_EQ(applyExpression<std::string>("Hallo Welt", "\" Welt\";-;"), "Hallo");
}

TEST(TestTracked, ReverseExpression) {
    EXPECT_EQ(reverseExpression(""), "");

    EXPECT_DOUBLE_EQ(applyExpression(5.0, reverseExpression("")), 5);
    EXPECT_DOUBLE_EQ(applyExpression(6.23, reverseExpression("\"1.23\";+;")), 5.0);

    EXPECT_DOUBLE_EQ(applyExpression(6.5, reverseExpression("2.5;+;")), 4.0);
    EXPECT_DOUBLE_EQ(applyExpression(-2.0, reverseExpression("2;-;")), 0);
    EXPECT_DOUBLE_EQ(applyExpression(12.0, reverseExpression("4;*;")), 3.0);
    EXPECT_DOUBLE_EQ(applyExpression(15.0/6.0, reverseExpression("6;/;")), 15.0);

    EXPECT_DOUBLE_EQ(applyExpression(6.0, reverseExpression("18;swap;/;")), 3.0);
    EXPECT_DOUBLE_EQ(applyExpression(2.0, reverseExpression("2;swap;-;")), 0.0);
    EXPECT_DOUBLE_EQ(applyExpression(24.0, reverseExpression("4;swap;*;")), 6.0);
    EXPECT_DOUBLE_EQ(applyExpression(5.0, reverseExpression("3;swap;+;")), 2.0);

    EXPECT_DOUBLE_EQ(applyExpression(8.0, reverseExpression("1;1;1;1;+;+;+;+;")), 4.0);
    EXPECT_DOUBLE_EQ(applyExpression(-7.0, reverseExpression("2;3;*;4;2;/;+;-;")), 1.0);

    EXPECT_DOUBLE_EQ(applyExpression(sin(1.0), reverseExpression("sin;")), 1.0);
    EXPECT_DOUBLE_EQ(applyExpression(cos(2.0), reverseExpression("cos;")), 2.0);
}

TEST(TestTracked, ReverseExpression2) {
    EXPECT_DOUBLE_EQ(applyExpression(1.2, reverseExpression("2.000000;+;10.000000;/;")), 10.0);
}

TEST(TestTracked, MakeTracked) {
    EXPECT_EQ(make_tracked(5).get_data(), 5);
    EXPECT_EQ(make_tracked(make_tracked(5)).get_data(), 5);

    EXPECT_TRUE(make_tracked(false, Location("foo", 42)).get_location().at(".").is_valid());
    EXPECT_FALSE(make_tracked("Hallo").get_location().at(".").is_valid());
}

TEST(TestTracked, IsTracked) {
    EXPECT_TRUE(is_tracked<Tracked<int>>::value);
    EXPECT_FALSE(is_tracked<int>::value);

    EXPECT_TRUE(is_tracked_v<Tracked<double>>);
    EXPECT_FALSE(is_tracked_v<double>);
}

TEST(TestTracked, SetPrimitiveField) {
    Tracked<visualization_msgs::msg::Marker> marker;
    Location loc("foo", 22);

    SET_FIELD(marker, id, make_tracked(42, loc));

    EXPECT_EQ(marker.get_location().at("id").location_id, 22);

    EXPECT_EQ(marker.get_data().id, 42);

    Tracked<int> id = GET_FIELD(marker, id);

    EXPECT_EQ(id.get_location().at(".").location_id, 22);
    EXPECT_EQ(id.get_data(), 42);

    SET_FIELD(marker, id, 25);

    id = GET_FIELD(marker, id);

    EXPECT_FALSE(id.get_location().at(".").is_valid());
    EXPECT_EQ(id.get_data(), 25);

    //test offsets after array

    SET_FIELD(marker, text, std::string("test"));

    EXPECT_EQ(marker.get_data().text, "test");
}

TEST(TestTracked, SetComplexField) {
    Tracked<visualization_msgs::msg::Marker> marker;
    Location loc("foo", 22);
    std_msgs::msg::Header head;

    head.frame_id = "bar";
    head.stamp = rclcpp::Time(12345);

    SET_FIELD(marker, header, head);

    EXPECT_EQ(marker.get_data().header.frame_id, "bar");
    EXPECT_FALSE(marker.get_location().at(".").is_valid());

    Tracked<std_msgs::msg::Header> header = GET_FIELD(marker, header);

    EXPECT_FALSE(header.get_location().at(".").is_valid());
    EXPECT_EQ(header.get_data().stamp, head.stamp);

    head.frame_id = "baz";
    head.stamp = rclcpp::Time(10);

    SET_FIELD(marker, header, make_tracked(head, loc));

    EXPECT_EQ(marker.get_data().header.frame_id, "baz");
    EXPECT_FALSE(marker.get_location().at(".").is_valid());

    header = GET_FIELD(marker, header);

    EXPECT_TRUE(header.get_location().at(".").is_valid());
    EXPECT_EQ(header.get_data().stamp, head.stamp);
}

TEST(TestTracked, VectorMethods) {
    Tracked<std::vector<int>> vec;
    Location loc("foo", 22);
    Location loc2("bar", 23);

    EXPECT_EQ(vec.size(), 0);

    vec.push_back(42);
    vec.push_back(make_tracked(7, loc));
    vec.push_back(-7);
    vec.push_back(make_tracked(15, loc2));

    EXPECT_EQ(vec.size(), 4);

    EXPECT_EQ(vec[0].get_data(), 42);
    EXPECT_EQ(vec[1].get_data(), 7);
    EXPECT_EQ(vec.front().get_data(), 42);
    EXPECT_EQ(vec.back().get_data(), 15);

    EXPECT_FALSE(vec[0].get_location().at(".").is_valid());
    EXPECT_EQ(vec[1].get_location().at(".").location_id, 22);

    vec.pop_back();

    EXPECT_EQ(vec.size(), 3);
    EXPECT_EQ(vec.back().get_data(), -7);
}

TEST(TestTracked, VectorIterator) {
    Tracked<std::vector<int>> vec;
    Location loc("foo", 22);
    Location loc2("bar", 23);

    EXPECT_EQ(vec.size(), 0);

    vec.push_back(42);
    vec.push_back(make_tracked(7, loc));
    vec.push_back(-7);
    vec.push_back(make_tracked(15, loc2));

    EXPECT_EQ(vec.size(), 4);

    for (Tracked<int>::Reference i : vec) {
        i = make_tracked(111, loc2);
    }

    EXPECT_TRUE(std::all_of(vec.begin(), vec.end(), [&](Tracked<int> i){ return i.get_data() == 111 && i.get_location().at(".") == loc2;}));

    vec.clear();

    EXPECT_EQ(vec.size(), 0);
}

TEST(TestTracked, SetArrayField) {
    Tracked<visualization_msgs::msg::Marker> marker;
    Location loc("foo", 22);
    std_msgs::msg::ColorRGBA col, col2;

    col.r = 0.5;
    col2.r = 0.2;

    Tracked<std::vector<std_msgs::msg::ColorRGBA>> colors = GET_FIELD(marker, colors);

    colors.push_back(col);
    colors.push_back(make_tracked(col2, loc));

    EXPECT_FLOAT_EQ(colors[0].get_data().r, 0.5);
    EXPECT_FLOAT_EQ(colors[1].get_data().r, 0.2);

    SET_FIELD(marker, colors, colors);

    EXPECT_EQ(marker.get_data().colors.size(), 2);
    EXPECT_FLOAT_EQ(marker.get_data().colors[1].r, 0.2);

    EXPECT_FALSE(GET_FIELD(marker, colors).get_location().at(".").is_valid());
    EXPECT_FALSE(GET_FIELD(marker, colors)[0].get_location().at(".").is_valid());
    EXPECT_TRUE(GET_FIELD(marker, colors)[1].get_location().at(".").is_valid());
    EXPECT_EQ(GET_FIELD(marker, colors)[1].get_location().at(".").location_id, 22);
}
