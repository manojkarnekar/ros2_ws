#include <rclcpp/rclcpp.hpp>

class MainNode : public rclcpp::Node {
public:
  MainNode() : rclcpp::Node("node", rclcpp::NodeOptions()) {

    // example: declare parameters, default value given
    declare_parameter("param0", 1);
    declare_parameter("param1", 0.1);
    declare_parameter("param2", "default");
    declare_parameter("param3.weight", 1);
    declare_parameter("param3.name", "default");
    declare_parameter("param4", false);
    // example: declare a variable when declaring a parameter
    declare_parameter("param5", std::vector<bool>(3, false));
    declare_parameter("param6", std::vector<int64_t>(4, 1));
    declare_parameter("param7", std::vector<double>(4, 0.1));
    declare_parameter("param8", std::vector<std::string>(5, "default"));

    // Get parameter values one by one
    auto p0 = get_parameter("param0").as_int();
    auto p1 = get_parameter("param1").as_double();
    auto p2 = get_parameter("param2").as_string();
    auto p3weight = get_parameter("param3.weight").as_int();
    auto p3name = get_parameter("param3.name").as_string();
    auto p4 = get_parameter("param4").as_bool();
    auto p5 = get_parameter("param5").as_bool_array();
    auto p6 = get_parameter("param6").as_integer_array();
    auto p7 = get_parameter("param7").as_double_array();
    auto p8 = get_parameter("param8").as_string_array();

    // Print parameters
    RCLCPP_INFO(get_logger(), "Integer parameter: %ld", p0);
    RCLCPP_INFO(get_logger(), "Double parameter: %f", p1);
    RCLCPP_INFO(get_logger(), "String parameter: %s", p2.c_str());
    RCLCPP_INFO(get_logger(), "Nested integer parameter: %ld", p3weight);
    RCLCPP_INFO(get_logger(), "Nested string parameter: %s", p3name.c_str());
    RCLCPP_INFO(get_logger(), "Boolean parameter: %d", p4);
    RCLCPP_INFO(get_logger(), "Boolean vector parameter [0]: %d",
                static_cast<int>(p5[0]));
    RCLCPP_INFO(get_logger(), "Integer vector parameter [0]: %d",
                static_cast<int>(p6[0]));
    RCLCPP_INFO(get_logger(), "Double vector parameter [0]: %f",
                static_cast<double>(p7[0]));
    RCLCPP_INFO(get_logger(), "String vector parameter [0]: %s", p8[0].c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
}