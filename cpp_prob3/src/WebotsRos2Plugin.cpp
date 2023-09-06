#include "cpp_prob3/WebotsRos2Plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <cstdio>

using std::placeholders::_1;

namespace webots_ros2_plugin
{
  void WebotsRos2Plugin::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
  {
    // This method is executed once the plugin is loaded by the `webots_ros2_driver` package.
    // The `webots_ros2_driver::WebotsNode` inherits the `rclcpp::Node`, so you have all methods available from there.
    // In addition, from the `webots_ros2_driver::WebotsNode` instance you can also get a `webots::Robot` reference (`node.robot()`).
    
    // Print a simple message to see if your plugin has been loaded correctly:

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello from WebotsRos2PluginExample!");
    node_ = node;

  }
  void WebotsRos2Plugin::step()
  {


  }

  bool WebotsRos2Plugin::preStep() {

    return true;
  }


}

// The class has to be exported with `PLUGINLIB_EXPORT_CLASS` macro.
// The first argument is the name of your class, while the second is always `webots_ros2_driver::PluginInterface`
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_ros2_plugin::WebotsRos2Plugin, webots_ros2_driver::PluginInterface)