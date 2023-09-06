#ifndef WEBOTS_ROS2_PLUGIN_HPP
#define WEBOTS_ROS2_PLUGIN_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace webots_ros2_plugin
{
  class WebotsRos2Plugin : public webots_ros2_driver::PluginInterface
  {
  public:
    // Your plugin has to override step() and init() methods
    WebotsRos2Plugin();
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool preStep();

    webots_ros2_driver::WebotsNode *node_;

    double publish_timestep;
    bool mAlwaysOn;
    int mPublishTimestepSyncedMs;
    double last_update;

  };
}
#endif