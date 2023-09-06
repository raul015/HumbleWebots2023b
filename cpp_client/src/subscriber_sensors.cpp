#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/msg/joint_position.hpp"
#include <unistd.h>  // per time_sleep


using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
  public:

    const double PI = 3.1415926535897932384626433832795028841971693993751058209;


    rclcpp::Subscription<fp_core_msgs::msg::JointPosition>::SharedPtr subscription_;

    SensorSubscriber(): Node("SensorSub_Cpp")
    {

      subscription_ = this->create_subscription<fp_core_msgs::msg::JointPosition>(
      "Sensors_values", 10, std::bind(&SensorSubscriber::topic_callback, this, _1));
    }

  private:

    void topic_callback(const fp_core_msgs::msg::JointPosition::SharedPtr msg) const
    {
        for(int i = 0; i<8 ; i++){

            RCLCPP_INFO(this->get_logger(), "%s :  [rad] %f [grad] %f ", msg.get()->name[i].c_str(), msg.get()->position[i],(msg.get()->position[i])*(180/PI) );
        }
        RCLCPP_INFO(this->get_logger(), "");

        sleep(0.8);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}