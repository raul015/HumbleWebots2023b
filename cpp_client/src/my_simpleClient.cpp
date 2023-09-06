#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// INIZIO CREAZIONE CLASSE PER IL NODO 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node_SimpleMoveJoint");

  rclcpp::Client<fp_core_msgs::srv::MoveJoint>::SharedPtr client =
    node->create_client<fp_core_msgs::srv::MoveJoint>("MoveJoint");
  auto request = std::make_shared<fp_core_msgs::srv::MoveJoint::Request>();


    // Primo comando da inviare 

//   std::vector<int64_t> vector_id_1 {1,2,3,4,5,6,7,8};
//   std::vector<double> vector_position_1 {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
//   double velocita_1 = 0.1;
//   double acceleratione_1 = 2.0;
//   bool block_1 = false;
//   bool relative_1 = false;

  request->actuator_ids = std::vector<int64_t>{1,2,3,4,5,6,7,8};
  request->position = std::vector<double> {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  request->velocity = 0.1;
  request->acceleration = 2.0;
  request->block = false;
  request->relative = false;


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  //Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==rclcpp::FutureReturnCode::SUCCESS)
    {                                       
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), result.get()->message.c_str());
  } 
  else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  } 


  rclcpp::shutdown();
  return 0;
}

