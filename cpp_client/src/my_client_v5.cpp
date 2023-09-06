#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"
#include "fp_core_msgs/msg/joint_position.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include<thread>
#include <memory>
#include <unistd.h>  // per time_sleep

const double PI = 3.1415926535897932384626433832795028841971693993751058209;

/*
msg.get()->name[i].c_str(), msg.get()->position[i],(msg.get()->position[i])*(180/PI)
*/

class JointPosition_class{ 

  public:


  std::vector<std::string> name = {
    
    "sensor 1",
    "sensor 2",
    "sensor 3",
    "sensor 4",
    "sensor 5",
    "sensor 6",
    "sensor 7",
    "sensor 8"
  };

  std::vector<double> position = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  };

  JointPosition_class(){}
};


// std::shared_ptr<JointPosition_class> class_posizione;

JointPosition_class class_posizione;

/*

  Idea, utilizzare un subscriber che continui a leggere i valori 
  pubblicati dal nodo publisher nel topic 'Sensors_values'

*/


using namespace std::chrono_literals;

class MoveJoint_class{

    public:

    MoveJoint_class(  // Ho impostato i valori di default
            std::vector<int64_t> actuator_ids,
            std::vector<double> position,
            double velocity,
            double acceleration = 2.0, 
            bool block = false,
            bool relative = false){

    this->actuator_ids = actuator_ids;
    this->position = position;
    this->velocity = velocity;
    this->acceleration = acceleration;
    this->block = block;
    this->relative = relative;

    }


    std::vector<int64_t> getActuatorIds(){
        return this->actuator_ids;
    }

    std::vector<double> getPosition(){
        return this->position;
    }

    double getVelocity(){
        return this->velocity;
    }

    double getAcceleration(){
        return this->acceleration;
    }

    bool getBlock(){
        return this->block;
    }

    bool getRelative(){
        return this->relative;
    }
    std::vector<int64_t> actuator_ids;
    std::vector<double> position;
    double velocity;
    double acceleration;
    bool block;
    bool relative;

    private:

};


// INIZIO CREAZIONE CLASSE PER IL NODO 

class ClientProb3 : public rclcpp::Node

{
  public:
    
  rclcpp::Client<fp_core_msgs::srv::MoveJoint>::SharedPtr cliente;

  rclcpp::Subscription<fp_core_msgs::msg::JointPosition>::SharedPtr subscription_;

    ClientProb3() : Node("Client_Prob3_cpp"){
    // using std::placeholders::_1;

    cliente = this->create_client<fp_core_msgs::srv::MoveJoint>("MoveJoint");
    subscription_ = this->create_subscription<fp_core_msgs::msg::JointPosition>("Sensors_values", 10, std::bind(&ClientProb3::topic_callback, this, std::placeholders::_1));
      while (! cliente->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "rclcpp::ok() = false ");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "servizio non disponibile,aspetta ancora");
      }

    }


  auto sendRequest(MoveJoint_class move){

    auto request = std::make_shared<fp_core_msgs::srv::MoveJoint::Request>();

    request->actuator_ids = move.getActuatorIds();
    request->position = move.getPosition();
    request->velocity = move.getVelocity();
    request->acceleration = move.getAcceleration();
    request->block = move.getBlock();
    request->relative = move.getRelative();
  
    auto future = cliente->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),future) ==rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Il servizio è stato chiamato correttamente");
    } 
    else{
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nella chiamata del servizio");
      } 
    // rclcpp::spin_until_future_complete(cliente, future);
    return future;
  }

  auto send_request_MoveJoint(MoveJoint_class oggetto_MoveJoint){

    /* Solo se in input ho i gradi 
    
    for(int i =0 ; i<oggetto_MoveJoint.position.size();i++){
      
      oggetto_MoveJoint.position[i] = oggetto_MoveJoint.position[i]*(PI/180);
    }
    */
    auto ritorno = sendRequest(oggetto_MoveJoint);
    return ritorno;
    }


    void topic_callback(const fp_core_msgs::msg::JointPosition::SharedPtr msg) const
    {
        for(int i = 0; i<8 ; i++){

            RCLCPP_INFO(this->get_logger(), "%s :  [rad] %f [grad] %f ", msg.get()->name[i].c_str(), msg.get()->position[i],(msg.get()->position[i])*(180/PI) );
        }
        RCLCPP_INFO(this->get_logger(), "");

        sleep(1);
    }

};


int main(int argc, char **argv){

  rclcpp::init(argc, argv);

  std::vector<MoveJoint_class> vettore_richieste;

  ClientProb3 cliente;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di sposstamento presenti nel main");

  //  Comandi per prendere la lattina, forse sarebbe interessante spostare la sedia sinistra più a sinistra...
  

  //  Comandi per appoggiare la lattina, da aggiustare un po' l'altezza del robot, si abbassa un pelo troppo 
   
  MoveJoint_class target_final_appoggio1(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{-0.4, -0.9, -0.4, 0.0, -0.2, 0.0, 0.1, 0.1}, 0.25);
  MoveJoint_class target_final_appoggio2(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{-0.4, -0.9, -0.4, 0.0, -0.37, 0.0, 0.1, 0.1}, 0.25);
  MoveJoint_class target_final_appoggio3(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{-0.4, -0.9, -0.4, 0.0, -0.37, 0.0, 0.5, 0.5}, 0.25);
  MoveJoint_class target_final_appoggio4(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.25);

  vettore_richieste.push_back(target_final_appoggio1);
  vettore_richieste.push_back(target_final_appoggio2);
  vettore_richieste.push_back(target_final_appoggio3);
  vettore_richieste.push_back(target_final_appoggio4);

  for(int i = 0; i < int(vettore_richieste.size());i++){

  // nel caso in cui si voglia dare in ingresso delle posizioni in gradi  pi/180

    auto risposta = cliente.send_request_MoveJoint(vettore_richieste[i]);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " %s : %f" , messaggio_sensori.get()->name[i].c_str(), messaggio_sensori.get()->position[i]);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), risposta.get()-> message.c_str() );
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d",risposta.get()-> success);

    
    if(risposta.get()->message == "error"){

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"errore in input");
        break;
    }
    
    sleep(4.0);
  }
  //----------------
  rclcpp::shutdown();


}