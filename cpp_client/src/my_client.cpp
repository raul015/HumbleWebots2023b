#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

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

    private:
    
    std::vector<int64_t> actuator_ids;
    std::vector<double> position;
    double velocity;
    double acceleration;
    bool block;
    bool relative;

};



// INIZIO CREAZIONE CLASSE PER IL NODO 

class ClientProb3 : public rclcpp::Node

{
  public:
    
     rclcpp::Client<fp_core_msgs::srv::MoveJoint>::SharedPtr cliente;

    ClientProb3() : Node("Client_Prob3_cpp"){
     cliente = this->create_client<fp_core_msgs::srv::MoveJoint>("MoveJoint");
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
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "errore nella chiamata del servizio");
      } 
    // rclcpp::spin_until_future_complete(cliente, future);
    return future;
  }

  void send_request_MoveJoint(std::vector<MoveJoint_class> vettore){

    int contatore = 0;
    bool check_messaggio = true;

    while(contatore < int(vettore.size()) && check_messaggio == true){

      auto ritorno = sendRequest(vettore[contatore]);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ritorno.get()->message.c_str());
      if(ritorno.get()->success == true){
        contatore += 1;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      if(ritorno.get()->message == "errore"){
        check_messaggio = false;
      }

    }

  }


  private:


};


int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  std::vector<MoveJoint_class> vettore_richieste;
  
  ClientProb3 cliente;

  // ClientProb3* cliente = new ClientProb3();

  MoveJoint_class target_1(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{1.0, 0.5, 0.2, 2.0, 1.5, 0.3, 0.5, 0.5}, 0.2);
  MoveJoint_class target_2(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.2);
  MoveJoint_class target_3(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, 0.2);


  // Movimenti relativi per il robot


  MoveJoint_class target_4(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{-0.5, -0.5,-0.5, -0.5,-0.5,-0.5,-0.5,-0.5}, 0.2, 2.0,false,true);
  MoveJoint_class target_5(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1, 2.0,false,true);
  MoveJoint_class target_6(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, 0.2, 2.0, false, true);


// Movimento assoluto

  MoveJoint_class target_finale(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.2);


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di sposstamento presenti nel main");

  vettore_richieste.push_back(target_1);
  vettore_richieste.push_back(target_2);
  vettore_richieste.push_back(target_3);


  vettore_richieste.push_back(target_4);
  vettore_richieste.push_back(target_5);
  vettore_richieste.push_back(target_6);
  vettore_richieste.push_back(target_finale);


  cliente.send_request_MoveJoint(vettore_richieste);
  rclcpp::shutdown();


}