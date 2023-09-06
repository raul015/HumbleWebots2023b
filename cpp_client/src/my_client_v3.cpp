#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

const double PI = 3.1415926535897932384626433832795028841971693993751058209;


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

  // Movimento assoluto per tornare alla posizione iniziale 
  
  MoveJoint_class target_finale(std::vector<int64_t>{1,2,3,4,5,6,7,8} , std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.2);




  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di sposstamento presenti nel main");

  vettore_richieste.push_back(target_1);
  vettore_richieste.push_back(target_2);
  vettore_richieste.push_back(target_3);


  vettore_richieste.push_back(target_4);
  vettore_richieste.push_back(target_5);
  vettore_richieste.push_back(target_6);
  vettore_richieste.push_back(target_finale);


  for(int i = 0; i< int(vettore_richieste.size());i++){

  // nel caso in cui si voglia dare in ingresso delle posizioni in gradi  pi/180
    auto risposta = cliente.send_request_MoveJoint(vettore_richieste[i]);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), risposta.get()-> message.c_str() );
    
    if(risposta.get()->message == "error"){

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"errore in input");
        break;
    }


        // if(ritorno.message == "error"):
        //     client.get_logger().info('errore in input')
        //     break

  }

  rclcpp::shutdown();


}