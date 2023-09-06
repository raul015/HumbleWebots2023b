#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include<thread>
#include <memory>
#include <unistd.h>  // per time_sleep


auto posizione = std_msgs::msg::Float64();

using namespace std::chrono_literals;

// INIZIO CREAZIONE CLASSE PER IL NODO 

class PortaSimulazione : public rclcpp::Node

{

  public:

    PortaSimulazione() : Node("Porta_Simulazione_cpp"){
        
        publisher__ = this->create_publisher<std_msgs::msg::Float64>("posizione_porta",10);

        // Qui determino ogni quanto tempo eseguire la funzione di callback

        timer__ = this->create_wall_timer(
        500ms, std::bind(&PortaSimulazione::timer_callback, this));
    }

  private:

    void timer_callback(){

        Apertura();
        }

    void Apertura(){

        RCLCPP_INFO(this->get_logger(), "chiama delle funzione per apertura e chiusura della porta");

        sleep(2.0);
        RCLCPP_INFO(this->get_logger(), "Porta: 0.0");

        posizione.data = 0.0;

        publisher__->publish(posizione);

        sleep(2.0);
        RCLCPP_INFO(this->get_logger(), "Porta: 0.5");

        posizione.data = 0.5;

        publisher__->publish(posizione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Porta: -0.5");

        posizione.data = -0.5;

        publisher__ ->publish(posizione);

    }

    rclcpp::TimerBase::SharedPtr timer__;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher__;

};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PortaSimulazione>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di apertura e chiusura delle porte");
    
    rclcpp::shutdown();


}