#include "rclcpp/rclcpp.hpp"
#include "fp_core_msgs/srv/move_joint.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include<thread>
#include <memory>
#include <unistd.h>  // per time_sleep


auto illuminazione = std_msgs::msg::Float64();

using namespace std::chrono_literals;


// INIZIO CREAZIONE CLASSE PER IL NODO 

class IlluminazioneSimulazione : public rclcpp::Node

{

  public:

    IlluminazioneSimulazione() : Node("Illuminazione_Simulazione_cpp"){
        
        publisher__ = this->create_publisher<std_msgs::msg::Float64>("tiago_illuminazione",10);

        // Qui determino ogni quanto tempo eseguire la funzione di callback

        timer__ = this->create_wall_timer(
        500ms, std::bind(&IlluminazioneSimulazione::timer_callback, this));
    }

  private:

    void timer_callback(){

        Illuminazione();
        }

    void Illuminazione(){

        RCLCPP_INFO(this->get_logger(), "chiama delle funzione illuminazione");

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 3.0");

        illuminazione.data = 3.0;

        publisher__->publish(illuminazione);

        sleep(2.0);
        RCLCPP_INFO(this->get_logger(), "Illuminazione: 2.0");

        illuminazione.data = 2.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 1.0");

        illuminazione.data = 1.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 0.0");

        illuminazione.data = 0.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 1.0");

        illuminazione.data = 1.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 2.0");

        illuminazione.data = 2.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 3.0");

        illuminazione.data = 3.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

        RCLCPP_INFO(this->get_logger(), "Illuminazione: 4.0");

        illuminazione.data = 4.0;

        publisher__->publish(illuminazione);

        sleep(2.0);

    }

    rclcpp::TimerBase::SharedPtr timer__;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher__;

};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IlluminazioneSimulazione>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di variazione illuminazione...");
    
    rclcpp::shutdown();


}