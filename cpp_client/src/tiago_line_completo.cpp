#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include<thread>
#include <memory>
#include <unistd.h>  // per time_sleep

auto cmd = geometry_msgs::msg::Twist();
auto GS_LEFT = std_msgs::msg::Float64();
auto GS_RIGHT = std_msgs::msg::Float64();
auto GS_middle = sensor_msgs::msg::Range();
auto permesso = std_msgs::msg::Int64();
auto presa = std_msgs::msg::Int64();

bool fine = false;

int contatore = 0;  // Conta quanto tempo sto nella linea nera
/*

  Idea, utilizzare un subscriber che continui a leggere i valori 
  pubblicati dal nodo publisher nel topic 'Sensors_values'

*/


using namespace std::chrono_literals;
// INIZIO CREAZIONE CLASSE PER IL NODO 

class LineFollower : public rclcpp::Node

{
  public:

    // _Float64 GS_middle = 100.0;


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subLetf;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subRight;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subMiddle;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_permesso;

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_presa;


    LineFollower() : Node("LineFollower_cpp"){
    // using std::placeholders::_1;
    subLetf = this->create_subscription<std_msgs::msg::Float64>("tiago_ir_left",10, std::bind(&LineFollower::leftIR_cb, this, std::placeholders::_1));
    subRight = this->create_subscription<std_msgs::msg::Float64>("tiago_ir_right",10, std::bind(&LineFollower::rightIR_cb, this, std::placeholders::_1));
    subMiddle = this->create_subscription<sensor_msgs::msg::Range>("middle_sensor",10, std::bind(&LineFollower::middleSensor_cb, this, std::placeholders::_1));
    cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("tiago_cmd_vel", 10);

    pub_presa = this->create_publisher<std_msgs::msg::Int64>("presa", 10);
    sub_permesso = this->create_subscription<std_msgs::msg::Int64>("permesso",10, std::bind(&LineFollower::permesso_cb, this, std::placeholders::_1));


    }

    void LineFolloModule()const {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inizio LineFolloModule");
    
        if(GS_middle.range>0.1){

            if(GS_LEFT.data<5 && GS_RIGHT.data >=5){

                cmd.linear.x = -0.1;
                cmd.angular.x = -0.2;
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data>=5 && GS_RIGHT.data <5){

                cmd.linear.x = -0.1;
                cmd.angular.x = 0.2;
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data>=5 && GS_RIGHT.data >=5){

                cmd.linear.x = -0.12;
                cmd.angular.x = 0.0;
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data<5 && GS_RIGHT.data <5){
                
                fine = true;
                cmd.linear.x = 0.0;
                cmd.angular.x = 0.0;
                cmd_vel->publish(cmd);

                presa.data = 1;
                pub_presa->publish(presa);
            }


        }
        else{
                cmd.linear.x = 0.0;
                cmd.angular.x = 0.0;
                cmd_vel->publish(cmd);


        }
    
    }

    void rightIR_cb( std_msgs::msg::Float64::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), " destro : %f ", msg.get()->data);
        GS_RIGHT.data = msg.get()->data;
    }

    void leftIR_cb( std_msgs::msg::Float64::SharedPtr msg) {

        RCLCPP_INFO(this->get_logger(), " sinistro : %f ", msg.get()->data);
        GS_LEFT.data = msg.get()->data;
    }

    void middleSensor_cb( sensor_msgs::msg::Range::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), " mezzo : %f ", msg.get()->range);
        GS_middle.range = msg.get()->range;
        //LineFolloModule();

    }

    void permesso_cb( std_msgs::msg::Int64::SharedPtr msg){

        permesso.data = msg.get()->data;
        if(permesso.data == 1){

            LineFolloModule();
        }
        else 
            RCLCPP_INFO(this->get_logger(), "Non hai ancora preso la lattina");

    }

};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LineFollower>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di sposstamento presenti nel main");

    // line.LineFolloModule();
    rclcpp::shutdown();


}