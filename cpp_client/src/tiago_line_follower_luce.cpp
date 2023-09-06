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
auto GS_LIGHT = std_msgs::msg::Float64();



using namespace std::chrono_literals;

class LineFollower : public rclcpp::Node

{
  public:

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subLetf;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subRight;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subMiddle;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subLight;


    LineFollower() : Node("LineFollower_cpp"){
    // using std::placeholders::_1;
    subLetf = this->create_subscription<std_msgs::msg::Float64>("tiago_ir_left",10, std::bind(&LineFollower::leftIR_cb, this, std::placeholders::_1));
    subRight = this->create_subscription<std_msgs::msg::Float64>("tiago_ir_right",10, std::bind(&LineFollower::rightIR_cb, this, std::placeholders::_1));
    subMiddle = this->create_subscription<sensor_msgs::msg::Range>("middle_sensor",10, std::bind(&LineFollower::middleSensor_cb, this, std::placeholders::_1));
    cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("tiago_cmd_vel", 10);
    subLight = this->create_subscription<std_msgs::msg::Float64>("tiago_light_sensor",10, std::bind(&LineFollower::lightSensor_cb, this, std::placeholders::_1));

    }

    void LineFolloModule()const {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inizio LineFolloModule");
    
        if(GS_middle.range>0.1 && GS_LIGHT.data>500){

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "La illuminazione è perfetta per il segui linea");

            /*
            Inserisco nuovi valori di offSet in quanto con olio la riflessione della 
            luce è circa intorno ai 70
            */ 
        
            if(GS_LEFT.data<100 && GS_RIGHT.data >=100){

                cmd.linear.x = -0.1;
                cmd.angular.x = -0.2;
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data>=100 && GS_RIGHT.data <100){

                cmd.linear.x = -0.1;
                cmd.angular.x = 0.2;
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data>=100 && GS_RIGHT.data >=100){ // dritto

                cmd.linear.x = -0.12;
                cmd.angular.x = -0.1; // Da sistemare questo angolo
                cmd_vel->publish(cmd);
            }

            else if(GS_LEFT.data<40 && GS_RIGHT.data <40){
                
                cmd.linear.x = 0.0;
                cmd.angular.x = 0.0;
                cmd_vel->publish(cmd);
            }


        }
        else{

                if(GS_LIGHT.data <= 500){

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "La illuminazione non va bene per il segui linea");

                }
                if(GS_middle.range<=0.1){

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "È stato trovato un ostacolo");

                }

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


    void lightSensor_cb(std_msgs::msg::Float64::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), " Illuminazione : %f ", msg.get()->data);
        GS_LIGHT.data = msg.get()->data;

    }

    void middleSensor_cb( sensor_msgs::msg::Range::SharedPtr msg){

        RCLCPP_INFO(this->get_logger(), " mezzo : %f ", msg.get()->range);
        GS_middle.range = msg.get()->range;
        LineFolloModule();

    }


};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LineFollower>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio sequenza di sposstamento presenti nel main");

    // line.LineFolloModule();
    rclcpp::shutdown();


}