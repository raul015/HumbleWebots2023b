#include"rclcpp/rclcpp.hpp"
#include"rclcpp/node.hpp"
#include"sensor_msgs/msg/range.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"std_msgs/msg/float64.hpp"


const double MAX_RANGE = 0.5;
const double MAX_SPEED = 10.2;


// Versione in c++ Del segui linea per il robot tiago_base....


using std::placeholders::_1;
class LineFollower : public rclcpp::Node{

    public:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_ir_left;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_ir_right;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_middle;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;


    double right_sensor_value;
    double left_sensor_value;
    double  middle_sensor_value;
    
    bool left_libero;
    bool right_libero;
    bool middle_libero;

    bool inizio;
    bool fine;


    LineFollower(): Node("LineFollower_cpp"){ 

        // Inizializzazione

        right_sensor_value = 0.0;
        left_sensor_value = 0.0;
        middle_sensor_value = 0.0;

        left_libero = false;
        right_libero = false;
        middle_libero = false;


        inizio = true;
        fine = false;

        // Costruttore in cui dichiare i miei subscriber alla lettura dei sensori

        sub_ir_left = this->create_subscription<std_msgs::msg::Float64>(
            "tiago_ir_left", 10, std::bind(&LineFollower::left_sensor_callback, this, _1));

        sub_ir_right = this->create_subscription<std_msgs::msg::Float64>(
            "tiago_ir_right", 10 , std::bind(&LineFollower::right_sensor_callback, this, _1));

        sub_sonar_middle = this->create_subscription<sensor_msgs::msg::Range>(
            "middle_sensor",10,std::bind(&LineFollower::middle_sensor_callback,this,_1));

        // Definisco qui il mio publisher 

        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("tiago_cmd_vel",10);

        // Non ho una funzione  in quanto devo solo pubblicare dei dati al topic tiago_cmd_vel

    }

    private:

    void left_sensor_callback(std_msgs::msg::Float64::SharedPtr msg) {

        left_sensor_value = msg.get()->data;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sensore sinistro");

        if(left_sensor_value <5.0)

            left_libero = false;
        
        else
            left_libero = true;
    
    }

    void right_sensor_callback(std_msgs::msg::Float64::SharedPtr msg){
          
        right_sensor_value = msg.get()->data;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sensore destro");

        if(right_sensor_value < 5.0)

            right_libero = false;

        else

            right_libero = true;

    }

    void middle_sensor_callback( sensor_msgs::msg::Range::SharedPtr msg){

        middle_sensor_value = msg.get()->range;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sensore di mezzo");

        // Oggetto da mandare al topic cmd_vel_per il settaggio delle velocità

        auto request = geometry_msgs::msg::Twist();

        if(inizio == true){

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inizio = True");

            if(left_libero == false && right_libero == false){

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inizio entrambi i sensori sulla riga nera");

                request.linear.x = -1.0;
                request.angular.z = 0.7;

                // Devo pubblicare il messaggio ... definire la funzione 
                
                pub_vel->publish(request);
            }
            else

                inizio = false;
    
        }

        else{

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inizio = False");

            if(left_libero == true && right_libero == true && fine == false){

                request.linear.x = -0.5;
                request.angular.z = 0.0;
                pub_vel->publish(request);

            }

            else if(left_libero == true && right_libero == false && fine == false){

                request.linear.x = -0.5;
                request.angular.z = 1.0;
                pub_vel->publish(request);

            }

            else if(left_libero == false && right_libero == true && fine == false){

                request.linear.x = -0.5;
                request.angular.z = -1.0;
                pub_vel->publish(request);

            }

            else if(left_libero == false && right_libero == false && fine == false){

                request.linear.x = 0.0;
                request.angular.z = 0.0;
                pub_vel->publish(request);
                fine = true;

            }

        }

    }

};

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollower>());  
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inizio delle operazione di Line follower....");
    rclcpp::shutdown();

    return 0;



}