import rclpy
from fp_core_msgs.msg import WheelPosition

# I motori vanno da 10.0 a 0.0
max_speed_motor = 10.0
min_speed_motor = 0.0

motor_left_name = 'left wheel motor'
motor_right_name = 'right wheel motor'

sensor_left_name = 'left wheel sensor'
sensor_right_name = 'right wheel sensor'


TIME_STEP = 32 # milliseconds

msg = WheelPosition()

msg.name.append(sensor_left_name)
msg.data.append(0.0)

msg.name.append(sensor_right_name)
msg.data.append(0.0)

class Pioneer2Driver:
        
    def init(self, webots_node, properties): 
        
                
        rclpy.init(args=None)
                
        self.__robot = webots_node.robot
        self.__timestep = TIME_STEP
        
        self.__node = rclpy.create_node('pioneer2_driver')
                
        
        self.__motor_left = self.__robot.getDevice(motor_left_name)
        self.__motor_right = self.__robot.getDevice(motor_right_name)
        
        self.__sensor_left = self.__robot.getDevice(sensor_left_name)
        
        self.__sensor_left.enable(self.__timestep)
        
        self.__sensor_right = self.__robot.getDevice(sensor_right_name)
        self.__sensor_right.enable(self.__timestep)

             
        # Inizializzo le velocita del robot 
        self.__motor_left.setPosition(float('inf'))
        self.__motor_right.setPosition(float('inf'))
        
        self.__motor_left.setVelocity(1.0)
        self.__motor_right.setVelocity(-1.0)
        
        self.publisher_sensor_values = self.__node.create_publisher(WheelPosition,'Pioneer2_sensors',10)

    
    def step(self): 
    
        msg.data[0] = self.__sensor_left.getValue()
        msg.data[1] = self.__sensor_right.getValue()
        
        # self.__node.get_logger().info('sensore pioneer L[' + str(0)+ '] = ' + str(msg.data[0]))
        # self.__node.get_logger().info('sensore pioneer R[' + str(1)+ '] = ' + str(msg.data[1]))

                            
        rclpy.spin_once(self.__node, timeout_sec=0)
        
                    