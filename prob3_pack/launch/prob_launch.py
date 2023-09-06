import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController


# from webots_ros2_driver.utils import get_wsl_ip_address, is_wsl

#Import per settare la porta
def generate_launch_description():
    package_dir = get_package_share_directory('prob3_pack')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'p-rob3.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher() 
    # controller_url = 'tcp://' + get_wsl_ip_address() + ':1234/' if is_wsl() else ''
 #'WEBOTS_CONTROLLER_URL': controller_url_prefix()+ 'P-Rob3'
 #WEBOTS_CONTROLLER_URL':'P-Rob3

    prob3_driver  = WebotsController(
        #robot_name= 'P-Rob3',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )


    
    # nodo_servizio = Node(
    #     package='prob3_pack',
    #     executable='nodo_servizio'
    # )
    
    # nodo_cliente = Node(
    #     package='prob3_pack',
    #     executable='nodo_cliente'
    # )
    #      above...  additional_env={'WEBOTS_CONTROLLER_URL': controller_url + 'my_robot'},
    
    
    # service_node_prob3 = Node(
    #     package='prob3_pack',
    #     executable='service_node_prob3',
    #     additional_env={'WEBOTS_CONTROLLER_URL':'P-Rob3'},
    #     parameters=[
    #         {'robot_description': robot_description},
    #     ]
    # )

    return LaunchDescription([
        webots,
        prob3_driver,
        #ros2_supervisor,
        # nodo_servizio,
        # nodo_cliente,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])