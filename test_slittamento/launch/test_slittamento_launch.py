import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController

# from webots_ros2_driver.utils import get_wsl_ip_address, is_wsl

#Import per settare la porta
def generate_launch_description():
    package_dir = get_package_share_directory('test_slittamento')
    
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'Pioneer2.urdf')).read_text()
    robot_description_x = pathlib.Path(os.path.join(package_dir, 'resource', 'Prob3.urdf')).read_text()
    robot_description_y = pathlib.Path(os.path.join(package_dir, 'resource', 'TiagoBase.urdf')).read_text()

    webots = WebotsLauncher(
        # world=os.path.join(package_dir, 'worlds', 'test_scivolamento_solido.wbt'),
        world=os.path.join(package_dir, 'worlds', 'test_scivolamento_solido_senza.wbt'),

    )
    
    pioneer2_driver = WebotsController(

        # robot_name= 'Pioneer%202',
        robot_name= 'Pioneer 2',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )
    
    prob3_driver = WebotsController(
        robot_name= 'P-Rob3',
        parameters=[
            {'robot_description': robot_description_x},
            {'use_sim_time': True},    
        ]
    )

    tiago_base_driver = WebotsController(

        # robot_name= 'TIAGo%20Base',
        robot_name= 'TIAGo Base',
        parameters=[
            {'robot_description': robot_description_y},
            {'use_sim_time': True},
        ] 

    )
            
    return LaunchDescription([
        webots,
        prob3_driver,
        pioneer2_driver,
        tiago_base_driver,
        # tiago_obstacle_avoidance,
        # nodo_servizio,
        # nodo_cliente,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])