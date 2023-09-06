import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

# from webots_ros2_driver.utils import get_wsl_ip_address, is_wsl

#Import per settare la porta
def generate_launch_description():
    package_dir = get_package_share_directory('final')
    
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'Pioneer2.urdf')).read_text()
    robot_description_x = pathlib.Path(os.path.join(package_dir, 'resource', 'Prob3.urdf')).read_text()
    robot_description_y = pathlib.Path(os.path.join(package_dir, 'resource', 'TiagoBase.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'final_robot.wbt'),
    )

    
    # controller_url = 'tcp://' + get_wsl_ip_address() + ':1234/' if is_wsl() else ''
 #'WEBOTS_CONTROLLER_URL': controller_url_prefix()+ 'P-Rob3'
 #WEBOTS_CONTROLLER_URL':'P-Rob3

    
    prob3_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL':controller_url_prefix() + 'P-Rob3'},
        parameters=[
            {'robot_description': robot_description_x},
            {'use_sim_time': True},
        ]
    )
    
    
    
    pioneer2_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL':controller_url_prefix() +'Pioneer%202'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )
    
    tiago_base_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL':controller_url_prefix() +'TIAGo%20Base'},
        parameters=[
            {'robot_description': robot_description_y},
            {'use_sim_time': True},
        ]
    )
    
    # tiago_obstacle_avoidance = Node(
    #     package = 'mix',
    #     executable = 'tiago_obstacle_avoidance',
    # )
    
    
    ros2_supervisor = Ros2SupervisorLauncher()
    
    return LaunchDescription([
        webots,
        prob3_driver,
        pioneer2_driver,
        tiago_base_driver,
        # tiago_obstacle_avoidance,
        ros2_supervisor,
        # nodo_servizio,
        # nodo_cliente,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])