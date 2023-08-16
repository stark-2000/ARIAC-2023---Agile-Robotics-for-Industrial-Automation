from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.actions
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters

def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('ariac_gazebo')

    service_client_exe_node = Node(
        package="rwa67",
        executable="service_client_exe",
    )

    locate_parts_trays_exe_node = Node(
        package="rwa67",
        executable="locate_parts_trays_exe",
    )
    
    order_manager_node = Node(
        package="rwa67",
        executable="order_manager.py",
    )

    ship_order_exe_node = Node(
        package="rwa67",
        executable="ship_order_exe",
    )

    submit_order_node = Node(
        package="rwa67",
        executable="submit_order.py",
    )
    
    end_competition_node = Node(
        package="rwa67",
        executable="end_comp_client_exe.py"
    )
    robot_commander_exe_node = Node(
        package="rwa67",
        executable="robot_commander_exe",
        output="screen",
        parameters=generate_parameters()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        )
    )
    
    # Set up an action to include another launach file, with launch arguments
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                pkg_dir + '/launch/ariac.launch.py'), launch_arguments={'competitor_pkg': 'rwa67', 'sensor_conﬁg': 'sensors', 
                                                                        'trial_name': 'rwa67_summer2023'}.items())
    ld.add_action(moveit)
    ld.add_action(included_launch)
    ld.add_action(service_client_exe_node)
    ld.add_action(locate_parts_trays_exe_node)
    ld.add_action(order_manager_node)
    ld.add_action(ship_order_exe_node)
    ld.add_action(submit_order_node)
    ld.add_action(end_competition_node)
    ld.add_action(robot_commander_exe_node)

    
    return ld
