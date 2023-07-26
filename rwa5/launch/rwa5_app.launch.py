from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.actions

def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('ariac_gazebo')

    service_client_exe_node = Node(
        package="rwa5",
        executable="service_client_exe",
    )

    locate_parts_trays_exe_node = Node(
        package="rwa5",
        executable="locate_parts_trays_exe",
    )
    
    order_manager_node = Node(
        package="rwa5",
        executable="order_manager.py",
    )

    ship_order_exe_node = Node(
        package="rwa5",
        executable="ship_order_exe",
    )

    submit_order_node = Node(
        package="rwa5",
        executable="submit_order.py",
    )
    
    end_competition_node = Node(
        package="rwa5",
        executable="end_comp_client_exe.py"
    )

    # Set up an action to include another launach file, with launch arguments
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                pkg_dir + '/launch/ariac.launch.py'), launch_arguments={'competitor_pkg': 'rwa5', 'sensor_conﬁg': 'sensors', 
                                                                        'trial_name': 'rwa5_summer2023'}.items())

    ld.add_action(included_launch)
    ld.add_action(service_client_exe_node)
    ld.add_action(locate_parts_trays_exe_node)
    ld.add_action(order_manager_node)
    ld.add_action(ship_order_exe_node)
    ld.add_action(submit_order_node)
    ld.add_action(end_competition_node)
    
    return ld
