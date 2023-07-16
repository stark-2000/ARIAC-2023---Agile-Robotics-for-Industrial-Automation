from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.actions



def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir = get_package_share_directory('ariac_gazebo')

    service_client_exe_node = Node(
        package="rwa4",
        executable="service_client_exe",
    )

    order_manager_node = Node(
        package="rwa4",
        executable="order_manager.py",
    )

    ship_order_exe_node = Node(
        package="rwa4",
        executable="ship_order_exe",
    )

    submit_order_node = Node(
        package="rwa4",
        executable="submit_order.py",
    )
    
    end_competition_node = Node(
        package="rwa4",
        executable="end_comp_client_exe.py"
    )
    

    #Launch the launch file from av_server package
    included_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                pkg_dir + '/launch/ariac.launch.py'), launch_arguments={'trial_name': 'rwa4_summer2023'}.items())


    ld.add_action(included_launch)
    ld.add_action(service_client_exe_node)
    ld.add_action(order_manager_node)
    ld.add_action(ship_order_exe_node)
    ld.add_action(submit_order_node)
    ld.add_action(end_competition_node)
    
    return ld
