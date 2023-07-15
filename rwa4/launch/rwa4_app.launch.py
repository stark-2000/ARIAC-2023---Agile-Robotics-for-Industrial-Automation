from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


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

    service_server_submit_orders_node = Node(
        package="rwa4",
        executable="service_server_submit_orders.py",
    )

    submit_order_node = Node(
        package="rwa4",
        executable="submit_order.py",
    )
    
    end_competition_node = Node(
        package="rwa4",
        executable="end_comp_client_exe.py"
    )
    


    ld.add_action(service_client_exe_node)
    ld.add_action(order_manager_node)
    ld.add_action(ship_order_exe_node)
    ld.add_action(service_server_submit_orders_node)
    ld.add_action(submit_order_node)
    ld.add_action(end_competition_node)
    
    return ld
