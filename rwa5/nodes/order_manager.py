#!/usr/bin/env python3

import rclpy
from rwa5.orders_interface import OrderManager
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    """
    Program to run an OrderPublisher Node

    Args:
        args (_type_, optional): _description_. Defaults to None.
    """
    rclpy.init(args=args)
    order_manager = OrderManager("order_manager")
    executor = MultiThreadedExecutor()
    executor.add_node(order_manager)
    try:
        order_manager.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        order_manager.get_logger().info('Keyboard interrupt, shutting down.\n')
    order_manager.destroy_node()
    rclpy.shutdown()

    # rclpy.spin(order_manager)
    # order_manager.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
