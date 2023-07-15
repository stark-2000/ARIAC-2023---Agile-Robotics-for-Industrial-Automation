#!/usr/bin/env python3

import rclpy
from rwa4.orders_interface import OrderManager


def main(args=None):
    """
    Program to run an OrderPublisher Node

    Args:
        args (_type_, optional): _description_. Defaults to None.
    """
    rclpy.init(args=args)
    order_manager = OrderManager("order_manager")
    try:

        order_manager.get_logger().info("Starting order manager node, shut down with CTRL-C")
        rclpy.spin(order_manager)
    except KeyboardInterrupt:
        order_manager.get_logger().warn('Keyboard interrupt, shutting down.\n')
    order_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
