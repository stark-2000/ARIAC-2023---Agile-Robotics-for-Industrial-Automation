#!/usr/bin/env python3

import rclpy
from rwa5.orders_interface import OrderManager


def main(args=None):
    """
    Program to run an OrderPublisher Node

    Args:
        args (_type_, optional): _description_. Defaults to None.
    """
    rclpy.init(args=args)
    order_manager = OrderManager("order_manager")
    rclpy.spin(order_manager)
    order_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
