#!/usr/bin/env python3

import rclpy
from rwa5.submit_order_interface import Submit_Orders


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    _Submit_Orders = Submit_Orders() #create object of class Submit_Orders
    rclpy.spin(_Submit_Orders) #keep the node running

    _Submit_Orders.destroy_node() #Destroy the node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()