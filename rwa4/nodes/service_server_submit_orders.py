#!/usr/bin/env python3

import rclpy
from rwa4.submit_order_interface import Service_Server

def main(args=None):
    rclpy.init(args=args) #initialize ros2 communication

    _Submit_Orders = Service_Server() #create object of class Service_Server
    rclpy.spin(_Submit_Orders) #keep the node running

    _Submit_Orders.destroy_node() #Destroy the node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()