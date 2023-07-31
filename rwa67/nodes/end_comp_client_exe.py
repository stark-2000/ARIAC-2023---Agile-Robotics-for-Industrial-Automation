#!/usr/bin/env python3

import rclpy
from rwa67.end_comp_client import CompetitionEndingClient


def main():
    '''
    Instantiates a ndoe of CompetitionEndingClient
    '''
    rclpy.init()
    ariac_end_comp_client  = CompetitionEndingClient("end_comp_client")

    try:
        rclpy.spin(ariac_end_comp_client)
    except KeyboardInterrupt:
        ariac_end_comp_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    ariac_end_comp_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()