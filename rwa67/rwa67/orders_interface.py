import time
import rclpy
from rclpy.parameter import Parameter
from collections import deque
from enum import Enum
from functools import partial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from ariac_msgs.msg import Order as OrderMsg, AdvancedLogicalCameraImage as ALCImage, CompetitionState, Part
from ariac_msgs.srv import ChangeGripper
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

# Import custom ROS services
from robot_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV
)

class GripperTypes(Enum):
    TRAY_GRIPPER = 'tray gripper'
    PART_GRIPPER = 'part gripper'


class TrayStations(Enum):
    KTS_1 = 1
    KTS_2 = 2


class Order:
    ''' 
    Class to store one order message from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority
        self.agv_number = msg.kitting_task.agv_number
        self.tray_id = msg.kitting_task.tray_id
        self.destination = msg.kitting_task.destination
        self.parts = msg.kitting_task.parts


class OrderManager(Node):
    """
    Class for subscribing to receive an order, and add it to the appropriate queue

    Args:
        Node (Node): Class for creating an ROS node
    """

    _part_colors = {
        Part.RED: 'red',
        Part.BLUE: 'blue',
        Part.GREEN: 'green',
        Part.ORANGE: 'orange',
        Part.PURPLE: 'purple',
    }
    _part_types = {
        Part.BATTERY: 'battery',
        Part.PUMP: 'pump',
        Part.REGULATOR: 'regulator',
        Part.SENSOR: 'sensor',
    }

    def __init__(self, node_name):
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])
        # Queue to hold regular orders
        self._order_queue = deque()
        # Queue to hold priority orders
        self._priority_queue = deque()
        self._current_order = None
        # Dictionaries to hold parts and their poses
        self._left_bin_inventory = []
        self._right_bin_inventory = []
        # Dictionaries to hold trays and their poses
        self._kts1_inventory = []
        self._kts2_inventory = []
        # Variable to hold paused order
        self._paused_order = None

        self._left_bin_data_received = False
        self._right_bin_data_received = False
        self._kts1_data_received = False
        self._kts2_data_received = False
        self._started_orders = False

        # Callback groups to allow multiple callbacks to be called at the same time
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Multiple flags to trigger the robot actions

        # The following flags prevent the same action to be called multiple times
        self._robot_moving_to_home = False
        self._moving_robot_to_table = False
        self._entering_tool_changer = False
        self._changing_gripper = False
        self._exiting_tool_changer = False
        self._activating_gripper = False
        self._deactivating_gripper = False
        self._moving_robot_to_tray = False
        self._moving_tray_to_agv = False
        self._ending_demo = False

        # The following flags are used to trigger the next action
        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None
        self._robot_moved_to_home = False
        self._moved_robot_to_table = False
        self._entered_tool_changer = False
        self._changed_gripper = False
        self._exited_tool_changer = False
        self._activated_gripper = False
        self._deactivated_gripper = False
        self._moved_robot_to_tray = False
        self._moved_tray_to_agv = False

        # Subscriber to listen for orders
        self._order_subscriber = self.create_subscription(
            OrderMsg, 'ariac/orders', self.order_callback, 10)
        # Publisher to notify that order is ready to ship
        self._order_ship_publisher = self.create_publisher(
            UInt8, 'ship_order', 10)

        self._fulfillment_callback_group = MutuallyExclusiveCallbackGroup()

        self._competition_state_subscriber = self.create_subscription(
            CompetitionState, 'ariac/competition_state', self.comp_state_callback, 10, callback_group=self._fulfillment_callback_group)

        # Subscribers to receive updated poses with world frame
        self._left_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/left_bin_camera_data', self.left_bin_callback, qos_profile_sensor_data)

        self._right_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/right_bin_camera_data', self.right_bin_callback, qos_profile_sensor_data)

        self._kts1_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray1_camera_data', self.kts1_callback, qos_profile_sensor_data)

        self._kts2_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray2_camera_data', self.kts2_callback, qos_profile_sensor_data)

        
        self._move_robot_to_tray_cli = self.create_client(
            MoveRobotToTray, '/commander/move_robot_to_tray')
        
        # client to move a robot to a table
        self._move_robot_to_table_cli = self.create_client(
            MoveRobotToTable,
            '/commander/move_robot_to_table')

    def left_bin_callback(self, camera_msg: ALCImage):
        for part_pose in camera_msg.part_poses:
            self._left_bin_inventory.append(part_pose)
        self._left_bin_data_received = True

    def right_bin_callback(self, camera_msg: ALCImage):
        for part_pose in camera_msg.part_poses:
            self._right_bin_inventory.append(part_pose)
        self._right_bin_data_received = True

    def kts1_callback(self, camera_msg: ALCImage):
        for tray_pose in camera_msg.tray_poses:
            self._kts1_inventory.append(tray_pose)
        self._kts1_data_received = True

    def kts2_callback(self, camera_msg: ALCImage):
        for tray_pose in camera_msg.tray_poses:
            self._kts2_inventory.append(tray_pose)
        self._kts2_data_received = True

    def comp_state_callback(self, state_msg: CompetitionState):
        if (state_msg.competition_state == CompetitionState.READY):
            self._started_orders = True
            self.get_logger().info(
                'starting orders')
            while (True):
                if (self._left_bin_data_received
                    and self._right_bin_data_received
                    and self._kts1_data_received
                        and self._kts2_data_received):
                    self.fulfill_order()
                time.sleep(0.2)

    def order_callback(self, order_msg: OrderMsg):
        """
        Callback for when an Order is published.

        Args:
            order (Order): A single Order
        """
        # Create an Order object from the msg
        order = Order(order_msg)
        # If not currently working on an order, set new order to current
        if self._current_order is None:
            self._current_order = order
            self.get_logger().info(
                f'No orders in queue. Starting order {order.order_id}.')
        else:
            # If incoming order is priority, add it to priority queue.
            # If priority queue is empty and current order is not priority,
            # replace current order with incoming one, and move current order back to the front of the order queue.
            if order.order_priority:
                if len(self._priority_queue == 0):
                    if not self._current_order.order_priority:
                        self._paused_order = self._current_order
                        self._current_order = order
                    else:
                        self._priority_queue.append(order)
                else:
                    self._priority_queue.append(order)
            else:
                self._order_queue.append(order)
            self.get_logger().info(
                f'Order {order.order_id} added to queue.')
            self.get_logger().info(
                f'Number of orders in priority queue: {len(self._priority_queue)}')
            self.get_logger().info(
                f'Number of orders in regular queue: {len(self._order_queue)}')

    def next_order(self):
        """
        Moves current order to start working on the next order
        """
        # If there is a priority order, start working on that next
        if len(self._priority_queue) > 0:
            self._current_order = self._priority_queue.popleft()
        # If an order has been paused, set as current order
        elif self._paused_order is not None:
            self._current_order = self._paused_order
            self._paused_order = None
        # If there are orders in regular queue, start working on the next order
        elif len(self._order_queue) != 0:
            self._current_order = self._order_queue.popleft()
        # If there are no orders, set current order to None
        else:
            self._current_order = None

    def queues_are_empty(self) -> bool:
        """
        Function to check queue status

        Returns:
            bool: Returns true if both queues are empty, false otherwise
        """
        if len(self._priority_queue) == 0 and len(self._order_queue) == 0:
            return True
        return False

    def complete_order(self):
        """_
        Function to notify next stage that an agv is ready to be shipped.
        """

        # Create message containing agv number of current order and publish it
        ship_msg = UInt8(data=self._current_order.agv_number)
        self._order_ship_publisher.publish(ship_msg)
        # Move to start the next order
        self.next_order()

    def fulfill_order(self):
        if self._current_order is None:
            return
        order = self._current_order
        self.get_logger().info(
            f'Fulfilling: {order.order_id}')
        target_tray = self._current_order.tray_id
        target_agv = self._current_order.agv_number
        tray_pose = None
        station = None
        for tray in self._kts1_inventory:
            if tray.id == target_tray:
                tray_pose = tray.pose
                station = TrayStations.KTS_1
                break
        if tray_pose == None:
            for tray in self._kts2_inventory:
                if tray.id == target_tray:
                    tray_pose = tray.pose
                    station = TrayStations.KTS_2
                    break
        if station == None:
            self.get_logger().fatal(
                f"Tray {target_tray} not found. Can not complete order {order.order_id}")
            return False

        # Change gripper to tray
        #self.change_gripper(station, GripperTypes.TRAY_GRIPPER)
        self._move_robot_to_table(station.value)
        self._move_robot_to_tray(target_tray, tray_pose)
        self.pick_up_tray(target_tray, tray_pose)
        while(True):
            continue
        # self.place_tray(target_tray, target_agv)

        # # Change gripper to part
        # self.change_gripper(station, GripperTypes.PART_GRIPPER)
        # for order_part in order.parts:
        #     part_found = False
        #     for item in self._left_bin_inventory:
        #         if item.part == order_part.part:
        #             self.pickup_part(
        #                 item.part.color, item.part.type, item.pose)
        #             part_found = True
        #             break
        #     if not part_found:
        #         for item in self._right_bin_inventory:
        #             if item.part == order_part.part:
        #                 self.pickup_part(
        #                     item.part.color, item.part.type, item.pose)
        #                 part_found = True
        #                 break
        #     if not part_found:
        #         self.get_logger().fatal(
        #             f"Part not found. Can not complete order {order.order_id}")
        #         return False
        #     self.place_part(order_part.part.color, order_part.part.type, order.tray_id,
        #                     order_part.quadrant)
        # self.complete_order()

    def change_gripper(self, table_name, gripper_type):
        """! callback to a service - ChangeGripper - format: "call_servicename_server"

        @param table_name   The station name or table name in which tray is located; of type: TrayStations
        @param gripper_type The type of the gripper - part gripper or tray gripper; of type: GripperTypes

        @return None
        """

        self.get_logger().info(
            f'Changing to {gripper_type.value} at {table_name.value}')
        return

    def pick_up_tray(self, id, pose):
        """
        Function for picking up a tray

        Parameters:
            id (int): The id of the tray to be picked up
            pose (Pose): The pose of the tray to be picked up
        """

        self.get_logger().info(
            f'Picking up tray {str(id)} at '\
                f'[{pose.position.x} {pose.position.y} {pose.position.z}]' \
                f'[{pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}]')

    def place_tray(self, id, agv):
        """
        Function for placing a tray

        Parameters:
            id (int): The id of the tray to be placed
            agv (int): The agv to place the tray on
        """

        self.get_logger().info(f'Placing tray {id} on agv {agv}')

    def pickup_part(self, color: int, part_type: int, pose: Pose):
        '''
        Function for picking up a part
        input: color - the color of the part to be picked up
               type - the type of part to be picked up

        output: result (boolean) - True for successful part pickup. False for failure

        Future work:
            This function will need to return a future callback that will provide the result of picking up the part
        '''

        self.get_logger().info(
            f'Picking up {OrderManager._part_colors[color]} {OrderManager._part_types[part_type]} '\
                f'located at [{pose.position.x} {pose.position.y} {pose.position.z}] ' \
                f'[{pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}]')

        return True

    def place_part(self, color: int, part_type: int, tray_id: int, quadrant: int):
        '''
        Function for placing a part
        input: color - the color of the part to be picked up
               type - the type of part to be picked up
               tray_id - the id of the tray to place the part in
               quadrent - the quadrant in the tray to place the part in

        output: result (boolean) - True for successful part pickup. False for failure

        Future work:
            This function will need to return a future callback that will provide the result of picking up the part
        '''

        self.get_logger().info(
            f"Placing {OrderManager._part_colors[color]} {OrderManager._part_types[part_type]} in quadrant {quadrant} in tray {tray_id}")

        return True

    def _move_robot_to_tray(self, tray_id, tray_pose):
        '''
        Move the floor robot to a tray to pick it up
        '''
        self.get_logger().info('👉 Moving robot to tray...')
        self._moving_robot_to_tray = True
        while not self._move_robot_to_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MoveRobotToTray.Request()
        request.tray_id = tray_id
        request.tray_pose_in_world = tray_pose
        future = self._move_robot_to_tray_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_tray_done_cb)

    def _move_robot_to_tray_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_tray

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._moved_robot_to_tray = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _move_robot_to_table(self, table_id):
        '''
        Move the floor robot to a table

        Args:
            table_id (int): 1 for kts1 and 2 for kts2
        '''

        self.get_logger().info('👉 Moving robot to changing station...')
        self._moving_robot_to_table = True
        while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = MoveRobotToTable.Request()
        request.kts = table_id
        future = self._move_robot_to_table_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_table_done_cb)

    def _move_robot_to_table_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_table

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._moved_robot_to_table = True
        else:
            self.get_logger().fatal(f'💀 {message}')