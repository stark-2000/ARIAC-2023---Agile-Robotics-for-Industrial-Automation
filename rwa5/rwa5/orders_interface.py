from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from ariac_msgs.msg import Order as OrderMsg, AdvancedLogicalCameraImage as ALCImage
from enum import Enum

class GripperTypes(Enum):
    TRAY_GRIPPER = 'tray gripper'
    PART_GRIPPER = 'part gripper'

class TrayStations(Enum):
    KTS_1 = 'table 1'
    KTS_2 = 'table 2'


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

    def __init__(self, node_name):
        super().__init__(node_name)
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
        self._paused_order =  None
        # Subscriber to listen for orders
        self._order_subscriber = self.create_subscription(
            OrderMsg, 'ariac/orders', self.order_callback, 10)
        # Publisher to notify that order is ready to ship
        self._order_ship_publisher = self.create_publisher(
            UInt8, 'ship_order', 10)

        # Subscribers to receive updated poses with world frame
        self._left_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/left_bin_camera_data', self.left_bin_callback, qos_profile_sensor_data)
        
        self._right_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/right_bin_camera_data', self.right_bin_callback, qos_profile_sensor_data)
        
        self._kts1_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray1_camera_data', self.kts1_callback, qos_profile_sensor_data)
        
        self._kts2_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray2_camera_data', self.kts2_callback, qos_profile_sensor_data)

    def left_bin_callback(self, camera_msg: ALCImage):
        for part_pose in camera_msg.part_poses:
            self._left_bin_inventory.append(part_pose)

    def right_bin_callback(self, camera_msg: ALCImage):
        for part_pose in camera_msg.part_poses:
            self._right_bin_inventory.append(part_pose)

    def kts1_callback(self, camera_msg: ALCImage):
        for tray_pose in camera_msg.tray_poses:
            self._kts1_inventory.append(tray_pose)

    def kts2_callback(self, camera_msg: ALCImage):
        for tray_pose in camera_msg.tray_poses:
            self._kts2_inventory.append(tray_pose)


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
        else:
            # If incoming order is priority, add it to priority queue.
            # If priority queue is empty and current order is not priority,
            # replace current order with incoming one, and move current order back to the front of the order queue.
            if order.order_priority:
                if len(self._priority_queue == 0):
                    if not self._current_order.order_priority:
                        self._paused_order = self._current_order
                        # self._order_queue.appendleft(self._current_order)
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
        # For this assignment, order is just set to be completed as soon as it is added
        self.complete_order()

    def next_order(self):
        """
        Moves current order to start working on the next order
        """
        # If there is a priority order, start working on that next
        if len(self._priority_queue) > 0:
            self._current_order = self._priority_queue.popleft()
        # If an order has been paused, set as current order
        elif self._paused_order != None:
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
        self.get_logger().info(
            f'{self._current_order}')
        self._order_ship_publisher.publish(ship_msg)
        # Move to start the next order
        self.next_order()
    
    def fulfill_order(self):
        order = self._current_order
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
            self.get_logger().fatal(f"Tray {target_tray} not found. Can not complete order {order.order_id}")
            return False
        
        # Change gripper to tray
        self.change_gripper(station, GripperTypes.TRAY_GRIPPER)
        self.pick_up_tray(target_tray.id, tray_pose)
        self.place_tray(target_tray, target_agv)

        # Change gripper to part
        self.change_gripper(station, GripperTypes.PART_GRIPPER)
        for order_part in order.parts:
            part_found = False
            for item in self._left_bin_inventory:
                if item.part == order_part.part:
                    self.pick_up_part(item.part.color, item.part.type, item.part.pose)
                    part_found = True
                    break
            if not part_found:
                for item in self._right_bin_inventory:
                    if item.part == order_part.part:
                        self.pick_up_part(item.part.color, item.part.type, item.part.pose)
                        part_found = True
                        break
            if not part_found:
               self.get_logger().fatal(f"Part not found. Can not complete order {order.order_id}")
               return False
            self.place_part(order_part.color, order_part.type, order_part.quadrant)

        