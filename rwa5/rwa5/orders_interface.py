from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from ariac_msgs.msg import Order as OrderMsg, AdvancedLogicalCameraImage as ALCImage
from ariac_msgs.srv import ChangeGripper
from enum import Enum
from functools import partial

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
    
class ChangeGripperClientNode(Node):
    """! The Change Gripper Client base class
    Defines the client to the server - Change Gripper
    """

    def __init__(self, node_name, table_name : TrayStations, gripper_type : GripperTypes):
        """! The ChangeGripperClient base class initializer

        @param table_name   The station name or table name in which tray is located; of type: TrayStations
        @param gripper_type The type of the gripper - part gripper or tray gripper; of type: GripperTypes

        @return  An instance of the class initialized with the specified name.
        """
        super().__init__(node_name)
        self.table_name = table_name
        self.gripper_type = gripper_type
        self.call_change_gripper_server()

    def call_change_gripper_server(self):
        """! calls a service - ChangeGripper - format: "call_servicename_server"

        @param None

        @return None
        """
        
        # Create a client to request the "ChangeGripper" type of service.
        # Service Type: ChangeGripper
        # Service Name: change_gripper
        client = self.create_client(ChangeGripper, "change_gripper")


        # Create a loop that waits for the service server to become available. 
        # Use client.wait_for_service(1.0) to wait for up to 1 second for the service to be ready. 
        # If the service is not available within that time, display a warning message using self.get_logger().warn()
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server - ChangeGripper")
        
        # Create a instance of the service request and provide inputs in the request
        request = ChangeGripper.Request()
        request.gripper_type = self.gripper_type

        # Call the service asynchronously using client.call_async(), 
        # passing the previously created request message. 
        # Return a Future object, which represents the result of the asynchronous call.
        future = client.call_async(request)
        # Add a callback function, to be executed when the service call is completed (whether successfully or with an error). 
        # The partial function is used to pass additional arguments {table_name, gripper_type} to the callback function.
        future.add_done_callback(partial(self.callback_change_gripper, table_name=self.table_name, gripper_type=self.gripper_type))

    def callback_change_gripper(self, future, gripper_type, table_name):
        """! callback to a service - ChangeGripper - format: "call_servicename_server"

        @param future       The future object, which represents result of async call
        @param table_name   The station name or table name in which tray is located; of type: TrayStations
        @param gripper_type The type of the gripper - part gripper or tray gripper; of type: GripperTypes

        @return None
        """

        # Use 'try - except' to handle possible potential exceptions during the service call
        # The try-except block is a way to gracefully handle these situations and prevent the program from crashing.
        try:
            # Retrieve the result of the service call from the Future object and display
            response = future.result()
            self.get_logger().info("Changing to " + str(gripper_type) + " at table: " + str(table_name) + 
                                   "\nMessage from Server: " + response.message +
                                   "\nService Success: " + response.success)
        # If there was an exception during the service call, 
        # this block will catch it and log an error using self.get_logger().error(), 
        # along with the specific exception message (e)
        except Exception as e:
            self.get_logger().error("Service call - change_gripper - failed %r" % (e,))


class ChangeGripperServer(Node):
    """! The Change Gripper Server base class
    Defines the server - ChangeGripper
    """
    def __init__(self, node_name):
        """! The ChangeGripperClient base class initializer

        @param node_name The name of the node

        @return  An instance of the class initialized with the specified name.
        """
        super().__init__(node_name)
        self.server_ = self.create_service(ChangeGripper, "change_gripper", self.callback_change_gripper)
        self.get_logger().info("ChangeGripper server has been started!")

    def callback_change_gripper(self, request, response):
        """! callback to a service - ChangeGripper"

        @param request      The request argument to the service

        @return response    The response from the service
        """

        response.success = True
        response.message = "Changing to " + str(request.gripper_type)
        self.get_logger().info("Changing to " + str(request.gripper_type))

        return response
    