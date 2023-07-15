from collections import deque
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from ariac_msgs.msg import Order as OrderMsg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


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
        idle_group = MutuallyExclusiveCallbackGroup()
        submission_group = MutuallyExclusiveCallbackGroup()
        # Queue to hold regular orders
        self._order_queue = deque()
        # Queue to hold priority orders
        self._priority_queue = deque()
        self._curent_order = None
        self._agvs_in_progress = []
        # Subscriber to listen for orders
        self._order_subscriber = self.create_subscription(
            OrderMsg, 'ariac/orders', self.order_callback, 10)
        # Publisher to notify that order is ready to ship
        self._order_ship_publisher = self.create_publisher(
            UInt8, 'ship_order', 10)
        # Subscriber to listen for notice of an agv submitting an order
        self._submission_subscriber = self.create_subscription(
            UInt8, 'agv_submit', self.order_submitted, 10, callback_group=submission_group)
        # Publisher to notify if manager is currently idle, no orders in process
        self._idle_publisher = self.create_publisher(
            Bool, 'manager_idle', 10,  callback_group=idle_group)
        self._idle_timer = self.create_timer(
            1/10, self.idle_callback, callback_group=idle_group)

    def order_callback(self, order_msg: OrderMsg):
        """
        Callback for when an Order is published.

        Args:
            order (Order): A single Order
        """
        # Create an Order object from the msg
        order = Order(order_msg)
        # If not currently working on an order, set new order to current
        if self._curent_order is None:
            self._curent_order = order
        else:
            # If incoming order is priority, add it to priority queue.
            # If priority queue is empty and current order is not priority,
            # replace current order with incoming one,
            # and move current order back to the front of the order queue.
            if order.order_priority:
                if len(self._priority_queue == 0):
                    if not self._curent_order.order_priority:
                        self._order_queue.appendleft(self._curent_order)
                        self._curent_order = order
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
            self._curent_order = self._priority_queue.popleft()
        # If there are orders in regular queue, start working on the next order
        if len(self._order_queue) != 0:
            self._order_queue.popleft()
        # If there are no orders, set current order to None
        else:
            self._curent_order = None

    def idle_callback(self):
        """
        Function to check queue status

        Returns:
            bool: Returns true if both queues are empty, false otherwise
        """
        # Boolean containing emptiness of both queues, true is both are empty, false otherwise
        queues_empty = len(self._priority_queue) == 0 and len(
            self._order_queue) == 0
        # Boolean containing emptiness of list with agvs in progress of submission
        # True is list is empty, false wtherwise
        no_orders_submitting = len(self._agvs_in_progress) == 0
        # Idle status is a Bool message containing true if both previous variables are true
        idle_status = Bool(data=queues_empty and no_orders_submitting)
        # Publish Bool message
        self._idle_publisher.publish(idle_status)

    def complete_order(self):
        """_
        Function to notify next stage that an agv is ready to be shipped.
        """
        # Create message containing agv number of current order and publish it
        ship_msg = UInt8(data=self._curent_order.agv_number)
        self._agvs_in_progress.append(self._curent_order.agv_number)
        self.get_logger().info(
            f'Sending {self._curent_order.agv_number} to submit order.')
        self._order_ship_publisher.publish(ship_msg)
        self.get_logger().info(
            f'Number of orders in progress of submission: {len(self._agvs_in_progress)}')
        # Move to start the next order
        self.next_order()

    def order_submitted(self, msg: UInt8):
        """
        Function to update list of agvs in progress of order shipping and submission
        Args:
            msg (UInt8): Message containg the agv number of the order that was just submitted
        """
        # Grab the agv number from the message
        order_agv_num = msg.data
        # Remove it from the list
        try:
            self._agvs_in_progress.remove(order_agv_num)
            self.get_logger().info(
            f'Agv {order_agv_num} sucessfully submitted its order.')
            self.get_logger().info(
            f'Number of orders in progress of submission: {len(self._agvs_in_progress)}')
        except ValueError:
            self.get_logger().warn(
                f'Agv {order_agv_num} was reported as submitted, but was not in list of orders in progress')
