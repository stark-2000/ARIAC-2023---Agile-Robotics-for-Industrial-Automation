from collections import deque
from rclpy.node import Node
from std_msgs.msg import UInt8
from ariac_msgs.msg import Order as OrderMsg


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
        self._curent_order = None
        # Subscriber to listen for orders
        self._order_subscriber = self.create_subscription(
            OrderMsg, 'order', self.order_callback, 10)
        # Publisher to notify that order is ready to ship
        self._order_ship_publisher = self.create_publisher(
            UInt8, 'ship_order', 10)

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
            # replace current order with incoming one, and move current order back to the front of the order queue.
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
        ship_msg = UInt8(data= self._curent_order.agv_number)
        self.get_logger().info(
            f'{self._curent_order}')
        self._order_ship_publisher.publish(ship_msg)
        # Move to start the next order
        self.next_order()
