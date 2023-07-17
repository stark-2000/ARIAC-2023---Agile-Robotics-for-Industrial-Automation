from rclpy.node import Node 
from ariac_msgs.msg import AGVStatus
from ariac_msgs.srv import SubmitOrder
from ariac_msgs.msg import Order as OrderMsg
from std_msgs.msg import Bool


class Submit_Orders(Node):
    """
    This is a class for service client functions for the ariac/submit_order, 
    publisher for order submission status, subscriber for ariac/agv*_status
    """

    def __init__(self):
        """
        The constructor for initializing the node & creating the service client, publisher & subscriber
        """

        self.node_name = 'submit_orders'
        super().__init__(self.node_name)

        #Dict to store the order ids and Array to store executed agvs
        self.order_id_array = {}
        self.executed_agvs = []
        
        #Create publisher for order submission status to topic "agv_submit"
        self.publisher_ = self.create_publisher(Bool, 'agv_submit', 10) 
        

        #Create subscriber to topic "ariac/orders" to get the order ids
        self.order_info_subscriber = self.create_subscription(
            OrderMsg,
            'ariac/orders',
            self.order_callback,
            10) 
        

        #Create client for service "ariac/submit_order"
        self.cli = self.create_client(SubmitOrder, 'SubmitOrder')
        self.req = SubmitOrder.Request()


        #Create subscriber to topic "ariac/agv1_status" to get the status of agv1
        self.agv1_status_subscriber = self.create_subscription(
            AGVStatus,
            'ariac/agv1_status',
            lambda msg: self.listener_callback(msg, agv_no=1),
            10) 

        #Create subscriber to topic "ariac/agv2_status" to get the status of agv2
        self.agv2_status_subscriber = self.create_subscription(
            AGVStatus,
            'ariac/agv2_status',
            lambda msg: self.listener_callback(msg, agv_no=2),
            10) 
        
        #Create subscriber to topic "ariac/agv3_status" to get the status of agv3
        self.agv3_status_subscriber = self.create_subscription(
            AGVStatus,
            'ariac/agv3_status',
            lambda msg: self.listener_callback(msg, agv_no=3),
            10) 

        

    def send_request(self, order_id):
        """
        The client function to send the request to the service server

        Parameters:
            order_id (string): The order id to be submitted
        """

        self.req.order_id = order_id
        self.future = self.cli.call_async(self.req)

    

    def order_callback(self, order_msg):
        """
        The callback function for the "order_info_subscriber" to get the order ids

        Parameters:
            order_msg (object): The message object received from the topic which contains the order id
        """

        self.order_id_array[order_msg.kitting_task.agv_number] = order_msg.id



    def listener_callback(self, msg, agv_no):
        """
        The callback function for the "agv*_status_subscriber" to get the status of the agvs

        Parameters:
            msg (object): The message object received from the topic which contains the status of the agv
            agv_no (int): The number of the agv is known from the topic name
        """

        #if the order is already submitted, return
        if agv_no in self.executed_agvs: 
            return
    
        #if the agv is at the Warehouse, send the request to submit the order
        if msg.location == AGVStatus.WAREHOUSE:
            self.get_logger().info('AGV ' + str(agv_no) + ' is at the Warehouse')

            if agv_no in self.order_id_array:
                self.send_request(self.order_id_array[agv_no])
            
            self.executed_agvs.append(agv_no) #add the agv to the executed agvs list once the order is submitted

            self.get_logger().info('Executed AGVs: ' + str(self.executed_agvs))

            #call the publish fxn to publish the final status once all orders are submitted
            self.publish_final_sub_status(self.executed_agvs) 
            
        

    def publish_final_sub_status(self, _executed_agvs):
        """
        The function to check if all orders are submitted & publish the final status

        Parameters:
            _executed_agvs (list): The list of agvs which have submitted the orders
        """
        
        shipped_and_submitted = Bool()
        
        #If all 3 AGVs have submitted the orders, publish the final status
        if len(_executed_agvs) == 3:
            shipped_and_submitted.data = True
            self.get_logger().info('All orders submitted')
            self.publisher_.publish(shipped_and_submitted)
