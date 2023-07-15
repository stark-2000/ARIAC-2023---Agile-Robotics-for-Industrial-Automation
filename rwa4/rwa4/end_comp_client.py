
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool as RosBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)


class CompetitionEndingClient(Node):
    '''
    Class for a node that ends the competition once all orders have been submitted

    '''
    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    def __init__(self, node_name):
        '''
        Constructor function for CompetitionEndingClient class

        Args:  node_name (str):  the name the instantiated node will have
        '''
        super().__init__(node_name= node_name)

        client_cb_group = MutuallyExclusiveCallbackGroup()

        #Service client for ending the competition
        self._end_competition_client = self.create_client(
            Trigger, '/ariac/end_competition', callback_group=client_cb_group)
        
        # Subscriber to the competition state topic
        # used for determining if state has reached ORDER_ANNOUNCEMENTS_DONE
        # prints updates to the current state to the CLI
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self._competition_state_cb,
            10)
        
        # Subscriber to the agv_submit topic
        # the submit_orders_interface will publish to this node when all orders are submitted.
        self._submission_status_sub = self.create_subscription(
            RosBool,
            '/agv_submit',
            self._submission_status_cb,
            10)
        
        # Timer is checked every one second to see if competition is done.
        self._timer = self.create_timer(1, self._timer_cb)
        
        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None
        self._order_manager_is_idle: bool = False


    def _competition_state_cb(self, msg: CompetitionStateMsg):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionEndingClient._competition_states[msg.competition_state]
            self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)

        self._competition_state = msg.competition_state


    def _submission_status_cb(self, msg: RosBool):
        '''Callback for the topic /agv_submit

        Arguments:
            msg -- Boolean
            True = The order manager is idle
            False = The order manager is handling an order
        '''
        
        if msg.data == True:
            self._order_manager_is_idle = True
        else:
            self._order_manager_is_idle = False
        
        

    def _timer_cb(self):
        '''
        Callback function for the timer

        Checks the state of the competition.  If parameters are met, the _end_competition routine is called.
        '''
        
        if (self._order_manager_is_idle) and (self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE):
            self.get_logger().info('Ending the competition...')
            self._end_competition()
            


    def _end_competition(self):
        '''
        asyncrinously submits a trigger request to end the competition
        '''
        
        request = Trigger.Request()
        future = self._end_competition_client.call_async(request)
        future.add_done_callback(self.future_callback)


    def future_callback(self, future):
        '''
        Callback function for the future object

        Args:
            future (Future): A future object
        '''
        self.get_logger().info(f'Competition Ended!')
