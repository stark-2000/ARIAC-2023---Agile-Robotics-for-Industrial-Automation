
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool as RosBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)


class CompetitionEndingClient(Node):

    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    def __init__(self, node_name):
        super().__init__(node_name= node_name)

        client_cb_group = MutuallyExclusiveCallbackGroup()

        #Service client for ending the competition
        self._end_competition_client = self.create_client(
            Trigger, '/ariac/end_competition', callback_group=client_cb_group)
        
        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self._competition_state_cb,
            10)
        
        self._manager_sub = self.create_subscription(
            RosBool,
            '/manager_idle',
            self._order_manager_cb,
            10)
        
        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None


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


    def _order_manager_cb(self, msg: RosBool):
        '''Callback for the topic /manager_idle

        Arguments:
            msg -- Boolean
            True = All orders have been shipped. End competition
        '''
      
        if msg.data == True:
            self._end_competition()
        else:
            self.get_logger().error("Order Manager returned False")



    def _end_competition(self):
        
        if self._competition_state == CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE:
            request = Trigger.Request
            future = self._end_competition_client.call_async(request)
        else:
            self.get_logger().info('end_competition called before "order announcements done"')
            return

        future.add_done_callback(self.future_callback)


    def future_callback(self, future):
        '''
        Callback function for the future object

        Args:
            future (Future): A future object
        '''
        self.get_logger().info(f'Competition Ended')
