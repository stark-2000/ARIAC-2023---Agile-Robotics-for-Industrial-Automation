import time
import rclpy
from rclpy.parameter import Parameter
from collections import deque
from enum import Enum
from functools import partial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from ariac_msgs.msg import (
    Order as OrderMsg,
    AdvancedLogicalCameraImage as ALCImage,
    CompetitionState,
    Part,
    KittingPart)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from ariac_msgs.srv import (
    ChangeGripper,
    VacuumGripperControl,
    PerformQualityCheck
)

# Import custom ROS services
from robot_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    MoveRobotToPart,
    MovePartToAGV,
    DiscardPart,
    DropTray,
    DropPart
)


class GripperTypes(Enum):
    TRAY_GRIPPER = 'trays'
    PART_GRIPPER = 'parts'


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
        self._dropping_tray = False
        self._dropping_part = False

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
        self._moved_robot_to_part = False
        self._moved_part_to_agv = False
        self._parts_quality_check = False
        self._dropped_tray = False
        self._dropped_part = False
        self._discarded_part = False

        # flags for errors
        self._failure = False

        # Dictionary for informing the faulty part quadrant
        self._faulty_parts_quadrant = {
            KittingPart.QUADRANT1: False,
            KittingPart.QUADRANT2: False,
            KittingPart.QUADRANT3: False,
            KittingPart.QUADRANT4: False
        }

        self._part_colors = {
            Part.RED: 0,
            Part.BLUE: 1,
            Part.GREEN: 2,
            Part.ORANGE: 3,
            Part.PURPLE: 4,
        }
        self._part_types = {
            Part.BATTERY: 10,
            Part.PUMP: 11,
            Part.REGULATOR: 12,
            Part.SENSOR: 13,
        }
        # ----------- Callback Groups ----------

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        self._fulfillment_callback_group = MutuallyExclusiveCallbackGroup()

        # ---------- Publishers ---------------

        # Publisher to notify that order is ready to ship
        self._order_ship_publisher = self.create_publisher(
            UInt8, 'ship_order', 10)

        # ---------- Subscribers --------------

        # Subscriber to listen for orders
        self._order_subscriber = self.create_subscription(
            OrderMsg, 'ariac/orders', self.order_callback, 10,)

        self._competition_state_subscriber = self.create_subscription(
            CompetitionState, 'ariac/competition_state', self.comp_state_callback, 10,
            callback_group=self._fulfillment_callback_group)

        # Subscribers to receive updated poses with world frame
        self._left_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/left_bin_camera_data', self.left_bin_callback, qos_profile_sensor_data)

        self._right_bin_parts_data_subscriber = self.create_subscription(
            ALCImage, '/right_bin_camera_data', self.right_bin_callback, qos_profile_sensor_data)

        self._kts1_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray1_camera_data', self.kts1_callback, qos_profile_sensor_data)

        self._kts2_tray_data_subscriber = self.create_subscription(
            ALCImage, '/kitting_tray2_camera_data', self.kts2_callback, qos_profile_sensor_data)

        # ---------- Clients -------------

        # client to start the competition
        self._start_competition_cli = self.create_client(
            Trigger,
            '/ariac/start_competition')

        # client to move the floor robot to the home position
        self._move_robot_home_cli = self.create_client(
            Trigger,
            '/commander/move_robot_home')

        # client to move a robot to a table
        self._move_robot_to_table_cli = self.create_client(
            MoveRobotToTable,
            '/commander/move_robot_to_table')

        # client to move a robot to a part
        self._move_robot_to_part_cli = self.create_client(
            MoveRobotToPart,
            '/commander/move_robot_to_part')

        # client to move a part to a AGV
        self._move_part_to_agv_cli = self.create_client(
            MovePartToAGV,
            '/commander/move_part_to_agv')

        self._drop_part_cli = self.create_client(
            DropPart,
            '/commander/drop_part')

        # client to move a robot to a table
        self._move_robot_to_tray_cli = self.create_client(
            MoveRobotToTray,
            '/commander/move_robot_to_tray')

        # client to move a tray to an agv
        self._move_tray_to_agv_cli = self.create_client(
            MoveTrayToAGV,
            '/commander/move_tray_to_agv')

        self._drop_tray_cli = self.create_client(
            DropTray,
            '/commander/drop_tray')

        # client to move the end effector inside a tool changer
        self._enter_tool_changer_cli = self.create_client(
            EnterToolChanger,
            '/commander/enter_tool_changer')

        # client to move the end effector outside a tool changer
        self._exit_tool_changer_cli = self.create_client(
            ExitToolChanger,
            '/commander/exit_tool_changer')

        # client to discard a part
        self._discard_part_cli = self.create_client(
            DiscardPart,
            '/commander/discard_part')

        # client to activate/deactivate the vacuum gripper
        self._set_gripper_state_cli = self.create_client(
            VacuumGripperControl,
            '/ariac/floor_robot_enable_gripper')

        # client to change the gripper type
        # the end effector must be inside the tool changer before calling this service
        self._change_gripper_cli = self.create_client(
            ChangeGripper,
            '/ariac/floor_robot_change_gripper')

        # client to change the gripper type
        # the end effector must be inside the tool changer before calling this service
        self._parts_quality_check_cli = self.create_client(
            PerformQualityCheck,
            '/ariac/perform_quality_check')

        # ---------- Timer --------------

        # Timer to trigger the robot actions
        # Every second the timer callback is called and based on the flags the robot actions are triggered
        # self._robot_action_timer = self.create_timer(1,
        #                                              self._robot_action_cb,
        #                                              callback_group=timer_cb_group)

    #  Callback functions for receiving camera images/world poses

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

    # function to start the competition
    # and continuously call fulfill_order
    def comp_state_callback(self, state_msg: CompetitionState):
        if (state_msg.competition_state == CompetitionState.READY):
            self._started_orders = True
            self._competition_started = True
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
                self._kts1_inventory.remove(tray)
                tray_pose = tray.pose
                station = TrayStations.KTS_1
                break
        if tray_pose == None:
            for tray in self._kts2_inventory:
                if tray.id == target_tray:
                    self._kts2_inventory.remove(tray)
                    tray_pose = tray.pose
                    station = TrayStations.KTS_2
                    break
        if station == None:
            self.get_logger().fatal(
                f"Tray {target_tray} not found. Can not complete order {order.order_id}")
            return False

        self._move_robot_to_table(station.value)
        while (not self._moved_robot_to_table):
            continue
        self._moved_robot_to_table = False

        self._enter_tool_changer(
            "kts" + str(station.value), GripperTypes.TRAY_GRIPPER.value)
        while (not self._entered_tool_changer):
            continue
        self._entered_tool_changer = False

        self._change_gripper(ChangeGripper.Request.TRAY_GRIPPER)
        while (not self._changed_gripper):
            continue
        self._changed_gripper = False

        self._exit_tool_changer(
            "kts" + str(station.value), GripperTypes.TRAY_GRIPPER.value)
        while (not self._exited_tool_changer):
            continue
        self._exited_tool_changer = False

        self._activate_gripper()
        while (not self._activated_gripper):
            continue
        self._activated_gripper = False

        self._move_robot_to_tray(target_tray, tray_pose)
        while (not self._moved_robot_to_tray):
            continue
        self._moved_robot_to_tray = False

        self._move_tray_to_agv(target_agv)
        while (not self._moved_tray_to_agv):
            if(self._failure):
                self.get_logger().info("Moving tray to ATV failed. Trying agian...")
                self._move_tray_to_agv(target_agv)
                self._failure = False
            continue
        self._moved_tray_to_agv = False

        self._deactivate_gripper()
        while (not self._deactivated_gripper):
            continue
        self._deactivated_gripper = False

        self._drop_tray(target_tray, target_agv)
        while (not self._dropped_tray):
            continue
        self._dropped_tray = False
        if (True in (item.part == order.parts[0].part for item in self._left_bin_inventory)):
            closest_table = TrayStations.KTS_1.value
        else:
            closest_table = TrayStations.KTS_2.value

        self._move_robot_to_table(closest_table)
        while (not self._moved_robot_to_table):
            continue
        self._moved_robot_to_table = False

        self._enter_tool_changer(
            "kts" + str(closest_table), GripperTypes.PART_GRIPPER.value)
        while (not self._entered_tool_changer):
            continue
        self._entered_tool_changer = False

        self._change_gripper(ChangeGripper.Request.PART_GRIPPER)
        while (not self._changed_gripper):
            continue
        self._changed_gripper = False

        self._exit_tool_changer(
            "kts" + str(closest_table), GripperTypes.PART_GRIPPER.value)
        while (not self._exited_tool_changer):
            continue
        self._exited_tool_changer = False

        order_parts = deque(order.parts)
        while (len(order_parts) > 0):
            order_part = order_parts.popleft()
            self._activate_gripper()
            while (not self._activated_gripper):
                continue
            self._activated_gripper = False
            part_found = False
            bin_location = None
            for item in self._left_bin_inventory:
                if item.part == order_part.part:
                    part_found = True
                    bin_location = MoveRobotToPart.Request.BIN_LEFT
                    self._left_bin_inventory.remove(item)
                    break
            if not part_found:
                for item in self._right_bin_inventory:
                    if item.part == order_part.part:
                        part_found = True
                        bin_location = MoveRobotToPart.Request.BIN_RIGHT
                        self._right_bin_inventory.remove(item)
                        break
            if not part_found:
                self.get_logger().warn(
                    f"Part not found. Order {order.order_id} will be incomplete")
                self._deactivate_gripper()
                while (not self._deactivated_gripper):
                    continue
                self._deactivated_gripper = False
                continue
            self.pick_part(
                self._part_colors[item.part.color], self._part_types[item.part.type], item.pose, bin_location)
            while (not self._moved_robot_to_part):
                continue
            self._moved_robot_to_part = False

            self.place_part(target_agv, order_part.quadrant)
            while (not self._moved_part_to_agv):
                continue
            self._moved_part_to_agv = False

            self.perform_quality_check(order.order_id)
            while (not self._parts_quality_check):
                continue
            self._parts_quality_check = False

            if self._faulty_parts_quadrant[order_part.quadrant]:
                self.get_logger().warn(
                    f"Faulty Part found.")
                self._discard_part(target_agv, order_part.quadrant)
                while (not self._discarded_part):
                    continue
                self._discarded_part = False
                self._deactivate_gripper()
                while (not self._deactivated_gripper):
                    continue
                self._deactivated_gripper = False
                order_parts.appendleft(order_part)
                self.get_logger().warn(
                    f"put part back in list")
                continue
            else:
                self.get_logger().warn(
                    f"No Faulty Part found.")

            self._deactivate_gripper()
            while (not self._deactivated_gripper):
                continue
            self._deactivated_gripper = False

            self._drop_part(target_agv, order_part.quadrant)
            while (not self._dropped_part):
                continue
            self._dropped_part = False

            self._drop_part(target_agv, order_part.quadrant)
            while(not self._dropped_part):
                continue
            self._dropped_part = False

        #     self.place_part(order_part.part.color, order_part.part.type, order.tray_id,
        #                     order_part.quadrant)
        self.complete_order()

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
            self._table_path_failed = True
            self.get_logger().fatal(f'💀 {message}')

    def _enter_tool_changer(self, station, gripper_type):
        '''
        Move the end effector inside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Entering tool changer...')
        self._entering_tool_changer = True
        self.get_logger().info(gripper_type)
        while not self._enter_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = EnterToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._enter_tool_changer_cli.call_async(request)
        future.add_done_callback(self._enter_tool_changer_done_cb)

    def _enter_tool_changer_done_cb(self, future):
        '''
        Client callback for the service /commander/enter_tool_changer

        output: result (boolean) - True for successful part pickup. False for failure
        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._entered_tool_changer = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _change_gripper(self, gripper_type):
        '''
        Change the gripper
        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Changing gripper...')
        self._changing_gripper = True
        while not self._change_gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = ChangeGripper.Request()
        request.gripper_type = gripper_type
        future = self._change_gripper_cli.call_async(request)
        future.add_done_callback(self._change_gripper_done_cb)

    def _change_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_change_gripper

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info('✅ Gripper changed')
            self._changed_gripper = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def perform_quality_check(self, order_id):
        '''
        performs quality check on the parts placed on AGV
        Args:
            order_id (str): Order ID
        '''
        self.get_logger().info('👉 Performing quality check...')

        # Reset Quality check quadrants dictionary
        self._faulty_parts_quadrant[KittingPart.QUADRANT1] = False
        self._faulty_parts_quadrant[KittingPart.QUADRANT2] = False
        self._faulty_parts_quadrant[KittingPart.QUADRANT3] = False
        self._faulty_parts_quadrant[KittingPart.QUADRANT4] = False

        self._parts_quality_check = False

        while not self._parts_quality_check_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = PerformQualityCheck.Request()
        request.order_id = order_id
        future = self._parts_quality_check_cli.call_async(request)
        future.add_done_callback(self._quality_check_done_cb)

    def _quality_check_done_cb(self, future):
        '''
        Client callback for the service /ariac/perform_quality_check

        Args:
            future (Future): A future object
        '''
        quality_status = future.result().all_passed
        valid_id = future.result().valid_id
        incorrect_tray = future.result().incorrect_tray

        if quality_status:
            self.get_logger().info('✅ Quality Check Passed.')
        else:
            self.get_logger().fatal(f'💀 Quality check : Faulty detected.')
            if future.result().quadrant1.faulty_part:
                self._faulty_parts_quadrant[KittingPart.QUADRANT1] = True
                self.get_logger().fatal(f'💀 Quality check : Faulty Part in Q1')
            if future.result().quadrant2.faulty_part:
                self._faulty_parts_quadrant[KittingPart.QUADRANT2] = True
                self.get_logger().fatal(f'💀 Quality check : Faulty Part in Q2')
            if future.result().quadrant3.faulty_part:
                self._faulty_parts_quadrant[KittingPart.QUADRANT3] = True
                self.get_logger().fatal(f'💀 Quality check : Faulty Part in Q3')
            if future.result().quadrant4.faulty_part:
                self._faulty_parts_quadrant[KittingPart.QUADRANT4] = True
                self.get_logger().fatal(f'💀 Quality check : Faulty Part in Q4')

        self._parts_quality_check = True

    def pick_part(self, color: int, part_type: int, pose: Pose, bins_location):
        '''
        Moves the floor robot to bin and pick up the part
        Args:
            agv_number (int): AGV number
            quadrant (int): Quadrant to which the part belongs to
        '''
        self.get_logger().info('👉 Moving robot to part...')
        self._moved_robot_to_part = False
        while not self._move_robot_to_part_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MoveRobotToPart.Request()
        request.color = color
        request.type = part_type
        request.part_pose_in_world = pose
        request.bin_location = bins_location
        future = self._move_robot_to_part_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_part_done_cb)

    def _move_robot_to_part_done_cb(self, future):
        '''
        Client callback for the service /commander/move_robot_to_part

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._moved_robot_to_part = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _exit_tool_changer(self, station, gripper_type):
        '''
        Move the end effector outside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        '''
        self.get_logger().info('👉 Exiting tool changer...')
        self._exiting_tool_changer = True
        while not self._exit_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        '''
        output: result (boolean) - True for successful part pickup. False for failure
        '''
        request = ExitToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._exit_tool_changer_cli.call_async(request)
        future.add_done_callback(self._exit_tool_changer_done_cb)

    def _exit_tool_changer_done_cb(self, future):
        '''
        Client callback for the service /commander/exit_tool_changer
        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._exited_tool_changer = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _activate_gripper(self):
        '''
        Activate the gripper
        '''
        self.get_logger().info('👉 Activating gripper...')
        self._activating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = True
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._activate_gripper_done_cb)

    def _activate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper activated')
            self._activated_gripper = True
        else:
            self.get_logger().fatal('💀 Gripper not activated')

    def _deactivate_gripper(self):
        '''
        Deactivate the gripper
        '''
        self.get_logger().info('👉 Deactivating gripper...')
        self._deactivating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = VacuumGripperControl.Request()
        request.enable = False
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._deactivate_gripper_done_cb)

    def _deactivate_gripper_done_cb(self, future):
        '''
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        '''
        if future.result().success:
            self.get_logger().info('✅ Gripper deactivated')
            self._deactivated_gripper = True
        else:
            self.get_logger().fatal('💀 Gripper not deactivated')

    def place_part(self, agv_number: int, quadrant: int):
        '''
        Move floor robot to agv and place the part
        Args:
            agv_number (int): AGV number
            quadrant (int): Quadrant to which the part belongs to
        '''
        self.get_logger().info('👉 Move the robot and place part on AGV...')
        self._moved_part_to_agv = False
        while not self._move_part_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available, waiting...')

        request = MovePartToAGV.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._move_part_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_part_to_agv_done_cb)

    def _move_part_to_agv_done_cb(self, future):
        '''
        Client callback for the service /commander/move_part_to_agv

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._moved_part_to_agv = True
        else:
            self.get_logger().fatal(f'💀 {message}')

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

    # @brief Move the floor robot to its home position
    def _move_tray_to_agv(self, agv_number):
        self._moving_tray_to_agv = True

        while not self._move_tray_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = MoveTrayToAGV.Request()
        request.agv_number = agv_number
        future = self._move_tray_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_tray_to_agv_done_cb)

    def _move_tray_to_agv_done_cb(self, future):
        '''
        Client callback for the service /commander/move_tray_to_agv

        Args:
            future (Future): A future object
        '''
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._moved_tray_to_agv = True
        else:
            self.get_logger().fatal(f'💀 {message}')
            self._failure = True

    def _drop_tray(self, tray_id, agv_number):
        self._dropping_tray = True

        while not self._drop_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = DropTray.Request()
        request.tray_id = tray_id
        request.agv_number = agv_number
        future = self._drop_tray_cli.call_async(request)
        future.add_done_callback(self._drop_tray_done)

    def _drop_tray_done(self, future):
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._dropped_tray = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _drop_part(self, agv_number, quadrant):
        self._dropping_part = True

        while not self._drop_part_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = DropPart.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._drop_part_cli.call_async(request)
        future.add_done_callback(self._drop_part_done)

    def _drop_part_done(self, future):
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._dropped_part = True
        else:
            self.get_logger().fatal(f'💀 {message}')

    def _discard_part(self, agv_number, quadrant):

        while not self._discard_part_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = DiscardPart.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._discard_part_cli.call_async(request)
        future.add_done_callback(self._discard_part_done)

    def _discard_part_done(self, future):
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f'✅ {message}')
            self._discarded_part = True
        else:
            self.get_logger().fatal(f'💀 {message}')
