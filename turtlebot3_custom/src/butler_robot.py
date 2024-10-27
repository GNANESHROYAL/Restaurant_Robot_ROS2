#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
from collections import deque

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')

        # Publishers and Subscribers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(String, '/task_topic', self.task_callback, 10)
        self.position_subscription = self.create_subscription(Odometry, '/odom', self.position_callback, 10)
        self.confirmation_subscription = self.create_subscription(String, '/confirmation_topic', self.confirmation_callback, 10)
        self.cancellation_subscription = self.create_subscription(String, '/cancellation_topic', self.cancellation_callback, 10)

        # Timeout settings
        self.timeout_duration = 5  # seconds
        self.remaining_timeout = self.timeout_duration
        self.timeout_timer = None

        # State variables
        self.task_queue = deque()
        self.current_task = None
        self.state = 'home'
        self.task_in_progress = False
        self.waiting_for_confirmation = False
        self.waiting_started = False
        self.confirmation_received = False
        self.current_position = None
        self.goal_position = None
        self.returning_home = False
        self.returning_kitchen = False
        self.timeout_reached = False
        self.unconfirmed_deliveries = False  # New flag to track unconfirmed deliveries

    def task_callback(self, msg):
        tasks = msg.data.split(',')
        for task in tasks:
            task = task.strip().lower()
            if task and task not in self.task_queue:
                self.task_queue.append(task)
        self.get_logger().info(f'Received tasks: {list(self.task_queue)}')
        
        if not self.task_in_progress and not self.returning_home:
            self.unconfirmed_deliveries = False  
            self.execute_task()

    def execute_task(self):
        if self.state == 'home':
            self.move_to_kitchen()
            self.current_task = self.task_queue.popleft()
        elif self.state == 'kitchen' and self.task_queue:
            self.current_task = self.task_queue.popleft()
            self.move_to_table(self.current_task)
        elif self.state == self.current_task and self.confirmation_received:
            self.get_logger().info(f'Delivery complete at {self.current_task}.')
            self.confirmation_received = False
            self.stop_timeout()
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_table(self.current_task)
            else:
                if self.unconfirmed_deliveries:
                    self.return_to_kitchen()
                else:
                    self.return_home()

    def confirmation_callback(self, msg):
        confirmation_point = msg.data.lower()
        if self.waiting_for_confirmation and not self.confirmation_received:
            if confirmation_point == 'kitchen' and self.state == 'kitchen':
                self.get_logger().info('Kitchen confirmation received.')
                self.confirmation_received = True
                self.waiting_for_confirmation = False
                if self.timeout_timer:
                    self.timeout_timer.cancel()
                self.proceed_to_next_phase()
            elif confirmation_point == self.current_task:
                self.get_logger().info(f'Table confirmation received: {confirmation_point}.')
                self.confirmation_received = True
                self.waiting_for_confirmation = False
                if self.timeout_timer:
                    self.timeout_timer.cancel()
                self.proceed_to_next_phase()

    def cancellation_callback(self, msg):
        cancellation_task = msg.data.lower()
        if cancellation_task in self.task_queue:
            self.task_queue.remove(cancellation_task)
            self.get_logger().info(f'Task {cancellation_task} canceled and removed from queue.')
        elif self.current_task == cancellation_task:
            self.get_logger().info(f'Current task {cancellation_task} canceled.')
            self.stop_timeout()
            self.waiting_for_confirmation = False
            self.confirmation_received = False
            self.task_in_progress = False
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_table(self.current_task)
                self.get_logger().info(f'Moving to next task: {self.current_task}')
            else:
                if self.state != 'kitchen':
                    self.return_to_kitchen()
                else:
                    self.return_home()

    def position_callback(self, msg):
        self.current_position = msg.pose.pose

        if self.goal_position and self.has_reached_goal():
            if self.returning_home:
                self.get_logger().info('Robot has returned home. Task complete.')
                self.end_task()
            elif self.returning_kitchen:
                self.get_logger().info('Robot has returned to kitchen. Heading home next.')
                self.return_home()
            elif not self.waiting_for_confirmation and self.state != 'home':
                self.get_logger().info('Robot has reached its goal.')
                self.task_in_progress = False
                self.start_waiting_for_confirmation()
                self.waiting_started = False

    def has_reached_goal(self):
        tolerance = 0.4
        distance = math.sqrt(
            (self.goal_position.position.x - self.current_position.position.x) ** 2 +
            (self.goal_position.position.y - self.current_position.position.y) ** 2
        )
        return distance < tolerance

    def move_to_kitchen(self):
        self.publish_goal(-3.001096725463867, 1.293923020362854, 0.002471923828125)
        self.state = 'kitchen'
        self.task_in_progress = True
        self.get_logger().info('Moving to kitchen')

    def move_to_table(self, table):
        table_coords = {
            'table1': (0.6848546266555786, -0.13853590190410614, 0.002471923828125),
            'table2': (3.0809290409088135, 0.06948712468147278, 0.002471923828125),
            'table3': (4.854504108428955, 0.06061259284615517, 0.002471923828125)
        }
        self.get_logger().info(f'table:{self.current_task}')
        if table in table_coords:
            x, y, z = table_coords[table]
            self.publish_goal(x, y, z)
            self.state = table
            self.task_in_progress = True
            self.get_logger().info(f'Moving to {table}')
        else:
            self.get_logger().info(f'Unknown table: {table}')

    def return_home(self):
        self.get_logger().info('Returning home')
        home_x, home_y, home_z = 5.496074199676514, 2.938681125640869, 0.11181640625
        self.publish_goal(home_x, home_y, home_z)
        self.returning_home = True
        self.task_in_progress = True
        self.state = 'home'

    def return_to_kitchen(self):
        if not self.returning_kitchen:
            self.get_logger().info('Returning to kitchen before heading home.')
            self.publish_goal(-3.001096725463867, 1.293923020362854, 0.002471923828125)
            self.returning_kitchen = True
            self.state = 'kitchen'
            self.task_in_progress = True

    def publish_goal(self, x, y, z):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal_msg)
        self.goal_position = goal_msg.pose
        self.get_logger().info(f'Published goal to x: {x}, y: {y}, z: {z}')

    def start_waiting_for_confirmation(self):
        if not self.waiting_started:
            self.waiting_for_confirmation = True
            self.remaining_timeout = self.timeout_duration
            self.timeout_reached = False
            self.waiting_started = True
            self.confirmation_received = False
            self.get_logger().info('Waiting for confirmation...')
            self.timeout_timer = self.create_timer(1.0, self.handle_timeout)

    def handle_timeout(self):
        if self.remaining_timeout > 0:
            self.get_logger().info(f'{self.remaining_timeout} seconds remaining')
            self.remaining_timeout -= 1
        elif self.remaining_timeout <= 0 and not self.timeout_reached:
            if not self.confirmation_received:
                if self.state == 'kitchen':
                    self.get_logger().info('Confirmation not received at kitchen. Returning home assuming orders are canceled.')
                    self.stop_timeout()
                    self.task_queue.clear()
                    self.current_task = None
                    self.return_home()
                elif self.state == self.current_task:  
                    self.get_logger().info(f'Confirmation not received at {self.current_task}.')
                    self.unconfirmed_deliveries = True  
                    self.stop_timeout()
                    if self.task_queue:
                        self.current_task = self.task_queue.popleft()
                        self.move_to_table(self.current_task)
                    else:
                        if self.unconfirmed_deliveries:
                            self.return_to_kitchen()
                        else:
                            self.return_home()
            self.timeout_reached = True
            self.waiting_for_confirmation = False
            self.waiting_started = False
            if self.timeout_timer:
                self.timeout_timer.cancel()

    def stop_timeout(self):
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        self.waiting_for_confirmation = False

    def proceed_to_next_phase(self):
        if self.state == 'kitchen':
            self.move_to_table(self.current_task)
        elif self.state == self.current_task:
            self.get_logger().info(f'Delivery complete at {self.current_task}.')
            self.stop_timeout()
            if self.task_queue:
                self.current_task = self.task_queue.popleft()
                self.move_to_table(self.current_task)
            else:
                if self.unconfirmed_deliveries:
                    self.return_to_kitchen()
                else:
                    self.return_home()

    def end_task(self):
        self.task_in_progress = False
        self.returning_home = False
        self.returning_kitchen = False
        self.confirmation_received = False
        self.current_task = None
        self.goal_position = None
        self.waiting_for_confirmation = False
        self.unconfirmed_deliveries = False
        self.get_logger().info('All tasks complete. Ready for new tasks.')

def main(args=None):
    rclpy.init(args=args)
    butler_robot = ButlerRobot()
    rclpy.spin(butler_robot)
    butler_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
