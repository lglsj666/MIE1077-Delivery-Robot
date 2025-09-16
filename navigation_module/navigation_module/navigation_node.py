#!/usr/bin/env python3
import json
import threading
import time
import math
import numpy as np
np.float = float
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.table = {
            1: [1.680, -4.264, 4.712],
            2: [-0.098, -4.264, 4.712],
            3: [-1.876, -4.264, 4.712],
            4: [-3.654, -4.264, 4.712],
            5: [-1.195, -1.864, 3.142],
            6: [-3.642, -2.819, 1.571],
            7: [-1.195, 1.253, 3.142],
            8: [-3.642, 0.295, 1.571],
            9: [-2.918, 3.973, 3.142],
            10: [-0.043, 4.835, 1.571],
            11: [-1.8206, 4.835, 1.571],
            12: [-0.043, 4.835, 1.571],
        }
        self.counter = {
            1: [1.998, -3.115, 1.309],
            2: [1.998, -3.115, 1.833],
            3: [1.998, 3.123, 4.974],
            4: [1.998, 3.123, 4.451],
        }
        self.action_queue = []
        self.is_busy = False
        self.in_avoidance = False
        self.obstacle_detected = False
        self.latest_scan = None
        self.current_pose = None
        self.primary_goal = None
        self.w_d = 0.7
        self.w_alpha = 0.3
        self.avoidance_step = 0.5
        self.lock = threading.Lock()
        self.create_subscription(String, 'actions', self.on_action_msg, 10)
        self.feedback_pub = self.create_publisher(String, 'navigation_feedback', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_pose,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.current_action = None
        self.get_logger().info('NavigationNode Initialization finished')

    def on_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.current_pose = {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw}

    def on_scan(self, scan: LaserScan):
        self.latest_scan = scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        mid = len(scan.ranges) // 2
        window = int((15.0 / 180.0 * math.pi) / angle_inc)
        front_ranges = scan.ranges[mid-window : mid+window]
        if any(d < 0.3 for d in front_ranges if d > 0.0):
            with self.lock:
                if not self.obstacle_detected:
                    self.obstacle_detected = True
                    threading.Thread(target=self.handle_obstacle).start()

    def on_action_msg(self, msg: String):
        act = json.loads(msg.data)
        if act.get('action') == 'navigation':
            self.action_queue.append(act)
            self.try_next_goal()
        elif act.get('action') == 'operation' and act.get('target') == 'completed':
            self.cancel_current_goal()
            self.action_queue.clear()
            self.is_busy = False
            self.get_logger().info('Receive operation completed, clear all status')

    def try_next_goal(self):
        if self.is_busy or not self.action_queue:
            return
        next_action = self.action_queue.pop(0)
        self.is_busy = True
        self.primary_goal = next_action
        self.send_navigation_goal(next_action)

    def send_navigation_goal(self, act):
        tgt = act.get('target')
        tid = act.get('target_id')
        if tgt == 'home':
            xyzyaw = [0.0, 0.0, 0.0]
        elif tgt == 'table':
            xyzyaw = self.table.get(tid, [0.0, 0.0, 0.0])
        elif tgt == 'counter':
            xyzyaw = self.counter.get(tid, [0.0, 0.0, 0.0])
        elif tgt == 'custom':
            xyzyaw = act.get('xyzyaw', [0.0,0.0,0.0])
        else:
            self.get_logger().warn(f'Unknown target: {tgt}, regard as fail')
            self.publish_feedback(act, 'fail')
            self.is_busy = False
            return

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = xyzyaw
        q = quaternion_from_euler(0, 0, yaw)
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        goal.pose = ps
        self.get_logger().info(f'Send navigation target: {tgt}/{tid} -> {xyzyaw}')
        self.current_action = act
        self.obstacle_detected = False
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Cannot connect to navigate_to_pose action server')
            self.publish_feedback(act, 'fail')
            self.is_busy = False
            return
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.on_nav_feedback
        )
        future.add_done_callback(self.on_goal_response)

    def send_direct_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        goal.pose = ps
        self.get_logger().info(f'Send direct navigation goal: [{x:.2f}, {y:.2f}, {yaw:.2f}]')
        self.current_action = {'action':'navigation','target':'custom','xyzyaw':[x,y,yaw],'index': self.primary_goal.get('index')}
        self.current_goal_handle = None
        self.is_busy = True
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Cannot connect to navigate_to_pose action server')
            self.publish_feedback(self.current_action, 'fail')
            self.is_busy = False
            return
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.on_nav_feedback
        )
        future.add_done_callback(self.on_goal_response)

    def handle_obstacle(self):
        with self.lock:
            if self.in_avoidance is True or self.obstacle_detected is False:
                return
            self.in_avoidance = True
            self.obstacle_detected = False
        self.get_logger().warn('Obstacle detected: executing avoidance recovery')
        self.cancel_current_goal()
        twist = Twist()
        twist.linear.x = -0.2
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        if not self.latest_scan or not self.current_pose:
            self.get_logger().error('No scan or pose data for avoidance, aborting recovery')
            with self.lock:
                self.is_busy = False
                self.in_avoidance = False
            return
        scan = self.latest_scan
        pose = self.current_pose
        headings = [math.radians(d) for d in range(-180, 181, 15)]
        best_score = -float('inf')
        best_heading = None
        mid = len(scan.ranges) // 2
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        global_yaw = self.primary_goal.get('xyzyaw', [0,0,0])[2]
        for theta in headings:
            idx = int((theta - angle_min) / angle_inc)
            if idx < 0 or idx >= len(scan.ranges):
                continue
            d = scan.ranges[idx] if scan.ranges[idx] > 0.0 else 0.0
            robot_yaw = pose['yaw']
            heading = robot_yaw + theta
            diff = abs(self.angle_diff(heading, global_yaw))
            score = self.w_d * d + self.w_alpha * (1 - diff / math.pi)
            if score > best_score:
                best_score = score
                best_heading = heading
        if best_heading is None:
            self.get_logger().error('No valid heading found for avoidance')
            with self.lock:
                self.is_busy = False
                self.in_avoidance = False
            return
        new_x = pose['x'] + self.avoidance_step * math.cos(best_heading)
        new_y = pose['y'] + self.avoidance_step * math.sin(best_heading)
        new_yaw = best_heading
        self.get_logger().info(f'Avoidance: new local goal at ({new_x:.2f}, {new_y:.2f}, {new_yaw:.2f})')
        self.send_direct_goal(new_x, new_y, new_yaw)
        with self.lock:
            self.in_avoidance = False

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.publish_feedback(self.current_action, 'fail')
            self.is_busy = False
            return
        self.get_logger().info('Navigation goal accepted')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_nav_feedback(self, feedback_msg):
        pass

    def on_result(self, future):
        result = future.result().result
        status = future.result().status
        if self.current_action.get('target') == 'custom':
            self.get_logger().info('Local avoidance goal succeeded, resuming original global goal')
            self.send_navigation_goal(self.primary_goal)
            return
        SUCCEEDED = 4
        if status == SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.publish_feedback(self.current_action, 'complete')
        else:
            self.get_logger().warn(f'Navigation failed, status: {status}')
            self.publish_feedback(self.current_action, 'fail')
        with self.lock:
            self.is_busy = False
        self.try_next_goal()

    def cancel_current_goal(self):
        if self.current_goal_handle:
            self.get_logger().info('Cancelling current goal')
            self.current_goal_handle.cancel_goal_async()
        self.current_goal_handle = None
        self.is_busy = False

    def publish_feedback(self, action, target_status):
        fb = {
            'index': action.get('index'),
            'action': 'navigation',
            'target': target_status,
            'target_id': action.get('target_id')
        }
        msg = String()
        msg.data = json.dumps(fb)
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'Publish navigation_feedback: {fb}')

    def angle_diff(self, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
