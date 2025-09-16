#!/usr/bin/env python3
import sys
import json
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from play_motion2_msgs.action import PlayMotion2
from play_motion2_msgs.srv import IsMotionReady, AddMotion, RemoveMotion
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration

ARM_JOINTS = [
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
]
TORSO_JOINT = 'torso_lift_joint'
HAND_JOINT = 'hand_thumb_joint'
GRIPPER_LEFT = 'gripper_left_finger_joint'
GRIPPER_RIGHT = 'gripper_right_finger_joint'
HEAD_JOINT_1 = 'head_1_joint'  # 左右摇头
HEAD_JOINT_2 = 'head_2_joint'  # 抬头低头
EEF_LINK = 'gripper_link'
BASE_LINK = 'base_link'

DEFAULT_SPEED = 0.08
DEFAULT_STEP = 0.02
DEFAULT_ROT_SPEED = 0.5
DEFAULT_ROT_STEP = 0.1
DEFAULT_GRIP_SPEED = 0.06
DEFAULT_GRIP_STEP = 0.01

IK_SERVICE = '/compute_ik'
IK_GROUP = 'arm'


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.ctrl_sub = self.create_subscription(String, 'control_cmd', self.ctrl_cb, 10)
        self.param_sub = self.create_subscription(String, 'motion_param', self.param_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self._is_ready_client = self.create_client(IsMotionReady, '/play_motion2/is_motion_ready')
        self._play_motion_client = ActionClient(self, PlayMotion2, 'play_motion2')
        self._add_motion_client = self.create_client(AddMotion, '/play_motion2/add_motion')
        self._remove_motion_client = self.create_client(RemoveMotion, '/play_motion2/remove_motion')
        self._ik_client = self.create_client(GetPositionIK, IK_SERVICE)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.latest_joints = {}
        self._sending = False
        self.speed = DEFAULT_SPEED
        self.step = DEFAULT_STEP
        self.rot_speed = DEFAULT_ROT_SPEED
        self.rot_step = DEFAULT_ROT_STEP
        self.grip_speed = DEFAULT_GRIP_SPEED
        self.grip_step = DEFAULT_GRIP_STEP
        self.lock = threading.Lock()
        self._wait_services_async()

    def _wait_services_async(self):
        def waiter():
            for client, name in [
                (self._is_ready_client, '/play_motion2/is_motion_ready'),
                (self._add_motion_client, '/play_motion2/add_motion'),
                (self._remove_motion_client, '/play_motion2/remove_motion'),
                (self._ik_client, IK_SERVICE)
            ]:
                while not client.wait_for_service(timeout_sec=0.2):
                    self.get_logger().info(f'Waiting for {name} service...')
            self.get_logger().info('All services ready')

        threading.Thread(target=waiter, daemon=True).start()

    def param_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if 'speed' in payload and payload['speed']:
                self.speed = float(payload['speed'])
            if 'step' in payload and payload['step']:
                self.step = float(payload['step'])
            if 'rot_speed' in payload and payload['rot_speed']:
                self.rot_speed = float(payload['rot_speed'])
            if 'rot_step' in payload and payload['rot_step']:
                self.rot_step = float(payload['rot_step'])
            if 'grip_speed' in payload and payload['grip_speed']:
                self.grip_speed = float(payload['grip_speed'])
            if 'grip_step' in payload and payload['grip_step']:
                self.grip_step = float(payload['grip_step'])
            self.get_logger().info(
                f'Set arm: {self.speed} m/s {self.step} m, rot: {self.rot_speed} rad/s {self.rot_step} rad, grip: {self.grip_speed} m/s {self.grip_step} m')
        except Exception as e:
            self.get_logger().error(f'Parse motion_param failed: {e}')

    def joint_cb(self, msg: JointState):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def ctrl_cb(self, msg: String):
        try:
            cmds = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse control_cmd: {e}')
            return
        if self._sending:
            return
        self.get_logger().info(f'Received control_cmd: {cmds}')
        move = self._parse_eef_cmd(cmds)
        if move is not None:
            self._sending = True
            self.move_eef_and_send(move)
            return
        if 'torso_up' in cmds or 'torso_down' in cmds:
            self._sending = True
            self.torso_control(cmds)
            return
        # hs��l�6/arm_1~arm_7
        arm_rot_handled = False
        for idx in range(1, 8):
            if f'arm{idx}_clockwise' in cmds or f'arm{idx}_counter' in cmds:
                self._sending = True
                self.arm_rot_control(cmds, idx)
                arm_rot_handled = True
                break
        if arm_rot_handled:
            return
        if 'grip' in cmds or 'release' in cmds:
            self._sending = True
            self.grip_control(cmds)
            return
        if 'home' in cmds:
            self._sending = True
            self.tuck_arm_home()
            return
        # --- Head Control ---
        head_handled = False
        for cmd_item in cmds:
            if isinstance(cmd_item, dict) and cmd_item.get('controller') == 'head_controller':
                self._sending = True
                self.head_control(cmd_item)
                head_handled = True
                break
        if head_handled:
            return

    def _parse_eef_cmd(self, cmds):
        dx, dy, dz = 0, 0, 0
        s = self.step
        if 'eef_up' in cmds:
            dz += s
        if 'eef_down' in cmds:
            dz -= s
        if 'eef_forward' in cmds:
            dx += s
        if 'eef_backward' in cmds:
            dx -= s
        if 'eef_left' in cmds:
            dy += s
        if 'eef_right' in cmds:
            dy -= s
        if dx != 0 or dy != 0 or dz != 0:
            return (dx, dy, dz)
        return None

    def tuck_arm_home(self):
        try:
            motion_name = 'home'
            skip_planning = True

            goal = PlayMotion2.Goal()
            goal.motion_name = motion_name
            goal.skip_planning = skip_planning

            goal_future = self._play_motion_client.send_goal_async(goal)
            goal_future.add_done_callback(self._on_goal_done_home)
        except Exception as e:
            self.get_logger().error(f'Tuck arm home error: {e}')
            self._sending = False

    def _on_goal_done_home(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Home goal rejected')
                self._sending = False
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result_done_home)
        except Exception as e:
            self.get_logger().error(f'Home goal callback error: {e}')
            self._sending = False

    def _on_result_done_home(self, future):
        try:
            result = future.result().result
            if hasattr(result, 'error') and getattr(result, 'error', '') == '':
                self.get_logger().info('Home motion succeeded')
            else:
                self.get_logger().error(f"Home motion failed: {getattr(result, 'error', '<unknown>')}")
        except Exception as e:
            self.get_logger().error(f'Home result callback error: {e}')
        self._sending = False

    def move_eef_and_send(self, delta):
        try:
            if len(self.latest_joints) < 7:
                self.get_logger().warn("Joint states not yet received. Ignoring EEF command.")
                self._sending = False
                return
            trans = self.tf_buffer.lookup_transform(BASE_LINK, EEF_LINK, rclpy.time.Time(),
                                                    timeout=Duration(seconds=1.0))
            goal_xyz = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]) + np.array(delta)
            cur_orientation = trans.transform.rotation
            current_joints = [self.latest_joints.get(j, 0.0) for j in ARM_JOINTS]
            self.get_logger().info(
                f"EEF control: xyz={goal_xyz}, orientation={cur_orientation}, joints={current_joints}")
            self.simple_ik(goal_xyz, current_joints, cur_orientation)
        except Exception as e:
            self.get_logger().error(f'EEF move error: {e}')
            self._sending = False

    def torso_control(self, cmds):
        try:
            cur = self.latest_joints.get(TORSO_JOINT, 0.0)
            tgt = cur
            s = self.step
            TORSO_MIN, TORSO_MAX = 0.0, 0.35
            if 'torso_up' in cmds:
                tgt = min(cur + s, TORSO_MAX)
            if 'torso_down' in cmds:
                tgt = max(cur - s, TORSO_MIN)
            self.send_dynamic_trajectory_async([TORSO_JOINT], [tgt],
                                               duration=self.step / self.speed if self.speed > 1e-5 else 1.0)
        except Exception as e:
            self.get_logger().error(f'Torso control error: {e}')
            self._sending = False

    def arm_rot_control(self, cmds, arm_idx):
        try:
            # / arm_1~arm_7
            joint = f'arm_{arm_idx}_joint'
            direction = 0
            if f'arm{arm_idx}_clockwise' in cmds:
                direction = -1
            if f'arm{arm_idx}_counter' in cmds:
                direction = 1
            if direction != 0:
                cur = self.latest_joints.get(joint, 0.0)
                step = self.rot_step
                tgt = cur + direction * step
                duration = abs(step) / max(self.rot_speed, 1e-5)
                if duration < 0.05:
                    duration = 0.05
                joint_list = [self.latest_joints.get(j, 0.0) for j in ARM_JOINTS]
                idx = ARM_JOINTS.index(joint)
                joint_list[idx] = tgt
                self.send_dynamic_trajectory_async(ARM_JOINTS, joint_list, duration=duration)
        except Exception as e:
            self.get_logger().error(f'Arm{arm_idx} rot control error: {e}')
            self._sending = False

    def grip_control(self, cmds):
        try:
            l_cur = self.latest_joints.get(GRIPPER_LEFT, 0.0)
            r_cur = self.latest_joints.get(GRIPPER_RIGHT, 0.0)
            l_tgt, r_tgt = l_cur, r_cur
            s = self.grip_step
            MIN_OPEN = 0.0
            MAX_OPEN = 0.045
            if 'grip' in cmds:
                l_tgt = max(l_cur - s, MIN_OPEN)
                r_tgt = max(r_cur - s, MIN_OPEN)
            if 'release' in cmds:
                l_tgt = min(l_cur + s, MAX_OPEN)
                r_tgt = min(r_cur + s, MAX_OPEN)
            duration = abs(l_tgt - l_cur) / (self.grip_speed if self.grip_speed > 1e-5 else 0.02)
            if duration < 0.05:
                duration = 0.05
            self.send_dynamic_trajectory_async(
                [GRIPPER_LEFT, GRIPPER_RIGHT], [l_tgt, r_tgt],
                duration=duration,
                controller='gripper_controller'
            )
        except Exception as e:
            self.get_logger().error(f'Grip control error: {e}')
            self._sending = False

    def head_control(self, cmd_item):
        try:
            cur1 = self.latest_joints.get(HEAD_JOINT_1, 0.0)
            cur2 = self.latest_joints.get(HEAD_JOINT_2, 0.0)
            step = self.rot_step if hasattr(self, 'rot_step') else 0.1
            tgt1, tgt2 = cur1, cur2
            # 左右摇头功能
            if cmd_item.get('cmd') == 'head_left':
                tgt1 = cur1 + step
            elif cmd_item.get('cmd') == 'head_right':
                tgt1 = cur1 - step
            # 点头/仰头功能
            if cmd_item.get('cmd') == 'head_up':
                tgt2 = cur2 + step
            elif cmd_item.get('cmd') == 'head_down':
                tgt2 = cur2 - step
            duration = abs(step) / max(self.rot_speed, 1e-5)
            if duration < 0.05:
                duration = 0.05
            # 一定要发送两个joint，姿态分别是目标/当前
            self.send_dynamic_trajectory_async(
                [HEAD_JOINT_1, HEAD_JOINT_2], [tgt1, tgt2], duration=duration, controller='head_controller'
            )
        except Exception as e:
            self.get_logger().error(f'Head control error: {e}')
            self._sending = False

    def simple_ik(self, target_xyz, q_init, cur_orientation):
        pose = PoseStamped()
        pose.header.frame_id = BASE_LINK
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target_xyz[0])
        pose.pose.position.y = float(target_xyz[1])
        pose.pose.position.z = float(target_xyz[2])
        pose.pose.orientation = cur_orientation
        req = GetPositionIK.Request()
        req.ik_request.group_name = IK_GROUP
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = EEF_LINK
        req.ik_request.robot_state.joint_state.name = ARM_JOINTS
        req.ik_request.robot_state.joint_state.position = q_init
        req.ik_request.timeout = Duration(seconds=0.5).to_msg()
        self.get_logger().info(f"Requesting IK: xyz={target_xyz}, q_init={q_init}, orientation={cur_orientation}")
        future = self._ik_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_ik_done(fut, target_xyz))

    def _on_ik_done(self, future, target_xyz):
        try:
            res = future.result()
            if res.error_code.val != 1:
                self.get_logger().error(f'MoveIt IK failed, error_code={res.error_code.val}')
                self._sending = False
                return
            positions = res.solution.joint_state.position
            names = res.solution.joint_state.name
            joint_dict = dict(zip(names, positions))
            joint_list = [joint_dict[j] for j in ARM_JOINTS]
            self.send_dynamic_trajectory_async(ARM_JOINTS, joint_list)
        except Exception as e:
            self.get_logger().error(f'IK callback error: {e}')
            self._sending = False

    def send_dynamic_trajectory_async(self, joint_names, positions, duration=None, controller=None):
        if duration is None:
            duration = self.step / self.speed if self.speed > 1e-5 else 1.0
        key = f'teleop_{int(time.time() * 1000)}'
        add_req = AddMotion.Request()
        add_req.motion.key = key
        add_req.motion.joints = joint_names
        add_req.motion.positions = positions
        add_req.motion.times_from_start = [duration]
        if hasattr(add_req.motion, 'controller') and controller:
            add_req.motion.controller = controller
        add_req.overwrite = True
        self.get_logger().info(
            f'Adding dynamic motion {key}, joints={joint_names}, positions={positions}, duration={duration:.3f}s, controller={controller}')
        future = self._add_motion_client.call_async(add_req)
        future.add_done_callback(lambda fut: self._on_add_motion_done(fut, key, joint_names, positions))

    def _on_add_motion_done(self, future, key, joint_names, positions):
        try:
            result = future.result()
            if not result or not result.success:
                self.get_logger().error('Failed to add motion')
                self._sending = False
                return
            goal = PlayMotion2.Goal()
            goal.motion_name = key
            goal.skip_planning = True
            self.get_logger().info(f'Executing motion {key}')
            goal_future = self._play_motion_client.send_goal_async(goal)
            goal_future.add_done_callback(lambda ghf: self._on_goal_done(ghf, key))
        except Exception as e:
            self.get_logger().error(f'AddMotion callback error: {e}')
            self._sending = False

    def _on_goal_done(self, future, key):
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                self._sending = False
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda fut: self._on_result_done(fut, key))
        except Exception as e:
            self.get_logger().error(f'Goal callback error: {e}')
            self._sending = False

    def _on_result_done(self, future, key):
        try:
            result = future.result().result
            if hasattr(result, 'error') and getattr(result, 'error', '') == '':
                self.get_logger().info('Motion succeeded')
            else:
                self.get_logger().error(f"Motion failed: {getattr(result, 'error', '<unknown>')}")
            rem_req = RemoveMotion.Request()
            rem_req.motion_key = key
            rem_future = self._remove_motion_client.call_async(rem_req)
            rem_future.add_done_callback(lambda fut: self._on_remove_motion_done(fut, key))
        except Exception as e:
            self.get_logger().error(f'Result callback error: {e}')
            self._sending = False

    def _on_remove_motion_done(self, future, key):
        try:
            _ = future.result()  # Not used
            self.get_logger().info(f'Removed dynamic motion {key}')
        except Exception as e:
            self.get_logger().error(f'RemoveMotion callback error: {e}')
        self._sending = False

    def main(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
