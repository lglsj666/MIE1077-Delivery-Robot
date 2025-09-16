#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json, os, time, threading
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import cv2
from cv_bridge import CvBridge
import math

GRAB_SAVE_DIR = '/home/lglsj/action_record/grab/transitions/'
GRAB_IMG_DIR = '/home/lglsj/action_record/grab/img/'
PLACE_SAVE_DIR = '/home/lglsj/action_record/place/transitions/'
PLACE_IMG_DIR = '/home/lglsj/action_record/place/img/'
os.makedirs(GRAB_SAVE_DIR, exist_ok=True)
os.makedirs(GRAB_IMG_DIR, exist_ok=True)
os.makedirs(PLACE_SAVE_DIR, exist_ok=True)
os.makedirs(PLACE_IMG_DIR, exist_ok=True)

def compute_distance_reward(gripper_center_pos, target_pos):
    if gripper_center_pos is None or target_pos is None:
        return 0.0
    dist = np.linalg.norm(np.array(gripper_center_pos) - np.array(target_pos))
    reward = np.exp(-dist)
    return reward

def compute_vertical_reward(joint_state, tf_buffer, get_logger):
    try:
        arm5_link = 'arm_5_link'
        arm6_link = 'arm_6_link'
        t5 = tf_buffer.lookup_transform('map', arm5_link, rclpy.time.Time())
        t6 = tf_buffer.lookup_transform('map', arm6_link, rclpy.time.Time())
        pos5 = np.array([t5.transform.translation.x, t5.transform.translation.y, t5.transform.translation.z])
        pos6 = np.array([t6.transform.translation.x, t6.transform.translation.y, t6.transform.translation.z])
        vec = pos6 - pos5
        if np.linalg.norm(vec) < 1e-6:
            return 0.0
        z_axis = np.array([0,0,1])
        cos_angle = np.dot(vec, z_axis) / np.linalg.norm(vec)
        angle = np.arccos(np.clip(np.abs(cos_angle), 0, 1))
        reward = np.exp(-abs(angle - (math.pi/2)))
        return reward
    except Exception as e:
        get_logger().warn(f"垂直奖励计算失败: {e}")
        return 0.0

def compute_gripper_action_reward(gripper_center_pos, target_pos, last_gripper_cmd, gripper_counts):
    if gripper_center_pos is None or target_pos is None:
        return 0.0
    dist = np.linalg.norm(np.array(gripper_center_pos) - np.array(target_pos))
    if dist > 0.3:
        return 0.0
    reward = 0.0
    if last_gripper_cmd == 'open':
        count = gripper_counts['open']
        reward = max(0.3 * (0.7 ** count), 0.03)
        gripper_counts['open'] += 1
    elif last_gripper_cmd == 'close':
        count = gripper_counts['close']
        reward = max(0.3 * (0.7 ** count), 0.03)
        gripper_counts['close'] += 1
    return reward

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self.recording = False
        self.place_recording = False
        self.grab_recording = False
        self.transitions = []
        self.last_state = None
        self.current_action = None
        self.last_action = None
        self.current_params = {}
        self.target_pose = None
        self.target_frame = None
        self.snapshot_records = []
        self._lock = threading.Lock()
        self.gripper_counts = {'open': 0, 'close': 0}
        self.last_gripper_cmd = None
        self.attached_flag = [False]
        self.is_attached = False
        self.gripper_center_pos = None
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.cmd_sub = self.create_subscription(
            String, 'record_cmd', self.cmd_cb, 10)
        self.ctrl_sub = self.create_subscription(
            String, 'control_cmd', self.ctrl_cb, 10)
        self.param_sub = self.create_subscription(
            String, 'motion_param', self.param_cb, 10)
        self.snap_sub = self.create_subscription(
            String, 'snapshot', self.snap_req_cb, 10)
        self.attach_sub = self.create_subscription(
            String, 'attacher_state', self.attacher_cb, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, 'object_position', self.target_cb, 10)
        self.img_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_cb, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.gripper_link = 'gripper_link'
        self.head_link = 'head_1_link'
        self.bridge = CvBridge()
        self.latest_rgb_image = None
        self.img_count = 0
        self.grab_target_received = False
        self.session_img_dir = None
        self.session_json_name = None
        self.session_mode = None  # 'grab' or 'place'

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        with self._lock:
            if cmd == 'grab recording':
                self.recording = False
                self.place_recording = False
                self.grab_recording = True
                self.transitions.clear()
                self.last_state = None
                self.snapshot_records.clear()
                self.target_pose = None
                self.target_frame = None
                self.gripper_counts = {'open': 0, 'close': 0}
                self.last_gripper_cmd = None
                self.attached_flag = [False]
                self.is_attached = False
                self.gripper_center_pos = None
                self.get_logger().info('Waiting for target point for grab recording...')
                self.grab_target_received = False
                self.session_img_dir = None
                self.session_json_name = None
                self.session_mode = 'grab'
            elif cmd == 'place recording':
                self.recording = True
                self.place_recording = True
                self.grab_recording = False
                self.transitions.clear()
                self.last_state = None
                self.snapshot_records.clear()
                self.target_pose = None
                self.target_frame = None
                self.gripper_counts = {'open': 0, 'close': 0}
                self.last_gripper_cmd = None
                self.attached_flag = [False]
                self.is_attached = False
                self.gripper_center_pos = None
                self.get_logger().info('Place recording started immediately.')
                # Place模式一启动就分配session名
                stamp = time.strftime('%Y_%m_%d_%H_%M_%S') + '_%03d' % int(time.time()*1000%1000)
                self.session_json_name = stamp
                self.session_img_dir = os.path.join(PLACE_IMG_DIR, self.session_json_name)
                os.makedirs(self.session_img_dir, exist_ok=True)
                self.session_mode = 'place'
            elif cmd == 'end':
                self.recording = False
                self.place_recording = False
                self.grab_recording = False
                self.save_transitions()
                self.get_logger().info('Recorder stopped.')

    def target_cb(self, msg: PoseStamped):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        with self._lock:
            if self.grab_recording and not self.grab_target_received:
                self.target_pose = pos
                self.target_frame = msg.header.frame_id
                self.recording = True
                self.grab_target_received = True
                # 分配grab session名
                stamp = time.strftime('%Y_%m_%d_%H_%M_%S') + '_%03d' % int(time.time()*1000%1000)
                self.session_json_name = stamp
                self.session_img_dir = os.path.join(GRAB_IMG_DIR, stamp)
                os.makedirs(self.session_img_dir, exist_ok=True)
                self.session_mode = 'grab'
                self.get_logger().info(f'Grab recording started at target: {pos} ({self.target_frame}), images to: {self.session_img_dir}')

    def param_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.current_params = payload
        except Exception as e:
            self.get_logger().warn(f'Failed to parse motion_param: {e}')

    def ctrl_cb(self, msg: String):
        try:
            action = json.loads(msg.data)
            if isinstance(action, list) and action:
                self.current_action = action
                if 'grip' in action:
                    self.last_gripper_cmd = 'close'
                elif 'release' in action:
                    self.last_gripper_cmd = 'open'
                else:
                    self.last_gripper_cmd = None
        except:
            self.current_action = None
            self.last_gripper_cmd = None

    def attacher_cb(self, msg: String):
        try:
            obj = json.loads(msg.data)
            self.is_attached = bool(obj.get('attached', False))
        except Exception as e:
            self.get_logger().warn(f'Failed to parse attacher_state: {e}')

    def get_gripper_center_pose_in_camera(self):
        try:
            if self.target_frame is None:
                return None
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.gripper_link,
                rclpy.time.Time())
            trans = t.transform.translation
            return [trans.x, trans.y, trans.z]
        except TransformException as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    def image_cb(self, msg: Image):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")
            self.latest_rgb_image = None

    def joint_cb(self, msg: JointState):
        if not self.recording:
            return
        try:
            gripper_center_pos = self.get_gripper_center_pose_in_camera()
            self.gripper_center_pos = gripper_center_pos
        except Exception as ex:
            self.get_logger().warn(f"TF lookup exception: {ex}")
            self.gripper_center_pos = None
        state = {n: p for n, p in zip(msg.name, msg.position)}
        with self._lock:
            img_name = None
            img_dir = self.session_img_dir
            if self.latest_rgb_image is not None and img_dir is not None:
                img_name = time.strftime('%Y_%m_%d_%H_%M_%S') + '_%03d.png' % int(time.time()*1000%1000)
                img_path = os.path.join(img_dir, img_name)
                try:
                    cv2.imwrite(img_path, self.latest_rgb_image)
                except Exception as e:
                    self.get_logger().warn(f"Image save error: {e}")
            reward = None
            gripper_action_reward = None
            distance_reward = None
            vertical_reward = None
            if self.grab_recording:
                distance_reward = compute_distance_reward(self.gripper_center_pos, self.target_pose)
                vertical_reward = compute_vertical_reward(msg, self.tf_buffer, self.get_logger())
                gripper_action_reward = compute_gripper_action_reward(self.gripper_center_pos, self.target_pose, self.last_gripper_cmd, self.gripper_counts)
                reward = distance_reward + vertical_reward + gripper_action_reward
            transition = {
                'state': state,
                'action': self.last_action if self.last_action else [],
                'params': dict(self.current_params),
                'next_state': state,
                'done': False,
                'timestamp': time.time(),
                'img_name': img_name
            }
            if self.grab_recording:
                transition['reward'] = reward
                transition['distance_reward'] = distance_reward
                transition['vertical_reward'] = vertical_reward
                transition['gripper_action_reward'] = gripper_action_reward
                transition['is_attached'] = self.is_attached
                transition['target_pose'] = self.target_pose
                transition['gripper_center_pos'] = self.gripper_center_pos
            self.transitions.append(transition)
            self.last_state = state
            self.last_action = self.current_action
            self.current_action = None
            self.last_gripper_cmd = None

    def snap_req_cb(self, msg: String):
        if self.recording:
            snap_content = {'time': time.time(), 'msg': msg.data}
            self.snapshot_records.append(snap_content)
            self.get_logger().info('Snapshot recorded: ' + msg.data)

    def save_transitions(self):
        if self.transitions:
            self.transitions[-1]['done'] = True
        data = {
            'transitions': self.transitions,
            'snapshot_records': self.snapshot_records,
            'target_object_pose': self.target_pose if self.grab_recording else None,
            'target_frame': self.target_frame,
            'record_time': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        # 以session_mode和session_json_name来决定存储路径
        if self.session_json_name is None:
            stamp = time.strftime('%Y_%m_%d_%H_%M_%S') + '_%03d' % int(time.time()*1000%1000)
            self.session_json_name = stamp
            # 并及时新建图片文件夹
            if self.session_mode == 'grab':
                self.session_img_dir = os.path.join(GRAB_IMG_DIR, self.session_json_name)
                os.makedirs(self.session_img_dir, exist_ok=True)
            elif self.session_mode == 'place':
                self.session_img_dir = os.path.join(PLACE_IMG_DIR, self.session_json_name)
                os.makedirs(self.session_img_dir, exist_ok=True)
        if self.session_mode == 'grab':
            fname = os.path.join(GRAB_SAVE_DIR, f'{self.session_json_name}.json')
        elif self.session_mode == 'place':
            fname = os.path.join(PLACE_SAVE_DIR, f'{self.session_json_name}.json')
        else:
            fname = os.path.join(GRAB_SAVE_DIR, f'{self.session_json_name}.json')
        with open(fname, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'Saved {len(self.transitions)} transitions to {fname}')
        self.transitions.clear()
        self.snapshot_records.clear()
        self.target_pose = None
        self.last_state = None
        self.gripper_center_pos = None
        self.target_frame = None
        self.session_img_dir = None
        self.session_json_name = None
        self.session_mode = None
        self.place_recording = False
        self.grab_recording = False

    def main(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

def main():
    rclpy.init()
    node = RecorderNode()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
