import sys
import os
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from openai import OpenAI
from openai.types.chat import ChatCompletionSystemMessageParam, ChatCompletionUserMessageParam
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QCheckBox,
    QTextEdit, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtCore import pyqtSignal, QTimer, QObject
from PyQt5.QtGui import QImage, QPixmap
import json

#------------------------------------------------------------------------------
# ROS2 Node: Handles camera, depth, YOLO, snapshots, and LLM calls
#------------------------------------------------------------------------------
class InputModuleNode(Node, QObject):
    nav_feedback_signal = pyqtSignal(str)
    def __init__(self):
        Node.__init__(self, 'input_module')
        QObject.__init__(self)
        # OpenAI client
        api_key = "xxx"
        self.openai_client = OpenAI(api_key=api_key)

        # YOLO model
        self.bridge = CvBridge()
        try:
            from ultralytics import YOLO
            pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            model_path = os.path.join(pkg_dir, 'yolo', 'yolo11m.pt')
            self.get_logger().info(f"Loading YOLO model from {model_path}")
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise
        self.model_names = self.model.names
        self.yolo_lock = threading.Lock()

        # State
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.intrinsics = None
        self.camera_frame = None

        # Subscriptions & Publisher
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/head_front_camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, 'snapshot', self.snapshot_callback, 10)
        self.create_subscription(String, 'navigation_feedback', self.nav_feedback_callback, 10)
        self.pos_pub = self.create_publisher(PoseStamped, 'object_position', 10)

        self.actions_pub = self.create_publisher(String, 'actions', 10)
        self.pending_actions = []
        self.completed_indexes = set()
        self.action_index = 1

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg)
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            self.latest_depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def camera_info_callback(self, msg):
        K = msg.k if hasattr(msg, 'k') else msg.K
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        self.intrinsics = (fx, fy, cx, cy)
        self.camera_frame = msg.header.frame_id

    def snapshot_callback(self, msg):
        # Trigger manual snapshot via topic
        label = msg.data.strip() or 'sprite'
        threading.Thread(target=self.perform_snapshot, args=(label, ''), daemon=True).start()

    def perform_snapshot(self, target_label, save_path=''):
        # Wait few frames
        frames = []
        depths = []
        for _ in range(10):
            time.sleep(0.1)
            if self.latest_rgb_image is not None and self.latest_depth_image is not None:
                frames.append(self.latest_rgb_image.copy())
                depths.append(self.latest_depth_image.copy())
        if len(frames) < 5:
            self.get_logger().warn("Insufficient frames for snapshot")
            return
        positions = []
        bbox = None
        for img, depth in zip(frames[4:], depths[4:]):
            with self.yolo_lock:
                results = self.model(img, verbose=False)
            if results and results[0].boxes:
                for box in results[0].boxes:
                    cls_id = int(box.cls)
                    name = self.model_names[cls_id]
                    if name.lower() != target_label.lower():
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    u, v = (x1 + x2)//2, (y1 + y2)//2
                    if self.intrinsics and 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]:
                        d = float(depth[v, u])
                        if not np.isnan(d) and d > 0:
                            fx, fy, cx, cy = self.intrinsics
                            Z = d
                            X = (u - cx)/fx * Z
                            Y = (v - cy)/fy * Z
                            positions.append((X, Y, Z))
                            bbox = (x1, y1, x2, y2)
        if not positions:
            self.get_logger().warn("No detections in snapshot")
            return
        avg = np.mean(np.array(positions), axis=0)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.camera_frame or 'camera_frame'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, avg)
        pose.pose.orientation.w = 1.0
        self.pos_pub.publish(pose)
        self.get_logger().info(f"Published position: {avg}")
        if save_path:
            img = frames[-1].copy()
            if bbox:
                cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,0,255), 2)
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            cv2.imwrite(save_path, img)
            self.get_logger().info(f"Saved snapshot to {save_path}")

    def call_llm(self, command, signal):
        # Build and send LLM request
        sys_ins = """
        You are a helpful assistant in planning. 
        Now, I'm going to give you some sentence, please plan a series of actions (An operation) according to the sentences.
        Definition for actions: An action is a piece of message that is in a specified format, and this message tells the robot what to do.
        Definition for operations: An operation is a series of actions through which the robot can reach its goal described in the sentences that I give.
        An action be in the following format:
{
index: 1,
action: "navigation",
target: "table",
target_id: 5,
}
        In this format, index and target_id are integer numbers, while action and target are strings.
        The argument index defines the order of the actions. It starts from 1 and ends at the index of the last action.
        The argument action is the name of an action. There are only 3 types of actions: 'navigation', 'pick' and 'place'.
        Navigation is the action that the robot moves from a point to another point.
        Pick is the action that the robot picks up a certain object.
        Place is the action that the robot put down the object in its hand to a certain point.
        The argument target defines the final goal of an action.
        If the action type is navigation, then the target can only be 'table', 'counter' or 'home'.
        And in this case the argument target_id indicates which table or which counter. There is only 1 'home', so the target_id is always 1.
        If the action type is pick, then the target is the object that the user wants the robot to pick up. The object can only be one word.
        For example, if the user says 'Pick up the sprite bottle', then the target should be 'sprite'.
        In this case, the target_id should be the class_id of the target.
        For example, if target is 'sprite', then the target_id should be '0'.
        If the action type is place, then the target can only be 'table' or 'counter'. 
        Also, in this case, the target_id indicates which table or which counter.
        An full example:
        User Message: 'Go to table 5, pick up the sprite bottle and place it at counter 1'
        Output Operations: '
{
index: 1,
action: "navigation",
target: "table",
target_id: 5,
}
{
index: 2,
action: "pick",
target: "sprite",
target_id: 0,
}
{
index: 3,
action: "navigation",
target: "counter",
target_id: 1,
}
{
index: 4,
action: "place",
target: "counter",
target_id: 1,
}
'
        You should only output in the format of a series of actions (AN operation) like what is shown in the example.
        Do not output other things. Do not explain your output.
        Specially, if you are told to go back home, then you output:
{
index: 1,
action: "navigation",
target: "home",
target_id: 1,
}
        """
        try:
            msgs = [
                ChatCompletionSystemMessageParam(role='system', content=sys_ins),
                ChatCompletionUserMessageParam(role='user', content=command)
            ]
            resp = self.openai_client.chat.completions.create(
                model='gpt-4.1-nano', messages=msgs
            )
            raw = resp.choices[0].message.content.strip()
        except Exception as e:
            raw = f"LLM error: {e}"
        # Parse actions
        try:
            actions = self.parse_actions(raw)
        except Exception:
            self.get_logger().warning("parse_actions error, skipping")
            actions = []
        if actions:
            self.action_index = 1
            for act in actions:
                act['index'] = self.action_index
                self.action_index += 1
                self.pending_actions.append(act)
                # 广播
                msg = String()
                msg.data = json.dumps(act)
                self.actions_pub.publish(msg)
                # GUI显示
                text = f"LLM: {json.dumps(act)}"
                signal.emit(text)
        else:
            signal.emit(f"LLM: {raw}")

    def parse_actions(self, raw):
        import re
        acts = []
        for m in re.findall(r'{[^}]*}', raw):
            d = {}
            for line in m.strip('{}').splitlines():
                if ':' in line:
                    k, v = line.split(':', 1)
                    k = k.strip().strip('"')
                    v = v.strip().strip('",')
                    if v.isdigit():
                        v = int(v)
                    d[k] = v
            if d:
                acts.append(d)
        return acts

    def nav_feedback_callback(self, msg):
        try:
            fb = json.loads(msg.data)
            prefix = "Navigation Module: "
            text = prefix + json.dumps(fb)
            self.nav_feedback_signal.emit(text)
            # 只针对 navigation action
            if fb.get('action') == 'navigation':
                idx = fb.get('index')
                target = fb.get('target')
                if target == 'complete':
                    self.completed_indexes.add(idx)
                    self.pending_actions = [a for a in self.pending_actions if a.get('index') != idx]
                    # 全部完成
                    if not self.pending_actions:
                        comp_msg = {
                            "index": 1,
                            "action": "operation",
                            "target": "completed",
                            "target_id": 1
                        }
                        self.actions_pub.publish(String(data=json.dumps(comp_msg)))
                        self.nav_feedback_signal.emit('LLM: Operation completed!')
                elif target == 'fail':
                    self.pending_actions.clear()
                    home_action = {
                        "index": 1,
                        "action": "navigation",
                        "target": "home",
                        "target_id": 1
                    }
                    self.actions_pub.publish(String(data=json.dumps(home_action)))
                    self.nav_feedback_signal.emit('LLM: Navigation failed, going home.')
                elif target == 'collision':
                    self.pending_actions.clear()
                    stop_action = {
                        "index": 1,
                        "action": "emergency_stop",
                        "target": "collision",
                        "target_id": fb.get('target_id', 0)
                    }
                    self.actions_pub.publish(String(data=json.dumps(stop_action)))
                    self.nav_feedback_signal.emit('LLM: Collision detected! Emergency stop.')
        except Exception as e:
            self.get_logger().error(f"Failed to parse nav feedback: {e}")

#------------------------------------------------------------------------------
# Qt GUI: MainWindow with composition of InputModuleNode
#------------------------------------------------------------------------------
class MainWindow(QMainWindow):
    llm_returned = pyqtSignal(str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        # 信号绑定：绑定 navigation module 的反馈信号
        if hasattr(ros_node, 'nav_feedback_signal'):
            ros_node.nav_feedback_signal.connect(self.update_nav_feedback)
        self.llm_returned.connect(self._on_llm_returned)

        # Widgets
        self.setWindowTitle('Input Module GUI')
        self.image_label = QLabel()
        self.cmd_input = QLineEdit(); self.cmd_input.setPlaceholderText('Enter command ...')
        self.cmd_send_btn = QPushButton('Send')
        self.test_checkbox = QCheckBox('Test')
        self.snapshot_input = QLineEdit(); self.snapshot_input.setPlaceholderText('Snapshot cmd')
        self.savepath_input = QLineEdit(); self.savepath_input.setPlaceholderText('Save path')
        self.output_text = QTextEdit(); self.output_text.setReadOnly(True)

        # Layouts
        control_row = QHBoxLayout()
        control_row.addWidget(self.cmd_input)
        control_row.addWidget(self.cmd_send_btn)

        ctl = QVBoxLayout()
        ctl.addLayout(control_row)
        ctl.addWidget(self.test_checkbox)
        ctl.addWidget(self.snapshot_input)
        ctl.addWidget(self.savepath_input)
        ctl.addWidget(self.output_text)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.image_label)
        main_layout.addLayout(ctl)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Connect signals
        self.cmd_send_btn.clicked.connect(self.on_send)
        self.cmd_input.returnPressed.connect(self.on_send)
        self.test_checkbox.toggled.connect(self.on_toggle)
        self.snapshot_input.returnPressed.connect(self.on_snapshot)

        # Timer for image preview
        self._timer = QTimer(); self._timer.timeout.connect(self.update_image)
        self._timer.start(100)

    def on_toggle(self, checked):
        self.snapshot_input.setVisible(checked)
        self.savepath_input.setVisible(checked)

    def on_send(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        self.cmd_input.clear()
        threading.Thread(
            target=self.ros_node.call_llm,
            args=(cmd, self.llm_returned),
            daemon=True
        ).start()

    def on_snapshot(self):
        cmd = self.snapshot_input.text().strip()
        label = cmd.split()[-1] if cmd else 'sprite'
        savep = self.savepath_input.text().strip()
        threading.Thread(
            target=self.ros_node.perform_snapshot,
            args=(label, savep),
            daemon=True
        ).start()

    def _on_llm_returned(self, text):
        current = self.output_text.toPlainText()
        if current and not current.endswith('\n'):
            current += '\n'
        self.output_text.setText(current + text)

    def update_image(self):
        img = self.ros_node.latest_rgb_image
        if img is None:
            return
        tmp = img.copy()
        if self.ros_node.yolo_lock.acquire(blocking=False):
            try:
                res = self.ros_node.model(tmp, verbose=False)
            finally:
                self.ros_node.yolo_lock.release()
            for box in res[0].boxes if res else []:
                cls_id = int(box.cls)
                if self.ros_node.model_names[cls_id].lower() == 'sprite':
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    cv2.rectangle(tmp, (x1, y1), (x2, y2), (0,0,255), 2)
        h, w, _ = tmp.shape
        qimg = QImage(tmp.data, w, h, w*3, QImage.Format_BGR888)
        self.image_label.setPixmap(QPixmap.fromImage(qimg))

    def update_nav_feedback(self, text):
        current = self.output_text.toPlainText()
        if current and not current.endswith('\n'):
            current += '\n'
        self.output_text.setText(current + text)


#------------------------------------------------------------------------------
# Entry Point
#------------------------------------------------------------------------------
def main():
    rclpy.init()
    ros_node = InputModuleNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
