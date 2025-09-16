#!/usr/bin/env python3
import sys, json
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QWidget, QGroupBox, QLineEdit, QFormLayout, QGridLayout, QSizePolicy, QSpacerItem
)
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
import threading
from rclpy.executors import MultiThreadedExecutor

class GUIWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle('Motion Preparation GUI')

        # Status labels
        self.torso_label = QLabel('Torso height: N/A')
        self.ee_label = QLabel('End effector: N/A')

        # --- Speed & Step Control ---
        self.arm_speed_input = QLineEdit(); self.arm_speed_input.setPlaceholderText('m/s (default 0.08)')
        self.arm_step_input = QLineEdit(); self.arm_step_input.setPlaceholderText('m (default 0.02)')
        self.rot_speed_input = QLineEdit(); self.rot_speed_input.setPlaceholderText('rad/s (default 0.5)')
        self.rot_step_input = QLineEdit(); self.rot_step_input.setPlaceholderText('rad (default 0.1)')
        self.grip_speed_input = QLineEdit(); self.grip_speed_input.setPlaceholderText('m/s (default 0.06)')
        self.grip_step_input = QLineEdit(); self.grip_step_input.setPlaceholderText('m (default 0.01)')
        self.submit_btn = QPushButton('Submit')
        self.submit_btn.clicked.connect(self.on_submit)
        speed_step_form = QFormLayout()
        speed_step_form.addRow('Arm/Torso Speed:', self.arm_speed_input)
        speed_step_form.addRow('Arm/Torso Step Length:', self.arm_step_input)
        speed_step_form.addRow('Rot Speed:', self.rot_speed_input)
        speed_step_form.addRow('Rot Step Length:', self.rot_step_input)
        speed_step_form.addRow('Grip Speed:', self.grip_speed_input)
        speed_step_form.addRow('Grip Step Length:', self.grip_step_input)
        speed_step_form.addWidget(self.submit_btn)
        speed_step_box = QGroupBox('Set Speed and Step (Arm/Torso/Grip)')
        speed_step_box.setLayout(speed_step_form)

        # --- End-Effector Control ---
        ee_box = QGroupBox('Arm End-Effector Control')
        ee_layout = QVBoxLayout()
        directions = [
            ('Up', 'eef_up'), ('Down', 'eef_down'),
            ('Forward', 'eef_forward'), ('Backward', 'eef_backward'),
            ('Left', 'eef_left'), ('Right', 'eef_right')
        ]
        for label, cmd in directions:
            btn = QPushButton(label)
            btn.setCheckable(False)
            btn.clicked.connect(lambda checked, c=cmd: self.btn_press_release(c))
            ee_layout.addWidget(btn)
        ee_box.setLayout(ee_layout)

        # --- Recording & Snapshot Buttons (left, MODIFIED) ---
        self.grab_btn = QPushButton('Grab Recording')
        self.place_btn = QPushButton('Place Recording')
        self.snap_btn = QPushButton('Send Snapshot Request')
        self.end_btn = QPushButton('End Recording')
        self.grab_btn.clicked.connect(self.on_grab)
        self.place_btn.clicked.connect(self.on_place)
        self.snap_btn.clicked.connect(self.on_snap)
        self.end_btn.clicked.connect(self.on_end)
        record_layout = QVBoxLayout()
        record_layout.addWidget(self.grab_btn)
        record_layout.addWidget(self.place_btn)
        record_layout.addWidget(self.snap_btn)
        record_layout.addWidget(self.end_btn)
        record_box = QGroupBox('Recording & Snapshot')
        record_box.setLayout(record_layout)

        # ------- Right column part -------
        # --- Arm Joint Control (1~7) ---
        arm_box = QGroupBox('Arm Joint Control')
        arm_grid = QGridLayout()
        arm_names = [f'Arm {i}' for i in range(1,8)]
        self.arm_btns = {}
        for i, name in enumerate(arm_names):
            lbl = QLabel(name)
            btn_cw = QPushButton('Clockwise')
            btn_ccw = QPushButton('Counter')
            btn_cw.setCheckable(False)
            btn_ccw.setCheckable(False)
            joint_cmd_cw = f'arm{i+1}_clockwise'
            joint_cmd_ccw = f'arm{i+1}_counter'
            btn_cw.clicked.connect(lambda checked, c=joint_cmd_cw: self.btn_press_release(c))
            btn_ccw.clicked.connect(lambda checked, c=joint_cmd_ccw: self.btn_press_release(c))
            arm_grid.addWidget(lbl, i, 0)
            arm_grid.addWidget(btn_cw, i, 1)
            arm_grid.addWidget(btn_ccw, i, 2)
            self.arm_btns[name] = (btn_cw, btn_ccw)
        arm_box.setLayout(arm_grid)

        # --- Torso Control (right) ---
        torso_box = QGroupBox('Torso Control')
        btn_t_up = QPushButton('Up')
        btn_t_down = QPushButton('Down')
        for btn, cmd in [(btn_t_up,'torso_up'),(btn_t_down,'torso_down')]:
            btn.setCheckable(False)
            btn.clicked.connect(lambda checked, c=cmd: self.btn_press_release(c))
        torso_layout = QHBoxLayout()
        torso_layout.addWidget(btn_t_up)
        torso_layout.addWidget(btn_t_down)
        torso_box.setLayout(torso_layout)

        # --- Grip Control (right) ---
        grip_box = QGroupBox('Grip Control')
        btn_grip = QPushButton('Grip (Close)')
        btn_release = QPushButton('Release (Open)')
        for btn, cmd in [(btn_grip,'grip'), (btn_release,'release')]:
            btn.setCheckable(False)
            btn.clicked.connect(lambda checked, c=cmd: self.btn_press_release(c))
        grip_layout = QHBoxLayout()
        grip_layout.addWidget(btn_grip)
        grip_layout.addWidget(btn_release)
        grip_box.setLayout(grip_layout)

        # --- Head Control (right, new) ---
        head_box = QGroupBox('Head Control')
        btn_head_left = QPushButton('Left')
        btn_head_right = QPushButton('Right')
        btn_head_down = QPushButton('Down')
        btn_head_up = QPushButton('Up')
        for btn, cmd in [
            (btn_head_left, 'head_left'), (btn_head_right, 'head_right'),
            (btn_head_down, 'head_down'), (btn_head_up, 'head_up')]:
            btn.setCheckable(False)
            btn.clicked.connect(lambda checked, c=cmd: self.btn_press_release_head(c))
        head_layout = QHBoxLayout()
        head_layout.addWidget(btn_head_left)
        head_layout.addWidget(btn_head_right)
        head_layout.addWidget(btn_head_up)
        head_layout.addWidget(btn_head_down)
        head_box.setLayout(head_layout)

        # --- Home (right) ---
        home_box_right = QGroupBox('Home')
        home_layout_right = QVBoxLayout()
        bt_home_right = QPushButton('Home')
        bt_home_right.clicked.connect(lambda checked: self.btn_press_release('home'))
        home_layout_right.addWidget(bt_home_right)
        home_box_right.setLayout(home_layout_right)

        # --------- Two-column Layout ---------
        left_col = QVBoxLayout()
        left_col.addWidget(self.torso_label)
        left_col.addWidget(self.ee_label)
        left_col.addWidget(speed_step_box)
        left_col.addWidget(ee_box)
        left_col.addWidget(record_box)
        left_col.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        right_col = QVBoxLayout()
        right_col.addWidget(arm_box)
        right_col.addWidget(torso_box)
        right_col.addWidget(grip_box)
        right_col.addWidget(head_box)
        right_col.addWidget(home_box_right)
        right_col.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_col)
        main_layout.addLayout(right_col)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Status update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(100)

    def on_submit(self):
        payload = {
            'speed': self.arm_speed_input.text(),
            'step': self.arm_step_input.text(),
            'rot_speed': self.rot_speed_input.text(),
            'rot_step': self.rot_step_input.text(),
            'grip_speed': self.grip_speed_input.text(),
            'grip_step': self.grip_step_input.text()
        }
        self.node.publish_speed_step(payload)

    def btn_press_release(self, cmd):
        self.node.publish_cmd_list([cmd])

    def btn_press_release_head(self, cmd):
        # head_controller 兼容格式：{"cmd":"head_up","controller":"head_controller"}
        msg = {'cmd': cmd, 'controller': 'head_controller'}
        self.node.publish_cmd_list([msg])

    def on_grab(self):
        self.node.cmd_pub.publish(String(data='grab recording'))
        self.grab_btn.setEnabled(False)
        self.place_btn.setEnabled(True)

    def on_place(self):
        self.node.cmd_pub.publish(String(data='place recording'))
        self.place_btn.setEnabled(False)
        self.grab_btn.setEnabled(True)

    def on_snap(self):
        self.node.snapshot_pub.publish(String(data='sprite'))

    def on_end(self):
        self.node.cmd_pub.publish(String(data='end'))
        self.grab_btn.setEnabled(True)
        self.place_btn.setEnabled(True)

    def update_status(self):
        pos = self.node.latest_joints.get('torso_lift_joint')
        if pos is not None:
            self.torso_label.setText(f'Torso height: {pos:.3f} m')
        try:
            trans = self.node.tf_buffer.lookup_transform(
                'base_link', 'gripper_link', rclpy.time.Time())
            t = trans.transform.translation
            self.ee_label.setText(
                f'End effector: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
        except:
            pass

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.latest_joints = {}
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.cmd_pub = self.create_publisher(
            String, 'record_cmd', 10)
        self.snapshot_pub = self.create_publisher(
            String, 'snapshot', 10)
        self.ctrl_pub = self.create_publisher(
            String, 'control_cmd', 10)
        self.param_pub = self.create_publisher(
            String, 'motion_param', 10)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

    def joint_cb(self, msg: JointState):
        names, pos = msg.name, msg.position
        self.latest_joints = {n: p for n, p in zip(names, pos) if n == 'torso_lift_joint'}

    def publish_cmd_list(self, cmd_list):
        msg = String()
        msg.data = json.dumps(cmd_list)
        self.get_logger().info(f'Publishing control_cmd: {msg.data}')
        self.ctrl_pub.publish(msg)

    def publish_speed_step(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.get_logger().info(f'Publishing motion_param: {msg.data}')
        self.param_pub.publish(msg)

    def main(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        threading.Thread(target=executor.spin, daemon=True).start()

        app = QApplication(sys.argv)
        win = GUIWindow(self)
        win.show()
        app.exec_()

        executor.shutdown()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = GUINode()
    node.main()

if __name__ == '__main__':
    main()
