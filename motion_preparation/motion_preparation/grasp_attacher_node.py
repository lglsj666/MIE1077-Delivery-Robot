import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from linkattacher_msgs.srv import AttachLink, DetachLink
import json
import math
import time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class GraspAttacherNode(Node):
    def __init__(self):
        super().__init__('grasp_attacher_node')

        # --- Initialize AttachLink and DetachLink clients ---
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ATTACHLINK service...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /DETACHLINK service...')
        self.get_logger().info('AttachLink and DetachLink services ready.')

        # --- Subscription and Publisher ---
        self.subscription = self.create_subscription(
            String, 'control_cmd', self.command_callback, 10)
        self.attach_status_pub = self.create_publisher(String, 'attacher_state', 10)
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        # --- TF2 Buffer and Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Timer for delayed attach ---
        self._attach_timer = None

        # --- Attachment state and configuration ---
        self.attached = False
        self.robot_model_name = 'tiago'
        self.eef_links = ["gripper_left_finger_link"]
        self.eef_link_name_in_tf = 'gripper_left_finger_link'  # Modify if needed
        self.sprite_models = [
            "sprite_table0", "sprite_table1", "sprite_table2", "sprite_table3",
            "sprite_table4", "sprite_table9", "sprite_counter1"
        ]
        self.sprite_link = "link"
        self.model_poses = {}  # 最新model位置缓存
        self.last_grip_time = None
        self.pending_grip = False
        self.closest_sprite = None

        self.attached_sprite = None
        self.attached_eef_link = None

    def model_states_callback(self, msg: ModelStates):
        # 缓存所有模型的位置
        for i, name in enumerate(msg.name):
            self.model_poses[name] = msg.pose[i]

    def command_callback(self, msg: String):
        self.get_logger().info(f"Received control command: {msg.data}")
        try:
            cmds = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse control command: {e}")
            return
        if not isinstance(cmds, list):
            cmds = [cmds]

        # --- Handle grip command ---
        if 'grip' in cmds and not self.attached:
            self.find_and_attach_closest_sprite()
        # --- Handle release command ---
        elif 'release' in cmds and self.attached:
            self.try_detach()
        elif 'release' in cmds and not self.attached:
            self.get_logger().info("No attached object, ignoring release command.")
        elif 'grip' in cmds and self.attached:
            self.get_logger().info("Object already attached, ignoring grip command.")

        self.get_logger().info("Waiting for new commands...")

    def find_and_attach_closest_sprite(self):
        # 当前model_poses必须包含tiago和所有sprite
        missing = [m for m in [self.robot_model_name] + self.sprite_models if m not in self.model_poses]
        if missing:
            self.get_logger().error(f"Missing poses for models: {missing}, cannot perform attachment.")
            return

        # --- 获取末端执行器的世界坐标（TF）---
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'map',    # 修正为 tf-tree 中存在的全局 frame
                self.eef_link_name_in_tf,  # 源坐标系（需与TF tree一致）
                rclpy.time.Time())
            ee_x = tf.transform.translation.x
            ee_y = tf.transform.translation.y
            ee_z = tf.transform.translation.z
        except Exception as e:
            self.get_logger().error(f"TF lookup for end-effector failed: {e}")
            return

        # 计算最近sprite及距离
        min_dist = float('inf')
        closest_sprite = None
        for sprite in self.sprite_models:
            pose = self.model_poses[sprite]
            dx = abs(pose.position.x - ee_x)
            dy = abs(pose.position.y - ee_y)
            dz = abs(pose.position.z - ee_z)
            manhattan_dist = dx + dy + dz
            if manhattan_dist < min_dist:
                min_dist = manhattan_dist
                closest_sprite = sprite
        self.get_logger().info(f"Closest sprite: {closest_sprite} at Manhattan dist={min_dist:.3f} m")

        # 只有曼哈顿距离<=0.5m时才执行attach
        if min_dist <= 0.5:
            self.closest_sprite = closest_sprite
            # 启动只触发一次的延迟定时器
            if self._attach_timer is not None:
                self.destroy_timer(self._attach_timer)
            self._attach_timer = self.create_timer(0.5, self._attach_once_callback)
        else:
            self.get_logger().info(f"No sprite within 0.15m of end-effector; not attaching.")

    def _attach_once_callback(self):
        if self._attach_timer is not None:
            self.destroy_timer(self._attach_timer)
            self._attach_timer = None
        self.try_attach_closest()

    def try_attach_closest(self):
        sprite = self.closest_sprite
        eef_link = self.eef_links[0]
        self.get_logger().info(
            f"Attaching {sprite}.{self.sprite_link} to {self.robot_model_name}.{eef_link} (closest sprite)"
        )
        req = AttachLink.Request()
        req.model1_name = self.robot_model_name
        req.link1_name = eef_link
        req.model2_name = sprite
        req.link2_name = self.sprite_link
        future = self.attach_client.call_async(req)
        future.add_done_callback(
            lambda fut, s=sprite, l=eef_link: self.attach_response_callback(fut, s, l)
        )

    def attach_response_callback(self, future, sprite, eef_link):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Attach service call failed: {e}")
            return

        if response.success:
            # --- Attachment succeeded ---
            self.attached = True
            self.attached_sprite = sprite
            self.attached_eef_link = eef_link
            self.get_logger().info(
                f"Successfully attached {sprite}.{self.sprite_link} to {self.robot_model_name}.{eef_link}"
            )
            # Publish attached state
            msg = String()
            msg.data = json.dumps({
                "attached": True,
                "sprite": sprite,
                "eef_link": eef_link
            })
            self.attach_status_pub.publish(msg)
        else:
            self.get_logger().error(
                f"Failed to attach ({sprite}): {response.message}"
            )

    def try_detach(self):
        if not self.attached_sprite or not self.attached_eef_link:
            self.get_logger().error("No attachment info saved, cannot detach.")
            return
        self.get_logger().info(
            f"Detaching: {self.attached_sprite}.{self.sprite_link} from {self.robot_model_name}.{self.attached_eef_link}"
        )
        req = DetachLink.Request()
        req.model1_name = self.robot_model_name
        req.link1_name  = self.attached_eef_link
        req.model2_name = self.attached_sprite
        req.link2_name  = self.sprite_link
        future = self.detach_client.call_async(req)
        future.add_done_callback(self.detach_response_callback)

    def detach_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Detach service call failed: {e}")
            return

        if response.success:
            self.get_logger().info("DetachLink succeeded: object detached.")
            # Reset state
            self.attached = False
            self.attached_sprite = None
            self.attached_eef_link = None
            # Publish detached state
            msg = String()
            msg.data = json.dumps({"attached": False})
            self.attach_status_pub.publish(msg)
        else:
            self.get_logger().error(
                f"DetachLink call failed: {response.message}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = GraspAttacherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
