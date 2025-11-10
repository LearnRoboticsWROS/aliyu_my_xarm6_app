#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped

from my_xarm6_app.llm.ollama_client import OllamaClient

import json
import tf_transformations as tf
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time

from my_xarm6_interfaces.srv import ObjectPosition
from my_xarm6_interfaces.srv import PlacePosition

import threading


class LLMTaskNode(Node):
    def __init__(self):
        super().__init__('llm_task_node')

        # --- Subscriber: comandi alto livello ---
        self.task_sub = self.create_subscription(
            String,
            'llm_task',
            self.task_callback,
            10
        )

        # --- Publisher: pose target per il nodo di motion ---
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            'target_pose',
            10
        )

        # --- TF buffer per recuperare orientamento attuale ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- LLM client ---
        self.ollama = OllamaClient(model='llama3')

        # --- Client per la visione (object_position_server) ---
        self.object_pos_client = self.create_client(ObjectPosition, 'get_object_position')
        self.get_logger().info("Waiting for 'get_object_position' service...")
        while not self.object_pos_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Still waiting for 'get_object_position' service...")

        # Buffer per l'ultima detection
        self.last_detection_pose = None


        self.pick_place_client = self.create_client(PlacePosition, 'pick_place_vision')
        self.get_logger().info("Waiting for 'pick_place_vision' service...")
        while not self.pick_place_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Still waiting for 'pick_place_vision' service...")


        # self.system_prompt = (
        #     "You are a robotics task planner for a 6-DOF manipulator.\n"
        #     "You receive high-level user commands and must output a JSON plan.\n\n"
        #     "Available tools (actions):\n"
        #     "1) detect_object:\n"
        #     "   - Use this when the user asks to find or locate an object by color.\n"
        #     "   - params:\n"
        #     "     { \"color\": <string> }\n"
        #     "   - This will call a vision service and store the last detected pose internally.\n\n"
        #     "2) move_tcp:\n"
        #     "   - Moves the robot TCP to a target pose.\n"
        #     "   - params:\n"
        #     "     {\n"
        #     "       \"frame\": \"link_base\",\n"
        #     "       \"position\": {\"x\": <float>, \"y\": <float>, \"z\": <float>},\n"
        #     "       \"orientation_rpy\": {\"roll\": <float>, \"pitch\": <float>, \"yaw\": <float>} OR null\n"
        #     "     }\n"
        #     "   - If orientation_rpy is null or missing, current TCP orientation is kept.\n\n"
        #     "Your response must be ONLY a valid JSON object of this form:\n"
        #     "{\n"
        #     "  \"actions\": [\n"
        #     "    {\"tool\": \"detect_object\", \"params\": {\"color\": \"red\"}},\n"
        #     "    {\n"
        #     "      \"tool\": \"move_tcp\",\n"
        #     "      \"params\": {\n"
        #     "        \"frame\": \"link_base\",\n"
        #     "        \"position\": {\"x\": 0.3, \"y\": 0.3, \"z\": 0.4},\n"
        #     "        \"orientation_rpy\": null\n"
        #     "      }\n"
        #     "    }\n"
        #     "  ]\n"
        #     "}\n"
        #     "Do not include any extra text or markdown, only JSON."
        # )


        # self.system_prompt = (
        #     "You are a robotics task planner for a 6-DOF manipulator.\n"
        #     "You receive high-level user commands and must output a JSON plan.\n\n"
        #     "Available tools (actions):\n"
        #     "1) detect_object:\n"
        #     "   - Detects an object by color using the camera.\n"
        #     "   - params: { \"color\": <string> }\n\n"
        #     "2) move_tcp:\n"
        #     "   - Moves the robot TCP to a target pose.\n"
        #     "   - params: { \"frame\": \"link_base\", \"position\": {\"x\":<f>,\"y\":<f>,\"z\":<f>}, \"orientation_rpy\": null }\n\n"
        #     "3) pick_place_vision:\n"
        #     "   - Performs a full pick and place using vision.\n"
        #     "   - params:\n"
        #     "     {\n"
        #     "       \"color\": <string>,\n"
        #     "       \"place_position\": {\"x\": <float>, \"y\": <float>, \"z\": <float>}\n"
        #     "     }\n\n"
        #     "Output only valid JSON:\n"
        #     "{ \"actions\": [ {\"tool\": \"pick_place_vision\", \"params\": {\"color\": \"red\", \"place_position\": {\"x\":0.3, \"y\":0.2, \"z\":0.1}} } ] }"
        # )


        # self.get_logger().info("🧠 LLM Task Node ready. Publish high-level commands to /llm_task.")





        self.system_prompt = (
            "You are a robotics task planner for a 6-DOF manipulator.\n"
            "You receive natural-language commands and must output a JSON plan using the available tools.\n\n"
            "Available tools:\n"
            "1) detect_object:\n"
            "   - Detects an object by color using the camera.\n"
            "   - params: { \"color\": <string> }\n\n"
            "2) move_tcp:\n"
            "   - Moves the robot TCP to a target pose.\n"
            "   - params: {\n"
            "       \"frame\": \"link_base\",\n"
            "       \"position\": {\"x\": <float>, \"y\": <float>, \"z\": <float>},\n"
            "       \"orientation_rpy\": {\"roll\": <float>, \"pitch\": <float>, \"yaw\": <float>} OR null\n"
            "     }\n"
            "   - If orientation_rpy is null, the current TCP orientation is preserved.\n\n"
            "3) pick_place_vision:\n"
            "   - Performs a complete pick-and-place operation using vision.\n"
            "   - params: {\n"
            "       \"color\": <string>,\n"
            "       \"place_position\": {\"x\": <float>, \"y\": <float>, \"z\": <float>},\n"
            "       \"pick_orientation_rpy\": {\"roll\": <float>, \"pitch\": <float>, \"yaw\": <float>} OR null,\n"
            "       \"place_orientation_rpy\": {\"roll\": <float>, \"pitch\": <float>, \"yaw\": <float>} OR null\n"
            "     }\n"
            "   - If no orientation is provided, the robot keeps its current orientation.\n\n"
            "Output format (only pure JSON):\n"
            "{\n"
            "  \"actions\": [\n"
            "    {\n"
            "      \"tool\": \"pick_place_vision\",\n"
            "      \"params\": {\n"
            "        \"color\": \"red\",\n"
            "        \"place_position\": {\"x\": 0.3, \"y\": 0.2, \"z\": 0.1},\n"
            "        \"pick_orientation_rpy\": {\"roll\": -3.14, \"pitch\": 0.0, \"yaw\": 0.0},\n"
            "        \"place_orientation_rpy\": {\"roll\": -3.14, \"pitch\": 1.57, \"yaw\": 0.0}\n"
            "      }\n"
            "    }\n"
            "  ]\n"
            "}\n"
            "Do not include explanations or extra text — output only valid JSON."
        )

        self.get_logger().info("🧠 LLM Task Node ready. Publish high-level commands to /llm_task.")


    # ============= CALLBACK PRINCIPLE =============
    def task_callback(self, msg: String):
        user_cmd = msg.data
        self.get_logger().info(f"🧠 Received high-level task: {user_cmd}")
        # Execute task in separate thread for not blocking ROS
        threading.Thread(target=self.handle_llm_task, args=(user_cmd,)).start()

    def handle_llm_task(self, user_cmd: str):
        try:
            raw_reply = self.ollama.chat(self.system_prompt, user_cmd).strip()
            if not raw_reply:
                self.get_logger().error("❌ Empty reply from LLM.")
                return

            if raw_reply.startswith("```"):
                raw_reply = raw_reply.strip("`")
                raw_reply = raw_reply.replace("json", "").strip()

            self.get_logger().info(f"💬 Raw LLM plan: {raw_reply}")
            plan = json.loads(raw_reply)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ JSON parsing error: {e}")
            self.get_logger().error(f"Raw reply was: {raw_reply}")
            return

        actions = plan.get("actions", [])
        if not actions:
            self.get_logger().warn("⚠️ Plan has no actions.")
            return

        # for i, action in enumerate(actions):
        #     tool = action.get("tool")
        #     params = action.get("params", {})
        #     self.get_logger().info(f"▶️ Executing action {i}: tool={tool}, params={params}")

        #     if tool == "move_tcp":
        #         self.execute_move_tcp(params)
        #     elif tool == "detect_object":
        #         self.execute_detect_object(params)
        #     else:
        #         self.get_logger().warn(f"⚠️ Unknown tool '{tool}', skipping.")


        for i, action in enumerate(actions):
            tool = action.get("tool")
            params = action.get("params", {})
            self.get_logger().info(f"▶️ Executing action {i}: tool={tool}, params={params}")

            if tool == "move_tcp":
                self.execute_move_tcp(params)
            elif tool == "detect_object":
                self.execute_detect_object(params)
            elif tool == "pick_place_vision":
                self.execute_pick_place_vision(params)
            else:
                self.get_logger().warn(f"⚠️ Unknown tool '{tool}', skipping.")




    # ============= TOOL: move_tcp =============
    def execute_move_tcp(self, params: dict):
        frame = params.get("frame", "link_base")
        pos = params.get("position", {})
        rpy = params.get("orientation_rpy", None)

        # --- posizione ---
        try:
            x = float(pos.get("x", 0.3))
            y = float(pos.get("y", 0.3))
            z = float(pos.get("z", 0.3))
        except Exception as e:
            self.get_logger().error(f"❌ Invalid position params: {pos}, error: {e}")
            return

        # --- orientamento ---
        if rpy is None:
            # mantieni orientamento attuale del TCP
            self.get_logger().info("🧭 No orientation given, keeping current TCP orientation from TF.")
            qx, qy, qz, qw = self.get_current_tcp_quaternion(frame)
        else:
            try:
                roll = float(rpy.get("roll", 0.0))
                pitch = float(rpy.get("pitch", 1.57))
                yaw = float(rpy.get("yaw", 0.0))
                qx, qy, qz, qw = tf.quaternion_from_euler(roll, pitch, yaw)
            except Exception as e:
                self.get_logger().error(f"❌ Invalid orientation_rpy: {rpy}, error: {e}")
                return
            
        if pos == {} and hasattr(self, "last_detection_pose") and self.last_detection_pose:
            x, y, z = self.last_detection_pose
            self.get_logger().info(f"📍 Using last detected pose as target: ({x:.3f}, {y:.3f}, {z:.3f})")

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.target_pose_pub.publish(pose)
        self.get_logger().info(
            f"📤 Published target pose to /target_pose: "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )

        


    # ============= Utility: orientamento attuale TCP =============
    def get_current_tcp_quaternion(self, frame="link_base"):
        try:
            # lookup_transform(target_frame, source_frame, ...)
            tf_stamped: TransformStamped = self.tf_buffer.lookup_transform(
                frame,
                "link_tcp",
                Time(),
                timeout=Duration(seconds=1.0)
            )
            q = tf_stamped.transform.rotation
            return q.x, q.y, q.z, q.w
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Could not get current TCP orientation from TF, using default. Error: {e}"
            )
            qx, qy, qz, qw = tf.quaternion_from_euler(0.0, 1.57, 0.0)
            return qx, qy, qz, qw
        
    # def execute_detect_object(self, params: dict):
    #     color = params.get("color", "red")
    #     self.get_logger().info(f"👁️ Detecting object with color: {color}")

    #     req = ObjectPosition.Request()
    #     req.object_name = color  # <--- allineato al tuo server

    #     future = self.object_pos_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is None:
    #         self.get_logger().error("❌ Vision service call failed.")
    #         return

    #     res = future.result()
    #     if not res.success:
    #         self.get_logger().warn(f"⚠️ No object of color '{color}' found.")
    #         return

    #     self.last_detection_pose = (res.x, res.y, res.z)
    #     self.get_logger().info(
    #         f"✅ Detected {color} object at X={res.x:.3f}, Y={res.y:.3f}, Z={res.z:.3f}"
    #     )


    def execute_detect_object(self, params: dict):
        color = params.get("color", "red")
        self.get_logger().info(f"👁️ Detecting object with color: {color}")

        req = ObjectPosition.Request()
        req.object_name = color

        future = self.object_pos_client.call_async(req)
        # ✅ invece di spin_until_future_complete, usiamo callback asincrono
        future.add_done_callback(lambda f: self.on_detect_done(color, f))

    def on_detect_done(self, color, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Vision service call failed: {e}")
            return

        if not res.success:
            self.get_logger().warn(f"⚠️ No object of color '{color}' found.")
            return

        self.last_detection_pose = (res.x, res.y, res.z)
        self.get_logger().info(
            f"✅ Detected {color} object at X={res.x:.3f}, Y={res.y:.3f}, Z={res.z:.3f}"
        )

    def execute_pick_place_vision(self, params: dict):
        color = params.get("color", "red")
        place = params.get("place_position", {"x": 0.3, "y": 0.2, "z": 0.15})
        self.get_logger().info(f"🤖 Executing pick & place using vision: color={color}, place={place}")

        req = PlacePosition.Request()
        req.object_name = color
        req.x = float(place.get("x", 0.3))
        req.y = float(place.get("y", 0.2))
        req.z = float(place.get("z", 0.15))

        future = self.pick_place_client.call_async(req)
        future.add_done_callback(lambda f: self.on_pick_place_done(color, f))

    def on_pick_place_done(self, color, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Pick & Place service failed: {e}")
            return
        if res.success:
            self.get_logger().info(f"✅ Pick & Place with {color} object completed successfully.")
        else:
            self.get_logger().warn(f"⚠️ Pick & Place with {color} object failed.")







def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
