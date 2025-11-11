#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from my_xarm6_interfaces.srv import ObjectPosition
from my_xarm6_interfaces.srv import PlacePosition


class PickPlaceVisionLLMNode(Node):
    def __init__(self):
        super().__init__('pick_place_vision_llm_node')

        # Mappa colore -> modello gazebo e link
        self.model_map = {
            "red":   {"model": "red_cube",       "link": "link_0"},
            "green": {"model": "green_cylinder", "link": "link_1"},
        }

        # --- Service server per richieste da LLM ---
        # PlacePosition.srv:
        # string object_name
        # float64 x
        # float64 y
        # float64 z
        # ---
        # bool success
        self.pick_place_srv = self.create_service(
            PlacePosition,
            'pick_place_vision',
            self.handle_pick_place
        )

        # --- Client per visione ---
        self.vision_client = self.create_client(ObjectPosition, 'get_object_position')
        self.get_logger().info("⏳ Waiting for 'get_object_position' service...")
        self.vision_client.wait_for_service()
        self.get_logger().info("✅ 'get_object_position' service is ready.")

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- IK & Trajectory clients ---
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm6_traj_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory'
        )
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')

        # Aspetta che tutto sia pronto (esattamente come nel nodo che funziona)
        for name, client in [
            ("IK service", self.ik_client),
            ("Arm action", self.arm_client),
            ("Gripper action", self.gripper_client),
            ("AttachLink", self.attach_srv),
            ("DetachLink", self.detach_srv),
        ]:
            if hasattr(client, 'wait_for_service'):
                client.wait_for_service()
            else:
                client.wait_for_server()
            self.get_logger().info(f'{name} ready.')

        self.get_logger().info("🤖 PickPlaceVision LLM Node ready. Waiting for LLM service calls.")

    # =====================================================================
    #  CALLBACK DEL SERVICE CHIAMATO DA llm_task_node
    # =====================================================================
    def handle_pick_place(self, request, response):
        # PlacePosition.srv: object_name = colore
        color = request.object_name.lower().strip()
        place_target = [request.x, request.y, request.z]
        self.get_logger().info(
            f"🎯 Received pick & place request from LLM: color={color}, place={place_target}"
        )

        # Avviamo la sequenza in un thread separato per NON bloccare il service
        import threading
        threading.Thread(
            target=self._run_pick_place_sequence,
            args=(color, place_target),
            daemon=True
        ).start()

        # Al chiamante (llm_task_node) rispondiamo solo che il comando è stato accettato.
        response.success = True
        return response

    # =====================================================================
    #  LOGICA REALE DI PICK & PLACE (in un thread separato)
    # =====================================================================
    def _run_pick_place_sequence(self, color: str, place_target):
        self.get_logger().info(f"▶️ Starting pick&place thread for color={color}...")

        # 1) Leggi posizione oggetto nel frame base
        grasp_pose = self.get_object_position_base(color)
        if not grasp_pose:
            self.get_logger().error(f"❌ Could not get position for object '{color}'. Aborting.")
            return

        # 2) Scegli il modello/link corretti per Gazebo
        if color in self.model_map:
            model = self.model_map[color]["model"]
            link = self.model_map[color]["link"]
        else:
            self.get_logger().warn(f"⚠️ Unknown color '{color}', using default model/link.")
            model, link = f"{color}_object", "link_0"

        self.get_logger().info(f"🧩 Using Gazebo object: {model}::{link}")

        # 3) Orientazione (come nel nodo originale)
        roll, pitch, yaw = (-3.141, 0.0, 0.0)

        # 4) Pose intermedie
        pre_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + 0.10]
        post_pose = pre_pose

        # 5) Esegui pick&place (stessa logica del nodo che funziona)
        try:
            self.pick_and_place(
                model=model,
                link=link,
                grasp=grasp_pose,
                pre=pre_pose,
                post=post_pose,
                place=place_target,
                rpy=(roll, pitch, yaw)
            )
            self.get_logger().info(f"✅ Pick & place of {color} completed.")
        except Exception as e:
            self.get_logger().error(f"❌ Exception in pick&place sequence: {e}")

    # =====================================================================
    #  Visione: camera_optical_link → link_base (COPIATO DAL NODO FUNZIONANTE)
    # =====================================================================
    def get_object_position_base(self, object_name):
        """Richiede posizione 3D dal servizio e trasforma in frame link_base."""
        req = ObjectPosition.Request()
        req.object_name = object_name
        future = self.vision_client.call_async(req)

        # Aspettare la risposta mentre lo spin principale continua a girare
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        res = future.result()
        if not res or not res.success:
            self.get_logger().error(f"Failed to get position of {object_name}")
            return None

        # Punto nel frame camera_optical_link
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_optical_link'
        point_cam.point.x = res.x
        point_cam.point.y = res.y
        point_cam.point.z = res.z

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                'camera_optical_link',
                rclpy.time.Time()
            )
            point_base = do_transform_point(point_cam, transform)
            xyz = [point_base.point.x, point_base.point.y, point_base.point.z]
            self.get_logger().info(f"📍 Object '{object_name}' in base frame: {xyz}")
            return xyz
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    # =====================================================================
    #  IK (COPIATO DAL NODO FUNZIONANTE)
    # =====================================================================
    def compute_ik(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        res = future.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error("IK failed.")
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        return [name_to_pos[j] for j in [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]]

    # =====================================================================
    #  Movimento braccio e gripper (COPIATO DAL NODO FUNZIONANTE)
    # =====================================================================
    def move_arm(self, positions, duration=3.0):
        if not positions:
            self.get_logger().error("Invalid joint positions, skipping arm motion.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)

        future = self.arm_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        handle = future.result()
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

    def move_gripper(self, position, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)

        future = self.gripper_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        handle = future.result()
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

    # =====================================================================
    #  Attach / Detach (COPIATO)
    # =====================================================================
    def attach_object(self, model, link):
        req = AttachLink.Request()
        req.model1_name, req.link1_name = 'UF_ROBOT', 'link6'
        req.model2_name, req.link2_name = model, link
        self.attach_srv.call_async(req)

    def detach_object(self, model, link):
        req = DetachLink.Request()
        req.model1_name, req.link1_name = 'UF_ROBOT', 'link6'
        req.model2_name, req.link2_name = model, link
        self.detach_srv.call_async(req)

    # =====================================================================
    #  Sequenza pick & place (IDENTICA AL NODO FUNZIONANTE)
    # =====================================================================
    def pick_and_place(self, model, link, grasp, pre, post, place, rpy):
        roll, pitch, yaw = rpy

        # Pre-grasp e grasp
        for pose in [pre, grasp]:
            pos = self.compute_ik(*pose, roll, pitch, yaw)
            self.move_arm(pos)

        # Chiudi gripper e attacca oggetto
        self.move_gripper(0.17)
        self.attach_object(model, link)

        # Post-grasp
        pos = self.compute_ik(*post, roll, pitch, yaw)
        self.move_arm(pos)

        # Pose di place
        pos = self.compute_ik(*place, roll, pitch, yaw)
        self.move_arm(pos)

        # Rilascia e torna al post
        self.move_gripper(0.0)
        self.detach_object(model, link)
        self.move_arm(post)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceVisionLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
