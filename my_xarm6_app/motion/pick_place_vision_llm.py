#!/usr/bin/env python3
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

import rclpy
from rclpy.duration import Duration as RosDuration
from rclpy.time import Time


class PickPlaceVisionLLMNode(Node):
    def __init__(self):
        super().__init__('pick_place_vision_llm_node')

        self.model_map = {
            "red":   {"model": "red_cube", "link": "link_0"},
            "green": {"model": "green_cylinder", "link": "link_1"}
        }

        

        # --- Service server per ricevere richieste LLM ---
        self.pick_place_srv = self.create_service(
            PlacePosition,
            'pick_place_vision',
            self.handle_pick_place
        )

        # --- Client per visione ---
        self.vision_client = self.create_client(ObjectPosition, 'get_object_position')
        self.vision_client.wait_for_service()

        # --- TF2 per trasformazioni ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- IK & Trajectory clients ---
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/xarm6_traj_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory')
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')

        self.get_logger().info("🤖 PickPlaceVision LLM Node ready. Waiting for LLM service calls...")

    # =====================================================================

    def handle_pick_place(self, request, response):
        color = request.object_name.lower().strip()
        place_target = [request.x, request.y, request.z]
        self.get_logger().info(f"🎯 Received pick & place request from LLM: color={color}, place={place_target}")

        # --- 1. Detect object ---
        grasp_pose = self.get_object_position_base(color)
        if not grasp_pose:
            self.get_logger().error(f"❌ Could not detect {color} object.")
            response.success = False
            return response

        # --- 2. Get model and link ---
        if color in self.model_map:
            model = self.model_map[color]["model"]
            link = self.model_map[color]["link"]
        else:
            self.get_logger().warn(f"⚠️ Unknown color '{color}', using defaults.")
            model, link = f"{color}_object", "link_0"

        # --- 3. Dynamic orientation ---
        pick_rpy = getattr(request, "pick_orientation_rpy", None)
        place_rpy = getattr(request, "place_orientation_rpy", None)

        # If not defined, keep the actial oreitnation
        if pick_rpy is None:
            pick_rpy = self.get_current_tcp_rpy()
            self.get_logger().info("🧭 Using current TCP orientation for PICK.")
        if place_rpy is None:
            place_rpy = pick_rpy
            self.get_logger().info("↩️ Using same orientation for PLACE (not specified).")

        # --- 4. Define intermediate ---
        pre_pose = [grasp_pose[0], grasp_pose[1], grasp_pose[2] + 0.1]
        post_pose = pre_pose

        # --- 5.execution pick & place ---
        try:
            self.pick_and_place(
                model=model,
                link=link,
                grasp=grasp_pose,
                pre=pre_pose,
                post=post_pose,
                place=place_target,
                pick_rpy=pick_rpy,
                place_rpy=place_rpy
            )
            self.get_logger().info(f"✅ Pick & place of {color} ({model}::{link}) completed successfully.")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"❌ Pick & place failed: {e}")
            response.success = False

        return response

    # =====================================================================
    def get_object_position_base(self, object_name):
        """Richiede posizione 3D dal servizio e trasforma in frame link_base."""
        req = ObjectPosition.Request()
        req.object_name = object_name
        future = self.vision_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.success:
            return None

        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_optical_link'
        point_cam.point.x = res.x
        point_cam.point.y = res.y
        point_cam.point.z = res.z

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                'camera_optical_link',
                Time(),
                timeout=RosDuration(seconds=1.0)
            )
            point_base = do_transform_point(point_cam, transform)
            return [point_base.point.x, point_base.point.y, point_base.point.z]
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    # =====================================================================
    def compute_ik(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = qx, qy, qz, qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = 'link_tcp'
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().warn("IK failed.")
            return None

        js = res.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        return [name_to_pos[j] for j in ['joint1','joint2','joint3','joint4','joint5','joint6']]

    # =====================================================================
    def move_arm(self, positions, duration=3.0):
        if not positions:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        rclpy.spin_until_future_complete(self, handle.get_result_async())

    def move_gripper(self, position, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['drive_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        rclpy.spin_until_future_complete(self, handle.get_result_async())

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

    def get_current_tcp_rpy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',
                'link_tcp',
                Time(),
                timeout=RosDuration(seconds=1.0)
            )
            q = transform.transform.rotation
            import tf_transformations as tf
            r, p, y = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return (r, p, y)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not get current TCP orientation: {e}")
            return (-3.141, 0.0, 0.0)

     # =====================================================================

    def pick_and_place(self, model, link, grasp, pre, post, place, pick_rpy, place_rpy):
        roll_p, pitch_p, yaw_p = pick_rpy
        roll_pl, pitch_pl, yaw_pl = place_rpy

        # Avvicinamento e grasp
        for pose in [pre, grasp]:
            pos = self.compute_ik(*pose, roll_p, pitch_p, yaw_p)
            self.move_arm(pos)

        self.move_gripper(0.17)
        self.attach_object(model, link)

        # Post grasp
        pos = self.compute_ik(*post, roll_p, pitch_p, yaw_p)
        self.move_arm(pos)

        # Movimento verso il place (nuovo orientamento)
        pos = self.compute_ik(*place, roll_pl, pitch_pl, yaw_pl)
        self.move_arm(pos)

        # Release
        self.move_gripper(0.0)
        self.detach_object(model, link)

        # Torna in posizione post pick
        self.move_arm(post)



def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceVisionLLMNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
