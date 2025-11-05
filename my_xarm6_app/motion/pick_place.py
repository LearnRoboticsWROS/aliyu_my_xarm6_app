# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.srv import GetPositionIK
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.action import FollowJointTrajectory
# from rclpy.action import ActionClient
# from builtin_interfaces.msg import Duration
# from linkattacher_msgs.srv import AttachLink, DetachLink
# from tf_transformations import quaternion_from_euler
# import time


# class PickPlaceNode(Node):
#     def __init__(self):
#         super().__init__('pick_place_node')

#         # 1️⃣ Client IK (MoveIt)
#         self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
#         self.ik_client.wait_for_service()
#         self.get_logger().info('IK service ready.')

#         # 2️⃣ Client Action per braccio
#         self.arm_action = '/xarm6_traj_controller/follow_joint_trajectory'
#         self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)
#         self.arm_client.wait_for_server()
#         self.get_logger().info('Arm trajectory action ready.')

#         # 3️⃣ Client Action per gripper
#         self.gripper_action = '/xarm_gripper_traj_controller/follow_joint_trajectory'
#         self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action)
#         self.gripper_client.wait_for_server()
#         self.get_logger().info('Gripper action ready.')

#         # 4️⃣ Client per servizi di attach/detach
#         self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
#         self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')
#         self.attach_srv.wait_for_service()
#         self.detach_srv.wait_for_service()
#         self.get_logger().info('Link attacher services ready.')

#         # 5️⃣ Sequenza pick & place
#         self.execute_sequence()

#     # ---------------------------------------------------------------------
#     def compute_ik(self, x, y, z, roll, pitch, yaw, link_name='link_tcp'):
#         """Richiede la soluzione IK per la posa desiderata."""
#         pose = PoseStamped()
#         pose.header.frame_id = 'link_base'
#         pose.header.stamp = self.get_clock().now().to_msg()

#         qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = z
#         pose.pose.orientation.x = qx
#         pose.pose.orientation.y = qy
#         pose.pose.orientation.z = qz
#         pose.pose.orientation.w = qw

#         req = GetPositionIK.Request()
#         req.ik_request.group_name = 'xarm6'
#         req.ik_request.ik_link_name = link_name
#         req.ik_request.pose_stamped = pose
#         req.ik_request.avoid_collisions = True

#         future = self.ik_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         res = future.result()

#         if not res or res.error_code.val != res.error_code.SUCCESS:
#             self.get_logger().error('IK computation failed.')
#             return None

#         js = res.solution.joint_state
#         name_to_pos = dict(zip(js.name, js.position))
#         arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
#         return [name_to_pos[j] for j in arm_joints if j in name_to_pos]

#     # ---------------------------------------------------------------------
#     def move_arm(self, positions, duration=3.0):
#         """Invia una traiettoria al controller del braccio."""
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start = Duration(sec=int(duration))
#         goal.trajectory.points.append(point)

#         self.get_logger().info('Sending arm trajectory...')
#         future = self.arm_client.send_goal_async(goal)
#         rclpy.spin_until_future_complete(self, future)
#         handle = future.result()
#         result_future = handle.get_result_async()
#         rclpy.spin_until_future_complete(self, result_future)
#         self.get_logger().info('Arm motion complete.')

#     # ---------------------------------------------------------------------
#     def move_gripper(self, position, duration=1.0):
#         """Muove il gripper aprendo o chiudendo."""
#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = ['drive_joint']
#         point = JointTrajectoryPoint()
#         point.positions = [position]
#         point.time_from_start = Duration(sec=int(duration))
#         goal.trajectory.points.append(point)

#         self.get_logger().info(f'Moving gripper to {position} rad...')
#         future = self.gripper_client.send_goal_async(goal)
#         rclpy.spin_until_future_complete(self, future)
#         handle = future.result()
#         result_future = handle.get_result_async()
#         rclpy.spin_until_future_complete(self, result_future)
#         self.get_logger().info('Gripper motion complete.')

#     # ---------------------------------------------------------------------
#     def attach_object(self):
#         req = AttachLink.Request()
#         req.model1_name = 'UF_ROBOT'
#         req.link1_name = 'link6'
#         req.model2_name = 'red_cube'
#         req.link2_name = 'link_0'
#         self.get_logger().info('Attaching object...')
#         self.attach_srv.call_async(req)

#     def detach_object(self):
#         req = DetachLink.Request()
#         req.model1_name = 'UF_ROBOT'
#         req.link1_name = 'link6'
#         req.model2_name = 'red_cube'
#         req.link2_name = 'link_0'
#         self.get_logger().info('Detaching object...')
#         self.detach_srv.call_async(req)

#     # ---------------------------------------------------------------------
#     def execute_sequence(self):
#         """Sequenza completa pick & place"""

#         # Orientamento di grasp (stesso del tf2_echo)
#         roll, pitch, yaw = (-3.141, 0.0, 0.0)

#         # Punti di riferimento
#         grasp = [0.105, 0.508, 0.027]
#         pre_grasp = [grasp[0], grasp[1], grasp[2] + 0.1]
#         post_grasp = pre_grasp
#         place = [0.3, 0.3, 0.1]

#         # Pre-grasp
#         self.get_logger().info('Going to pre-grasp...')
#         pre_grasp_pos = self.compute_ik(*pre_grasp, roll, pitch, yaw)
#         self.move_arm(pre_grasp_pos)

#         # Grasp
#         self.get_logger().info('Going to grasp position...')
#         grasp_pos = self.compute_ik(*grasp, roll, pitch, yaw)
#         self.move_arm(grasp_pos)

#         # Chiudi gripper e attacca
#         self.move_gripper(0.17)   # ≈10°
#         time.sleep(0.5)
#         self.attach_object()
#         time.sleep(1.0)

#         # Post-grasp
#         self.get_logger().info('Lifting object...')
#         self.move_arm(pre_grasp_pos)

#         # Place
#         self.get_logger().info('Moving to place...')
#         place_pos = self.compute_ik(*place, roll, pitch, yaw)
#         self.move_arm(place_pos)

#         # Rilascia
#         self.move_gripper(0.0)
#         time.sleep(0.5)
#         self.detach_object()
#         time.sleep(1.0)

#         # Torna su
#         self.get_logger().info('Returning to home height...')
#         self.move_arm(pre_grasp_pos)

#         self.get_logger().info('✅ Pick & Place completed successfully.')


# def main(args=None):
#     rclpy.init(args=args)
#     node = PickPlaceNode()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from tf_transformations import quaternion_from_euler
import time


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        # === Clients initialization ===
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/xarm6_traj_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory')
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK')

        # Wait for all services
        for name, client in [
            ("IK service", self.ik_client),
            ("Arm action", self.arm_client),
            ("Gripper action", self.gripper_client),
            ("AttachLink service", self.attach_srv),
            ("DetachLink service", self.detach_srv),
        ]:
            client.wait_for_service() if hasattr(client, 'wait_for_service') else client.wait_for_server()
            self.get_logger().info(f'{name} ready.')

        self.execute_sequence()

    # ---------------------------------------------------------------------
    def compute_ik(self, x, y, z, roll, pitch, yaw, link_name='link_tcp'):
        pose = PoseStamped()
        pose.header.frame_id = 'link_base'
        pose.header.stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'xarm6'
        req.ik_request.ik_link_name = link_name
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res or res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error('IK failed.')
            return None

        js = res.solution.joint_state
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        name_to_pos = dict(zip(js.name, js.position))
        return [name_to_pos[j] for j in arm_joints if j in name_to_pos]

    # ---------------------------------------------------------------------
    def move_arm(self, positions, duration=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points.append(point)

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Arm motion complete.')

    # ---------------------------------------------------------------------
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
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper motion complete.')

    # ---------------------------------------------------------------------
    def attach_object(self, model2, link2):
        req = AttachLink.Request()
        req.model1_name = 'UF_ROBOT'
        req.link1_name = 'link6'
        req.model2_name = model2
        req.link2_name = link2
        self.get_logger().info(f'Attaching {model2}...')
        self.attach_srv.call_async(req)

    def detach_object(self, model2, link2):
        req = DetachLink.Request()
        req.model1_name = 'UF_ROBOT'
        req.link1_name = 'link6'
        req.model2_name = model2
        req.link2_name = link2
        self.get_logger().info(f'Detaching {model2}...')
        self.detach_srv.call_async(req)

    # ---------------------------------------------------------------------
    def pick_and_place(self, name, link, grasp, pre_grasp, post_grasp, place, rpy):
        roll, pitch, yaw = rpy
        self.get_logger().info(f'▶️ Starting Pick & Place for {name}')

        # Pre-grasp
        pre_grasp_pos = self.compute_ik(*pre_grasp, roll, pitch, yaw)
        self.move_arm(pre_grasp_pos)

        # Grasp
        grasp_pos = self.compute_ik(*grasp, roll, pitch, yaw)
        self.move_arm(grasp_pos)

        # Close gripper + attach
        self.move_gripper(0.17)
        time.sleep(0.5)
        self.attach_object(name, link)
        time.sleep(1.0)

        # Post-grasp
        post_pos = self.compute_ik(*post_grasp, roll, pitch, yaw)
        self.move_arm(post_pos)

        # Move to place
        place_pos = self.compute_ik(*place, roll, pitch, yaw)
        self.move_arm(place_pos)

        # Open gripper + detach
        self.move_gripper(0.0)
        time.sleep(0.5)
        self.detach_object(name, link)
        time.sleep(1.0)

        # Lift
        self.move_arm(post_pos)
        self.get_logger().info(f'✅ {name} pick & place done.')

    # ---------------------------------------------------------------------
    def execute_sequence(self):
        """Run both pick & place sequences"""

        # === Red cube ===
        grasp_cube = [0.105, 0.508, 0.027]
        pre_cube = [grasp_cube[0], grasp_cube[1], grasp_cube[2] + 0.1]
        post_cube = pre_cube
        place_cube = [0.3, 0.3, 0.1]
        rpy_cube = (-3.141, 0.0, 0.0)
        self.pick_and_place('red_cube', 'link_0', grasp_cube, pre_cube, post_cube, place_cube, rpy_cube)

        # === Green cylinder ===
        grasp_cyl = [-0.069, 0.508, 0.095]
        pre_cyl = [grasp_cyl[0] + 0.1, grasp_cyl[1], grasp_cyl[2]]  # x + 1.0
        post_cyl = [grasp_cyl[0], grasp_cyl[1], grasp_cyl[2] + 0.1]
        place_cyl = [-0.3, 0.3, 0.1]
        rpy_cyl = (-3.14, 1.57, 0.0)
        self.pick_and_place('green_cylinder', 'link_1', grasp_cyl, pre_cyl, post_cyl, place_cyl, rpy_cyl)

        self.get_logger().info('🎯 All pick & place sequences completed.')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
