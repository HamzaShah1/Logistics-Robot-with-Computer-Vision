import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveToXYZ(Node):
    def __init__(self):
        super().__init__("move_to_xyz")

        # ---- ROS params (xyz input) ----
        self.declare_parameter("x", 0.2)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.3)
        self.declare_parameter("duration", 2.5)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.duration = float(self.get_parameter("duration").value)

        # ---- MoveIt settings ----
        self.group_name = "arm"
        self.ik_link_name = "claw_support"
        self.robot_base_frame = "base_link"   # pose is expressed in this frame

        # ---- Publisher to your existing controller ----
        self.traj_topic = "/arm_controller/joint_trajectory"
        self.pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        # ---- IK service client ----
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available. Is MoveIt running?")
            raise RuntimeError("/compute_ik service not available")

        self.get_logger().info(
            f"Requesting IK for x={self.x}, y={self.y}, z={self.z} in frame {self.robot_base_frame}"
        )

        # Do one-shot execution
        self.timer = self.create_timer(0.5, self.run_once)
        self.sent = False

    def run_once(self):
        if self.sent:
            return
        self.sent = True

        # 1) Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = False


        pose = PoseStamped()
        pose.header.frame_id = self.robot_base_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z

        # Orientation: keep neutral quaternion (no rotation)
        pose.pose.orientation.w = 1.0

        req.ik_request.pose_stamped = pose

        # 2) Call IK service
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.on_ik_result)

    def on_ik_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            return

        # MoveIt error codes: SUCCESS = 1
        if res.error_code.val != 1:
            self.get_logger().error(f"IK failed. error_code = {res.error_code.val}")
            return

        # 3) Extract joint solution
        joint_state = res.solution.joint_state
        names = list(joint_state.name)
        positions = list(joint_state.position)

        # Filter to your 3 joints in correct order
        wanted = ["joint_1", "joint_2", "joint_3"]
        target = []
        for j in wanted:
            if j not in names:
                self.get_logger().error(f"IK solution missing {j}. Got joints: {names}")
                return
            target.append(float(positions[names.index(j)]))

        self.get_logger().info(f"IK solution (joint_1..3): {target}")

        # ---- Go → wait 5s → return home (3-point trajectory) ----
        home = [0.0, -0.3, -0.3]   # <-- set your HOME joint angles here
        wait_time = 5.0

        t_to_target = float(self.duration)   # time to reach target
        t_hold = wait_time                   # wait at target
        t_return = float(self.duration)      # time to return home (reuse duration)

        traj = JointTrajectory()
        traj.joint_names = wanted

        # Point 1: reach TARGET
        p1 = JointTrajectoryPoint()
        p1.positions = target
        s1 = int(t_to_target)
        n1 = int((t_to_target - s1) * 1e9)
        p1.time_from_start = Duration(sec=s1, nanosec=n1)

        # Point 2: HOLD at TARGET (same joints, later time)
        p_hold = JointTrajectoryPoint()
        p_hold.positions = target
        th = t_to_target + t_hold
        sh = int(th)
        nh = int((th - sh) * 1e9)
        p_hold.time_from_start = Duration(sec=sh, nanosec=nh)

        # Point 3: return HOME
        p2 = JointTrajectoryPoint()
        p2.positions = home
        t2 = t_to_target + t_hold + t_return
        s2 = int(t2)
        n2 = int((t2 - s2) * 1e9)
        p2.time_from_start = Duration(sec=s2, nanosec=n2)

        traj.points = [p1, p_hold, p2]

        self.pub.publish(traj)
        self.get_logger().info("Published: go → wait 5s → return home")

        # End node shortly after
        self.create_timer(1.0, self.shutdown)

    def shutdown(self):
        self.get_logger().info("Done.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveToXYZ()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
