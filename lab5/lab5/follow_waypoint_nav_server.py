import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs_stero.action import WaypointFollow
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

custom_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self._nav = BasicNavigator()
        self._waypoints = None
        self._waypoints_poses = None
        self._current_pose = None
        self._previous_orientation = None
        self._head_yaw = 0
        self._current_path = None
        self._current_path_length = None
        self._total_path_length = None
        self._paths = []
        self._paths_length = []

        self._action_server = ActionServer(
            self,
            WaypointFollow,
            '/waypoint_follow',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._path_subscriber = self.create_subscription(
            Path, "/plan", self._get_current_path_callback, 100, callback_group=ReentrantCallbackGroup())

        self._amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._get_current_pose_callback, qos_profile=custom_qos, callback_group=ReentrantCallbackGroup())

        self._head_publisher = self.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", qos_profile=custom_qos)

        self.get_logger().info("Waypoint Follower Action Server started.")

    def execute_callback(self, goal_handle):
        feedback = WaypointFollow.Feedback()

        while self._current_pose is None:
            pass

        initial_pose = self._current_pose
        self._previous_orientation = initial_pose.pose.orientation.z
        self._nav.setInitialPose(initial_pose)

        self._waypoints = goal_handle.request.waypoints
        self.get_logger().info("Received goal with waypoints.")

        self._nav.waitUntilNav2Active()
        self._waypoints_poses = self._convert_points_to_poses(self._waypoints)
        self._waypoints_poses.insert(0, initial_pose)

        self.setup_paths_length()

        self._nav.followWaypoints(self._waypoints_poses)

        while self._current_path_length is None:
            pass
        self.get_logger().info("Path set.")
        self.get_logger().info(
            f"Total path: {self._total_path_length:.2f}")

        rate = self.create_rate(0.1)
        while not self._nav.isTaskComplete():
            self.get_logger().info(
                f"current path: {self._current_path_length:.2f}")
            waypoint = self._nav.getFeedback().current_waypoint
            driven_path = sum(self._paths_length[:waypoint]) - self._current_path_length

            percentage_complete = driven_path / self._total_path_length * 100
            percentage_complete = max(percentage_complete, 0)
            feedback.percentage_completed = float(percentage_complete)
            self.get_logger().info(
                f"Path completed: {percentage_complete:.2f}%")

            self.control_head_movement()
            rate.sleep()

        goal_handle.succeed()
        result = WaypointFollow.Result()
        result.status = 0
        return result

    def control_head_movement(self):
        if self._current_pose is not None and self._previous_orientation is not None:

            current_orientation = self._current_pose.pose.orientation.z

            delta_orientation = current_orientation - self._previous_orientation
            self.get_logger().info(f"delta orientation: {delta_orientation}")
            self._previous_orientation = current_orientation

            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ["head_1_joint", "head_2_joint"]
            self._head_yaw += delta_orientation

            point = JointTrajectoryPoint()
            point.positions = [self._head_yaw, 0.0] 
            point.time_from_start.sec = 1
            trajectory_msg.points = [point]

            self._head_publisher.publish(trajectory_msg)


    def _convert_points_to_poses(self, points: list) -> list[PoseStamped]:
        poses = []
        for point in points:
            msg = PoseStamped()
            msg.pose.position.x = point.x
            msg.pose.position.y = point.y
            msg.pose.orientation.z = point.z
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            poses.append(msg)
        return poses

    def _get_current_pose_callback(self, msg: PoseWithCovarianceStamped):
        self._current_pose = PoseStamped()
        self._current_pose.pose = msg.pose.pose
        self._current_pose.header = msg.header

    def _get_current_path_callback(self, msg: Path):
        self._current_path = msg.poses
        self._current_path_length = self.calculate_path_length(self._current_path)

    def calculate_path_length(self, path: list[PoseStamped]) -> float:
        length = 0
        for i in range(1, len(path)):
            dx = path[i].pose.position.x - path[i-1].pose.position.x
            dy = path[i].pose.position.y - path[i-1].pose.position.y
            length += math.sqrt(pow(dx, 2) + pow(dy, 2))
        return length

    def setup_paths_length(self):
        self._total_path_length = 0.0
        for i in range(1, len(self._waypoints_poses)):
            path = self._nav.getPath(self._waypoints_poses[i-1], self._waypoints_poses[i], use_start=True)
            path_length = self.calculate_path_length(path.poses)
            self._total_path_length += path_length
            self._paths_length.append(path_length)

def main(args=None):
    rclpy.init(args=args)

    waypoint_follower = WaypointFollower()
    executor = MultiThreadedExecutor()

    rclpy.spin(waypoint_follower, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
