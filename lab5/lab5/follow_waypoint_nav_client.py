import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs_stero.action import WaypointFollow
from geometry_msgs.msg import Point


class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        
        # Inicjalizacja klienta akcji
        self._action_client = ActionClient(self, WaypointFollow, 'waypoint_follow')

    def send_goal(self, waypoints):
        # Tworzenie celu
        goal_msg = WaypointFollow.Goal()
        goal_msg.waypoints = waypoints

        # Czekanie na dostępność serwera
        self._action_client.wait_for_server()
        
        self.get_logger().info("Sending goal to the waypoint follower...")

        # Wysłanie celu do serwera
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received: {feedback.percentage_completed}% completed.')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received. Task completed.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Inicjalizacja klienta
    waypoint_client = WaypointClient()

    # Tworzenie listy waypointów
    waypoints = []
    
    wp1 = Point()
    wp1.x = 1.0
    wp1.y = 0.0
    wp1.z = 0.0
    waypoints.append(wp1)

    wp2 = Point()
    wp2.x = 2.0
    wp2.y = 0.0
    wp2.z = 0.0
    waypoints.append(wp2)

    wp3 = Point()
    wp3.x = 4.0
    wp3.y = 0.0
    wp3.z = 0.0
    waypoints.append(wp3)

    # Wysłanie celu
    waypoint_client.send_goal(waypoints)

    # Uruchomienie wątku pętli ROS2
    rclpy.spin(waypoint_client)

if __name__ == '__main__':
    main()
