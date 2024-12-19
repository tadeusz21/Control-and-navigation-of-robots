import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Navigation node started, waiting for action server...")

    def send_navigation_goal(self, x, y, z, qx, qy, qz, qw):
        # Tworzymy komunikat typu PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # Układ odniesienia
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Ustawiamy pozycję i orientację celu
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        # Wysyłamy cel
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal)

    def get_pose_for_room(self, room_name):
        """
        Funkcja zwraca pozycję (x, y, z) oraz orientację (quaternion) w zależności od nazwy pokoju.
        """
        room_name = room_name.lower()

        # Definiowanie pozycji nawigacji dla każdego pokoju
        if room_name == 'hall':
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        elif room_name == 'kitchen':
            return 3.5, -4.5, 0.0, 0.0, 0.0, 0.0, 1.0
        elif room_name == 'wc':
            return 1.0, -4.0, 0.0, 0.0, 0.0, 0.0, 1.0
        elif room_name == 'living':
            return 2.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0
        elif room_name == 'bedroom':
            return 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        elif room_name == 'storage':
            return 11.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0
        else:
            return None  # Jeśli nie rozpoznamy pokoju


def main(args=None):
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    # Pętla do wyboru pokoju przez użytkownika
    while True:
        place = input("Wprowadź nazwę miejsca (lub wpisz 'koniec' aby zakończyć): ")

        if place.lower() == 'koniec':
            break

        # Pobieramy pozycję dla wybranego pokoju
        position = navigation_node.get_pose_for_room(place)

        if position:
            navigation_node.send_navigation_goal(*position)
            print(f"Wysyłam do pokoju: {place}")
        else:
            print(f"Nie rozpoznałem pokoju o nazwie {place}. Proszę spróbować ponownie.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
