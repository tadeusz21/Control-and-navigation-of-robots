#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class RobotGraspingTask : public rclcpp::Node {
public:
    RobotGraspingTask() : Node("robot_grasping_task") {
        // Inicjalizacja MoveGroupInterface z wskaźnikiem do węzła i nazwą grupy
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator" // Zmień "manipulator" na odpowiednią nazwę grupy
        );

        // Pobranie układu bazowego (B) i końcówki (E)
        auto base_frame = move_group_interface_->getPlanningFrame();
        auto end_effector_link = move_group_interface_->getEndEffectorLink();

        RCLCPP_INFO(this->get_logger(), "Układ bazowy (B): %s", base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "Układ końcówki (E): %s", end_effector_link.c_str());

        // Inicjalizacja transform listenera TF2
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Wyznaczenie transformacji F względem E
        // try {
        //     auto transform = tf_buffer_->lookupTransform(end_effector_link, "frame_F", tf2::TimePointZero, tf2::durationFromSec(1.0));
        //     RCLCPP_INFO(this->get_logger(), "Transformacja F względem E: x: %f, y: %f, z: %f",
        //                 transform.transform.translation.x,
        //                 transform.transform.translation.y,
        //                 transform.transform.translation.z);
        // } catch (const tf2::TransformException &ex) {
        //     RCLCPP_ERROR(this->get_logger(), "Nie można znaleźć transformacji F względem E: %s", ex.what());
        // }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotGraspingTask>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}