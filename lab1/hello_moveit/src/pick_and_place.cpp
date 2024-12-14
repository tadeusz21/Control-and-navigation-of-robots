#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_msgs/msg/collision_object.hpp>

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = transform.translation().x();
    pose_msg.position.y = transform.translation().y();
    pose_msg.position.z = transform.translation().z();
    Eigen::Quaterniond q(transform.rotation());
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    return pose_msg;
}
int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "pick_and_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("pick_and_place");

  // Create a client to the /get_entity_state service
  auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "green_cube_3";

  // Wait for the service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger, "Service /get_entity_state not available.");
    return 1;
  }

  // Declare a variable to store the position
  geometry_msgs::msg::Point position;

  // Call the service and retrieve the position
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    position = result.get()->state.pose.position;
    RCLCPP_INFO(logger, "Green Cube Position: x = %.2f, y = %.2f, z = %.2f", position.x, position.y, position.z);
  } else {
    RCLCPP_ERROR(logger, "Failed to call service /get_entity_state");
    return 1;
  }

  // Initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Visualize the position of the green cube in RViz
  Eigen::Isometry3d cube_pose = Eigen::Isometry3d::Identity();
  cube_pose.translation().x() = position.x;
  cube_pose.translation().y() = position.y;
  cube_pose.translation().z() = position.z;
  
  // Publish an axis at the cube's position to represent its coordinate frame
  moveit_visual_tools.publishAxis(cube_pose, 0.1, 0.01, "green_cube_3");  // Adjust size as needed

  Eigen::Isometry3d T_FE = Eigen::Translation3d(0.0, 0.0, 0.046) *
  Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5).normalized();

  Eigen::Isometry3d T_BF = Eigen::Translation3d(0.5810065155903941, 0.12452790088114665, 0.6040067554081324) *
  Eigen::Quaterniond(-0.015403366175301204, -0.9974549831560183, -0.0160488771360209, -0.06774013898989165).normalized();

  Eigen::Isometry3d T_BC = Eigen::Translation3d(0.6129532489572659, 0.12172421971793858, 0.33512117754742166) *
  Eigen::Quaterniond(-2.121740371074175e-05, -0.0003902260631371974, 0.014435026569939098, 0.999895733204901).normalized();

  Eigen::Isometry3d T_BT = Eigen::Translation3d(0.696705, -0.052918, 0.15) *
  Eigen::Quaterniond(0.0, 0.0, 0.0, -1.0).normalized();

  Eigen::Isometry3d T_GC = Eigen::Translation3d(-0.3, 0.0, 0.0) *
  Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0).normalized();

  Eigen::Isometry3d T_spin = Eigen::Translation3d(0.0, 0.0, 0.0) *
  Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0).normalized();

  Eigen::Isometry3d T_BE = T_BF * T_FE;
  Eigen::Isometry3d T_CE  = T_BC.inverse() * T_BE;

  Eigen::Isometry3d end_effector_pose = cube_pose * T_CE * T_spin;
  Eigen::Isometry3d pre_grip_pose = end_effector_pose * T_GC * T_spin;



  moveit_visual_tools.publishAxis(end_effector_pose, 0.5, 0.01, "end_effector");  // Adjust size as needed
  moveit_visual_tools.publishAxis(pre_grip_pose, 0.1, 0.05, "pre_grisp");
  moveit_visual_tools.trigger();



using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(node, "arm_torso"); 

  move_group_interface.setMaxVelocityScalingFactor(1.0); 
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



  auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");
  // auto move_group_interface_arm = MoveGroupInterface(node, "arm");
  // move_group_interface_arm.setMaxVelocityScalingFactor(1.0); 
  // move_group_interface_arm.setMaxAccelerationScalingFactor(1.0);



  auto table_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), table_pos = T_BT] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "table_collision_object";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 1.0;
        primitive.dimensions[primitive.BOX_Z] = 0.3;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose = tf2::toMsg(table_pos);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();



auto cube_collision_object = [frame_id =
    move_group_interface.getPlanningFrame(), cube_pos = cube_pose] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "green_cube_3_addedd";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.07;
        primitive.dimensions[primitive.BOX_Y] = 0.07;
        primitive.dimensions[primitive.BOX_Z] = 0.07;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose = tf2::toMsg(cube_pos);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(table_collision_object);
    collision_objects.push_back(cube_collision_object);

     std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};
    planning_scene_interface.addCollisionObjects(collision_objects);
  

 move_group_interface.setJointValueTarget(std::vector<double>({0.187, 130.0/180.0*3.14, 41.0/180*3.14, 0.0, 63.0/180.0*3.14, 0.0, 0.0, -78.0/180.0*3.14})); 
 moveit::planning_interface::MoveGroupInterface::Plan plan_0;
  auto const success_0 = static_cast<bool>(move_group_interface.plan(plan_0));


  if (success_0) {
  move_group_interface.execute(plan_0);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}


  move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.05, 0.05}));
  moveit::planning_interface::MoveGroupInterface::Plan plan_grip_1;
  auto const success_grip_1 = static_cast<bool>(move_group_interface_gripper.plan(plan_grip_1));

  move_group_interface_gripper.setMaxVelocityScalingFactor(1.0); 
  move_group_interface_gripper.setMaxAccelerationScalingFactor(1.0);



  if (success_grip_1) {
  move_group_interface.execute(plan_grip_1);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}



// Skonwertuj wyliczoną pozycję na geometry_msgs::msg::Pose
geometry_msgs::msg::Pose pre_target_pose = toPoseMsg(pre_grip_pose);

// Ustaw cel pozycji
move_group_interface.setPoseTarget(pre_target_pose);

// Stwórz plan do tej pozycji
auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
}();

// Wykonaj plan
if (success) {
    move_group_interface.execute(plan);
} else {
    RCLCPP_ERROR(logger, "Pick failed!");
  rclcpp::shutdown();
  return 0;
}

std::vector<std::string> object_ids_to_remove = {"green_cube_3_addedd", "table_collision_object"};

planning_scene_interface.removeCollisionObjects(object_ids_to_remove);


geometry_msgs::msg::Pose grip_target_pose = toPoseMsg(end_effector_pose);

// Ustaw cel pozycji
move_group_interface.setPoseTarget(grip_target_pose);

// Stwórz plan do tej pozycji
auto const [success_2, plan_2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg_2;
    auto const ok_2 = static_cast<bool>(move_group_interface.plan(msg_2));
    return std::make_pair(ok_2, msg_2);
}();

// Wykonaj plan
if (success_2) {
    move_group_interface.execute(plan_2);
} else {
    RCLCPP_ERROR(logger, "Pick failed!");
  rclcpp::shutdown();
  return 0;
}


move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.03, 0.03}));
  moveit::planning_interface::MoveGroupInterface::Plan plan_grip_2;
  auto const success_grip_2 = static_cast<bool>(move_group_interface_gripper.plan(plan_grip_2));

  move_group_interface.setMaxVelocityScalingFactor(1.0); 
  move_group_interface.setMaxAccelerationScalingFactor(1.0);

  move_group_interface_gripper.setMaxVelocityScalingFactor(1.0); 
  move_group_interface_gripper.setMaxAccelerationScalingFactor(1.0);



  if (success_grip_2) {
  move_group_interface.execute(plan_grip_2);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}


// Ustaw cel pozycji
move_group_interface.setPoseTarget(pre_target_pose);

// Stwórz plan do tej pozycji
auto const [success_3, plan_3] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg_3;
    auto const ok_3 = static_cast<bool>(move_group_interface.plan(msg_3));
    return std::make_pair(ok_3, msg_3);
}();

// Wykonaj plan
if (success_3) {
    move_group_interface.execute(plan_3);
} else {
    RCLCPP_ERROR(logger, "Pick failed!");
  rclcpp::shutdown();
  return 0;
}



geometry_msgs::msg::Pose grip_target_place = grip_target_pose;
grip_target_place.position.y -= 0.3;
geometry_msgs::msg::Pose pre_target_pose_place = pre_target_pose;
pre_target_pose_place.position.y -= 0.3;



move_group_interface.setPoseTarget(pre_target_pose_place);

// Stwórz plan do tej pozycji
auto const [success_4, plan_4] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg_4;
    auto const ok_4 = static_cast<bool>(move_group_interface.plan(msg_4));
    return std::make_pair(ok_4, msg_4);
}();

// Wykonaj plan
if (success_4) {
    move_group_interface.execute(plan_4);
} else {
    RCLCPP_ERROR(logger, "Planning failed!");
}

move_group_interface.setPoseTarget(grip_target_place);

// Stwórz plan do tej pozycji
auto const [success_5, plan_5] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg_5;
    auto const ok_5 = static_cast<bool>(move_group_interface.plan(msg_5));
    return std::make_pair(ok_5, msg_5);
}();

// Wykonaj plan
if (success_5) {
    move_group_interface.execute(plan_5);
} else {
    RCLCPP_ERROR(logger, "Place failed!");
  rclcpp::shutdown();
  return 0;
}

move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.05, 0.05}));
  moveit::planning_interface::MoveGroupInterface::Plan plan_grip_3;
  auto const success_grip_3 = static_cast<bool>(move_group_interface_gripper.plan(plan_grip_3));


  if (success_grip_3) {
  move_group_interface.execute(plan_grip_3);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}




  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}







  