// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>
// #include <gazebo_msgs/srv/get_entity_state.hpp>

// // #include <gazebo_msgs/srv/GetEntityState.hpp>
// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "one_grasp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("one_grasp");

//   // We spin up a SingleThreadedExecutor for the current state monitor to get
//   // information about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);l_tools.h>
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "arm_torso");

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools =
//       moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//                                               move_group_interface.getRobotModel() };
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Create a closure for updating the text in rviz
//   auto const draw_title = [&moveit_visual_tools](auto text) {
//     auto const text_pose = [] {
//       auto msg = Eigen::Isometry3d::Identity();
//       msg.translation().z() = 1.0;  // Place text 1m above the base link
//       return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   };
//   auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
//   auto const draw_trajectory_tool_path =
//       [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm_torso")](
//           auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  
//   // Service get_entity_state

//   using namespace std::literals::chrono_literals;

//   rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client =
//     node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "one_grasp", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("one_grasp");

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

  Eigen::Isometry3d T_BE = T_BF * T_FE;
  Eigen::Isometry3d T_CE  = T_BC.inverse() * T_BE;

  Eigen::Isometry3d end_effector_pose = cube_pose * T_CE;

  moveit_visual_tools.publishAxis(end_effector_pose, 0.5, 0.01, "end_effector");  // Adjust size as needed
  moveit_visual_tools.trigger();



  


  // geometry_msgs::Pose pose;

  //   // Ustaw pozycję
  //   pose.position.x = 0.0;
  //   pose.position.y = 0.0;
  //   pose.position.z = 0.046;

  //   // Ustaw orientację jako kwaternion
  //   // W ROS kwaternion jest w formacie (x, y, z, w)
  //   pose.orientation.x = 0.5;
  //   pose.orientation.y = 0.5;
  //   pose.orientation.z = 0.5;
  //   pose.orientation.w = -0.5;

 
  // Isometry3d p = Translation3d(0.62, 0.10, 0.33) *
  // Quaterniond(0.0, 0.0, 0.0, 1.0).normalized();

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}







  // // Set a target Pose
  // auto const target_pose = [] {
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.4;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();
  // move_group_interface.setPoseTarget(target_pose);

  // // Create a plan to that target pose
  // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  // draw_title("Planning");
  // moveit_visual_tools.trigger();
  // auto const [success, plan] = [&move_group_interface] {
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();

  // // Execute the plan
  // if (success)
  // {
  //   draw_trajectory_tool_path(plan.trajectory);
  //   moveit_visual_tools.trigger();
  //   prompt("Press 'next' in the RvizVisualToolsGui window to execute");
  //   draw_title("Executing");
  //   moveit_visual_tools.trigger();
  //   move_group_interface.execute(plan);
  // }
  // else
  // {
  //   draw_title("Planning Failed!");
  //   moveit_visual_tools.trigger();
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  // auto planning_frame = move_group_interface.getPlanningFrame();
  // auto end_effector_link = move_group_interface.getEndEffectorLink();

  // RCLCPP_INFO(logger, "układ bazowy (B): %s", planning_frame.c_str());
  // RCLCPP_INFO(logger, "układ końcówki (E): %s", end_effector_link.c_str());







  // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }