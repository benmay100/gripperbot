#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char** argv)
{
  // 1. Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // 2. Create the node
  auto node = std::make_shared<rclcpp::Node>(
    "move_gripperbot_node1",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 3. Spin the node in a background thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 4. Create the MoveGroupInterface
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  // Optional: Set max velocity/acceleration scaling factors (0.0 - 1.0)
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  // 5. Setup MoveIt Visual Tools
  // This allows us to interact with RvizVisualToolsGui
  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node,
      "base_link",  // Reference frame
      "move_gripperbot_node1",  // Namespace for RVizVisualToolsGui
      move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  
  // Create a closure for updating text in RViz
  auto const draw_title = [&moveit_visual_tools](auto& text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rvt::WHITE, rvt::LARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto& text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)](auto const trajectory) {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };
  
  draw_title("GripperBot_Demo");
  moveit_visual_tools.trigger();

  // =================================================================================
  // EXAMPLE 1: Move to a named target
  // =================================================================================
  RCLCPP_INFO(node->get_logger(), "Example 1: Planning to named target 'horizontal_fold'...");
  draw_title("Example_1:_Named_Pose");
  prompt("Press 'Next' in the RvizVisualToolsGui window to start the demo");
  moveit_visual_tools.trigger();
  
  bool success = move_group_interface.setNamedTarget("horizontal_fold");
  if (success) {
      moveit::planning_interface::MoveGroupInterface::Plan plan1;
      bool plan_success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

      if (plan_success)
      {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        draw_trajectory_tool_path(plan1.trajectory);
        moveit_visual_tools.trigger();
        prompt("Press 'Next' to execute");
        draw_title("Executing...");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan1);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
      }
  } else {
      RCLCPP_ERROR(node->get_logger(), "Named target not found!");
  }

  // PROMPT FOR NEXT STEP
  RCLCPP_INFO(node->get_logger(), "Press 'Next' in the RvizVisualToolsGui window to continue to Example 2");
  prompt("Press 'Next' in the RvizVisualToolsGui window to continue to Example 2");

  // =================================================================================
  // EXAMPLE 2: Move to a Pose based on the current pose
  // =================================================================================
  RCLCPP_INFO(node->get_logger(), "Example 2: Planning to a specific pose (relative)...");
  draw_title("Example_2:_Relative_Pose");
  moveit_visual_tools.trigger();

  // Get the current pose to use as a reference
  geometry_msgs::msg::Pose target_pose_rel = move_group_interface.getCurrentPose().pose;

  // Modify the pose (e.g., move up by 20cm)
  target_pose_rel.position.z += 0.2;
  
  move_group_interface.setPoseTarget(target_pose_rel);

  moveit::planning_interface::MoveGroupInterface::Plan pose_plan_rel;
  bool pose_plan_rel_success = (move_group_interface.plan(pose_plan_rel) == moveit::core::MoveItErrorCode::SUCCESS);

  if (pose_plan_rel_success){
    RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
    draw_trajectory_tool_path(pose_plan_rel.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' to execute");
    draw_title("Executing...");
    moveit_visual_tools.trigger();
    move_group_interface.execute(pose_plan_rel);
    prompt("Press 'Next' to execute");
    move_group_interface.execute(pose_plan_rel);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Pose Planning failed!");
  }

  // PROMPT FOR NEXT STEP
  RCLCPP_INFO(node->get_logger(), "Press 'Next' in the RvizVisualToolsGui window to continue to Example 3");
  prompt("Press 'Next' in the RvizVisualToolsGui window to continue to Example 3");


  // =================================================================================
  // EXAMPLE 3: Move to a Pose based on provided coordinates
  // =================================================================================

  RCLCPP_INFO(node->get_logger(), "Example 3: Planning to a specific pose (absolute)...");
  draw_title("Example_3:_Absolute_Pose");
  moveit_visual_tools.trigger();

  // Get the current pose to use as a reference
  // You can open Moveit/RVIZ, and move the robot to a desired pose, then run 'ros2 run tf2_ros tf2_echo base_link wrist3_tool_link' in a separate terminal to get the desired coordinates
  geometry_msgs::msg::Pose target_pose_abs = []{
      geometry_msgs::msg::Pose pose;
      pose.position.x = -0.468;
      pose.position.y = 0.145;
      pose.position.z = 0.578;
      // Orientation as a quaternion (w, x, y, z)
      pose.orientation.w = 0.177;
      pose.orientation.x = 0.471;
      pose.orientation.y = -0.386;
      pose.orientation.z = 0.773;
      return pose;
  }();
  
  // Set the target pose
  move_group_interface.setPoseTarget(target_pose_abs);

  // Plan and Execute
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan_abs;
  bool pose_plan_abs_success = (move_group_interface.plan(pose_plan_abs) == moveit::core::MoveItErrorCode::SUCCESS);

  if (pose_plan_abs_success){
    RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
    draw_trajectory_tool_path(pose_plan_abs.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' to execute");
    draw_title("Executing...");
    moveit_visual_tools.trigger();
    move_group_interface.execute(pose_plan_abs);
    prompt("Press 'Next' to execute");
    move_group_interface.execute(pose_plan_abs);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Pose Planning failed!");
  }

  RCLCPP_INFO(node->get_logger(), "Demo complete!");
  draw_title("Demo_Complete");
  moveit_visual_tools.trigger();

  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
