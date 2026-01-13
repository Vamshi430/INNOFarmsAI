#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

bool planAndExecute(MoveGroupInterface& move_group, rclcpp::Logger logger)
{
  MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    return move_group.execute(plan) ==
           moveit::core::MoveItErrorCode::SUCCESS;
  }
  RCLCPP_WARN(logger, "Planning failed");
  return false;
}

void waitForCurrentState(MoveGroupInterface& move_group)
{
  while (!move_group.getCurrentState(1.0))
  {
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place_node");

  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  MoveGroupInterface move_group(node, "ur_manipulator");
  PlanningSceneInterface planning_scene;

  move_group.setPoseReferenceFrame("base_link");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  waitForCurrentState(move_group);

  /* ---------------- FIXED TOOL ORIENTATION (DOWNWARD) ---------------- */
  geometry_msgs::msg::Quaternion fixed_orientation;
  fixed_orientation.x = 1.0;
  fixed_orientation.y = 0.0;
  fixed_orientation.z = 0.0;
  fixed_orientation.w = 0.0;

  /* ---------------- TABLE ---------------- */
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = table_shape.BOX;
  table_shape.dimensions = {0.6, 1.0, 0.35};

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.5;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.175;
  table_pose.orientation.w = 1.0;

  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  /* ---------------- CUBE ---------------- */
  moveit_msgs::msg::CollisionObject cube;
  cube.id = "cube";
  cube.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive cube_shape;
  cube_shape.type = cube_shape.BOX;
  cube_shape.dimensions = {0.05, 0.05, 0.05};

  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = 0.45;
  cube_pose.position.y = 0.0;
  cube_pose.position.z = 0.375;
  cube_pose.orientation.w = 1.0;

  cube.primitives.push_back(cube_shape);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;

  planning_scene.applyCollisionObjects({table, cube});
  rclcpp::sleep_for(std::chrono::seconds(2));

  /* ---------------- HOME ---------------- */
  move_group.setNamedTarget("home");
  planAndExecute(move_group, node->get_logger());
  waitForCurrentState(move_group);

  /* ---------------- PRE-GRASP ---------------- */
  geometry_msgs::msg::Pose pre_grasp;
  pre_grasp.position.x = 0.45;
  pre_grasp.position.y = 0.0;
  pre_grasp.position.z = 0.55;
  pre_grasp.orientation = fixed_orientation;

  move_group.setPoseTarget(pre_grasp);
  planAndExecute(move_group, node->get_logger());
  waitForCurrentState(move_group);

  /* ---------------- CARTESIAN DESCEND (ROBUST) ---------------- */
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pre_grasp);

  geometry_msgs::msg::Pose grasp_pose = pre_grasp;
  grasp_pose.position.z -= 0.12;
  waypoints.push_back(grasp_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;

  // First attempt
  fraction = move_group.computeCartesianPath(
      waypoints,
      0.01,   // larger step = more stable
      0.0,    // disable jump detection
      trajectory,
      true);

  if (fraction < 0.9)
  {
    RCLCPP_WARN(node->get_logger(),
                "Cartesian descend low fraction (%.2f). Retrying with smaller step...",
                fraction);

    // Retry with smaller step
    fraction = move_group.computeCartesianPath(
        waypoints,
        0.005,
        0.0,
        trajectory,
        true);
  }

  if (fraction >= 0.9)
  {
    move_group.execute(trajectory);
    waitForCurrentState(move_group);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(),
                "Cartesian failed (%.2f). Falling back to pose planning.",
                fraction);

    move_group.setPoseTarget(grasp_pose);
    planAndExecute(move_group, node->get_logger());
    waitForCurrentState(move_group);
  }


  /* ---------------- ATTACH ---------------- */
  moveit_msgs::msg::AttachedCollisionObject attached_cube;
  attached_cube.link_name = move_group.getEndEffectorLink(); // tool0
  attached_cube.object = cube;
  attached_cube.touch_links = move_group.getLinkNames();

  planning_scene.applyAttachedCollisionObject(attached_cube);
  rclcpp::sleep_for(std::chrono::seconds(1));

  /* ---------------- LIFT ---------------- */
  geometry_msgs::msg::Pose lift_pose = grasp_pose;
  lift_pose.position.z += 0.15;
  lift_pose.orientation = fixed_orientation;

  move_group.setPoseTarget(lift_pose);
  planAndExecute(move_group, node->get_logger());
  waitForCurrentState(move_group);

  /* ---------------- PLACE ---------------- */
  geometry_msgs::msg::Pose place_pose = lift_pose;
  place_pose.position.y += 0.25;

  move_group.setPoseTarget(place_pose);
  planAndExecute(move_group, node->get_logger());
  waitForCurrentState(move_group);

  /* ---------------- DETACH ---------------- */
  move_group.detachObject("cube");
  rclcpp::sleep_for(std::chrono::seconds(1));

  /* ---------------- HOME ---------------- */
  move_group.setNamedTarget("home");
  planAndExecute(move_group, node->get_logger());

  rclcpp::shutdown();
  return 0;
}
