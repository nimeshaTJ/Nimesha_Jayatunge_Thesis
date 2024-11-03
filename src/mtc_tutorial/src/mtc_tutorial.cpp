#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// for allowing the base link inertia to collide with the surface.
#include <moveit_msgs/msg/allowed_collision_matrix.h>
#include <moveit_msgs/msg/allowed_collision_entry.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{

  moveit_msgs::msg::CollisionObject collision_object_target1;
  collision_object_target1.header.frame_id = "world";
  collision_object_target1.id = "target1";
  shape_msgs::msg::SolidPrimitive primitive_target1;
  primitive_target1.type = primitive_target1.BOX;
  primitive_target1.dimensions.resize(3);
  primitive_target1.dimensions[primitive_target1.BOX_X] = 0.06; // x
  primitive_target1.dimensions[primitive_target1.BOX_Y] = 0.06; // y
  primitive_target1.dimensions[primitive_target1.BOX_Z] = 0.06; // z
  geometry_msgs::msg::Pose target1_pose;
  target1_pose.orientation.w = 1.0;                  
  target1_pose.position.x = -0.6;
  target1_pose.position.y = -0.11;
  target1_pose.position.z = 0.25;
  collision_object_target1.primitives.push_back(primitive_target1);
  collision_object_target1.primitive_poses.push_back(target1_pose);
  collision_object_target1.operation = collision_object_target1.ADD;

  moveit_msgs::msg::CollisionObject collision_object_surface;
  collision_object_surface.header.frame_id = "world";
  collision_object_surface.id = "surface";
  shape_msgs::msg::SolidPrimitive primitive_surface;
  primitive_surface.type = primitive_surface.BOX;
  primitive_surface.dimensions.resize(3);
  primitive_surface.dimensions[primitive_surface.BOX_X] = 3;
  primitive_surface.dimensions[primitive_surface.BOX_Y] = 3;
  primitive_surface.dimensions[primitive_surface.BOX_Z] = 0.1;
  geometry_msgs::msg::Pose surface_pose;
  surface_pose.orientation.w = 1.0;
  surface_pose.position.x = 0.16;
  surface_pose.position.y = 0.25;
  surface_pose.position.z = -0.05;
  collision_object_surface.primitives.push_back(primitive_surface);
  collision_object_surface.primitive_poses.push_back(surface_pose);
  collision_object_surface.operation = collision_object_surface.ADD;

  RCLCPP_INFO(LOGGER, "collision objects created");

  // Define colors

  moveit_msgs::msg::ObjectColor target1_color;
  target1_color.id = collision_object_target1.id;
  target1_color.color.r = 1; // Red
  target1_color.color.g = 0.1; // Green
  target1_color.color.b = 0.1; // Blue
  target1_color.color.a = 1.0; // Alpha

  moveit_msgs::msg::ObjectColor surface_color;
  surface_color.id = collision_object_surface.id;
  surface_color.color.r = 0.5; // Gray
  surface_color.color.g = 0.5;
  surface_color.color.b = 0.5;
  surface_color.color.a = 1.0; // Alpha

  // Create a PlanningScene message and add the collision object and its color
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects.push_back(collision_object_target1);
  planning_scene_msg.world.collision_objects.push_back(collision_object_surface);
  planning_scene_msg.object_colors.push_back(target1_color);
  planning_scene_msg.object_colors.push_back(surface_color);

  // Apply the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  // Create a RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(robot_model);

  // Modify the ACM to allow collision between base_link_inertia and surface
  collision_detection::AllowedCollisionMatrix& acm = planning_scene.getAllowedCollisionMatrixNonConst();
  acm.setEntry("base_link_inertia", "surface", true);
//   acm.setEntry("wrist_3_link", "marker1", true);
//   acm.setEntry("wrist_3_link", "marker1", true);
//   acm.setEntry("wrist_3_link", "marker1", true);
//   acm.setEntry("wrist_3_link", "marker1", true);

  // Update the planning scene with the modified ACM
  moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
  acm.getMessage(acm_msg);
  planning_scene_msg.allowed_collision_matrix = acm_msg;
  planning_scene_interface.applyPlanningScene(planning_scene_msg);

  RCLCPP_INFO(LOGGER, "allow collision between base_link_inertia and surface.");
}


void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(20))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "flange";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  // Open hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Define the target pose
  auto const target_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = -M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = -0.6;
	  msg.pose.position.y = -0.35;
	  msg.pose.position.z = 0.27;
	  return msg;
	}();
  
  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));

  // Define the slide pose
  auto const slide_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll = -M_PI/2.0;  // Rotation around the x-axis
	  double pitch = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll, pitch, yaw);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = -0.6;
	  msg.pose.position.y = -0.2;
	  msg.pose.position.z = 0.27;
	  return msg;
	}();
  
  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task.add(std::move(move_to_slide));

  // Allow collision (hand, target1) temporarily
  auto allow_collision_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand, target1)");
  allow_collision_target1->allowCollisions("target1",
                                   task.getRobotModel()
                                       ->getJointModelGroup(hand_group_name)
                                       ->getLinkModelNamesWithCollisionGeometry(),
                                   true);
  task.add(std::move(allow_collision_target1));

  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task.add(std::move(stage_close_hand));

  // Define the slide2 pose
  auto const slide2_pose = []{
	  geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
	  
    // Define roll, pitch, and yaw angles in radians
	  double roll2 = -M_PI/2.0;  // Rotation around the x-axis
	  double pitch2 = 0.0;  // Rotation around the y-axis (90 degrees)
	  double yaw2 = 0.0;  // Rotation around the z-axis
	  // Convert roll, pitch, yaw to quaternion
	  tf2::Quaternion q;
	  q.setRPY(roll2, pitch2, yaw2);

	  msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
	  msg.pose.position.x = -0.6;
	  msg.pose.position.y = -0.4;
	  msg.pose.position.z = 0.27;
	  return msg;
	}();
  
  // Move to slide2 pose
  auto move_to_slide2 = std::make_unique<mtc::stages::MoveTo>("move to slide2", cartesian_planner);
  move_to_slide2->setGroup(arm_group_name);
  move_to_slide2->setGoal(slide2_pose);
  task.add(std::move(move_to_slide2));
  
//   // Open hand
//   auto stage2_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
//   stage2_open_hand->setGroup(hand_group_name);
//   stage2_open_hand->setGoal("open");
//   task.add(std::move(stage2_open_hand));

//   // Detach target1 from the gripper
//   auto detach_target1 = std::make_unique<mtc::stages::ModifyPlanningScene>("detach target1");
//   detach_target1->detachObject("target1", hand_frame);
//   task.add(std::move(detach_target1));


  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
  /* // Move wrist joint back to its initial position
  std::map<std::string, double> initial_joint_positions;
  initial_joint_positions["wrist_3_joint"] = 3.14; // Adjust this value based on the initial position of wrist_3_joint
  auto move_to_initial = std::make_unique<mtc::stages::MoveTo>("reset wrist joint", sampling_planner);
  move_to_initial->setGroup(arm_group_name);
  move_to_initial->setGoal(initial_joint_positions);
  task.add(std::move(move_to_initial));
 */
