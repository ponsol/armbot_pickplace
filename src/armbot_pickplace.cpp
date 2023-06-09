
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

  void getpos(geometry_msgs::msg::PoseStamped &p, std::vector<float> &pos,  std::vector<float> &ang );
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
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.8, 0.06 };

  geometry_msgs::msg::Pose pose;

  pose.position.y = 2.0;
  pose.position.z = 1.0;
  pose.position.x = -2.0;

#ifdef PANDA 
  object.primitives[0].dimensions = { 0.1, 0.02 };
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.position.z = 0.0;
#endif


  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
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

  if (!task_.plan(5))
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

void MTCTaskNode::getpos(geometry_msgs::msg::PoseStamped &p, std::vector<float> &pos,  std::vector<float> &ang ) {
  p.pose.position.x = pos[0];
  p.pose.position.y = pos[1];
  p.pose.position.z = pos[2];
  p.pose.orientation.x = ang[0];
  p.pose.orientation.y = ang[1];
  p.pose.orientation.z = ang[2];
  p.pose.orientation.w = ang[3];
return ;
}


mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& hand_group_name = "hand";



#ifdef PANDA
  const auto& hand_frame = "panda_hand";
  const auto& arm_group_name = "panda_arm";
#else
  const auto& arm_group_name = "arm";
  const auto& hand_frame = "eef_link";
#endif



  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));



  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
     mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner } });

  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
  auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  //auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", sampling_planner);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.0, 0.1);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  //vec.header.frame_id = hand_frame ;
  vec.header.frame_id = "world";
  //vec.vector.y = 0.7071;
  //vec.vector.z = 0.7071;
  vec.vector.z = 1.0;
#ifdef PANDA
  vec.vector.x = 0.0; vec.vector.y = 0.0; vec.vector.z = 1.0;
#endif
  stage->setDirection(vec);

  grasp->insert(std::move(stage));
  }



  {
   // grasp poses
   /*
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject("object");
  stage->setAngleDelta(M_PI / 8);
  stage->setMonitoredStage(current_state_ptr);
  */

  auto stage = std::make_unique<mtc::stages::GeneratePose>("gen grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setMonitoredStage(current_state_ptr);

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "world";

  std::vector<float> pos{-1.6014, 1.51181, 0.917797}; std::vector<float> ang{0.0374589, 0.0239122,  0.366514, 0.92935};

  getpos(p, pos, ang);

  stage->setPose(p);


  // translate/rotate grasp poses from eef
  Eigen::Isometry3d grasp_frame_transform;

  Eigen::Vector3d tvec  = Eigen::Vector3d(0.0, 0.0, 0.0);
  float trot[] = {0.0, 0.0, 0.0};

#ifdef PANDA
   tvec  = Eigen::Vector3d(0.0,0.0, 0.1);
   trot[0] = M_PI/2; trot[1] = M_PI/2; trot[2] = M_PI/2;
#endif

  Eigen::Quaterniond q = Eigen::AngleAxisd(trot[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(trot[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(trot[2], Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation() = tvec;

  //Eigen::Affine3d egrasp_frame_transform = Eigen::Affine3d::Identity();
  //egrasp_frame_transform.translation() = tvec ;


  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.00);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group"  });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
  }



  {
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
  grasp->insert(std::move(stage));
  }


  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("close");
  grasp->insert(std::move(stage));
  }


  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject("object", hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
  }


  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 2.0);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
  }

   task.add(std::move(grasp));
  }


  /*
  */

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
