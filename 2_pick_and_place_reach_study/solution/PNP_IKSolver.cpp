#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <reach/plugin_utils.h>
#include <reach_ros/ik/moveit_ik_solver.h>
#include <reach_ros/utils.h>
#include <tf2_eigen/tf2_eigen.h>

/**
 * @brief Creates a collision representation of a bin for MoveIt, assuming the input frame represents the bottom center of the bin
 */
moveit_msgs::msg::CollisionObject createBinCollisionObject(const std::string& frame,
                                                           const Eigen::Vector3d& dims,
                                                           const double wall_thickness)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  obj.header.frame_id = frame;

  const auto bin_center = Eigen::Isometry3d::Identity(); // * Eigen::Translation3d(dims.x() / 2.0, dims.y() / 2.0, 0.0);

  // Add a box for the bottom of the bin
  {
    // Create the primitive
    shape_msgs::msg::SolidPrimitive p;
    p.type = shape_msgs::msg::SolidPrimitive::BOX;

    p.dimensions.resize(3);
    p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dims.x();
    p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dims.y();
    p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = wall_thickness;

    // Add to the collision object message
    obj.primitives.push_back(p);
    obj.primitive_poses.push_back(tf2::toMsg(bin_center * Eigen::Translation3d(0.0, 0.0, -wall_thickness / 2.0)));
  }

  // Create boxes for the walls for each side of the bin
  {
    // Create the walls normal to the x-axis
    {
      auto offset = Eigen::Translation3d((dims.x() + wall_thickness) / 2.0, 0.0, dims.z() / 2.0);

      shape_msgs::msg::SolidPrimitive p;
      p.type = shape_msgs::msg::SolidPrimitive::BOX;

      p.dimensions.resize(3);
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = wall_thickness;
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dims.y();
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dims.z();

      // Add the first wall
      obj.primitives.push_back(p);
      obj.primitive_poses.push_back(tf2::toMsg(bin_center * offset));

      // Add the second wall with a rotation of 180 degrees about the z-axis
      obj.primitives.push_back(p);
      obj.primitive_poses.push_back(tf2::toMsg(bin_center * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * offset));
    }

    // Create the walls normal to the y-axis
    {
      auto offset = Eigen::Translation3d(0.0, (dims.y() + wall_thickness) / 2.0, dims.z() / 2.0);

      shape_msgs::msg::SolidPrimitive p;
      p.type = shape_msgs::msg::SolidPrimitive::BOX;

      p.dimensions.resize(3);
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dims.x();
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = wall_thickness;
      p.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dims.z();

      // Add the first wall
      obj.primitives.push_back(p);
      obj.primitive_poses.push_back(tf2::toMsg(bin_center * offset));

      // Add the second wall with a rotation of 180 degrees around the z-axis
      obj.primitives.push_back(p);
      obj.primitive_poses.push_back(tf2::toMsg(bin_center * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * offset));
    }
  }

  return obj;
}

/***********************************************/
/** Define the pick and place IK solver class **/
/***********************************************/

class PNP_IKSolver : public reach_ros::ik::MoveItIKSolver
{
public:
  using reach_ros::ik::MoveItIKSolver::MoveItIKSolver;

  void addBinCollisionObjects(const std::string& frame,
                              const Eigen::Vector3d& dims,
                              const double wall_thickness)
  {
    moveit_msgs::msg::CollisionObject obj = createBinCollisionObject(frame, dims, wall_thickness);
    if (!scene_->processCollisionObjectMsg(obj))
      throw std::runtime_error("Failed to add collision mesh to planning scene");

    moveit_msgs::msg::PlanningScene scene_msg;
    scene_->getPlanningSceneMsg(scene_msg);
    scene_pub_->publish(scene_msg);
  }
};

/**************************************************************/
/** Define the plugin that creates pick and place IK solvers **/
/**************************************************************/

struct PNP_IKSolverFactory : public reach::IKSolverFactory
{
  reach::IKSolver::ConstPtr create(const YAML::Node &config) const override
  {
    auto planning_group = reach::get<std::string>(config, "planning_group");
    auto dist_threshold = reach::get<double>(config, "distance_threshold");

    moveit::core::RobotModelConstPtr model =
        moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
    if (!model)
      throw std::runtime_error("Failed to initialize robot model pointer");

    auto ik_solver = std::make_shared<PNP_IKSolver>(model, planning_group, dist_threshold);

    // Optionally add touch links
    const std::string touch_links_key = "touch_links";
    if (config[touch_links_key])
    {
      auto touch_links = reach::get<std::vector<std::string>>(config, touch_links_key);
      ik_solver->setTouchLinks(touch_links);
    }

    // Add the collision representation of the bin
    {
      auto frame = reach::get<std::string>(config, "bin_frame");

      auto dims_vec = reach::get<std::vector<double>>(config, "bin_dims");
      if (dims_vec.size() != 3)
        throw std::runtime_error("Bin dimensions must be 3 values (x, y, z)");
      Eigen::Map<Eigen::Vector3d> dims(dims_vec.data());

      // Optionally load a bin dimension scale factor
      if (config["bin_dims_collision_scale"])
      {
        auto dims_scale_vec = reach::get<std::vector<double>>(config, "bin_dims_collision_scale");
        if(dims_scale_vec.size() != 3)
          throw std::runtime_error("Bin dimension scale factor must be 3 values (x, y, z)");
        Eigen::Map<Eigen::Array3d> dims_scale(dims_scale_vec.data());
        dims.array() *= dims_scale;
      }

      auto wall_thickness = reach::get<double>(config, "bin_wall_thickness");

      ik_solver->addBinCollisionObjects(frame, dims, wall_thickness);
    }

    return ik_solver;
  }
};

/************************************************/
/** Export the pick and place IK solver plugin **/
/************************************************/
EXPORT_IK_SOLVER_PLUGIN(PNP_IKSolverFactory, PNP_IKSolver)
