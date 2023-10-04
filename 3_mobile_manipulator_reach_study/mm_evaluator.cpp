#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <reach/interfaces/evaluator.h>
#include <reach/plugin_utils.h>
#include <reach/utils.h>
#include <reach_ros/utils.h>

class MM_Evaluator : public reach::Evaluator
{
public:
  MM_Evaluator(moveit::core::RobotModelConstPtr model,
               const std::string& planning_group,
               std::string mobile_base_frame)
    : model_(model)
    , jmg_(model_->getJointModelGroup(planning_group))
    , mobile_base_frame_(std::move(mobile_base_frame))
  {
    if(!jmg_)
    {
      std::stringstream ss;
      ss << "MoveIt robot model does not have planning group '" << planning_group << "'";
      throw std::runtime_error(ss.str());
    }

    if(!model_->hasLinkModel(mobile_base_frame_))
    {
      std::stringstream ss;
      ss << "MoveIt robot model does not have link named '" << mobile_base_frame_ << "'";
      throw std::runtime_error(ss.str());
    }
  }

  double calculateScore(const std::map<std::string, double>& pose) const override
  {
    // Create a robot state object to look up the transform to the mobile base frame
    moveit::core::RobotState state(model_);

    // Set the positions of the actuatable joints in the robot state
    std::vector<double> pose_subset = reach::extractSubset(pose, jmg_->getActiveJointModelNames());
    Eigen::Map<Eigen::VectorXd> pose_subset_map(pose_subset.data(), pose_subset.size());
    state.setJointGroupPositions(jmg_, pose_subset_map);

    // Look up the transform from the root frame to the mobile base frame
    bool found;
    Eigen::Isometry3d root_to_mobile_base = state.getFrameTransform(mobile_base_frame_, &found);
    if(!found)
    {
      std::stringstream ss;
      ss << "Failed to find transform to link '" << mobile_base_frame_ << "' in MoveIt robot state";
      throw std::runtime_error(ss.str());
    }

    // Compute a score that the reach study can maximize
    return 1.0 / std::max(root_to_mobile_base.translation().norm(), 1.0e-6);
  }

protected:
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  const std::string mobile_base_frame_;
};

struct MM_EvaluatorFactory : public reach::EvaluatorFactory
{
  reach::Evaluator::ConstPtr create(const YAML::Node &config) const override
  {
    auto planning_group = reach::get<std::string>(config, "planning_group");

    moveit::core::RobotModelConstPtr model =
        moveit::planning_interface::getSharedRobotModel(reach_ros::utils::getNodeInstance(), "robot_description");
    if (!model)
      throw std::runtime_error("Failed to initialize robot model pointer");

    auto mobile_base_frame = reach::get<std::string>(config, "mobile_base_frame");

    return std::make_shared<MM_Evaluator>(model, planning_group, mobile_base_frame);
  }
};

EXPORT_EVALUATOR_PLUGIN(MM_EvaluatorFactory, MM_Evaluator)
