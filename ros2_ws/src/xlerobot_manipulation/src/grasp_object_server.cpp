#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <xlerobot_interfaces/action/execute_learned_policy.hpp>
#include <xlerobot_interfaces/action/grasp_object.hpp>
#include <xlerobot_interfaces/action/prepare_grasp.hpp>
#include <xlerobot_interfaces/action/verify_grasp.hpp>
#include <xlerobot_interfaces/msg/capability_error.hpp>
#include <xlerobot_interfaces/msg/policy_start_context.hpp>
#include <xlerobot_interfaces/msg/top_grasp_plan.hpp>
#include <yaml-cpp/yaml.h>

#include "xlerobot_manipulation/grasp_plan_validator.hpp"

namespace xlerobot_manipulation
{

namespace
{

using namespace std::chrono_literals;
using CapabilityError = xlerobot_interfaces::msg::CapabilityError;
using GraspObject = xlerobot_interfaces::action::GraspObject;
using PrepareGrasp = xlerobot_interfaces::action::PrepareGrasp;
using LearnedPolicy = xlerobot_interfaces::action::ExecuteLearnedPolicy;
using VerifyGrasp = xlerobot_interfaces::action::VerifyGrasp;
using PolicyStartContext = xlerobot_interfaces::msg::PolicyStartContext;
using TopGraspPlan = xlerobot_interfaces::msg::TopGraspPlan;
using FollowTrajectory = control_msgs::action::FollowJointTrajectory;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;

struct GraspFailure : public std::runtime_error
{
  GraspFailure(uint16_t code_value, const std::string & message_value, bool canceled_value = false)
  : std::runtime_error(message_value), code(code_value), canceled(canceled_value) {}

  uint16_t code;
  bool canceled;
};

struct GraspVerificationFailure : public GraspFailure
{
  explicit GraspVerificationFailure(const std::string & message_value)
  : GraspFailure(CapabilityError::NOT_FOUND, message_value) {}
};

constexpr int kMaximumGraspAttempts = 2;

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool finite_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return !pose.header.frame_id.empty() && finite_point(pose.pose.position) &&
         std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-3;
}

bool valid_classical_plan(
  const TopGraspPlan & plan, const std::string & backend, bool dry_run)
{
  const bool backend_valid = backend == "centroid" || backend == "gpd";
  const std::string expected_method = backend == "centroid" ?
    "sam2_rgbd_top_centroid" : "sam2_rgbd_gpd_top";
  const bool method_valid = plan.method == expected_method ||
    (dry_run && plan.method == "contract_only");
  return plan.valid && backend_valid && plan.backend == backend && method_valid &&
         !plan.observation_id.empty() && !plan.selection_reason.empty() &&
         finite_pose(plan.pregrasp) && finite_pose(plan.grasp) && finite_pose(plan.lift) &&
         plan.pregrasp.header.frame_id == plan.grasp.header.frame_id &&
         plan.lift.header.frame_id == plan.grasp.header.frame_id &&
         std::isfinite(plan.score) &&
         std::isfinite(plan.table_height_m) &&
         std::isfinite(plan.object_height_m) && plan.object_height_m > 0.0F &&
         std::isfinite(plan.grasp_width_m) && plan.grasp_width_m > 0.0F &&
         std::isfinite(plan.wrist_yaw_seed_rad) &&
         std::isfinite(plan.wrist_yaw_min_rad) &&
         std::isfinite(plan.wrist_yaw_max_rad) &&
         plan.wrist_yaw_min_rad < plan.wrist_yaw_max_rad &&
         plan.wrist_yaw_seed_rad >= plan.wrist_yaw_min_rad &&
         plan.wrist_yaw_seed_rad <= plan.wrist_yaw_max_rad &&
         plan.pregrasp.pose.position.z > plan.grasp.pose.position.z &&
         plan.lift.pose.position.z > plan.grasp.pose.position.z;
}

}  // namespace

class GraspObjectServer : public rclcpp::Node
{
public:
  using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<GraspObject>;
  using GoalHandlePrepare = rclcpp_action::ServerGoalHandle<PrepareGrasp>;
  using VerifyGoalHandle = rclcpp_action::ClientGoalHandle<VerifyGrasp>;

  struct PendingVerification
  {
    VerifyGoalHandle::SharedPtr handle;
    std::shared_future<VerifyGoalHandle::WrappedResult> result;
  };

  GraspObjectServer()
  : Node("grasp_object_server")
  {
    execution_enabled_ = declare_parameter<bool>("execution_enabled", false);
    action_name_ = declare_parameter<std::string>("action_name", "grasp_object");
    prepare_action_name_ = declare_parameter<std::string>(
      "prepare_action_name", "prepare_grasp");
    policy_id_ = declare_parameter<std::string>(
      "policy_id", "act_xlerobot_box_wrist_only");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    arm_group_ = declare_parameter<std::string>("arm_group", "right_arm");
    ee_link_ = declare_parameter<std::string>("ee_link", "right_arm_ee_link");
    pregrasp_offset_m_ = declare_parameter<double>("pregrasp_offset_m", 0.08);
    vision_fk_compensation_m_ = declare_parameter<std::vector<double>>(
      "vision_fk_compensation_m", {0.0, 0.0, 0.0});
    grasp_alignment_file_ = declare_parameter<std::string>("grasp_alignment_file", "");
    top_grasp_orientation_xyzw_ = declare_parameter<std::vector<double>>(
      "top_grasp_orientation_xyzw", {0.70710678, 0.0, 0.0, 0.70710678});
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 1.0);
    service_timeout_s_ = declare_parameter<double>("service_timeout_s", 5.0);
    planning_timeout_s_ = declare_parameter<double>("planning_timeout_s", 5.0);
    planning_velocity_scale_ = declare_parameter<double>("planning_velocity_scale", 1.0);
    planning_acceleration_scale_ =
      declare_parameter<double>("planning_acceleration_scale", 1.0);
    controller_timeout_s_ = declare_parameter<double>("controller_timeout_s", 12.0);
    policy_timeout_s_ = declare_parameter<double>("policy_timeout_s", 25.0);
    policy_duration_s_ = declare_parameter<double>("policy_duration_s", 20.0);
    verification_timeout_s_ = declare_parameter<double>("verification_timeout_s", 10.0);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 0.50);
    head_move_duration_s_ = declare_parameter<double>("head_move_duration_s", 1.5);
    pregrasp_gripper_duration_s_ = declare_parameter<double>(
      "pregrasp_gripper_duration_s", 2.0);
    grasp_gripper_duration_s_ = declare_parameter<double>(
      "grasp_gripper_duration_s", 2.0);
    pregrasp_move_duration_s_ = declare_parameter<double>("pregrasp_move_duration_s", 4.0);
    return_ready_duration_s_ = declare_parameter<double>("return_ready_duration_s", 4.0);
    pregrasp_settle_timeout_s_ = declare_parameter<double>("pregrasp_settle_timeout_s", 2.0);
    pregrasp_settle_hold_s_ = declare_parameter<double>("pregrasp_settle_hold_s", 0.10);
    pregrasp_arrival_tolerance_rad_ =
      declare_parameter<double>("pregrasp_arrival_tolerance_rad", 0.05);
    pregrasp_arrival_velocity_tolerance_radps_ =
      declare_parameter<double>("pregrasp_arrival_velocity_tolerance_radps", 0.05);
    head_ready_positions_ = declare_parameter<std::vector<double>>(
      "head_ready_positions", {0.0, 0.0});
    gripper_joint_ = declare_parameter<std::string>(
      "gripper_joint", "right_arm_gripper");
    pregrasp_gripper_position_ = declare_parameter<double>(
      "pregrasp_gripper_position", 1.64);
    gripper_lower_position_ = declare_parameter<double>("gripper_lower_position", 0.0);
    gripper_upper_position_ = declare_parameter<double>("gripper_upper_position", 1.65);
    gripper_closed_position_ = declare_parameter<double>("gripper_closed_position", 0.0);
    empty_gripper_tolerance_rad_ =
      declare_parameter<double>("empty_gripper_tolerance_rad", 0.05);
    head_lower_positions_ = declare_parameter<std::vector<double>>(
      "head_lower_positions", {-1.57, -0.76});
    head_upper_positions_ = declare_parameter<std::vector<double>>(
      "head_upper_positions", {1.57, 1.45});
    ready_positions_ = declare_parameter<std::vector<double>>(
      "ready_positions", {-0.11505, 1.62142, 1.62602, 0.50621, 0.00153});
    wrist_roll_seeds_ = declare_parameter<std::vector<double>>(
      "wrist_roll_seeds", {0.06981317, 0.03067962});
    include_radial_wrist_roll_seed_ =
      declare_parameter<bool>("include_radial_wrist_roll_seed", true);
    pregrasp_seed_positions_ = declare_parameter<std::vector<double>>(
      "pregrasp_seed_positions", {-0.10278, -0.53076, -0.56450, 1.20, 0.03068});
    pregrasp_wrist_flex_range_ = declare_parameter<std::vector<double>>(
      "pregrasp_wrist_flex_range", {0.65, 1.75});
    workspace_min_ = declare_parameter<std::vector<double>>(
      "workspace_min", {-0.8, -0.8, 0.0});
    workspace_max_ = declare_parameter<std::vector<double>>(
      "workspace_max", {0.8, 0.8, 1.6});
    if (!grasp_alignment_file_.empty()) {
      load_grasp_alignment(grasp_alignment_file_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "no grasp_alignment_file: ACT, centroid, and GPD pregrasp execution are unavailable");
    }
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll"});
    policy_joint_names_ = declare_parameter<std::vector<std::string>>(
      "policy_joint_names",
      {"right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
        "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"});
    const auto lower = declare_parameter<std::vector<double>>(
      "lower_positions", {-2.05, -1.40, -1.65, -1.75, -3.09});
    const auto upper = declare_parameter<std::vector<double>>(
      "upper_positions", {2.05, 1.85, 1.70, 1.75, 3.09});
    const double plan_max_duration_s =
      declare_parameter<double>("plan_max_duration_s", 10.0);
    const double plan_start_tolerance_rad =
      declare_parameter<double>("plan_start_tolerance_rad", 0.05);
    validate_parameters(lower, upper);

    GraspPlanValidationConfig validation_config;
    validation_config.max_duration_s = plan_max_duration_s;
    validation_config.start_tolerance_rad = plan_start_tolerance_rad;
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      validation_config.joints.push_back(
        {joint_names_[index], lower[index], upper[index]});
    }
    plan_validator_ = std::make_unique<GraspPlanValidator>(std::move(validation_config));

    const std::string joint_state_topic =
      declare_parameter<std::string>("joint_state_topic", "/joint_states");
    const std::string ik_service =
      declare_parameter<std::string>("ik_service", "/compute_ik");
    const std::string plan_service =
      declare_parameter<std::string>("plan_service", "/plan_kinematic_path");
    const std::string head_action = declare_parameter<std::string>(
      "head_action", "/head_controller/follow_joint_trajectory");
    const std::string arm_action = declare_parameter<std::string>(
      "arm_action", "/right_arm_controller/follow_joint_trajectory");
    const std::string gripper_action = declare_parameter<std::string>(
      "gripper_action", "/right_gripper_controller/follow_joint_trajectory");
    const std::string policy_action = declare_parameter<std::string>(
      "policy_action", "/execute_learned_policy");
    const std::string verification_action = declare_parameter<std::string>(
      "verification_action", "/verify_grasp");

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_joint_state_ = *message;
        joint_state_received_at_ = std::chrono::steady_clock::now();
      });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ik_client_ = create_client<GetPositionIK>(ik_service);
    plan_client_ = create_client<GetMotionPlan>(plan_service);
    head_client_ = rclcpp_action::create_client<FollowTrajectory>(this, head_action);
    arm_client_ = rclcpp_action::create_client<FollowTrajectory>(this, arm_action);
    gripper_client_ = rclcpp_action::create_client<FollowTrajectory>(this, gripper_action);
    policy_client_ = rclcpp_action::create_client<LearnedPolicy>(this, policy_action);
    verification_client_ = rclcpp_action::create_client<VerifyGrasp>(
      this, verification_action);
    action_server_ = rclcpp_action::create_server<GraspObject>(
      this, action_name_,
      std::bind(&GraspObjectServer::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&GraspObjectServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GraspObjectServer::handle_accepted, this, std::placeholders::_1));
    prepare_action_server_ = rclcpp_action::create_server<PrepareGrasp>(
      this, prepare_action_name_,
      std::bind(&GraspObjectServer::handle_prepare_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&GraspObjectServer::handle_prepare_cancel, this, std::placeholders::_1),
      std::bind(&GraspObjectServer::handle_prepare_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "GraspObject and PrepareGrasp ready; execution_enabled=%s backends=act,centroid,gpd",
      execution_enabled_ ? "true" : "false");
  }

  ~GraspObjectServer() override
  {
    shutting_down_ = true;
    cancel_active_child();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  static std::vector<double> yaml_vector3(
    const YAML::Node & value, const std::string & label)
  {
    if (!value.IsSequence() || value.size() != 3) {
      throw std::invalid_argument(label + " must contain three values");
    }
    std::vector<double> result;
    result.reserve(3);
    for (size_t index = 0; index < 3; ++index) {
      const double item = value[index].as<double>();
      if (!std::isfinite(item)) {
        throw std::invalid_argument(label + " must contain finite values");
      }
      result.push_back(item);
    }
    return result;
  }

  void load_grasp_alignment(const std::string & path)
  {
    try {
      const auto document = YAML::LoadFile(path);
      if (!document.IsMap() ||
        document["schema"].as<std::string>("") != "xlerobot_grasp_alignment/v1" ||
        document["frame"].as<std::string>("") != base_frame_)
      {
        throw std::invalid_argument(
                "grasp_alignment_file must be xlerobot_grasp_alignment/v1 in " + base_frame_);
      }
      vision_fk_compensation_m_ = yaml_vector3(
        document["vision_fk_compensation_m"], "vision_fk_compensation_m");
      gravity_sag_z_m_ = document["gravity_sag_z_m"].as<double>();
      if (!std::isfinite(gravity_sag_z_m_) || gravity_sag_z_m_ < 0.0) {
        throw std::invalid_argument("gravity_sag_z_m must be finite and nonnegative");
      }
      const auto head = document["head_pose_rad"];
      if (!head.IsMap() || !head["pan"] || !head["tilt"] ||
        !std::isfinite(head["pan"].as<double>()) ||
        !std::isfinite(head["tilt"].as<double>()))
      {
        throw std::invalid_argument("head_pose_rad must contain finite pan and tilt");
      }
      const auto workspace = document["workspace_m"];
      if (!workspace.IsMap()) {
        throw std::invalid_argument("workspace_m must contain min and max");
      }
      calibration_workspace_min_ = yaml_vector3(workspace["min"], "workspace_m.min");
      calibration_workspace_max_ = yaml_vector3(workspace["max"], "workspace_m.max");
      for (size_t index = 0; index < 3; ++index) {
        if (calibration_workspace_min_[index] >= calibration_workspace_max_[index]) {
          throw std::invalid_argument("workspace_m bounds must be ordered");
        }
      }
      const auto metrics = document["metrics"];
      if (!metrics.IsMap() || !metrics["sample_count"] || !metrics["plane_rmse_mm"] ||
        metrics["sample_count"].as<double>() <= 0.0 ||
        !std::isfinite(metrics["plane_rmse_mm"].as<double>()) ||
        metrics["plane_rmse_mm"].as<double>() < 0.0)
      {
        throw std::invalid_argument("grasp alignment metrics are incomplete");
      }
      grasp_alignment_ready_ = true;
    } catch (const YAML::Exception & error) {
      throw std::invalid_argument(
              "cannot load grasp_alignment_file " + path + ": " + error.what());
    }
  }

  void validate_parameters(
    const std::vector<double> & lower,
    const std::vector<double> & upper) const
  {
    const size_t count = joint_names_.size();
    const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
    if (action_name_.empty() || prepare_action_name_.empty() || policy_id_.empty() ||
      base_frame_.empty() || arm_group_.empty() ||
      ee_link_.empty() || count == 0 || lower.size() != count || upper.size() != count ||
      ready_positions_.size() != count || head_ready_positions_.size() != 2 ||
      head_lower_positions_.size() != 2 || head_upper_positions_.size() != 2 ||
      workspace_min_.size() != 3 || workspace_max_.size() != 3 ||
      wrist_roll_seeds_.empty() || vision_fk_compensation_m_.size() != 3 ||
      top_grasp_orientation_xyzw_.size() != 4 ||
      pregrasp_seed_positions_.size() != count ||
      pregrasp_wrist_flex_range_.size() != 2 ||
      pregrasp_wrist_flex_range_[0] > pregrasp_wrist_flex_range_[1] ||
      !positive(tf_timeout_s_) ||
      !positive(service_timeout_s_) || !positive(planning_timeout_s_) ||
      !positive(planning_velocity_scale_) || planning_velocity_scale_ > 1.0 ||
      !positive(planning_acceleration_scale_) || planning_acceleration_scale_ > 1.0 ||
      !positive(controller_timeout_s_) || !positive(policy_timeout_s_) ||
      !positive(policy_duration_s_) || !positive(verification_timeout_s_) ||
      !positive(joint_state_timeout_s_) ||
      !positive(head_move_duration_s_) ||
      !positive(pregrasp_gripper_duration_s_) || !positive(grasp_gripper_duration_s_) ||
      !positive(pregrasp_move_duration_s_) ||
      gripper_joint_.empty() ||
      !std::isfinite(gripper_lower_position_) || !std::isfinite(gripper_upper_position_) ||
      gripper_lower_position_ >= gripper_upper_position_ ||
      !std::isfinite(gripper_closed_position_) ||
      gripper_closed_position_<gripper_lower_position_ ||
      gripper_closed_position_> gripper_upper_position_ ||
      !positive(empty_gripper_tolerance_rad_) ||
      gripper_closed_position_ + empty_gripper_tolerance_rad_ >= gripper_upper_position_ ||
      !std::isfinite(pregrasp_gripper_position_) ||
      pregrasp_gripper_position_<gripper_lower_position_ ||
      pregrasp_gripper_position_> gripper_upper_position_ ||
      !positive(return_ready_duration_s_) || !positive(pregrasp_settle_timeout_s_) ||
      !positive(pregrasp_settle_hold_s_) ||
      pregrasp_settle_hold_s_ > pregrasp_settle_timeout_s_ ||
      !positive(pregrasp_arrival_tolerance_rad_) ||
      !positive(pregrasp_arrival_velocity_tolerance_radps_) ||
      policy_joint_names_.size() < count || !std::isfinite(pregrasp_offset_m_) ||
      pregrasp_offset_m_ < 0.0)
    {
      throw std::invalid_argument("grasp object parameters are invalid");
    }
    for (size_t index = 0; index < count; ++index) {
      if (!std::isfinite(ready_positions_[index]) || ready_positions_[index] < lower[index] ||
        ready_positions_[index] > upper[index])
      {
        throw std::invalid_argument("return-ready pose exceeds a right-arm limit");
      }
      if (!std::isfinite(pregrasp_seed_positions_[index]) ||
        pregrasp_seed_positions_[index] < lower[index] ||
        pregrasp_seed_positions_[index] > upper[index])
      {
        throw std::invalid_argument("calibrated pregrasp seed exceeds a right-arm limit");
      }
    }
    if (!std::isfinite(pregrasp_wrist_flex_range_[0]) ||
      !std::isfinite(pregrasp_wrist_flex_range_[1]))
    {
      throw std::invalid_argument("pregrasp wrist-flex range must be finite");
    }
    for (const double value : vision_fk_compensation_m_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("vision/FK compensation must be finite");
      }
    }
    double quaternion_norm = 0.0;
    for (const double value : top_grasp_orientation_xyzw_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("top-grasp orientation must be finite");
      }
      quaternion_norm += value * value;
    }
    if (std::abs(quaternion_norm - 1.0) > 1e-3) {
      throw std::invalid_argument("top-grasp orientation must be normalized");
    }
    const std::unordered_set<std::string> unique_joint_names(
      joint_names_.begin(), joint_names_.end());
    if (unique_joint_names.size() != count || unique_joint_names.count("") != 0) {
      throw std::invalid_argument("right-arm joint names must be unique and nonempty");
    }
    const std::unordered_set<std::string> unique_policy_joint_names(
      policy_joint_names_.begin(), policy_joint_names_.end());
    if (unique_policy_joint_names.size() != policy_joint_names_.size() ||
      unique_policy_joint_names.count("") != 0)
    {
      throw std::invalid_argument("policy context joint names must be unique and nonempty");
    }
    for (const auto & name : joint_names_) {
      if (unique_policy_joint_names.count(name) == 0) {
        throw std::invalid_argument("policy context must include every planned arm joint");
      }
    }
    if (unique_policy_joint_names.count(gripper_joint_) == 0) {
      throw std::invalid_argument("policy context must include the pregrasp gripper joint");
    }
    for (size_t index = 0; index < 2; ++index) {
      if (!std::isfinite(head_ready_positions_[index]) ||
        !std::isfinite(head_lower_positions_[index]) ||
        !std::isfinite(head_upper_positions_[index]) ||
        head_lower_positions_[index] >= head_upper_positions_[index] ||
        head_ready_positions_[index] < head_lower_positions_[index] ||
        head_ready_positions_[index] > head_upper_positions_[index])
      {
        throw std::invalid_argument("head ready preset exceeds a joint limit");
      }
    }
    for (size_t index = 0; index < 3; ++index) {
      if (!std::isfinite(workspace_min_[index]) || !std::isfinite(workspace_max_[index]) ||
        workspace_min_[index] >= workspace_max_[index])
      {
        throw std::invalid_argument("grasp workspace bounds are invalid");
      }
    }
    if (!std::all_of(
        wrist_roll_seeds_.begin(), wrist_roll_seeds_.end(),
        [&lower, &upper](double value) {
          return std::isfinite(value) && value >= lower.back() && value <= upper.back();
        }))
    {
      throw std::invalid_argument("wrist-roll IK seed exceeds the configured joint limit");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GraspObject::Goal> goal)
  {
    const std::string & backend = goal->backend;
    const bool act_valid = backend == "act" && !goal->target.header.frame_id.empty() &&
      finite_point(goal->target.point);
    const bool classical_valid = valid_classical_plan(
      goal->grasp_plan, backend, goal->dry_run);
    if (goal->object_id.empty() || (!act_valid && !classical_valid)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGrasp>)
  {
    cancel_active_child();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::GoalResponse handle_prepare_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PrepareGrasp::Goal> goal)
  {
    if (goal->object_id.empty() || goal->target.header.frame_id.empty() ||
      !finite_point(goal->target.point))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (goal_reserved_ || shutting_down_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_prepare_cancel(
    const std::shared_ptr<GoalHandlePrepare>)
  {
    cancel_active_child();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_prepare_accepted(const std::shared_ptr<GoalHandlePrepare> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute_prepare(goal_handle);});
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGrasp> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  void execute(const std::shared_ptr<GoalHandleGrasp> goal_handle)
  {
    struct ReservationGuard
    {
      GraspObjectServer * server;
      ~ReservationGuard()
      {
        server->clear_active_child();
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<GraspObject::Result>();
    result->backend_used = goal_handle->get_goal()->backend;
    VerifyGoalHandle::SharedPtr verification_handle;
    try {
      const auto & goal = *goal_handle->get_goal();
      if (goal.backend != "act") {
        execute_classical(goal_handle, goal);
        result->error.code = CapabilityError::NONE;
        result->error.message = goal.dry_run ?
          "dry-run classical plans validated without controller commands" :
          goal.backend + " grasp executed locally, verified, and arm returned ready";
        publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
        goal_handle->succeed(result);
        return;
      }
      // Freeze the detected point in base_link once. A visual retry happens
      // several seconds later, after ACT and verification, when the original
      // timestamp may already have fallen out of the TF buffer.
      const auto retry_target = transform_target_to_base(goal.target);
      const int maximum_attempts = goal.dry_run ? 1 : kMaximumGraspAttempts;
      for (int attempt = 1; attempt <= maximum_attempts; ++attempt) {
        try {
          const auto policy_start_context = prepare_grasp(
            goal_handle, retry_target, goal.dry_run);

          publish_feedback(
            goal_handle, "policy", 0.70F,
            attempt == 1 ? "starting learned-policy grasp session" :
            "retrying learned-policy grasp session with the detected target");
          run_policy(goal_handle, goal.object_id, goal.dry_run, policy_start_context);

          if (!goal.dry_run) {
            publish_feedback(
              goal_handle, "return_ready", 0.90F,
              "lifting the grasp clear of the table before wrist verification");
            send_named_trajectory(
              goal_handle, arm_client_, joint_names_, ready_positions_,
              return_ready_duration_s_, controller_timeout_s_, "return ready");

            publish_feedback(
              goal_handle, "verify_grasp", 0.94F,
              "capturing a fresh wrist image after the grasp cleared the table");
            auto verification = start_grasp_verification(goal_handle, goal.object_id);
            verification_handle = verification.handle;
            const double gripper_position = wait_for_gripper_position(goal_handle);
            finish_grasp_verification(goal_handle, verification, gripper_position);
            verification_handle.reset();

            publish_feedback(goal_handle, "head_ready", 0.98F, "raising head after grasp");
            send_named_trajectory(
              goal_handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
              head_ready_positions_, head_move_duration_s_, controller_timeout_s_, "head ready");
          }
          break;
        } catch (const GraspVerificationFailure & failure) {
          verification_handle.reset();
          if (attempt >= maximum_attempts) {
            publish_feedback(
              goal_handle, "head_ready", 0.98F,
              "raising head after the final failed grasp verification");
            send_named_trajectory(
              goal_handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
              head_ready_positions_, head_move_duration_s_, controller_timeout_s_, "head ready");
            throw;
          }
          publish_feedback(
            goal_handle, "retry_grasp", 0.50F,
            std::string("VLM and fully closed gripper both reported empty; retrying once: ") +
            failure.what());
        }
      }
      result->error.code = CapabilityError::NONE;
      result->error.message = goal.dry_run ?
        "dry-run pregrasp and policy contracts valid" :
        "pregrasp policy session completed, grasp verified, and arm returned ready";
      publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
      goal_handle->succeed(result);
    } catch (const GraspFailure & failure) {
      cancel_verification_noexcept(verification_handle);
      result->error.code = failure.code;
      result->error.message = failure.what();
      if (failure.canceled || goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      cancel_verification_noexcept(verification_handle);
      result->error.code = CapabilityError::INTERNAL_ERROR;
      result->error.message = error.what();
      goal_handle->abort(result);
    }
  }

  void execute_classical(
    const std::shared_ptr<GoalHandleGrasp> & goal_handle,
    const GraspObject::Goal & goal)
  {
    if (!grasp_alignment_ready_) {
      throw GraspFailure(
              CapabilityError::UNAVAILABLE,
              "centroid/gpd execution requires a validated grasp_alignment_file");
    }
    if (!valid_classical_plan(goal.grasp_plan, goal.backend, goal.dry_run)) {
      throw GraspFailure(
              CapabilityError::INVALID_GOAL, "classical TopGraspPlan is invalid");
    }
    VerifyGoalHandle::SharedPtr verification_handle;
    const int maximum_attempts = goal.dry_run ? 1 : kMaximumGraspAttempts;
    for (int attempt = 1; attempt <= maximum_attempts; ++attempt) {
      try {
        execute_classical_attempt(goal_handle, goal.grasp_plan, goal.dry_run);
        if (goal.dry_run) {
          return;
        }
        publish_feedback(
          goal_handle, "return_ready", 0.90F,
          "returning the locally executed grasp to the verified ready pose");
        send_named_trajectory(
          goal_handle, arm_client_, joint_names_, ready_positions_,
          return_ready_duration_s_, controller_timeout_s_, "return ready");
        publish_feedback(
          goal_handle, "verify_grasp", 0.94F,
          "capturing a fresh wrist image after the traditional grasp");
        auto verification = start_grasp_verification(goal_handle, goal.object_id);
        verification_handle = verification.handle;
        const double gripper_position = wait_for_gripper_position(goal_handle);
        finish_grasp_verification(goal_handle, verification, gripper_position);
        verification_handle.reset();
        publish_feedback(goal_handle, "head_ready", 0.98F, "raising head after grasp");
        send_named_trajectory(
          goal_handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
          head_ready_positions_, head_move_duration_s_, controller_timeout_s_, "head ready");
        return;
      } catch (const GraspVerificationFailure & failure) {
        cancel_verification_noexcept(verification_handle);
        verification_handle.reset();
        if (attempt >= maximum_attempts) {
          publish_feedback(
            goal_handle, "head_ready", 0.98F,
            "raising head after the final failed grasp verification");
          send_named_trajectory(
            goal_handle, head_client_, {"head_pan_joint", "head_tilt_joint"},
            head_ready_positions_, head_move_duration_s_, controller_timeout_s_, "head ready");
          throw;
        }
        publish_feedback(
          goal_handle, "retry_grasp", 0.45F,
          goal.backend + " grasp verification failed; retrying the same explicit backend: " +
          failure.what());
      } catch (...) {
        cancel_verification_noexcept(verification_handle);
        throw;
      }
    }
  }

  void execute_classical_attempt(
    const std::shared_ptr<GoalHandleGrasp> & goal_handle,
    const TopGraspPlan & plan,
    bool dry_run)
  {
    publish_feedback(
      goal_handle, "validate", 0.05F,
      "validating explicit " + plan.backend + " TopGraspPlan for local execution");
    if (!dry_run) {
      require_motion_gate();
    }
    const std::vector<double> wrist_seeds{static_cast<double>(plan.wrist_yaw_seed_rad)};
    if (!dry_run) {
      publish_feedback(
        goal_handle, "pregrasp_gripper", 0.12F,
        "opening gripper before traditional pregrasp");
      send_named_trajectory(
        goal_handle, gripper_client_, {gripper_joint_}, {pregrasp_gripper_position_},
        pregrasp_gripper_duration_s_, controller_timeout_s_, "pregrasp gripper");
    }
    execute_classical_pose(
      goal_handle, plan.pregrasp, wrist_seeds, dry_run,
      "pregrasp", 0.28F, true, false);
    execute_classical_pose(
      goal_handle, plan.grasp, wrist_seeds, dry_run,
      "grasp_descent", 0.52F, false, true);
    if (!dry_run) {
      publish_feedback(goal_handle, "close_gripper", 0.66F, "closing gripper on object");
      send_named_trajectory(
        goal_handle, gripper_client_, {gripper_joint_}, {gripper_closed_position_},
        grasp_gripper_duration_s_, controller_timeout_s_, "grasp gripper");
    }
    execute_classical_pose(
      goal_handle, plan.lift, wrist_seeds, dry_run,
      "lift", 0.82F, true, false);
  }

  void execute_classical_pose(
    const std::shared_ptr<GoalHandleGrasp> & goal_handle,
    const geometry_msgs::msg::PoseStamped & measured_pose,
    const std::vector<double> & wrist_seeds,
    bool dry_run,
    const std::string & label,
    float progress,
    bool apply_gravity_sag,
    bool enforce_calibration_envelope)
  {
    check_parent(goal_handle);
    const auto measured = wait_for_measured_state(goal_handle);
    const auto command_pose = transform_classical_pose(
      measured_pose, apply_gravity_sag, enforce_calibration_envelope);
    publish_feedback(
      goal_handle, label + "_ik", progress - 0.10F,
      "solving collision-aware IK for " + label);
    const auto ik_state = solve_ik(
      goal_handle, command_pose, measured.message, wrist_seeds);
    publish_feedback(
      goal_handle, label + "_plan", progress - 0.05F,
      "planning local MoveIt trajectory for " + label);
    auto trajectory = plan_pregrasp(goal_handle, ik_state, measured.message);
    const auto validation = plan_validator_->validate(trajectory, measured.positions);
    if (!validation) {
      throw GraspFailure(
              CapabilityError::SAFETY_REJECTED,
              "local " + label + " plan validation failed: " + validation.message);
    }
    if (dry_run) {
      publish_feedback(
        goal_handle, label, progress,
        "dry-run " + label + " MoveIt plan validated; controller not contacted");
      return;
    }
    trajectory = pregrasp_execution_trajectory(trajectory);
    publish_feedback(
      goal_handle, label, progress, "executing validated local " + label + " trajectory");
    send_trajectory(
      goal_handle, arm_client_, trajectory, controller_timeout_s_, label);
    wait_for_arm_arrival(goal_handle, trajectory, label);
  }

  geometry_msgs::msg::PoseStamped transform_classical_pose(
    const geometry_msgs::msg::PoseStamped & input,
    bool apply_gravity_sag,
    bool enforce_calibration_envelope)
  {
    geometry_msgs::msg::PoseStamped transformed;
    if (input.header.frame_id == base_frame_) {
      transformed = input;
    } else {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          base_frame_, input.header.frame_id, rclcpp::Time(input.header.stamp),
          tf2::durationFromSec(tf_timeout_s_));
        tf2::doTransform(input, transformed, transform);
      } catch (const std::exception & error) {
        throw GraspFailure(
                CapabilityError::UNAVAILABLE,
                "classical pose transform failed: " + std::string(error.what()));
      }
    }
    transformed.header.frame_id = base_frame_;
    if (enforce_calibration_envelope) {
      const double measured_values[3] = {
        transformed.pose.position.x,
        transformed.pose.position.y,
        transformed.pose.position.z,
      };
      for (size_t index = 0; index < 3; ++index) {
        if (measured_values[index] < calibration_workspace_min_[index] ||
          measured_values[index] > calibration_workspace_max_[index])
        {
          throw GraspFailure(
                  CapabilityError::SAFETY_REJECTED,
                  "classical grasp is outside the calibrated workspace envelope");
        }
      }
    }
    transformed.pose.position.x += vision_fk_compensation_m_[0];
    transformed.pose.position.y += vision_fk_compensation_m_[1];
    transformed.pose.position.z += vision_fk_compensation_m_[2] +
      (apply_gravity_sag ? gravity_sag_z_m_ : 0.0);
    const double values[3] = {
      transformed.pose.position.x,
      transformed.pose.position.y,
      transformed.pose.position.z,
    };
    for (size_t index = 0; index < 3; ++index) {
      if (!std::isfinite(values[index]) || values[index] < workspace_min_[index] ||
        values[index] > workspace_max_[index])
      {
        throw GraspFailure(
                CapabilityError::SAFETY_REJECTED,
                "classical " + input.header.frame_id + " pose is outside workspace");
      }
    }
    return transformed;
  }

  void wait_for_arm_arrival(
    const std::shared_ptr<GoalHandleGrasp> & goal_handle,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::string & label)
  {
    if (trajectory.points.empty() ||
      trajectory.points.back().positions.size() != trajectory.joint_names.size())
    {
      throw GraspFailure(
              CapabilityError::SAFETY_REJECTED,
              label + " trajectory has no complete final state");
    }
    std::map<std::string, double> target;
    for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      target[trajectory.joint_names[index]] = trajectory.points.back().positions[index];
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(pregrasp_settle_timeout_s_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    auto last_sample = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(goal_handle);
      require_motion_gate();
      const auto measured = measured_state();
      if (measured.received_at == last_sample) {
        std::this_thread::sleep_for(20ms);
        continue;
      }
      last_sample = measured.received_at;
      std::map<std::string, size_t> indices;
      for (size_t index = 0; index < measured.message.name.size(); ++index) {
        indices[measured.message.name[index]] = index;
      }
      bool stable = true;
      for (const auto & name : joint_names_) {
        const auto expected = target.find(name);
        const auto actual = indices.find(name);
        if (expected == target.end() || actual == indices.end() ||
          actual->second >= measured.message.position.size() ||
          !std::isfinite(measured.message.position[actual->second]) ||
          std::abs(measured.message.position[actual->second] - expected->second) >
          pregrasp_arrival_tolerance_rad_)
        {
          stable = false;
          break;
        }
        if (actual->second < measured.message.velocity.size() &&
          (!std::isfinite(measured.message.velocity[actual->second]) ||
          std::abs(measured.message.velocity[actual->second]) >
          pregrasp_arrival_velocity_tolerance_radps_))
        {
          stable = false;
          break;
        }
      }
      if (!stable) {
        stable_since = std::chrono::steady_clock::time_point{};
      } else {
        const auto current = std::chrono::steady_clock::now();
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          stable_since = current;
        }
        if (std::chrono::duration<double>(current - stable_since).count() >=
          pregrasp_settle_hold_s_)
        {
          return;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    throw GraspFailure(
            CapabilityError::SAFETY_REJECTED,
            label + " controller reported success but measured arm did not settle");
  }

  void execute_prepare(const std::shared_ptr<GoalHandlePrepare> goal_handle)
  {
    struct ReservationGuard
    {
      GraspObjectServer * server;
      ~ReservationGuard()
      {
        server->clear_active_child();
        std::lock_guard<std::mutex> lock(server->active_mutex_);
        server->goal_reserved_ = false;
      }
    } guard{this};

    auto result = std::make_shared<PrepareGrasp::Result>();
    try {
      const auto & goal = *goal_handle->get_goal();
      result->start_context = prepare_grasp(goal_handle, goal.target, goal.dry_run);
      result->error.code = CapabilityError::NONE;
      result->error.message = goal.dry_run ?
        "dry-run pregrasp plan validated" : "pregrasp reached and measured context captured";
      publish_feedback(goal_handle, "complete", 1.0F, result->error.message);
      goal_handle->succeed(result);
    } catch (const GraspFailure & failure) {
      result->error.code = failure.code;
      result->error.message = failure.what();
      if (failure.canceled || goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      result->error.code = CapabilityError::INTERNAL_ERROR;
      result->error.message = error.what();
      goal_handle->abort(result);
    }
  }

  template<typename GoalHandleT>
  PolicyStartContext prepare_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PointStamped & target,
    bool dry_run)
  {
    publish_feedback(goal_handle, "validate", 0.05F, "validating target and runtime");
    if (!dry_run) {
      require_motion_gate();
    }
    if (!grasp_alignment_ready_) {
      throw GraspFailure(
              CapabilityError::UNAVAILABLE,
              "ACT pregrasp requires a validated grasp_alignment_file");
    }
    const auto measured = wait_for_measured_state(goal_handle);
    const auto pregrasp = transform_pregrasp(target);
    publish_feedback(goal_handle, "ik", 0.15F, "solving bounded position-only pregrasp IK");
    geometry_msgs::msg::PoseStamped pregrasp_pose;
    pregrasp_pose.header = pregrasp.header;
    pregrasp_pose.pose.position = pregrasp.point;
    pregrasp_pose.pose.orientation.x = top_grasp_orientation_xyzw_[0];
    pregrasp_pose.pose.orientation.y = top_grasp_orientation_xyzw_[1];
    pregrasp_pose.pose.orientation.z = top_grasp_orientation_xyzw_[2];
    pregrasp_pose.pose.orientation.w = top_grasp_orientation_xyzw_[3];
    const auto ik_state = solve_ik(goal_handle, pregrasp_pose, measured.message, {});
    publish_feedback(goal_handle, "plan", 0.30F, "planning collision-aware pregrasp trajectory");
    auto trajectory = plan_pregrasp(goal_handle, ik_state, measured.message);
    const auto validation = plan_validator_->validate(
      trajectory, measured.positions);
    if (!validation) {
      throw GraspFailure(
              CapabilityError::SAFETY_REJECTED,
              "local pregrasp plan validation failed: " + validation.message);
    }
    if (dry_run) {
      publish_feedback(goal_handle, "pregrasp", 0.55F, "dry-run pregrasp plan validated");
      return PolicyStartContext{};
    }
    trajectory = pregrasp_execution_trajectory(trajectory);
    publish_feedback(
      goal_handle, "pregrasp_gripper", 0.40F,
      "opening gripper to the verified pregrasp position");
    send_named_trajectory(
      goal_handle, gripper_client_, {gripper_joint_}, {pregrasp_gripper_position_},
      pregrasp_gripper_duration_s_, controller_timeout_s_, "pregrasp gripper");
    publish_feedback(goal_handle, "pregrasp", 0.55F, "executing validated pregrasp plan");
    send_trajectory(goal_handle, arm_client_, trajectory, controller_timeout_s_, "pregrasp");
    publish_feedback(
      goal_handle, "pregrasp_verify", 0.65F,
      "verifying measured pregrasp arrival before ACT");
    return wait_for_pregrasp_context(goal_handle, trajectory);
  }

  geometry_msgs::msg::PointStamped transform_target_to_base(
    const geometry_msgs::msg::PointStamped & target)
  {
    geometry_msgs::msg::PointStamped transformed;
    if (target.header.frame_id == base_frame_) {
      transformed = target;
    } else {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          base_frame_, target.header.frame_id, rclcpp::Time(target.header.stamp),
          tf2::durationFromSec(tf_timeout_s_));
        tf2::doTransform(target, transformed, transform);
      } catch (const std::exception & error) {
        throw GraspFailure(
              CapabilityError::UNAVAILABLE,
            "target transform failed: " + std::string(error.what()));
      }
    }
    transformed.header.frame_id = base_frame_;
    return transformed;
  }

  geometry_msgs::msg::PointStamped transform_pregrasp(
    const geometry_msgs::msg::PointStamped & target)
  {
    if (!grasp_alignment_ready_) {
      throw GraspFailure(
              CapabilityError::UNAVAILABLE,
              "ACT pregrasp requires a validated grasp_alignment_file");
    }
    auto transformed = transform_target_to_base(target);
    const auto measured_target = transformed.point;
    transformed.point.x += vision_fk_compensation_m_[0];
    transformed.point.y += vision_fk_compensation_m_[1];
    transformed.point.z += vision_fk_compensation_m_[2] +
      gravity_sag_z_m_ + pregrasp_offset_m_;
    RCLCPP_INFO(
      get_logger(),
      "pregrasp target base_link measured=(%.5f,%.5f,%.5f) commanded=(%.5f,%.5f,%.5f)",
      measured_target.x, measured_target.y, measured_target.z,
      transformed.point.x, transformed.point.y, transformed.point.z);
    const double values[3] = {transformed.point.x, transformed.point.y, transformed.point.z};
    for (size_t index = 0; index < 3; ++index) {
      if (values[index] < workspace_min_[index] || values[index] > workspace_max_[index]) {
        throw GraspFailure(CapabilityError::SAFETY_REJECTED,
            "pregrasp target is outside workspace");
      }
    }
    return transformed;
  }

  struct MeasuredState
  {
    sensor_msgs::msg::JointState message;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::chrono::steady_clock::time_point received_at{};
    bool velocity_complete{true};
  };

  MeasuredState measured_state() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - joint_state_received_at_).count();
    if (age_s > joint_state_timeout_s_) {
      throw GraspFailure(CapabilityError::TIMEOUT, "right-arm joint state is missing or stale");
    }
    std::map<std::string, size_t> indices;
    for (size_t index = 0; index < latest_joint_state_.name.size(); ++index) {
      indices[latest_joint_state_.name[index]] = index;
    }
    MeasuredState measured;
    measured.message = latest_joint_state_;
    measured.received_at = joint_state_received_at_;
    for (const auto & name : joint_names_) {
      const auto item = indices.find(name);
      if (item == indices.end() || item->second >= latest_joint_state_.position.size()) {
        throw GraspFailure(CapabilityError::UNAVAILABLE, "right-arm joint state is incomplete");
      }
      const size_t index = item->second;
      const double position = latest_joint_state_.position[index];
      const bool have_velocity = index < latest_joint_state_.velocity.size();
      const double velocity = have_velocity ? latest_joint_state_.velocity[index] : 0.0;
      measured.velocity_complete = measured.velocity_complete && have_velocity;
      if (!std::isfinite(position) || !std::isfinite(velocity)) {
        throw GraspFailure(CapabilityError::SAFETY_REJECTED, "right-arm joint state is nonfinite");
      }
      measured.positions.push_back(position);
      measured.velocities.push_back(velocity);
    }
    return measured;
  }

  template<typename GoalHandleT>
  MeasuredState wait_for_measured_state(
    const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(service_timeout_s_));
    while (rclcpp::ok() && !shutting_down_) {
      check_parent(goal_handle);
      try {
        return measured_state();
      } catch (const GraspFailure & failure) {
        if (failure.code != CapabilityError::TIMEOUT &&
          failure.code != CapabilityError::UNAVAILABLE)
        {
          throw;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          throw;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    throw GraspFailure(CapabilityError::CANCELED, "joint-state wait interrupted", true);
  }

  double measured_gripper_position() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - joint_state_received_at_).count();
    if (age_s > joint_state_timeout_s_) {
      throw GraspFailure(CapabilityError::TIMEOUT, "right-gripper state is missing or stale");
    }
    const auto item = std::find(
      latest_joint_state_.name.begin(), latest_joint_state_.name.end(), gripper_joint_);
    if (item == latest_joint_state_.name.end()) {
      throw GraspFailure(CapabilityError::UNAVAILABLE, "right-gripper state is incomplete");
    }
    const size_t index = std::distance(latest_joint_state_.name.begin(), item);
    if (index >= latest_joint_state_.position.size() ||
      !std::isfinite(latest_joint_state_.position[index]))
    {
      throw GraspFailure(
              CapabilityError::SAFETY_REJECTED,
              "right-gripper position is nonfinite or incomplete");
    }
    return latest_joint_state_.position[index];
  }

  template<typename GoalHandleT>
  double wait_for_gripper_position(const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_s_);
    while (rclcpp::ok() && !shutting_down_) {
      check_parent(goal_handle);
      try {
        return measured_gripper_position();
      } catch (const GraspFailure & failure) {
        if (failure.code != CapabilityError::TIMEOUT &&
          failure.code != CapabilityError::UNAVAILABLE)
        {
          throw;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          throw;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    throw GraspFailure(CapabilityError::CANCELED, "gripper-state wait interrupted", true);
  }

  template<typename GoalHandleT>
  PolicyStartContext wait_for_pregrasp_context(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    if (trajectory.points.empty() ||
      trajectory.points.back().positions.size() != trajectory.joint_names.size())
    {
      throw GraspFailure(
              CapabilityError::SAFETY_REJECTED,
              "validated pregrasp trajectory has no complete final state");
    }
    std::map<std::string, double> target;
    for (size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      target[trajectory.joint_names[index]] = trajectory.points.back().positions[index];
    }
    target[gripper_joint_] = pregrasp_gripper_position_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(pregrasp_settle_timeout_s_));
    auto stable_since = std::chrono::steady_clock::time_point{};
    auto last_processed_state = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(goal_handle);
      require_motion_gate();
      try {
        const auto measured = measured_state();
        if (measured.received_at == last_processed_state) {
          std::this_thread::sleep_for(20ms);
          continue;
        }
        last_processed_state = measured.received_at;
        std::map<std::string, size_t> indices;
        for (size_t index = 0; index < measured.message.name.size(); ++index) {
          indices[measured.message.name[index]] = index;
        }
        bool stable = true;
        for (const auto & name : policy_joint_names_) {
          const auto expected = target.find(name);
          const auto measured_index = indices.find(name);
          if (expected == target.end() || measured_index == indices.end() ||
            measured_index->second >= measured.message.position.size() ||
            measured_index->second >= measured.message.velocity.size() ||
            !std::isfinite(measured.message.position[measured_index->second]) ||
            !std::isfinite(measured.message.velocity[measured_index->second]) ||
            std::abs(measured.message.position[measured_index->second] - expected->second) >
            pregrasp_arrival_tolerance_rad_ ||
            std::abs(measured.message.velocity[measured_index->second]) >
            pregrasp_arrival_velocity_tolerance_radps_)
          {
            stable = false;
            break;
          }
        }
        if (stable) {
          const auto steady_now = std::chrono::steady_clock::now();
          if (stable_since == std::chrono::steady_clock::time_point{}) {
            stable_since = steady_now;
          }
          if (std::chrono::duration<double>(steady_now - stable_since).count() >=
            pregrasp_settle_hold_s_)
          {
            std::map<std::string, size_t> indices;
            for (size_t index = 0; index < measured.message.name.size(); ++index) {
              indices[measured.message.name[index]] = index;
            }
            PolicyStartContext context;
            const auto established = now();
            context.context_id = "pregrasp-" + std::to_string(established.nanoseconds()) + "-" +
              std::to_string(++context_counter_);
            context.established_at = established;
            context.joint_names = policy_joint_names_;
            for (const auto & name : policy_joint_names_) {
              const auto item = indices.find(name);
              if (item == indices.end() || item->second >= measured.message.position.size()) {
                throw GraspFailure(
                        CapabilityError::UNAVAILABLE,
                        "pregrasp context joint state is incomplete");
              }
              const double position = measured.message.position[item->second];
              if (!std::isfinite(position)) {
                throw GraspFailure(
                        CapabilityError::SAFETY_REJECTED,
                        "pregrasp context joint state is nonfinite");
              }
              context.positions.push_back(position);
            }
            return context;
          }
        } else {
          stable_since = std::chrono::steady_clock::time_point{};
        }
      } catch (const GraspFailure & failure) {
        if (failure.code != CapabilityError::TIMEOUT &&
          failure.code != CapabilityError::UNAVAILABLE)
        {
          throw;
        }
        stable_since = std::chrono::steady_clock::time_point{};
      }
      std::this_thread::sleep_for(20ms);
    }
    throw GraspFailure(
            CapabilityError::SAFETY_REJECTED,
            "pregrasp controller reported success but measured joints did not settle at the plan");
  }

  template<typename GoalHandleT>
  sensor_msgs::msg::JointState solve_ik(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target,
    const sensor_msgs::msg::JointState & seed_state,
    const std::vector<double> & preferred_wrist_seeds)
  {
    wait_for_service<GetPositionIK>(ik_client_, service_timeout_s_, goal_handle, "compute_ik");
    std::string errors;
    auto wrist_candidates = preferred_wrist_seeds;
    wrist_candidates.insert(
      wrist_candidates.end(), wrist_roll_seeds_.begin(), wrist_roll_seeds_.end());
    if (include_radial_wrist_roll_seed_) {
      wrist_candidates.push_back(
        std::atan2(target.pose.position.y, target.pose.position.x));
    }
    for (double wrist_seed : wrist_candidates) {
      auto request = std::make_shared<GetPositionIK::Request>();
      request->ik_request.group_name = arm_group_;
      request->ik_request.ik_link_name = ee_link_;
      request->ik_request.robot_state.is_diff = true;
      request->ik_request.robot_state.joint_state = seed_state;
      for (size_t joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
        for (size_t state_index = 0;
          state_index < request->ik_request.robot_state.joint_state.name.size() &&
          state_index < request->ik_request.robot_state.joint_state.position.size();
          ++state_index)
        {
          if (request->ik_request.robot_state.joint_state.name[state_index] ==
            joint_names_[joint_index])
          {
            request->ik_request.robot_state.joint_state.position[state_index] =
              joint_index + 1 == joint_names_.size() ?
              wrist_seed : pregrasp_seed_positions_[joint_index];
          }
        }
      }
      auto & pose = request->ik_request.pose_stamped;
      pose = target;
      request->ik_request.timeout = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(1.0));
      request->ik_request.avoid_collisions = true;
      auto future = ik_client_->async_send_request(request);
      auto response = wait_service_future(future, planning_timeout_s_, goal_handle, "compute_ik");
      if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        const auto & solution = response->solution.joint_state;
        for (size_t index = 0;
          index < solution.name.size() && index < solution.position.size(); ++index)
        {
          if (solution.name[index] == joint_names_[3] &&
            (solution.position[index] < pregrasp_wrist_flex_range_[0] ||
            solution.position[index] > pregrasp_wrist_flex_range_[1]))
          {
            errors += " seed=" + std::to_string(wrist_seed) + " wrist_flex_out_of_range";
            break;
          }
          if (solution.name[index] == joint_names_[3]) {
            return solution;
          }
        }
      }
      errors += " seed=" + std::to_string(wrist_seed) +
        " code=" + std::to_string(response->error_code.val);
    }
    throw GraspFailure(CapabilityError::BACKEND_FAILURE, "pregrasp IK failed:" + errors);
  }

  template<typename GoalHandleT>
  trajectory_msgs::msg::JointTrajectory plan_pregrasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const sensor_msgs::msg::JointState & ik_state,
    const sensor_msgs::msg::JointState & start_state)
  {
    wait_for_service<GetMotionPlan>(
      plan_client_, service_timeout_s_, goal_handle, "plan_kinematic_path");
    std::map<std::string, double> target;
    for (size_t index = 0; index < ik_state.name.size() && index < ik_state.position.size();
      ++index)
    {
      target[ik_state.name[index]] = ik_state.position[index];
    }
    moveit_msgs::msg::Constraints constraints;
    for (const auto & name : joint_names_) {
      const auto item = target.find(name);
      if (item == target.end() || !std::isfinite(item->second)) {
        throw GraspFailure(CapabilityError::BACKEND_FAILURE,
            "IK solution omitted a right-arm joint");
      }
      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = name;
      constraint.position = item->second;
      constraint.tolerance_above = 0.001;
      constraint.tolerance_below = 0.001;
      constraint.weight = 1.0;
      constraints.joint_constraints.push_back(constraint);
    }
    auto request = std::make_shared<GetMotionPlan::Request>();
    request->motion_plan_request.group_name = arm_group_;
    request->motion_plan_request.start_state.is_diff = true;
    request->motion_plan_request.start_state.joint_state = start_state;
    request->motion_plan_request.goal_constraints = {constraints};
    request->motion_plan_request.num_planning_attempts = 2;
    request->motion_plan_request.allowed_planning_time = planning_timeout_s_;
    request->motion_plan_request.max_velocity_scaling_factor = planning_velocity_scale_;
    request->motion_plan_request.max_acceleration_scaling_factor = planning_acceleration_scale_;
    auto future = plan_client_->async_send_request(request);
    auto response = wait_service_future(future, planning_timeout_s_ + 1.0, goal_handle, "plan");
    const auto & plan = response->motion_plan_response;
    if (plan.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      throw GraspFailure(
              CapabilityError::BACKEND_FAILURE,
              "pregrasp planning failed with MoveIt code " + std::to_string(plan.error_code.val));
    }
    return plan.trajectory.joint_trajectory;
  }

  trajectory_msgs::msg::JointTrajectory pregrasp_execution_trajectory(
    const trajectory_msgs::msg::JointTrajectory & planned) const
  {
    if (planned.points.empty() ||
      planned.points.back().positions.size() != planned.joint_names.size())
    {
      throw GraspFailure(
              CapabilityError::BACKEND_FAILURE,
              "MoveIt pregrasp plan has no complete final state");
    }

    // Preserve the proven prototype execution contract. MoveIt validates a
    // collision-aware path and supplies the final joint target; the low-cost
    // servo controller then interpolates one four-second position command.
    // Forwarding MoveIt's dense derivatives makes ros2_control's command
    // limiter clip intermediate samples and is not how the working demo ran.
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names = planned.joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = planned.points.back().positions;
    point.velocities.assign(command.joint_names.size(), 0.0);
    point.accelerations.assign(command.joint_names.size(), 0.0);
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(pregrasp_move_duration_s_));
    command.points.push_back(std::move(point));
    return command;
  }

  void run_policy(
    const std::shared_ptr<GoalHandleGrasp> & parent,
    const std::string & object_id,
    bool dry_run,
    const PolicyStartContext & start_context)
  {
    LearnedPolicy::Goal goal;
    goal.policy_id = policy_id_;
    goal.object_id = object_id;
    goal.max_duration = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(policy_duration_s_));
    goal.dry_run = dry_run;
    goal.start_context = start_context;
    wait_for_action<LearnedPolicy>(
      policy_client_, service_timeout_s_, parent, "learned policy");
    auto send_future = policy_client_->async_send_goal(goal);
    auto handle = wait_action_future(send_future, service_timeout_s_, parent, "policy goal");
    if (!handle) {
      throw GraspFailure(CapabilityError::BACKEND_FAILURE, "learned policy rejected the goal");
    }
    set_active_child([this, handle]() {policy_client_->async_cancel_goal(handle);});
    auto result_future = policy_client_->async_get_result(handle);
    auto wrapped = wait_action_future(
      result_future, policy_timeout_s_, parent, "learned policy", !dry_run);
    clear_active_child();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      throw GraspFailure(CapabilityError::CANCELED, "learned policy canceled", true);
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      wrapped.result->error.code != CapabilityError::NONE)
    {
      throw GraspFailure(
              wrapped.result->error.code == CapabilityError::NONE ?
              CapabilityError::BACKEND_FAILURE : wrapped.result->error.code,
              "learned policy failed: " + wrapped.result->error.message);
    }
  }

  PendingVerification start_grasp_verification(
    const std::shared_ptr<GoalHandleGrasp> & parent,
    const std::string & object_id)
  {
    wait_for_action<VerifyGrasp>(
      verification_client_, service_timeout_s_, parent, "grasp verification");
    VerifyGrasp::Goal goal;
    goal.object_id = object_id;
    goal.dry_run = false;
    auto image_captured = std::make_shared<std::atomic<bool>>(false);
    rclcpp_action::Client<VerifyGrasp>::SendGoalOptions options;
    options.feedback_callback =
      [image_captured](
      VerifyGoalHandle::SharedPtr,
      const std::shared_ptr<const VerifyGrasp::Feedback> feedback) {
        if (feedback && feedback->state.phase == "verifying") {
          *image_captured = true;
        }
      };
    auto send_future = verification_client_->async_send_goal(goal, options);
    auto handle = wait_action_future(
      send_future, service_timeout_s_, parent, "grasp verification goal");
    if (!handle) {
      throw GraspFailure(
              CapabilityError::BACKEND_FAILURE,
              "grasp verification rejected the goal");
    }
    auto result = verification_client_->async_get_result(handle);
    set_active_child(
      [this, handle]() {verification_client_->async_cancel_goal(handle);});
    const auto capture_deadline = std::chrono::steady_clock::now() + 1s;
    while (rclcpp::ok() && !shutting_down_ && !*image_captured) {
      check_parent(parent);
      if (std::chrono::steady_clock::now() >= capture_deadline) {
        cancel_active_child();
        clear_active_child();
        throw GraspFailure(
                CapabilityError::TIMEOUT,
                "grasp verification did not capture a fresh wrist image");
      }
      std::this_thread::sleep_for(10ms);
    }
    clear_active_child();
    return PendingVerification{handle, result};
  }

  void finish_grasp_verification(
    const std::shared_ptr<GoalHandleGrasp> & parent,
    PendingVerification & pending,
    double gripper_position)
  {
    set_active_child(
      [this, handle = pending.handle]() {
        verification_client_->async_cancel_goal(handle);
      });
    auto wrapped = wait_action_future(
      pending.result, verification_timeout_s_, parent, "grasp verification");
    clear_active_child();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      throw GraspFailure(
              CapabilityError::CANCELED, "grasp verification canceled", true);
    }
    if (wrapped.result && !wrapped.result->grasped &&
      wrapped.result->error.code == CapabilityError::NOT_FOUND)
    {
      const bool gripper_fully_closed =
        gripper_position <= gripper_closed_position_ + empty_gripper_tolerance_rad_;
      if (gripper_fully_closed) {
        throw GraspVerificationFailure(
                "VLM and closed gripper both indicate an empty grasp: " +
                wrapped.result->error.message + "; gripper_position=" +
                std::to_string(gripper_position));
      }
      publish_feedback(
        parent, "verify_grasp", 0.98F,
        "VLM reported empty, but gripper stopped before fully closed at " +
        std::to_string(gripper_position) + " rad; accepting grasp");
      return;
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error.code != CapabilityError::NONE || !wrapped.result->grasped)
    {
      throw GraspFailure(
              !wrapped.result || wrapped.result->error.code == CapabilityError::NONE ?
              CapabilityError::BACKEND_FAILURE : wrapped.result->error.code,
              "grasp verification failed: " +
              (wrapped.result ? wrapped.result->error.message : "missing result"));
    }
    publish_feedback(
      parent, "verify_grasp", 0.98F,
      wrapped.result->error.message);
  }

  void cancel_verification_noexcept(const VerifyGoalHandle::SharedPtr & handle)
  {
    if (!handle) {
      return;
    }
    try {
      verification_client_->async_cancel_goal(handle);
    } catch (const std::exception & error) {
      RCLCPP_DEBUG(
        get_logger(), "grasp verification already finished while cleaning up: %s",
        error.what());
    }
  }

  template<typename GoalHandleT>
  void send_named_trajectory(
    const std::shared_ptr<GoalHandleT> & parent,
    const rclcpp_action::Client<FollowTrajectory>::SharedPtr & client,
    const std::vector<std::string> & names,
    const std::vector<double> & positions,
    double duration_s,
    double timeout_s,
    const std::string & label)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(duration_s));
    trajectory.points.push_back(point);
    send_trajectory(parent, client, trajectory, timeout_s, label);
  }

  template<typename GoalHandleT>
  void send_trajectory(
    const std::shared_ptr<GoalHandleT> & parent,
    const rclcpp_action::Client<FollowTrajectory>::SharedPtr & client,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double timeout_s,
    const std::string & label)
  {
    require_motion_gate();
    wait_for_action<FollowTrajectory>(client, service_timeout_s_, parent, label);
    FollowTrajectory::Goal goal;
    goal.trajectory = trajectory;
    auto send_future = client->async_send_goal(goal);
    auto handle = wait_action_future(send_future, service_timeout_s_, parent, label + " goal");
    if (!handle) {
      throw GraspFailure(CapabilityError::BACKEND_FAILURE, label + " controller rejected goal");
    }
    set_active_child([client, handle]() {client->async_cancel_goal(handle);});
    auto result_future = client->async_get_result(handle);
    auto wrapped = wait_action_future(
      result_future, timeout_s, parent, label, true);
    clear_active_child();
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      throw GraspFailure(CapabilityError::CANCELED, label + " canceled", true);
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      wrapped.result->error_code != FollowTrajectory::Result::SUCCESSFUL)
    {
      const std::string detail = wrapped.result->error_string.empty() ?
        std::string{} : ": " + wrapped.result->error_string;
      throw GraspFailure(
              CapabilityError::BACKEND_FAILURE,
              label + " controller failed with error_code=" +
              std::to_string(wrapped.result->error_code) + detail);
    }
  }

  template<typename ServiceT, typename GoalHandleT>
  void wait_for_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    double timeout_s,
    const std::shared_ptr<GoalHandleT> & parent,
    const std::string & label)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(parent);
      if (client->wait_for_service(50ms)) {
        return;
      }
    }
    throw GraspFailure(CapabilityError::UNAVAILABLE, label + " service is unavailable");
  }

  template<typename FutureT, typename GoalHandleT>
  auto wait_service_future(
    FutureT & future,
    double timeout_s,
    const std::shared_ptr<GoalHandleT> & parent,
    const std::string & label) -> decltype(future.get())
  {
    return wait_future(future, timeout_s, parent, label, false);
  }

  template<typename ActionT, typename GoalHandleT>
  void wait_for_action(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    double timeout_s,
    const std::shared_ptr<GoalHandleT> & parent,
    const std::string & label)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
    while (rclcpp::ok() && !shutting_down_ && std::chrono::steady_clock::now() < deadline) {
      check_parent(parent);
      if (client->wait_for_action_server(50ms)) {
        return;
      }
    }
    throw GraspFailure(CapabilityError::UNAVAILABLE, label + " action is unavailable");
  }

  template<typename FutureT, typename GoalHandleT>
  auto wait_action_future(
    FutureT & future,
    double timeout_s,
    const std::shared_ptr<GoalHandleT> & parent,
    const std::string & label,
    bool require_gate = false) -> decltype(future.get())
  {
    return wait_future(future, timeout_s, parent, label, require_gate);
  }

  template<typename FutureT, typename GoalHandleT>
  auto wait_future(
    FutureT & future,
    double timeout_s,
    const std::shared_ptr<GoalHandleT> & parent,
    const std::string & label,
    bool require_gate) -> decltype(future.get())
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
    while (rclcpp::ok() && !shutting_down_ && future.wait_for(20ms) != std::future_status::ready) {
      check_parent(parent);
      if (require_gate) {
        try {
          require_motion_gate();
        } catch (...) {
          cancel_active_child();
          throw;
        }
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        cancel_active_child();
        throw GraspFailure(CapabilityError::TIMEOUT, label + " timed out");
      }
    }
    if (!rclcpp::ok() || shutting_down_) {
      throw GraspFailure(CapabilityError::CANCELED, label + " interrupted", true);
    }
    return future.get();
  }

  template<typename GoalHandleT>
  void check_parent(const std::shared_ptr<GoalHandleT> & parent)
  {
    if (parent->is_canceling()) {
      cancel_active_child();
      throw GraspFailure(CapabilityError::CANCELED, "grasp canceled", true);
    }
  }

  void require_motion_gate() const
  {
    if (!execution_enabled_) {
      throw GraspFailure(CapabilityError::SAFETY_REJECTED, "execution_enabled is false");
    }
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleGrasp> & goal_handle,
    const std::string & phase,
    float progress,
    const std::string & message)
  {
    auto feedback = std::make_shared<GraspObject::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    goal_handle->publish_feedback(feedback);
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandlePrepare> & goal_handle,
    const std::string & phase,
    float progress,
    const std::string & message)
  {
    auto feedback = std::make_shared<PrepareGrasp::Feedback>();
    feedback->state.phase = phase;
    feedback->state.progress = progress;
    feedback->state.message = message;
    goal_handle->publish_feedback(feedback);
  }

  void set_active_child(std::function<void()> cancel)
  {
    std::lock_guard<std::mutex> lock(child_mutex_);
    cancel_active_ = std::move(cancel);
  }

  void clear_active_child()
  {
    std::lock_guard<std::mutex> lock(child_mutex_);
    cancel_active_ = nullptr;
  }

  void cancel_active_child()
  {
    std::function<void()> cancel;
    {
      std::lock_guard<std::mutex> lock(child_mutex_);
      cancel = cancel_active_;
    }
    if (cancel) {
      cancel();
    }
  }

  bool execution_enabled_{false};
  std::string action_name_;
  std::string prepare_action_name_;
  std::string policy_id_;
  std::string base_frame_;
  std::string arm_group_;
  std::string ee_link_;
  std::string grasp_alignment_file_;
  double pregrasp_offset_m_{0.08};
  double gravity_sag_z_m_{0.0};
  bool grasp_alignment_ready_{false};
  bool include_radial_wrist_roll_seed_{true};
  double tf_timeout_s_{1.0};
  double service_timeout_s_{5.0};
  double planning_timeout_s_{5.0};
  double planning_velocity_scale_{1.0};
  double planning_acceleration_scale_{1.0};
  double controller_timeout_s_{12.0};
  double policy_timeout_s_{25.0};
  double policy_duration_s_{20.0};
  double verification_timeout_s_{10.0};
  double joint_state_timeout_s_{0.50};
  double head_move_duration_s_{1.5};
  double pregrasp_gripper_duration_s_{2.0};
  double grasp_gripper_duration_s_{2.0};
  double pregrasp_move_duration_s_{4.0};
  double return_ready_duration_s_{4.0};
  double pregrasp_settle_timeout_s_{2.0};
  double pregrasp_settle_hold_s_{0.10};
  double pregrasp_arrival_tolerance_rad_{0.05};
  double pregrasp_arrival_velocity_tolerance_radps_{0.05};
  std::string gripper_joint_;
  double pregrasp_gripper_position_{1.64};
  double gripper_lower_position_{0.0};
  double gripper_upper_position_{1.65};
  double gripper_closed_position_{0.0};
  double empty_gripper_tolerance_rad_{0.05};
  std::vector<double> head_ready_positions_;
  std::vector<double> head_lower_positions_;
  std::vector<double> head_upper_positions_;
  std::vector<double> ready_positions_;
  std::vector<double> vision_fk_compensation_m_;
  std::vector<double> top_grasp_orientation_xyzw_;
  std::vector<double> wrist_roll_seeds_;
  std::vector<double> pregrasp_seed_positions_;
  std::vector<double> pregrasp_wrist_flex_range_;
  std::vector<double> workspace_min_;
  std::vector<double> workspace_max_;
  std::vector<double> calibration_workspace_min_;
  std::vector<double> calibration_workspace_max_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> policy_joint_names_;
  std::unique_ptr<GraspPlanValidator> plan_validator_;

  mutable std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::chrono::steady_clock::time_point joint_state_received_at_{};
  std::mutex active_mutex_;
  bool goal_reserved_{false};
  std::mutex child_mutex_;
  std::function<void()> cancel_active_;
  std::atomic<bool> shutting_down_{false};
  std::atomic<uint64_t> context_counter_{0};
  std::thread worker_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<GetMotionPlan>::SharedPtr plan_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr head_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Client<LearnedPolicy>::SharedPtr policy_client_;
  rclcpp_action::Client<VerifyGrasp>::SharedPtr verification_client_;
  rclcpp_action::Server<GraspObject>::SharedPtr action_server_;
  rclcpp_action::Server<PrepareGrasp>::SharedPtr prepare_action_server_;
};

}  // namespace xlerobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xlerobot_manipulation::GraspObjectServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
