#include <elfin_freedrive_controller/elfin_freedrive_controller.h>
#include <elfin_freedrive_controller/freedrive_math.h>

#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace elfin_freedrive_controller {

ElfinFreedriveController::ElfinFreedriveController()
    : gravity_vector_(0.0, 0.0, -9.81),
      gravity_scale_(1.0),
      handoff_duration_(0.5),
      settle_handoff_duration_(0.10),
      minimum_validation_effort_(1.5),
      minimum_validation_joints_(2),
      minimum_model_alignment_(0.90),
      minimum_model_scale_(0.50),
      maximum_model_scale_(2.0),
      maximum_model_residual_(0.30),
      maximum_entry_effort_error_(5.0),
      minimum_adaptive_scale_(0.50),
      maximum_adaptive_scale_(2.0),
      maximum_gravity_effort_fraction_(0.90),
      verified_payload_maximum_gravity_effort_fraction_(0.92),
      limit_margin_(0.08),
      limit_stiffness_(5.0),
      limit_damping_(0.5),
      velocity_limit_damping_(2.0),
      hard_limit_margin_(0.02),
      hard_limit_toward_velocity_(0.02),
      hard_limit_inward_travel_(0.003),
      hard_stop_damping_(12.0),
      velocity_limit_scale_(1.0),
      minimum_velocity_limit_scale_(0.50),
      maximum_velocity_limit_scale_(3.0),
      minimum_damping_scale_(0.05),
      maximum_damping_scale_(5.0),
      maximum_payload_mass_(5.0),
      require_model_validation_(true),
      allow_model_validation_warning_(true),
      gravity_calibration_verified_(false),
      adaptive_entry_scale_(false),
      controlled_hold_validation_(false),
      payload_hold_verified_(false),
      command_initialized_(false),
      model_validation_passed_(false),
      model_validation_warning_(false),
      gravity_capacity_valid_(false),
      model_excited_joints_(0),
      model_direction_mismatches_(0),
      model_alignment_(0.0),
      model_scale_estimate_(0.0),
      model_normalized_residual_(0.0),
      minimum_warning_alignment_(0.50),
      handoff_progress_(0.0),
      settle_active_(false),
      settle_requested_(false),
      running_(false) {}

ElfinFreedriveController::~ElfinFreedriveController() = default;

bool ElfinFreedriveController::finiteVector(
    const std::vector<double>& value) const {
  for (double item : value) {
    if (!std::isfinite(item)) {
      return false;
    }
  }
  return true;
}

bool ElfinFreedriveController::getVectorParam(
    const ros::NodeHandle& nh,
    const std::string& name,
    std::size_t expected_size,
    const std::vector<double>& default_value,
    std::vector<double>& value) const {
  if (!nh.getParam(name, value)) {
    value = default_value;
  }
  if (value.size() != expected_size || !finiteVector(value)) {
    ROS_ERROR_STREAM("Parameter '" << nh.getNamespace() << "/" << name
                     << "' must contain " << expected_size
                     << " finite values.");
    return false;
  }
  return true;
}

bool ElfinFreedriveController::init(
    hardware_interface::EffortJointInterface* hw,
    ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  if (!controller_nh.getParam("joints", joint_names_) || joint_names_.empty()) {
    ROS_ERROR("elfin_freedrive_controller requires a non-empty 'joints' list");
    return false;
  }
  if (joint_names_.size() != 6) {
    ROS_ERROR_STREAM("This controller requires six joints; got "
                     << joint_names_.size());
    return false;
  }

  joints_.reserve(joint_names_.size());
  try {
    for (const std::string& name : joint_names_) {
      joints_.push_back(hw->getHandle(name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& error) {
    ROS_ERROR_STREAM("Cannot claim effort joint: " << error.what());
    return false;
  }

  std::string root_link;
  std::string tip_link;
  controller_nh.param<std::string>("root_link", root_link, "elfin_base");
  controller_nh.param<std::string>("tip_link", tip_link, "elfin_end_link");

  std::string robot_description;
  urdf::Model model;
  if (!root_nh.getParam("robot_description", robot_description) ||
      !model.initString(robot_description)) {
    ROS_ERROR("Cannot parse robot_description for gravity compensation");
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree) ||
      !tree.getChain(root_link, tip_link, chain_)) {
    ROS_ERROR_STREAM("Cannot construct KDL chain from '" << root_link
                     << "' to '" << tip_link << "'");
    return false;
  }
  if (chain_.getNrOfJoints() != joint_names_.size()) {
    ROS_ERROR_STREAM("KDL chain contains " << chain_.getNrOfJoints()
                     << " joints, but controller has " << joint_names_.size());
    return false;
  }

  std::vector<std::string> chain_joint_names;
  chain_joint_names.reserve(chain_.getNrOfJoints());
  for (unsigned int segment_index = 0;
       segment_index < chain_.getNrOfSegments(); ++segment_index) {
    const KDL::Joint& joint = chain_.getSegment(segment_index).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      chain_joint_names.push_back(joint.getName());
    }
  }

  chain_to_joint_.clear();
  chain_to_joint_.reserve(chain_joint_names.size());
  for (const std::string& chain_name : chain_joint_names) {
    const auto found =
        std::find(joint_names_.begin(), joint_names_.end(), chain_name);
    if (found == joint_names_.end()) {
      ROS_ERROR_STREAM("KDL chain joint '" << chain_name
                       << "' is absent from controller joints");
      return false;
    }
    chain_to_joint_.push_back(
        static_cast<std::size_t>(std::distance(joint_names_.begin(), found)));
  }

  joint_to_chain_.assign(joint_names_.size(), chain_to_joint_.size());
  for (std::size_t chain_index = 0;
       chain_index < chain_to_joint_.size(); ++chain_index) {
    joint_to_chain_[chain_to_joint_[chain_index]] = chain_index;
  }
  if (std::find(joint_to_chain_.begin(), joint_to_chain_.end(),
                chain_to_joint_.size()) != joint_to_chain_.end()) {
    ROS_ERROR("KDL/controller joint mapping is incomplete");
    return false;
  }

  std::vector<double> gravity_values;
  if (!getVectorParam(controller_nh, "gravity", 3,
                      {0.0, 0.0, -9.81}, gravity_values)) {
    return false;
  }
  gravity_vector_ =
      KDL::Vector(gravity_values[0], gravity_values[1], gravity_values[2]);

  controller_nh.param("gravity_scale", gravity_scale_, 1.0);
  controller_nh.param("handoff_duration", handoff_duration_, 0.5);
  controller_nh.param("settle_handoff_duration",
                      settle_handoff_duration_, 0.10);
  controller_nh.param("minimum_validation_effort",
                      minimum_validation_effort_, 1.5);
  int minimum_validation_joints = 2;
  controller_nh.param("minimum_validation_joints",
                      minimum_validation_joints, 2);
  controller_nh.param("minimum_model_alignment",
                      minimum_model_alignment_, 0.90);
  controller_nh.param("minimum_model_scale", minimum_model_scale_, 0.50);
  controller_nh.param("maximum_model_scale", maximum_model_scale_, 2.0);
  controller_nh.param("maximum_model_residual",
                      maximum_model_residual_, 0.30);
  controller_nh.param("maximum_entry_effort_error",
                      maximum_entry_effort_error_, 5.0);
  controller_nh.param("minimum_adaptive_scale",
                      minimum_adaptive_scale_, 0.50);
  controller_nh.param("maximum_adaptive_scale",
                      maximum_adaptive_scale_, 2.0);
  controller_nh.param("maximum_gravity_effort_fraction",
                      maximum_gravity_effort_fraction_, 0.90);
  controller_nh.param("verified_payload_maximum_gravity_effort_fraction",
                      verified_payload_maximum_gravity_effort_fraction_, 0.92);
  controller_nh.param("require_model_validation",
                      require_model_validation_, true);
  controller_nh.param("allow_model_validation_warning",
                      allow_model_validation_warning_, true);
  controller_nh.param("gravity_calibration_verified",
                      gravity_calibration_verified_, false);
  controller_nh.param("minimum_warning_alignment",
                      minimum_warning_alignment_, 0.50);
  controller_nh.param("adaptive_entry_scale", adaptive_entry_scale_, false);
  controller_nh.param("limit_margin", limit_margin_, 0.08);
  controller_nh.param("hard_limit_margin", hard_limit_margin_, 0.02);
  controller_nh.param("hard_limit_toward_velocity",
                      hard_limit_toward_velocity_, 0.02);
  controller_nh.param("hard_limit_inward_travel",
                      hard_limit_inward_travel_, 0.003);
  controller_nh.param("limit_stiffness", limit_stiffness_, 5.0);
  controller_nh.param("limit_damping", limit_damping_, 0.5);
  controller_nh.param("velocity_limit_damping", velocity_limit_damping_, 2.0);
  controller_nh.param("hard_stop_damping", hard_stop_damping_, 12.0);
  double velocity_limit_scale = 1.0;
  controller_nh.param("velocity_limit_scale", velocity_limit_scale, 1.0);
  controller_nh.param("minimum_velocity_limit_scale",
                      minimum_velocity_limit_scale_, 0.50);
  controller_nh.param("maximum_velocity_limit_scale",
                      maximum_velocity_limit_scale_, 3.0);
  controller_nh.param("minimum_damping_scale", minimum_damping_scale_, 0.05);
  controller_nh.param("maximum_damping_scale", maximum_damping_scale_, 5.0);
  controller_nh.param("maximum_payload_mass", maximum_payload_mass_, 5.0);
  if (!std::isfinite(gravity_scale_) || gravity_scale_ < 0.0 ||
      gravity_scale_ > 2.0 || !std::isfinite(limit_margin_) ||
      !std::isfinite(handoff_duration_) || handoff_duration_ < 0.1 ||
      handoff_duration_ > 30.0 ||
      !std::isfinite(settle_handoff_duration_) ||
      settle_handoff_duration_ < 0.02 || settle_handoff_duration_ > 1.0 ||
      !std::isfinite(minimum_validation_effort_) ||
      minimum_validation_effort_ < 0.0 ||
      minimum_validation_joints < 1 || minimum_validation_joints > 6 ||
      !std::isfinite(minimum_model_alignment_) ||
      minimum_model_alignment_ < -1.0 || minimum_model_alignment_ > 1.0 ||
      !std::isfinite(minimum_model_scale_) || minimum_model_scale_ <= 0.0 ||
      !std::isfinite(maximum_model_scale_) ||
      maximum_model_scale_ <= minimum_model_scale_ ||
      !std::isfinite(maximum_model_residual_) ||
      maximum_model_residual_ < 0.0 || maximum_model_residual_ > 2.0 ||
      !std::isfinite(maximum_entry_effort_error_) ||
      maximum_entry_effort_error_ <= 0.0 ||
      !std::isfinite(minimum_warning_alignment_) ||
      minimum_warning_alignment_ < 0.0 ||
      minimum_warning_alignment_ > minimum_model_alignment_ ||
      !std::isfinite(minimum_adaptive_scale_) ||
      minimum_adaptive_scale_ <= 0.0 ||
      !std::isfinite(maximum_adaptive_scale_) ||
      maximum_adaptive_scale_ < minimum_adaptive_scale_ ||
      !std::isfinite(maximum_gravity_effort_fraction_) ||
      maximum_gravity_effort_fraction_ < 0.50 ||
      maximum_gravity_effort_fraction_ >= 1.0 ||
      !std::isfinite(verified_payload_maximum_gravity_effort_fraction_) ||
      verified_payload_maximum_gravity_effort_fraction_ <
          maximum_gravity_effort_fraction_ ||
      verified_payload_maximum_gravity_effort_fraction_ >= 1.0 ||
      !std::isfinite(hard_limit_margin_) || hard_limit_margin_ <= 0.0 ||
      hard_limit_margin_ >= limit_margin_ || limit_margin_ >= 0.5 ||
      !std::isfinite(hard_limit_toward_velocity_) ||
      hard_limit_toward_velocity_ <= 0.0 ||
      !std::isfinite(hard_limit_inward_travel_) ||
      hard_limit_inward_travel_ <= 0.0 ||
      hard_limit_inward_travel_ >= hard_limit_margin_ ||
      !std::isfinite(limit_stiffness_) || limit_stiffness_ < 0.0 ||
      !std::isfinite(limit_damping_) || limit_damping_ < 0.0 ||
      !std::isfinite(velocity_limit_damping_) ||
      velocity_limit_damping_ < 0.0 || !std::isfinite(hard_stop_damping_) ||
      hard_stop_damping_ <= 0.0 ||
      !math::velocityScaleValid(velocity_limit_scale,
                                minimum_velocity_limit_scale_,
                                maximum_velocity_limit_scale_) ||
      !math::velocityScaleValid(1.0, minimum_damping_scale_,
                                maximum_damping_scale_) ||
      !std::isfinite(maximum_payload_mass_) || maximum_payload_mass_ <= 0.0) {
    ROS_ERROR("Invalid scalar safety parameter in freedrive controller config");
    return false;
  }
  minimum_validation_joints_ =
      static_cast<unsigned int>(minimum_validation_joints);
  velocity_limit_scale_.store(velocity_limit_scale);

  double effort_limit_scale = 1.0;
  controller_nh.param("effort_limit_scale", effort_limit_scale, 1.0);
  if (!std::isfinite(effort_limit_scale) || effort_limit_scale <= 0.0 ||
      effort_limit_scale > 1.0) {
    ROS_ERROR("effort_limit_scale must be in (0, 1]");
    return false;
  }

  const std::vector<double> six_zeros(6, 0.0);
  const std::vector<double> six_ones(6, 1.0);
  std::vector<double> damping_scales;
  if (!getVectorParam(controller_nh, "damping", 6,
                      {0.8, 0.8, 0.5, 0.25, 0.12, 0.12}, damping_) ||
      !getVectorParam(controller_nh, "friction", 6,
                      six_zeros, friction_) ||
      !getVectorParam(controller_nh, "friction_velocity", 6,
                      {0.05, 0.05, 0.05, 0.05, 0.05, 0.05},
                      friction_velocity_) ||
      !getVectorParam(controller_nh, "torque_rate_limits", 6,
                      {100.0, 100.0, 60.0, 40.0, 20.0, 20.0},
                      torque_rate_limits_) ||
      !getVectorParam(controller_nh, "velocity_soft_limits", 6,
                      {0.35, 0.35, 0.35, 0.45, 0.55, 0.55},
                      velocity_soft_limits_) ||
      !getVectorParam(controller_nh, "velocity_hard_limits", 6,
                      {0.60, 0.60, 0.60, 0.75, 0.90, 0.90},
                      velocity_hard_limits_) ||
      !getVectorParam(controller_nh, "gravity_joint_scales", 6,
                      six_ones, gravity_joint_scales_) ||
      !getVectorParam(controller_nh, "gravity_bias", 6,
                      six_zeros, gravity_bias_) ||
      !getVectorParam(controller_nh, "settle_damping", 6,
                      {8.0, 8.0, 6.0, 3.0, 2.0, 2.0},
                      settle_damping_) ||
      !getVectorParam(controller_nh, "damping_scales", 6,
                      six_ones, damping_scales)) {
    return false;
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (damping_[i] < 0.0 || friction_[i] < 0.0 ||
        friction_velocity_[i] <= 0.0 || torque_rate_limits_[i] <= 0.0 ||
        velocity_soft_limits_[i] <= 0.0 || velocity_hard_limits_[i] <= 0.0 ||
        velocity_soft_limits_[i] >= velocity_hard_limits_[i] ||
        gravity_joint_scales_[i] <= 0.0 ||
        gravity_joint_scales_[i] > 4.0 || settle_damping_[i] <= 0.0) {
      ROS_ERROR_STREAM("Invalid vector safety parameter for " << joint_names_[i]);
      return false;
    }
    if (!math::velocityScaleValid(damping_scales[i],
                                  minimum_damping_scale_,
                                  maximum_damping_scale_)) {
      ROS_ERROR_STREAM("Invalid damping scale for " << joint_names_[i]);
      return false;
    }
  }
  std::array<double, 6> initial_damping_scales{};
  std::copy(damping_scales.begin(), damping_scales.end(),
            initial_damping_scales.begin());
  damping_scales_.writeFromNonRT(initial_damping_scales);

  double initial_payload_mass = 0.0;
  controller_nh.param("payload_mass", initial_payload_mass, 0.0);
  std::vector<double> initial_payload_center;
  if (!getVectorParam(controller_nh, "payload_center_of_mass", 3,
                      {0.0, 0.0, 0.0}, initial_payload_center)) {
    return false;
  }
  payload::Model initial_payload;
  initial_payload.mass = initial_payload_mass;
  std::copy(initial_payload_center.begin(), initial_payload_center.end(),
            initial_payload.center_of_mass.begin());
  if (!payload::valid(initial_payload, maximum_payload_mass_)) {
    ROS_ERROR("Initial payload mass/center of mass is invalid or mass is out of bounds");
    return false;
  }
  payload_model_.writeFromNonRT(initial_payload);

  lower_limits_.assign(6, -std::numeric_limits<double>::infinity());
  upper_limits_.assign(6, std::numeric_limits<double>::infinity());
  velocity_limits_.assign(6, std::numeric_limits<double>::infinity());
  effort_limits_.assign(6, 0.0);
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const urdf::JointConstSharedPtr joint = model.getJoint(joint_names_[i]);
    if (!joint || !joint->limits) {
      ROS_ERROR_STREAM("Joint '" << joint_names_[i]
                       << "' has no URDF limits; refusing torque mode");
      return false;
    }
    lower_limits_[i] = joint->limits->lower;
    upper_limits_[i] = joint->limits->upper;
    velocity_limits_[i] = joint->limits->velocity;
    velocity_hard_limits_[i] =
        std::min(velocity_hard_limits_[i], velocity_limits_[i]);
    velocity_soft_limits_[i] =
        std::min(velocity_soft_limits_[i], 0.8 * velocity_hard_limits_[i]);
    effort_limits_[i] = joint->limits->effort * effort_limit_scale;
    if (!std::isfinite(effort_limits_[i]) || effort_limits_[i] <= 0.0 ||
        !std::isfinite(velocity_hard_limits_[i]) ||
        velocity_hard_limits_[i] <= 0.0) {
      ROS_ERROR_STREAM("Joint '" << joint_names_[i]
                       << "' has invalid URDF limits");
      return false;
    }
  }

  std::vector<double> configured_effort_limits;
  if (controller_nh.hasParam("effort_limits")) {
    if (!getVectorParam(controller_nh, "effort_limits", 6,
                        six_zeros, configured_effort_limits)) {
      return false;
    }
    for (std::size_t i = 0; i < effort_limits_.size(); ++i) {
      if (configured_effort_limits[i] <= 0.0) {
        ROS_ERROR("effort_limits values must be positive");
        return false;
      }
      effort_limits_[i] =
          std::min(effort_limits_[i], configured_effort_limits[i]);
    }
  }

  dynamics_.reset(new KDL::ChainDynParam(chain_, gravity_vector_));
  payload_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
  payload_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
  q_chain_ = KDL::JntArray(chain_.getNrOfJoints());
  gravity_torque_chain_ = KDL::JntArray(chain_.getNrOfJoints());
  payload_jacobian_ = KDL::Jacobian(chain_.getNrOfJoints());
  payload_effort_.fill(0.0);
  last_command_.assign(6, 0.0);
  desired_command_.assign(6, 0.0);
  initial_effort_.assign(6, 0.0);
  entry_positions_.assign(6, 0.0);
  effective_gravity_scales_ = gravity_joint_scales_;
  gravity_command_.assign(6, 0.0);
  settle_base_effort_.assign(6, 0.0);
  settle_entry_command_.assign(6, 0.0);

  status_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::UInt8>(
          controller_nh, "status", 1));
  state_publisher_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
          controller_nh, "command_state", 1));
  state_publisher_->msg_.name = joint_names_;
  state_publisher_->msg_.position.resize(joint_names_.size());
  state_publisher_->msg_.velocity.resize(joint_names_.size());
  state_publisher_->msg_.effort.resize(joint_names_.size());
  telemetry_publisher_.reset(
      new realtime_tools::RealtimePublisher<FreedriveTelemetry>(
          controller_nh, "telemetry", 1));
  telemetry_publisher_->msg_.joint_names = joint_names_;
  telemetry_publisher_->msg_.position.resize(joint_names_.size());
  telemetry_publisher_->msg_.velocity.resize(joint_names_.size());
  telemetry_publisher_->msg_.measured_effort.resize(joint_names_.size());
  telemetry_publisher_->msg_.gravity_effort.resize(joint_names_.size());
  telemetry_publisher_->msg_.commanded_effort.resize(joint_names_.size());
  telemetry_publisher_->msg_.effective_gravity_scale.resize(
      joint_names_.size());
  settle_server_ = controller_nh.advertiseService(
      "request_settle", &ElfinFreedriveController::requestSettle, this);
  velocity_scale_server_ = controller_nh.advertiseService(
      "set_velocity_limit_scale",
      &ElfinFreedriveController::setVelocityScale, this);
  damping_scales_server_ = controller_nh.advertiseService(
      "set_damping_scales", &ElfinFreedriveController::setDampingScales,
      this);
  payload_model_server_ = controller_nh.advertiseService(
      "set_payload_model", &ElfinFreedriveController::setPayloadModel, this);

  ROS_WARN_STREAM("Loaded bounded Elfin gravity controller. Torque limits are "
                  << effort_limit_scale * 100.0
                  << "% of URDF effort limits. Hardware use remains locked "
                     "until the manager's explicit validation gate is enabled.");
  return true;
}

void ElfinFreedriveController::starting(const ros::Time& time) {
  running_.store(true);
  started_time_ = time;
  last_status_time_ = time;
  last_state_time_ = time;
  handoff_progress_ = 0.0;
  settle_active_ = false;
  settle_requested_.store(false);
  settle_started_time_ = time;
  model_alignment_ = 0.0;
  model_scale_estimate_ = 0.0;
  model_normalized_residual_ = 0.0;
  model_validation_warning_ = false;
  model_excited_joints_ = 0;
  model_direction_mismatches_ = 0;
  gravity_capacity_valid_ = true;

  bool valid_state = true;
  for (std::size_t chain_index = 0;
       chain_index < chain_to_joint_.size(); ++chain_index) {
    const hardware_interface::JointHandle& joint =
        joints_[chain_to_joint_[chain_index]];
    const double position = joint.getPosition();
    valid_state = valid_state && std::isfinite(position) &&
                  std::isfinite(joint.getVelocity()) &&
                  std::isfinite(joint.getEffort());
    q_chain_(chain_index) = position;
    entry_positions_[chain_to_joint_[chain_index]] = position;
  }

  if (valid_state &&
      dynamics_->JntToGravity(q_chain_, gravity_torque_chain_) >= 0 &&
      updatePayloadEffort()) {
    const double capacity_fraction = math::gravityCapacityFraction(
        payload_hold_verified_.load(), maximum_gravity_effort_fraction_,
        verified_payload_maximum_gravity_effort_fraction_);
    std::vector<double> configured_gravity(joints_.size(), 0.0);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      configured_gravity[i] =
          gravity_scale_ * gravity_joint_scales_[i] *
              gravity_torque_chain_(joint_to_chain_[i]) +
          gravity_bias_[i] + payload_effort_[joint_to_chain_[i]];
      if (!math::gravityEffortHasCapacity(
              configured_gravity[i], effort_limits_[i],
              capacity_fraction)) {
        gravity_capacity_valid_ = false;
      }
      initial_effort_[i] = math::clamp(
          joints_[i].getEffort(), -effort_limits_[i], effort_limits_[i]);
    }

    const math::GravityValidation validation =
        math::validateGravityObservation(configured_gravity, initial_effort_,
                                         minimum_validation_effort_);
    const math::EffortModelError entry_error =
        math::maximumAbsoluteEffortError(configured_gravity, initial_effort_);
    const bool controlled_hold_validation =
        controlled_hold_validation_.load();
    const bool payload_hold_verified = payload_hold_verified_.load();
    const bool entry_effort_matches =
        !require_model_validation_ || controlled_hold_validation ||
        payload_hold_verified ||
        (entry_error.valid &&
         entry_error.maximum_absolute_error <= maximum_entry_effort_error_);
    model_alignment_ = validation.alignment;
    model_scale_estimate_ = validation.scale_estimate;
    model_normalized_residual_ = validation.normalized_residual;
    model_excited_joints_ =
        static_cast<unsigned int>(validation.excited_joints);
    model_direction_mismatches_ =
        static_cast<unsigned int>(validation.direction_mismatches);
    const bool strict_model_validation_passed =
        gravity_capacity_valid_ && entry_effort_matches &&
        (!require_model_validation_ ||
         math::gravityValidationAccepted(
             validation, minimum_validation_joints_, minimum_model_alignment_,
             minimum_model_scale_, maximum_model_scale_,
             maximum_model_residual_));
    model_validation_warning_ =
        gravity_capacity_valid_ && require_model_validation_ &&
        entry_effort_matches && !strict_model_validation_passed &&
        allow_model_validation_warning_ && gravity_calibration_verified_ &&
        math::gravityValidationWarningAccepted(
            validation, minimum_warning_alignment_, minimum_model_scale_,
            maximum_model_scale_);
    model_validation_passed_ =
        strict_model_validation_passed || model_validation_warning_;

    effective_gravity_scales_ = gravity_joint_scales_;
    if (model_validation_passed_ && adaptive_entry_scale_ &&
        validation.sufficient_excitation) {
      const double adaptive_scale =
          math::clamp(validation.scale_estimate,
                      minimum_adaptive_scale_, maximum_adaptive_scale_);
      for (double& scale : effective_gravity_scales_) {
        scale *= adaptive_scale;
      }
    }

    if (model_validation_passed_) {
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        const double adapted_gravity =
            gravity_scale_ * effective_gravity_scales_[i] *
                gravity_torque_chain_(joint_to_chain_[i]) +
            gravity_bias_[i] + payload_effort_[joint_to_chain_[i]];
        if (!math::gravityEffortHasCapacity(
                adapted_gravity, effort_limits_[i],
                capacity_fraction)) {
          gravity_capacity_valid_ = false;
          model_validation_passed_ = false;
          model_validation_warning_ = false;
        }
      }
    }

    for (std::size_t i = 0; i < joints_.size(); ++i) {
      gravity_command_[i] = math::clamp(
          gravity_scale_ * effective_gravity_scales_[i] *
                  gravity_torque_chain_(joint_to_chain_[i]) +
              gravity_bias_[i] + payload_effort_[joint_to_chain_[i]],
          -effort_limits_[i], effort_limits_[i]);
      joints_[i].setCommand(initial_effort_[i]);
      last_command_[i] = initial_effort_[i];
    }
    command_initialized_ = true;
    if (!model_validation_passed_) {
      settle_requested_.store(true);
      const uint8_t rejected_status = gravity_capacity_valid_
                                          ? kStatusModelMismatch
                                          : kStatusGravityCapacity;
      if (gravity_capacity_valid_) {
        ROS_ERROR_STREAM(
            "Freedrive gravity preflight rejected the hardware model: alignment="
            << model_alignment_ << ", scale=" << model_scale_estimate_
            << ", residual=" << model_normalized_residual_
            << ", excited_joints=" << model_excited_joints_
            << ", direction_mismatches=" << model_direction_mismatches_
            << ", max_abs_effort_error="
            << entry_error.maximum_absolute_error << " Nm at J"
            << entry_error.joint + 1
            << ". Holding the measured entry torque and requesting a controlled exit.");
      } else {
        ROS_ERROR_STREAM(
            "Freedrive gravity preflight rejected the pose because the configured "
            "gravity effort reserve is insufficient. alignment="
            << model_alignment_ << ", scale=" << model_scale_estimate_
            << ", residual=" << model_normalized_residual_
            << ". Holding the measured entry torque and requesting a controlled exit.");
      }
      publishStatus(time, rejected_status);
      publishTelemetry(time, rejected_status);
    } else if (controlled_hold_validation &&
               entry_error.maximum_absolute_error >
                   maximum_entry_effort_error_) {
      ROS_WARN_STREAM(
          "Freedrive entered only for the <=1 s controlled calibration hold: "
          "the position-mode absolute effort mismatch is diagnostic ("
          << entry_error.maximum_absolute_error << " Nm at J"
          << entry_error.joint + 1
          << "). Capacity and gravity direction/scale checks still passed; "
             "the calibrator must abort on measured motion.");
      publishStatus(time, kStatusModelWarning);
      publishTelemetry(time, kStatusModelWarning);
    } else if (model_validation_warning_) {
      ROS_WARN_STREAM(
          "Freedrive gravity preflight entered with a calibrated-model warning: "
          "alignment=" << model_alignment_
          << ", entry scale=" << model_scale_estimate_
          << ", residual=" << model_normalized_residual_
          << ". Torque direction, scale bounds and gravity capacity passed; "
             "blending to the calibrated model over "
          << handoff_duration_ << " s.");
      publishStatus(time, kStatusModelWarning);
      publishTelemetry(time, kStatusModelWarning);
    } else {
      ROS_WARN_STREAM(
          "Freedrive gravity preflight passed: alignment=" << model_alignment_
          << ", entry scale=" << model_scale_estimate_
          << ", residual=" << model_normalized_residual_
          << ". Blending measured holding torque into the angle-dependent model over "
          << handoff_duration_ << " s.");
      publishStatus(time, kStatusActive);
      publishTelemetry(time, kStatusActive);
    }
  } else {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const double measured = joints_[i].getEffort();
      initial_effort_[i] = std::isfinite(measured)
                               ? math::clamp(measured, -effort_limits_[i],
                                             effort_limits_[i])
                               : 0.0;
      gravity_command_[i] = initial_effort_[i];
      joints_[i].setCommand(initial_effort_[i]);
      last_command_[i] = initial_effort_[i];
    }
    command_initialized_ = true;
    model_validation_passed_ = false;
    model_validation_warning_ = false;
    gravity_capacity_valid_ = false;
    settle_requested_.store(true);
    publishStatus(time, kStatusSolverError);
    publishTelemetry(time, kStatusSolverError);
  }
}

void ElfinFreedriveController::stopping(const ros::Time& time) {
  for (auto& joint : joints_) {
    joint.setCommand(0.0);
  }
  std::fill(last_command_.begin(), last_command_.end(), 0.0);
  command_initialized_ = false;
  settle_requested_.store(false);
  settle_active_ = false;
  handoff_progress_ = 0.0;
  publishStatus(time, kStatusInactive);
  publishTelemetry(time, kStatusInactive);
  running_.store(false);
}

void ElfinFreedriveController::publishStatus(
    const ros::Time& time, uint8_t status) {
  if (!status_publisher_ || !status_publisher_->trylock()) {
    return;
  }
  status_publisher_->msg_.data = status;
  status_publisher_->unlockAndPublish();
  last_status_time_ = time;
}

void ElfinFreedriveController::publishCommandState(const ros::Time& time) {
  if (!state_publisher_ || !state_publisher_->trylock()) {
    return;
  }
  state_publisher_->msg_.header.stamp = time;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    state_publisher_->msg_.position[i] = joints_[i].getPosition();
    state_publisher_->msg_.velocity[i] = joints_[i].getVelocity();
    state_publisher_->msg_.effort[i] = last_command_[i];
  }
  state_publisher_->unlockAndPublish();
  last_state_time_ = time;
}

void ElfinFreedriveController::publishTelemetry(
    const ros::Time& time, uint8_t status) {
  if (!telemetry_publisher_ || !telemetry_publisher_->trylock()) {
    return;
  }
  FreedriveTelemetry& message = telemetry_publisher_->msg_;
  message.header.stamp = time;
  message.status = status;
  message.handoff_progress = handoff_progress_;
  message.model_alignment = model_alignment_;
  message.model_scale_estimate = model_scale_estimate_;
  message.model_normalized_residual = model_normalized_residual_;
  message.model_excited_joints = model_excited_joints_;
  message.model_direction_mismatches = model_direction_mismatches_;
  message.model_validation_passed = model_validation_passed_;
  message.settling = settle_requested_.load();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    message.position[i] = joints_[i].getPosition();
    message.velocity[i] = joints_[i].getVelocity();
    message.measured_effort[i] = joints_[i].getEffort();
    message.gravity_effort[i] = gravity_command_[i];
    message.commanded_effort[i] = last_command_[i];
    message.effective_gravity_scale[i] = effective_gravity_scales_[i];
  }
  telemetry_publisher_->unlockAndPublish();
}

bool ElfinFreedriveController::requestSettle(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (!request.data) {
    response.success = false;
    response.message =
        "settle mode is one-way for the current activation; exit and re-enter to resume freedrive";
    return true;
  }
  settle_requested_.store(true);
  response.success = true;
  response.message = "high-damping settling requested";
  return true;
}

bool ElfinFreedriveController::setVelocityScale(
    elfin_robot_msgs::SetFloat64::Request& request,
    elfin_robot_msgs::SetFloat64::Response& response) {
  if (!math::velocityScaleValid(request.data,
                                minimum_velocity_limit_scale_,
                                maximum_velocity_limit_scale_)) {
    response.success = false;
    response.message = "velocity limit scale is outside the configured bounds";
    return true;
  }
  if (running_.load()) {
    response.success = false;
    response.message =
        "velocity limit scale cannot change while freedrive is running";
    return true;
  }
  velocity_limit_scale_.store(request.data);
  response.success = true;
  response.message = "velocity limit scale updated";
  return true;
}

bool ElfinFreedriveController::setDampingScales(
    SetDampingScales::Request& request,
    SetDampingScales::Response& response) {
  if (request.scales.size() != 6) {
    response.success = false;
    response.message = "damping scales must contain six values";
    return true;
  }
  std::array<double, 6> scales{};
  for (std::size_t i = 0; i < scales.size(); ++i) {
    if (!math::velocityScaleValid(request.scales[i],
                                  minimum_damping_scale_,
                                  maximum_damping_scale_)) {
      response.success = false;
      response.message = "a damping scale is outside the configured bounds";
      return true;
    }
    scales[i] = request.scales[i];
  }
  if (running_.load()) {
    response.success = false;
    response.message = "damping scales cannot change while freedrive is running";
    return true;
  }
  damping_scales_.writeFromNonRT(scales);
  response.applied_scales.assign(scales.begin(), scales.end());
  response.success = true;
  response.message = "damping scales updated";
  return true;
}

bool ElfinFreedriveController::setPayloadModel(
    SetPayloadModel::Request& request,
    SetPayloadModel::Response& response) {
  if (running_.load()) {
    response.success = false;
    response.message =
        "payload model cannot change while freedrive is running";
    return true;
  }
  payload::Model model;
  model.mass = request.mass;
  for (std::size_t axis = 0; axis < model.center_of_mass.size(); ++axis) {
    model.center_of_mass[axis] = request.center_of_mass[axis];
  }
  if (model.mass <= 1e-9) {
    model.mass = 0.0;
    model.center_of_mass.fill(0.0);
  }
  if (!payload::valid(model, maximum_payload_mass_)) {
    response.success = false;
    response.message =
        "payload mass/center of mass is invalid or mass is out of bounds";
    return true;
  }
  payload_model_.writeFromNonRT(model);
  controlled_hold_validation_.store(request.controlled_hold_validation);
  payload_hold_verified_.store(request.hold_verified);
  response.success = true;
  response.message = "payload model updated while controller is stopped";
  response.profile_file.clear();
  return true;
}

bool ElfinFreedriveController::updatePayloadEffort() {
  const payload::Model model = *payload_model_.readFromRT();
  if (!payload::buildRegressor(q_chain_, gravity_vector_,
                               *payload_jacobian_solver_, *payload_fk_solver_,
                               payload_jacobian_, payload_regressor_)) {
    return false;
  }
  payload_effort_ = payload::evaluate(payload_regressor_, model);
  return std::all_of(payload_effort_.begin(), payload_effort_.end(),
                     [](double effort) { return std::isfinite(effort); });
}

void ElfinFreedriveController::update(
    const ros::Time& time, const ros::Duration& period) {
  double dt = period.toSec();
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = 0.001;
  }
  dt = math::clamp(dt, 1e-4, 0.02);

  bool valid_state = true;
  for (std::size_t chain_index = 0;
       chain_index < chain_to_joint_.size(); ++chain_index) {
    const std::size_t joint_index = chain_to_joint_[chain_index];
    const double position = joints_[joint_index].getPosition();
    const double velocity = joints_[joint_index].getVelocity();
    valid_state = valid_state && std::isfinite(position) &&
                  std::isfinite(velocity);
    q_chain_(chain_index) = position;
  }

  if (!valid_state ||
      dynamics_->JntToGravity(q_chain_, gravity_torque_chain_) < 0 ||
      !updatePayloadEffort()) {
    settle_requested_.store(true);
    if (!settle_active_) {
      settle_active_ = true;
      settle_started_time_ = time;
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        settle_base_effort_[i] = command_initialized_
                                     ? last_command_[i]
                                     : initial_effort_[i];
        settle_entry_command_[i] = settle_base_effort_[i];
      }
    }
    const double settle_elapsed =
        std::max(0.0, (time - settle_started_time_).toSec());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const double velocity = joints_[i].getVelocity();
      double settling_target = settle_base_effort_[i];
      if (std::isfinite(velocity)) {
        settling_target -= settle_damping_[i] * velocity;
      }
      double command = math::settlingHandoffCommand(
          settle_entry_command_[i], settling_target, settle_elapsed,
          settle_handoff_duration_);
      command = math::clamp(command, -effort_limits_[i], effort_limits_[i]);
      command = math::rateLimit(
          command, last_command_[i], torque_rate_limits_[i], dt);
      joints_[i].setCommand(command);
      last_command_[i] = command;
    }
    if ((time - last_state_time_).toSec() >= 0.02) {
      publishCommandState(time);
      publishTelemetry(time, kStatusSolverError);
    }
    publishStatus(time, kStatusSolverError);
    return;
  }

  handoff_progress_ = math::smoothStep(
      std::max(0.0, (time - started_time_).toSec()) / handoff_duration_);

  const double capacity_fraction = math::gravityCapacityFraction(
      payload_hold_verified_.load(), maximum_gravity_effort_fraction_,
      verified_payload_maximum_gravity_effort_fraction_);
  bool gravity_capacity_exceeded = false;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double requested_gravity =
        gravity_scale_ * effective_gravity_scales_[i] *
            gravity_torque_chain_(joint_to_chain_[i]) +
        gravity_bias_[i] + payload_effort_[joint_to_chain_[i]];
    if (!math::gravityEffortHasCapacity(
            requested_gravity, effort_limits_[i],
            capacity_fraction)) {
      gravity_capacity_exceeded = true;
    }
    gravity_command_[i] = math::clamp(
        requested_gravity, -effort_limits_[i], effort_limits_[i]);
  }

  bool limit_active = false;
  bool hard_limit_safety_stop = false;
  bool velocity_safety_stop = false;
  bool safety_stop = gravity_capacity_exceeded;
  const bool transition_protection =
      handoff_progress_ < 1.0 || settle_requested_.load() || settle_active_;
  const double velocity_limit_scale = math::transitionVelocityScale(
      velocity_limit_scale_.load(), transition_protection);
  const std::array<double, 6> damping_scales = *damping_scales_.readFromRT();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double hard_velocity_limit = std::min(
        velocity_hard_limits_[i] * velocity_limit_scale,
        velocity_limits_[i]);
    if (std::abs(joints_[i].getVelocity()) > hard_velocity_limit) {
      velocity_safety_stop = true;
      safety_stop = true;
    }
  }
  if (safety_stop) {
    settle_requested_.store(true);
  }
  const bool settling = settle_requested_.load();
  if (settling && !settle_active_) {
    settle_active_ = true;
    settle_started_time_ = time;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      settle_base_effort_[i] =
          model_validation_passed_
              ? math::interpolate(initial_effort_[i], gravity_command_[i],
                                  handoff_progress_)
              : initial_effort_[i];
      settle_entry_command_[i] = last_command_[i];
    }
  }
  const double settle_elapsed =
      settle_active_
          ? std::max(0.0, (time - settle_started_time_).toSec())
          : 0.0;

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double position = joints_[i].getPosition();
    const double velocity = joints_[i].getVelocity();
    const bool hard_limit_stop = math::hardLimitStopRequired(
        position, velocity, entry_positions_[i], lower_limits_[i],
        upper_limits_[i], hard_limit_margin_,
        hard_limit_toward_velocity_, hard_limit_inward_travel_);
    const double base_effort = settling
                                   ? settle_base_effort_[i]
                                   : (model_validation_passed_
                                          ? math::interpolate(
                                                initial_effort_[i],
                                                gravity_command_[i],
                                                handoff_progress_)
                                          : initial_effort_[i]);
    const double damping = settling ? settle_damping_[i]
                                    : damping_[i] * damping_scales[i];
    const double damped_target = base_effort - damping * velocity;
    double command =
        settling
            ? math::settlingHandoffCommand(
                  settle_entry_command_[i], damped_target, settle_elapsed,
                  settle_handoff_duration_)
            : damped_target;
    if (!settling) {
      command -= math::smoothFriction(
          friction_[i], velocity, friction_velocity_[i]);
    }

    const double hard_velocity_limit = std::min(
        velocity_hard_limits_[i] * velocity_limit_scale,
        velocity_limits_[i]);
    const double soft_velocity_limit = std::min(
        velocity_soft_limits_[i] * velocity_limit_scale,
        0.8 * hard_velocity_limit);
    if (std::abs(velocity) > soft_velocity_limit) {
      command -= velocity_limit_damping_ *
                 (std::abs(velocity) - soft_velocity_limit) *
                 math::sign(velocity);
      limit_active = true;
    }
    if (std::abs(velocity) > hard_velocity_limit) {
      command = base_effort - hard_stop_damping_ * velocity;
      safety_stop = true;
    }

    if (std::isfinite(lower_limits_[i]) &&
        position < lower_limits_[i] + limit_margin_) {
      command += limit_stiffness_ *
                 (lower_limits_[i] + limit_margin_ - position);
      command -= limit_damping_ * velocity;
      if (command < 0.0) {
        command = 0.0;
      }
      limit_active = true;
      if (hard_limit_stop) {
        command = effort_limits_[i];
        hard_limit_safety_stop = true;
        safety_stop = true;
        settle_requested_.store(true);
      }
    }
    if (std::isfinite(upper_limits_[i]) &&
        position > upper_limits_[i] - limit_margin_) {
      command -= limit_stiffness_ *
                 (position - (upper_limits_[i] - limit_margin_));
      command -= limit_damping_ * velocity;
      if (command > 0.0) {
        command = 0.0;
      }
      limit_active = true;
      if (hard_limit_stop) {
        command = -effort_limits_[i];
        hard_limit_safety_stop = true;
        safety_stop = true;
        settle_requested_.store(true);
      }
    }

    command = math::clamp(command, -effort_limits_[i], effort_limits_[i]);
    if (command_initialized_) {
      command = math::rateLimit(
          command, last_command_[i], torque_rate_limits_[i], dt);
    }
    desired_command_[i] = command;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].setCommand(desired_command_[i]);
    last_command_[i] = desired_command_[i];
  }
  command_initialized_ = true;

  const uint8_t control_status =
      (!gravity_capacity_valid_ || gravity_capacity_exceeded)
          ? kStatusGravityCapacity
          : (!model_validation_passed_
                 ? kStatusModelMismatch
                 : (hard_limit_safety_stop
                        ? kStatusHardLimit
                        : (velocity_safety_stop
                               ? kStatusOverspeed
                               : (safety_stop
                                      ? kStatusSafetyStop
                                      : (settling
                                             ? kStatusSettling
                                             : (limit_active
                                                    ? kStatusLimit
                                                    : (model_validation_warning_
                                                           ? kStatusModelWarning
                                                           : kStatusActive)))))));
  if ((time - last_state_time_).toSec() >= 0.02) {
    publishCommandState(time);
    publishTelemetry(time, control_status);
  }
  if (!model_validation_passed_ || safety_stop ||
      (time - last_status_time_).toSec() >= 0.1) {
    publishStatus(time, control_status);
  }
}

}  // namespace elfin_freedrive_controller

PLUGINLIB_EXPORT_CLASS(elfin_freedrive_controller::ElfinFreedriveController,
                       controller_interface::ControllerBase)
