#include <elfin_freedrive_controller/elfin_freedrive_controller.h>
#include <elfin_freedrive_controller/DeleteRecordedPoint.h>
#include <elfin_freedrive_controller/EvaluatePayloadModel.h>
#include <elfin_freedrive_controller/FreedriveTelemetry.h>
#include <elfin_freedrive_controller/GetPayloadModel.h>
#include <elfin_freedrive_controller/ListRecordedPoints.h>
#include <elfin_freedrive_controller/SetDampingScales.h>
#include <elfin_freedrive_controller/SetPayloadModel.h>
#include <elfin_freedrive_controller/freedrive_math.h>
#include <elfin_freedrive_controller/payload_model.h>
#include <elfin_freedrive_controller/tool_button_logic.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <elfin_robot_msgs/ElfinIODRead.h>
#include <elfin_robot_msgs/SetFloat64.h>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

namespace elfin_freedrive_controller {

namespace {
constexpr std::size_t kJointCount = 6;

bool finite(double value) {
  return std::isfinite(value);
}

double steadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string defaultRosDataPath(const std::string& filename) {
  const char* ros_home = std::getenv("ROS_HOME");
  if (ros_home != nullptr && ros_home[0] != '\0') {
    return std::string(ros_home) + "/" + filename;
  }
  const char* home = std::getenv("HOME");
  if (home != nullptr && home[0] != '\0') {
    return std::string(home) + "/.ros/" + filename;
  }
  return std::string("/tmp/") + filename;
}

std::string trimAsciiWhitespace(const std::string& value) {
  const std::size_t first = value.find_first_not_of(" \t\r");
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = value.find_last_not_of(" \t\r");
  return value.substr(first, last - first + 1);
}

bool removeLegacyPointNullPadding(const std::string& input,
                                  std::string& normalized) {
  normalized.clear();
  std::size_t cursor = 0;
  bool removed = false;
  while (cursor < input.size()) {
    const std::size_t null_begin = input.find('\0', cursor);
    if (null_begin == std::string::npos) {
      normalized.append(input, cursor, std::string::npos);
      break;
    }
    std::size_t null_end = null_begin;
    while (null_end < input.size() && input[null_end] == '\0') {
      ++null_end;
    }
    const bool starts_at_line_boundary =
        null_begin == 0 || input[null_begin - 1] == '\n';
    const bool precedes_point_record =
        input.compare(null_end, 8, "- index:") == 0;
    if (!starts_at_line_boundary || !precedes_point_record) {
      normalized.clear();
      return false;
    }
    normalized.append(input, cursor, null_begin - cursor);
    cursor = null_end;
    removed = true;
  }
  return removed;
}

bool normalizeLegacyPointYaml(const std::string& input,
                              std::string& normalized) {
  std::string without_nulls;
  const bool removed_nulls =
      removeLegacyPointNullPadding(input, without_nulls);
  const std::string& source = removed_nulls ? without_nulls : input;
  std::istringstream lines(source);
  std::ostringstream output;
  std::string line;
  bool removed_empty_root = false;
  bool saw_point_record = false;
  while (std::getline(lines, line)) {
    const std::string trimmed = trimAsciiWhitespace(line);
    if (!removed_empty_root && !saw_point_record && trimmed == "[]") {
      removed_empty_root = true;
      continue;
    }
    if (trimmed.compare(0, 8, "- index:") == 0) {
      saw_point_record = true;
    }
    output << line << "\n";
  }
  if (removed_empty_root && !saw_point_record) {
    return false;
  }
  normalized = output.str();
  return removed_nulls || removed_empty_root;
}
}  // namespace

class ElfinFreedriveManager {
public:
  ElfinFreedriveManager()
      : private_nh_("~"),
        simulation_(false),
        allow_hardware_freedrive_(false),
        poll_tool_buttons_(true),
        free_press_hold_seconds_(ToolButtonLogic::defaultFreePressSeconds()),
        joint_state_timeout_(0.35),
        driver_state_timeout_(0.75),
        controller_status_timeout_(1.0),
        entry_velocity_limit_(0.02),
        velocity_limit_scale_(1.0),
        minimum_velocity_limit_scale_(0.50),
        maximum_velocity_limit_scale_(3.0),
        exit_velocity_limit_(0.003),
        exit_settle_timeout_(8.0),
        protective_exit_settle_timeout_(1.0),
        exit_stable_samples_required_(10),
        preflight_min_samples_(12),
        preflight_min_duration_(0.25),
        preflight_window_(0.75),
        preflight_velocity_limit_(0.003),
        preflight_position_tolerance_(0.003),
        preflight_max_effort_stddev_(2.0),
        preflight_max_model_error_(5.0),
        allow_model_validation_warning_(true),
        minimum_warning_alignment_(0.50),
        minimum_damping_scale_(0.05),
        maximum_damping_scale_(5.0),
        calibration_min_samples_(6),
        calibration_min_paired_poses_(5),
        calibration_min_pose_separation_(0.08),
        calibration_min_model_change_(1.0),
        calibration_min_approach_delta_(0.05),
        calibration_pair_pose_tolerance_(0.02),
        calibration_min_model_range_(4.0),
        calibration_min_scale_(0.50),
        calibration_max_scale_(4.0),
        calibration_max_normalized_residual_(0.15),
        calibration_max_absolute_residual_(5.0),
        gravity_model_ready_(false),
        gravity_calibration_verified_(false),
        maximum_payload_mass_(5.0),
        payload_fit_rmse_(0.0),
        payload_fit_max_error_(0.0),
        payload_validation_drift_(0.0),
        payload_sample_count_(0),
        payload_model_synchronized_(false),
        controlled_hold_validation_(false),
        payload_hold_verified_(false),
        incident_lockout_latched_(false),
        model_maximum_gravity_effort_fraction_(0.90),
        model_adaptive_entry_scale_(false),
        model_minimum_adaptive_scale_(0.75),
        model_maximum_adaptive_scale_(1.25),
        joint_state_valid_(false),
        effort_state_valid_(false),
        servo_received_(false),
        servo_enabled_(false),
        fault_received_(false),
        faulted_(false),
        telemetry_valid_(false),
        controller_status_received_(false),
        controller_status_(kStatusInactive),
        freedrive_active_(false),
        exit_requested_(false),
        exit_protective_(false),
        transition_busy_(false),
        position_recovery_pending_(false),
        physical_free_entry_pending_(false),
        last_physical_free_entry_attempt_seconds_(0.0),
        idle_initialized_(false),
        stable_exit_samples_(0),
        trial_log_rows_(0),
        point_count_(0),
        state_("STARTING"),
        state_detail_("等待关节、驱动和控制器状态") {
    expected_joint_names_ = {
        "elfin_joint1", "elfin_joint2", "elfin_joint3",
        "elfin_joint4", "elfin_joint5", "elfin_joint6"};
    velocity_hard_limits_ = {0.20, 0.20, 0.20, 0.30, 0.40, 0.40};
    damping_scales_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    positions_.fill(0.0);
    velocities_.fill(0.0);
    efforts_.fill(0.0);

    private_nh_.param("simulation", simulation_, false);
    private_nh_.param("allow_hardware_freedrive",
                      allow_hardware_freedrive_, false);
    private_nh_.param("poll_tool_buttons", poll_tool_buttons_, true);
    private_nh_.param("free_press_hold_seconds", free_press_hold_seconds_,
                      ToolButtonLogic::defaultFreePressSeconds());
    private_nh_.param("joint_state_timeout", joint_state_timeout_, 0.35);
    private_nh_.param("driver_state_timeout", driver_state_timeout_, 0.75);
    private_nh_.param("controller_status_timeout",
                      controller_status_timeout_, 1.0);
    private_nh_.param("entry_velocity_limit", entry_velocity_limit_, 0.02);
    private_nh_.param("velocity_limit_scale", velocity_limit_scale_, 1.0);
    private_nh_.param("minimum_velocity_limit_scale",
                      minimum_velocity_limit_scale_, 0.50);
    private_nh_.param("maximum_velocity_limit_scale",
                      maximum_velocity_limit_scale_, 3.0);
    private_nh_.param("exit_velocity_limit", exit_velocity_limit_, 0.003);
    private_nh_.param("exit_settle_timeout", exit_settle_timeout_, 8.0);
    private_nh_.param("protective_exit_settle_timeout",
                      protective_exit_settle_timeout_, 1.0);
    private_nh_.param("exit_stable_samples_required",
                      exit_stable_samples_required_, 10);
    int preflight_min_samples = 12;
    private_nh_.param("preflight_min_samples", preflight_min_samples, 12);
    private_nh_.param("preflight_min_duration",
                      preflight_min_duration_, 0.25);
    private_nh_.param("preflight_window", preflight_window_, 0.75);
    private_nh_.param("preflight_velocity_limit",
                      preflight_velocity_limit_, 0.003);
    private_nh_.param("preflight_position_tolerance",
                      preflight_position_tolerance_, 0.003);
    private_nh_.param("preflight_max_effort_stddev",
                      preflight_max_effort_stddev_, 2.0);
    private_nh_.param("preflight_max_model_error",
                      preflight_max_model_error_, 5.0);
    private_nh_.param("allow_model_validation_warning",
                      allow_model_validation_warning_, true);
    private_nh_.param("minimum_warning_alignment",
                      minimum_warning_alignment_, 0.50);
    private_nh_.param("minimum_damping_scale", minimum_damping_scale_, 0.05);
    private_nh_.param("maximum_damping_scale", maximum_damping_scale_, 5.0);
    private_nh_.param<std::string>("position_controller",
                                   position_controller_,
                                   "elfin_arm_controller");
    private_nh_.param<std::string>("freedrive_controller",
                                   freedrive_controller_,
                                   "elfin_freedrive_controller");
    private_nh_.param<std::string>("record_file", record_file_,
                                   defaultRosDataPath(
                                       "elfin_freedrive_points.yaml"));
    private_nh_.param<std::string>(
        "trial_log_directory", trial_log_directory_,
        defaultRosDataPath("elfin_freedrive_trials"));
    private_nh_.param<std::string>(
        "gravity_calibration_samples_file", calibration_samples_file_,
        defaultRosDataPath("elfin_freedrive_gravity_samples.csv"));
    private_nh_.param<std::string>(
        "gravity_calibration_candidate_file", calibration_candidate_file_,
        defaultRosDataPath("elfin_freedrive_gravity_candidate.yaml"));
    private_nh_.param<std::string>(
        "payload_profile_file", payload_profile_file_,
        defaultRosDataPath("elfin_freedrive_payload.yaml"));
    private_nh_.param<std::string>(
        "freedrive_lockout_file", freedrive_lockout_file_,
        defaultRosDataPath("ELFIN_FREEDRIVE_LOCKOUT"));
    private_nh_.param("maximum_payload_mass", maximum_payload_mass_, 5.0);
    int calibration_min_samples = 6;
    private_nh_.param("calibration_min_samples", calibration_min_samples, 6);
    calibration_min_samples_ = static_cast<unsigned int>(
        std::max(3, calibration_min_samples));
    int calibration_min_paired_poses = 5;
    private_nh_.param("calibration_min_paired_poses",
                      calibration_min_paired_poses, 5);
    calibration_min_paired_poses_ = static_cast<unsigned int>(
        std::max(3, calibration_min_paired_poses));
    private_nh_.param("calibration_min_pose_separation",
                      calibration_min_pose_separation_, 0.08);
    private_nh_.param("calibration_min_model_change",
                      calibration_min_model_change_, 1.0);
    private_nh_.param("calibration_min_approach_delta",
                      calibration_min_approach_delta_, 0.05);
    private_nh_.param("calibration_pair_pose_tolerance",
                      calibration_pair_pose_tolerance_, 0.02);
    private_nh_.param("calibration_min_model_range",
                      calibration_min_model_range_, 4.0);
    private_nh_.param("calibration_min_scale", calibration_min_scale_, 0.50);
    private_nh_.param("calibration_max_scale", calibration_max_scale_, 4.0);
    private_nh_.param("calibration_max_normalized_residual",
                      calibration_max_normalized_residual_, 0.15);
    private_nh_.param("calibration_max_absolute_residual",
                      calibration_max_absolute_residual_, 5.0);
    private_nh_.getParam("joints", expected_joint_names_);
    private_nh_.getParam("velocity_hard_limits", velocity_hard_limits_);
    private_nh_.getParam("damping_scales", damping_scales_);
    preflight_min_samples_ =
        static_cast<unsigned int>(std::max(1, preflight_min_samples));
    button_logic_ = ToolButtonLogic(free_press_hold_seconds_);
    validateParameters();
    initializeGravityModel();
    loadPayloadProfile();
    loadGravityCalibrationSamples();
    refreshIncidentLockout();
    point_count_ = countExistingPoints();

    switch_client_ = root_nh_.serviceClient<
        controller_manager_msgs::SwitchController>(
        "/controller_manager/switch_controller");
    list_client_ = root_nh_.serviceClient<
        controller_manager_msgs::ListControllers>(
        "/controller_manager/list_controllers");
    load_client_ = root_nh_.serviceClient<
        controller_manager_msgs::LoadController>(
        "/controller_manager/load_controller");
    read_di_client_ = root_nh_.serviceClient<elfin_robot_msgs::ElfinIODRead>(
        "/elfin_ros_control/elfin/io_port1/read_di");
    raw_disable_client_ = root_nh_.serviceClient<std_srvs::SetBool>(
        "/elfin_ros_control/elfin/disable_robot");
    settle_client_ = root_nh_.serviceClient<std_srvs::SetBool>(
        "/elfin_freedrive_controller/request_settle");
    velocity_scale_client_ = root_nh_.serviceClient<
        elfin_robot_msgs::SetFloat64>(
        "/elfin_freedrive_controller/set_velocity_limit_scale");
    damping_scales_client_ = root_nh_.serviceClient<SetDampingScales>(
        "/elfin_freedrive_controller/set_damping_scales");
    payload_model_client_ = root_nh_.serviceClient<SetPayloadModel>(
        "/elfin_freedrive_controller/set_payload_model");

    joint_state_sub_ = root_nh_.subscribe(
        "/joint_states", 5, &ElfinFreedriveManager::jointStateCallback, this);
    servo_sub_ = root_nh_.subscribe(
        "/elfin_ros_control/elfin/enable_state", 5,
        &ElfinFreedriveManager::servoCallback, this);
    fault_sub_ = root_nh_.subscribe(
        "/elfin_ros_control/elfin/fault_state", 5,
        &ElfinFreedriveManager::faultCallback, this);
    controller_status_sub_ = root_nh_.subscribe(
        "/elfin_freedrive_controller/status", 5,
        &ElfinFreedriveManager::controllerStatusCallback, this);
    telemetry_sub_ = root_nh_.subscribe(
        "/elfin_freedrive_controller/telemetry", 5,
        &ElfinFreedriveManager::telemetryCallback, this);

    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
    state_detail_pub_ =
        private_nh_.advertise<std_msgs::String>("state_detail", 1, true);
    ring_state_pub_ =
        private_nh_.advertise<std_msgs::String>("ring_state", 1, true);
    ring_color_pub_ =
        private_nh_.advertise<std_msgs::ColorRGBA>("ring_color", 1, true);
    active_pub_ = private_nh_.advertise<std_msgs::Bool>("active", 1, true);
    raw_di_pub_ = private_nh_.advertise<std_msgs::UInt16>("raw_di", 1, true);
    button_online_pub_ =
        private_nh_.advertise<std_msgs::Bool>("button_io_online", 1, true);
    recorded_point_pub_ =
        private_nh_.advertise<sensor_msgs::JointState>(
            "recorded_point", 1, true);
    point_count_pub_ =
        private_nh_.advertise<std_msgs::UInt32>("point_count", 1, true);
    validation_pub_ =
        private_nh_.advertise<std_msgs::String>("model_validation", 1, true);
    trial_log_pub_ =
        private_nh_.advertise<std_msgs::String>("trial_log_path", 1, true);
    velocity_scale_pub_ =
        private_nh_.advertise<std_msgs::Float64>(
            "velocity_limit_scale", 1, true);
    velocity_hard_limits_pub_ =
        private_nh_.advertise<std_msgs::Float64MultiArray>(
            "velocity_hard_limits", 1, true);
    damping_scales_pub_ =
        private_nh_.advertise<std_msgs::Float64MultiArray>(
            "damping_scales", 1, true);
    payload_profile_pub_ =
        private_nh_.advertise<std_msgs::String>("payload_profile", 1, true);
    payload_model_pub_ =
        private_nh_.advertise<std_msgs::Float64MultiArray>(
            "payload_model", 1, true);

    set_freedrive_server_ = private_nh_.advertiseService(
        "set_freedrive", &ElfinFreedriveManager::setFreedriveCallback, this);
    record_point_server_ = private_nh_.advertiseService(
        "record_point", &ElfinFreedriveManager::recordPointCallback, this);
    list_recorded_points_server_ = private_nh_.advertiseService(
        "list_recorded_points",
        &ElfinFreedriveManager::listRecordedPointsCallback, this);
    delete_recorded_point_server_ = private_nh_.advertiseService(
        "delete_recorded_point",
        &ElfinFreedriveManager::deleteRecordedPointCallback, this);
    record_gravity_sample_server_ = private_nh_.advertiseService(
        "record_gravity_sample",
        &ElfinFreedriveManager::recordGravitySampleCallback, this);
    fit_gravity_calibration_server_ = private_nh_.advertiseService(
        "fit_gravity_calibration",
        &ElfinFreedriveManager::fitGravityCalibrationCallback, this);
    set_velocity_scale_server_ = private_nh_.advertiseService(
        "set_velocity_limit_scale",
        &ElfinFreedriveManager::setVelocityScaleCallback, this);
    set_damping_scales_server_ = private_nh_.advertiseService(
        "set_damping_scales",
        &ElfinFreedriveManager::setDampingScalesCallback, this);
    set_payload_model_server_ = private_nh_.advertiseService(
        "set_payload_model", &ElfinFreedriveManager::setPayloadModelCallback,
        this);
    get_payload_model_server_ = private_nh_.advertiseService(
        "get_payload_model", &ElfinFreedriveManager::getPayloadModelCallback,
        this);
    evaluate_payload_model_server_ = private_nh_.advertiseService(
        "evaluate_payload_model",
        &ElfinFreedriveManager::evaluatePayloadModelCallback, this);

    button_timer_ = root_nh_.createWallTimer(
        ros::WallDuration(0.10),
        &ElfinFreedriveManager::buttonTimerCallback, this);
    monitor_timer_ = root_nh_.createWallTimer(
        ros::WallDuration(0.05),
        &ElfinFreedriveManager::monitorTimerCallback, this);

    std_msgs::Bool button_online;
    button_online.data = false;
    button_online_pub_.publish(button_online);
    std_msgs::String empty_log_path;
    trial_log_pub_.publish(empty_log_path);
    publishValidation("等待位置控制下的静止力矩样本；尚未切入力矩模式");
    publishPointCount();
    publishVelocityLimits();
    publishDampingScales();
    publishPayloadModel();
    synchronizePayloadModel();
    publishState();
    ROS_WARN_STREAM(
        "Elfin freedrive manager started. Physical FREE is DI bit 5 and "
        "POINT is DI bit 4. FREE requires at least "
        << std::fixed << std::setprecision(2)
        << button_logic_.requiredFreePressSeconds() << " s and "
        << button_logic_.requiredFreePressSamples()
        << " high samples; isolated low gaps up to "
        << button_logic_.maximumFreeLowGapSeconds()
        << " s are filtered. Hardware torque mode is "
        << ((simulation_ ||
             (allow_hardware_freedrive_ && !incident_lockout_latched_))
                ? "UNLOCKED"
                : "LOCKED"));
  }

private:
  struct ControllerSnapshot {
    bool position_loaded = false;
    bool position_running = false;
    bool freedrive_loaded = false;
    bool freedrive_running = false;
  };

  struct StaticEffortSample {
    ros::WallTime stamp;
    std::array<double, kJointCount> position;
    std::array<double, kJointCount> effort;
  };

  struct GravityCalibrationSample {
    double stamp = 0.0;
    std::array<double, kJointCount> position{};
    std::array<double, kJointCount> measured_effort{};
    std::array<double, kJointCount> base_model_effort{};
    std::array<double, kJointCount> effort_stddev{};
  };

  void validateParameters() {
    if (expected_joint_names_.size() != kJointCount ||
        velocity_hard_limits_.size() != kJointCount ||
        damping_scales_.size() != kJointCount ||
        !finite(free_press_hold_seconds_) ||
        free_press_hold_seconds_ < ToolButtonLogic::minimumFreePressSeconds() ||
        free_press_hold_seconds_ > 10.0 ||
        !finite(joint_state_timeout_) || joint_state_timeout_ <= 0.0 ||
        !finite(driver_state_timeout_) || driver_state_timeout_ <= 0.0 ||
        !finite(controller_status_timeout_) ||
        controller_status_timeout_ <= 0.0 ||
        !finite(entry_velocity_limit_) || entry_velocity_limit_ <= 0.0 ||
        !math::velocityScaleValid(velocity_limit_scale_,
                                  minimum_velocity_limit_scale_,
                                  maximum_velocity_limit_scale_) ||
        !finite(exit_velocity_limit_) || exit_velocity_limit_ <= 0.0 ||
        !finite(exit_settle_timeout_) || exit_settle_timeout_ <= 0.0 ||
        !finite(protective_exit_settle_timeout_) ||
        protective_exit_settle_timeout_ <= 0.0 ||
        protective_exit_settle_timeout_ > exit_settle_timeout_ ||
        exit_stable_samples_required_ < 3 ||
        exit_stable_samples_required_ > 200 ||
        preflight_min_samples_ < 3 || preflight_min_samples_ > 500 ||
        !finite(preflight_min_duration_) || preflight_min_duration_ <= 0.0 ||
        !finite(preflight_window_) ||
        preflight_window_ < preflight_min_duration_ ||
        !finite(preflight_velocity_limit_) ||
        preflight_velocity_limit_ <= 0.0 ||
        !finite(preflight_position_tolerance_) ||
        preflight_position_tolerance_ <= 0.0 ||
        !finite(preflight_max_effort_stddev_) ||
        preflight_max_effort_stddev_ <= 0.0 ||
        !finite(preflight_max_model_error_) ||
        preflight_max_model_error_ <= 0.0 ||
        !finite(minimum_warning_alignment_) ||
        minimum_warning_alignment_ < 0.0 ||
        minimum_warning_alignment_ > 1.0 ||
        !math::velocityScaleValid(1.0, minimum_damping_scale_,
                                  maximum_damping_scale_) ||
        calibration_min_samples_ < 3 || calibration_min_samples_ > 100 ||
        calibration_min_paired_poses_ < 3 ||
        calibration_min_paired_poses_ > 50 ||
        !finite(calibration_min_pose_separation_) ||
        calibration_min_pose_separation_ <= 0.0 ||
        !finite(calibration_min_model_change_) ||
        calibration_min_model_change_ <= 0.0 ||
        !finite(calibration_min_approach_delta_) ||
        calibration_min_approach_delta_ <= 0.0 ||
        !finite(calibration_pair_pose_tolerance_) ||
        calibration_pair_pose_tolerance_ <= 0.0 ||
        !finite(calibration_min_model_range_) ||
        calibration_min_model_range_ <= 0.0 ||
        !finite(calibration_min_scale_) || calibration_min_scale_ <= 0.0 ||
        !finite(calibration_max_scale_) ||
        calibration_max_scale_ <= calibration_min_scale_ ||
        !finite(calibration_max_normalized_residual_) ||
        calibration_max_normalized_residual_ < 0.0 ||
        !finite(calibration_max_absolute_residual_) ||
        calibration_max_absolute_residual_ <= 0.0 ||
        !finite(maximum_payload_mass_) || maximum_payload_mass_ <= 0.0 ||
        calibration_samples_file_.empty() ||
        calibration_candidate_file_.empty() || payload_profile_file_.empty() ||
        freedrive_lockout_file_.empty()) {
      ROS_FATAL("Invalid elfin_freedrive_manager parameter set");
      throw std::runtime_error("invalid freedrive manager parameters");
    }
    for (double value : velocity_hard_limits_) {
      if (!finite(value) || value <= 0.0) {
        ROS_FATAL("velocity_hard_limits must contain six positive values");
        throw std::runtime_error("invalid velocity hard limits");
      }
    }
    for (double value : damping_scales_) {
      if (!math::velocityScaleValid(value, minimum_damping_scale_,
                                    maximum_damping_scale_)) {
        ROS_FATAL("damping_scales contains a value outside configured bounds");
        throw std::runtime_error("invalid damping scales");
      }
    }
  }

  bool lockoutFilePresent() const {
    struct stat file_info;
    return !simulation_ &&
           stat(freedrive_lockout_file_.c_str(), &file_info) == 0;
  }

  bool refreshIncidentLockout() {
    if (lockoutFilePresent()) {
      incident_lockout_latched_ = true;
    }
    return incident_lockout_latched_;
  }

  std::string incidentLockoutDetail() const {
    std::ifstream input(freedrive_lockout_file_);
    std::string first_line;
    std::getline(input, first_line);
    std::string detail =
        "真机 FREE 已因 2026-07-26 下坠事故锁定；仅允许位置模式";
    if (!first_line.empty()) {
      detail += "（" + first_line + "）";
    }
    detail += "。修复、离线测试和监督复验完成前不得删除 " +
              freedrive_lockout_file_;
    return detail;
  }

  void initializeGravityModel() {
    gravity_model_ready_ = false;
    gravity_model_error_.clear();

    std::string controller_namespace = freedrive_controller_;
    if (controller_namespace.empty()) {
      gravity_model_error_ = "零力控制器名称为空";
      return;
    }
    if (controller_namespace.front() != '/') {
      controller_namespace.insert(controller_namespace.begin(), '/');
    }
    ros::NodeHandle controller_nh(controller_namespace);

    std::string robot_description;
    urdf::Model model;
    if (!root_nh_.getParam("robot_description", robot_description) ||
        !model.initString(robot_description)) {
      gravity_model_error_ = "无法解析 robot_description";
      return;
    }

    std::string root_link;
    std::string tip_link;
    controller_nh.param<std::string>("root_link", root_link, "elfin_base");
    controller_nh.param<std::string>("tip_link", tip_link,
                                     "elfin_end_link");
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree) ||
        !tree.getChain(root_link, tip_link, gravity_chain_)) {
      gravity_model_error_ = "无法建立 " + root_link + " -> " + tip_link +
                             " 的 KDL 重力链";
      return;
    }
    if (gravity_chain_.getNrOfJoints() != kJointCount) {
      std::ostringstream error;
      error << "KDL 重力链关节数为 " << gravity_chain_.getNrOfJoints()
            << "，不是 6";
      gravity_model_error_ = error.str();
      return;
    }

    std::vector<std::string> chain_joint_names;
    for (unsigned int segment = 0;
         segment < gravity_chain_.getNrOfSegments(); ++segment) {
      const KDL::Joint& joint = gravity_chain_.getSegment(segment).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        chain_joint_names.push_back(joint.getName());
      }
    }
    gravity_joint_to_chain_.assign(kJointCount, kJointCount);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto found = std::find(chain_joint_names.begin(),
                                   chain_joint_names.end(),
                                   expected_joint_names_[joint]);
      if (found == chain_joint_names.end()) {
        gravity_model_error_ = "KDL 重力链缺少 " +
                               expected_joint_names_[joint];
        return;
      }
      gravity_joint_to_chain_[joint] = static_cast<std::size_t>(
          std::distance(chain_joint_names.begin(), found));
    }

    std::vector<double> gravity{0.0, 0.0, -9.81};
    controller_nh.getParam("gravity", gravity);
    gravity_joint_scales_.assign(kJointCount, 1.0);
    gravity_bias_.assign(kJointCount, 0.0);
    controller_nh.getParam("gravity_joint_scales", gravity_joint_scales_);
    controller_nh.getParam("gravity_bias", gravity_bias_);
    controller_nh.param("gravity_scale", model_gravity_scale_, 1.0);
    controller_nh.param("minimum_validation_effort",
                        model_minimum_validation_effort_, 1.5);
    int minimum_validation_joints = 2;
    controller_nh.param("minimum_validation_joints",
                        minimum_validation_joints, 2);
    controller_nh.param("minimum_model_alignment",
                        model_minimum_alignment_, 0.90);
    controller_nh.param("minimum_model_scale",
                        model_minimum_scale_, 0.50);
    controller_nh.param("maximum_model_scale",
                        model_maximum_scale_, 2.0);
    controller_nh.param("maximum_model_residual",
                        model_maximum_residual_, 0.30);
    controller_nh.param("gravity_calibration_verified",
                        gravity_calibration_verified_, false);
    controller_nh.param("maximum_gravity_effort_fraction",
                        model_maximum_gravity_effort_fraction_, 0.90);
    controller_nh.param("adaptive_entry_scale",
                        model_adaptive_entry_scale_, false);
    controller_nh.param("minimum_adaptive_scale",
                        model_minimum_adaptive_scale_, 0.75);
    controller_nh.param("maximum_adaptive_scale",
                        model_maximum_adaptive_scale_, 1.25);
    double effort_limit_scale = 0.20;
    controller_nh.param("effort_limit_scale", effort_limit_scale, 0.20);
    std::vector<double> configured_effort_limits;
    controller_nh.getParam("effort_limits", configured_effort_limits);
    model_effort_limits_.assign(kJointCount, 0.0);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const urdf::JointConstSharedPtr urdf_joint =
          model.getJoint(expected_joint_names_[joint]);
      if (!urdf_joint || !urdf_joint->limits) {
        gravity_model_error_ = "URDF 缺少 " + expected_joint_names_[joint] +
                               " 的力矩限制";
        return;
      }
      model_effort_limits_[joint] = urdf_joint->limits->effort *
                                    effort_limit_scale;
      if (configured_effort_limits.size() == kJointCount) {
        model_effort_limits_[joint] = std::min(
            model_effort_limits_[joint], configured_effort_limits[joint]);
      }
    }
    model_minimum_validation_joints_ = static_cast<unsigned int>(
        std::max(1, minimum_validation_joints));

    const bool vectors_valid =
        gravity.size() == 3 && gravity_joint_scales_.size() == kJointCount &&
        gravity_bias_.size() == kJointCount &&
        model_effort_limits_.size() == kJointCount &&
        std::all_of(gravity.begin(), gravity.end(), finite) &&
        std::all_of(gravity_joint_scales_.begin(),
                    gravity_joint_scales_.end(), finite) &&
        std::all_of(gravity_bias_.begin(), gravity_bias_.end(), finite) &&
        std::all_of(model_effort_limits_.begin(),
                    model_effort_limits_.end(),
                    [](double value) { return finite(value) && value > 0.0; });
    if (!vectors_valid || !finite(model_gravity_scale_) ||
        model_gravity_scale_ < 0.0 ||
        !finite(effort_limit_scale) || effort_limit_scale <= 0.0 ||
        effort_limit_scale > 1.0 ||
        !finite(model_maximum_gravity_effort_fraction_) ||
        model_maximum_gravity_effort_fraction_ < 0.50 ||
        model_maximum_gravity_effort_fraction_ >= 1.0 ||
        !finite(model_minimum_adaptive_scale_) ||
        model_minimum_adaptive_scale_ <= 0.0 ||
        !finite(model_maximum_adaptive_scale_) ||
        model_maximum_adaptive_scale_ < model_minimum_adaptive_scale_ ||
        !finite(model_minimum_validation_effort_) ||
        model_minimum_validation_effort_ < 0.0 ||
        model_minimum_validation_joints_ > kJointCount ||
        !finite(model_minimum_alignment_) ||
        minimum_warning_alignment_ > model_minimum_alignment_ ||
        !finite(model_minimum_scale_) || !finite(model_maximum_scale_) ||
        model_minimum_scale_ <= 0.0 ||
        model_maximum_scale_ <= model_minimum_scale_ ||
        !finite(model_maximum_residual_) || model_maximum_residual_ < 0.0) {
      gravity_model_error_ = "零力控制器的重力预检参数无效";
      return;
    }

    gravity_q_ = KDL::JntArray(gravity_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(gravity_chain_.getNrOfJoints());
    gravity_vector_ = KDL::Vector(gravity[0], gravity[1], gravity[2]);
    gravity_dynamics_.reset(
        new KDL::ChainDynParam(gravity_chain_, gravity_vector_));
    payload_jacobian_solver_.reset(
        new KDL::ChainJntToJacSolver(gravity_chain_));
    payload_fk_solver_.reset(
        new KDL::ChainFkSolverPos_recursive(gravity_chain_));
    payload_jacobian_ = KDL::Jacobian(gravity_chain_.getNrOfJoints());
    current_payload_effort_.fill(0.0);
    gravity_model_ready_ = true;
  }

  std::string payloadSummary() const {
    std::ostringstream summary;
    summary << payload_profile_name_ << "；质量=" << std::fixed
            << std::setprecision(3) << payload_model_.mass << " kg，重心=["
            << payload_model_.center_of_mass[0] << ", "
            << payload_model_.center_of_mass[1] << ", "
            << payload_model_.center_of_mass[2] << "] m；拟合 RMSE="
            << payload_fit_rmse_ << " Nm，留出最大误差="
            << payload_fit_max_error_ << " Nm，保持漂移="
            << payload_validation_drift_ << " rad；样本="
            << payload_sample_count_ << "；保持验证="
            << (payload_hold_verified_ ? "通过" : "未通过")
            << "；控制器同步="
            << (payload_model_synchronized_ ? "是" : "否")
            << "；文件=" << payload_profile_file_;
    return summary.str();
  }

  void loadPayloadProfile() {
    payload_model_ = payload::Model();
    payload_profile_name_ = "empty-base";
    payload_fit_rmse_ = 0.0;
    payload_fit_max_error_ = 0.0;
    payload_validation_drift_ = 0.0;
    payload_sample_count_ = 0;
    payload_hold_verified_ = false;

    std::ifstream input(payload_profile_file_);
    if (!input) {
      ROS_WARN_STREAM("No persisted payload profile at "
                      << payload_profile_file_
                      << "; using the calibrated empty-arm base model");
      return;
    }
    try {
      const YAML::Node profile = YAML::Load(input);
      if (!profile || !profile.IsMap() || !profile["schema_version"] ||
          profile["schema_version"].as<int>() != 1 ||
          !profile["profile_name"] || !profile["mass_kg"] ||
          !profile["center_of_mass_m"] ||
          !profile["center_of_mass_m"].IsSequence() ||
          profile["center_of_mass_m"].size() != 3) {
        throw std::runtime_error("missing or incompatible payload profile fields");
      }
      payload::Model loaded;
      loaded.mass = profile["mass_kg"].as<double>();
      for (std::size_t axis = 0; axis < loaded.center_of_mass.size(); ++axis) {
        loaded.center_of_mass[axis] =
            profile["center_of_mass_m"][axis].as<double>();
      }
      if (loaded.mass <= 1e-9) {
        loaded.mass = 0.0;
        loaded.center_of_mass.fill(0.0);
      }
      if (!payload::valid(loaded, maximum_payload_mass_)) {
        throw std::runtime_error(
            "payload mass/center is invalid or mass is out of bounds");
      }
      payload_model_ = loaded;
      payload_profile_name_ = profile["profile_name"].as<std::string>();
      payload_fit_rmse_ =
          profile["fit_rmse_nm"] ? profile["fit_rmse_nm"].as<double>() : 0.0;
      payload_fit_max_error_ = profile["fit_max_error_nm"]
                                   ? profile["fit_max_error_nm"].as<double>()
                                   : 0.0;
      payload_validation_drift_ =
          profile["validation_drift_rad"]
              ? profile["validation_drift_rad"].as<double>()
              : 0.0;
      payload_sample_count_ =
          profile["sample_count"]
              ? profile["sample_count"].as<std::uint32_t>()
              : 0;
      payload_hold_verified_ =
          profile["hold_verified"]
              ? profile["hold_verified"].as<bool>()
              : false;
      if (payload_profile_name_.empty() || !finite(payload_fit_rmse_) ||
          payload_fit_rmse_ < 0.0 || !finite(payload_fit_max_error_) ||
          payload_fit_max_error_ < 0.0 ||
          !finite(payload_validation_drift_) ||
          payload_validation_drift_ < 0.0) {
        throw std::runtime_error("payload profile metadata is invalid");
      }
      ROS_WARN_STREAM("Loaded persisted payload profile: " << payloadSummary());
    } catch (const std::exception& error) {
      payload_model_ = payload::Model();
      payload_profile_name_ = "empty-base";
      payload_fit_rmse_ = 0.0;
      payload_fit_max_error_ = 0.0;
      payload_validation_drift_ = 0.0;
      payload_sample_count_ = 0;
      payload_hold_verified_ = false;
      ROS_ERROR_STREAM("Ignoring invalid payload profile "
                       << payload_profile_file_ << ": " << error.what());
    }
  }

  bool writePayloadProfile(std::string& error) const {
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "schema_version" << YAML::Value << 1;
    emitter << YAML::Key << "profile_name" << YAML::Value
            << payload_profile_name_;
    emitter << YAML::Key << "created_wall_time" << YAML::Value
            << ros::WallTime::now().toSec();
    emitter << YAML::Key << "mass_kg" << YAML::Value << payload_model_.mass;
    emitter << YAML::Key << "center_of_mass_m" << YAML::Value
            << YAML::Flow << YAML::BeginSeq
            << payload_model_.center_of_mass[0]
            << payload_model_.center_of_mass[1]
            << payload_model_.center_of_mass[2] << YAML::EndSeq;
    emitter << YAML::Key << "fit_rmse_nm" << YAML::Value
            << payload_fit_rmse_;
    emitter << YAML::Key << "fit_max_error_nm" << YAML::Value
            << payload_fit_max_error_;
    emitter << YAML::Key << "validation_drift_rad" << YAML::Value
            << payload_validation_drift_;
    emitter << YAML::Key << "sample_count" << YAML::Value
            << payload_sample_count_;
    emitter << YAML::Key << "hold_verified" << YAML::Value
            << payload_hold_verified_;
    emitter << YAML::EndMap;
    if (!emitter.good()) {
      error = "无法生成末端负载 YAML：" + emitter.GetLastError();
      return false;
    }

    const std::string temporary = payload_profile_file_ + ".tmp";
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    if (!output) {
      error = "无法创建末端负载临时文件：" + temporary;
      return false;
    }
    output << "# Elfin E05 active rigid payload profile.\n";
    output << emitter.c_str() << "\n";
    output.flush();
    if (!output) {
      output.close();
      std::remove(temporary.c_str());
      error = "写入末端负载临时文件失败";
      return false;
    }
    output.close();
    if (std::rename(temporary.c_str(), payload_profile_file_.c_str()) != 0) {
      const int rename_error = errno;
      std::remove(temporary.c_str());
      error = "无法原子替换末端负载文件：" +
              std::string(std::strerror(rename_error));
      return false;
    }
    return true;
  }

  bool applyPayloadToController(const payload::Model& model,
                                const std::string& profile_name,
                                double fit_rmse,
                                double fit_max_error,
                                double validation_drift,
                                std::uint32_t sample_count,
                                bool controlled_hold_validation,
                                bool hold_verified,
                                std::string& error) {
    if (!payload_model_client_.exists()) {
      error = "零力控制器的末端负载服务尚未上线";
      return false;
    }
    SetPayloadModel service;
    service.request.profile_name = profile_name;
    service.request.mass = model.mass;
    for (std::size_t axis = 0; axis < model.center_of_mass.size(); ++axis) {
      service.request.center_of_mass[axis] = model.center_of_mass[axis];
    }
    service.request.fit_rmse = fit_rmse;
    service.request.fit_max_error = fit_max_error;
    service.request.validation_drift = validation_drift;
    service.request.sample_count = sample_count;
    service.request.persist = false;
    service.request.controlled_hold_validation = controlled_hold_validation;
    service.request.hold_verified = hold_verified;
    if (!payload_model_client_.call(service)) {
      error = "调用零力控制器末端负载服务失败";
      return false;
    }
    if (!service.response.success) {
      error = "零力控制器拒绝末端负载：" + service.response.message;
      return false;
    }
    return true;
  }

  bool synchronizePayloadModel() {
    if (freedrive_active_ || transition_busy_) {
      return false;
    }
    std::string error;
    if (!applyPayloadToController(
            payload_model_, payload_profile_name_, payload_fit_rmse_,
            payload_fit_max_error_, payload_validation_drift_,
            payload_sample_count_, false, payload_hold_verified_, error)) {
      payload_model_synchronized_ = false;
      return false;
    }
    payload_model_synchronized_ = true;
    root_nh_.setParam("/elfin_freedrive_controller/payload_mass",
                      payload_model_.mass);
    root_nh_.setParam(
        "/elfin_freedrive_controller/payload_center_of_mass",
        std::vector<double>(payload_model_.center_of_mass.begin(),
                            payload_model_.center_of_mass.end()));
    publishPayloadModel();
    return true;
  }

  void publishPayloadModel() {
    std_msgs::String profile;
    profile.data = payloadSummary();
    payload_profile_pub_.publish(profile);
    std_msgs::Float64MultiArray values;
    values.data = {payload_model_.mass,
                   payload_model_.center_of_mass[0],
                   payload_model_.center_of_mass[1],
                   payload_model_.center_of_mass[2],
                   payload_fit_rmse_, payload_fit_max_error_,
                   payload_validation_drift_};
    payload_model_pub_.publish(values);
  }

  bool getPayloadModelCallback(GetPayloadModel::Request& request,
                               GetPayloadModel::Response& response) {
    (void)request;
    response.success = true;
    response.message = payloadSummary();
    response.profile_name = payload_profile_name_;
    response.mass = payload_model_.mass;
    for (std::size_t axis = 0; axis < payload_model_.center_of_mass.size();
         ++axis) {
      response.center_of_mass[axis] = payload_model_.center_of_mass[axis];
    }
    response.fit_rmse = payload_fit_rmse_;
    response.fit_max_error = payload_fit_max_error_;
    response.validation_drift = payload_validation_drift_;
    response.sample_count = payload_sample_count_;
    response.hold_verified = payload_hold_verified_;
    response.synchronized = payload_model_synchronized_;
    response.profile_file = payload_profile_file_;
    return true;
  }

  bool setPayloadModelCallback(SetPayloadModel::Request& request,
                               SetPayloadModel::Response& response) {
    response.profile_file = payload_profile_file_;
    if (freedrive_active_ || transition_busy_ || exit_requested_ ||
        position_recovery_pending_) {
      response.success = false;
      response.message = "正在拖拽或切换控制器，不能更换末端负载模型";
      return true;
    }
    payload::Model candidate;
    candidate.mass = request.mass;
    for (std::size_t axis = 0; axis < candidate.center_of_mass.size(); ++axis) {
      candidate.center_of_mass[axis] = request.center_of_mass[axis];
    }
    if (candidate.mass <= 1e-9) {
      candidate.mass = 0.0;
      candidate.center_of_mass.fill(0.0);
    }
    if (request.profile_name.empty() ||
        (request.persist && request.controlled_hold_validation) ||
        (request.controlled_hold_validation && request.hold_verified) ||
        !payload::valid(candidate, maximum_payload_mass_) ||
        !finite(request.fit_rmse) || request.fit_rmse < 0.0 ||
        !finite(request.fit_max_error) || request.fit_max_error < 0.0 ||
        !finite(request.validation_drift) || request.validation_drift < 0.0) {
      response.success = false;
      response.message = "末端负载参数、拟合指标或配置名称无效";
      return true;
    }

    std::string error;
    if (!applyPayloadToController(
            candidate, request.profile_name, request.fit_rmse,
            request.fit_max_error, request.validation_drift,
            request.sample_count, request.controlled_hold_validation,
            request.hold_verified, error)) {
      payload_model_synchronized_ = false;
      response.success = false;
      response.message = error;
      publishPayloadModel();
      return true;
    }

    const payload::Model previous_model = payload_model_;
    const std::string previous_name = payload_profile_name_;
    const double previous_rmse = payload_fit_rmse_;
    const double previous_max_error = payload_fit_max_error_;
    const double previous_drift = payload_validation_drift_;
    const std::uint32_t previous_samples = payload_sample_count_;
    const bool previous_hold_verified = payload_hold_verified_;
    payload_model_ = candidate;
    payload_profile_name_ = request.profile_name;
    payload_fit_rmse_ = request.fit_rmse;
    payload_fit_max_error_ = request.fit_max_error;
    payload_validation_drift_ = request.validation_drift;
    payload_sample_count_ = request.sample_count;
    payload_hold_verified_ = request.hold_verified;
    payload_model_synchronized_ = true;
    controlled_hold_validation_ = request.controlled_hold_validation;

    if (request.persist && !writePayloadProfile(error)) {
      const std::string persistence_error = error;
      payload_model_ = previous_model;
      payload_profile_name_ = previous_name;
      payload_fit_rmse_ = previous_rmse;
      payload_fit_max_error_ = previous_max_error;
      payload_validation_drift_ = previous_drift;
      payload_sample_count_ = previous_samples;
      payload_hold_verified_ = previous_hold_verified;
      std::string rollback_error;
      payload_model_synchronized_ = applyPayloadToController(
          previous_model, previous_name, previous_rmse, previous_max_error,
          previous_drift, previous_samples, false, previous_hold_verified,
          rollback_error);
      controlled_hold_validation_ = false;
      response.success = false;
      response.message = "持久化失败：" + persistence_error;
      if (payload_model_synchronized_) {
        response.message += "；控制器已回滚原负载模型";
      } else {
        response.message += "；控制器回滚也失败：" + rollback_error +
                            "。FREE 已锁定，请保持位置模式并重启控制栈";
      }
      publishPayloadModel();
      return true;
    }

    root_nh_.setParam("/elfin_freedrive_controller/payload_mass",
                      payload_model_.mass);
    root_nh_.setParam(
        "/elfin_freedrive_controller/payload_center_of_mass",
        std::vector<double>(payload_model_.center_of_mass.begin(),
                            payload_model_.center_of_mass.end()));
    preflight_samples_.clear();
    publishPayloadModel();
    response.success = true;
    response.message =
        std::string(request.persist ? "已验证并持久启用：" : "已暂存候选：") +
        payloadSummary();
    return true;
  }

  bool evaluatePayloadModelCallback(
      EvaluatePayloadModel::Request& request,
      EvaluatePayloadModel::Response& response) {
    if (!gravity_model_ready_) {
      response.success = false;
      response.message = gravity_model_error_;
      return true;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (!finite(request.joint_positions[joint])) {
        response.success = false;
        response.message = "请求包含非有限关节角";
        return true;
      }
      gravity_q_(gravity_joint_to_chain_[joint]) =
          request.joint_positions[joint];
    }
    if (gravity_dynamics_->JntToGravity(gravity_q_, gravity_effort_) < 0 ||
        !payload::buildRegressor(
            gravity_q_, gravity_vector_, *payload_jacobian_solver_,
            *payload_fk_solver_, payload_jacobian_, payload_regressor_)) {
      response.success = false;
      response.message = "KDL 无法计算该姿态的基础重力或末端回归矩阵";
      return true;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const std::size_t chain_joint = gravity_joint_to_chain_[joint];
      response.base_effort[joint] =
          model_gravity_scale_ * gravity_joint_scales_[joint] *
              gravity_effort_(chain_joint) +
          gravity_bias_[joint];
      response.effort_limits[joint] = model_effort_limits_[joint];
      for (std::size_t parameter = 0;
           parameter < payload::kParameterCount; ++parameter) {
        response.payload_regressor[
            joint * payload::kParameterCount + parameter] =
            payload_regressor_[chain_joint][parameter];
      }
    }
    response.maximum_gravity_effort_fraction =
        model_maximum_gravity_effort_fraction_;
    response.success = true;
    response.message = "已计算空臂基础重力和未知末端线性回归矩阵";
    return true;
  }

  void updatePreflightSamples() {
    if (freedrive_active_ || transition_busy_ || !effort_state_valid_ ||
        maxAbsVelocity() > preflight_velocity_limit_) {
      preflight_samples_.clear();
      return;
    }

    if (!preflight_samples_.empty()) {
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (std::abs(positions_[joint] -
                     preflight_samples_.front().position[joint]) >
            preflight_position_tolerance_) {
          preflight_samples_.clear();
          break;
        }
      }
    }

    StaticEffortSample sample;
    sample.stamp = ros::WallTime::now();
    sample.position = positions_;
    sample.effort = efforts_;
    preflight_samples_.push_back(sample);
    while (!preflight_samples_.empty() &&
           (sample.stamp - preflight_samples_.front().stamp).toSec() >
               preflight_window_) {
      preflight_samples_.pop_front();
    }
  }

  bool buildStaticObservation(GravityCalibrationSample& observation,
                              std::string& detail) {
    if (!gravity_model_ready_) {
      detail = "未通过：" + gravity_model_error_;
      return false;
    }
    if (!jointStateFresh() || !effort_state_valid_) {
      detail = "未通过：六轴反馈力矩缺失或已过期";
      return false;
    }
    if (preflight_samples_.size() < preflight_min_samples_) {
      std::ostringstream waiting;
      waiting << "等待静止力矩样本 " << preflight_samples_.size() << "/"
              << preflight_min_samples_;
      detail = waiting.str();
      return false;
    }
    const double duration =
        (preflight_samples_.back().stamp -
         preflight_samples_.front().stamp).toSec();
    if (duration < preflight_min_duration_) {
      std::ostringstream waiting;
      waiting << "等待静止采样时长：当前 " << std::fixed
              << std::setprecision(3) << duration << " s，至少 "
              << preflight_min_duration_ << " s";
      detail = waiting.str();
      return false;
    }

    observation = GravityCalibrationSample();
    observation.stamp = ros::WallTime::now().toSec();
    for (const StaticEffortSample& sample : preflight_samples_) {
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        observation.position[joint] += sample.position[joint];
        observation.measured_effort[joint] += sample.effort[joint];
      }
    }
    const double sample_count =
        static_cast<double>(preflight_samples_.size());
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      observation.position[joint] /= sample_count;
      observation.measured_effort[joint] /= sample_count;
      gravity_q_(gravity_joint_to_chain_[joint]) =
          observation.position[joint];
    }
    if (gravity_dynamics_->JntToGravity(gravity_q_, gravity_effort_) < 0) {
      detail = "未通过：KDL 无法计算当前位置的重力力矩";
      return false;
    }
    if (!payload::buildRegressor(
            gravity_q_, gravity_vector_, *payload_jacobian_solver_,
            *payload_fk_solver_, payload_jacobian_, payload_regressor_)) {
      detail = "未通过：KDL 无法计算未知末端负载回归矩阵";
      return false;
    }
    current_payload_effort_ = payload::evaluate(payload_regressor_,
                                                payload_model_);

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      observation.base_model_effort[joint] =
          model_gravity_scale_ *
          gravity_effort_(gravity_joint_to_chain_[joint]);
      double variance = 0.0;
      for (const StaticEffortSample& sample : preflight_samples_) {
        const double residual =
            sample.effort[joint] - observation.measured_effort[joint];
        variance += residual * residual;
      }
      observation.effort_stddev[joint] =
          std::sqrt(variance / sample_count);
    }
    return true;
  }

  double configuredModelEffort(const GravityCalibrationSample& observation,
                               std::size_t joint) const {
    return gravity_joint_scales_[joint] *
               observation.base_model_effort[joint] +
           gravity_bias_[joint] +
           current_payload_effort_[gravity_joint_to_chain_[joint]];
  }

  bool evaluateGravityPreflight(std::string& detail,
                                bool* warning = nullptr) {
    if (warning != nullptr) {
      *warning = false;
    }
    if (simulation_) {
      detail = "通过（仿真）：真机反馈力矩一致性门禁不适用";
      return true;
    }
    GravityCalibrationSample observation;
    if (!buildStaticObservation(observation, detail)) {
      return false;
    }

    std::vector<double> model_effort(kJointCount, 0.0);
    std::vector<double> measured_effort(kJointCount, 0.0);
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      model_effort[joint] = configuredModelEffort(observation, joint);
      measured_effort[joint] = observation.measured_effort[joint];
    }
    const math::GravityValidation validation =
        math::validateGravityObservation(
            model_effort, measured_effort,
            model_minimum_validation_effort_);
    const math::EffortModelError model_error =
        math::maximumAbsoluteEffortError(model_effort, measured_effort);
    const bool absolute_model_error_valid =
        model_error.valid &&
        model_error.maximum_absolute_error <= preflight_max_model_error_;
    const bool model_error_allows_entry =
        absolute_model_error_valid || controlled_hold_validation_ ||
        payload_hold_verified_;

    double adaptive_scale = 1.0;
    if (model_adaptive_entry_scale_ && validation.sufficient_excitation) {
      adaptive_scale = math::clamp(validation.scale_estimate,
                                   model_minimum_adaptive_scale_,
                                   model_maximum_adaptive_scale_);
    }
    bool gravity_capacity_valid = true;
    std::size_t limiting_joint = 0;
    double limiting_requested = 0.0;
    double limiting_available = 0.0;
    double limiting_ratio = -1.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double adapted_gravity =
          adaptive_scale * gravity_joint_scales_[joint] *
              observation.base_model_effort[joint] +
          gravity_bias_[joint] +
          current_payload_effort_[gravity_joint_to_chain_[joint]];
      const double available = model_effort_limits_[joint] *
                               model_maximum_gravity_effort_fraction_;
      const double ratio = std::abs(adapted_gravity) / available;
      if (ratio > limiting_ratio) {
        limiting_ratio = ratio;
        limiting_joint = joint;
        limiting_requested = std::abs(adapted_gravity);
        limiting_available = available;
      }
      if (!math::gravityEffortHasCapacity(
              adapted_gravity, model_effort_limits_[joint],
              model_maximum_gravity_effort_fraction_)) {
        gravity_capacity_valid = false;
      }
    }

    double maximum_effort_stddev = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (std::abs(model_effort[joint]) <
          model_minimum_validation_effort_) {
        continue;
      }
      maximum_effort_stddev = std::max(
          maximum_effort_stddev, observation.effort_stddev[joint]);
    }

    // A payload profile that already survived the real hold test does not
    // need a well-excited pose on every later FREE entry. Keep the static
    // stability and actuator-capacity checks below. Candidate calibration
    // profiles still have to pass the model-direction checks before their
    // first controlled hold test.
    const bool hold_validation_valid = payload_hold_verified_;
    const bool strictly_accepted =
        maximum_effort_stddev <= preflight_max_effort_stddev_ &&
        gravity_capacity_valid &&
        (hold_validation_valid ||
         (math::gravityValidationAccepted(
              validation, model_minimum_validation_joints_,
              model_minimum_alignment_, model_minimum_scale_,
              model_maximum_scale_, model_maximum_residual_) &&
          model_error_allows_entry));
    const bool warning_accepted =
        !strictly_accepted && allow_model_validation_warning_ &&
        gravity_calibration_verified_ &&
        maximum_effort_stddev <= preflight_max_effort_stddev_ &&
        model_error_allows_entry &&
        gravity_capacity_valid &&
        math::gravityValidationWarningAccepted(
            validation, minimum_warning_alignment_, model_minimum_scale_,
            model_maximum_scale_);
    const bool accepted = strictly_accepted || warning_accepted;
    if (warning != nullptr) {
      *warning = warning_accepted;
    }
    std::ostringstream result;
    result << (strictly_accepted
                   ? "通过"
                   : (warning_accepted ? "警告（允许进入）" : "未通过"))
           << "（位置模式静止预检）"
           << "；有效轴=" << validation.excited_joints
           << "，反向轴=" << validation.direction_mismatches
           << "，方向一致度=" << std::fixed << std::setprecision(3)
           << validation.alignment
           << "，反馈/模型比例=" << validation.scale_estimate
           << "，归一化残差=" << validation.normalized_residual
           << "，最大力矩波动=" << maximum_effort_stddev << " Nm"
           << "，最大模型误差=" << model_error.maximum_absolute_error
           << " Nm（J" << model_error.joint + 1 << "，门限 "
           << preflight_max_model_error_ << " Nm"
           << (controlled_hold_validation_
                   ? "；受控保持验证有效"
                   : (payload_hold_verified_
                          ? "；实际保持验证已通过"
                          : ""))
           << "）"
           << "，重力容量="
           << (gravity_capacity_valid ? "通过" : "不足")
           << "（J" << limiting_joint + 1 << " 需求="
           << limiting_requested << " Nm，可用=" << limiting_available
           << " Nm，入口微调=" << adaptive_scale << "）";
    detail = result.str();
    return accepted;
  }

  void publishValidation(const std::string& detail) {
    std_msgs::String message;
    message.data = detail;
    validation_pub_.publish(message);
  }

  void loadGravityCalibrationSamples() {
    calibration_samples_.clear();
    std::ifstream input(calibration_samples_file_);
    if (!input) {
      return;
    }
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::vector<double> values;
      std::istringstream row(line);
      std::string field;
      bool valid = true;
      while (std::getline(row, field, ',')) {
        try {
          std::size_t parsed = 0;
          const double value = std::stod(field, &parsed);
          if (parsed != field.size() || !finite(value)) {
            valid = false;
            break;
          }
          values.push_back(value);
        } catch (const std::exception&) {
          valid = false;
          break;
        }
      }
      if (!valid || values.size() != 1 + 4 * kJointCount) {
        ROS_WARN_STREAM("Ignoring invalid gravity calibration row in "
                        << calibration_samples_file_);
        continue;
      }
      GravityCalibrationSample sample;
      sample.stamp = values[0];
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        sample.position[joint] = values[1 + joint];
        sample.measured_effort[joint] = values[1 + kJointCount + joint];
        sample.base_model_effort[joint] =
            values[1 + 2 * kJointCount + joint];
        sample.effort_stddev[joint] =
            values[1 + 3 * kJointCount + joint];
      }
      calibration_samples_.push_back(sample);
    }
    if (!calibration_samples_.empty()) {
      ROS_WARN_STREAM("Loaded " << calibration_samples_.size()
                      << " passive gravity calibration samples from "
                      << calibration_samples_file_);
    }
  }

  bool appendGravityCalibrationSample(
      const GravityCalibrationSample& sample,
      std::string& error) {
    bool needs_header = true;
    {
      std::ifstream existing(calibration_samples_file_);
      needs_header = !existing || existing.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream output(calibration_samples_file_,
                         std::ios::out | std::ios::app);
    if (!output) {
      error = "无法写入 " + calibration_samples_file_;
      return false;
    }
    if (needs_header) {
      output << "# stamp,q1,q2,q3,q4,q5,q6,measured1,measured2,"
                "measured3,measured4,measured5,measured6,model1,model2,"
                "model3,model4,model5,model6,stddev1,stddev2,stddev3,"
                "stddev4,stddev5,stddev6\n";
    }
    output << std::setprecision(17) << sample.stamp;
    const std::array<const std::array<double, kJointCount>*, 4> groups{{
        &sample.position, &sample.measured_effort,
        &sample.base_model_effort, &sample.effort_stddev}};
    for (const auto* group : groups) {
      for (double value : *group) {
        output << "," << value;
      }
    }
    output << "\n";
    output.flush();
    if (!output) {
      error = "写入标定样本时发生 I/O 错误";
      return false;
    }
    return true;
  }

  bool recordGravitySampleCallback(
      std_srvs::Trigger::Request& request,
      std_srvs::Trigger::Response& response) {
    (void)request;
    if (simulation_) {
      response.success = false;
      response.message = "真机静态标定不在 Gazebo 中采样";
      return true;
    }
    if (freedrive_active_ || transition_busy_ || exit_requested_) {
      response.success = false;
      response.message = "标定样本只能在正常位置控制模式记录";
      return true;
    }
    if (!driverStateFresh() || !servo_enabled_ || faulted_) {
      response.success = false;
      response.message = "需要 Servo On、无 Fault 且状态新鲜";
      return true;
    }
    ControllerSnapshot snapshot;
    std::string detail;
    if (!queryControllers(snapshot, detail) || !snapshot.position_running ||
        snapshot.freedrive_running) {
      response.success = false;
      response.message = detail.empty()
                             ? "位置控制器未独占六轴"
                             : detail;
      return true;
    }

    GravityCalibrationSample sample;
    if (!buildStaticObservation(sample, detail)) {
      response.success = false;
      response.message = detail;
      return true;
    }
    const double maximum_stddev = *std::max_element(
        sample.effort_stddev.begin(), sample.effort_stddev.end());
    if (maximum_stddev > preflight_max_effort_stddev_) {
      std::ostringstream message;
      message << "力矩尚未稳定：最大波动 " << std::fixed
              << std::setprecision(3) << maximum_stddev << " Nm，需不超过 "
              << preflight_max_effort_stddev_ << " Nm";
      response.success = false;
      response.message = message.str();
      return true;
    }
    if (!calibration_samples_.empty()) {
      const GravityCalibrationSample& previous = calibration_samples_.back();
      double maximum_position_change = 0.0;
      double maximum_model_change = 0.0;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        maximum_position_change = std::max(
            maximum_position_change,
            std::abs(sample.position[joint] - previous.position[joint]));
        maximum_model_change = std::max(
            maximum_model_change,
            std::abs(sample.base_model_effort[joint] -
                     previous.base_model_effort[joint]));
      }
      if (maximum_position_change < calibration_min_pose_separation_ ||
          maximum_model_change < calibration_min_model_change_) {
        std::ostringstream message;
        message << "与上一标定姿态过于接近：最大关节变化="
                << std::fixed << std::setprecision(3)
                << maximum_position_change << " rad，最大模型力矩变化="
                << maximum_model_change << " Nm";
        response.success = false;
        response.message = message.str();
        return true;
      }
    }
    if (!appendGravityCalibrationSample(sample, detail)) {
      response.success = false;
      response.message = detail;
      return true;
    }
    calibration_samples_.push_back(sample);
    std::ostringstream message;
    message << "已记录静态重力样本 " << calibration_samples_.size() << "/"
            << calibration_min_samples_ << "；J2 反馈/基础模型="
            << std::fixed << std::setprecision(3)
            << sample.measured_effort[1] << "/"
            << sample.base_model_effort[1] << " Nm，J3="
            << sample.measured_effort[2] << "/"
            << sample.base_model_effort[2] << " Nm";
    response.success = true;
    response.message = message.str();
    return true;
  }

  bool fitGravityCalibrationCallback(
      std_srvs::Trigger::Request& request,
      std_srvs::Trigger::Response& response) {
    (void)request;
    if (calibration_samples_.size() < calibration_min_samples_) {
      std::ostringstream message;
      message << "样本不足：当前 " << calibration_samples_.size()
              << "，至少需要 " << calibration_min_samples_;
      response.success = false;
      response.message = message.str();
      return true;
    }

    std::vector<math::StaticGravityObservation> raw_observations;
    raw_observations.reserve(calibration_samples_.size());
    for (const GravityCalibrationSample& sample : calibration_samples_) {
      math::StaticGravityObservation observation;
      observation.position.assign(sample.position.begin(),
                                  sample.position.end());
      observation.model.assign(sample.base_model_effort.begin(),
                               sample.base_model_effort.end());
      observation.measured.assign(sample.measured_effort.begin(),
                                  sample.measured_effort.end());
      raw_observations.push_back(observation);
    }
    const std::vector<math::StaticGravityObservation> centered_observations =
        math::centerOppositeApproaches(
            raw_observations, 1, calibration_min_approach_delta_,
            calibration_pair_pose_tolerance_);
    if (centered_observations.size() < calibration_min_paired_poses_) {
      std::ostringstream message;
      message << "双向成对姿态不足：当前 " << centered_observations.size()
              << "，至少需要 " << calibration_min_paired_poses_
              << "；单向到达样本不会用于抵消减速器静摩擦";
      response.success = false;
      response.message = message.str();
      return true;
    }

    std::array<math::AffineGravityFit, kJointCount> fits;
    std::array<bool, kJointCount> accepted{};
    std::array<double, kJointCount> candidate_scales{};
    std::array<double, kJointCount> candidate_bias{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      std::vector<double> model;
      std::vector<double> measured;
      model.reserve(centered_observations.size());
      measured.reserve(centered_observations.size());
      for (const auto& observation : centered_observations) {
        model.push_back(observation.model[joint]);
        measured.push_back(observation.measured[joint]);
      }
      fits[joint] = math::fitAffineGravity(
          model, measured, calibration_min_paired_poses_,
          calibration_min_model_range_);
      accepted[joint] = math::affineGravityFitAccepted(
          fits[joint], calibration_min_scale_, calibration_max_scale_,
          calibration_max_normalized_residual_,
          calibration_max_absolute_residual_);
      candidate_scales[joint] =
          accepted[joint] ? fits[joint].scale : gravity_joint_scales_[joint];
      candidate_bias[joint] =
          accepted[joint] ? fits[joint].bias : gravity_bias_[joint];
    }

    // J2 and J3 carry the arm's principal gravity load. A candidate that has
    // not independently excited and validated both axes must never unlock CST.
    if (!accepted[1] || !accepted[2]) {
      std::ostringstream message;
      message << "标定未通过：J2(valid=" << accepted[1]
              << ", range=" << std::fixed << std::setprecision(3)
              << fits[1].model_range << ", LOO max="
              << fits[1].cross_validation_maximum << " Nm)，J3(valid="
              << accepted[2] << ", range=" << fits[2].model_range
              << ", LOO max=" << fits[2].cross_validation_maximum
              << " Nm)";
      response.success = false;
      response.message = message.str();
      return true;
    }

    std::ofstream output(calibration_candidate_file_, std::ios::out);
    if (!output) {
      response.success = false;
      response.message = "无法写入 " + calibration_candidate_file_;
      return true;
    }
    output << "# Passive position-mode fit. Review, restart, and run the "
              "static CST test before hand guiding.\n";
    output << "calibration_sample_count: " << calibration_samples_.size()
           << "\n";
    output << "calibration_paired_pose_count: "
           << centered_observations.size() << "\n";
    output << "elfin_freedrive_controller:\n";
    output << "  gravity_calibration_verified: true\n";
    output << std::setprecision(17);
    output << "  gravity_joint_scales: [";
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      output << (joint == 0 ? "" : ", ") << candidate_scales[joint];
    }
    output << "]\n  gravity_bias: [";
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      output << (joint == 0 ? "" : ", ") << candidate_bias[joint];
    }
    output << "]\nfit_diagnostics:\n";
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      output << "  elfin_joint" << joint + 1 << ": {accepted: "
             << (accepted[joint] ? "true" : "false")
             << ", model_range_nm: " << fits[joint].model_range
             << ", scale: " << fits[joint].scale
             << ", bias_nm: " << fits[joint].bias
             << ", rms_nm: " << fits[joint].rms_residual
             << ", max_nm: " << fits[joint].maximum_residual
             << ", loo_rms_nm: " << fits[joint].cross_validation_rms
             << ", loo_max_nm: " << fits[joint].cross_validation_maximum
             << "}\n";
    }
    output.flush();
    if (!output) {
      response.success = false;
      response.message = "写入标定候选文件时发生 I/O 错误";
      return true;
    }
    std::ostringstream message;
    message << "标定候选已通过 J2/J3 留一验证并写入 "
            << calibration_candidate_file_ << "；J2 scale/bias="
            << std::fixed << std::setprecision(3) << fits[1].scale << "/"
            << fits[1].bias << "，J3=" << fits[2].scale << "/"
            << fits[2].bias << "，双向成对姿态="
            << centered_observations.size();
    response.success = true;
    response.message = message.str();
    return true;
  }

  void maybePublishPassivePreflight() {
    const ros::WallTime now = ros::WallTime::now();
    if (!last_preflight_publish_wall_.isZero() &&
        (now - last_preflight_publish_wall_).toSec() < 0.5) {
      return;
    }
    last_preflight_publish_wall_ = now;
    std::string detail;
    if (!simulation_ &&
        (!driverStateFresh() || !servo_enabled_ || faulted_)) {
      detail = "等待 Servo On、无 Fault 后采集位置模式静止力矩";
    } else {
      evaluateGravityPreflight(detail);
    }
    publishValidation(detail);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message) {
    if (message->name.size() != message->position.size() ||
        message->name.size() != message->velocity.size()) {
      joint_state_valid_ = false;
      effort_state_valid_ = false;
      preflight_samples_.clear();
      return;
    }
    std::array<bool, kJointCount> found{};
    bool all_efforts_valid = true;
    for (std::size_t source = 0; source < message->name.size(); ++source) {
      const auto match = std::find(expected_joint_names_.begin(),
                                   expected_joint_names_.end(),
                                   message->name[source]);
      if (match == expected_joint_names_.end()) {
        continue;
      }
      const std::size_t target = static_cast<std::size_t>(
          std::distance(expected_joint_names_.begin(), match));
      if (!finite(message->position[source]) ||
          !finite(message->velocity[source])) {
        joint_state_valid_ = false;
        return;
      }
      positions_[target] = message->position[source];
      velocities_[target] = message->velocity[source];
      if (source < message->effort.size() && finite(message->effort[source])) {
        efforts_[target] = message->effort[source];
      } else {
        efforts_[target] = 0.0;
        all_efforts_valid = false;
      }
      found[target] = true;
    }
    joint_state_valid_ =
        std::all_of(found.begin(), found.end(), [](bool value) { return value; });
    effort_state_valid_ = joint_state_valid_ && all_efforts_valid;
    if (joint_state_valid_) {
      last_joint_state_wall_ = ros::WallTime::now();
      updatePreflightSamples();
    } else {
      preflight_samples_.clear();
    }
  }

  void servoCallback(const std_msgs::Bool::ConstPtr& message) {
    if (!servo_received_ || servo_enabled_ != message->data) {
      preflight_samples_.clear();
    }
    servo_received_ = true;
    servo_enabled_ = message->data;
    last_servo_wall_ = ros::WallTime::now();
  }

  void faultCallback(const std_msgs::Bool::ConstPtr& message) {
    if (message->data) {
      preflight_samples_.clear();
    }
    fault_received_ = true;
    faulted_ = message->data;
    last_fault_wall_ = ros::WallTime::now();
  }

  void controllerStatusCallback(const std_msgs::UInt8::ConstPtr& message) {
    controller_status_received_ = true;
    controller_status_ = message->data;
    last_controller_status_wall_ = ros::WallTime::now();
  }

  void startTrialLog(const std::string& source) {
    if (trial_log_.is_open()) {
      trial_log_.flush();
      trial_log_.close();
    }
    trial_log_rows_ = 0;
    if (mkdir(trial_log_directory_.c_str(), 0755) != 0 && errno != EEXIST) {
      ROS_ERROR_STREAM("Cannot create freedrive trial log directory: "
                       << trial_log_directory_);
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    std::ostringstream path;
    path << trial_log_directory_ << "/trial_" << now.sec << "_"
         << now.nsec << ".csv";
    trial_log_path_ = path.str();
    trial_log_.open(trial_log_path_, std::ios::out | std::ios::trunc);
    if (!trial_log_) {
      ROS_ERROR_STREAM("Cannot open freedrive trial log: " << trial_log_path_);
      trial_log_path_.clear();
      return;
    }
    trial_log_ << "# source," << source << "\n";
    trial_log_ << "stamp,status,handoff_progress,model_alignment,"
                  "model_scale_estimate,model_normalized_residual,"
                  "model_excited_joints,model_direction_mismatches,"
                  "model_validation_passed,settling";
    const std::array<std::string, 4> groups = {
        "position", "velocity", "measured_effort", "gravity_effort"};
    for (const std::string& group : groups) {
      for (const std::string& joint : expected_joint_names_) {
        trial_log_ << "," << group << "_" << joint;
      }
    }
    for (const std::string& joint : expected_joint_names_) {
      trial_log_ << ",commanded_effort_" << joint;
    }
    for (const std::string& joint : expected_joint_names_) {
      trial_log_ << ",effective_scale_" << joint;
    }
    trial_log_ << "\n";
    trial_log_.flush();
    std_msgs::String path_message;
    path_message.data = trial_log_path_;
    trial_log_pub_.publish(path_message);
  }

  void telemetryCallback(const FreedriveTelemetry::ConstPtr& message) {
    if (message->joint_names.size() != kJointCount ||
        message->position.size() != kJointCount ||
        message->velocity.size() != kJointCount ||
        message->measured_effort.size() != kJointCount ||
        message->gravity_effort.size() != kJointCount ||
        message->commanded_effort.size() != kJointCount ||
        message->effective_gravity_scale.size() != kJointCount) {
      telemetry_valid_ = false;
      return;
    }
    last_telemetry_ = *message;
    telemetry_valid_ = true;
    last_telemetry_wall_ = ros::WallTime::now();

    std_msgs::String validation;
    std::ostringstream detail;
    if (simulation_) {
      detail << std::fixed << std::setprecision(1)
             << "仿真跳过真机反馈力矩一致性检查；平滑接管="
             << message->handoff_progress * 100.0 << "%";
    } else {
      detail << std::fixed << std::setprecision(3)
             << (message->model_validation_passed ? "通过" : "未通过")
             << "；方向一致度=" << message->model_alignment
             << "，入口力矩/模型比例=" << message->model_scale_estimate
             << "，归一化残差=" << message->model_normalized_residual
             << "，有效轴=" << message->model_excited_joints
             << "，反向轴=" << message->model_direction_mismatches
             << "，平滑接管=" << message->handoff_progress * 100.0 << "%";
    }
    validation.data = detail.str();
    validation_pub_.publish(validation);

    if (!trial_log_.is_open()) {
      return;
    }
    const double stamp = message->header.stamp.isZero()
                             ? ros::WallTime::now().toSec()
                             : message->header.stamp.toSec();
    trial_log_ << std::setprecision(17) << stamp << ","
               << static_cast<unsigned int>(message->status) << ","
               << message->handoff_progress << ","
               << message->model_alignment << ","
               << message->model_scale_estimate << ","
               << message->model_normalized_residual << ","
               << message->model_excited_joints << ","
               << message->model_direction_mismatches << ","
               << (message->model_validation_passed ? 1 : 0) << ","
               << (message->settling ? 1 : 0);
    const std::array<const std::vector<double>*, 6> values = {
        &message->position, &message->velocity, &message->measured_effort,
        &message->gravity_effort, &message->commanded_effort,
        &message->effective_gravity_scale};
    for (const std::vector<double>* vector : values) {
      for (double value : *vector) {
        trial_log_ << "," << value;
      }
    }
    trial_log_ << "\n";
    ++trial_log_rows_;
    if (trial_log_rows_ % 10 == 0) {
      trial_log_.flush();
    }
  }

  bool jointStateFresh() const {
    return joint_state_valid_ && !last_joint_state_wall_.isZero() &&
           (ros::WallTime::now() - last_joint_state_wall_).toSec() <=
               joint_state_timeout_;
  }

  bool driverStateFresh() const {
    if (simulation_) {
      return true;
    }
    const ros::WallTime now = ros::WallTime::now();
    return servo_received_ && fault_received_ && !last_servo_wall_.isZero() &&
           !last_fault_wall_.isZero() &&
           (now - last_servo_wall_).toSec() <= driver_state_timeout_ &&
           (now - last_fault_wall_).toSec() <= driver_state_timeout_;
  }

  double maxAbsVelocity() const {
    double result = 0.0;
    for (double velocity : velocities_) {
      result = std::max(result, std::abs(velocity));
    }
    return result;
  }

  bool velocitySafetyExceeded() const {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(velocities_[i]) >
          velocity_hard_limits_[i] * velocity_limit_scale_) {
        return true;
      }
    }
    return false;
  }

  bool queryControllers(ControllerSnapshot& snapshot, std::string& error) {
    controller_manager_msgs::ListControllers service;
    if (!list_client_.call(service)) {
      error = "无法调用 /controller_manager/list_controllers";
      return false;
    }
    for (const auto& controller : service.response.controller) {
      if (controller.name == position_controller_) {
        snapshot.position_loaded = true;
        snapshot.position_running = controller.state == "running";
      }
      if (controller.name == freedrive_controller_) {
        snapshot.freedrive_loaded = true;
        snapshot.freedrive_running = controller.state == "running";
      }
    }
    return true;
  }

  bool ensureFreedriveLoaded(std::string& error) {
    ControllerSnapshot snapshot;
    if (!queryControllers(snapshot, error)) {
      return false;
    }
    if (snapshot.freedrive_loaded) {
      return true;
    }
    controller_manager_msgs::LoadController service;
    service.request.name = freedrive_controller_;
    if (!load_client_.call(service) || !service.response.ok) {
      error = "控制器未预加载，自动 load 也失败";
      return false;
    }
    return true;
  }

  bool switchControllers(const std::vector<std::string>& start,
                         const std::vector<std::string>& stop,
                         std::string& error) {
    controller_manager_msgs::SwitchController service;
    service.request.start_controllers = start;
    service.request.stop_controllers = stop;
    service.request.strictness =
        controller_manager_msgs::SwitchController::Request::STRICT;
    service.request.start_asap = true;
    service.request.timeout = 2.0;
    if (!switch_client_.call(service) || !service.response.ok) {
      error = "controller_manager 严格切换失败";
      return false;
    }
    return true;
  }

  bool enterFreedrive(const std::string& source, std::string& result,
                      bool* retryable = nullptr) {
    if (retryable != nullptr) {
      *retryable = false;
    }
    if (freedrive_active_) {
      result = "零力拖拽已经处于 ACTIVE";
      return true;
    }
    if (transition_busy_ || exit_requested_) {
      result = "控制器正在切换，请等待当前切换结束";
      return false;
    }
    if (refreshIncidentLockout()) {
      result = incidentLockoutDetail();
      setState("INCIDENT_LOCKOUT", result);
      return false;
    }
    if (!simulation_ && !allow_hardware_freedrive_) {
      result = "真机零力拖拽仍被验证锁锁定：allow_hardware_freedrive=false";
      setState("LOCKED", result);
      return false;
    }
    if (!simulation_ && !gravity_calibration_verified_) {
      result = "拒绝进入：多姿态静态重力标定尚未验证；可继续正常点动和记录标定样本";
      setState("CALIBRATION_REQUIRED", result);
      return false;
    }
    if (!payload_model_synchronized_) {
      result = "拒绝进入：当前末端负载模型尚未同步到零力控制器";
      setState("CALIBRATION_REQUIRED", result);
      return false;
    }
    if (!jointStateFresh()) {
      result = "拒绝进入：六轴 joint_states 缺失、不完整或已过期";
      return false;
    }
    if (maxAbsVelocity() > entry_velocity_limit_) {
      result = "拒绝进入：机械臂尚未静止";
      if (retryable != nullptr) {
        *retryable = true;
      }
      return false;
    }
    if (!driverStateFresh()) {
      result = "拒绝进入：Servo/Fault 状态缺失或已过期";
      return false;
    }
    if (!simulation_ && (!servo_enabled_ || faulted_)) {
      result = !servo_enabled_ ? "拒绝进入：当前不是 Servo On"
                              : "拒绝进入：底层 Fault 已锁存";
      return false;
    }
    std::string preflight_detail;
    bool preflight_warning = false;
    if (!evaluateGravityPreflight(preflight_detail, &preflight_warning)) {
      publishValidation(preflight_detail);
      result = "拒绝进入：" + preflight_detail;
      if (retryable != nullptr) {
        *retryable = true;
      }
      setState("READY", result);
      return false;
    }
    publishValidation(preflight_detail);

    transition_busy_ = true;
    setState("ENTERING", source + " 请求切换到零力拖拽");
    std::string error;
    ControllerSnapshot before;
    if (!queryControllers(before, error) ||
        (!simulation_ && !before.position_running) ||
        !ensureFreedriveLoaded(error)) {
      transition_busy_ = false;
      result = error.empty() ? "拒绝进入：位置控制器没有运行" : error;
      setState("ERROR", result);
      return false;
    }
    if (!switchControllers({freedrive_controller_},
                           before.position_running
                               ? std::vector<std::string>{position_controller_}
                               : std::vector<std::string>{},
                           error)) {
      transition_busy_ = false;
      result = error;
      emergencyFallback("进入零力拖拽时切换失败");
      return false;
    }

    ControllerSnapshot after;
    if (!queryControllers(after, error) || !after.freedrive_running ||
        after.position_running) {
      transition_busy_ = false;
      result = "切换服务返回成功，但控制器实际状态不一致";
      emergencyFallback(result);
      return false;
    }

    freedrive_active_ = true;
    transition_busy_ = false;
    exit_requested_ = false;
    exit_protective_ = false;
    position_recovery_pending_ = false;
    stable_exit_samples_ = 0;
    controller_status_received_ = false;
    entered_wall_ = ros::WallTime::now();
    // switch_controller runs inside this node's single-threaded service
    // callback, so joint/driver subscriber callbacks queue behind it. The
    // states were checked immediately before the strict switch; restart their
    // freshness window here and require normal updates within the usual
    // timeout instead of treating callback scheduling as a device dropout.
    last_joint_state_wall_ = entered_wall_;
    last_servo_wall_ = entered_wall_;
    last_fault_wall_ = entered_wall_;
    startTrialLog(source);
    result = preflight_warning
                 ? "零力拖拽 ACTIVE（单姿态预检警告）；请轻扶并观察，不要猛拉"
                 : "零力拖拽 ACTIVE；末端灯环应由固件进入蓝色零力示教状态";
    setState("ACTIVE", result);
    return true;
  }

  bool requestControllerSettle(std::string& error) {
    std_srvs::SetBool service;
    service.request.data = true;
    if (!settle_client_.call(service) || !service.response.success) {
      error = service.response.message.empty()
                  ? "无法请求零力控制器进入增强阻尼减速"
                  : service.response.message;
      return false;
    }
    return true;
  }

  bool beginControlledExit(const std::string& source,
                           const std::string& reason,
                           bool protective,
                           std::string& result) {
    if (!freedrive_active_) {
      result = "零力拖拽已经关闭";
      return true;
    }
    if (!exit_requested_) {
      exit_requested_ = true;
      exit_protective_ = protective;
      stable_exit_samples_ = 0;
      exit_requested_wall_ = ros::WallTime::now();
      last_exit_switch_attempt_ = ros::WallTime();
      exit_reason_ = reason;
    } else if (protective && !exit_protective_) {
      exit_protective_ = true;
      stable_exit_samples_ = 0;
      exit_requested_wall_ = ros::WallTime::now();
      exit_reason_ = reason;
    }
    std::string settle_error;
    const bool settle_requested = requestControllerSettle(settle_error);
    std::ostringstream message;
    message << source << "：" << reason
            << "；保持力矩控制并增强阻尼，静止后切回位置保持";
    if (!settle_requested) {
      message << "；减速请求返回异常：" << settle_error;
    }
    result = message.str();
    setState(protective ? "RECOVERING" : "EXITING", result);
    return true;
  }

  bool requestExit(const std::string& source, std::string& result) {
    if (exit_requested_) {
      result = "已经收到退出请求，正在增强阻尼并等待六轴静止";
      return true;
    }
    return beginControlledExit(source, "正常退出请求", false, result);
  }

  bool completeExit(std::string& result) {
    transition_busy_ = true;
    last_exit_switch_attempt_ = ros::WallTime::now();
    std::string error;
    if (!switchControllers({position_controller_}, {freedrive_controller_},
                           error)) {
      transition_busy_ = false;
      result = "位置控制器切换尚未成功：" + error +
               "；保持增强阻尼并等待下一次重试";
      setState("RECOVERING", result);
      return false;
    }
    ControllerSnapshot after;
    if (!queryControllers(after, error) || !after.position_running ||
        after.freedrive_running) {
      transition_busy_ = false;
      result = "退出切换后控制器实际状态不一致";
      setState("RECOVERING", result + "；保持增强阻尼并重试");
      return false;
    }
    transition_busy_ = false;
    freedrive_active_ = false;
    exit_requested_ = false;
    exit_protective_ = false;
    stable_exit_samples_ = 0;
    preflight_samples_.clear();
    result = "已恢复当前位置保持；零力拖拽关闭";
    setState("READY", result);
    return true;
  }

  bool forceIntermediatePositionHold(const std::string& reason) {
    transition_busy_ = true;
    std::string error;
    if (simulation_) {
      // Gazebo has no EtherCAT drive that can hold the measured position after
      // the effort controller stops. Transfer ownership atomically instead.
      if (!switchControllers({position_controller_}, {freedrive_controller_},
                             error)) {
        transition_busy_ = false;
        return false;
      }
      ControllerSnapshot after;
      std::string verify_error;
      if (!queryControllers(after, verify_error) || !after.position_running ||
          after.freedrive_running) {
        transition_busy_ = false;
        return false;
      }
      transition_busy_ = false;
      freedrive_active_ = false;
      exit_requested_ = false;
      exit_protective_ = false;
      stable_exit_samples_ = 0;
      preflight_samples_.clear();
      position_recovery_pending_ = false;
      setState("READY", reason + "；仿真已原子恢复当前位置控制");
      return true;
    }

    if (!switchControllers({}, {freedrive_controller_}, error)) {
      transition_busy_ = false;
      return false;
    }
    transition_busy_ = false;
    ControllerSnapshot after;
    std::string verify_error;
    if (!queryControllers(after, verify_error) || after.freedrive_running) {
      return false;
    }
    freedrive_active_ = false;
    exit_requested_ = false;
    exit_protective_ = false;
    stable_exit_samples_ = 0;
    preflight_samples_.clear();
    position_recovery_pending_ = !after.position_running;
    setState(position_recovery_pending_ ? "HOLDING" : "READY",
             reason +
                 (position_recovery_pending_
                      ? "；已切到驱动器当前位置保持，正在恢复 ROS 位置控制器"
                      : "；已恢复位置控制"));
    return true;
  }

  void recoverPositionController() {
    ControllerSnapshot snapshot;
    std::string error;
    if (!queryControllers(snapshot, error)) {
      setState("HOLDING", "驱动器正在当前位置保持；暂时无法查询控制器状态");
      return;
    }
    if (snapshot.position_running && !snapshot.freedrive_running) {
      position_recovery_pending_ = false;
      setState("READY", "ROS 位置控制器已经恢复");
      return;
    }
    if (!simulation_ && (!driverStateFresh() || !servo_enabled_ || faulted_)) {
      position_recovery_pending_ = false;
      updateIdleState();
      return;
    }
    if (!jointStateFresh() || maxAbsVelocity() > exit_velocity_limit_) {
      setState("HOLDING", "驱动器当前位置保持中；等待编码器确认静止");
      return;
    }
    if (!last_exit_switch_attempt_.isZero() &&
        (ros::WallTime::now() - last_exit_switch_attempt_).toSec() < 0.25) {
      return;
    }
    last_exit_switch_attempt_ = ros::WallTime::now();
    if (!switchControllers({position_controller_}, {}, error)) {
      setState("HOLDING",
               "驱动器保持正常，但 ROS 位置控制器恢复失败，稍后重试：" +
                   error);
      return;
    }
    position_recovery_pending_ = false;
    setState("READY", "已从驱动器临时保持恢复 ROS 位置控制器");
  }

  void emergencyFallback(const std::string& reason) {
    if (transition_busy_) {
      transition_busy_ = false;
    }
    if (freedrive_active_ && jointStateFresh() && driverStateFresh() &&
        (simulation_ || (servo_enabled_ && !faulted_))) {
      std::string result;
      beginControlledExit("保护回退", reason, true, result);
      return;
    }
    setState("FALLBACK", reason + "；正在执行保护回退");
    std::string switch_error;
    ControllerSnapshot snapshot;
    if (queryControllers(snapshot, switch_error) &&
        snapshot.position_running && !snapshot.freedrive_running) {
      freedrive_active_ = false;
      exit_requested_ = false;
      exit_protective_ = false;
      preflight_samples_.clear();
      setState("READY", reason + "；已回退到当前位置保持");
      return;
    }
    const std::vector<std::string> start =
        snapshot.position_loaded && !snapshot.position_running
            ? std::vector<std::string>{position_controller_}
            : std::vector<std::string>{};
    const std::vector<std::string> stop =
        snapshot.freedrive_running
            ? std::vector<std::string>{freedrive_controller_}
            : std::vector<std::string>{};
    if ((!start.empty() || !stop.empty()) &&
        switchControllers(start, stop, switch_error)) {
      ControllerSnapshot after;
      std::string verify_error;
      if (queryControllers(after, verify_error) && after.position_running &&
          !after.freedrive_running) {
        freedrive_active_ = false;
        exit_requested_ = false;
        exit_protective_ = false;
        preflight_samples_.clear();
        setState("READY", reason + "；已回退到当前位置保持");
        return;
      }
    }

    if (snapshot.freedrive_running &&
        forceIntermediatePositionHold(reason + "；直接切回位置控制失败")) {
      return;
    }

    if (!simulation_) {
      std_srvs::SetBool disable;
      disable.request.data = true;
      const bool disabled =
          raw_disable_client_.call(disable) && disable.response.success;
      std::string ignored;
      switchControllers({}, {freedrive_controller_}, ignored);
      freedrive_active_ = false;
      exit_requested_ = false;
      exit_protective_ = false;
      preflight_samples_.clear();
      setState("ERROR",
               reason + (disabled
                             ? "；位置保持失败，已确认 Servo Off"
                             : "；位置保持与 Servo Off 均未确认，请立即断上游电闸"));
      return;
    }

    std::string ignored;
    switchControllers({}, {freedrive_controller_}, ignored);
    freedrive_active_ = false;
    exit_requested_ = false;
    exit_protective_ = false;
    preflight_samples_.clear();
    setState("ERROR", reason + "；仿真 effort 控制器已请求停止");
  }

  bool setFreedriveCallback(std_srvs::SetBool::Request& request,
                            std_srvs::SetBool::Response& response) {
    std::string result;
    response.success = request.data
                           ? enterFreedrive("软件服务", result)
                           : requestExit("软件服务", result);
    response.message = result;
    publishState();
    return true;
  }

  bool setVelocityScaleCallback(
      elfin_robot_msgs::SetFloat64::Request& request,
      elfin_robot_msgs::SetFloat64::Response& response) {
    if (!math::velocityScaleValid(request.data,
                                  minimum_velocity_limit_scale_,
                                  maximum_velocity_limit_scale_)) {
      response.success = false;
      std::ostringstream message;
      message << "拖拽速度倍率必须在 "
              << minimum_velocity_limit_scale_ * 100.0 << "% 到 "
              << maximum_velocity_limit_scale_ * 100.0 << "% 之间";
      response.message = message.str();
      return true;
    }
    if (freedrive_active_ || transition_busy_ || exit_requested_ ||
        position_recovery_pending_) {
      response.success = false;
      response.message = "正在拖拽或切换控制器；请退出到 READY 后再修改";
      return true;
    }

    elfin_robot_msgs::SetFloat64 controller_service;
    controller_service.request.data = request.data;
    if (!velocity_scale_client_.call(controller_service)) {
      response.success = false;
      response.message = "零力控制器速度设置服务不可用；请确认硬件启动脚本已运行";
      return true;
    }
    if (!controller_service.response.success) {
      response.success = false;
      response.message = "零力控制器拒绝修改：" +
                         controller_service.response.message;
      return true;
    }

    velocity_limit_scale_ = request.data;
    private_nh_.setParam("velocity_limit_scale", velocity_limit_scale_);
    root_nh_.setParam("/elfin_freedrive_controller/velocity_limit_scale",
                      velocity_limit_scale_);
    publishVelocityLimits();

    std::ostringstream message;
    message << std::fixed << std::setprecision(1)
            << "拖拽速度倍率已设为 " << velocity_limit_scale_ * 100.0
            << "%；六轴硬上限约为 ";
    constexpr double kDegreesPerRadian = 57.29577951308232;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (i != 0) {
        message << ", ";
      }
      message << "J" << i + 1 << " "
              << velocity_hard_limits_[i] * velocity_limit_scale_ *
                     kDegreesPerRadian
              << " deg/s";
    }
    message << "。下一次进入零力拖拽时使用；其他保护不变";
    response.success = true;
    response.message = message.str();
    return true;
  }

  bool setDampingScalesCallback(
      SetDampingScales::Request& request,
      SetDampingScales::Response& response) {
    if (request.scales.size() != kJointCount) {
      response.success = false;
      response.message = "必须提供 J1--J6 共六个阻尼倍率";
      return true;
    }
    for (double scale : request.scales) {
      if (!math::velocityScaleValid(scale, minimum_damping_scale_,
                                    maximum_damping_scale_)) {
        response.success = false;
        response.message = "每轴阻尼倍率必须在 5% 到 500% 之间";
        return true;
      }
    }
    if (freedrive_active_ || transition_busy_ || exit_requested_ ||
        position_recovery_pending_) {
      response.success = false;
      response.message = "正在拖拽或切换控制器；请退出到 READY 后再修改阻尼";
      return true;
    }

    SetDampingScales controller_service;
    controller_service.request.scales = request.scales;
    if (!damping_scales_client_.call(controller_service)) {
      response.success = false;
      response.message = "零力控制器阻尼设置服务不可用；请确认硬件启动脚本已运行";
      return true;
    }
    if (!controller_service.response.success) {
      response.success = false;
      response.message = "零力控制器拒绝修改：" +
                         controller_service.response.message;
      return true;
    }

    damping_scales_ = request.scales;
    private_nh_.setParam("damping_scales", damping_scales_);
    root_nh_.setParam("/elfin_freedrive_controller/damping_scales",
                      damping_scales_);
    publishDampingScales();
    response.applied_scales = damping_scales_;
    std::ostringstream message;
    message << "六轴拖拽阻尼倍率已更新：";
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (i != 0) {
        message << "，";
      }
      message << "J" << i + 1 << "="
              << std::fixed << std::setprecision(0)
              << damping_scales_[i] * 100.0 << "%";
    }
    message << "；下一次进入零力拖拽时生效";
    response.success = true;
    response.message = message.str();
    return true;
  }

  bool recordPointCallback(std_srvs::Trigger::Request& request,
                           std_srvs::Trigger::Response& response) {
    (void)request;
    response.success = recordPoint("软件服务", response.message);
    return true;
  }

  bool recordPoint(const std::string& source, std::string& result) {
    if (!jointStateFresh()) {
      result = "记录失败：六轴 joint_states 不完整或已过期";
      return false;
    }

    const std::ios::openmode write_mode =
        std::ios::out |
        (point_count_ == 0 ? std::ios::trunc : std::ios::app);
    std::ofstream output(record_file_, write_mode);
    if (!output) {
      result = "记录失败：无法写入 " + record_file_;
      return false;
    }
    if (point_count_ == 0) {
      output << "# Elfin freedrive POINT records. Joint values are radians.\n";
    }
    ++point_count_;
    output << std::setprecision(17);
    output << "- index: " << point_count_ << "\n";
    output << "  stamp: " << ros::Time::now().toSec() << "\n";
    output << "  source: \"" << source << "\"\n";
    output << "  joints_rad:\n";
    for (std::size_t i = 0; i < kJointCount; ++i) {
      output << "    " << expected_joint_names_[i] << ": "
             << positions_[i] << "\n";
    }
    output << "  measured_effort_nm:\n";
    for (std::size_t i = 0; i < kJointCount; ++i) {
      output << "    " << expected_joint_names_[i] << ": "
             << efforts_[i] << "\n";
    }
    if (telemetry_valid_ && !last_telemetry_wall_.isZero() &&
        (ros::WallTime::now() - last_telemetry_wall_).toSec() <= 0.5) {
      output << "  model_gravity_nm:\n";
      for (std::size_t i = 0; i < kJointCount; ++i) {
        output << "    " << expected_joint_names_[i] << ": "
               << last_telemetry_.gravity_effort[i] << "\n";
      }
    }
    output.flush();
    if (!output) {
      --point_count_;
      result = "记录失败：写入过程中发生 I/O 错误";
      return false;
    }

    sensor_msgs::JointState point;
    point.header.stamp = ros::Time::now();
    point.name = expected_joint_names_;
    point.position.assign(positions_.begin(), positions_.end());
    recorded_point_pub_.publish(point);
    publishPointCount();

    std::ostringstream message;
    message << source << " 已记录第 " << point_count_
            << " 个姿态到 " << record_file_;
    result = message.str();
    state_detail_ = result;
    publishState();
    return true;
  }

  bool loadRecordedPoints(YAML::Node& records, std::string& error) const {
    std::ifstream input(record_file_);
    if (!input) {
      records = YAML::Node(YAML::NodeType::Sequence);
      return true;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    std::string yaml_text = contents.str();
    std::string normalized;
    const bool recovered_legacy =
        normalizeLegacyPointYaml(yaml_text, normalized);
    if (recovered_legacy) {
      yaml_text = normalized;
    }
    try {
      records = YAML::Load(yaml_text);
    } catch (const YAML::Exception& exception) {
      error = "姿态文件 YAML 解析失败：" + std::string(exception.what());
      return false;
    }
    if (!records || records.IsNull()) {
      records = YAML::Node(YAML::NodeType::Sequence);
    }
    if (!records.IsSequence()) {
      error = "姿态文件根节点不是 YAML 列表：" + record_file_;
      return false;
    }
    bool renumbered = false;
    try {
      for (std::size_t row = 0; row < records.size(); ++row) {
        const std::uint32_t expected = static_cast<std::uint32_t>(row + 1);
        const YAML::Node index = records[row]["index"];
        if (index && index.as<std::uint32_t>() != expected) {
          records[row]["index"] = expected;
          renumbered = true;
        }
      }
    } catch (const YAML::Exception& exception) {
      error = "姿态序号解析失败：" + std::string(exception.what());
      return false;
    }
    if (recovered_legacy) {
      ROS_WARN_STREAM_ONCE(
          "Recovered exact legacy artifacts in POINT YAML: " << record_file_);
    }
    if (renumbered) {
      ROS_WARN_STREAM_ONCE(
          "Normalized non-contiguous POINT indices in memory: " << record_file_);
    }
    return true;
  }

  bool listRecordedPointsCallback(
      ListRecordedPoints::Request& request,
      ListRecordedPoints::Response& response) {
    (void)request;
    response.record_file = record_file_;
    YAML::Node records;
    if (!loadRecordedPoints(records, response.message)) {
      response.success = false;
      return true;
    }

    try {
      for (std::size_t row = 0; row < records.size(); ++row) {
        const YAML::Node record = records[row];
        const YAML::Node joints = record["joints_rad"];
        if (!record["index"] || !record["stamp"] || !joints ||
            !joints.IsMap()) {
          std::ostringstream message;
          message << "姿态文件第 " << row + 1 << " 条记录缺少必要字段";
          response.success = false;
          response.message = message.str();
          return true;
        }
        response.indices.push_back(record["index"].as<std::uint32_t>());
        response.stamps.push_back(record["stamp"].as<double>());
        response.sources.push_back(
            record["source"] ? record["source"].as<std::string>() : "未知来源");
        for (const std::string& joint_name : expected_joint_names_) {
          if (!joints[joint_name]) {
            response.success = false;
            response.message = "姿态记录缺少关节字段：" + joint_name;
            return true;
          }
          response.joints_rad.push_back(joints[joint_name].as<double>());
        }
      }
    } catch (const YAML::Exception& exception) {
      response.success = false;
      response.message = "姿态字段解析失败：" + std::string(exception.what());
      return true;
    }

    response.success = true;
    std::ostringstream message;
    message << "已读取 " << records.size() << " 个姿态";
    response.message = message.str();
    return true;
  }

  bool writeRecordedPointsAtomically(const YAML::Node& records,
                                     std::string& error) const {
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << records;
    if (!emitter.good()) {
      error = "无法生成姿态 YAML：" + emitter.GetLastError();
      return false;
    }

    const std::string temporary_file = record_file_ + ".tmp";
    std::ofstream output(temporary_file, std::ios::out | std::ios::trunc);
    if (!output) {
      error = "无法创建姿态临时文件：" + temporary_file;
      return false;
    }
    output << "# Elfin freedrive POINT records. Joint values are radians.\n";
    output << emitter.c_str() << "\n";
    output.flush();
    if (!output) {
      output.close();
      std::remove(temporary_file.c_str());
      error = "写入姿态临时文件时发生 I/O 错误";
      return false;
    }
    output.close();
    if (std::rename(temporary_file.c_str(), record_file_.c_str()) != 0) {
      const int rename_error = errno;
      std::remove(temporary_file.c_str());
      error = "无法原子替换姿态文件：" +
              std::string(std::strerror(rename_error));
      return false;
    }
    return true;
  }

  bool deleteRecordedPointCallback(
      DeleteRecordedPoint::Request& request,
      DeleteRecordedPoint::Response& response) {
    YAML::Node records;
    if (!loadRecordedPoints(records, response.message)) {
      response.success = false;
      response.remaining_count = point_count_;
      return true;
    }

    if (request.index == 0) {
      YAML::Node empty_records(YAML::NodeType::Sequence);
      if (!writeRecordedPointsAtomically(empty_records, response.message)) {
        response.success = false;
        response.remaining_count = point_count_;
        return true;
      }
      const std::size_t deleted_count = records.size();
      point_count_ = 0;
      publishPointCount();
      response.success = true;
      response.remaining_count = 0;
      response.message = "已删除全部 " + std::to_string(deleted_count) +
                         " 个 POINT 姿态";
      state_detail_ = response.message;
      publishState();
      return true;
    }

    YAML::Node retained(YAML::NodeType::Sequence);
    bool found = false;
    try {
      for (std::size_t row = 0; row < records.size(); ++row) {
        const YAML::Node record = records[row];
        const std::uint32_t index =
            record["index"] ? record["index"].as<std::uint32_t>() : 0;
        if (index == request.index) {
          found = true;
          continue;
        }
        YAML::Node copy = YAML::Clone(record);
        copy["index"] = static_cast<std::uint32_t>(retained.size() + 1);
        retained.push_back(copy);
      }
    } catch (const YAML::Exception& exception) {
      response.success = false;
      response.message = "删除前解析姿态失败：" +
                         std::string(exception.what());
      response.remaining_count = point_count_;
      return true;
    }
    if (!found) {
      response.success = false;
      response.message = "未找到序号为 " + std::to_string(request.index) +
                         " 的姿态，请先刷新列表";
      response.remaining_count = point_count_;
      return true;
    }
    if (!writeRecordedPointsAtomically(retained, response.message)) {
      response.success = false;
      response.remaining_count = point_count_;
      return true;
    }

    point_count_ = static_cast<std::uint32_t>(retained.size());
    publishPointCount();
    response.success = true;
    response.remaining_count = point_count_;
    std::ostringstream message;
    message << "已删除姿态 " << request.index << "，剩余 "
            << point_count_ << " 个；其余序号已连续重排";
    response.message = message.str();
    state_detail_ = response.message;
    publishState();
    return true;
  }

  std::uint32_t countExistingPoints() const {
    std::ifstream input(record_file_);
    if (!input) {
      return 0;
    }
    std::uint32_t count = 0;
    std::string line;
    while (std::getline(input, line)) {
      if (line.compare(0, 9, "- index: ") == 0) {
        ++count;
      }
    }
    return count;
  }

  void publishPointCount() {
    std_msgs::UInt32 message;
    message.data = point_count_;
    point_count_pub_.publish(message);
  }

  void publishVelocityLimits() {
    std_msgs::Float64 scale;
    scale.data = velocity_limit_scale_;
    velocity_scale_pub_.publish(scale);
    std_msgs::Float64MultiArray hard_limits;
    hard_limits.data.reserve(kJointCount);
    for (double nominal_limit : velocity_hard_limits_) {
      hard_limits.data.push_back(nominal_limit * velocity_limit_scale_);
    }
    velocity_hard_limits_pub_.publish(hard_limits);
  }

  void publishDampingScales() {
    std_msgs::Float64MultiArray message;
    message.data = damping_scales_;
    damping_scales_pub_.publish(message);
  }

  void buttonTimerCallback(const ros::WallTimerEvent& event) {
    (void)event;
    if (!poll_tool_buttons_ || simulation_) {
      return;
    }
    elfin_robot_msgs::ElfinIODRead service;
    service.request.data = true;
    std_msgs::Bool online;
    if (!read_di_client_.call(service)) {
      online.data = false;
      button_online_pub_.publish(online);
      const ToolButtonLogic::Events lost = button_logic_.inputUnavailable();
      physical_free_entry_pending_ = false;
      if (lost.free_released && (freedrive_active_ || exit_requested_)) {
        std::string result;
        beginControlledExit("实体 FREE 输入丢失",
                            "DI bit 5 状态无法继续确认", true, result);
      }
      return;
    }
    online.data = true;
    button_online_pub_.publish(online);
    const uint16_t raw =
        static_cast<uint16_t>(service.response.digital_input & 0xffff);
    std_msgs::UInt16 raw_message;
    raw_message.data = raw;
    raw_di_pub_.publish(raw_message);

    const ToolButtonLogic::Events events =
        button_logic_.update(raw, steadySeconds());
    const bool free_raw_high =
        ((raw >> ToolButtonLogic::kFreeBit) & 0x1U) != 0;
    if (events.free_short_pulse) {
      std::ostringstream detail;
      detail << "已忽略实体 FREE 短脉冲：观测高电平 " << std::fixed
             << std::setprecision(3) << events.free_high_seconds << "/"
             << button_logic_.requiredFreePressSeconds() << " 秒，"
             << events.free_high_samples << "/"
             << button_logic_.requiredFreePressSamples()
             << " 个有效高电平样本；未切入力矩模式";
      ROS_WARN_STREAM(detail.str());
      setState(state_, detail.str());
    }
    if (events.point_pressed) {
      std::string result;
      const bool success = recordPoint("实体 POINT (DI bit 4)", result);
      if (!success) {
        setState(state_, result);
      }
    }
    if (events.free_released) {
      physical_free_entry_pending_ = false;
      if (freedrive_active_ || exit_requested_) {
        std::string result;
        beginControlledExit("实体 FREE 松开", "实体保持输入已释放", true,
                            result);
      }
    }
    const double now_seconds = steadySeconds();
    const bool should_attempt_entry =
        events.free_pressed ||
        (physical_free_entry_pending_ && events.free_confirmed_held &&
         free_raw_high &&
         now_seconds - last_physical_free_entry_attempt_seconds_ >= 0.20);
    if (should_attempt_entry) {
      last_physical_free_entry_attempt_seconds_ = now_seconds;
      std::string result;
      bool retryable = false;
      const bool success = enterFreedrive("实体 FREE (DI bit 5)", result,
                                          &retryable);
      physical_free_entry_pending_ = !success && retryable &&
                                     events.free_confirmed_held &&
                                     free_raw_high;
      if (!success) {
        setState(state_, result);
      }
    }
  }

  void monitorTimerCallback(const ros::WallTimerEvent& event) {
    (void)event;
    const ros::WallTime monitor_now = ros::WallTime::now();
    if (!payload_model_synchronized_ && !freedrive_active_ &&
        !transition_busy_ &&
        (last_payload_sync_attempt_.isZero() ||
         (monitor_now - last_payload_sync_attempt_).toSec() >= 1.0)) {
      last_payload_sync_attempt_ = monitor_now;
      synchronizePayloadModel();
    }
    if (position_recovery_pending_) {
      recoverPositionController();
      publishState();
      return;
    }
    if (freedrive_active_) {
      if (refreshIncidentLockout() &&
          (!exit_requested_ || !exit_protective_)) {
        std::string result;
        beginControlledExit("事故锁", incidentLockoutDetail(), true, result);
        return;
      }
      if (!jointStateFresh()) {
        emergencyFallback("零力拖拽期间 joint_states 过期");
        return;
      }
      if (!driverStateFresh()) {
        emergencyFallback("零力拖拽期间 Servo 状态丢失或已关闭");
        return;
      }
      if (!simulation_ && (!servo_enabled_ || faulted_)) {
        const std::string reason =
            faulted_ ? "零力拖拽期间底层 Fault 触发"
                     : "零力拖拽期间驱动器已 Servo Off";
        if (!forceIntermediatePositionHold(reason)) {
          emergencyFallback(reason);
        }
        return;
      }
      if (velocitySafetyExceeded() &&
          (!exit_requested_ || !exit_protective_)) {
        std::string result;
        beginControlledExit("速度保护", "零力拖拽速度超过管理器硬上限",
                            true, result);
        return;
      }
      if (controller_status_received_ &&
          (controller_status_ == kStatusSolverError ||
           controller_status_ == kStatusSafetyStop ||
           controller_status_ == kStatusHardLimit ||
           controller_status_ == kStatusOverspeed ||
           controller_status_ == kStatusModelMismatch ||
           controller_status_ == kStatusGravityCapacity) &&
          (!exit_requested_ || !exit_protective_)) {
        std::string reason;
        if (controller_status_ == kStatusSolverError) {
          reason = "重力求解器或关节状态异常";
        } else if (controller_status_ == kStatusModelMismatch) {
          reason = "入口保持力矩与重力模型方向或量级不一致";
        } else if (controller_status_ == kStatusGravityCapacity) {
          reason = "当前姿态所需重力补偿超过零力控制器的保留容量";
        } else if (controller_status_ == kStatusHardLimit) {
          reason = "控制器检测到关节继续向硬限位运动";
        } else if (controller_status_ == kStatusOverspeed) {
          reason = "控制器检测到关节速度超过硬上限";
        } else {
          reason = "控制器触发通用安全停止";
        }
        std::string result;
        beginControlledExit("控制器保护", reason, true, result);
        return;
      }
      const ros::WallTime now = ros::WallTime::now();
      if ((!controller_status_received_ &&
           (now - entered_wall_).toSec() > controller_status_timeout_) ||
          (controller_status_received_ &&
           (now - last_controller_status_wall_).toSec() >
               controller_status_timeout_)) {
        emergencyFallback("零力控制器状态话题过期");
        return;
      }

      if (exit_requested_) {
        if (maxAbsVelocity() <= exit_velocity_limit_) {
          ++stable_exit_samples_;
        } else {
          stable_exit_samples_ = 0;
        }
        if (stable_exit_samples_ >=
                static_cast<unsigned int>(exit_stable_samples_required_) &&
            (last_exit_switch_attempt_.isZero() ||
             (now - last_exit_switch_attempt_).toSec() >= 0.25)) {
          std::string result;
          completeExit(result);
          return;
        }
        const double settle_timeout = exit_protective_
                                          ? protective_exit_settle_timeout_
                                          : exit_settle_timeout_;
        if ((now - exit_requested_wall_).toSec() > settle_timeout) {
          if (!forceIntermediatePositionHold(
                  exit_reason_.empty()
                      ? "增强阻尼后仍未达到严格静止阈值"
                      : exit_reason_ + "；增强阻尼后仍未达到严格静止阈值")) {
            emergencyFallback("受控减速超时且驱动器当前位置保持切换失败");
          }
          return;
        }
      }
    } else if (!transition_busy_) {
      updateIdleState();
      maybePublishPassivePreflight();
    }
    publishState();
  }

  void updateIdleState() {
    if (!idle_initialized_) {
      idle_initialized_ = true;
      ControllerSnapshot snapshot;
      std::string error;
      if (queryControllers(snapshot, error) && snapshot.freedrive_running) {
        emergencyFallback("管理器启动时发现遗留的零力控制器仍在运行");
        return;
      }
    }
    if (refreshIncidentLockout()) {
      setState("INCIDENT_LOCKOUT", incidentLockoutDetail());
    } else if (!simulation_ && !allow_hardware_freedrive_) {
      setState("LOCKED",
               "零力控制器已加载但真机入口锁定；POINT 记录仍可使用");
    } else if (!simulation_ && !gravity_calibration_verified_) {
      std::ostringstream detail;
      detail << "需要多姿态静态重力标定；已记录 "
             << calibration_samples_.size() << "/"
             << calibration_min_samples_ << " 个位置模式样本";
      setState("CALIBRATION_REQUIRED", detail.str());
    } else if (!jointStateFresh()) {
      setState("WAITING", "等待完整且新鲜的六轴 joint_states");
    } else if (!driverStateFresh()) {
      setState("WAITING", "等待 Servo/Fault 状态");
    } else if (!simulation_ && faulted_) {
      setState("FAULT", "底层 Fault 已锁存，不能进入零力拖拽");
    } else if (!simulation_ && !servo_enabled_) {
      setState("SERVO_OFF", "Servo Off；先通过正常流程 Servo On");
    } else {
      ControllerSnapshot snapshot;
      std::string error;
      if (state_ != "READY" &&
          (!queryControllers(snapshot, error) ||
           !snapshot.position_running || snapshot.freedrive_running)) {
        setState("WAITING",
                 error.empty()
                     ? "Servo On，但六轴位置控制器尚未运行"
                     : "等待控制器状态：" + error);
      } else {
        setState("READY", "位置控制在线；按下 FREE 即进入零力拖拽，松开退出");
      }
    }
  }

  void setState(const std::string& state, const std::string& detail) {
    if (state_ == state && state_detail_ == detail) {
      return;
    }
    state_ = state;
    state_detail_ = detail;
    ROS_WARN_STREAM("Freedrive manager state=" << state_ << ": "
                    << state_detail_);
    if (trial_log_.is_open()) {
      trial_log_ << "# event," << std::setprecision(17)
                 << ros::WallTime::now().toSec() << "," << state_ << ","
                 << state_detail_ << "\n";
      trial_log_.flush();
    }
    publishState();
  }

  void publishState() {
    std_msgs::String state_message;
    state_message.data = state_;
    state_pub_.publish(state_message);
    std_msgs::String detail_message;
    detail_message.data = state_detail_;
    state_detail_pub_.publish(detail_message);
    std_msgs::Bool active_message;
    active_message.data = freedrive_active_;
    active_pub_.publish(active_message);
    publishRingState();
  }

  void publishRingState() {
    std_msgs::String state;
    std_msgs::ColorRGBA color;
    color.a = 1.0F;
    if ((!simulation_ && faulted_) || state_ == "ERROR" ||
        state_ == "FAULT") {
      state.data = "RED_FAULT_EXPECTED";
      color.r = 1.0F;
    } else if (freedrive_active_ || state_ == "ENTERING" ||
               state_ == "EXITING") {
      state.data = "BLUE_ZERO_FORCE_EXPECTED";
      color.b = 1.0F;
    } else if (!simulation_ && servo_received_ && !servo_enabled_) {
      state.data = "YELLOW_SERVO_OFF_EXPECTED";
      color.r = 1.0F;
      color.g = 0.85F;
    } else if (simulation_ || servo_enabled_) {
      state.data = "GREEN_SERVO_ON_EXPECTED";
      color.g = 1.0F;
    } else {
      state.data = "UNKNOWN_RING_STATE";
      color.a = 0.25F;
    }
    ring_state_pub_.publish(state);
    ring_color_pub_.publish(color);
  }

  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;

  bool simulation_;
  bool allow_hardware_freedrive_;
  bool poll_tool_buttons_;
  double free_press_hold_seconds_;
  double joint_state_timeout_;
  double driver_state_timeout_;
  double controller_status_timeout_;
  double entry_velocity_limit_;
  double velocity_limit_scale_;
  double minimum_velocity_limit_scale_;
  double maximum_velocity_limit_scale_;
  double exit_velocity_limit_;
  double exit_settle_timeout_;
  double protective_exit_settle_timeout_;
  int exit_stable_samples_required_;
  unsigned int preflight_min_samples_;
  double preflight_min_duration_;
  double preflight_window_;
  double preflight_velocity_limit_;
  double preflight_position_tolerance_;
  double preflight_max_effort_stddev_;
  double preflight_max_model_error_;
  bool allow_model_validation_warning_;
  double minimum_warning_alignment_;
  double minimum_damping_scale_;
  double maximum_damping_scale_;
  unsigned int calibration_min_samples_;
  unsigned int calibration_min_paired_poses_;
  double calibration_min_pose_separation_;
  double calibration_min_model_change_;
  double calibration_min_approach_delta_;
  double calibration_pair_pose_tolerance_;
  double calibration_min_model_range_;
  double calibration_min_scale_;
  double calibration_max_scale_;
  double calibration_max_normalized_residual_;
  double calibration_max_absolute_residual_;
  std::string position_controller_;
  std::string freedrive_controller_;
  std::string record_file_;
  std::string trial_log_directory_;
  std::string calibration_samples_file_;
  std::string calibration_candidate_file_;
  std::string payload_profile_file_;
  std::string freedrive_lockout_file_;
  std::vector<std::string> expected_joint_names_;
  std::vector<double> velocity_hard_limits_;
  std::vector<double> damping_scales_;

  bool gravity_model_ready_;
  bool gravity_calibration_verified_;
  double maximum_payload_mass_;
  payload::Model payload_model_;
  std::string payload_profile_name_;
  double payload_fit_rmse_;
  double payload_fit_max_error_;
  double payload_validation_drift_;
  std::uint32_t payload_sample_count_;
  bool payload_model_synchronized_;
  bool controlled_hold_validation_;
  bool payload_hold_verified_;
  bool incident_lockout_latched_;
  std::string gravity_model_error_;
  KDL::Chain gravity_chain_;
  std::unique_ptr<KDL::ChainDynParam> gravity_dynamics_;
  std::unique_ptr<KDL::ChainJntToJacSolver> payload_jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> payload_fk_solver_;
  KDL::Vector gravity_vector_;
  KDL::JntArray gravity_q_;
  KDL::JntArray gravity_effort_;
  KDL::Jacobian payload_jacobian_;
  payload::Regressor payload_regressor_;
  payload::JointEffort current_payload_effort_;
  std::vector<std::size_t> gravity_joint_to_chain_;
  std::vector<double> gravity_joint_scales_;
  std::vector<double> gravity_bias_;
  std::vector<double> model_effort_limits_;
  double model_gravity_scale_;
  double model_maximum_gravity_effort_fraction_;
  bool model_adaptive_entry_scale_;
  double model_minimum_adaptive_scale_;
  double model_maximum_adaptive_scale_;
  double model_minimum_validation_effort_;
  unsigned int model_minimum_validation_joints_;
  double model_minimum_alignment_;
  double model_minimum_scale_;
  double model_maximum_scale_;
  double model_maximum_residual_;

  std::array<double, kJointCount> positions_;
  std::array<double, kJointCount> velocities_;
  std::array<double, kJointCount> efforts_;
  bool joint_state_valid_;
  bool effort_state_valid_;
  bool servo_received_;
  bool servo_enabled_;
  bool fault_received_;
  bool faulted_;
  bool telemetry_valid_;
  FreedriveTelemetry last_telemetry_;
  bool controller_status_received_;
  uint8_t controller_status_;
  bool freedrive_active_;
  bool exit_requested_;
  bool exit_protective_;
  bool transition_busy_;
  bool position_recovery_pending_;
  bool physical_free_entry_pending_;
  double last_physical_free_entry_attempt_seconds_;
  bool idle_initialized_;
  unsigned int stable_exit_samples_;
  std::uint64_t trial_log_rows_;
  std::uint32_t point_count_;
  std::string state_;
  std::string state_detail_;
  std::string exit_reason_;
  std::string trial_log_path_;
  std::ofstream trial_log_;
  ToolButtonLogic button_logic_;
  std::deque<StaticEffortSample> preflight_samples_;
  std::vector<GravityCalibrationSample> calibration_samples_;

  ros::WallTime last_joint_state_wall_;
  ros::WallTime last_servo_wall_;
  ros::WallTime last_fault_wall_;
  ros::WallTime last_controller_status_wall_;
  ros::WallTime last_telemetry_wall_;
  ros::WallTime entered_wall_;
  ros::WallTime exit_requested_wall_;
  ros::WallTime last_exit_switch_attempt_;
  ros::WallTime last_preflight_publish_wall_;
  ros::WallTime last_payload_sync_attempt_;

  ros::ServiceClient switch_client_;
  ros::ServiceClient list_client_;
  ros::ServiceClient load_client_;
  ros::ServiceClient read_di_client_;
  ros::ServiceClient raw_disable_client_;
  ros::ServiceClient settle_client_;
  ros::ServiceClient velocity_scale_client_;
  ros::ServiceClient damping_scales_client_;
  ros::ServiceClient payload_model_client_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber servo_sub_;
  ros::Subscriber fault_sub_;
  ros::Subscriber controller_status_sub_;
  ros::Subscriber telemetry_sub_;
  ros::Publisher state_pub_;
  ros::Publisher state_detail_pub_;
  ros::Publisher ring_state_pub_;
  ros::Publisher ring_color_pub_;
  ros::Publisher active_pub_;
  ros::Publisher raw_di_pub_;
  ros::Publisher button_online_pub_;
  ros::Publisher recorded_point_pub_;
  ros::Publisher point_count_pub_;
  ros::Publisher validation_pub_;
  ros::Publisher trial_log_pub_;
  ros::Publisher velocity_scale_pub_;
  ros::Publisher velocity_hard_limits_pub_;
  ros::Publisher damping_scales_pub_;
  ros::Publisher payload_profile_pub_;
  ros::Publisher payload_model_pub_;
  ros::ServiceServer set_freedrive_server_;
  ros::ServiceServer record_point_server_;
  ros::ServiceServer list_recorded_points_server_;
  ros::ServiceServer delete_recorded_point_server_;
  ros::ServiceServer record_gravity_sample_server_;
  ros::ServiceServer fit_gravity_calibration_server_;
  ros::ServiceServer set_velocity_scale_server_;
  ros::ServiceServer set_damping_scales_server_;
  ros::ServiceServer set_payload_model_server_;
  ros::ServiceServer get_payload_model_server_;
  ros::ServiceServer evaluate_payload_model_server_;
  ros::WallTimer button_timer_;
  ros::WallTimer monitor_timer_;
};

}  // namespace elfin_freedrive_controller

int main(int argc, char** argv) {
  ros::init(argc, argv, "elfin_freedrive_manager");
  try {
    elfin_freedrive_controller::ElfinFreedriveManager manager;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("elfin_freedrive_manager stopped: " << error.what());
    return 1;
  }
  return 0;
}
