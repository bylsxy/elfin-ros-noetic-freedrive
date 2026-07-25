#ifndef ELFIN_FREEDRIVE_CONTROLLER_H
#define ELFIN_FREEDRIVE_CONTROLLER_H

#include <elfin_freedrive_controller/FreedriveTelemetry.h>
#include <elfin_freedrive_controller/SetDampingScales.h>
#include <elfin_robot_msgs/SetFloat64.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <urdf/model.h>

#include <atomic>
#include <array>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace elfin_freedrive_controller {

// Values are deliberately stable: the manager and the Panel use them as a
// small, dependency-free safety state machine.
enum Status : uint8_t {
  kStatusInactive = 0,
  kStatusActive = 1,
  kStatusLimit = 2,
  kStatusSolverError = 3,
  kStatusSafetyStop = 4,
  kStatusModelMismatch = 5,
  kStatusSettling = 6,
  kStatusModelWarning = 7,
  kStatusGravityCapacity = 8
};

class ElfinFreedriveController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  ElfinFreedriveController();
  ~ElfinFreedriveController() override;

  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  bool getVectorParam(const ros::NodeHandle& nh,
                      const std::string& name,
                      std::size_t expected_size,
                      const std::vector<double>& default_value,
                      std::vector<double>& value) const;
  bool finiteVector(const std::vector<double>& value) const;
  void publishStatus(const ros::Time& time, uint8_t status);
  void publishCommandState(const ros::Time& time);
  void publishTelemetry(const ros::Time& time, uint8_t status);
  bool requestSettle(std_srvs::SetBool::Request& request,
                     std_srvs::SetBool::Response& response);
  bool setVelocityScale(elfin_robot_msgs::SetFloat64::Request& request,
                        elfin_robot_msgs::SetFloat64::Response& response);
  bool setDampingScales(SetDampingScales::Request& request,
                        SetDampingScales::Response& response);

  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<std::size_t> chain_to_joint_;
  std::vector<std::size_t> joint_to_chain_;

  KDL::Chain chain_;
  KDL::Vector gravity_vector_;
  boost::shared_ptr<KDL::ChainDynParam> dynamics_;
  KDL::JntArray q_chain_;
  KDL::JntArray gravity_torque_chain_;

  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> effort_limits_;
  std::vector<double> damping_;
  realtime_tools::RealtimeBuffer<std::array<double, 6> > damping_scales_;
  std::vector<double> friction_;
  std::vector<double> friction_velocity_;
  std::vector<double> torque_rate_limits_;
  std::vector<double> velocity_soft_limits_;
  std::vector<double> velocity_hard_limits_;
  std::vector<double> last_command_;
  std::vector<double> desired_command_;
  std::vector<double> initial_effort_;
  std::vector<double> gravity_joint_scales_;
  std::vector<double> effective_gravity_scales_;
  std::vector<double> gravity_bias_;
  std::vector<double> gravity_command_;
  std::vector<double> settle_damping_;

  double gravity_scale_;
  double handoff_duration_;
  double minimum_validation_effort_;
  unsigned int minimum_validation_joints_;
  double minimum_model_alignment_;
  double minimum_model_scale_;
  double maximum_model_scale_;
  double maximum_model_residual_;
  double minimum_adaptive_scale_;
  double maximum_adaptive_scale_;
  double maximum_gravity_effort_fraction_;
  double limit_margin_;
  double limit_stiffness_;
  double limit_damping_;
  double velocity_limit_damping_;
  double hard_limit_margin_;
  double hard_stop_damping_;
  std::atomic<double> velocity_limit_scale_;
  double minimum_velocity_limit_scale_;
  double maximum_velocity_limit_scale_;
  double minimum_damping_scale_;
  double maximum_damping_scale_;
  bool require_model_validation_;
  bool allow_model_validation_warning_;
  bool gravity_calibration_verified_;
  bool adaptive_entry_scale_;
  bool command_initialized_;
  bool model_validation_passed_;
  bool model_validation_warning_;
  bool gravity_capacity_valid_;
  unsigned int model_excited_joints_;
  unsigned int model_direction_mismatches_;
  double model_alignment_;
  double model_scale_estimate_;
  double model_normalized_residual_;
  double minimum_warning_alignment_;
  double handoff_progress_;
  std::atomic<bool> settle_requested_;
  std::atomic<bool> running_;
  ros::Time started_time_;
  ros::Time last_status_time_;
  ros::Time last_state_time_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::UInt8> > status_publisher_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > state_publisher_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<FreedriveTelemetry> > telemetry_publisher_;
  ros::ServiceServer settle_server_;
  ros::ServiceServer velocity_scale_server_;
  ros::ServiceServer damping_scales_server_;
};

}  // namespace elfin_freedrive_controller

#endif
