#pragma once

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace elfin_freedrive_controller {
namespace payload {

constexpr std::size_t kJointCount = 6;
constexpr std::size_t kParameterCount = 4;

struct Model {
  double mass{0.0};
  std::array<double, 3> center_of_mass{{0.0, 0.0, 0.0}};
};

using JointEffort = std::array<double, kJointCount>;
using RegressorRow = std::array<double, kParameterCount>;
using Regressor = std::array<RegressorRow, kJointCount>;

inline double centerOfMassRadius(const Model& model) {
  return std::sqrt(model.center_of_mass[0] * model.center_of_mass[0] +
                   model.center_of_mass[1] * model.center_of_mass[1] +
                   model.center_of_mass[2] * model.center_of_mass[2]);
}

inline bool valid(const Model& model, double maximum_mass) {
  if (!std::isfinite(model.mass) || model.mass < 0.0 ||
      model.mass > maximum_mass || !std::isfinite(maximum_mass) ||
      maximum_mass <= 0.0) {
    return false;
  }
  for (double value : model.center_of_mass) {
    // Gravity compensation depends on the first moment m*c. A radius-only
    // limit rejects light, long tools even when their required effort is well
    // inside the controller's independently enforced capacity envelope.
    if (!std::isfinite(value) || !std::isfinite(model.mass * value)) {
      return false;
    }
  }
  return true;
}

inline bool buildRegressor(const KDL::JntArray& position,
                           const KDL::Vector& gravity,
                           KDL::ChainJntToJacSolver& jacobian_solver,
                           KDL::ChainFkSolverPos_recursive& fk_solver,
                           KDL::Jacobian& jacobian,
                           Regressor& regressor) {
  if (position.rows() != kJointCount || jacobian.columns() != kJointCount) {
    return false;
  }

  KDL::Frame tip_frame;
  if (jacobian_solver.JntToJac(position, jacobian) < 0 ||
      fk_solver.JntToCart(position, tip_frame) < 0) {
    return false;
  }

  const std::array<KDL::Vector, 3> tip_axes{{
      KDL::Vector(1.0, 0.0, 0.0),
      KDL::Vector(0.0, 1.0, 0.0),
      KDL::Vector(0.0, 0.0, 1.0)}};
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    regressor[joint][0] =
        -(jacobian(0, joint) * gravity.x() +
          jacobian(1, joint) * gravity.y() +
          jacobian(2, joint) * gravity.z());
    for (std::size_t axis = 0; axis < tip_axes.size(); ++axis) {
      // h = mass * CoM is expressed in the flange frame. R*h cross g is the
      // gravity moment at the flange origin, keeping the model linear in
      // [mass, h_x, h_y, h_z].
      const KDL::Vector moment = (tip_frame.M * tip_axes[axis]) * gravity;
      regressor[joint][axis + 1] =
          -(jacobian(3, joint) * moment.x() +
            jacobian(4, joint) * moment.y() +
            jacobian(5, joint) * moment.z());
    }
  }
  return true;
}

inline JointEffort evaluate(const Regressor& regressor, const Model& model) {
  const std::array<double, kParameterCount> parameters{{
      model.mass,
      model.mass * model.center_of_mass[0],
      model.mass * model.center_of_mass[1],
      model.mass * model.center_of_mass[2]}};
  JointEffort effort{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    for (std::size_t parameter = 0; parameter < kParameterCount; ++parameter) {
      effort[joint] += regressor[joint][parameter] * parameters[parameter];
    }
  }
  return effort;
}

}  // namespace payload
}  // namespace elfin_freedrive_controller
