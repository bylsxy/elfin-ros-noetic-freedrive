#ifndef ELFIN_FREEDRIVE_CONTROLLER_FREEDRIVE_MATH_H
#define ELFIN_FREEDRIVE_CONTROLLER_FREEDRIVE_MATH_H

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>

namespace elfin_freedrive_controller {
namespace math {

inline double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

inline double sign(double value) {
  return value < 0.0 ? -1.0 : 1.0;
}

inline double smoothFriction(double coefficient, double velocity,
                             double transition_velocity) {
  return coefficient * std::tanh(velocity / transition_velocity);
}

inline double rateLimit(double value, double previous, double rate,
                        double period) {
  const double delta = std::max(0.0, rate) * std::max(0.0, period);
  return clamp(value, previous - delta, previous + delta);
}

inline double smoothStep(double progress) {
  const double bounded = clamp(progress, 0.0, 1.0);
  return bounded * bounded * (3.0 - 2.0 * bounded);
}

inline double interpolate(double start, double end, double progress) {
  return start + (end - start) * clamp(progress, 0.0, 1.0);
}

inline double settlingHandoffCommand(double entry_command,
                                     double settling_target,
                                     double elapsed,
                                     double duration) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    return settling_target;
  }
  return interpolate(entry_command, settling_target,
                     smoothStep(elapsed / duration));
}

inline double transitionVelocityScale(double requested_scale,
                                      bool transition_active) {
  return transition_active ? std::min(requested_scale, 1.0)
                           : requested_scale;
}

inline bool velocityScaleValid(double scale, double minimum,
                               double maximum) {
  return std::isfinite(scale) && std::isfinite(minimum) &&
         std::isfinite(maximum) && minimum > 0.0 && maximum >= minimum &&
         scale >= minimum && scale <= maximum;
}

inline bool gravityEffortHasCapacity(double requested_effort,
                                     double effort_limit,
                                     double maximum_fraction) {
  return std::isfinite(requested_effort) && std::isfinite(effort_limit) &&
         effort_limit > 0.0 && std::isfinite(maximum_fraction) &&
         maximum_fraction > 0.0 && maximum_fraction < 1.0 &&
         std::abs(requested_effort) < maximum_fraction * effort_limit;
}

inline double gravityCapacityFraction(bool payload_hold_verified,
                                      double standard_fraction,
                                      double verified_payload_fraction) {
  return payload_hold_verified ? verified_payload_fraction
                               : standard_fraction;
}

inline bool hardLimitStopRequired(double position, double velocity,
                                  double entry_position, double lower,
                                  double upper, double hard_margin,
                                  double toward_limit_velocity,
                                  double inward_travel_tolerance) {
  if (!std::isfinite(position) || !std::isfinite(velocity) ||
      !std::isfinite(entry_position) || !std::isfinite(hard_margin) ||
      !std::isfinite(toward_limit_velocity) ||
      !std::isfinite(inward_travel_tolerance) || hard_margin <= 0.0 ||
      toward_limit_velocity <= 0.0 || inward_travel_tolerance <= 0.0) {
    return true;
  }
  if (std::isfinite(lower) && position <= lower + hard_margin) {
    return position <= lower || velocity < -toward_limit_velocity ||
           position < entry_position - inward_travel_tolerance;
  }
  if (std::isfinite(upper) && position >= upper - hard_margin) {
    return position >= upper || velocity > toward_limit_velocity ||
           position > entry_position + inward_travel_tolerance;
  }
  return false;
}

struct GravityValidation {
  bool sufficient_excitation = false;
  std::size_t excited_joints = 0;
  std::size_t direction_mismatches = 0;
  double alignment = 0.0;
  double scale_estimate = 0.0;
  double normalized_residual = std::numeric_limits<double>::infinity();
};

struct EffortModelError {
  bool valid = false;
  std::size_t joint = 0;
  double maximum_absolute_error = std::numeric_limits<double>::infinity();
};

inline EffortModelError maximumAbsoluteEffortError(
    const std::vector<double>& model,
    const std::vector<double>& measured) {
  EffortModelError result;
  if (model.size() != measured.size() || model.empty()) {
    return result;
  }
  result.maximum_absolute_error = 0.0;
  for (std::size_t joint = 0; joint < model.size(); ++joint) {
    if (!std::isfinite(model[joint]) || !std::isfinite(measured[joint])) {
      return EffortModelError();
    }
    const double error = std::abs(measured[joint] - model[joint]);
    if (error > result.maximum_absolute_error) {
      result.maximum_absolute_error = error;
      result.joint = joint;
    }
  }
  result.valid = true;
  return result;
}

struct AffineGravityFit {
  bool valid = false;
  std::size_t sample_count = 0;
  double scale = 0.0;
  double bias = 0.0;
  double model_range = 0.0;
  double rms_residual = std::numeric_limits<double>::infinity();
  double maximum_residual = std::numeric_limits<double>::infinity();
  double normalized_residual = std::numeric_limits<double>::infinity();
  double cross_validation_rms = std::numeric_limits<double>::infinity();
  double cross_validation_maximum = std::numeric_limits<double>::infinity();
};

struct StaticGravityObservation {
  std::vector<double> position;
  std::vector<double> model;
  std::vector<double> measured;
};

// Groups repeated poses and gives equal weight to samples approached from the
// positive and negative directions of a reference joint. Averaging those two
// directional means cancels the first-order Coulomb/static-friction term.
inline std::vector<StaticGravityObservation> centerOppositeApproaches(
    const std::vector<StaticGravityObservation>& samples,
    std::size_t approach_joint,
    double minimum_approach_delta,
    double pose_tolerance) {
  std::vector<StaticGravityObservation> centered;
  if (samples.size() < 3 || !std::isfinite(minimum_approach_delta) ||
      minimum_approach_delta <= 0.0 || !std::isfinite(pose_tolerance) ||
      pose_tolerance <= 0.0) {
    return centered;
  }
  const std::size_t width = samples.front().position.size();
  if (width == 0 || approach_joint >= width) {
    return centered;
  }
  for (const auto& sample : samples) {
    if (sample.position.size() != width || sample.model.size() != width ||
        sample.measured.size() != width) {
      return std::vector<StaticGravityObservation>();
    }
    for (std::size_t joint = 0; joint < width; ++joint) {
      if (!std::isfinite(sample.position[joint]) ||
          !std::isfinite(sample.model[joint]) ||
          !std::isfinite(sample.measured[joint])) {
        return std::vector<StaticGravityObservation>();
      }
    }
  }

  struct DirectionGroup {
    std::vector<double> representative_position;
    std::vector<double> positive_model;
    std::vector<double> positive_measured;
    std::vector<double> negative_model;
    std::vector<double> negative_measured;
    std::size_t positive_count = 0;
    std::size_t negative_count = 0;
  };
  std::vector<DirectionGroup> groups;
  for (std::size_t sample_index = 1; sample_index < samples.size();
       ++sample_index) {
    const double approach_delta =
        samples[sample_index].position[approach_joint] -
        samples[sample_index - 1].position[approach_joint];
    if (std::abs(approach_delta) < minimum_approach_delta) {
      continue;
    }
    DirectionGroup* matched = nullptr;
    for (DirectionGroup& group : groups) {
      bool same_pose = true;
      for (std::size_t joint = 0; joint < width; ++joint) {
        if (std::abs(samples[sample_index].position[joint] -
                     group.representative_position[joint]) >
            pose_tolerance) {
          same_pose = false;
          break;
        }
      }
      if (same_pose) {
        matched = &group;
        break;
      }
    }
    if (matched == nullptr) {
      DirectionGroup group;
      group.representative_position = samples[sample_index].position;
      group.positive_model.assign(width, 0.0);
      group.positive_measured.assign(width, 0.0);
      group.negative_model.assign(width, 0.0);
      group.negative_measured.assign(width, 0.0);
      groups.push_back(group);
      matched = &groups.back();
    }
    const bool positive = approach_delta > 0.0;
    std::vector<double>& model_sum =
        positive ? matched->positive_model : matched->negative_model;
    std::vector<double>& measured_sum =
        positive ? matched->positive_measured : matched->negative_measured;
    std::size_t& count =
        positive ? matched->positive_count : matched->negative_count;
    for (std::size_t joint = 0; joint < width; ++joint) {
      model_sum[joint] += samples[sample_index].model[joint];
      measured_sum[joint] += samples[sample_index].measured[joint];
    }
    ++count;
  }

  for (const DirectionGroup& group : groups) {
    if (group.positive_count == 0 || group.negative_count == 0) {
      continue;
    }
    StaticGravityObservation observation;
    observation.position = group.representative_position;
    observation.model.resize(width, 0.0);
    observation.measured.resize(width, 0.0);
    for (std::size_t joint = 0; joint < width; ++joint) {
      observation.model[joint] = 0.5 *
          (group.positive_model[joint] /
               static_cast<double>(group.positive_count) +
           group.negative_model[joint] /
               static_cast<double>(group.negative_count));
      observation.measured[joint] = 0.5 *
          (group.positive_measured[joint] /
               static_cast<double>(group.positive_count) +
           group.negative_measured[joint] /
               static_cast<double>(group.negative_count));
    }
    centered.push_back(observation);
  }
  return centered;
}

// Fits measured = scale * model + bias. Static friction cannot be separated
// from gravity at one pose, so this deliberately requires model excitation
// across several position-controlled poses.
inline AffineGravityFit fitAffineGravity(
    const std::vector<double>& model,
    const std::vector<double>& measured,
    std::size_t minimum_samples,
    double minimum_model_range) {
  AffineGravityFit result;
  if (model.size() != measured.size() || model.size() < minimum_samples ||
      minimum_samples < 2 || !std::isfinite(minimum_model_range) ||
      minimum_model_range <= 0.0) {
    return result;
  }

  double model_sum = 0.0;
  double measured_sum = 0.0;
  double model_minimum = std::numeric_limits<double>::infinity();
  double model_maximum = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < model.size(); ++i) {
    if (!std::isfinite(model[i]) || !std::isfinite(measured[i])) {
      return AffineGravityFit();
    }
    model_sum += model[i];
    measured_sum += measured[i];
    model_minimum = std::min(model_minimum, model[i]);
    model_maximum = std::max(model_maximum, model[i]);
  }
  result.sample_count = model.size();
  result.model_range = model_maximum - model_minimum;
  if (result.model_range < minimum_model_range) {
    return result;
  }

  const double count = static_cast<double>(model.size());
  const double model_mean = model_sum / count;
  const double measured_mean = measured_sum / count;
  double model_variance_sum = 0.0;
  double covariance_sum = 0.0;
  for (std::size_t i = 0; i < model.size(); ++i) {
    const double centered_model = model[i] - model_mean;
    model_variance_sum += centered_model * centered_model;
    covariance_sum += centered_model * (measured[i] - measured_mean);
  }
  if (model_variance_sum <= 1e-12) {
    return result;
  }

  result.scale = covariance_sum / model_variance_sum;
  result.bias = measured_mean - result.scale * model_mean;
  if (!std::isfinite(result.scale) || !std::isfinite(result.bias)) {
    return AffineGravityFit();
  }

  double residual_squared_sum = 0.0;
  double measured_squared_sum = 0.0;
  double maximum_residual = 0.0;
  for (std::size_t i = 0; i < model.size(); ++i) {
    const double predicted = result.scale * model[i] + result.bias;
    const double residual = measured[i] - predicted;
    residual_squared_sum += residual * residual;
    measured_squared_sum += measured[i] * measured[i];
    maximum_residual = std::max(maximum_residual, std::abs(residual));
  }
  result.rms_residual = std::sqrt(residual_squared_sum / count);
  result.maximum_residual = maximum_residual;
  result.normalized_residual =
      measured_squared_sum > 1e-12
          ? std::sqrt(residual_squared_sum / measured_squared_sum)
          : std::numeric_limits<double>::infinity();

  double cross_validation_squared_sum = 0.0;
  double cross_validation_maximum = 0.0;
  for (std::size_t omitted = 0; omitted < model.size(); ++omitted) {
    const double reduced_count = count - 1.0;
    const double reduced_model_mean =
        (model_sum - model[omitted]) / reduced_count;
    const double reduced_measured_mean =
        (measured_sum - measured[omitted]) / reduced_count;
    double reduced_variance_sum = 0.0;
    double reduced_covariance_sum = 0.0;
    for (std::size_t i = 0; i < model.size(); ++i) {
      if (i == omitted) {
        continue;
      }
      const double centered_model = model[i] - reduced_model_mean;
      reduced_variance_sum += centered_model * centered_model;
      reduced_covariance_sum +=
          centered_model * (measured[i] - reduced_measured_mean);
    }
    if (reduced_variance_sum <= 1e-12) {
      return result;
    }
    const double reduced_scale =
        reduced_covariance_sum / reduced_variance_sum;
    const double reduced_bias =
        reduced_measured_mean - reduced_scale * reduced_model_mean;
    const double predicted =
        reduced_scale * model[omitted] + reduced_bias;
    const double residual = measured[omitted] - predicted;
    if (!std::isfinite(residual)) {
      return result;
    }
    cross_validation_squared_sum += residual * residual;
    cross_validation_maximum =
        std::max(cross_validation_maximum, std::abs(residual));
  }
  result.cross_validation_rms =
      std::sqrt(cross_validation_squared_sum / count);
  result.cross_validation_maximum = cross_validation_maximum;
  result.valid = std::isfinite(result.rms_residual) &&
                 std::isfinite(result.maximum_residual) &&
                 std::isfinite(result.normalized_residual) &&
                 std::isfinite(result.cross_validation_rms) &&
                 std::isfinite(result.cross_validation_maximum);
  return result;
}

inline bool affineGravityFitAccepted(
    const AffineGravityFit& fit,
    double minimum_scale,
    double maximum_scale,
    double maximum_normalized_residual,
    double maximum_absolute_residual) {
  return fit.valid && std::isfinite(minimum_scale) &&
         std::isfinite(maximum_scale) && minimum_scale > 0.0 &&
         maximum_scale > minimum_scale &&
         std::isfinite(maximum_normalized_residual) &&
         maximum_normalized_residual >= 0.0 &&
         std::isfinite(maximum_absolute_residual) &&
         maximum_absolute_residual >= 0.0 && fit.scale >= minimum_scale &&
         fit.scale <= maximum_scale &&
         fit.normalized_residual <= maximum_normalized_residual &&
         fit.maximum_residual <= maximum_absolute_residual &&
         fit.cross_validation_maximum <= maximum_absolute_residual;
}

inline GravityValidation validateGravityObservation(
    const std::vector<double>& model,
    const std::vector<double>& measured,
    double minimum_model_effort) {
  GravityValidation result;
  if (model.size() != measured.size() || model.empty() ||
      !std::isfinite(minimum_model_effort) || minimum_model_effort < 0.0) {
    return result;
  }

  double dot = 0.0;
  double model_norm_squared = 0.0;
  double measured_norm_squared = 0.0;
  for (std::size_t i = 0; i < model.size(); ++i) {
    if (!std::isfinite(model[i]) || !std::isfinite(measured[i])) {
      return GravityValidation();
    }
    if (std::abs(model[i]) < minimum_model_effort) {
      continue;
    }
    dot += model[i] * measured[i];
    model_norm_squared += model[i] * model[i];
    measured_norm_squared += measured[i] * measured[i];
    ++result.excited_joints;
    // Zero feedback is not evidence of opposite torque. Some drives expose a
    // zero sample at the controller handoff; alignment/residual checks still
    // account for the missing support, while only a strict sign reversal is
    // classified as a direction mismatch.
    if (model[i] * measured[i] < 0.0) {
      ++result.direction_mismatches;
    }
  }

  if (result.excited_joints == 0 || model_norm_squared <= 1e-12 ||
      measured_norm_squared <= 1e-12) {
    return result;
  }
  result.sufficient_excitation = true;
  result.alignment =
      dot / std::sqrt(model_norm_squared * measured_norm_squared);
  result.scale_estimate = dot / model_norm_squared;
  double residual_squared = 0.0;
  for (std::size_t i = 0; i < model.size(); ++i) {
    if (std::abs(model[i]) < minimum_model_effort) {
      continue;
    }
    const double residual =
        measured[i] - result.scale_estimate * model[i];
    residual_squared += residual * residual;
  }
  result.normalized_residual =
      std::sqrt(residual_squared / measured_norm_squared);
  return result;
}

inline bool gravityValidationAccepted(
    const GravityValidation& validation,
    std::size_t minimum_excited_joints,
    double minimum_alignment,
    double minimum_scale,
    double maximum_scale,
    double maximum_normalized_residual) {
  return validation.sufficient_excitation &&
         validation.excited_joints >= minimum_excited_joints &&
         validation.direction_mismatches == 0 &&
         std::isfinite(validation.alignment) &&
         validation.alignment >= minimum_alignment &&
         std::isfinite(validation.scale_estimate) &&
         validation.scale_estimate >= minimum_scale &&
         validation.scale_estimate <= maximum_scale &&
         std::isfinite(validation.normalized_residual) &&
         validation.normalized_residual <= maximum_normalized_residual;
}

// A calibrated arm can have only one gravity-excited axis at some poses, and
// gearbox stiction can make a single-pose residual look poor. This relaxed
// classification never accepts reversed torque or a gross scale mismatch.
inline bool gravityValidationWarningAccepted(
    const GravityValidation& validation,
    double minimum_alignment,
    double minimum_scale,
    double maximum_scale) {
  if (!std::isfinite(minimum_alignment) || minimum_alignment < 0.0 ||
      minimum_alignment > 1.0 || !std::isfinite(minimum_scale) ||
      minimum_scale <= 0.0 || !std::isfinite(maximum_scale) ||
      maximum_scale <= minimum_scale || validation.direction_mismatches != 0) {
    return false;
  }
  if (validation.excited_joints == 0) {
    return true;
  }
  return validation.sufficient_excitation &&
         std::isfinite(validation.alignment) &&
         validation.alignment >= minimum_alignment &&
         std::isfinite(validation.scale_estimate) &&
         validation.scale_estimate >= minimum_scale &&
         validation.scale_estimate <= maximum_scale;
}

}  // namespace math
}  // namespace elfin_freedrive_controller

#endif
