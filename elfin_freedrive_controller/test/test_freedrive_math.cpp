#include <gtest/gtest.h>

#include <elfin_freedrive_controller/freedrive_math.h>
#include <elfin_freedrive_controller/payload_model.h>

#include <kdl/chaindynparam.hpp>

#include <cmath>

namespace math = elfin_freedrive_controller::math;

TEST(ElfinFreedriveMath, TorqueClampIsSymmetric) {
  EXPECT_DOUBLE_EQ(math::clamp(4.0, -2.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(math::clamp(-4.0, -2.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(math::clamp(0.5, -2.0, 2.0), 0.5);
}

TEST(ElfinFreedriveMath, FrictionSmoothsAtZeroVelocity) {
  const double velocity = 1e-6;
  const double friction = math::smoothFriction(1.0, velocity, 0.05);
  EXPECT_LT(std::abs(friction), 1e-4);
}

TEST(ElfinFreedriveMath, RateLimitIsIndependentOfLoopFrequency) {
  EXPECT_DOUBLE_EQ(math::rateLimit(10.0, 0.0, 20.0, 0.001), 0.02);
  EXPECT_DOUBLE_EQ(math::rateLimit(-10.0, 0.0, 20.0, 0.001), -0.02);
  EXPECT_DOUBLE_EQ(math::rateLimit(0.01, 0.0, 20.0, 0.001), 0.01);
}

TEST(ElfinFreedriveMath, SmoothStepHasBoundedEndpoints) {
  EXPECT_DOUBLE_EQ(math::smoothStep(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(math::smoothStep(0.0), 0.0);
  EXPECT_DOUBLE_EQ(math::smoothStep(1.0), 1.0);
  EXPECT_DOUBLE_EQ(math::smoothStep(2.0), 1.0);
  EXPECT_DOUBLE_EQ(math::smoothStep(0.5), 0.5);
}

TEST(ElfinFreedriveMath, SettlingHandoffStartsAtPreviousCommand) {
  EXPECT_DOUBLE_EQ(
      math::settlingHandoffCommand(8.0, -2.0, 0.0, 0.10), 8.0);
  EXPECT_DOUBLE_EQ(
      math::settlingHandoffCommand(8.0, -2.0, 0.10, 0.10), -2.0);
  EXPECT_NEAR(
      math::settlingHandoffCommand(8.0, -2.0, 0.05, 0.10), 3.0, 1e-12);
}

TEST(ElfinFreedriveMath, UserScaleCannotRelaxTransitionProtection) {
  EXPECT_DOUBLE_EQ(math::transitionVelocityScale(3.0, true), 1.0);
  EXPECT_DOUBLE_EQ(math::transitionVelocityScale(0.5, true), 0.5);
  EXPECT_DOUBLE_EQ(math::transitionVelocityScale(3.0, false), 3.0);
}

TEST(ElfinFreedriveMath, VelocityScaleHasExplicitSafeBounds) {
  EXPECT_TRUE(math::velocityScaleValid(0.5, 0.5, 3.0));
  EXPECT_TRUE(math::velocityScaleValid(1.5, 0.5, 3.0));
  EXPECT_TRUE(math::velocityScaleValid(3.0, 0.5, 3.0));
  EXPECT_FALSE(math::velocityScaleValid(0.49, 0.5, 3.0));
  EXPECT_FALSE(math::velocityScaleValid(3.01, 0.5, 3.0));
  EXPECT_FALSE(math::velocityScaleValid(
      std::numeric_limits<double>::quiet_NaN(), 0.5, 3.0));
  EXPECT_FALSE(math::velocityScaleValid(1.0, 0.0, 3.0));
}

TEST(ElfinFreedriveMath, GravityEffortCapacityHasReservedHeadroom) {
  EXPECT_TRUE(math::gravityEffortHasCapacity(58.5, 65.0, 0.90));
  EXPECT_TRUE(math::gravityEffortHasCapacity(-58.5, 65.0, 0.90));
  EXPECT_FALSE(math::gravityEffortHasCapacity(58.6, 65.0, 0.90));
  EXPECT_FALSE(math::gravityEffortHasCapacity(
      std::numeric_limits<double>::quiet_NaN(), 65.0, 0.90));
  EXPECT_FALSE(math::gravityEffortHasCapacity(10.0, 0.0, 0.90));
}

TEST(ElfinFreedriveMath, HardLimitBandAllowsMotionAwayFromEntryPose) {
  EXPECT_FALSE(math::hardLimitStopRequired(
      -3.121, 0.0, -3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_FALSE(math::hardLimitStopRequired(
      -3.118, 0.02, -3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
}

TEST(ElfinFreedriveMath, HardLimitBandStopsMotionTowardOrBeyondLimit) {
  EXPECT_TRUE(math::hardLimitStopRequired(
      -3.121, -0.021, -3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_TRUE(math::hardLimitStopRequired(
      -3.125, 0.0, -3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_TRUE(math::hardLimitStopRequired(
      -3.141, 0.0, -3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_TRUE(math::hardLimitStopRequired(
      3.121, 0.021, 3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_TRUE(math::hardLimitStopRequired(
      3.125, 0.0, 3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
  EXPECT_TRUE(math::hardLimitStopRequired(
      3.141, 0.0, 3.121, -3.14, 3.14, 0.02, 0.02, 0.003));
}

TEST(ElfinFreedriveMath, E05HighLoadJ2PoseFitsTwentyPercentEnvelope) {
  // The 2026-07-25 empty-arm log required about 67.3 Nm at J2. The E05
  // 420 Nm URDF rating leaves an 84 Nm controller envelope at 20%, and the
  // 90% gravity guard still retains 8.4 Nm for damping and transients.
  EXPECT_TRUE(math::gravityEffortHasCapacity(67.3, 84.0, 0.90));
  EXPECT_FALSE(math::gravityEffortHasCapacity(75.7, 84.0, 0.90));
}

TEST(ElfinFreedriveMath, GravityValidationDetectsOppositeTorque) {
  const std::vector<double> model{0.0, 10.0, -4.0, 0.1};
  const std::vector<double> matching{0.0, 12.0, -4.8, 10.0};
  const std::vector<double> opposite{0.0, -12.0, 4.8, 10.0};

  const auto accepted =
      math::validateGravityObservation(model, matching, 0.5);
  EXPECT_TRUE(accepted.sufficient_excitation);
  EXPECT_EQ(accepted.excited_joints, 2U);
  EXPECT_EQ(accepted.direction_mismatches, 0U);
  EXPECT_NEAR(accepted.alignment, 1.0, 1e-12);
  EXPECT_NEAR(accepted.scale_estimate, 1.2, 1e-12);
  EXPECT_NEAR(accepted.normalized_residual, 0.0, 1e-12);
  EXPECT_TRUE(math::gravityValidationAccepted(
      accepted, 2, 0.9, 0.5, 2.0, 0.5));

  const auto rejected =
      math::validateGravityObservation(model, opposite, 0.5);
  EXPECT_TRUE(rejected.sufficient_excitation);
  EXPECT_EQ(rejected.direction_mismatches, 2U);
  EXPECT_NEAR(rejected.alignment, -1.0, 1e-12);
  EXPECT_NEAR(rejected.scale_estimate, -1.2, 1e-12);
  EXPECT_FALSE(math::gravityValidationAccepted(
      rejected, 2, 0.9, 0.5, 2.0, 0.5));
}

TEST(ElfinFreedriveMath, GravityValidationRequiresExcitation) {
  const auto result = math::validateGravityObservation(
      std::vector<double>{0.0, 0.1}, std::vector<double>{0.0, 2.0}, 0.5);
  EXPECT_FALSE(result.sufficient_excitation);
}

TEST(ElfinFreedriveMath, AbsoluteErrorIncludesUnexcitedPayloadAxes) {
  const std::vector<double> stale_model{0.0, -35.0, 15.6, 0.7, 0.55, 0.0};
  const std::vector<double> measured{-3.6, -62.7, 26.3, -1.7, 8.0, -1.2};
  const math::EffortModelError error =
      math::maximumAbsoluteEffortError(stale_model, measured);
  ASSERT_TRUE(error.valid);
  EXPECT_EQ(error.joint, 1U);
  EXPECT_NEAR(error.maximum_absolute_error, 27.7, 1e-12);
  EXPECT_GT(std::abs(measured[4] - stale_model[4]), 5.0);
}

TEST(ElfinFreedriveMath, GravityValidationRejectsOneReversedLoadedJoint) {
  const auto result = math::validateGravityObservation(
      std::vector<double>{-24.0, 10.0},
      std::vector<double>{-24.0, -10.0}, 1.0);
  EXPECT_EQ(result.excited_joints, 2U);
  EXPECT_EQ(result.direction_mismatches, 1U);
  EXPECT_GT(result.normalized_residual, 0.5);
  EXPECT_FALSE(math::gravityValidationAccepted(
      result, 2, 0.9, 0.5, 2.0, 0.5));
}

TEST(ElfinFreedriveMath, GravityValidationRejectsNonUniformScale) {
  const auto result = math::validateGravityObservation(
      std::vector<double>{-20.0, 10.0},
      std::vector<double>{-40.0, 5.0}, 1.0);
  EXPECT_EQ(result.direction_mismatches, 0U);
  EXPECT_GT(result.normalized_residual, 0.3);
  EXPECT_FALSE(math::gravityValidationAccepted(
      result, 2, 0.8, 0.25, 4.0, 0.3));
}

TEST(ElfinFreedriveMath, GravityValidationRejectsNonFiniteInput) {
  const auto result = math::validateGravityObservation(
      std::vector<double>{10.0, std::numeric_limits<double>::quiet_NaN()},
      std::vector<double>{10.0, 2.0}, 1.0);
  EXPECT_FALSE(result.sufficient_excitation);
}

TEST(ElfinFreedriveMath, CalibratedWarningStillRejectsDirectionAndScale) {
  const auto one_axis = math::validateGravityObservation(
      std::vector<double>{-20.0, 0.1},
      std::vector<double>{-24.0, 8.0}, 1.0);
  EXPECT_TRUE(math::gravityValidationWarningAccepted(
      one_axis, 0.5, 0.5, 2.0));

  const auto reversed = math::validateGravityObservation(
      std::vector<double>{-20.0}, std::vector<double>{24.0}, 1.0);
  EXPECT_FALSE(math::gravityValidationWarningAccepted(
      reversed, 0.5, 0.5, 2.0));

  const auto gross_scale = math::validateGravityObservation(
      std::vector<double>{-20.0}, std::vector<double>{-60.0}, 1.0);
  EXPECT_FALSE(math::gravityValidationWarningAccepted(
      gross_scale, 0.5, 0.5, 2.0));
}

TEST(ElfinFreedriveMath, AffineGravityFitRecoversScaleAndBias) {
  const std::vector<double> model{-30.0, -20.0, -10.0, 5.0, 15.0, 25.0};
  std::vector<double> measured;
  for (double value : model) {
    measured.push_back(1.8 * value - 3.5);
  }

  const auto fit = math::fitAffineGravity(model, measured, 6, 20.0);
  ASSERT_TRUE(fit.valid);
  EXPECT_EQ(fit.sample_count, 6U);
  EXPECT_NEAR(fit.scale, 1.8, 1e-12);
  EXPECT_NEAR(fit.bias, -3.5, 1e-12);
  EXPECT_NEAR(fit.rms_residual, 0.0, 1e-12);
  EXPECT_NEAR(fit.cross_validation_maximum, 0.0, 1e-12);
  EXPECT_TRUE(math::affineGravityFitAccepted(
      fit, 0.5, 3.0, 0.1, 2.0));
}

TEST(ElfinFreedriveMath, AffineGravityFitRejectsRepeatedPose) {
  const auto fit = math::fitAffineGravity(
      std::vector<double>{10.0, 10.1, 9.9, 10.0},
      std::vector<double>{20.0, 20.1, 19.9, 20.0}, 4, 2.0);
  EXPECT_FALSE(fit.valid);
  EXPECT_LT(fit.model_range, 2.0);
}

TEST(ElfinFreedriveMath, AffineGravityFitRejectsLargeOutlier) {
  const std::vector<double> model{-20.0, -10.0, 0.0, 10.0, 20.0, 30.0};
  std::vector<double> measured;
  for (double value : model) {
    measured.push_back(2.0 * value + 1.0);
  }
  measured.back() += 25.0;

  const auto fit = math::fitAffineGravity(model, measured, 6, 20.0);
  ASSERT_TRUE(fit.valid);
  EXPECT_FALSE(math::affineGravityFitAccepted(
      fit, 0.5, 3.0, 0.10, 5.0));
  EXPECT_GT(fit.maximum_residual, 5.0);
  EXPECT_GT(fit.cross_validation_maximum, fit.maximum_residual);
}

TEST(ElfinFreedriveMath, AffineGravityFitRejectsNonFiniteInput) {
  const auto fit = math::fitAffineGravity(
      std::vector<double>{-10.0, 0.0, 10.0},
      std::vector<double>{-20.0, std::numeric_limits<double>::infinity(),
                          20.0},
      3, 5.0);
  EXPECT_FALSE(fit.valid);
}

TEST(ElfinFreedriveMath, OppositeApproachesCancelDirectionalFriction) {
  std::vector<math::StaticGravityObservation> samples;
  const auto make = [](double position, double model, double measured) {
    math::StaticGravityObservation sample;
    sample.position = {0.0, position};
    sample.model = {0.0, model};
    sample.measured = {0.0, measured};
    return sample;
  };
  samples.push_back(make(0.0, 0.0, 0.0));
  samples.push_back(make(0.3, -10.0, -13.0));  // gravity -18, friction +5
  samples.push_back(make(0.6, -20.0, -31.0));  // gravity -36, friction +5
  samples.push_back(make(0.3, -10.1, -23.2));  // gravity -18.2, friction -5
  samples.push_back(make(0.0, 0.0, -5.0));

  const auto centered =
      math::centerOppositeApproaches(samples, 1, 0.05, 0.02);
  ASSERT_EQ(centered.size(), 1U);
  EXPECT_NEAR(centered[0].position[1], 0.3, 1e-12);
  EXPECT_NEAR(centered[0].model[1], -10.05, 1e-12);
  EXPECT_NEAR(centered[0].measured[1], -18.1, 1e-12);
}

TEST(ElfinFreedriveMath, OppositeApproachesIgnoreOneWayAndInvalidInput) {
  math::StaticGravityObservation start{{0.0}, {0.0}, {0.0}};
  math::StaticGravityObservation one_way{{0.5}, {1.0}, {2.0}};
  EXPECT_TRUE(math::centerOppositeApproaches(
                  {start, one_way}, 0, 0.05, 0.02)
                  .empty());

  math::StaticGravityObservation malformed{{0.5}, {1.0, 2.0}, {2.0}};
  EXPECT_TRUE(math::centerOppositeApproaches(
                  {start, malformed, start}, 0, 0.05, 0.02)
                  .empty());
}

TEST(ElfinPayloadModel, RegressorMatchesKdlRigidPayloadGravity) {
  namespace payload = elfin_freedrive_controller::payload;
  KDL::Chain chain;
  const std::array<KDL::Vector, 6> axes{{
      KDL::Vector(0.0, 1.0, 0.0), KDL::Vector(0.0, 0.0, 1.0),
      KDL::Vector(1.0, 0.0, 0.0), KDL::Vector(0.0, 1.0, 0.0),
      KDL::Vector(0.0, 0.0, 1.0), KDL::Vector(1.0, 0.0, 0.0)}};
  for (std::size_t joint = 0; joint < axes.size(); ++joint) {
    chain.addSegment(KDL::Segment(
        "link" + std::to_string(joint + 1),
        KDL::Joint("joint" + std::to_string(joint + 1), KDL::Vector::Zero(),
                   axes[joint], KDL::Joint::RotAxis),
        KDL::Frame(KDL::Vector(0.12, 0.03, 0.08))));
  }

  KDL::JntArray position(6);
  for (std::size_t joint = 0; joint < 6; ++joint) {
    position(joint) = 0.11 * static_cast<double>(joint + 1);
  }
  const KDL::Vector gravity(0.0, 0.0, -9.81);
  KDL::ChainJntToJacSolver jacobian_solver(chain);
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::Jacobian jacobian(6);
  payload::Regressor regressor{};
  ASSERT_TRUE(payload::buildRegressor(position, gravity, jacobian_solver,
                                      fk_solver, jacobian, regressor));

  payload::Model model;
  model.mass = 2.4;
  model.center_of_mass = {{0.09, -0.04, 0.16}};
  const payload::JointEffort predicted = payload::evaluate(regressor, model);

  KDL::Chain extended = chain;
  extended.addSegment(KDL::Segment(
      "payload", KDL::Joint("payload_fixed", KDL::Joint::None),
      KDL::Frame::Identity(),
      KDL::RigidBodyInertia(
          model.mass,
          KDL::Vector(model.center_of_mass[0], model.center_of_mass[1],
                      model.center_of_mass[2]),
          KDL::RotationalInertia())));
  KDL::ChainDynParam dynamics(extended, gravity);
  KDL::JntArray expected(6);
  ASSERT_EQ(dynamics.JntToGravity(position, expected), 0);
  for (std::size_t joint = 0; joint < 6; ++joint) {
    EXPECT_NEAR(predicted[joint], expected(joint), 1e-9);
  }
}

TEST(ElfinPayloadModel, RejectsImpossibleMassAndCenterOfMass) {
  namespace payload = elfin_freedrive_controller::payload;
  payload::Model model;
  model.mass = 5.0;
  model.center_of_mass = {{0.1, -0.1, 0.2}};
  EXPECT_TRUE(payload::valid(model, 5.0));
  model.mass = 5.01;
  EXPECT_FALSE(payload::valid(model, 5.0));
  model.mass = 0.466;
  model.center_of_mass = {{0.057, -0.063, 0.891}};
  EXPECT_TRUE(payload::valid(model, 5.0));
  model.mass = 2.0;
  model.center_of_mass[2] = std::numeric_limits<double>::max();
  EXPECT_FALSE(payload::valid(model, 5.0));
  model.center_of_mass[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(payload::valid(model, 5.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
