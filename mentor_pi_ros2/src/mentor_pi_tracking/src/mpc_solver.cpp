// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/mpc_solver.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/problem/discretized_model.hpp"
#include "altro/problem/problem.hpp"

namespace mentor_pi::tracking {
namespace {

class TrackingCost final : public altro::problem::CostFunction {
 public:
  TrackingCost(Eigen::Vector3d state_reference,
               Eigen::VectorXd control_reference, bool terminal)
      : state_reference_(std::move(state_reference)),
        control_reference_(std::move(control_reference)),
        terminal_(terminal) {}

  int StateDimension() const override { return 3; }
  int ControlDimension() const override { return control_reference_.size(); }
  bool HasHessian() const override { return true; }

  double Evaluate(const altro::VectorXdRef& state,
                  const altro::VectorXdRef& control) override {
    Eigen::Vector3d error = state - state_reference_;
    error(2) = WrapAngle(error(2));
    const double state_cost =
        8.0 * error.head<2>().squaredNorm() + 3.0 * error(2) * error(2);
    if (terminal_) {
      return 5.0 * state_cost;
    }
    return state_cost + 0.1 * (control - control_reference_).squaredNorm();
  }

  void Gradient(const altro::VectorXdRef& state,
                const altro::VectorXdRef& control,
                Eigen::Ref<altro::VectorXd> state_gradient,
                Eigen::Ref<altro::VectorXd> control_gradient) override {
    Eigen::Vector3d error = state - state_reference_;
    error(2) = WrapAngle(error(2));
    const double multiplier = terminal_ ? 5.0 : 1.0;
    state_gradient << multiplier * 16.0 * error(0),
        multiplier * 16.0 * error(1), multiplier * 6.0 * error(2);
    if (terminal_) {
      control_gradient.setZero();
    } else {
      control_gradient = 0.2 * (control - control_reference_);
    }
  }

  void Hessian(const altro::VectorXdRef&, const altro::VectorXdRef&,
               Eigen::Ref<altro::MatrixXd> state_hessian,
               Eigen::Ref<altro::MatrixXd> cross_hessian,
               Eigen::Ref<altro::MatrixXd> control_hessian) override {
    const double multiplier = terminal_ ? 5.0 : 1.0;
    state_hessian.setZero();
    state_hessian.diagonal() << multiplier * 16.0, multiplier * 16.0,
        multiplier * 6.0;
    cross_hessian.setZero();
    control_hessian.setZero();
    if (!terminal_) {
      control_hessian.diagonal().setConstant(0.2);
    }
  }

 private:
  Eigen::Vector3d state_reference_;
  Eigen::VectorXd control_reference_;
  bool terminal_;
};

class MecanumDynamics final : public altro::problem::ContinuousDynamics {
 public:
  int StateDimension() const override { return 3; }
  int ControlDimension() const override { return 3; }
  bool HasHessian() const override { return true; }

  void Evaluate(const altro::VectorXdRef& state,
                const altro::VectorXdRef& control, float,
                Eigen::Ref<altro::VectorXd> derivative) override {
    const double cosine = std::cos(state(2));
    const double sine = std::sin(state(2));
    derivative << cosine * control(0) - sine * control(1),
        sine * control(0) + cosine * control(1), control(2);
  }

  void Jacobian(const altro::VectorXdRef& state,
                const altro::VectorXdRef& control, float,
                Eigen::Ref<altro::MatrixXd> jacobian) override {
    const double cosine = std::cos(state(2));
    const double sine = std::sin(state(2));
    jacobian.setZero();
    jacobian(0, 2) = -sine * control(0) - cosine * control(1);
    jacobian(1, 2) = cosine * control(0) - sine * control(1);
    jacobian(0, 3) = cosine;
    jacobian(0, 4) = -sine;
    jacobian(1, 3) = sine;
    jacobian(1, 4) = cosine;
    jacobian(2, 5) = 1.0;
  }

  void Hessian(const altro::VectorXdRef&, const altro::VectorXdRef&, float,
               const altro::VectorXdRef&,
               Eigen::Ref<altro::MatrixXd> hessian) override {
    hessian.setZero();
  }
};

class AckermannDynamics final : public altro::problem::ContinuousDynamics {
 public:
  explicit AckermannDynamics(double wheelbase) : wheelbase_(wheelbase) {}
  int StateDimension() const override { return 3; }
  int ControlDimension() const override { return 2; }
  bool HasHessian() const override { return true; }

  void Evaluate(const altro::VectorXdRef& state,
                const altro::VectorXdRef& control, float,
                Eigen::Ref<altro::VectorXd> derivative) override {
    derivative << control(0) * std::cos(state(2)),
        control(0) * std::sin(state(2)),
        control(0) * std::tan(control(1)) / wheelbase_;
  }

  void Jacobian(const altro::VectorXdRef& state,
                const altro::VectorXdRef& control, float,
                Eigen::Ref<altro::MatrixXd> jacobian) override {
    const double cosine = std::cos(state(2));
    const double sine = std::sin(state(2));
    const double tangent = std::tan(control(1));
    const double secant_squared = 1.0 / std::pow(std::cos(control(1)), 2);
    jacobian.setZero();
    jacobian(0, 2) = -control(0) * sine;
    jacobian(1, 2) = control(0) * cosine;
    jacobian(0, 3) = cosine;
    jacobian(1, 3) = sine;
    jacobian(2, 3) = tangent / wheelbase_;
    jacobian(2, 4) = control(0) * secant_squared / wheelbase_;
  }

  void Hessian(const altro::VectorXdRef&, const altro::VectorXdRef&, float,
               const altro::VectorXdRef&,
               Eigen::Ref<altro::MatrixXd> hessian) override {
    hessian.setZero();
  }

 private:
  double wheelbase_;
};

class ControlBounds final
    : public altro::constraints::Constraint<altro::constraints::Inequality> {
 public:
  explicit ControlBounds(Eigen::VectorXd limits) : limits_(std::move(limits)) {}
  int StateDimension() const override { return 3; }
  int ControlDimension() const override { return limits_.size(); }
  int OutputDimension() const override { return 2 * limits_.size(); }
  std::string GetLabel() const override { return "tracking control bounds"; }

  void Evaluate(const altro::VectorXdRef&, const altro::VectorXdRef& control,
                Eigen::Ref<altro::VectorXd> output) override {
    output.head(limits_.size()) = control - limits_;
    output.tail(limits_.size()) = -control - limits_;
  }

  void Jacobian(const altro::VectorXdRef&, const altro::VectorXdRef&,
                Eigen::Ref<altro::MatrixXd> jacobian) override {
    jacobian.setZero();
    const int controls = limits_.size();
    jacobian.block(0, 3, controls, controls).setIdentity();
    jacobian.block(controls, 3, controls, controls).setIdentity();
    jacobian.block(controls, 3, controls, controls) *= -1.0;
  }

 private:
  Eigen::VectorXd limits_;
};

Eigen::VectorXd ReferenceControl(VehicleType vehicle, double wheelbase,
                                 const ReferenceState& reference) {
  const double cosine = std::cos(reference.yaw);
  const double sine = std::sin(reference.yaw);
  const double forward =
      cosine * reference.vx_world + sine * reference.vy_world;
  if (vehicle == VehicleType::kMecanum) {
    Eigen::Vector3d control;
    control << forward,
        -sine * reference.vx_world + cosine * reference.vy_world,
        reference.yaw_rate;
    return control;
  }
  Eigen::Vector2d control;
  const double steering =
      std::abs(forward) < 1.0e-6
          ? 0.0
          : std::atan(reference.yaw_rate * wheelbase / forward);
  control << forward, steering;
  return control;
}

bool IsFinite(const ReferenceState& reference) {
  return std::isfinite(reference.x) && std::isfinite(reference.y) &&
         std::isfinite(reference.yaw) && std::isfinite(reference.vx_world) &&
         std::isfinite(reference.vy_world) && std::isfinite(reference.yaw_rate);
}

bool IsFinite(const MpcConfiguration& configuration) {
  return std::isfinite(configuration.prediction_step) &&
         std::isfinite(configuration.wheelbase) &&
         std::isfinite(configuration.mecanum_radius_sum) &&
         std::isfinite(configuration.max_linear_speed) &&
         std::isfinite(configuration.max_lateral_speed) &&
         std::isfinite(configuration.max_yaw_rate) &&
         std::isfinite(configuration.max_steering_angle);
}

}  // namespace

MpcSolver::MpcSolver(MpcConfiguration configuration)
    : configuration_(std::move(configuration)) {}

MpcCommand MpcSolver::Solve(const MpcRequest& request) const {
  if (request.trajectory == nullptr || configuration_.horizon <= 0 ||
      configuration_.prediction_step <= 0.0 || !IsFinite(configuration_) ||
      !std::isfinite(request.elapsed_seconds) ||
      !std::all_of(request.state.begin(), request.state.end(),
                   [](double value) { return std::isfinite(value); })) {
    return MpcCommand{false, 0.0, 0.0, 0.0, "invalid MPC request"};
  }
  const int controls = configuration_.vehicle == VehicleType::kMecanum ? 3 : 2;
  altro::problem::Problem problem(configuration_.horizon);
  Eigen::Vector3d initial_state(request.state.data());
  problem.SetInitialState(initial_state);

  std::shared_ptr<altro::problem::DiscreteDynamics> dynamics;
  Eigen::VectorXd limits(controls);
  if (configuration_.vehicle == VehicleType::kMecanum) {
    dynamics =
        std::make_shared<altro::problem::DiscretizedModel<MecanumDynamics>>(
            MecanumDynamics{});
    limits << configuration_.max_linear_speed, configuration_.max_lateral_speed,
        configuration_.max_yaw_rate;
  } else {
    dynamics =
        std::make_shared<altro::problem::DiscretizedModel<AckermannDynamics>>(
            AckermannDynamics{configuration_.wheelbase});
    limits << configuration_.max_linear_speed,
        configuration_.max_steering_angle;
  }
  const auto bounds = std::make_shared<ControlBounds>(limits);
  auto solution =
      std::make_shared<altro::Trajectory<Eigen::Dynamic, Eigen::Dynamic>>(
          3, controls, configuration_.horizon);
  solution->SetUniformStep(configuration_.prediction_step);

  for (int step = 0; step <= configuration_.horizon; ++step) {
    const ReferenceState reference = request.trajectory->Evaluate(
        request.elapsed_seconds + configuration_.prediction_step * step);
    if (!IsFinite(reference)) {
      return MpcCommand{false, 0.0, 0.0, 0.0,
                        "trajectory evaluation is non-finite"};
    }
    Eigen::Vector3d reference_state(reference.x, reference.y, reference.yaw);
    Eigen::VectorXd reference_control = ReferenceControl(
        configuration_.vehicle, configuration_.wheelbase, reference);
    problem.SetCostFunction(
        std::make_shared<TrackingCost>(reference_state, reference_control,
                                       step == configuration_.horizon),
        step);
    solution->State(step) = reference_state;
    solution->Control(step) =
        reference_control.cwiseMax(-limits).cwiseMin(limits);
    if (step < configuration_.horizon) {
      problem.SetDynamics(dynamics, step);
      problem.SetConstraint(bounds, step);
    }
  }
  solution->State(0) = initial_state;

  using Solver =
      altro::augmented_lagrangian::AugmentedLagrangianiLQR<Eigen::Dynamic,
                                                           Eigen::Dynamic>;
  Solver solver(problem);
  solver.SetTrajectory(solution);
  auto& options = solver.GetOptions();
  options.max_iterations_total = 40;
  options.max_iterations_outer = 4;
  options.max_iterations_inner = 10;
  options.constraint_tolerance = 1.0e-2;
  options.cost_tolerance = 1.0e-3;
  options.gradient_tolerance = 1.0e-2;
  options.nthreads = 1;
  solver.Solve();
  if (solver.GetStatus() != altro::SolverStatus::kSolved) {
    return MpcCommand{false, 0.0, 0.0, 0.0, "ALTO did not converge"};
  }

  const Eigen::VectorXd control = solution->Control(0);
  if (!control.allFinite()) {
    return MpcCommand{false, 0.0, 0.0, 0.0, "ALTO returned non-finite control"};
  }
  if (configuration_.vehicle == VehicleType::kMecanum) {
    return EnforceCommandBounds(
        configuration_,
        MpcCommand{true, control(0), control(1), control(2), "ALTO solved"});
  }
  return EnforceCommandBounds(
      configuration_,
      MpcCommand{true, control(0), 0.0,
                 control(0) * std::tan(control(1)) / configuration_.wheelbase,
                 "ALTO solved"});
}

MpcCommand FeedbackCommand(const MpcConfiguration& configuration,
                           const MpcRequest& request) {
  if (request.trajectory == nullptr || !IsFinite(configuration) ||
      !std::isfinite(request.elapsed_seconds) ||
      !std::all_of(request.state.begin(), request.state.end(),
                   [](double value) { return std::isfinite(value); })) {
    return {};
  }
  const ReferenceState reference =
      request.trajectory->Evaluate(request.elapsed_seconds);
  if (!IsFinite(reference)) {
    return {};
  }
  const double dx = reference.x - request.state[0];
  const double dy = reference.y - request.state[1];
  const double cosine = std::cos(request.state[2]);
  const double sine = std::sin(request.state[2]);
  const double forward_error = cosine * dx + sine * dy;
  const double lateral_error = -sine * dx + cosine * dy;
  Eigen::VectorXd feedforward = ReferenceControl(
      configuration.vehicle, configuration.wheelbase, reference);
  if (configuration.vehicle == VehicleType::kMecanum) {
    return EnforceCommandBounds(
        configuration,
        MpcCommand{
            true,
            std::clamp(feedforward(0) + forward_error,
                       -configuration.max_linear_speed,
                       configuration.max_linear_speed),
            std::clamp(feedforward(1) + lateral_error,
                       -configuration.max_lateral_speed,
                       configuration.max_lateral_speed),
            std::clamp(
                feedforward(2) + WrapAngle(reference.yaw - request.state[2]),
                -configuration.max_yaw_rate, configuration.max_yaw_rate),
            "bounded feedback fallback"});
  }
  const double speed = std::clamp(feedforward(0) + forward_error,
                                  -configuration.max_linear_speed,
                                  configuration.max_linear_speed);
  const double steering = std::clamp(
      feedforward(1) + 0.8 * lateral_error +
          WrapAngle(reference.yaw - request.state[2]),
      -configuration.max_steering_angle, configuration.max_steering_angle);
  return EnforceCommandBounds(
      configuration,
      MpcCommand{true, speed, 0.0,
                 speed * std::tan(steering) / configuration.wheelbase,
                 "bounded feedback fallback"});
}

MpcCommand EnforceCommandBounds(const MpcConfiguration& configuration,
                                MpcCommand command) {
  if (!command.solved || !std::isfinite(command.linear_x) ||
      !std::isfinite(command.linear_y) || !std::isfinite(command.angular_z) ||
      configuration.max_linear_speed <= 0.0 || configuration.wheelbase <= 0.0) {
    return {};
  }
  if (configuration.vehicle == VehicleType::kMecanum) {
    if (configuration.mecanum_radius_sum <= 0.0) {
      return {};
    }
    const double yaw_component =
        configuration.mecanum_radius_sum * command.angular_z;
    const double largest_wheel_speed = std::max(
        {std::abs(command.linear_x - command.linear_y - yaw_component),
         std::abs(command.linear_x + command.linear_y + yaw_component),
         std::abs(command.linear_x + command.linear_y - yaw_component),
         std::abs(command.linear_x - command.linear_y + yaw_component)});
    if (largest_wheel_speed > configuration.max_linear_speed) {
      const double scale = configuration.max_linear_speed / largest_wheel_speed;
      command.linear_x *= scale;
      command.linear_y *= scale;
      command.angular_z *= scale;
    }
    return command;
  }
  command.linear_x =
      std::clamp(command.linear_x, -configuration.max_linear_speed,
                 configuration.max_linear_speed);
  command.linear_y = 0.0;
  const double maximum_yaw_rate = std::abs(command.linear_x) *
                                  std::tan(configuration.max_steering_angle) /
                                  configuration.wheelbase;
  command.angular_z =
      std::clamp(command.angular_z, -maximum_yaw_rate, maximum_yaw_rate);
  return command;
}

}  // namespace mentor_pi::tracking
