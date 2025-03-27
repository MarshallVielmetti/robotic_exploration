#include "exploration_sim_planner/util/MpcController.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/Constants.hpp>
#include <cmath>
#include <iostream>

MpcController::MpcController(/*double wheelbase,*/ double dt, double v_max, double omega_max,
                             const MPCParameters& params)
    : /*wheelbase_(wheelbase),*/
      dt_(dt),
      v_max_(v_max),
      omega_max_(omega_max),
      mpc_params_(params) {
  // The solver will be initialized in the first call to compute_control
}

Eigen::Vector2d MpcController::compute_control(const Eigen::Vector2d& state, double theta, double v_ref,
                                               const std::vector<Eigen::Vector2d>& path) {
  if (path.empty()) {
    return Eigen::Vector2d::Zero();
  }

  v_ref = std::clamp(v_ref, 0.01, v_max_);

  // Initialize the solver if not done yet
  if (!is_initialized_) {
    std::cout << "Initializing Solver" << std::endl;
    initialize_solver();
  }

  // Create current state vector [x, y, theta]
  Eigen::VectorXd current_state(3);
  current_state << state[0], state[1], theta;

  // Convert path to required format for MPC
  std::vector<Eigen::VectorXd> reference_trajectory;
  for (size_t i = 0; i < static_cast<size_t>(mpc_params_.horizon); i++) {
    // Use the last path point if we've reached the end
    size_t idx = i < path.size() ? i : path.size() - 1;

    Eigen::VectorXd ref_state(state_dim_);
    ref_state << path[idx][0], path[idx][1],
        0.0;  // theta reference set to 0 for now

    // Calculate desired orientation if we have at least two points
    if (idx < path.size() - 1) {
      double dx = path[idx + 1][0] - path[idx][0];
      double dy = path[idx + 1][1] - path[idx][1];
      ref_state[2] = std::atan2(dy, dx);  // Set reference theta
    }

    reference_trajectory.push_back(ref_state);
  }

  // Update problem matrices and solve
  update_problem_matrices(current_state, v_ref, reference_trajectory);

  // Get the control solution (first set of controls)
  static Eigen::VectorXd prev_solution;
  if (prev_solution.size() == n_variables_) {
    solver_.setPrimalVariable(prev_solution);
  }

  // Solve the QP problem
  if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cerr << "Error solving MPC problem." << std::endl;
    return Eigen::Vector2d::Zero();
  }

  Eigen::VectorXd QPSolution = solver_.getSolution();
  prev_solution = QPSolution;

  // Extract first control actions
  double v = QPSolution(state_dim_ * mpc_params_.horizon);
  double omega = QPSolution(state_dim_ * mpc_params_.horizon + 1);

  // Clamp to limits
  v = std::clamp(v, 0.0, v_max_);
  omega = std::clamp(omega, -omega_max_, omega_max_);

  return Eigen::Vector2d(v, omega);
}

void MpcController::set_parameters(const MPCParameters& params) {
  mpc_params_ = params;

  // If already initialized, we need to update hessian weights for new
  // parameters
  if (is_initialized_) {
    // Update the weights in the cost function (hessian matrix)
    // Size depends on the state_dim, control_dim, and horizon from previous
    // initialization
    int variables_per_horizon = hessian_.rows() / mpc_params_.horizon;
    int state_dim = variables_per_horizon - 2;  // Assuming 2 control variables

    // Update the diagonal elements of the hessian for state costs
    for (int i = 0; i < mpc_params_.horizon; i++) {
      hessian_.coeffRef(i * variables_per_horizon, i * variables_per_horizon) = mpc_params_.Q_x;
      hessian_.coeffRef(i * variables_per_horizon + 1, i * variables_per_horizon + 1) = mpc_params_.Q_y;
      hessian_.coeffRef(i * variables_per_horizon + 2, i * variables_per_horizon + 2) = mpc_params_.Q_theta;
    }

    // Update the diagonal elements for control costs
    for (int i = 0; i < mpc_params_.horizon; i++) {
      hessian_.coeffRef(state_dim * mpc_params_.horizon + 2 * i, state_dim * mpc_params_.horizon + 2 * i) =
          mpc_params_.R_v;
      hessian_.coeffRef(state_dim * mpc_params_.horizon + 2 * i + 1, state_dim * mpc_params_.horizon + 2 * i + 1) =
          mpc_params_.R_omega;
    }

    solver_.updateHessianMatrix(hessian_);
  }
}

const MpcController::MPCParameters& MpcController::get_parameters() const { return mpc_params_; }

void MpcController::initialize_solver() {
  NUM_STATES_ = mpc_params_.horizon + 1;  // +1 for the initial state
  NUM_CONTROLS_ = mpc_params_.horizon;

  n_variables_ = NUM_STATES_ * state_dim_ + NUM_CONTROLS_ * control_dim_;
  n_constraints_ = 2 * NUM_STATES_ * state_dim_ + NUM_CONTROLS_ * control_dim_;

  // Initialize matrices
  init_hessian();

  // The gradient will be -Q*x_ref for the state part and 0 for the control part
  gradient_.resize(state_dim_ * NUM_STATES_ + control_dim_ * NUM_CONTROLS_);
  gradient_.setZero();

  init_linear_constraints_matrix();

  // Init A_ and B_ matrices
  // A_matrix_.resize(state_dim_, state_dim_);
  // A_matrix_.setIdentity();
  B_matrix_.resize(state_dim_, control_dim_);
  B_matrix_.setZero();

  A_matrix_.resize(state_dim_, state_dim_);
  A_matrix_.setIdentity();

  // Initialize OsqpEigen solver
  solver_.data()->setNumberOfVariables(n_variables_);
  solver_.data()->setNumberOfConstraints(n_constraints_);

  // Set the hessian and gradient
  solver_.data()->setHessianMatrix(hessian_);
  solver_.data()->setGradient(gradient_);

  // Set the constraints
  solver_.data()->setLinearConstraintsMatrix(linearConstraintsMatrix_);
  solver_.data()->setLowerBound(lowerBound_);
  solver_.data()->setUpperBound(upperBound_);

  // Set OSQP solver settings
  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);

  // Initialize the solver
  solver_.initSolver();

  is_initialized_ = true;
}

void MpcController::update_problem_matrices(const Eigen::VectorXd& state, double v_ref,
                                            const std::vector<Eigen::VectorXd>& reference) {
  if (!is_initialized_) {
    return;
  }

  linearize_about_state(state, v_ref);

  // Update linear constraint matrix with new A matrix
  for (int i = 0; i < NUM_STATES_ - 1; i++) {
    linearConstraintsMatrix_.coeffRef((i + 1) * state_dim_, i * state_dim_ + 2) = A_matrix_(0, 2);
    linearConstraintsMatrix_.coeffRef((i + 1) * state_dim_ + 1, i * state_dim_ + 2) = A_matrix_(1, 2);
  }

  // Update the linear constraint matrix with the new B matrix
  int control_col_offset = state_dim_ * NUM_STATES_;
  for (int i = 0; i < NUM_CONTROLS_; i++) {
    linearConstraintsMatrix_.coeffRef((i + 1) * state_dim_, control_col_offset + i * control_dim_) = B_matrix_(0, 0);
    linearConstraintsMatrix_.coeffRef((i + 1) * state_dim_ + 1, control_col_offset + i * control_dim_) =
        B_matrix_(1, 0);
    linearConstraintsMatrix_.coeffRef((i + 1) * state_dim_ + 2, control_col_offset + i * control_dim_ + 1) =
        B_matrix_(2, 1);
  }

  // Just need to update the first state constraint
  for (int i = 0; i < state_dim_; i++) {
    lowerBound_[i] = -state[i];
    upperBound_[i] = -state[i];
  }

  gradient_.segment(0, state_dim_) =
      Eigen::Vector3d(-mpc_params_.Q_x * state[0], -mpc_params_.Q_y * state[1], -mpc_params_.Q_theta * state[2]);

  // now need to update the gradient to be -Q * x_ref
  for (int i = 0; i < NUM_STATES_ - 1; i++) {
    gradient_.segment((i + 1) * state_dim_, state_dim_) = Eigen::Vector3d(
        -mpc_params_.Q_x * reference[i][0], -mpc_params_.Q_y * reference[i][1], -mpc_params_.Q_theta * reference[i][2]);
  }

  // Update the QP problem
  solver_.updateGradient(gradient_);
  solver_.updateBounds(lowerBound_, upperBound_);
  solver_.updateLinearConstraintsMatrix(linearConstraintsMatrix_);
}

void MpcController::linearize_about_state(const Eigen::VectorXd& state, double v_ref) {
  // A matrix is just identity always so we do not need to update it here
  // B matrix is the linearized bicycle model

  // Linearize about the current state and zero input (f(X, 0))
  double theta = state[2];

  A_matrix_.setIdentity();
  A_matrix_(0, 2) = -v_ref * std::sin(theta) * dt_;
  A_matrix_(1, 2) = v_ref * std::cos(theta) * dt_;

  // Linearize the bicycle model
  B_matrix_(0, 0) = std::cos(theta) * dt_;
  B_matrix_(1, 0) = std::sin(theta) * dt_;
  B_matrix_(2, 1) = dt_;
}

void MpcController::init_hessian() {
  hessian_.resize(n_variables_, n_variables_);
  hessian_.setZero();

  // Set up the quadratic cost function (Hessian)
  // It is a giant diagonal matrix with Q_x, Q_y, Q_theta for state errors
  // followed by R_v, R_omega for control costs

  // State cost (x, y, theta errors)
  for (int i = 0; i < NUM_STATES_; i++) {
    hessian_.insert(i * state_dim_, i * state_dim_) = mpc_params_.Q_x;              // x error
    hessian_.insert(i * state_dim_ + 1, i * state_dim_ + 1) = mpc_params_.Q_y;      // y error
    hessian_.insert(i * state_dim_ + 2, i * state_dim_ + 2) = mpc_params_.Q_theta;  // theta error
  }

  // Control cost (v, omega)
  int control_start = state_dim_ * NUM_STATES_;
  for (int i = 0; i < NUM_CONTROLS_; i++) {
    hessian_.insert(control_start + i * control_dim_, control_start + i * control_dim_) = mpc_params_.R_v;
    hessian_.insert(control_start + i * control_dim_ + 1, control_start + i * control_dim_ + 1) = mpc_params_.R_omega;
  }
}

// https://robotology.github.io/osqp-eigen/md_pages_mpc.html
void MpcController::init_linear_constraints_matrix() {
  linearConstraintsMatrix_.resize(n_constraints_, n_variables_);
  linearConstraintsMatrix_.setZero();

  // The linear constraints matrix is really a block matrix

  // The upper left is the state transition matrix A
  // The upper right is the control matrix B
  // The lower left enforces state constraitns
  // The lower right enforces control constraints

  // The state transition matrix A is just the identity matrix
  // so we can set everything except for the top right block

  // Init the top left
  for (int state = 0; state < NUM_STATES_; state++) {
    for (int offset = 0; offset < state_dim_; offset++) {
      linearConstraintsMatrix_.insert(state * state_dim_ + offset, state * state_dim_ + offset) = -1.0;

      if (state == 0) continue;

      // Set A matrix as identity
      linearConstraintsMatrix_.insert(state * state_dim_ + offset, (state - 1) * state_dim_ + offset) = 1.0;
    }
  }

  // Set the bottom of the matrix as one giant identity matrix
  int num_rows = state_dim_ * NUM_STATES_ + control_dim_ * NUM_CONTROLS_;
  int bottom_offset = state_dim_ * NUM_STATES_;
  for (int i = 0; i < num_rows; i++) {
    linearConstraintsMatrix_.insert(bottom_offset + i, i) = 1.0;
  }

  // // For efficiency, we can set ones in the matrix where values for the B matrix will go
  // for (int control = 1; control < mpc_params_.horizon; control++) {
  //   linearConstraintsMatrix_.insert(control * state_dim_, control_col_offset + (control - 1) * control_dim_) = dt_;
  //   linearConstraintsMatrix_.insert(control * state_dim_ + 1, control_col_offset + (control - 1) * control_dim_) =
  //   dt_; linearConstraintsMatrix_.insert(control * state_dim_ + 2, control_col_offset + (control - 1) * control_dim_
  //   + 1) =
  //       dt_;
  // }

  // Initialize the lower and upper bounds too
  lowerBound_.resize(n_constraints_);
  upperBound_.resize(n_constraints_);

  lowerBound_.setZero();
  upperBound_.setZero();

  // Will need to set the bounds for the first state at every iteration
  // to be the current state
  // for (int i = 0; i < state_dim_; i++) {
  //   lowerBound_[i] = 0.0;
  //   upperBound_[i] = 0.0;
  // }

  // Everything after that is zero -- force equality between x_k+1 = x_k +  Bu_k

  // Set the state bounds
  for (int i = 0; i < NUM_STATES_ * state_dim_; i++) {
    lowerBound_[bottom_offset + i] = -OsqpEigen::INFTY;
    upperBound_[bottom_offset + i] = OsqpEigen::INFTY;
  }

  // Set the control bounds
  int control_constraint_offset = bottom_offset + state_dim_ * NUM_STATES_;
  for (int i = 0; i < NUM_CONTROLS_; i++) {
    lowerBound_[control_constraint_offset + i * control_dim_] = 0.0;
    upperBound_[control_constraint_offset + i * control_dim_] = v_max_;

    lowerBound_[control_constraint_offset + i * control_dim_ + 1] = -omega_max_;
    upperBound_[control_constraint_offset + i * control_dim_ + 1] = omega_max_;
  }
}
