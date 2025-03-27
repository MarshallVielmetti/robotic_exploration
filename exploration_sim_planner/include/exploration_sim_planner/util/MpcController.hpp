/**
 * @file MpcController.hpp
 * @brief This file declares the MpcController class which manages state between
 * MPC solver runs
 */

#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

class MpcController {
 public:
  /**
   * @brief Configuration parameters for Model Predictive Control (MPC).
   *
   * This struct contains the tuning parameters for an MPC controller used in
   * vehicle/robot navigation. The Q parameters represent weights for state
   * errors in the cost function, the R parameters represent weights for control
   * inputs in the cost function, and horizon defines how many steps ahead the
   * controller predicts.
   *
   * Higher Q values prioritize tracking accuracy, while higher R values promote
   * smoother/more conservative control actions.
   */
  struct MPCParameters {
    double Q_x = 1.0;      // Weight for x position error
    double Q_y = 1.0;      // Weight for y position error
    double Q_theta = 1.0;  // Weight for orientation error
    double R_v = 0.1;      // Weight for linear velocity control
    double R_omega = 0.1;  // Weight for angular velocity control
    int horizon = 10;      // Prediction horizon
  };

  /**
   * @brief Construct a new MpcController object
   * @param wheelbase Wheelbase
   * @param dt Time step
   * @param v_max Maximum linear velocity
   * @param omega_max Maximum angular velocity
   * @param params Initial MPC parameters
   */
  MpcController(/*double wheelbase, */ double dt, double v_max, double omega_max, const MPCParameters& params);

  /**
   * @brief Destructor
   */
  ~MpcController() = default;

  /**
   * @brief Compute control commands for trajectory following
   * @param state Current robot state
   * @param theta Current robot orientation
   * @param path Reference trajectory to follow
   * @return Control commands (linear and angular velocity)
   */
  Eigen::Vector2d compute_control(const Eigen::Vector2d& state, double theta, double v_ref,
                                  const std::vector<Eigen::Vector2d>& path);

  /**
   * @brief Set MPC parameters
   * @param params New MPC parameters
   */
  void set_parameters(const MPCParameters& params);

  /**
   * @brief Get current MPC parameters
   * @return Current MPC parameters
   */
  const MPCParameters& get_parameters() const;

  //  private:
  // Vehicle parameters
  //   double wheelbase_;
  double dt_;
  double v_max_;
  double omega_max_;

  // MPC parameters
  MPCParameters mpc_params_;

  // Solver instance using OsqpEigen
  OsqpEigen::Solver solver_;
  bool is_initialized_{false};

  // Problem matrices
  Eigen::SparseMatrix<double> hessian_;
  Eigen::VectorXd gradient_;
  Eigen::SparseMatrix<double> linearConstraintsMatrix_;
  Eigen::VectorXd lowerBound_;
  Eigen::VectorXd upperBound_;

  // Eigen::SparseMatrix<double> A_matrix_;
  // Eigen::SparseMatrix<double> B_matrix_;
  Eigen::MatrixXd A_matrix_;
  Eigen::MatrixXd B_matrix_;

  const int state_dim_ = 3;
  const int control_dim_ = 2;
  int NUM_STATES_;
  int NUM_CONTROLS_;
  int n_variables_;
  int n_constraints_;

  // Initialize the solver with problem dimensions and structure
  void initialize_solver();

  // Update problem matrices for the current state and reference trajectory
  void update_problem_matrices(const Eigen::VectorXd& state, double v_ref,
                               const std::vector<Eigen::VectorXd>& reference);

  // Update the A and B matrices to be linearized about the current state and zero input (f(X, 0))
  void linearize_about_state(const Eigen::VectorXd& state, double v_ref);

  // Sets up the Hessian Matrix P = diag(Q, Q, ..., Q_N, R, ..., R_N)
  // The hessian never needs to change after it is initialized
  void init_hessian();

  void init_linear_constraints_matrix();
};