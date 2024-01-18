#include "gtest/gtest.h"
#include "mpc_lateral_controller/mpc.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{


// Aliases for various message types
using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

// Function creates a TrajectoryPoint with specified x, y, and longitudinal velocity.
TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

// Creates an Odometry message with a specified pose and linear velocity.
nav_msgs::msg::Odometry makeOdometry(const geometry_msgs::msg::Pose & pose, const double velocity)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = pose;
  odometry.twist.twist.linear.x = velocity;
  return odometry;
}

// defines a test fixture (MPCTest), including various parameters and test inputs for the MPC lateral controller.
class MPCTest : public ::testing::Test
{
protected:
  MPCParam param;
  // Test inputs
  Trajectory dummy_straight_trajectory;
  Trajectory dummy_right_turn_trajectory;
  SteeringReport neutral_steer;
  Pose pose_zero;
  double default_velocity = 1.0;
  rclcpp::Logger logger = rclcpp::get_logger("mpc_test_logger");

  // Vehicle model parameters
  double wheelbase = 2.7;
  double steer_limit = 1.0;
  double steer_tau = 0.1;
  double mass_fl = 600.0;
  double mass_fr = 600.0;
  double mass_rl = 600.0;
  double mass_rr = 600.0;
  double cf = 155494.663;
  double cr = 155494.663;

  // Filters parameter
  double steering_lpf_cutoff_hz = 3.0;
  double error_deriv_lpf_cutoff_hz = 5.0;

  // Test Parameters
  double admissible_position_error = 5.0;
  double admissible_yaw_error_rad = M_PI_2;
  double steer_lim = 0.610865;      // 35 degrees
  double steer_rate_lim = 2.61799;  // 150 degrees
  double ctrl_period = 0.03;

  bool use_steer_prediction = true;

  TrajectoryFilteringParam trajectory_param;

  //  Function initializes the MPC parameters with specific values.
  void initParam()
  {
    param.prediction_horizon = 50;
    param.prediction_dt = 0.1;
    param.zero_ff_steer_deg = 0.5;
    param.input_delay = 0.0;
    param.acceleration_limit = 2.0;
    param.velocity_time_constant = 0.3;
    param.min_prediction_length = 5.0;
    param.steer_tau = 0.1;
    param.nominal_weight.lat_error = 1.0;
    param.nominal_weight.heading_error = 1.0;
    param.nominal_weight.heading_error_squared_vel = 1.0;
    param.nominal_weight.terminal_lat_error = 1.0;
    param.nominal_weight.terminal_heading_error = 0.1;
    param.low_curvature_weight.lat_error = 0.1;
    param.low_curvature_weight.heading_error = 0.0;
    param.low_curvature_weight.heading_error_squared_vel = 0.3;
    param.nominal_weight.steering_input = 1.0;
    param.nominal_weight.steering_input_squared_vel = 0.25;
    param.nominal_weight.lat_jerk = 0.0;
    param.nominal_weight.steer_rate = 0.0;
    param.nominal_weight.steer_acc = 0.000001;
    param.low_curvature_weight.steering_input = 1.0;
    param.low_curvature_weight.steering_input_squared_vel = 0.25;
    param.low_curvature_weight.lat_jerk = 0.0;
    param.low_curvature_weight.steer_rate = 0.0;
    param.low_curvature_weight.steer_acc = 0.000001;
    param.low_curvature_thresh_curvature = 0.0;

    trajectory_param.traj_resample_dist = 0.1;
    trajectory_param.path_filter_moving_ave_num = 35;
    trajectory_param.curvature_smoothing_num_traj = 1;
    trajectory_param.curvature_smoothing_num_ref_steer = 35;
    trajectory_param.enable_path_smoothing = true;
    trajectory_param.extend_trajectory_for_end_yaw_control = true;

    dummy_straight_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(1.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(2.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(3.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(4.0, 0.0, 1.0f));

    dummy_right_turn_trajectory.points.push_back(makePoint(-1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(2.0, -2.0, 1.0f));

    neutral_steer.steering_tire_angle = 0.0;
    pose_zero.position.x = 0.0;
    pose_zero.position.y = 0.0;
  }

  // Set various parameters for the MPC
  void initializeMPC(mpc_lateral_controller::MPC & mpc)
  {
    mpc.m_param = param;
    mpc.m_admissible_position_error = admissible_position_error;
    mpc.m_admissible_yaw_error_rad = admissible_yaw_error_rad;
    mpc.m_steer_lim = steer_lim;
    mpc.m_steer_rate_lim_map_by_curvature.emplace_back(0.0, steer_rate_lim);
    mpc.m_steer_rate_lim_map_by_velocity.emplace_back(0.0, steer_rate_lim);
    mpc.m_ctrl_period = ctrl_period;
    mpc.m_use_steer_prediction = use_steer_prediction;

    // Initialize filters and steering predictor
    mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
    mpc.initializeSteeringPredictor();

    // Initialize trajectory
    const auto current_kinematics =
      makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
    mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  }

  // Test fixture setup and is executed before each test.
  void SetUp() override
  {
    // Initialize ROS2 node and parameters
    rclcpp::init(0, nullptr);
    initParam();
  }

  // Google Test fixture teardown is executed after each test.
  void TearDown() override
  {
  // Shutdown ROS2 node
  rclcpp::shutdown();
  }
};  // class MPCTest



/* cppcheck-suppress syntaxError */
/*
This is a Google Test case for initializing and calculating MPC.
It creates an MPC object, sets a vehicle model, sets a QP solver, initializes parameters and the reference trajectory,
and then calculates MPC. It checks the expected values of the resulting Ackermann command.
*/
TEST_F(MPCTest, InitializeAndCalculate) // Case-1
{
  // Test fixture for initializing and calculating MPC
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});

  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());

  // Set vehicle model for MPC
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  // Set QP solver for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
 This test case initializes an MPC controller with a specific right turn trajectory, sets up a vehicle model and a QP solver,
 and then calculates the MPC for the given trajectory. The expectations include checking that
 the MPC has the correct components (vehicle model and QP solver), and verifying that the resulting
 control commands for a right turn trajectory meet the expected conditions (steering angle and rotation rate less than 0.0).
*/
TEST_F(MPCTest, InitializeAndCalculateRightTurn) // Case-2
{
  // Similar to the previous test but with a right turn trajectory
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

   // Calculate MPC for a right turn trajectory
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  // Expectations: The calculation of MPC should succeed without errors
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  // Expectations: The resulting steering angle and rotation rate should be less than 0.0
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
This test case initializes an MPC controller with a specific straight trajectory, sets up a vehicle model (KinematicsBicycleModel),
sets up an OSQP (Operator Splitting Quadratic Program) solver for the MPC, and then calculates the MPC for the given trajectory.
The expectations include checking that the MPC has the correct components (vehicle model and QP solver), and verifying that
the resulting control commands for a straight trajectory meet the expected conditions (steering angle and rotation rate equal to 0.0).
*/
TEST_F(MPCTest, OsqpCalculate) // Case-3
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  EXPECT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
 This test case is similar to the "OsqpCalculate" test case but with a different reference trajectory (right turn trajectory).
 The purpose is to verify that the MPC, with the specified parameters and trajectory,
 produces control commands that meet the expected conditions for a right turn (steering angle and rotation rate less than 0.0).
*/
TEST_F(MPCTest, OsqpCalculateRightTurn) // Case-4
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});

  // Create an MPC (Model Predictive Controller) object
  auto mpc = std::make_unique<MPC>(node);

  // Initialize MPC parameters and set a reference trajectory (right turn trajectory)
  initializeMPC(*mpc);

  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  // Set a vehicle model for the MPC (using KinematicsBicycleModel)
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);

  // Expectations: After setting the vehicle model, the MPC should have a vehicle model
  ASSERT_TRUE(mpc->hasVehicleModel());

  // Set a QP solver for the MPC (using QPSolverOSQP)
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  // Expectations: The calculation of MPC should succeed without errors
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  // Expectations: The resulting steering angle and rotation rate should be less than 0.0
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
This test case is designed to verify the behavior of the MPC when using a vehicle model without delay (KinematicsBicycleModelNoDelay).
The purpose is to check that, under specified conditions and a straight trajectory,
the MPC produces control commands with the expected values (0.0 for steering angle and rotation rate).
*/
TEST_F(MPCTest, KinematicsNoDelayCalculate) // Case-5
{
  // Create a ROS2 node for testing
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});

  // Create an MPC (Model Predictive Controller) object
  auto mpc = std::make_unique<MPC>(node);
  // Initialize MPC parameters
  initializeMPC(*mpc);

  // Set a vehicle model for the MPC (using KinematicsBicycleModelNoDelay)
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc->setVehicleModel(vehicle_model_ptr);
  // Expectations: After setting the vehicle model, the MPC should have a vehicle model
  ASSERT_TRUE(mpc->hasVehicleModel());

  // Set a QP solver for the MPC (using QPSolverEigenLeastSquareLLT)
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  // Expectations: After setting the QP solver, the MPC should have a QP solver
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init filters
  mpc->initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  // Init trajectory
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  // Expectations: The calculation of MPC should succeed without errors
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  // Expectations: The resulting steering angle and rotation rate should be 0.0
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
This test case is similar to the "KinematicsNoDelayCalculate" test case but focuses on a right turn trajectory.
The purpose is to check that, under specified conditions and a right turn trajectory,
the MPC using a vehicle model without delay produces control commands with the expected values (steering angle and rotation rate less than 0.0).
*/
TEST_F(MPCTest, KinematicsNoDelayCalculateRightTurn) // Case-6
{
  // Create a ROS2 node for testing
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});

  // Create an MPC (Model Predictive Controller) object
  auto mpc = std::make_unique<MPC>(node);
  // Initialize MPC parameters
  initializeMPC(*mpc);
  // Set a reference trajectory for MPC (right turn trajectory)
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);
  // Set a vehicle model for the MPC (using KinematicsBicycleModelNoDelay)
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_limit);
  mpc->setVehicleModel(vehicle_model_ptr);

  // Expectations: After setting the vehicle model, the MPC should have a vehicle model
  ASSERT_TRUE(mpc->hasVehicleModel());
  // Set a QP solver for the MPC (using QPSolverEigenLeastSquareLLT)
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Expectations: After setting the QP solver, the MPC should have a QP solver
  ASSERT_TRUE(mpc->hasQPSolver());

  // Initialize low-pass filters for MPC
  mpc->initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  // Expectations: The calculation of MPC should succeed without errors
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));

  // Expectations: The resulting steering angle and rotation rate should be less than 0.0
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

/*
This test case focuses on testing the MPC when using a dynamic bicycle model (DynamicsBicycleModel) as the vehicle model.
It checks that, under specified conditions, the MPC produces control commands with the expected values (steering angle and rotation rate equal to 0.0).
*/
TEST_F(MPCTest, DynamicCalculate) // Case-7
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
   // Create an MPC (Model Predictive Controller) object
  auto mpc = std::make_unique<MPC>(node);
  // Initialize MPC parameters
  initializeMPC(*mpc);

  // Set a vehicle model for the MPC (using DynamicsBicycleModel)
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<DynamicsBicycleModel>(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);

  mpc->setVehicleModel(vehicle_model_ptr);
  // Expectations: After setting the vehicle model, the MPC should have a vehicle model
  ASSERT_TRUE(mpc->hasVehicleModel());

  // Set a QP solver for the MPC (using QPSolverEigenLeastSquareLLT)
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Expectations: After setting the QP solver, the MPC should have a QP solver
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  // Expectations: The calculation of MPC should succeed without errors
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  // Expectations: The resulting steering angle and rotation rate should be equal to 0.0
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}
/*
This test case focuses on testing the MPC with multiple calculations, ensuring that the input buffer retains its size after each calculation.
The test checks that the calculated control commands have the expected values (steering angle and rotation rate equal to 0.0)
and that the input buffer maintains its size throughout the calculations.
*/

TEST_F(MPCTest, MultiSolveWithBuffer) // Case-8
{
  // Create a ROS2 node for testing
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});

  // Create an MPC (Model Predictive Controller) object
  auto mpc = std::make_unique<MPC>(node);
  // Set a vehicle model for the MPC (using KinematicsBicycleModel)
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  // Set a QP solver for the MPC (using QPSolverEigenLeastSquareLLT)
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Initialize MPC parameters and reference trajectory
  initializeMPC(*mpc);

  // Set the input buffer for the MPC
  mpc->m_input_buffer = {0.0, 0.0, 0.0};

  // Calculate MPC multiple times and check the results
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  // 1st calculation
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));

  // 2nd calculation
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));

  // 3rd calculation
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));

   // 4th calculation
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
}
/*
This test case focuses on testing failure cases for the MPC. It checks the behavior of the MPC in two scenarios:
1) Pose Too Far from Trajectory
2) Fast Velocity Exceeding Prediction
*/

TEST_F(MPCTest, FailureCases) // Case-9
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(wheelbase, steer_limit, steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC with a pose too far from the trajectory
  Pose pose_far;
  pose_far.position.x = pose_zero.position.x - admissible_position_error - 1.0;
  pose_far.position.y = pose_zero.position.y - admissible_position_error - 1.0;
  AckermannLateralCommand ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_far, default_velocity);
  /*
  1) Pose Too Far from Trajectory:
    It creates a pose (pose_far) that is positioned outside the admissible position error from the reference trajectory.
    It attempts to calculate the MPC with this pose and expects the calculation to fail (EXPECT_FALSE).
  */
  EXPECT_FALSE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag)); // Pose Too Far from Trajectory

  /*
  2) Fast Velocity Exceeding Prediction:
    It calculates the MPC with a pose (pose_far) and a velocity that is significantly higher than the default velocity.
    The goal is to make the prediction go further than the reference path.
    It expects the calculation to fail (EXPECT_FALSE).
  */
  EXPECT_FALSE(mpc->calculateMPC(neutral_steer, makeOdometry(pose_far, default_velocity + 10.0), ctrl_cmd, pred_traj, diag)); // Fast Velocity Exceeding Prediction
}
}  // namespace autoware::motion::control::mpc_lateral_controller
