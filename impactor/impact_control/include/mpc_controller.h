#pragma once

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <flatness.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
// #include <state_predictor/state_predictor.h>

#include <queue>
#include "mpc_wrapper.h"
#include "mpc_params.h"
#include "polynomial_trajectory.h"

namespace PayloadMPC
{

  enum STATE
  {
    kPosX = 0,
    kPosY = 1,
    kPosZ = 2,
    kOriW = 3,
    kOriX = 4,
    kOriY = 5,
    kOriZ = 6,
    kVelX = 7,
    kVelY = 8,
    kVelZ = 9,
    kPayloadX = 10,
    kPayloadY = 11,
    kPayloadZ = 12,
    kPayloadVx = 13,
    kPayloadVy = 14,
    kPayloadVz = 15,
    kCableX = 16,
    kCableY = 17,
    kCableZ = 18,
    kDcableX = 19,
    kDcableY = 20,
    kDcableZ = 21,
    // kCableX = 10,
    // kCableY = 11,
    // kCableZ = 12,
    // kDcableX = 13,
    // kDcableY = 14,
    // kDcableZ = 15,

  };

  enum INPUT_BODYRATE
  {
    kThrust = 0,
    kRateX = 1,
    kRateY = 2,
    kRateZ = 3
  };

  struct TorquesAndThrust
  {
    Eigen::Vector3d body_torques;
    double collective_thrust;
  };

  class MpcController
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert(kStateSize == 22,
                  "MpcController: Wrong model size. Number of states does not match.");
    static_assert(kInputSize == 4,
                  "MpcController: Wrong model size. Number of inputs does not match.");

    MpcController(MpcParams &params);

    void execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_state,
                 const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_input,
                 const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw);
    void setTrajectoyReference(Trajectory &traj, double tstart,double start_yaw);
    double getTimeStep(){return mpc_time_step_;}
    void setDynamicParams(const real_t mass_q, const real_t mass_l, const real_t l_length)
      {mpc_wrapper_.setDynamicParams(mass_q, mass_l, l_length);}
    void setExternalForce(const Eigen::Ref<const Eigen::Vector3d>& fl, const Eigen::Ref<const Eigen::Vector3d>& fq)
      {mpc_wrapper_.setExternalForce(fl, fq);fq_=fq; fl_=fl;}
    // Thrust to control
    std::queue<std::pair<ros::Time, double>> timed_thrust;
    double thr_scale_compensate;
    const double rho2 = 0.998; // do not change
    double thrustscale_;
    double P;
    quadrotor_msgs::Px4ctrlDebug debug;

    void resetThrustMapping(void);
    double convertThrust(const double& thrust, const double voltage);
    bool estimateThrustModel(
       const Eigen::Vector3d &est_a,
      const Eigen::Quaterniond &quad_q,
      const Eigen::Vector3d &est_loadp,
      const Eigen::Vector3d &est_loaddp,
      const Eigen::Vector4d &rpm,
      const double voltage,
      const MpcParams &param);
    double AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const MpcParams &param) const;

  private:
    double last_yaw_;
    double last_yaw_dot_;
    // Internal helper functions.

    // void offCallback(const std_msgs::Empty::ConstPtr& msg);
    void calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
    double inline rotor2thrust(const double& sqrt_kf, const Eigen::Vector4d& rpm);
    double inline acc2thrust(const double& Ml, const double& Mq,const double& l_length, const double& g, const double& acc_z,
                              const Eigen::Quaterniond& quad, const Eigen::Vector3d& cable, const Eigen::Vector3d& dcable);
    void preparationThread();
    double angle_limit(double ang);
    double angle_diff(double a, double b);

    // Parameters
    MpcParams& params_;

    Flatness payload_flat_;
    Eigen::Vector3d fq_;
    Eigen::Vector3d fl_;
    // MPC
    MpcWrapper mpc_wrapper_;
    const double mpc_time_step_;

    // Preparation Thread
    std::thread preparation_thread_;

    // Variables
    real_t timing_feedback_, timing_preparation_;
    bool solve_from_scratch_;
    
  public:
    Eigen::Matrix<real_t, kStateSize, kSamples + 1> reference_states_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> reference_inputs_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> hover_input_;
  };

} // namespace MPC
