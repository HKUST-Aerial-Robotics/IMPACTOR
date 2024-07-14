#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include <geometry_msgs/PoseStamped.h>
//#include <quadrotor_msgs/Trajectory.h>
#include <mavros_msgs/ESCTelemetry.h>
#include <polynomial_trajectory.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <deque>
#include "lowpassfilter2p.h"

class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q;
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp{0};
  bool rcv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Start_Trigger_Data_t
{
public:

  bool recv_start_trig;

  Start_Trigger_Data_t();
  void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};

class Cmd_Trigger_Data_t
{
public:

  bool recv_cmd_trig;

  Cmd_Trigger_Data_t();
  void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_a{Eigen::Vector3d::Zero()};

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp{0};

  LowPassFilter2p<Eigen::Vector3d> accel_filter;
  LowPassFilter2p<Eigen::Vector3d> gyro_filter;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
  void set_filter_params(double accel_sample_freq, double accel_cutoff_freq, double gyro_sample_freq, double gyro_cutoff_freq)
  {
    accel_filter.set_cutoff_frequency(accel_sample_freq, accel_cutoff_freq);
    accel_filter.reset(a);
    gyro_filter.set_cutoff_frequency(gyro_sample_freq, gyro_cutoff_freq);
    gyro_filter.reset(w);
  }
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

struct oneTraj_Data_t
{
  public:
  ros::Time traj_start_time{0};
  ros::Time traj_end_time{0};
  PayloadMPC::Trajectory traj;
};

class Trajectory_Data_t
{
public:
  ros::Time total_traj_start_time{0};
  ros::Time total_traj_end_time{0};
  int exec_traj = 0; // use for aborting the trajectory, 0 means no trajectory is executing  
                      // -1 means the trajectory is aborting, 1 means the trajectory is executing 
  std::deque<oneTraj_Data_t> traj_queue;
  
  Trajectory_Data_t();
  void adjust_end_time()
  {
    if (traj_queue.size() < 2)
    {
      return;
    }
    for (auto it = traj_queue.begin(); it != (traj_queue.end()-1); it++)
    {
      it->traj_end_time = (it+1)->traj_start_time;
    }
    
  }
  void feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp{0};

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp{0};

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp{0};

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

class Rpm_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mavros_msgs::ESCTelemetry msg;
  ros::Time rcv_stamp{0};
  int rpm[4]{0,0,0,0};
  Eigen::Vector4d rpm_vec{Eigen::Vector4d::Zero()};
  Eigen::Vector4d filtered_rpm;
  LowPassFilter2p<Eigen::Vector4d> rpm_filter;

  Rpm_Data_t();
  void feed(mavros_msgs::ESCTelemetryConstPtr pMsg);
  void set_filter_params(double sample_freq, double cutoff_freq)
  {
    rpm_filter.set_cutoff_frequency(sample_freq, cutoff_freq);
    rpm_filter.reset(rpm_vec);
  }
};

#endif
