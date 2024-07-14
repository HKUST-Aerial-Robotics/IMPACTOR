#include "mpc_input.h"
#include <algorithm> 
RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    // std::cout << mode << std::endl;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[1]) < 1e-5 && abs(ch[2]) < 1e-5 && abs(ch[3]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    rcv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif
}

Start_Trigger_Data_t::Start_Trigger_Data_t()
{
    recv_start_trig = false;
};

void Start_Trigger_Data_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
{
    recv_start_trig = true;
}

Cmd_Trigger_Data_t::Cmd_Trigger_Data_t()
{
    recv_cmd_trig = false;
};

void Cmd_Trigger_Data_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
{
    recv_cmd_trig = true;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    filtered_a = accel_filter.apply(a);
    filtered_w = gyro_filter.apply(w);
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
}

Trajectory_Data_t::Trajectory_Data_t()
{
    total_traj_start_time = ros::Time(0);
    total_traj_end_time = ros::Time(0);
    exec_traj = 0;
}

void Trajectory_Data_t::feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg)
{

    // #1. try to execuse the action
    const quadrotor_msgs::PolynomialTraj &traj = *pMsg;
    if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ADD)
    {
        // exec_traj = false;
        ROS_INFO("[MPCCtrl Traj] Loading the trajectory.");
        if ((int)traj.trajectory_id < 1)
        {
            ROS_ERROR("[MPCCtrl Traj] The trajectory_id must start from 1"); //. zxzxzxzx
            return;
        }
        // if ((int)traj.trajectory_id > 1 && (int)traj.trajectory_id < _traj_id) return ;
        
        oneTraj_Data_t traj_data;
        traj_data.traj_start_time = pMsg->header.stamp;
        double t_total = 0;
        for (auto &piece : traj.trajectory)
        {
            traj_data.traj.emplace_back(piece.duration, Eigen::Map<const Eigen::MatrixXd>(&piece.data[0], piece.num_dim, (piece.num_order + 1)));
            t_total += piece.duration;
        }
        traj_data.traj_end_time = traj_data.traj_start_time + ros::Duration(t_total);
        if (ros::Time::now() > traj_data.traj_start_time  ) //Future traj
        {
            // A future trajectory
            while ((!traj_queue.empty()) && traj_queue.back().traj_start_time > traj_data.traj_start_time)
            {
                traj_queue.pop_back(); //remove old trajectory (remove the traj newer than the new traj)
            }
            traj_queue.push_back(traj_data);
            // adjust_end_time();
            total_traj_end_time = traj_queue.back().traj_end_time;
            total_traj_start_time = traj_queue.front().traj_start_time;
        }
        else
        {
            while ((!traj_queue.empty()) && traj_queue.front().traj_start_time < traj_data.traj_start_time)
            {
                traj_queue.pop_front(); //remove old trajectory
            }
            traj_queue.push_front(traj_data);
            // adjust_end_time();
            total_traj_end_time = traj_queue.back().traj_end_time;
            total_traj_start_time = traj_queue.front().traj_start_time;
        }
        exec_traj = 1;
    }
    else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ABORT)
    {
        ROS_WARN("[MPCCtrl] Aborting the trajectory.");
        total_traj_start_time = ros::Time(0);
        total_traj_end_time = ros::Time(0);
        traj_queue.clear();
        exec_traj = -1;
    }
    else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_WARN_IMPOSSIBLE)
    {
        total_traj_start_time = ros::Time(0);
        total_traj_end_time = ros::Time(0);
        traj_queue.clear();
        exec_traj = -1;
    }
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double vlotage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        vlotage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * vlotage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0); // mark
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else if(percentage < 0.0)
    {
        return; //no battery info
    }
    else 
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}

Rpm_Data_t::Rpm_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Rpm_Data_t::feed(mavros_msgs::ESCTelemetryConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    rpm[0] = pMsg->esc_telemetry[0].rpm;
    rpm[1] = pMsg->esc_telemetry[1].rpm;
    rpm[2] = pMsg->esc_telemetry[2].rpm;
    rpm[3] = pMsg->esc_telemetry[3].rpm;
    rpm_vec(0) = rpm[0];
    rpm_vec(1) = rpm[1];
    rpm_vec(2) = rpm[2];
    rpm_vec(3) = rpm[3];
    filtered_rpm = rpm_filter.apply(rpm_vec);
}