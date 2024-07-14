#pragma once

#include <ros/ros.h>
#include <mpc_wrapper.h>

namespace PayloadMPC
{

	class MpcParams
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		struct Q_Gain
		{
			real_t Q_pos_xy;
			real_t Q_pos_z;
			real_t Q_attitude_rp;
			real_t Q_attitude_yaw;
			real_t Q_velocity;
			real_t Q_payload_xy;
			real_t Q_payload_z;
			real_t Q_payload_velocity;
			real_t Q_cable;
			real_t Q_dcable;
		};

		struct R_Gain
		{
			real_t R_thrust;
			real_t R_pitchroll;
			real_t R_yaw;
		};

		struct ForceEstimator
		{
			double sample_freq_fq{100.0}; 
			double sample_freq_fl{100.0}; 
			double cutoff_freq_fq{10.0};
			double cutoff_freq_fl{10.0};
			double imu_body_length;
			double kf;
			double sqrt_kf;
			double max_force; 
			bool use_force_estimator;
			int USE_CONSTANT_MOMENT;
			int max_queue;
			double force_observer_freq;
			double var_weight;
		};

		struct filter
		{
			double sample_freq_quad_acc{333.333};
			double sample_freq_quad_omg{333.333};
			double sample_freq_load_acc{333.333};
			double sample_freq_load_omg{333.333};
			double sample_freq_rpm{333.333}; 
			double sample_freq_cable{333.333};
			double sample_freq_dcable{333.333};

			double cutoff_freq_quad_acc{100.0};
			double cutoff_freq_quad_omg{100.0};
			double cutoff_freq_load_acc{100.0};
			double cutoff_freq_load_omg{100.0};
			double cutoff_freq_rpm{100.0};
			double cutoff_freq_cable{100.0};
			double cutoff_freq_dcable{333.333};

		};
		

		// For real-world experiments
		struct MsgTimeout
		{
			double odom;
			double rc;
			double cmd;
			double imu;
			double bat;
		};

		struct ThrustMapping
		{
			bool print_val;
			double K1;
			double K2;
			double K3;
			int accurate_thrust_model;
			// 0 means use the scale factor to map thrust to pwm without online calibration
			// 1 means use the scale factor to map thrust to pwm with online calibration(use the rotor speed to calibrate the scale factor)
			// 2 means use the scale factor to map thrust to pwm with online calibration(use the IMU calibrate the scale factor confilct with the force estimator)
			// 3 means use the accurate thrust model to map thrust to pwm(Not implemented yet. DON'T use it)
			double hover_percentage;
			double filter_factor;
			// bool noisy_imu;
		};

		struct AutoTakeoffLand
		{
			bool enable;
			bool enable_auto_arm;
			bool no_RC;
			double height;
			double speed;
		};

		struct DynmaicParams
		{
			real_t mass_q;
			real_t mass_l;
			real_t l_length;
		};

		struct RCReverse
		{
			bool roll;
			bool pitch;
			bool yaw;
			bool throttle;
		};

		Q_Gain q_gain_;
		R_Gain r_gain_;
		MsgTimeout msg_timeout_;
		ThrustMapping thr_map_;
		// AutoTakeoffLand takeoff_land_;
		DynmaicParams dyn_params_;
		RCReverse rc_reverse_;

		ForceEstimator force_estimator_param_;
		filter filter_param_;

		real_t gravity_;
		// mpc constraint
		real_t min_thrust_;
		real_t max_thrust_;
		real_t max_bodyrate_xy_;
		real_t max_bodyrate_z_;

		real_t state_cost_exponential_;
		real_t input_cost_exponential_;

		real_t control_command_delay_;
		Eigen::Matrix<real_t, kCostSize, kCostSize> Q_;
		Eigen::Matrix<real_t, kInputSize, kInputSize> R_;
		double step_T_;
		int step_N_;

		double ctrl_freq_max_;

		double pos_y_;
		double pos_x_;

		double takeoff_height_;

		bool use_trajectory_ending_pos_;

		bool print_info_;

		double max_angle_;
		double max_manual_vel_;
		double low_voltage_;

		bool use_simulation_;
		bool use_fix_yaw_;

		MpcParams()
		{
			print_info_ = false;
			state_cost_exponential_ = 0.0;
			input_cost_exponential_ = 0.0;
			min_thrust_ = 0.0;
			max_thrust_ = 0.0;
			max_bodyrate_z_ = 0.0;
			max_bodyrate_xy_ = 0.0;
		}

		~MpcParams()
		{
		}

		void config_from_ros_handle(const ros::NodeHandle &nh)
		{
			read_essential_param(nh, "Q_pos_xy", q_gain_.Q_pos_xy);
			read_essential_param(nh, "Q_pos_z", q_gain_.Q_pos_z);
			read_essential_param(nh, "Q_attitude_rp", q_gain_.Q_attitude_rp);
			read_essential_param(nh, "Q_attitude_yaw", q_gain_.Q_attitude_yaw);
			read_essential_param(nh, "Q_velocity", q_gain_.Q_velocity);
			read_essential_param(nh, "Q_payload_xy", q_gain_.Q_payload_xy);
			read_essential_param(nh, "Q_payload_z", q_gain_.Q_payload_z);
			read_essential_param(nh, "Q_payload_velocity", q_gain_.Q_payload_velocity);
			read_essential_param(nh, "Q_cable", q_gain_.Q_cable);
			read_essential_param(nh, "Q_dcable", q_gain_.Q_dcable);

			read_essential_param(nh, "R_thrust", r_gain_.R_thrust);
			read_essential_param(nh, "R_pitchroll", r_gain_.R_pitchroll);
			read_essential_param(nh, "R_yaw", r_gain_.R_yaw);

			read_essential_param(nh, "min_thrust", min_thrust_);
			read_essential_param(nh, "max_thrust", max_thrust_);
			read_essential_param(nh, "max_bodyrate_xy", max_bodyrate_xy_);
			read_essential_param(nh, "max_bodyrate_z", max_bodyrate_z_);

			read_essential_param(nh, "state_cost_exponential", state_cost_exponential_);
			read_essential_param(nh, "input_cost_exponential", input_cost_exponential_);
			read_essential_param(nh, "step_T", step_T_);
			read_essential_param(nh, "step_N", step_N_);

			read_essential_param(nh, "use_trajectory_ending_pos", use_trajectory_ending_pos_);

			read_essential_param(nh, "mass_l", dyn_params_.mass_l);
			read_essential_param(nh, "l_length", dyn_params_.l_length);
			read_essential_param(nh, "mass_q", dyn_params_.mass_q);
			read_essential_param(nh, "gravity", gravity_);

			read_essential_param(nh, "control_command_delay", control_command_delay_);

			read_essential_param(nh, "pos_x", pos_x_);
			read_essential_param(nh, "pos_y", pos_y_);
			read_essential_param(nh, "takeoff_height", takeoff_height_);

			read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max_);

			read_essential_param(nh, "rc_reverse/roll", rc_reverse_.roll);
			read_essential_param(nh, "rc_reverse/pitch", rc_reverse_.pitch);
			read_essential_param(nh, "rc_reverse/yaw", rc_reverse_.yaw);
			read_essential_param(nh, "rc_reverse/throttle", rc_reverse_.throttle);

			read_essential_param(nh, "thrust_model/print_value", thr_map_.print_val);
			read_essential_param(nh, "thrust_model/K1", thr_map_.K1);
			read_essential_param(nh, "thrust_model/K2", thr_map_.K2);
			read_essential_param(nh, "thrust_model/K3", thr_map_.K3);
			read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map_.accurate_thrust_model);
			read_essential_param(nh, "thrust_model/hover_percentage", thr_map_.hover_percentage);
			read_essential_param(nh, "thrust_model/filter_factor", thr_map_.filter_factor);

			read_essential_param(nh, "filter/sample_freq_quad_acc", filter_param_.sample_freq_quad_acc);
			read_essential_param(nh, "filter/sample_freq_quad_omg", filter_param_.sample_freq_quad_omg);
			read_essential_param(nh, "filter/sample_freq_load_acc", filter_param_.sample_freq_load_acc);
			read_essential_param(nh, "filter/sample_freq_load_omg", filter_param_.sample_freq_load_omg);
			read_essential_param(nh, "filter/sample_freq_rpm", filter_param_.sample_freq_rpm);
			read_essential_param(nh, "filter/cutoff_freq_quad_acc", filter_param_.cutoff_freq_quad_acc);
			read_essential_param(nh, "filter/cutoff_freq_quad_omg", filter_param_.cutoff_freq_quad_omg);
			read_essential_param(nh, "filter/cutoff_freq_load_acc", filter_param_.cutoff_freq_load_acc);
			read_essential_param(nh, "filter/cutoff_freq_load_omg", filter_param_.cutoff_freq_load_omg);
			read_essential_param(nh, "filter/cutoff_freq_rpm", filter_param_.cutoff_freq_rpm);
			read_essential_param(nh, "filter/sample_freq_cable", filter_param_.sample_freq_cable);
			read_essential_param(nh, "filter/cutoff_freq_cable", filter_param_.cutoff_freq_cable);
			read_essential_param(nh, "filter/sample_freq_dcable", filter_param_.sample_freq_dcable);
			read_essential_param(nh, "filter/cutoff_freq_dcable", filter_param_.cutoff_freq_dcable);

			read_essential_param(nh, "force_estimator/kf", force_estimator_param_.kf);
			force_estimator_param_.sqrt_kf= sqrt(force_estimator_param_.kf);
			read_essential_param(nh, "force_estimator/sample_freq_fq", force_estimator_param_.sample_freq_fq);
			read_essential_param(nh, "force_estimator/sample_freq_fl", force_estimator_param_.sample_freq_fl);
			read_essential_param(nh, "force_estimator/cutoff_freq_fq", force_estimator_param_.cutoff_freq_fq);
			read_essential_param(nh, "force_estimator/cutoff_freq_fl", force_estimator_param_.cutoff_freq_fl);
			read_essential_param(nh, "force_estimator/imu_body_length", force_estimator_param_.imu_body_length);
			read_essential_param(nh, "force_estimator/use_force_estimator", force_estimator_param_.use_force_estimator);
			read_essential_param(nh, "force_estimator/USE_CONSTANT_MOMENT", force_estimator_param_.USE_CONSTANT_MOMENT);
			read_essential_param(nh, "force_estimator/max_force", force_estimator_param_.max_force);
			read_essential_param(nh, "force_estimator/max_queue", force_estimator_param_.max_queue);
			read_essential_param(nh, "force_estimator/force_observer_freq", force_estimator_param_.force_observer_freq);
			read_essential_param(nh, "force_estimator/var_weight", force_estimator_param_.var_weight);

			read_essential_param(nh, "use_simulation", use_simulation_);
			read_essential_param(nh, "use_fix_yaw", use_fix_yaw_);
			read_essential_param(nh, "print_info", print_info_);

			// revised by wyz
			read_essential_param(nh, "max_manual_vel", max_manual_vel_);
			read_essential_param(nh, "max_angle", max_angle_);
			read_essential_param(nh, "low_voltage", low_voltage_);

			read_essential_param(nh, "msg_timeout/odom", msg_timeout_.odom);
			read_essential_param(nh, "msg_timeout/rc", msg_timeout_.rc);
			read_essential_param(nh, "msg_timeout/cmd", msg_timeout_.cmd);
			read_essential_param(nh, "msg_timeout/imu", msg_timeout_.imu);
			read_essential_param(nh, "msg_timeout/bat", msg_timeout_.bat);

			max_angle_ /= (180.0 / M_PI);
			/*read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land_.enable);
			read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land_.enable_auto_arm);
			read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land_.no_RC);
			read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land_.height);
			read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land_.speed);
			if ( takeoff_land_.enable_auto_arm && !takeoff_land_.enable )
			{
				takeoff_land_.enable_auto_arm = false;
				ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
			}
			if ( takeoff_land_.no_RC && (!takeoff_land_.enable_auto_arm || !takeoff_land_.enable) )
			{
				takeoff_land_.no_RC = false;
				ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
			}*/

			if (thr_map_.print_val)
			{
				ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
			}
			if (rc_reverse_.roll || rc_reverse_.pitch || rc_reverse_.yaw || rc_reverse_.throttle)
			{
				ROS_WARN("RC reverse is enabled. Becareful when you use it.");
			}
			if (use_simulation_)
			{
				ROS_WARN("You are using simulation. DON'T set this in the real drone.");
			}

			std::cout << "param ended!" << std::endl;
		};
		// void config_full_thrust(double hov);

	private:
		template <typename TName, typename TVal>
		void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
		{
			if (nh.getParam(name, val))
			{
				// pass
			}
			else
			{
				ROS_ERROR_STREAM("Read param: " << name << " failed.");
				ROS_BREAK();
			}
		};
	};
}
