#include "so3_quadrotor/quadrotor_dynamics.hpp"
#include <quadrotor_msgs/SO3Command.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <random>
namespace so3_quadrotor
{

    class Nodelet : public nodelet::Nodelet
    {
    private:
        std::shared_ptr<Quadrotor> quadrotorPtr_;
        Control control_;
        Cmd cmd_;
        ros::Publisher odom_pub_, imu_pub_, vis_pub_, odom_payload_pub_, imu_payload_pub_;
        ros::Subscriber cmd_sub_;
        ros::Timer simulation_timer;
        tf::TransformBroadcaster tf_br_;
        double noise_uav_, noise_payload_, noise_vel_uav_, noise_vel_payload_;
        std::default_random_engine generator;
        std::normal_distribution<double> distribution{0.0, 1.0};
        double l_length;
        // parameters
        int simulation_rate_ = 1e3;
        int odom_rate_ = 400;
        // msgs
        nav_msgs::Odometry odom_msg_;
        nav_msgs::Odometry odom_payload_msg_;
        sensor_msgs::Imu imu_msg_;
        sensor_msgs::Imu imu_payload_msg_; // TODO: Change the message type
        visualization_msgs::MarkerArray vis_msg_;
        double payload_size_;
        double motor_yaw_[4] = {0, 0, 0, 0};

        Eigen::Vector3d guass_random(double std)
        {
            Eigen::Vector3d noise(distribution(generator), distribution(generator), distribution(generator));
            return noise * std;
        }

        void cmd_callback(const mavros_msgs::AttitudeTarget::ConstPtr &cmd_msg)
        {
            cmd_.thrust = cmd_msg->thrust;
            cmd_.body_rates[0] = cmd_msg->body_rate.x;
            cmd_.body_rates[1] = cmd_msg->body_rate.y;
            cmd_.body_rates[2] = cmd_msg->body_rate.z;
        }
        void timer_callback(const ros::TimerEvent &event)
        {
            auto last_control = control_;
            control_ = quadrotorPtr_->getControl(cmd_);
            for (size_t i = 0; i < 4; ++i)
            {
                if (std::isnan(control_.rpm[i]))
                    control_.rpm[i] = last_control.rpm[i];
            }
            quadrotorPtr_->setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2], control_.rpm[3]);
            quadrotorPtr_->step(1.0 / simulation_rate_);
            const Eigen::Vector3d &pos = quadrotorPtr_->getPos() + guass_random(noise_uav_);
            const Eigen::Vector3d &vel = quadrotorPtr_->getVel() + guass_random(noise_vel_uav_);
            const Eigen::Vector3d &acc = quadrotorPtr_->getAcc();
            const Eigen::Quaterniond &quat = quadrotorPtr_->getQuat();
            const Eigen::Vector3d &omega = quadrotorPtr_->getOmega();
            const Eigen::Vector3d &pos_l = quadrotorPtr_->getLoadPos() + guass_random(noise_payload_);
            const Eigen::Vector3d &vel_l = quadrotorPtr_->getLoadVel() + guass_random(noise_vel_payload_);
            const Eigen::Vector3d &q = quadrotorPtr_->getLoadq();
            const Eigen::Vector3d &dq = quadrotorPtr_->getLoaddq();
            const Eigen::Vector3d &pos_real = quadrotorPtr_->getPos();
            const Eigen::Vector3d &pos_l_real = quadrotorPtr_->getLoadPos();
            Eigen::Vector3d fake_q = pos_l - pos;
            fake_q.normalize();
            Eigen::Vector3d fake_dq = (vel_l - vel) / l_length;

            static ros::Time next_odom_pub_time = ros::Time::now();
            ros::Time tnow = ros::Time::now();
            if (tnow >= next_odom_pub_time)
            {
                next_odom_pub_time += ros::Duration(1.0 / odom_rate_);
                // const Eigen::Vector3d&     pos = quadrotorPtr_->getPos() + guass_random(noise_uav_);
                // const Eigen::Vector3d&     vel = quadrotorPtr_->getVel() ;
                // const Eigen::Vector3d&     acc = quadrotorPtr_->getAcc() ;
                // const Eigen::Quaterniond& quat = quadrotorPtr_->getQuat();
                // const Eigen::Vector3d&   omega = quadrotorPtr_->getOmega();
                // const Eigen::Vector3d&   pos_l = quadrotorPtr_->getLoadPos() + guass_random(noise_payload_);
                // const Eigen::Vector3d&   vel_l = quadrotorPtr_->getLoadVel() ;
                // const Eigen::Vector3d&   q = quadrotorPtr_->getLoadq();
                // const Eigen::Vector3d&   dq = quadrotorPtr_->getLoaddq();
                // tf
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
                transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
                tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
                // odom
                odom_msg_.header.stamp = tnow;
                odom_msg_.pose.pose.position.x = pos(0);
                odom_msg_.pose.pose.position.y = pos(1);
                odom_msg_.pose.pose.position.z = pos(2);
                odom_msg_.pose.pose.orientation.x = quat.x();
                odom_msg_.pose.pose.orientation.y = quat.y();
                odom_msg_.pose.pose.orientation.z = quat.z();
                odom_msg_.pose.pose.orientation.w = quat.w();

                odom_msg_.twist.twist.linear.x = vel(0);
                odom_msg_.twist.twist.linear.y = vel(1);
                odom_msg_.twist.twist.linear.z = vel(2);

                odom_msg_.twist.twist.angular.x = omega(0);
                odom_msg_.twist.twist.angular.y = omega(1);
                odom_msg_.twist.twist.angular.z = omega(2);
                odom_pub_.publish(odom_msg_);

                // odom payload
                odom_payload_msg_.header.stamp = tnow;
                odom_payload_msg_.pose.pose.position.x = pos_l(0);
                odom_payload_msg_.pose.pose.position.y = pos_l(1);
                odom_payload_msg_.pose.pose.position.z = pos_l(2);
                odom_payload_msg_.pose.pose.orientation.x = 0;
                odom_payload_msg_.pose.pose.orientation.y = 0;
                odom_payload_msg_.pose.pose.orientation.z = 0;
                odom_payload_msg_.pose.pose.orientation.w = 1;

                odom_payload_msg_.twist.twist.linear.x = vel_l(0);
                odom_payload_msg_.twist.twist.linear.y = vel_l(1);
                odom_payload_msg_.twist.twist.linear.z = vel_l(2);

                odom_payload_msg_.twist.twist.angular.x = dq(0);
                odom_payload_msg_.twist.twist.angular.y = dq(1);
                odom_payload_msg_.twist.twist.angular.z = dq(2);
                odom_payload_pub_.publish(odom_payload_msg_);
                // imu
                imu_msg_.header.stamp = tnow;
                imu_msg_.orientation.x = quat.x();
                imu_msg_.orientation.y = quat.y();
                imu_msg_.orientation.z = quat.z();
                imu_msg_.orientation.w = quat.w();

                imu_msg_.angular_velocity.x = omega(0);
                imu_msg_.angular_velocity.y = omega(1);
                imu_msg_.angular_velocity.z = omega(2);

                imu_msg_.linear_acceleration.x = acc[0];
                imu_msg_.linear_acceleration.y = acc[1];
                imu_msg_.linear_acceleration.z = acc[2];
                imu_pub_.publish(imu_msg_);

                // payload
                imu_payload_msg_.header.stamp = tnow;
                imu_payload_msg_.orientation.x = 0;
                imu_payload_msg_.orientation.y = 0;
                imu_payload_msg_.orientation.z = 0;
                imu_payload_msg_.orientation.w = 1;

                imu_payload_msg_.angular_velocity.x = fake_q(0);
                imu_payload_msg_.angular_velocity.y = fake_q(1);
                imu_payload_msg_.angular_velocity.z = fake_q(2);

                imu_payload_msg_.linear_acceleration.x = fake_dq[0];
                imu_payload_msg_.linear_acceleration.y = fake_dq[1];
                imu_payload_msg_.linear_acceleration.z = fake_dq[2];
                imu_payload_pub_.publish(imu_payload_msg_);
                // drone visualization
                std::vector<Eigen::Vector3d> propellers;
                propellers.emplace_back(0, +quadrotorPtr_->config.arm_length / 2, 0.02);
                propellers.emplace_back(0, -quadrotorPtr_->config.arm_length / 2, 0.02);
                propellers.emplace_back(-quadrotorPtr_->config.arm_length / 2, 0, 0.02);
                propellers.emplace_back(+quadrotorPtr_->config.arm_length / 2, 0, 0.02);
                for (size_t i = 0; i < 4; ++i)
                {
                    double rpm = quadrotorPtr_->state.motor_rpm.coeff(i);
                    if (i / 2)
                    {
                        motor_yaw_[i] += rpm * 60 / odom_rate_;
                    }
                    else
                    {
                        motor_yaw_[i] -= rpm * 60 / odom_rate_;
                    }
                    motor_yaw_[i] = std::fmod(motor_yaw_[i], 2 * M_PI);
                    Eigen::Quaterniond quat_propeller = quat * uav_utils::ypr_to_quaternion(
                                                                   Eigen::Vector3d(motor_yaw_[i], 0, 0));
                    Eigen::Vector3d pos_propeller = pos_real + quat.toRotationMatrix() * propellers[i];

                    vis_msg_.markers[i].pose.position.x = pos_propeller(0);
                    vis_msg_.markers[i].pose.position.y = pos_propeller(1);
                    vis_msg_.markers[i].pose.position.z = pos_propeller(2);
                    vis_msg_.markers[i].pose.orientation.x = quat_propeller.x();
                    vis_msg_.markers[i].pose.orientation.y = quat_propeller.y();
                    vis_msg_.markers[i].pose.orientation.z = quat_propeller.z();
                    vis_msg_.markers[i].pose.orientation.w = quat_propeller.w();
                }
                for (size_t i = 4; i < 6; ++i)
                {
                    vis_msg_.markers[i].pose.position.x = pos_real(0);
                    vis_msg_.markers[i].pose.position.y = pos_real(1);
                    vis_msg_.markers[i].pose.position.z = pos_real(2);
                    vis_msg_.markers[i].id = i;
                }
                vis_msg_.markers[6].pose.orientation.w = 1;
                vis_msg_.markers[6].pose.position.x = pos_l_real(0);
                vis_msg_.markers[6].pose.position.y = pos_l_real(1);
                vis_msg_.markers[6].pose.position.z = pos_l_real(2);

                vis_msg_.markers[7].points.clear();
                vis_msg_.markers[7].pose.orientation.w = 1;
                geometry_msgs::Point p;
                p.x = pos_l_real(0);
                p.y = pos_l_real(1);
                p.z = pos_l_real(2);
                vis_msg_.markers[7].points.push_back(p);
                p.x = pos_real(0);
                p.y = pos_real(1);
                p.z = pos_real(2);
                vis_msg_.markers[7].points.push_back(p);

                vis_pub_.publish(vis_msg_);
            }
        }

    public:
        void onInit(void)
        {
            ros::NodeHandle nh(getPrivateNodeHandle());
            // parameters
            double init_x, init_y, init_z, init_yaw, init_q1, init_q2, init_q3;
            nh.getParam("init_x", init_x);
            nh.getParam("init_y", init_y);
            nh.getParam("init_z", init_z);
            nh.getParam("init_yaw", init_yaw);
            nh.param("init_q1", init_q1, 0.0);
            nh.param("init_q2", init_q2, 0.0);
            nh.param("init_q3", init_q3, -1.0);
            // config of quadrotor
            Config config;
            nh.getParam("g", config.g);
            nh.getParam("mass", config.mass);
            nh.getParam("mass_load", config.mass_l);
            nh.getParam("l_length", config.l_length);
            l_length = config.l_length;
            double Ixx, Iyy, Izz;
            nh.getParam("Ixx", Ixx);
            nh.getParam("Iyy", Iyy);
            nh.getParam("Izz", Izz);
            config.J = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();
            nh.getParam("kf", config.kf);
            double prop_radius;
            nh.getParam("prop_radius", prop_radius);
            config.km = 0.07 * (3 * prop_radius) * config.kf;
            nh.getParam("arm_length", config.arm_length);
            nh.getParam("motor_time_constant", config.motor_time_constant);
            nh.getParam("max_rpm", config.max_rpm);
            nh.getParam("min_rpm", config.min_rpm);
            nh.getParam("simulation_rate", simulation_rate_);
            nh.getParam("odom_rate", odom_rate_);
            nh.getParam("payload_size", payload_size_);

            nh.param("noise_uav", noise_uav_, 0.0);
            nh.param("noise_payload", noise_payload_, 0.0);
            nh.param("noise_vel_uav", noise_vel_uav_, 0.0);
            nh.param("noise_vel_payload", noise_vel_payload_, 0.0);

            std::vector<double> kom;
            nh.getParam("kOmega", kom);
            for (size_t i = 0; i < 3; i++)
            {
                config.kOmega[i] = kom[i];
            }
            std::vector<double> kdom;
            nh.getParam("kdOmega", kdom);
            for (size_t i = 0; i < 3; i++)
            {
                config.kdOmega[i] = kdom[i];
            }

            quadrotorPtr_ = std::make_shared<Quadrotor>(config);
            quadrotorPtr_->setPos(Eigen::Vector3d(init_x, init_y, init_z));
            quadrotorPtr_->setYpr(Eigen::Vector3d(init_yaw, 0, 0));
            double rpm = sqrt(config.mass * config.g / 4 / config.kf);
            quadrotorPtr_->setRpm(Eigen::Vector4d(rpm, rpm, rpm, rpm));
            quadrotorPtr_->setq(Eigen::Vector3d(init_q1, init_q2, init_q3));
            Eigen::Quaterniond quat = uav_utils::ypr_to_quaternion(Eigen::Vector3d(init_yaw, 0, 0));
            cmd_.thrust = (config.mass + config.mass_l) * config.g;
            cmd_.body_rates.setZero();
            odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
            odom_payload_pub_ = nh.advertise<nav_msgs::Odometry>("odom_payload", 100);
            imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);
            imu_payload_pub_ = nh.advertise<sensor_msgs::Imu>("imu_payload", 10);
            vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vis", 10);
            cmd_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>("so3cmd", 10, &Nodelet::cmd_callback, this, ros::TransportHints().tcpNoDelay());
            simulation_timer = nh.createTimer(ros::Duration(1.0 / simulation_rate_), &Nodelet::timer_callback, this);

            odom_msg_.header.frame_id = "world";
            imu_msg_.header.frame_id = "world";
            vis_msg_.markers.resize(4);
            visualization_msgs::Marker propeller;
            propeller.header.frame_id = "world";
            propeller.type = visualization_msgs::Marker::CYLINDER;
            propeller.action = visualization_msgs::Marker::ADD;
            propeller.scale.x = 2 * prop_radius;
            propeller.scale.y = 0.1 * prop_radius;
            propeller.scale.z = 0.05 * prop_radius;
            propeller.color.a = 0.7;
            propeller.color.r = 0.6;
            propeller.color.g = 0.7;
            propeller.color.b = 0.8;
            for (size_t i = 0; i < 4; ++i)
            {
                vis_msg_.markers[i] = propeller;
                vis_msg_.markers[i].id = i;
            }
            visualization_msgs::Marker drone_body = propeller;
            drone_body.type = visualization_msgs::Marker::SPHERE;
            drone_body.color.a = 1.0;
            drone_body.scale.x = config.arm_length;
            drone_body.scale.y = 0.1 * config.arm_length;
            drone_body.scale.z = 0.05 * config.arm_length;
            vis_msg_.markers.push_back(drone_body);
            drone_body.scale.y = config.arm_length;
            drone_body.scale.x = 0.1 * config.arm_length;
            vis_msg_.markers.push_back(drone_body);

            visualization_msgs::Marker payload = propeller;
            payload.type = visualization_msgs::Marker::SPHERE;
            payload.color.a = 1.0;
            payload.color.r = 0.3;
            payload.color.g = 0.4;
            payload.color.b = 0.5;
            payload.scale.x = payload_size_ * 2;
            payload.scale.y = payload_size_ * 2;
            payload.scale.z = payload_size_ * 2;
            payload.id = (int32_t)vis_msg_.markers.size();
            vis_msg_.markers.push_back(payload);
            visualization_msgs::Marker payload_line = payload;
            payload_line.type = visualization_msgs::Marker::LINE_STRIP;
            payload_line.scale.x = 0.02;
            payload_line.id = (int32_t)vis_msg_.markers.size();
            vis_msg_.markers.push_back(payload_line);
            // drone_body.scale.y = config.arm_length;
            // drone_body.scale.x = 0.1 * config.arm_length;
            // vis_msg_.markers.push_back(payload);
        };
    };

} // so3_quadrotor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(so3_quadrotor::Nodelet, nodelet::Nodelet);
