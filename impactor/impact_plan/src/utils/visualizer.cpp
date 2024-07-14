/************************************************************************
 * Date:        2023.02
 * Author:      Haokun Wang <hwangeh at connect dot ust dot hk>, 
                Aerial Robotics Group <https://uav.ust.hk>, HKUST.
 * E-mail:      hwangeh_at_connect_dot_ust_dot_hk
 * Description: This is the source file for the visualization tools,
 *              which is used to visualize the trajectory and the
 *              planned path.
 * License:     GNU General Public License <http://www.gnu.org/licenses/>.
 * Project:     IMPACTOR is free software: you can redistribute it and/or 
 *              modify it under the terms of the GNU Lesser General Public 
 *              License as published by the Free Software Foundation, 
 *              either version 3 of the License, or (at your option) any 
 *              later version.
 *              IMPACTOR is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty 
 *              of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *              See the GNU General Public License for more details.
 * Website:     https://github.com/HKUST-Aerial-Robotics/IMPACTOR
 ************************************************************************/
 
#include "utils/visualizer.hpp"

namespace utils {
void Visualizer::init(ros::NodeHandle& nh) {
	a_star_path_pub1 =
	    nh.advertise<visualization_msgs::Marker>("a_star_quad_path", 2);
	a_star_path_pub2 =
	    nh.advertise<visualization_msgs::Marker>("a_star_load_path", 2);
	init_path1 = nh.advertise<visualization_msgs::Marker>("init_quad_path", 2);
	init_path2 = nh.advertise<visualization_msgs::Marker>("init_load_path", 2);
	waypoint_pub1 =
	    nh.advertise<visualization_msgs::Marker>("waypoint_quad", 2);
	waypoint_pub2 =
	    nh.advertise<visualization_msgs::Marker>("waypoint_load", 2);
	ellipsoidPub =
	    nh.advertise<visualization_msgs::MarkerArray>("drone_status", 1000);

	tensionPub = nh.advertise<std_msgs::Float64>("/visualizer/tension", 1000);
	distancePub = nh.advertise<std_msgs::Float64>("/visualizer/distance", 1000);

	loadPosPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/loadPos", 1000);
	quadPosPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/quadPos", 1000);
	loadVelPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/loadVel", 1000);
	quadVelPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/quadVel", 1000);
	loadAccPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/loadAcc", 1000);
	quadAccPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/quadAcc", 1000);
	loadJerPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/loadJerk", 1000);
	quadJerPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/quadJerk", 1000);
	loadSnaPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/loadSnap", 1000);
	quadSnaPub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/quadSnap", 1000);

	omgPub = nh.advertise<geometry_msgs::PoseStamped>("/visualizer/omg", 1000);
	anglePub =
	    nh.advertise<geometry_msgs::PoseStamped>("/visualizer/angle", 1000);
	thrustPub = nh.advertise<std_msgs::Float64>("/visualizer/thrust", 1000);

	meshPub =
	    nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
	edgePub =
	    nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
}

void Visualizer::displayMarkerList(ros::Publisher& pub,
                                   const vector<Eigen::Vector3d>& marker_list,
                                   double scale, Eigen::Vector4d color, int id,
                                   bool show_sphere, /* = true */
                                   bool show_line /* = true */) {
	visualization_msgs::Marker sphere, line_strip;
	sphere.header.frame_id = line_strip.header.frame_id = "world";
	sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
	sphere.type = visualization_msgs::Marker::SPHERE_LIST;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
	sphere.id = id;
	line_strip.id = id + 1000;

	sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	sphere.color.r = line_strip.color.r = color(0);
	sphere.color.g = line_strip.color.g = color(1);
	sphere.color.b = line_strip.color.b = color(2);
	sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
	sphere.scale.x = scale;
	sphere.scale.y = scale;
	sphere.scale.z = scale;
	line_strip.scale.x = scale / 2;
	geometry_msgs::Point pt;
	for (int i = 0; i < int(marker_list.size()); i++) {
		pt.x = marker_list[i](0);
		pt.y = marker_list[i](1);
		pt.z = marker_list[i](2);
		if (show_sphere)
			sphere.points.push_back(pt);
		if (show_line)
			line_strip.points.push_back(pt);
	}
	if (show_sphere)
		pub.publish(sphere);
	if (show_line)
		pub.publish(line_strip);
}

void Visualizer::quat2EulerRad(const Eigen::Vector4d& q,
                               Eigen::Vector3d& euler) {
	euler.setZero();
	double t0 = (q(1) + q(3)) * (q(1) - q(3));     // x^2-z^2 0
	double t1 = (q(0) + q(2)) * (q(0) - q(2));     // w^2-y^2 1
	double xx = 0.5 * (t0 + t1);                   // 1/2 x^2 0.5
	double xy = (q(1) * q(2) + q(0) * q(3));       // 1/2 x*y 0
	double xz = (q(0) * q(2) - q(1) * q(3));       // 1/2 x*z 0
	double t = xx * xx + xy * xy;                  // cos^2(theta) 0.25
	double yz = 2.0 * (q(2) * q(3) + q(0) * q(1)); // 1/2 y*z 0

	double z = atan2(xy, xx);
	double y = atan(xz / sqrt(t));
	double x = 0;
	if (t != 0) {
		x = atan2(yz, t1 - t0);
	} else {
		x = (2.0 * atan2(q(1), q(0) - copysignf(1.f, xz) * z));
	}
	normalizeAngles(Eigen::Vector3d(x, y, z), euler);
}

void Visualizer::normalizeAngles(const Eigen::Vector3d& angles,
                                 Eigen::Vector3d& normalized_angles) {
	double x = normalizeAngle(angles(0));
	double y = normalizeAngle(angles(1));
	double z = normalizeAngle(angles(2));
	normalized_angles << x, y, z;
}

double Visualizer::normalizeAngle(double angle) {
	double new_angle = angle;
	while (new_angle > M_PI)
		new_angle -= 2 * M_PI;
	while (new_angle < -M_PI)
		new_angle += 2 * M_PI;
	return new_angle;
}

void Visualizer::B2W(const Eigen::Vector3d& euler, Eigen::Vector3d& omg) {
	Eigen::Matrix3d T;
	T.setZero();
	T(0, 0) = 1.0;
	T(0, 1) = sin(euler(0)) * tan(euler(1));
	T(0, 2) = cos(euler(0)) * tan(euler(1));
	T(1, 1) = cos(euler(0));
	T(1, 2) = -sin(euler(0));
	T(2, 1) = sin(euler(0)) / cos(euler(1));
	T(2, 2) = cos(euler(0)) / cos(euler(1));
	omg = T * omg;
}

void Visualizer::visualizeDoubleball(
    const poly_traj::Trajectory::Ptr& trajL,
    const poly_traj::Trajectory::Ptr& trajQ, const int samples,
    const double& PayloadSize, const double& QuadrotorSize,
    const double& gravity, const double& load_mass, const bool& real_time) {
	visualization_msgs::Marker ballMarker;
	visualization_msgs::Marker droneMarker;
	visualization_msgs::Marker LineMarker;
	visualization_msgs::MarkerArray ellipsoidMarkers;
	flatness::FlatnessMap flatmap;
	flatmap.reset(0.8, 0.005, gravity, 0.0001);

	ballMarker.id = 0;
	ballMarker.type = visualization_msgs::Marker::SPHERE_LIST;
	ballMarker.header.stamp = ros::Time::now();
	ballMarker.header.frame_id = "world";
	ballMarker.pose.orientation.w = 1.00;
	ballMarker.action = visualization_msgs::Marker::ADD;
	ballMarker.ns = "Balls";
	ballMarker.color.r = 231.0 / 255.0;
	ballMarker.color.g = 205.0 / 255.0;
	ballMarker.color.b = 121.0 / 255.0;
	ballMarker.color.a = 0.8;
	ballMarker.scale.x = PayloadSize;
	ballMarker.scale.y = PayloadSize;
	ballMarker.scale.z = PayloadSize;
	ballMarker.pose.orientation.w = 1;
	ballMarker.pose.orientation.x = 0;
	ballMarker.pose.orientation.y = 0;
	ballMarker.pose.orientation.z = 0;

	droneMarker = ballMarker;
	droneMarker.id = 1;
	droneMarker.color.r = 70.0 / 255.0;
	droneMarker.color.g = 120.0 / 255.0;
	droneMarker.color.b = 150.0 / 255.0;
	droneMarker.color.a = 0.6;
	droneMarker.scale.x = QuadrotorSize;
	droneMarker.scale.y = QuadrotorSize;
	droneMarker.scale.z = QuadrotorSize;

	LineMarker = droneMarker;
	LineMarker.id = 2;
	LineMarker.type = visualization_msgs::Marker::LINE_LIST;
	LineMarker.scale.x = 0.02;
	LineMarker.scale.y = 0.02;
	LineMarker.scale.z = 0.02;

	ballMarker.action = visualization_msgs::Marker::DELETEALL;
	ellipsoidMarkers.markers.push_back(ballMarker);
	ellipsoidPub.publish(ellipsoidMarkers);

	ballMarker.action = visualization_msgs::Marker::ADD;
	ellipsoidMarkers.markers.clear();

	uint t_sample = samples * trajL->getPieceNum();
	double dt = trajL->getTotalDuration() / t_sample;
	geometry_msgs::Point load_point, quad_point;
	geometry_msgs::PoseStamped loadPos, quadPos;
	geometry_msgs::PoseStamped loadVel, quadVel;
	geometry_msgs::PoseStamped loadAcc, quadAcc;
	geometry_msgs::PoseStamped loadJer, quadJer;
	geometry_msgs::PoseStamped loadSna, quadSna;
	geometry_msgs::PoseStamped OMG, eulerAngle;
	Eigen::Vector3d load_pos, load_vel, load_acc, load_jer, load_sna;
	Eigen::Vector3d quad_pos, quad_vel, quad_acc, quad_jer, quad_sna;
	double thr;
	Eigen::Vector4d quat;
	Eigen::Vector3d euler, omg;
	ros::Time cur_time = ros::Time::now();
	for (uint64_t i = 0; i <= t_sample; i++) {
		if (real_time) {
			ros::Duration(dt).sleep();
		}
		cur_time += ros::Duration(dt);
		load_pos = trajL->getPos(dt * i);
		load_vel = trajL->getVel(dt * i);
		load_acc = trajL->getAcc(dt * i);
		load_jer = trajL->getJer(dt * i);
		load_sna = trajL->getSna(dt * i);
		quad_pos = trajQ->getPos(dt * i);
		quad_vel = trajQ->getVel(dt * i);
		quad_acc = trajQ->getAcc(dt * i);
		quad_jer = trajQ->getJer(dt * i);
		quad_sna = trajQ->getSna(dt * i);

		flatmap.forward(load_acc, load_jer, quad_acc, quad_jer, 0.0, 0.0, thr,
		                quat, omg);
		quat2EulerRad(quat, euler);
		// B2W(euler, omg);

		load_point.x = load_pos(0);
		load_point.y = load_pos(1);
		load_point.z = load_pos(2);
		quad_point.x = quad_pos(0);
		quad_point.y = quad_pos(1);
		quad_point.z = quad_pos(2);

		ballMarker.points.push_back(load_point);
		droneMarker.points.push_back(quad_point);
		LineMarker.points.push_back(quad_point);
		LineMarker.points.push_back(load_point);
		if (i >= 1000 * samples) {
			ballMarker.points.erase(ballMarker.points.begin());
			droneMarker.points.erase(droneMarker.points.begin());
			LineMarker.points.erase(LineMarker.points.begin());
			LineMarker.points.erase(LineMarker.points.begin());
		}
		ellipsoidMarkers.markers.clear();
		ellipsoidMarkers.markers.push_back(ballMarker);
		ellipsoidMarkers.markers.push_back(droneMarker);
		ellipsoidMarkers.markers.push_back(LineMarker);
		ellipsoidPub.publish(ellipsoidMarkers);

		loadPos.pose.position.x = load_pos(0);
		loadPos.pose.position.y = load_pos(1);
		loadPos.pose.position.z = load_pos(2);
		loadPos.header.stamp = cur_time;
		loadPos.header.frame_id = "world";

		loadVel.pose.position.x = load_vel(0);
		loadVel.pose.position.y = load_vel(1);
		loadVel.pose.position.z = load_vel(2);
		loadVel.header.stamp = cur_time;
		loadVel.header.frame_id = "world";

		loadAcc.pose.position.x = load_acc(0);
		loadAcc.pose.position.y = load_acc(1);
		loadAcc.pose.position.z = load_acc(2);
		loadAcc.header.stamp = cur_time;
		loadAcc.header.frame_id = "world";

		loadJer.pose.position.x = load_jer(0);
		loadJer.pose.position.y = load_jer(1);
		loadJer.pose.position.z = load_jer(2);
		loadJer.header.stamp = cur_time;
		loadJer.header.frame_id = "world";

		loadSna.pose.position.x = load_sna(0);
		loadSna.pose.position.y = load_sna(1);
		loadSna.pose.position.z = load_sna(2);
		loadSna.header.stamp = cur_time;
		loadSna.header.frame_id = "world";

		quadPos.pose.position.x = quad_pos(0);
		quadPos.pose.position.y = quad_pos(1);
		quadPos.pose.position.z = quad_pos(2);
		quadPos.header.stamp = cur_time;
		quadPos.header.frame_id = "world";

		quadVel.pose.position.x = quad_vel(0);
		quadVel.pose.position.y = quad_vel(1);
		quadVel.pose.position.z = quad_vel(2);
		quadVel.header.stamp = cur_time;
		quadVel.header.frame_id = "world";

		quadAcc.pose.position.x = quad_acc(0);
		quadAcc.pose.position.y = quad_acc(1);
		quadAcc.pose.position.z = quad_acc(2);
		quadAcc.header.stamp = cur_time;
		quadAcc.header.frame_id = "world";

		quadSna.pose.position.x = quad_sna(0);
		quadSna.pose.position.y = quad_sna(1);
		quadSna.pose.position.z = quad_sna(2);
		quadSna.header.stamp = cur_time;
		quadSna.header.frame_id = "world";

		quadJer.pose.position.x = quad_jer(0);
		quadJer.pose.position.y = quad_jer(1);
		quadJer.pose.position.z = quad_jer(2);
		quadJer.header.stamp = cur_time;
		quadJer.header.frame_id = "world";

		OMG.pose.position.x = omg(0);
		OMG.pose.position.y = omg(1);
		OMG.pose.position.z = omg(2);
		OMG.header.stamp = cur_time;
		OMG.header.frame_id = "world";

		eulerAngle.pose.position.x = euler(0);
		eulerAngle.pose.position.y = euler(1);
		eulerAngle.pose.position.z = euler(2);
		eulerAngle.header.stamp = cur_time;
		eulerAngle.header.frame_id = "world";

		Eigen::Vector3d Tp = load_mass * (trajL->getAcc(dt * i) +
		                                  Eigen::Vector3d(0, 0, gravity));
		Eigen::Vector3d p = Tp / Tp.norm();
		std_msgs::Float64 tension, distance, thrust;
		tension.data = Tp.norm();
		distance.data = (load_pos - quad_pos).norm();
		thrust.data = thr;
		if (real_time) {
			loadPosPub.publish(loadPos);
			loadVelPub.publish(loadVel);
			loadAccPub.publish(loadAcc);
			loadJerPub.publish(loadJer);
			loadSnaPub.publish(loadSna);
			quadPosPub.publish(quadPos);
			quadVelPub.publish(quadVel);
			quadAccPub.publish(quadAcc);
			quadJerPub.publish(quadJer);
			quadSnaPub.publish(quadSna);
			omgPub.publish(OMG);
			anglePub.publish(eulerAngle);
			tensionPub.publish(tension);
			distancePub.publish(distance);
			thrustPub.publish(thrust);
		}
	}
}

void Visualizer::displayAStarPath(vector<Eigen::VectorXd> init_pts,
                                  const double scale, int id) {
	if ((a_star_path_pub1.getNumSubscribers() == 0) &&
	    (a_star_path_pub2.getNumSubscribers() == 0)) {
		return;
	}
	vector<Eigen::Vector3d> quad_pts, load_pts;
	for (int i = 0; i < init_pts.size(); i++) {
		load_pts.push_back(init_pts[i].head(3));
		quad_pts.push_back(init_pts[i].tail(3));
	}
	displayMarkerList(a_star_path_pub1, quad_pts, scale,
	                  Eigen::Vector4d(0, 0, 1, 0.4), id);
	displayMarkerList(a_star_path_pub2, load_pts, scale,
	                  Eigen::Vector4d(0, 1, 0, 0.4), id);
}

void Visualizer::displayInitPath(Eigen::MatrixXd init_pts, const double scale,
                                 int id) {
	if ((init_path1.getNumSubscribers() == 0) &&
	    (init_path2.getNumSubscribers() == 0)) {
		return;
	}
	vector<Eigen::Vector3d> quad_pts, load_pts;
	for (int i = 0; i < init_pts.cols(); i++) {
		load_pts.push_back(init_pts.col(i).head(3));
		quad_pts.push_back(init_pts.col(i).tail(3));
	}
	displayMarkerList(init_path1, quad_pts, scale,
	                  Eigen::Vector4d(0, 0, 1, 0.2), id);
	displayMarkerList(init_path2, load_pts, scale,
	                  Eigen::Vector4d(0, 1, 0, 0.2), id);
}

// visualize the waypoints
void Visualizer::displayWaypoints(vector<Eigen::VectorXd> waypoints,
                                  const double scale, int id) {
	if ((waypoint_pub1.getNumSubscribers() == 0) &&
	    (waypoint_pub2.getNumSubscribers() == 0)) {
		return;
	}
	vector<Eigen::Vector3d> quad_pts, load_pts;
	for (int i = 1; i < waypoints.size() - 1; i++) {
		load_pts.push_back(waypoints[i].head(3));
		quad_pts.push_back(waypoints[i].tail(3));
	}
	displayMarkerList(waypoint_pub1, quad_pts, scale,
	                  Eigen::Vector4d(0.698, 0.2588, 0.4745, 1.0), id, true,
	                  false);
	displayMarkerList(waypoint_pub2, load_pts, scale,
	                  Eigen::Vector4d(0.855, 0.8078, 0.8078, 1.0), id, true,
	                  false);
}

} // namespace utils
