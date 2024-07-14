#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

namespace env_generator
{
    class MapGenerator
    {
    public:
        void init(ros::NodeHandle &nh);
        void pubPoints(void);
        void generateMap(void);

        typedef std::unique_ptr<MapGenerator> Ptr;

    private:
        ros::NodeHandle nh_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap_, kdtreeVisualMap_;

        ros::Publisher map_pub_, vis_pub_;

        struct GeneratorParameters
        {
            double resolution_;

            double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_;
            double radius_l_, radius_h_;
            double w_l_, w_h_, h_l_, h_h_;
            double theta_;

            bool map_ok_ = false;
        } generator_params_;

        sensor_msgs::PointCloud2 globalMap_pcd_, visualMap_pcd_;
        pcl::PointCloud<pcl::PointXYZ> cloud_map_, visual_map_;

        void generateWall(std::vector<Eigen::VectorXd> &params,
                          pcl::PointCloud<pcl::PointXYZ> &cloud_map);
        void generateBox(std::vector<Eigen::VectorXd> &params,
                         pcl::PointCloud<pcl::PointXYZ> &cloud_map);
        void generateRing(std::vector<Eigen::VectorXd> &params,
                          pcl::PointCloud<pcl::PointXYZ> &cloud_map);
    };
}
