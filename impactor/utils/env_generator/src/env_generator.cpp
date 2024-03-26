#include "env_generator/env_generator.hpp"

namespace env_generator
{
    void MapGenerator::init(ros::NodeHandle &nh)
    {
        nh_ = nh;
        map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/env_generator/global_cloud", 1);
        vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/env_generator/visual_cloud", 1);
        nh.param("/env_generator/map/resolution", generator_params_.resolution_, 0.1);
    }

    void MapGenerator::generateWall(std::vector<Eigen::VectorXd> &params,
                                    pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        int x_num_, y_num_, z_num_;
        double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_, theta_;
        double px_, py_, pz_;
        pcl::PointXYZ pt_;

        for (int i = 0; i < params.size(); i++)
        {
            x_l_ = params[i](0);
            x_h_ = params[i](1);
            y_l_ = params[i](2);
            y_h_ = params[i](3);
            z_l_ = params[i](4);
            z_h_ = params[i](5);
            theta_ = params[i](6);

            x_num_ = ceil((x_h_ - x_l_) / generator_params_.resolution_);
            y_num_ = ceil((y_h_ - y_l_) / generator_params_.resolution_);
            z_num_ = ceil((z_h_ - z_l_) / generator_params_.resolution_);

            for (int i = 0; i < x_num_; i++)
            {
                for (int j = 0; j < y_num_; j++)
                {
                    for (int k = 0; k < z_num_; k++)
                    {
                        px_ = i * generator_params_.resolution_;
                        py_ = j * generator_params_.resolution_;
                        pz_ = k * generator_params_.resolution_;
                        pt_.x = x_l_ + px_ * cos(theta_) + py_ * sin(theta_);
                        pt_.y = y_l_ - px_ * sin(theta_) + py_ * cos(theta_);
                        pt_.z = z_l_ + pz_;
                        cloud_map.push_back(pt_);
                    }
                }
            }
        }
    }

    void MapGenerator::generateBox(std::vector<Eigen::VectorXd> &params,
                                   pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        std::vector<Eigen::VectorXd> box_params_;
        Eigen::VectorXd box_param_(7);
        for (int i = 0; i < params.size(); i++)
        {
            box_param_(0) = params[i](0) - params[i](3) / 2.0;
            box_param_(1) = params[i](0) + params[i](3) / 2.0;
            box_param_(2) = params[i](1) - params[i](3) / 2.0;
            box_param_(3) = params[i](1) + params[i](3) / 2.0;
            box_param_(4) = params[i](2);
            box_param_(5) = params[i](2) + params[i](4);
            box_param_(6) = params[i](5);
            box_params_.push_back(box_param_);
        }
        generateWall(box_params_, cloud_map);
    }

    void MapGenerator::generateRing(std::vector<Eigen::VectorXd> &params,
                                    pcl::PointCloud<pcl::PointXYZ> &cloud_map)
    {
        pcl::PointXYZ pt_;
        double x_, y_, z_, radius_x_, radius_y_, theta_;
        Eigen::Vector3d cpt_, translation_;
        Eigen::Matrix3d rotate_;

        for (int i = 0; i < params.size(); i++)
        {
            x_ = floor(params[i](0) / 0.01) * 0.01 + 0.01 / 2.0;
            y_ = floor(params[i](1) / 0.01) * 0.01 + 0.01 / 2.0;
            z_ = floor(params[i](2) / 0.01) * 0.01 + 0.01 / 2.0;
            radius_x_ = params[i](3);
            radius_y_ = params[i](4);
            theta_ = params[i](5);

            translation_ = Eigen::Vector3d(x_, y_, z_);
            rotate_ << cos(theta_), -sin(theta_), 0.0, sin(theta_), cos(theta_), 0.0, 0.0, 0.0, 1.0;

            for (double angle_ = 0.0; angle_ < 6.282; angle_ += 0.01 / 2.0)
            {
                cpt_ = rotate_ * Eigen::Vector3d(0.0, radius_x_ * cos(angle_), radius_y_ * sin(angle_)) + translation_;
                pt_.x = cpt_(0);
                pt_.y = cpt_(1);
                pt_.z = cpt_(2);
                cloud_map.push_back(pt_);
            }
        }
    }

    void MapGenerator::generateMap(void)
    {
        std::vector<Eigen::VectorXd> params_;
        int ring_num_, box_num_, wall_num_;
        nh_.param("/env_generator/obstacle/ring_num", ring_num_, 0);
        nh_.param("/env_generator/obstacle/box_num", box_num_, 0);
        nh_.param("/env_generator/obstacle/wall_num", wall_num_, 0);
        Eigen::VectorXd param_;

        // generate ring
        for (int i = 0; i < ring_num_; i++)
        {
            param_.resize(6);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/x", param_(0), 0.0);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/y", param_(1), 0.0);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/z", param_(2), 0.0);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/radius_x", param_(3), 0.0);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/radius_y", param_(4), 0.0);
            nh_.param("/env_generator/obstacle/ring_" + std::to_string(i) + "/theta", param_(5), 0.0);
            params_.push_back(param_);
        }
        generateRing(params_, visual_map_);
        params_.clear();

        // generate box
        for (int i = 0; i < box_num_; i++)
        {
            param_.resize(6);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/x", param_(0), 0.0);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/y", param_(1), 0.0);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/z", param_(2), 0.0);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/width", param_(3), 0.0);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/height", param_(4), 0.0);
            nh_.param("/env_generator/obstacle/box_" + std::to_string(i) + "/theta", param_(5), 0.0);
            params_.push_back(param_);
        }
        generateBox(params_, visual_map_);
        params_.clear();

        // generate wall
        for (int i = 0; i < wall_num_; i++)
        {
            param_.resize(7);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/x_l", param_(0), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/x_h", param_(1), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/y_l", param_(2), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/y_h", param_(3), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/z_l", param_(4), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/z_h", param_(5), 0.0);
            nh_.param("/env_generator/obstacle/wall_" + std::to_string(i) + "/theta", param_(6), 0.0);
            params_.push_back(param_);
        }
        generateWall(params_, cloud_map_);
        params_.clear();

        std::cout << "ring num: " << ring_num_ << std::endl;
        std::cout << "box num: " << box_num_ << std::endl;
        std::cout << "wall num: " << wall_num_ << std::endl;

        // publish map
        cloud_map_.width = cloud_map_.points.size();
        cloud_map_.height = 1;
        cloud_map_.is_dense = true;
        visual_map_.width = visual_map_.points.size();
        visual_map_.height = 1;
        visual_map_.is_dense = true;

        kdtreeLocalMap_.setInputCloud(cloud_map_.makeShared());
        kdtreeVisualMap_.setInputCloud(visual_map_.makeShared());

        generator_params_.map_ok_ = true;
    }

    void MapGenerator::pubPoints(void)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if (generator_params_.map_ok_)
                break;
        }
        pcl::toROSMsg(cloud_map_, globalMap_pcd_);
        pcl::toROSMsg(visual_map_, visualMap_pcd_);
        globalMap_pcd_.header.frame_id = "world";
        visualMap_pcd_.header.frame_id = "world";
        map_pub_.publish(globalMap_pcd_);
        vis_pub_.publish(visualMap_pcd_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_map_sensing");
    ros::NodeHandle n("~");

    env_generator::MapGenerator::Ptr generator_;

    generator_.reset(new env_generator::MapGenerator());
    generator_->init(n);
    generator_->generateMap();

    ros::Rate loop_rate(1.0);

    // real map
    while (ros::ok())
    {
        generator_->pubPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}