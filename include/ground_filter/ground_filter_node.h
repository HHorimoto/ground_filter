/*
 * ground_filter_node.h
 *
 *  Updated on: Oct 15, 2022
 *      Author: HHorimoto
 * 	LICENSE is BSD
 */

#ifndef GROUND_FILTER_H__
#define GROUND_FILTER_H__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointIndices PointIndices;
typedef pcl::ModelCoefficients ModelCoefficients;
typedef pcl::SACSegmentation<PointT> SACSegmentation;
typedef pcl::ExtractIndices<PointT> ExtractIndices;


namespace ground_filter
{
    class GroundFilterNone
    {
    public:
        GroundFilterNone();
        ~GroundFilterNone();

    private:
        ros::NodeHandle node_handle_;
        ros::Subscriber cloud_sub_;
        ros::Publisher cloud_lanes_pub_;
        ros::Publisher cloud_ground_pub_;

        std::string subscribe_topic_;

        bool floor_removal_;

        double points_distance_;
        double angle_threshold_;

        SACSegmentation seg;
        PointIndices::Ptr inliers;
        ModelCoefficients::Ptr coefficients;

        PointCloud::Ptr ground_cloud_ptr;
        PointCloud::Ptr lanes_cloud_ptr;

        void VelodyneCallback(const PointCloud::ConstPtr &in_sensor_cloud_ptr);
        void RemoveFloor(const PointCloud::ConstPtr &in_cloud_ptr,
                        PointCloud::Ptr &out_nofloor_cloud_ptr,
                        PointCloud::Ptr &out_onlyfloor_cloud_ptr,
                        float in_max_height,
                        float in_floor_max_angle);    
    };
}

#endif // GROUND_FILTER_H__