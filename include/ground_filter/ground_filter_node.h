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
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointIndices PointIndices;
typedef pcl::ModelCoefficients ModelCoefficients;
typedef pcl::SACSegmentation<PointT> SACSegmentation;
typedef pcl::ExtractIndices<PointT> ExtractIndices;
typedef pcl::VoxelGrid<PointT> VoxelGrid;
typedef pcl::PCLHeader Header;

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

        float points_distance_;
        float angle_threshold_;

        float voxel_size_x_;
        float voxel_size_y_;
        float voxel_size_z_;

        SACSegmentation seg;
        PointIndices::Ptr inliers;
        ModelCoefficients::Ptr coefficients;

        PointCloud::Ptr ground_cloud_ptr;
        PointCloud::Ptr lanes_cloud_ptr;
        PointCloud::Ptr nonan_ptr;
        PointCloud::Ptr processed_ptr;

        void VelodyneCallback(const PointCloud::ConstPtr &in_sensor_cloud_ptr);
        void RemoveFloor(const PointCloud::ConstPtr &in_cloud_ptr,
                         PointCloud::Ptr &out_nofloor_cloud_ptr,
                         PointCloud::Ptr &out_onlyfloor_cloud_ptr,
                         float in_max_height,
                         float in_floor_max_angle);
        void Preprocessing(const PointCloud::ConstPtr &in_cloud_ptr, 
                            PointCloud::Ptr &removed_nan_ptr, 
                            PointCloud::Ptr &out_cloud_ptr, 
                            float size_x, 
                            float size_y, 
                            float size_z);
    };
}

#endif // GROUND_FILTER_H__