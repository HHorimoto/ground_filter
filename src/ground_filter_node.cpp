/*
 * ground_filter_node.cpp
 *
 *  Created on: Oct 15, 2022
 *      Author: HHorimoto
 * 	LICENSE is BSD
 */

/*
 * Some lines are derived from
 * https://github.com/PolySync/Autoware/tree/master/ros/src/sensing/filters/packages/points_preprocessor.
 *
 * ground_filter.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: ne0
 *  LICENSE is BSD
 */

#include <ground_filter/ground_filter_node.h>

namespace ground_filter
{
	GroundFilterNone::GroundFilterNone() : node_handle_("~")
	{

		node_handle_.param<std::string>("subscribe_topic", subscribe_topic_, "/velodyne_points");

		node_handle_.param("remove_floor", floor_removal_, true);
		node_handle_.param("points_distance", points_distance_, float(0.1));
		node_handle_.param("angle_threshold", angle_threshold_, float(0.35));
		node_handle_.param("voxel_size_x", voxel_size_x_, float(0.065));
		node_handle_.param("voxel_size_y", voxel_size_y_, float(0.065));
		node_handle_.param("voxel_size_z", voxel_size_z_, float(0.065));

		cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &GroundFilterNone::VelodyneCallback, this);
		cloud_lanes_pub_ = node_handle_.advertise<PointCloud>("/points_lanes", 10);
		cloud_ground_pub_ = node_handle_.advertise<PointCloud>("/points_ground", 10);

		inliers.reset(new PointIndices());
		coefficients.reset(new ModelCoefficients());

		ground_cloud_ptr.reset(new PointCloud());
		lanes_cloud_ptr.reset(new PointCloud());
		processed_ptr.reset(new PointCloud());
		nonan_ptr.reset(new PointCloud());
	}
	GroundFilterNone::~GroundFilterNone() {}
	void GroundFilterNone::Preprocessing(const PointCloud::ConstPtr &in_cloud_ptr, 
										PointCloud::Ptr &removed_nan_ptr, 
										PointCloud::Ptr &out_cloud_ptr, 
										float size_x, 
										float size_y, 
										float size_z)
	{
		removed_nan_ptr->points.reserve(in_cloud_ptr->points.size());
		out_cloud_ptr->points.reserve(in_cloud_ptr->points.size());
		out_cloud_ptr->header = in_cloud_ptr->header;
		
		// Remvoe NaN
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*in_cloud_ptr, *removed_nan_ptr, indices);

		// Voxel filter
		VoxelGrid filter;
		filter.setInputCloud(removed_nan_ptr);
		filter.setLeafSize(size_x, size_y, size_z);
		filter.filter(*out_cloud_ptr);
	}

	void GroundFilterNone::RemoveFloor(const PointCloud::ConstPtr &in_cloud_ptr,
									   PointCloud::Ptr &out_nofloor_cloud_ptr,
									   PointCloud::Ptr &out_onlyfloor_cloud_ptr,
									   float in_max_distance,
									   float in_floor_max_angle)
	{
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000);
		seg.setAxis(Eigen::Vector3f(0, 0, 1));
		seg.setEpsAngle(in_floor_max_angle);

		seg.setDistanceThreshold(in_max_distance); // floor distance
		seg.setOptimizeCoefficients(true);
		seg.setInputCloud(in_cloud_ptr);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			ROS_WARN("Could not estimate a planar model for the given dataset.");
		}

		/*REMOVE THE FLOOR FROM THE CLOUD*/
		ExtractIndices extract;
		extract.setInputCloud(in_cloud_ptr);
		extract.setIndices(inliers);
		extract.setNegative(true); // true removes the indices, false leaves only the indices
		extract.filter(*out_nofloor_cloud_ptr);

		/*EXTRACT THE FLOOR FROM THE CLOUD*/
		extract.setNegative(false); // true removes the indices, false leaves only the indices
		extract.filter(*out_onlyfloor_cloud_ptr);
	}

	void GroundFilterNone::VelodyneCallback(const PointCloud::ConstPtr &in_sensor_cloud_ptr)
	{
		Header header = in_sensor_cloud_ptr->header;
		// Preprocessing PointCloud
		Preprocessing(in_sensor_cloud_ptr, nonan_ptr, processed_ptr, voxel_size_x_, voxel_size_y_, voxel_size_z_);

		// Ground Segmentation by ransac
		RemoveFloor(processed_ptr, lanes_cloud_ptr, ground_cloud_ptr, points_distance_, angle_threshold_);

		if (floor_removal_)
		{
			/*PUBLISH LANES CLOUD*/
			lanes_cloud_ptr->header = header;
			cloud_lanes_pub_.publish(lanes_cloud_ptr);
		}
		else
		{
			cloud_lanes_pub_.publish(in_sensor_cloud_ptr);
		}

		/*PUBLISH GROUND CLOUD*/
		ground_cloud_ptr->header = header;
		cloud_ground_pub_.publish(ground_cloud_ptr);
	}
}

int main(int argc, char **argv)
{

	ROS_INFO("Started ground filter");
	ros::init(argc, argv, "ground_filter");

	ground_filter::GroundFilterNone node;

	ros::spin();

	return 0;
}
