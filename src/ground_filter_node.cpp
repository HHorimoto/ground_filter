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
		node_handle_.param("points_distance", points_distance_, 0.1);
		node_handle_.param("angle_threshold", angle_threshold_, 0.35);

		cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &GroundFilterNone::VelodyneCallback, this);
		cloud_lanes_pub_ = node_handle_.advertise<PointCloud>("/points_lanes", 10);
		cloud_ground_pub_ = node_handle_.advertise<PointCloud>("/points_ground", 10);

		inliers.reset(new PointIndices());
		coefficients.reset(new ModelCoefficients());

		ground_cloud_ptr.reset(new PointCloud());
		lanes_cloud_ptr.reset(new PointCloud());
	}
	GroundFilterNone::~GroundFilterNone(){}
	void GroundFilterNone::RemoveFloor(const PointCloud::ConstPtr &in_cloud_ptr,
								   PointCloud::Ptr &out_nofloor_cloud_ptr,
								   PointCloud::Ptr &out_onlyfloor_cloud_ptr,
								   float in_max_distance,
								   float in_floor_max_angle)
	{

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
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

		RemoveFloor(in_sensor_cloud_ptr, lanes_cloud_ptr, ground_cloud_ptr, points_distance_, angle_threshold_);

		if (floor_removal_)
		{
			/*PUBLISH LANES CLOUD*/
			lanes_cloud_ptr->header = in_sensor_cloud_ptr->header;
			cloud_lanes_pub_.publish(lanes_cloud_ptr);
		}
		else
		{
			cloud_lanes_pub_.publish(in_sensor_cloud_ptr);
		}

		/*PUBLISH GROUND CLOUD*/
		ground_cloud_ptr->header = in_sensor_cloud_ptr->header;
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
