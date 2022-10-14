/*
 * ground_filter.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: ne0
 *  LICENSE is BSD
 */

/*
 * ground_filter.cpp
 *
 *  Updated on: Oct 15, 2022
 *      Author: HHorimoto
 * 	LICENSE is BSD
 */

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

class GroundFilter
{
public:
	GroundFilter();

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

GroundFilter::GroundFilter() : node_handle_("~")
{

	node_handle_.param<std::string>("subscribe_topic", subscribe_topic_, "/velodyne_points");

	node_handle_.param("remove_floor", floor_removal_, true);
	node_handle_.param("points_distance", points_distance_, 0.1);
	node_handle_.param("angle_threshold", angle_threshold_, 0.35);

	cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &GroundFilter::VelodyneCallback, this);
	cloud_lanes_pub_ = node_handle_.advertise<PointCloud>("/points_lanes", 10);
	cloud_ground_pub_ = node_handle_.advertise<PointCloud>("/points_ground", 10);

	inliers.reset(new PointIndices());
	coefficients.reset(new ModelCoefficients());

	ground_cloud_ptr.reset(new PointCloud());
	lanes_cloud_ptr.reset(new PointCloud());
}

void GroundFilter::RemoveFloor(const PointCloud::ConstPtr &in_cloud_ptr, 
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

void GroundFilter::VelodyneCallback(const PointCloud::ConstPtr &in_sensor_cloud_ptr)
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

int main(int argc, char **argv)
{

	ROS_INFO("Started ground filter");
	ros::init(argc, argv, "ground_filter");

	GroundFilter node;

	ros::spin();

	return 0;

}
