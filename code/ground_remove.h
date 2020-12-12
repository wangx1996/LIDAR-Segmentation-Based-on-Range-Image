/*
   Copyright (c) 2020 WX96

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/



#ifndef GROUND_REMOVE2_H
#define GROUND_REMOVE2_H

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <thread>
#include <mutex>  
//PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
//Eigen
#include <Eigen/Dense>
/*//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_driver_msgs/GpswithHeading.h>*/
//using namespace message_filters;

struct PointCloudXYZRDP{
PCL_ADD_POINT4D;
float range;
int ring;
int pcaketnum;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCloudXYZRDP,
(float,x,x)
(float,y,y)
(float,z,z)
(int,ring,ring)
(float,range,range)
(int,pcaketnum,pcaketnum)
)

class GroundRemove {

public:

	GroundRemove(int num_iter, int num_lpr, double th_seeds, double th_dist);
	void extract_initial_seeds_(const pcl::PointCloud<PointCloudXYZRDP>& p_sorted,
			pcl::PointCloud<PointCloudXYZRDP>& g_seeds_pc);
	void estimate_plane_(const pcl::PointCloud<PointCloudXYZRDP>& g_ground_pc);
	void RemoveGround_Thread(pcl::PointCloud<PointCloudXYZRDP>& cloudIn,
			pcl::PointCloud<PointCloudXYZRDP>& cloudgc,
			pcl::PointCloud<PointCloudXYZRDP>& cloudngc,
			pcl::PointCloud<PointCloudXYZRDP>& g_ground_pc1,
			pcl::PointCloud<PointCloudXYZRDP>& g_not_ground_pc1);
	void RemoveGround(pcl::PointCloud<PointCloudXYZRDP>& cloudIn,
			pcl::PointCloud<PointCloudXYZRDP>& g_ground_pc,
			pcl::PointCloud<PointCloudXYZRDP>& g_not_ground_pc);

private:

	std::mutex regionmutex;
	Eigen::MatrixXf normal_;
	float th_dist_d_ = 0;
	int num_iter_ = 3;
	int num_lpr_ = 20;
	double th_seeds_ = 1.0;
	double th_dist_ = 0.15;
	int num_seg_ = 3;

};

#endif
