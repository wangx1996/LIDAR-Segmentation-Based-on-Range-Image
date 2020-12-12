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

#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "ground_remove.h"


const float PI = 3.1415926;

struct Range{
	float z;
	float range;
	float planerange;
	int index;
	int row;
	int col;
};

static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

float Polar_angle_cal(float x, float y) {
	float temp_tangle = 0;
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * PI;
	}
	return temp_tangle;
}

//可视化
template<typename T> std::string toString(const T& t) {
	std::ostringstream oss;
	oss << t;
	return oss.str();
}

template<typename PointInT>
float CalculateRangeXY(const PointInT pointIn) {

	return sqrt(pointIn.x * pointIn.x + pointIn.y * pointIn.y);
}

template<typename PointInT>
float CalculateRangeZXY(const PointInT pointIn) {

	return sqrt(
			pointIn.x * pointIn.x + pointIn.y * pointIn.y
					+ (pointIn.z-2.1) * (pointIn.z-2.1));
}



template<typename PointInT>
void CloudFilter(const pcl::PointCloud<PointInT>& cloudIn,
		pcl::PointCloud<PointInT>& cloudOut, float x_min, float x_max,
		float y_min, float y_max, float z_min, float z_max) {
	cloudOut.header = cloudIn.header;
	cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
	cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
	cloudOut.points.clear();
	//1) set parameters for removing cloud reflect on ego vehicle
	float x_limit_min = -0.8, x_limit_max = 0.8, y_limit_forward = 3.5,
			y_limit_backward = -1.0;
	//2 apply the filter
	for (int i = 0; i < cloudIn.size(); ++i) {
		float x = cloudIn.points[i].x;
		float y = cloudIn.points[i].y;
		float z = cloudIn.points[i].z;
		// whether on ego vehicle
		if ((x > x_limit_min && x < x_limit_max && y > y_limit_backward
				&& y < y_limit_forward))
			continue;
		if ((x > x_min && x < x_max && y > y_min && y < y_max && z > z_min
				&& z < z_max)) {

			cloudOut.points.push_back(cloudIn.points[i]);

		}

	}
	cout<<"LLL " <<cloudOut.points.size()<<endl;
}

int total_frame = 0;

void PreProcess(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, pcl::PointCloud<PointCloudXYZRDP>& cloudOut){

	total_frame = int(cloudIn.points.size() / 32) - 1;

	cout<<cloudIn.points.size()<<endl;

        cloudOut.points.resize(cloudIn.points.size());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorcloud(
				new pcl::PointCloud<pcl::PointXYZRGB>);


	for (int j = 0; j < total_frame; ++j) {
		int num_odd = 16;                //基数从16位开始排
		int num_even = 0;                //偶数从头

		for (int i = 0; i < 32; ++i) {
			if (float(i % 2) == 0.0) {
				cloudOut.points[j * 32 + num_even].x = cloudIn.points[j * 32 + i].x ;
				cloudOut.points[j * 32 + num_even].y = cloudIn.points[j * 32 + i].y ;
				cloudOut.points[j * 32 + num_even].z = cloudIn.points[j * 32 + i].z ;
				cloudOut.points[j * 32 + num_even].ring =num_even;
				cloudOut.points[j * 32 + num_even].pcaketnum =j;
				cloudOut.points[j * 32 + num_even].range =cloudIn.points[j * 32 + i].intensity;
	
				pcl::PointXYZRGB p;
				p.x = cloudIn.points[j * 32 + i].x;
				p.y = cloudIn.points[j * 32 + i].y;
				p.z = cloudIn.points[j * 32 + i].z;
				p.r = 100;
				p.g = 100;
				p.b = 100;
				colorcloud->points.push_back(p);		
				num_even++;

			} else {
				cloudOut.points[j * 32 + num_odd].x  = cloudIn.points[j * 32 + i].x;
				cloudOut.points[j * 32 + num_odd].y = cloudIn.points[j * 32 + i].y ;
				cloudOut.points[j * 32 + num_odd].z = cloudIn.points[j * 32 + i].z ;
				cloudOut.points[j * 32 + num_odd].ring =num_odd;
				cloudOut.points[j * 32 + num_odd].pcaketnum =j;
				cloudOut.points[j * 32 + num_odd].range =cloudIn.points[j * 32 + i].intensity;

				pcl::PointXYZRGB p;
				p.x = cloudIn.points[j * 32 + i].x;
				p.y = cloudIn.points[j * 32 + i].y;
				p.z = cloudIn.points[j * 32 + i].z;
				p.r = 100;
				p.g = 100;
				p.b = 100;
				colorcloud->points.push_back(p);		
				num_odd++;
			}
		} //按索引顺序排列
        }

	for(int j = 0; j < total_frame; ++j) {
		bool flag = false;
		std::pair<int,int> p;
		std::vector<std::pair<int,int>> vt;
		for (int i = 1; i < 31; ++i) {
			if(!flag && cloudOut.points[j * 32 + i].range<=0.1){
				flag = true;
				p.first = i;
			}
			if(flag && cloudOut.points[j * 32 + i + 1].range > 0.1){
				p.second = i;
				flag = false;
				vt.push_back(p);
			}
		}

		for(int i=0; i<vt.size(); ++i){
			float dist = (cloudOut.points[j * 32 + vt[i].second + 1].range - cloudOut.points[j * 32 + vt[i].first - 1].range);
			float disring = vt[i].second - vt[i].first +1;
			float pre = cloudOut.points[j * 32 + vt[i].first - 1].range;

			float dx = (cloudOut.points[j * 32 + vt[i].second + 1].x - cloudOut.points[j * 32 + vt[i].first - 1].x)/disring;
			float dy = (cloudOut.points[j * 32 + vt[i].second + 1].y - cloudOut.points[j * 32 + vt[i].first - 1].y)/disring;
			float dz = (cloudOut.points[j * 32 + vt[i].second + 1].z - cloudOut.points[j * 32 + vt[i].first - 1].z)/disring;

			float x = cloudOut.points[j * 32 + vt[i].first - 1].x;
			float y = cloudOut.points[j * 32 + vt[i].first - 1].y;
			float z = cloudOut.points[j * 32 + vt[i].first - 1].z;
			if(fabs(dist) < 0.3){
				for(int k= vt[i].first; k<=vt[i].second ; ++k){
					pre += dist/disring;
					x += dx;
					y += dy;
					z += dz;
					cloudOut.points[j * 32 + k ].range  = pre;
					cloudOut.points[j * 32 + k ].x = x;
					cloudOut.points[j * 32 + k ].y = y;
					cloudOut.points[j * 32 + k ].z = z;
					cloudOut.points[j * 32 + k ].ring =k;
					cloudOut.points[j * 32 + k ].pcaketnum =j;


					pcl::PointXYZRGB p;
					p.x = cloudOut.points[j * 32 + k ].x;
					p.y = cloudOut.points[j * 32 + k ].y;
					p.z = cloudOut.points[j * 32 + k ].z;
					p.r = 0;
					p.g = 0;
					p.b = 255;
					colorcloud->points.push_back(p);		
				}
			}
		}
	}


	cloudOut.height = 1;
	cloudOut.width = cloudOut.points.size();

	colorcloud->height = 1;
	colorcloud->width = colorcloud->points.size();
}

void toRangeImage(const pcl::PointCloud<PointCloudXYZRDP>& cloudIn, pcl::PointCloud<PointCloudXYZRDP>& cloudObstacle, pcl::PointCloud<PointCloudXYZRDP>& cloudGround, std::unordered_map<int, Range> &unordered_map_out){
	
	pcl::PointCloud<PointCloudXYZRDP>::Ptr cloud_filtered(
			new pcl::PointCloud<PointCloudXYZRDP>);


	float xmin = -30, xmax = 30, ymin = -50, ymax = 50, zmin = -1.0, zmax = 3.0;
        CloudFilter(cloudIn, *cloud_filtered, xmin, xmax, ymin, ymax, zmin, zmax);

   	int64_t tm0 = gtm();
	GroundRemove ground_remove(3, 20, 1.0, 0.15);
	ground_remove.RemoveGround(*cloud_filtered, cloudGround, cloudObstacle);
   	int64_t tm1 = gtm();
   	printf("[INFO]ground remove cast time:%ld us\n",  tm1-tm0);
	
	//view
	cv::Mat image = cv::Mat::zeros(32, total_frame, CV_8UC3);
  	int scale = 200, min_dis = 1, max_dis = 30;


	for(int i = 0; i < cloudObstacle.points.size(); ++i){
		int col = cloudObstacle.points[i].pcaketnum;
		int row = 31 - cloudObstacle.points[i].ring;
		int imageindex = row * total_frame + col;
		if(!unordered_map_out.count(imageindex)){
			Range r;
			r.col = col;
			r.row = row;
			r.index = i;
			r.range = cloudObstacle.points[i].range;
			r.z = cloudObstacle.points[i].z;
			r.planerange = CalculateRangeXY(cloudObstacle.points[i]);
			unordered_map_out[imageindex] = r;	
    			int color = scale - ((r.range - min_dis) / (max_dis - min_dis))*scale;
			image.at<cv::Vec3b>(row,col) = cv::Vec3b(color,color,color);

		}
	}

	//ground on the image 
	/*for(int i = 0; i < cloudGround.points.size(); ++i){
		int col = cloudGround.points[i].pcaketnum;
		int row = 31 - cloudGround.points[i].ring;
    		//int color = scale - ((cloudGround.points[i].range; - min_dis) / (max_dis - min_dis))*scale;
		image.at<cv::Vec3b>(row,col) = cv::Vec3b(0,0,255);
	}*/

	/*cv::resize(image, image, cv::Size(1000, 100));
	cv::imshow("test", image);
	cv::waitKey(0);*/
}


void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}


std::vector<int> RangeSeg(const pcl::PointCloud<PointCloudXYZRDP>& cloudIn, pcl::PointCloud<PointCloudXYZRDP>& cloudbreak,std::unordered_map<int, Range> &unordered_map_out){

	int direction[6][2] = {{0,1},{0,2},{0,-1},{0,-2},{-1,0},{1,0}};

	int size = cloudIn.points.size();
	std::vector<int> cluster_indices(size, -1);

	float horizon_angle_resolution = 0.16 * PI / 180;
	float vertical_angle_resolution = 1.33 * PI / 180;

	int current_cluster = 0;

	int for_num = 0;
	float theta_thresh = 0.3 * PI / 180;
	float theta_thresh2 = 20 * PI / 180;
	int count = 0;

	for(int i =0 ; i< (total_frame * 32); ++i){

		if(!unordered_map_out.count(i))
			continue;

		count++;		

		int col = unordered_map_out[i].col;
		int row = unordered_map_out[i].row;
		if (cluster_indices[unordered_map_out[i].index] != -1)
			continue;


		float dorigin = unordered_map_out[i].range;

		for(int k = 0; k<4; ++k){
			int neighbor = (row + direction[k][0]) * total_frame + (col + direction[k][1]);
			if(!unordered_map_out.count(neighbor))
				continue;

			float dnei = unordered_map_out[neighbor].range;

                        float dmax = (dorigin) * sin(360/(float)total_frame*PI/180)/sin(20*PI/180 -360/(float)total_frame*PI/180) + 3*0.15;

			if(fabs(dorigin - dnei) < dmax) {
				int oc = cluster_indices[unordered_map_out[i].index]; // original point's cluster
				int nc = cluster_indices[unordered_map_out[neighbor].index]; // neighbor point's cluster
				if (oc != -1 && nc != -1) {
					if (oc != nc)
						mergeClusters(cluster_indices, oc, nc);
				}
				else {
					if (nc != -1) {
						cluster_indices[unordered_map_out[i].index] = nc;
					}
					else if (oc != -1) {
						cluster_indices[unordered_map_out[neighbor].index] = oc;
					}
				}	
			}
		}


		for(int k = 4; k<6; ++k){
			int neighbor = (row + direction[k][0]) * total_frame + (col + direction[k][1]);
			if(!unordered_map_out.count(neighbor))
				continue;
			float dnei = unordered_map_out[neighbor].range;

                        float dmax = (dorigin) * sin(1.33*PI/180)/sin(40*PI/180 -1.33*PI/180) + 0.5;
			if(fabs(dnei-dorigin)<dmax  ) {
				int oc = cluster_indices[unordered_map_out[i].index]; // original point's cluster
				int nc = cluster_indices[unordered_map_out[neighbor].index]; // neighbor point's cluster
				if (oc != -1 && nc != -1) {
					if (oc != nc)
						mergeClusters(cluster_indices, oc, nc);
				}
				else {
					if (nc != -1) {
						cluster_indices[unordered_map_out[i].index] = nc;
					}
					else {
						if (oc != -1) {
							cluster_indices[unordered_map_out[neighbor].index] = oc;
						}
					}
				}	
			}	
		}			


		if (cluster_indices[unordered_map_out[i].index] == -1) {
			current_cluster++;
			cluster_indices[unordered_map_out[i].index] = current_cluster;
			for(int k = 0; k<4; ++k){
				int neighbor = (row + direction[k][0]) * total_frame + (col + direction[k][1]);
				if(!unordered_map_out.count(neighbor))
					continue;

				float dnei = unordered_map_out[neighbor].range;
                        	float dmax = (unordered_map_out[i].range) * sin((360/(float)total_frame)*PI/180)/sin(20*PI/180 -(360/(float)total_frame)*PI/180) + 3*0.1;

				if(fabs(dnei -dorigin)< dmax) {

					cluster_indices[unordered_map_out[neighbor].index] = current_cluster;

				}
			}

			for(int k = 4; k<6; ++k){
				int neighbor = (row + direction[k][0]) * total_frame + (col + direction[k][1]);
				if(!unordered_map_out.count(neighbor))
					continue;
				float dnei = unordered_map_out[neighbor].range;

                        	float dmax = (dorigin) * sin(1.33*PI/180)/sin(40*PI/180 -1.33*PI/180) + 0.5;;

				if(fabs(dnei-dorigin)<dmax ) {
					cluster_indices[unordered_map_out[neighbor].index] = current_cluster;
				}

			}
		}
	}
	return cluster_indices;
}


bool compare_cluster(std::pair<int, int> a, std::pair<int, int> b) {
	return a.second < b.second;
} //升序

bool most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index) {
	std::unordered_map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		} else {
			histcounts[values[i]] += 1;
		}
	}

	int max = 0, maxi;
	std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
	sort(tr.begin(), tr.end(), compare_cluster);
	for (int i = 0; i < tr.size(); ++i) {
		if (tr[i].second > 10) {
			cluster_index.push_back(tr[i].first);
		}
	}

	return true;
}


std::mutex regionmutex;

void PointviewrThread(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vt){
	std::lock_guard < std::mutex > lock(regionmutex);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("cloud view")); //PCLVisualizer 可视化类
	viewer->setBackgroundColor(1, 1, 1);
	viewer->addCoordinateSystem(1);

	for(int i=0; i<vt.size(); ++i){
		if (vt[i]->points.size() > 5) {
			vt[i]->height = 1;
			vt[i]->width = vt[i]->points.size();
			float xmax1 = vt[i]->points[0].x;
			float xmin1 = vt[i]->points[0].x;
			float ymax1 = vt[i]->points[0].y;
			float ymin1 = vt[i]->points[0].y;
			float zmax1 = vt[i]->points[0].z;
			float zmin1 = vt[i]->points[0].z;
			//find the xmax, xmin, ymax, ymin
			for (int k = 0; k < vt[i]->points.size(); ++k) {
				if (vt[i]->points[k].z > zmax1) {
					zmax1 = vt[i]->points[k].z;
				}
				if (vt[i]->points[k].z < zmin1) {
					zmin1 = vt[i]->points[k].z;
				}
				if (vt[i]->points[k].x > xmax1) {
					xmax1 = vt[i]->points[k].x;
				}
				if (vt[i]->points[k].x < xmin1) {
					xmin1 = vt[i]->points[k].x;
				}
				if (vt[i]->points[k].y > ymax1) {
					ymax1 = vt[i]->points[k].y;
				}
				if (vt[i]->points[k].y < ymin1) {
					ymin1 = vt[i]->points[k].y;
				}

			}

			if (zmin1 < 0)
				zmin1 = 0; //make sure object is up the ground

			double depth = zmax1 - zmin1;

			Eigen::Vector3f eulerAngle(0.0, 0.0, 0.0);
			Eigen::AngleAxisf rollAngle(
					Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
			Eigen::AngleAxisf pitchAngle(
					Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
			Eigen::AngleAxisf yawAngle(
					Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));

			const Eigen::Quaternionf bboxQ1(yawAngle * pitchAngle * rollAngle);

			Eigen::Vector3f translation;
			translation(0) = (xmax1 - xmin1) / 2 + xmin1;
			translation(1) = (ymax1 - ymin1) / 2 + ymin1;
			translation(2) = (zmax1 - zmin1) / 2 + zmin1;
			double length = ymax1 - ymin1;
			double width = xmax1 - xmin1;

			viewer->addCube(translation, bboxQ1, width, length, depth,
					"cube" + toString(i));
			viewer->setShapeRenderingProperties(
					pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0,
					"cube" + toString(i)); //DeepSkyBlue

		}

		viewer->addPointCloud(vt[i], "cloud2" + toString(i));
	}

	while (!viewer->wasStopped()) {
		viewer->spin();
	}
}


void ImageViewThread(cv::Mat image){
	std::mutex regionmutex;
	std::lock_guard < std::mutex > lock(regionmutex);
	cv::resize(image, image, cv::Size(1000, 100));
	cv::imshow("range", image);
	cv::waitKey(0);
}



int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PointCloud<PointCloudXYZRDP>::Ptr cloud_trans(
			new pcl::PointCloud<PointCloudXYZRDP>);
		
	pcl::PointCloud<PointCloudXYZRDP>::Ptr cloud_obstacle(
			new pcl::PointCloud<PointCloudXYZRDP>);

	pcl::PointCloud<PointCloudXYZRDP>::Ptr cloud_ground(
			new pcl::PointCloud<PointCloudXYZRDP>);


	pcl::PointCloud<PointCloudXYZRDP>::Ptr cloud_break(
			new pcl::PointCloud<PointCloudXYZRDP>);

	std::unordered_map<int , Range> hash_range;

	bool view = true;

	pcl::io::loadPCDFile<pcl::PointXYZI>(argv[1], *cloud);//加载pcd点云并放入cloud中
	cloud->height = 1;
	cloud->width = cloud->points.size();

	PreProcess(*cloud, *cloud_trans);

	toRangeImage(*cloud_trans, *cloud_obstacle, *cloud_ground, hash_range);

   	int64_t tm0 = gtm();
	std::vector<int> cluster_index = RangeSeg(*cloud_obstacle, *cloud_break, hash_range);
   	int64_t tm1 = gtm();
   	printf("[INFO]segmentation cast time:%ld us\n",  tm1-tm0);

	std::vector<int> cluster_id;
	most_frequent_value(cluster_index, cluster_id);
	std::cout<<"final cluster size "<<cluster_id.size()<<std::endl;


	if(view){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudview;

		cv::RNG rng(12345);

		cv::Mat image = cv::Mat::zeros(32, total_frame, CV_8UC3);

		for (int k = 0; k < cluster_id.size(); ++k) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colorcloud2(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		
                	int r = rng.uniform(0, 200);
                	int g = rng.uniform(0, 200);
                	int b = rng.uniform(0, 200);
			for(int j=0; j<cluster_index.size(); ++j){
				if(cluster_index[j] == cluster_id[k]){
					pcl::PointXYZRGB p;
					p.x = cloud_obstacle->points[j].x;
					p.y = cloud_obstacle->points[j].y;
					p.z = cloud_obstacle->points[j].z;
					p.r = r;
					p.g = g;
					p.b = b;
					Colorcloud2->points.push_back(p);
					int col = cloud_obstacle->points[j].pcaketnum;
					int row = 31 - cloud_obstacle->points[j].ring;
					image.at<cv::Vec3b>(row,col) = cv::Vec3b(b,g,r);
				}
			}
			cloudview.push_back(Colorcloud2);
		}

        	int r = rng.uniform(0, 200);
       		int g = rng.uniform(0, 200);
       		int b = rng.uniform(0, 200);
		for(int i = 0; i < cloud_ground->points.size(); ++i){
			int col = cloud_ground->points[i].pcaketnum;
			int row = 31 - cloud_ground->points[i].ring;
			image.at<cv::Vec3b>(row,col) = cv::Vec3b(b,g,r);
		}


		std::vector<std::thread> thread_vec(2);
 		thread_vec[0] = std::thread(&PointviewrThread, std::ref(cloudview));
		thread_vec[1] = std::thread(&ImageViewThread, std::ref(image));
		for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
			it->join();
		}
	}

	return 0;
}
