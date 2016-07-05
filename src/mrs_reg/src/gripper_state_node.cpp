/*
 * gripper_state_node.cpp
 *
 *  Created on: Jun 1, 2015
 *      Author: raj
 */


#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <cstdlib>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Bool.h"

//#include <moveit/move_group_interface/move_group.h>

//boost bind to use multiple arguments
#include "boost/bind.hpp"
#include "boost/shared_ptr.hpp"
#include "boost/ref.hpp"

#include <pcl/segmentation/extract_clusters.h>

//Custom message
#include <mrs_reg/gripper_data.h>

/////////////////////////////////////////////// INCLUDES


//ROS

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//filters, registration
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/3dsc.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

//visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/time.h>

#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudDP;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudDRP;
using namespace ros;
ros::Publisher pub;

//Data to be published together
int model_number;
double threshold_open;
double threshold_close;
double open_percent;
double close_percent;
double thres_object;
bool obj_cluster_detection;
bool obj_removal_detection;
int clusters_n;

//Message
mrs_reg::gripper_data message;
bool message_available;

PointCloudDP::Ptr object_cluster;
PointCloudDP::Ptr open_model;
PointCloudDP::Ptr close_model;

PointCloudDP::Ptr gripper_cloud;
PointCloudDP::Ptr object_cloud;


void gridSampleApprox (const PointCloudDP::Ptr &cloud, PointCloudDP &result)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (0.02f, 0.02f, 0.02f);
  grid.setInputCloud (cloud);
  grid.filter (result);
}

double model_inliers_outliers(PointCloudDP::Ptr scene, PointCloudDP::Ptr model, PointCloudDP::Ptr inlier_cloud,  PointCloudDP::Ptr outlier_cloud, float radius){

	double inlier_percent;
	int count = 0;

	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*scene, *scene, mapping);

	PointCloudDP::iterator it;
	for(it = scene->begin(); it < scene->end(); it++){

		pcl::PointXYZ search_center;
		search_center.x = it->x;
		search_center.y = it->y;
		search_center.z = it->z;
		//ROS_INFO_STREAM("inout: " << model->points.size());
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(model);

		std::vector<int> pointIdxRadius;
		std::vector<float> pointsSquaredDistRadius;

		int c = kdtree.radiusSearch(search_center, radius, pointIdxRadius, pointsSquaredDistRadius);

		if(c>0){			//Add point to inlier_cloud
			inlier_cloud->points.push_back(search_center);
			count++;
		}
		else{				//Add point to outlier_cloud
			outlier_cloud->points.push_back(search_center);
		}
	}

	pcl::removeNaNFromPointCloud(*inlier_cloud, *inlier_cloud, mapping);
	pcl::removeNaNFromPointCloud(*outlier_cloud, *outlier_cloud, mapping);

	inlier_cloud->height = 1;
	inlier_cloud->width = inlier_cloud->points.size();
	outlier_cloud->height = 1;
	outlier_cloud->width = outlier_cloud->points.size();

	inlier_percent = count*1.0/scene->points.size();
	return inlier_percent;

}

double model_inliers(PointCloudDP::Ptr scene, PointCloudDP::Ptr model, float radius){

		double inlier_percent;
		int count = 0;

		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*scene, *scene, mapping);

		PointCloudDP::iterator it;
		for(it = scene->begin(); it < scene->end(); it++){

			pcl::PointXYZ search_center;
			search_center.x = it->x;
			search_center.y = it->y;
			search_center.z = it->z;
			//ROS_INFO_STREAM("in: " << model->points.size());

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(model);

			std::vector<int> pointIdxRadius;
			std::vector<float> pointsSquaredDistRadius;

			int c = kdtree.radiusSearch(search_center, radius, pointIdxRadius, pointsSquaredDistRadius);

			if(c>0)
				count++;
		}

		inlier_percent = count*1.0/scene->points.size();




		return inlier_percent;
}

//SEGMENTATION

int segmentation_euclidean(PointCloudDP::Ptr cloud)
{
		//Cloud for output
		PointCloudDP::Ptr temp_cloud(new PointCloudDP);

		*temp_cloud = *cloud;
		pcl::io::savePCDFileASCII("/home/ir/catkin_ws/build/gripper_state_files/euclidean/temp1.pcd", *temp_cloud);

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-0.01,0.1);
		pass.setKeepOrganized (true);
		pass.setInputCloud (temp_cloud);
		pass.filter (*temp_cloud);
		pcl::io::savePCDFileASCII("/home/ir/catkin_ws/build/gripper_state_files/euclidean/temp2.pcd", *temp_cloud);

		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, mapping);

		//Project points onto YZ plane
		PointCloudDP::iterator it;
		for(it = temp_cloud->begin(); it < temp_cloud->end(); it++){
				pcl::PointXYZ point;
				point.x = 0.0;
				point.y = it->y;
				point.z = it->z;
				temp_cloud->erase(it);
				temp_cloud->insert(it,point);
		}

		pcl::io::savePCDFileASCII("/home/ir/catkin_ws/build/gripper_state_files/euclidean/temp3.pcd", *temp_cloud);

		pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, mapping);
		temp_cloud->width = 1;
		temp_cloud->height = temp_cloud->points.size();

		// kd-tree object for searches.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(temp_cloud);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering; 	// Euclidean clustering object.
		clustering.setClusterTolerance(0.02); 						// Cluster tolerance to 1cm (original was 2cm); smaller values => more clusters
		clustering.setMinClusterSize(50);							// Minimum points a cluster can have
		clustering.setMaxClusterSize(25000);						// Maximum points a cluster can have
		clustering.setSearchMethod(kdtree);
		clustering.setInputCloud(temp_cloud);
		std::vector<pcl::PointIndices> clusters;
		clustering.extract(clusters);


		// Loop over clusters
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
				// Load its points to a new cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
					cluster->points.push_back(temp_cloud->points[*point]);
				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;

				if (cluster->points.size() <= 0)
									break;


/*
				double x;
				if(model_number == 1)
					x = model_inliers(cluster, open_model, 0.01);
				else
					x = model_inliers(cluster, close_model, 0.01);

				if(x > 0.7){	//Cluster belongs to the model (might be a finger)
					temp_cloud += *cluster;
					pcl::io::savePCDFileASCII("gripper_state_files/euclidean/temp_cloud.pcd", temp_cloud);
				}
				else{			//Cluster does not belong to the model (probably an object)
					if(obj_cluster_detection == false){	//Reset the cloud in first cluster found
						object_cluster.reset(new PointCloudDP);
						obj_cluster_detection = true;
						obj_removal_detection = false;
					}
					if(open_percent > 0.7)				//Object reference is stored
						*object_cluster += *cluster;

				}
*/

				pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, mapping);
				temp_cloud->width = 1;
				temp_cloud->height = temp_cloud->points.size();

				// Save cloud to disk

				//ROS_INFO_STREAM("		Cluster " << currentClusterNum << " has " << cluster->points.size() << " points.");
				mkdir("/home/ir/catkin_ws/build/gripper_state_files/euclidean", S_IRWXU | S_IRWXG | S_IRWXO);
				std::string fileName = "/home/ir/catkin_ws/build/gripper_state_files/euclidean/cluster" + boost::to_string(currentClusterNum) + ".pcd";
				pcl::io::savePCDFileASCII(fileName, *cluster);
				currentClusterNum++;
		}

		//If there is only one cluster, there is no need to change the cloud. A single cluster might be considered part of the model or not.
		//To avoid the 'not' possibility, let's just bypass segmentation if there is only one cluster.

	//	if(clusters.size() != 1 && temp_cloud.points.size() != 0)
		//	*cloud_out = temp_cloud;

		return clusters.size();
}

double grasp_box_passthrough (PointCloudDP::Ptr hand, PointCloudDP::Ptr graspbox){

		pcl::PassThrough<pcl::PointXYZ> pass_;
		pass_.setKeepOrganized (true);

		pass_.setFilterFieldName ("z");
		pass_.setFilterLimits (-0.015,0.1);
		pass_.setInputCloud (hand);
		pass_.filter (*graspbox);

		pass_.setFilterFieldName ("x");
		pass_.setFilterLimits (-0.2,0.2);
		pass_.setInputCloud (graspbox);
		pass_.filter (*graspbox);

		pass_.setFilterFieldName ("y");
		pass_.setFilterLimits (-0.04,0.04);
		pass_.setInputCloud (graspbox);
		pass_.filter (*graspbox);

		//Remove NaN
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*graspbox, *graspbox, mapping);
//		ROS_INFO_STREAM("size: " << object_cluster->points.size());
/*
		//Check if remaining points belong to the object cloud
		if(graspbox->points.size() >= 5)
			return model_inliers(graspbox, object_cld, 0.01);
*/
		return graspbox->points.size();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
		// 1.Convert input data do pointcloud

		sensor_msgs::PointCloud2 cloudmsg = *input;
		PointCloudDP::Ptr cloudpcl (new PointCloudDP);
		PointCloudDP::Ptr graspbox (new PointCloudDP);

		pcl::fromROSMsg (cloudmsg,*cloudpcl);

		// 2. If there are no points, there is no point to proceed
		if(cloudpcl->points.size()){

	/*		//Detect object removal
			if(obj_removal_detection == false){	//if this is false, it means an object was detected. Now, its removal should be checked continuosly

				double s = grasp_box_passthrough(cloudpcl, graspbox);
				if(graspbox->points.size() < 5)
					obj_removal_detection = true;
				else	//there is something there
					if(s < 0.7) //if this something is the object
						obj_removal_detection = true;
			}*/

			//Extract hand
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (-0.07,0.10);
			pass.setKeepOrganized (true);
			pass.setInputCloud (cloudpcl);
			pass.filter (*cloudpcl);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (-0.1,0.1);
			pass.setInputCloud (cloudpcl);
			pass.filter (*cloudpcl);


			//Remove NaN
			std::vector<int> mapping;
			pcl::removeNaNFromPointCloud(*cloudpcl, *cloudpcl, mapping);

			//Update parameters
			cloudpcl->header.frame_id = "current_pose";
			cloudpcl->width = 1;
			cloudpcl->height = cloudpcl->points.size();

			//SEGMENTATION: reinforce object detection, but mainly gives grasping information
			clusters_n = segmentation_euclidean(cloudpcl);	//objects nearby should now be removed. If clusters_n == 1, segmentation is bypassed

			//Now that the clusters are found (and objects removed), no more need for the front part of the wrist, only fingers are useful for state detection
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (-0.03,0.10);
			pass.setKeepOrganized (true);
			pass.setInputCloud (cloudpcl);
			pass.filter (*cloudpcl);

			//----OBJECT DETECTION + GRIPPER CLOUD EXTRACTION
			//If the hand is fully opened, objects can be found!
			if(open_percent > thres_object){
				//model_inliers_outliers(PointCloudDP::Ptr scene, PointCloudDP::Ptr model, PointCloudDP::Ptr inlier_cloud,  PointCloudDP::Ptr outlier_cloud, float radius){
				gripper_cloud.reset(new PointCloudDP);
				object_cloud.reset(new PointCloudDP);

				//PointCloudDP::Ptr cloud_down(new PointCloudDP);
			//	gridSampleApprox(cloudpcl, *cloud_down);

				double x = model_inliers_outliers(cloudpcl, open_model, gripper_cloud, object_cloud, 0.02);
				double s = grasp_box_passthrough(object_cloud, object_cloud);

	//			ROS_INFO_STREAM("gri: " << gripper_cloud->points.size());
		//		ROS_INFO_STREAM("obj: " << object_cloud->points.size());

				//	if(object_cloud->points.size() > 15)
				//	pcl::io::savePCDFileASCII("OBJ.pcd", *object_cloud);
					//if(gripper_cloud->points.size())


				//Object detection can be done
				if(object_cloud->points.size() > 5){	//There is an object in a full-opened gripper scenario
					obj_cluster_detection = true;
					//obj_removal_detection = false;
				}
				else{	//No object
					obj_cluster_detection = false;
				//	obj_removal_detection = true;
				}
				*cloudpcl = *gripper_cloud;
			}
		/*	else
				if(obj_cluster_detection == true){	//GRASPING: Check always for object removal
					double s = grasp_box_passthrough(cloudpcl, object_cloud, graspbox);
					if(graspbox->points.size() < 5){
						obj_removal_detection = true;
						obj_cluster_detection = false;
					}
					else	//there is something there
						if(s < 0.7){ //if this something is the object
							obj_removal_detection = true;
							obj_cluster_detection = false;
						}
			}*/
			//pcl::io::savePCDFileASCII("GRIDDDD.pcd", *cloudpcl);

			//Compute Model similarities
			open_percent = model_inliers(cloudpcl, open_model, 0.015);
			close_percent = model_inliers(cloudpcl, close_model, 0.02);
			//Make decision concerning the model to be tracked
			if(model_number != 1 && (open_percent > threshold_open || close_percent < 0.7))  //swap to open model
				model_number = 1;
			if(model_number != 2 && (close_percent > threshold_close))//swap to close model
				model_number = 2;

			//Update message
				message.m1_sim = open_percent;
				message.m2_sim = close_percent;
				message.model_n = model_number;
				message.thres_1 = threshold_open;
				message.thres_2 = threshold_close;
				message.thres_obj = thres_object;
				message.obj_detect = obj_cluster_detection;
				message.cluster_n = clusters_n;
			//	message.obj_remov_detec = obj_removal_detection;
				message_available = true;

		}
}

int main(int argc, char **argv)
{
		ros::init(argc,argv,"gripper_state_node");
		ros::NodeHandle n;

		//Read models
		open_model.reset(new PointCloudDP);
		close_model.reset(new PointCloudDP);
		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/gripper_state_files/open_model.pcd",*open_model);
		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/gripper_state_files/close_model.pcd",*close_model);

		gridSampleApprox (open_model,*open_model);
		gridSampleApprox (close_model,*close_model);

		//Initialize data
		model_number = 1;
		threshold_open = 0.95;
		threshold_close = 0.95;
		thres_object = 0.9;
		obj_cluster_detection = false;
		//obj_removal_detection = true;
		object_cluster.reset(new PointCloudDP);
		open_percent = 1;
		close_percent = 0;

		// Create a ROS subscriber for the input point cloud
		ros::Subscriber sub = n.subscribe ("/mrs_tracking_node/tracked_scene_cloud", 1, cloud_cb);
		ros::Publisher pub = n.advertise<mrs_reg::gripper_data>("gripper_state_node/gripper_data",1);


		  while(n.ok()){

				  	 if(message_available){
				  		 pub.publish(message);
				  		 message_available = false;
				  	 }
			  	    ros::spinOnce ();
			        }

		return 0;
}

