/*
 * mrs_reg_node.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Rajkumar Muthusamy
 */

/////////////////////////////////////////////// INCLUDES
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <Eigen/Geometry>
#include <math.h>
#define PIOVER180 (3.14159265359/180.0) // for fromEuler()
#include <Eigen/Dense>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//filters, registration
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
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

//segmentation
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>
#include <boost/lexical_cast.hpp>
////////////////////////////////////// FUNCTIONS

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudDP;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudDRP;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef std::string CloudFile;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFH_Descriptor;
typedef pcl::PointCloud<pcl::PFHSignature125> PFH_Descriptor;
typedef pcl::PointCloud<pcl::SHOT352> SHOT_Descriptor;
typedef pcl::PointCloud<pcl::ShapeContext1980> DSC_Descriptor;
typedef pcl::search::KdTree<pcl::PointXYZ> Search;
using namespace ros;


///////////////////////////////////////////////VIEWERS


int CloudViewer (PointCloudDP::Ptr Input_Cloud, std::string window_name)
{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer(window_name));

		viewer->setBackgroundColor (0, 0, 0);
	//	viewer->setSize(1000,1000);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorpose(Input_Cloud, 255, 0, 0);
		viewer->addPointCloud(Input_Cloud, colorpose, "Input_Cloud");

		while (!viewer->wasStopped())
		{
		viewer->spinOnce ();
		}
		return 0;
}

int Multiple_CloudViewer (PointCloudDP::Ptr model_Cloud,PointCloudDP::Ptr scene_Cloud, std::string window_name)
{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer(window_name));
		viewer->setBackgroundColor (0, 0, 0);
	//	viewer->setSize(1000,1000);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorpose(model_Cloud, 255, 0, 0);
		viewer->addPointCloud(model_Cloud, colorpose, "model_Cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorposes(scene_Cloud, 155, 0, 0);
		viewer->addPointCloud(scene_Cloud, colorposes, "scene_Cloud");
		//viewer->setSize(1000,1000);

		while (!viewer->wasStopped())
		{


		viewer->spinOnce ();
		}
		return 0;
}

int Single_NormalViewer (PointCloudDP::Ptr Input_Cloud,PointCloudN::Ptr Normal, std::string window_name)
{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer(window_name));
		viewer->setBackgroundColor (0, 0, 0);
	//	viewer->setSize(1000,1000);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorpose(Input_Cloud, 255, 0, 0);
		viewer->addPointCloud(Input_Cloud, colorpose, "Input_Cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"single normal viewer");
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (Input_Cloud, Normal, 15, 0.05,"normals");

		while (!viewer->wasStopped()){
			viewer->spinOnce ();
		}
		return 0;
}

int Multiple_NormalViewer (PointCloudDP::Ptr Scene_Cloud,PointCloudN::Ptr Scene_Normal,PointCloudDP::Ptr Model_Cloud,PointCloudN::Ptr Model_Normal, std::string window_name)
{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer(window_name));
		viewer->setBackgroundColor (0, 0, 0);
	//	viewer->setSize(1000,1000);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorposes(Scene_Cloud, 215, 0, 0);
		viewer->addPointCloud(Scene_Cloud, colorposes, "scene");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorpose(Model_Cloud, 125, 0, 0);
		viewer->addPointCloud(Model_Cloud, colorpose, "model");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"scene");
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (Scene_Cloud,Scene_Normal, 10, 0.05, "scene_cloud");
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (Model_Cloud,Model_Normal, 10, 0.05, "object_cloud");

		while (!viewer->wasStopped()){
			viewer->spinOnce ();
		}
		return 0;
}

///////////////////////////////////////////////FILTERS

// Pass Through Filter to clean unwanted depths
PointCloudDP::Ptr PassThroughFilterz(PointCloudDP::Ptr Input_Cloud, CloudFile PassThroughFilterSavez)
{
		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::PassThrough<pcl::PointXYZ> limit;
		limit.setInputCloud (Input_Cloud);
		limit.setFilterFieldName ("z");
		limit.setFilterLimits (0.0, 1.7);
		limit.setKeepOrganized (true);
		limit.filter (*Output_Cloud);
		pcl::io::savePCDFileASCII (PassThroughFilterSavez, *Output_Cloud);
		return(Output_Cloud);
}

PointCloudDP::Ptr PassThroughFiltery(PointCloudDP::Ptr Input_Cloud, CloudFile PassThroughFilterSavey)
{
		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::PassThrough<pcl::PointXYZ> limit;
		limit.setInputCloud (Input_Cloud);
		limit.setFilterFieldName ("y");
		limit.setFilterLimits (-10, 10);
		limit.setKeepOrganized (true);
		limit.filter (*Output_Cloud);
		pcl::io::savePCDFileASCII (PassThroughFilterSavey, *Output_Cloud);
		return(Output_Cloud);
}

PointCloudDP::Ptr PassThroughFilterx(PointCloudDP::Ptr Input_Cloud, CloudFile PassThroughFilterSavex)
{
		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::PassThrough<pcl::PointXYZ> limit;
		limit.setInputCloud (Input_Cloud);
		limit.setFilterFieldName ("x");
		limit.setFilterLimits (-0.5, 0.5);
		limit.setKeepOrganized (true);
		limit.filter (*Output_Cloud);
		pcl::io::savePCDFileASCII (PassThroughFilterSavex, *Output_Cloud);
		return(Output_Cloud);
}

PointCloudDP::Ptr VoxelFilter(PointCloudDP::Ptr Input_Cloud, CloudFile VoxelFilterSave)
{
		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::VoxelGrid<pcl::PointXYZ> Downsample;
		Downsample.setInputCloud (Input_Cloud);
		Downsample.setLeafSize (0.003f, 0.03f, 0.003f); //previous (0.001f, 0.001f, 0.001f)
		Downsample.filter(*Output_Cloud);
		pcl::io::savePCDFile (VoxelFilterSave, *Output_Cloud);
		return(Output_Cloud);
}

PointCloudDP::Ptr StisticalFilter (PointCloudDP::Ptr Input_Cloud,CloudFile Statistical_FilterSave)
{
		PointCloudDP::Ptr Output_Clouds (new PointCloudDP);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat(true);
		stat.setInputCloud (Input_Cloud);
		stat.setMeanK(50);				//choose a suitable parameter manually//50-30
		stat.setStddevMulThresh(0.3);	//choose a suitable parameter manually//0.5-0.8
		stat.filter(*Output_Clouds);
		pcl::io::savePCDFile (Statistical_FilterSave, *Output_Clouds);
		return(Output_Clouds);
}

PointCloudN::Ptr Normal_Gen (PointCloudDP::Ptr Input_Cloud,CloudFile NormalSave)
{
		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals;
		normals.setInputCloud( Input_Cloud );
		normals.setRadiusSearch(0.1);
		PointCloudN::Ptr Input_Normals (new PointCloudN);
		normals.compute( *Input_Normals );
		pcl::copyPointCloud(*Input_Normals,*Output_Cloud);
		pcl::io::savePCDFile (NormalSave, *Output_Cloud);
		return(Input_Normals);
}

PointCloudN::Ptr Model_Normal_Gen (PointCloudDP::Ptr Input_Cloud,CloudFile NormalSave)
{
		//Compute centroid
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*Input_Cloud, centroid);
		float vx,vy,vz;
		vx = centroid[0];
		vy = centroid[1];
		vz = centroid[2];

		PointCloudDP::Ptr Output_Cloud (new PointCloudDP);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals;
		normals.setInputCloud( Input_Cloud );
		normals.setViewPoint(vx,vy,vz);
		normals.setRadiusSearch(0.1);
		PointCloudN::Ptr Input_Normals (new PointCloudN);
		normals.compute( *Input_Normals );

		PointCloudN::iterator it;
		int count = 0;
		for(it = Input_Normals->begin(); it < Input_Normals->end(); it++){
			Eigen::Vector4f n = Input_Normals->points[count].getNormalVector4fMap();
			pcl::Normal normal(-n[0],-n[1],-n[2]);
			Input_Normals->points[count] = normal;
			count++;
		}

		pcl::copyPointCloud(*Input_Normals,*Output_Cloud);
		pcl::io::savePCDFile (NormalSave, *Output_Cloud);
		return(Input_Normals);
}

FPFH_Descriptor::Ptr FPFH_Feature (PointCloudDP::Ptr Input_Cloud, PointCloudN::Ptr Input_Normal, CloudFile FeatureSave)
{
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr Input_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> descriptors;
        descriptors.setRadiusSearch (0.01);		//previous (0.09)
        Search::Ptr tree (new Search);
         descriptors.setSearchMethod (tree);
        descriptors.setInputCloud (Input_Cloud);
        descriptors.setInputNormals (Input_Normal);
		descriptors.compute (*Input_features);
        pcl::io::savePCDFileASCII (FeatureSave, *Input_features);
        return(Input_features);
}

Eigen::Matrix4f Initial_Alignment(PointCloudDP::Ptr model,FPFH_Descriptor::Ptr model_features,PointCloudDP::Ptr scene,FPFH_Descriptor::Ptr scene_features, PointCloudDP::Ptr aligned_model)
{
	//1. Setup parameters
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> Initial;
		Initial.setMinSampleDistance (0.01f); //previous 0.01
		Initial.setMaxCorrespondenceDistance (0.005f*0.005f); //previous 0.1 0.4
		Initial.setMaximumIterations (500); //160
		//Clouds and descriptors
		Initial.setInputSource(model);
		Initial.setSourceFeatures (model_features);
		Initial.setInputTarget (scene);
		Initial.setTargetFeatures (scene_features);
		PointCloudN::Ptr norm_model     (new  PointCloudN);

	//2. Align
		double score;
		int trials = 0;
		do{
			Initial.align (*aligned_model);
			score = Initial.getFitnessScore();
			ROS_INFO_STREAM("		SCORE: " << score);
			trials++;
			if(trials == 10)
				break;
		}while(score > 0.0007);

	//3. Get transformation
		Eigen::Matrix4f initial_Transform = Initial.getFinalTransformation();
		Eigen::Matrix3f rotation = initial_Transform.block<3, 3>(0, 0);
		Eigen::Vector3f translation = initial_Transform.block<3, 1>(0, 3);
		//Save it in a text file
		std::ofstream transs("/home/ir/catkin_ws/build/registration_files/object2cam_initial.txt");
		transs<<initial_Transform<<std::endl;

	//4. Save transformed model
		if(aligned_model->points.size())
		pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/Initial_aligned_model.pcd", *aligned_model);


		return initial_Transform;
}

Eigen::Matrix4f Final_Alignment (PointCloudDP::Ptr aligned_model, PointCloudDP::Ptr scene, Eigen::Matrix4f initial_Transform, PointCloudDP::Ptr icpfinalpoints)
{

	//1. Setup ICP
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //Final ICP
		icp.setInputSource(aligned_model);
		icp.setInputTarget(scene);
		icp.setMaxCorrespondenceDistance(0.001);//0.2 works //previous 0.1
		icp.setMaximumIterations(300); //180
		icp.setEuclideanFitnessEpsilon(0);			//The same as RelativeMSE: has not been helping, so it was set to 0 (inactive)

//		icp.setRANSACOutlierRejectionThreshold(0.001);//0.1 works

	//2. Align
		//pcl::PointCloud<pcl::PointXYZ> icpFinal;
		icp.align(*icpfinalpoints);

	//3. Save new aligned model
//		if(icpFinal.points.size())
//		pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/pose_object_align_icp.pcd", icpFinal);
		if(icpfinalpoints->points.size())
			pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/pose_object_align_icp.pcd", *icpfinalpoints);

	//4. Concatenate scene and save
//		icpFinal += *scene;
		*icpfinalpoints += *scene;
//		if(icpFinal.points.size())
//		pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/final_pose_object_align.pcd", icpFinal);
		if(icpfinalpoints->points.size())
		pcl::io::loadPCDFile ("/home/ir/catkin_ws/build/registration_files/final_pose_object_align.pcd", *icpfinalpoints);


		//CloudViewer(*icpfinalpoints, "ICP Alignmnet prior");

//	//	/enum state =
		//		;
	    /*static std::map<Errors, std::string> strings;

	    static std::map<Errors, std::string> strings;
	    pcl::registration::DefaultConvergenceCriteria
		*/
		//std::cout << icp.getConvergeCriteria()->getConvergenceState();

	//	icp.getConvergeCriteria()->convergence_state_;

		std::string state;
		switch(icp.getConvergeCriteria()->getConvergenceState()){
			case 0: state= "CONVERGENCE_CRITERIA_NOT_CONVERGED"; 		break;
			case 1: state= "CONVERGENCE_CRITERIA_ITERATIONS";    		break;
			case 2: state= "CONVERGENCE_CRITERIA_TRANSFORM";	 		break;
			case 3: state= "CONVERGENCE_CRITERIA_ABS_MSE";		 		break;
			case 4: state= "CONVERGENCE_CRITERIA_REL_MSE";		 		break;
			case 5: state= "CONVERGENCE_CRITERIA_NO_CORRESPONDENCES";	break;
		};

		if(icp.hasConverged()){
			ROS_INFO_STREAM("		ICP converged the criteria: "<< state);
			ROS_INFO_STREAM(" 		Convergence Fitnesse Score: "<< icp.getFitnessScore());
		}

		Eigen::Matrix4f tranm = icp.getFinalTransformation();
		return tranm;
}

//SEGMENTATION

int segmentation_euclidean(PointCloudDP::Ptr cloud)
{
		// kd-tree object for searches.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(cloud);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering; 	// Euclidean clustering object.
		clustering.setClusterTolerance(0.01); 						// Cluster tolerance to 1cm (original was 2cm); smaller values => more clusters
		clustering.setMinClusterSize(100);							// Minimum points a cluster can have
		clustering.setMaxClusterSize(25000);						// Maximum points a cluster can have
		clustering.setSearchMethod(kdtree);
		clustering.setInputCloud(cloud);
		std::vector<pcl::PointIndices> clusters;
		clustering.extract(clusters);

		// Loop over clusters
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
				// Load its points to a new cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
					cluster->points.push_back(cloud->points[*point]);
				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;

				// Save cloud to disk
				if (cluster->points.size() <= 0)
					break;
				ROS_INFO_STREAM("		Cluster " << currentClusterNum << " has " << cluster->points.size() << " points.");
				mkdir("/home/ir/catkin_ws/build/registration_files/euclidean", S_IRWXU | S_IRWXG | S_IRWXO);
				std::string fileName = "/home/ir/catkin_ws/build/registration_files/euclidean/cluster" + boost::to_string(currentClusterNum) + ".pcd";
				if(cluster->points.size())
				pcl::io::savePCDFileASCII(fileName, *cluster);
				currentClusterNum++;
		}
		return clusters.size();
}

int segmentation_region_grow(PointCloudDP::Ptr cloud, PointCloudN::Ptr normals)
{
		// kd-tree object for searches.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(cloud);

		// Region growing clustering object.
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clustering;
		clustering.setMinClusterSize(100);
		clustering.setMaxClusterSize(10000);
		clustering.setSearchMethod(kdtree);
		clustering.setNumberOfNeighbours(30);
		clustering.setInputCloud(cloud);
		clustering.setInputNormals(normals);
		// Set the angle in radians for the smoothness threshold (the maximum allowable deviation of the normals).
		clustering.setSmoothnessThreshold(15.0 / 180.0 * M_PI); // 7 degrees.
		// Set the curvature threshold. The disparity between curvatures will be tested after the normal deviation check has passed.
		clustering.setCurvatureThreshold(1);
		std::vector <pcl::PointIndices> clusters;
		clustering.extract(clusters);

		// Loop over clusters
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
				// Load its points to a new cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
					cluster->points.push_back(cloud->points[*point]);
				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;
				// Save cloud to disk
				if (cluster->points.size() <= 0)
					break;
				ROS_INFO_STREAM("		Cluster " << currentClusterNum << " has " << cluster->points.size() << " points.");
				mkdir("/home/ir/catkin_ws/build/registration_files/regiongrow", S_IRWXU | S_IRWXG | S_IRWXO);
				std::string fileName = "/home/ir/catkin_ws/build/registration_files/regiongrow/cluster" + boost::to_string(currentClusterNum) + ".pcd";
				if(cluster->points.size())
				pcl::io::savePCDFileASCII(fileName, *cluster);
				currentClusterNum++;
		}
		return clusters.size();
}

std::string ChooseBestCluster(){
	/*std::vector <pcl::PointIndices> clusters;
	std::string bestPCDFile;
	PointCloudDP::Ptr bestCluster(new PointCloudDP);
	int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			// Load its points to a new cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			// Save cloud to disk
			if (cluster->points.size() <= 0)
				break;
			std::string fileName = "/home/ir/catkin_ws/build/registration_files/regiongrow/cluster" + boost::to_string(currentClusterNum) + ".pcd";
			pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cluster);
			currentClusterNum++;
		}
		return bestCluster;*/
}

bool PlaneSAC_segmentation(PointCloudDP::Ptr cloud, PointCloudDP::Ptr output_cloud)
{
		//Inlier points
		pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

		*output_cloud = *cloud;															//Output cloud
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);			// Object for storing the plane model coeff	icients.
		pcl::SACSegmentation<pcl::PointXYZ> segmentation;								// Create the segmentation object.
		segmentation.setInputCloud(cloud);												// Configure the object to look for a plane.
		segmentation.setModelType(pcl::SACMODEL_PLANE);									// Use RANSAC method.
		segmentation.setMethodType(pcl::SAC_RANSAC);									// Set the maximum allowed distance to the model.
		segmentation.setDistanceThreshold(0.01);										// Enable model coefficient refinement (optional).
		segmentation.setOptimizeCoefficients(true);

		pcl::PointIndices inlierIndices;
		segmentation.segment(inlierIndices, *coefficients);

		if (inlierIndices.indices.size() == 0){
			ROS_INFO("		Could not find any points that fitted the plane model.");
		    return false;
		}
		else
		{
//Uncommnet this to see model coefficients
//			std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//					  << coefficients->values[1] << " "
//					  << coefficients->values[2] << " "
//					  << coefficients->values[3];
//			std::cout << std::endl;
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);	// Copy all inliers of the model to another cloud.
		}

		boost::shared_ptr<pcl::PointIndices> indices (new pcl::PointIndices);			// Delete inliers from scene
		*indices = inlierIndices;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(output_cloud);
		extract.setIndices(indices);
		extract.setNegative(true);
		extract.filter(*output_cloud);
		return true;
}

int main (int argc, char** argv)
{

// 1. Initialize ROS
		ros::init (argc, argv, "mrs_registration_node");
		ros::NodeHandle nh;

// 2. Pointers Initialized
		PointCloudDP::Ptr scene          (new PointCloudDP);
		PointCloudDP::Ptr model_no_trans (new PointCloudDP);
		PointCloudDP::Ptr model          (new PointCloudDP);
		PointCloudDP::Ptr filter_scene   (new PointCloudDP);
		PointCloudDP::Ptr filter_model   (new PointCloudDP);
		PointCloudN::Ptr norm_scene     (new  PointCloudN);
		PointCloudN::Ptr norm_model     (new  PointCloudN);

		FPFH_Descriptor::Ptr scene_features (new FPFH_Descriptor);
		FPFH_Descriptor::Ptr model_features (new FPFH_Descriptor);

		//PointCloudDP::Ptr new_aligned_model (new  PointCloudDP);
		//PointCloudDP::Ptr aligned_model (new  PointCloudDP);
		//PointCloudDP::Ptr Final_aligned_model (new  PointCloudDP);
		PointCloudDP::Ptr f_aligned_model (new  PointCloudDP);
		Eigen::Matrix4f Initial_Transform ;
		Eigen::Matrix4f Final_Transform ;

		ROS_INFO(" 1. Pointers Initialized");

// 3. Loading Scene and object point clouds
		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/scene.pcd",*scene);
		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/model.pcd",*model);
		ROS_INFO(" 2. Pointclouds loaded");
		//Create folder for files
		mkdir("registration_files", S_IRWXU | S_IRWXG | S_IRWXO);
		Multiple_CloudViewer (model,scene,"Model and scene");

// 4. Passthrough filtering
		scene = PassThroughFilterz(scene,"/home/ir/catkin_ws/build/registration_files/P_scene1.pcd");      //Pass Through Filter
		ROS_INFO(" 3. Scene passthrough-filtered Z");
		Multiple_CloudViewer (model,scene,"Scene passthrough-filtered w.r.t Z-axis");

		scene = PassThroughFiltery(scene,"/home/ir/catkin_ws/build/registration_files/P_scene1.pcd");      //Pass Through Filter
		ROS_INFO(" 4. Scene passthrough-filtered Y");
		Multiple_CloudViewer (model,scene,"Scene passthrough-filtered w.r.t Y-axis");

		scene = PassThroughFilterx(scene,"/home/ir/catkin_ws/build/registration_files/P_scene1.pcd");      //Pass Through Filter.
		ROS_INFO(" 5. Scene passthrough-filtered X");
		Multiple_CloudViewer (model,scene,"Scene passthrough-filtered w.r.t X-axis");

// 5. Downsampling scene and model
		int s = scene->points.size();
		scene = VoxelFilter(scene,"/home/ir/catkin_ws/build/registration_files/V_scene1.pcd");            //Voxel Filter
		ROS_INFO_STREAM(" 6. Scene downsampled from " << s << " to " << scene->points.size() << " points");
		s = model->points.size();
		model = VoxelFilter(model,"/home/ir/catkin_ws/build/registration_files/V_model2.pcd");           //Voxel Filter
		ROS_INFO_STREAM(" 7. Model downsampled from " << s << " to " << model->points.size() << " points");
		Multiple_CloudViewer (model,scene,"Downsampling of scene and model");

// 6. Segmenting and removing planes using RANSAC
		if(PlaneSAC_segmentation(scene, scene))
			ROS_INFO(" 8. Segmented plane removed from scene");
		else
			ROS_INFO(" 8. No plane was found for segmentation");
		Multiple_CloudViewer (model,scene,"Removing planes from scene");

// 7. Outlier removal: statistical filtering
		scene = StisticalFilter(scene,"/home/ir/catkin_ws/build/registration_files/S_scene1.pcd");       //Statistical filter
		ROS_INFO(" 9. Statistical filter applied to scene");

// 8. Computing Normals
		norm_scene= Normal_Gen(scene,"/home/ir/catkin_ws/build/registration_files/Norm_scene1.pcd");
		norm_model= Model_Normal_Gen(model,"/home/ir/catkin_ws/build/registration_files/Norm_model2.pcd");
		ROS_INFO("10. Scene and model normals computed");

		//Visualizing scene and model with normalsoperand
		ROS_INFO("11. Visualizing normals <close viewer to proceed>");
		Multiple_NormalViewer(scene,norm_scene,model,norm_model, "Visualizing scene and model with normals");

// 9. Euclidean Segmentation
		ROS_INFO("12. Euclidean segmentation results: (check 'euclidean' folder)");
		segmentation_euclidean(scene);
//10. Region-growing Segmentation
		ROS_INFO("13. Region growing results: (check 'regiongrow' folder)");
		segmentation_region_grow(scene, norm_scene);

		/*ADD CODE TO PICK BEST CLUSTER*/

		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/scene_cluster.pcd",*scene);
		norm_scene= Normal_Gen(scene,"/home/ir/catkin_ws/build/registration_files/Norm_scene1.pcd");
		Multiple_NormalViewer(scene,norm_scene,model,norm_model, "Visualizing selected cluster and model with normals");

//11. Feature Extraction
		scene_features= FPFH_Feature(scene,norm_scene,"/home/ir/catkin_ws/build/registration_files/Scene_feature1_FPFH.pcd") ;
		model_features= FPFH_Feature(model,norm_model,"/home/ir/catkin_ws/build/registration_files/model_feature2_FPFH.pcd") ;
		ROS_INFO("14. Scene and model FPFH features extracted");

//12. Initial Alignment
		ROS_INFO("15. Performing initial alignment...");
		PointCloudDP::Ptr aligned_model(new PointCloudDP);
		Eigen::Matrix4f initial_Transform = Initial_Alignment(model, model_features, scene, scene_features, aligned_model);

		ROS_INFO_STREAM("Frame: " << aligned_model->header.frame_id);

		//Printing transformation
		ROS_INFO("	   Initial alignment transformation (from object to camera frame):	");
		ROS_INFO_STREAM("		    | " << std::setprecision(4) << std::fixed << std::right << initial_Transform.row(0) << " |				");
		ROS_INFO_STREAM("		T = | " << std::setprecision(4) << std::fixed << std::right << initial_Transform.row(1) << " |				");
		ROS_INFO_STREAM("		    | " << std::setprecision(4) << std::fixed << std::right << initial_Transform.row(2) << " |				");
		ROS_INFO_STREAM("		    | " << "      0       0       0       1 |				");

		//Visualizing initial alignment
		ROS_INFO("16. Visualizing initial alignment <close viewer to proceed>");
		Multiple_CloudViewer(aligned_model, scene, "Visualizing initial alignment");

//13. Getting transformation in camera frame
		ROS_INFO("17. Transformation from camera to object frame computed");

		Eigen::Matrix4f Tm=initial_Transform.inverse();  //camera to object
		tf::Vector3 origin;
		tf::Matrix3x3 tf3d;
		tf::Quaternion tfqt;
		tf::Quaternion itfqt;

		//Define origin of camera from the translation column
		origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

		//Extract rotation
		tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
					  static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
					  static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

		//Convert rotation to quaternions
		tf3d.getRotation(tfqt);
		tf3d.transpose().getRotation(itfqt);
		ROS_INFO_STREAM("		t = " << origin.x() << ", " << origin.y() << ", " << origin.z() << "						");
		ROS_INFO("		Quaternion values:								");
		ROS_INFO_STREAM("		Camera(parent) to robot = < " << std::right << tfqt.x() << ", " << tfqt.y() << ", " << tfqt.z() << ", " << tfqt.w() << " >		");
		ROS_INFO_STREAM("		Robot(parent) to camera = < " << std::right << itfqt.x() << ", " << itfqt.y() << ", " << itfqt.z() << ", " << itfqt.w() << " >		");



//14. Improving alignment
		ROS_INFO("18. Performing ICP for smooth alignment...");
		PointCloudDP::Ptr icp_aligned_model(new PointCloudDP);
		Eigen::Matrix4f tranm = Final_Alignment (aligned_model, scene, initial_Transform, icp_aligned_model);

		//Printing transformation
		ROS_INFO("	   Initial alignment transformation (from object to camera frame):	");
		ROS_INFO_STREAM("		    | " << std::setprecision(4) << std::fixed << std::right << tranm.row(0) << " |				");
		ROS_INFO_STREAM("		T = | " << std::setprecision(4) << std::fixed << std::right << tranm.row(1) << " |				");
		ROS_INFO_STREAM("		    | " << std::setprecision(4) << std::fixed << std::right << tranm.row(2) << " |				");
		ROS_INFO_STREAM("		    | " << "      0       0       0       1 |				");

		//Visualizing
		ROS_INFO("19. Visualizing ICP alignment <close viewer to proceed>");
	//	CloudViewer(icp_aligned_model, "ICP Alignment");

//15. Getting transformation in camera frame


		pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/icp_aligned_model.pcd", *icp_aligned_model);
		PointCloudDP icp_aligned_model_scene = *icp_aligned_model;
		icp_aligned_model_scene += *scene;
		pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/icp_final_aligned_model_scene.pcd", icp_aligned_model_scene);
		PointCloudDP::Ptr finalscene (new PointCloudDP);
		ROS_INFO("======================================================================");
		ROS_INFO(" please check the icp alignment");
		ROS_INFO("======================================================================");
		pcl::io::loadPCDFile ("/home/ir/catkin_ws/build/registration_files/icp_final_aligned_model_scene.pcd", *finalscene);

		ROS_INFO("======================================================================");
		ROS_INFO(" 3. CLOUD VIEWER TRANSFORMATION ICP ");
		ROS_INFO("======================================================================");
//		CloudViewer(finalscene,"lk");

		ROS_INFO(" object to icp Frame");
		ROS_INFO("======================================================================");

		std::cout << "object to icp Transformation matrix:"<< std::endl << tranm << std::endl;
		std::ofstream trans("/home/ir/catkin_ws/build/registration_files/object2icp_icp.txt");
		trans<<tranm<<std::endl;
		Eigen::Matrix3f icprotation = tranm.block<3, 3>(0, 0);
		Eigen::Vector3f icptranslation = tranm.block<3, 1>(0, 3);


//13. Getting transformation in camera frame
		ROS_INFO("20. Transformation from camera to object frame computed");

		Eigen::Matrix4f eTm=tranm.inverse();  //camera to object
		tf::Vector3 eorigin;
		tf::Matrix3x3 etf3d;
		tf::Quaternion etfqt;
		tf::Quaternion eitfqt;

		//Define origin of camera from the translation column
		eorigin.setValue(static_cast<double>(eTm(0,3)),static_cast<double>(eTm(1,3)),static_cast<double>(eTm(2,3)));

		//Extract rotation
		etf3d.setValue(static_cast<double>(eTm(0,0)), static_cast<double>(eTm(0,1)), static_cast<double>(eTm(0,2)),
					  static_cast<double>(eTm(1,0)), static_cast<double>(eTm(1,1)), static_cast<double>(eTm(1,2)),
					  static_cast<double>(eTm(2,0)), static_cast<double>(eTm(2,1)), static_cast<double>(eTm(2,2)));

		//Convert rotation to quaternions
		etf3d.getRotation(etfqt);
		etf3d.transpose().getRotation(eitfqt);
		ROS_INFO_STREAM("		t = " << eorigin.x() << ", " << eorigin.y() << ", " << eorigin.z() << "						");
		ROS_INFO("		Quaternion values:								");
		ROS_INFO_STREAM("		Camera(parent) to robot = < " << std::right << etfqt.x() << ", " << etfqt.y() << ", " << etfqt.z() << ", " << etfqt.w() << " >		");
		ROS_INFO_STREAM("		Robot(parent) to camera = < " << std::right << eitfqt.x() << ", " << eitfqt.y() << ", " << eitfqt.z() << ", " << eitfqt.w() << " >		");



		ROS_INFO(" Final Transformation ");

		ROS_INFO(" camera to Object_icp Frame Final Transformation");
		ROS_INFO("======================================================================");
			//Total transformation in camera frame

		    Eigen::Matrix4f t_final = tranm*initial_Transform;



		    PointCloudDP::Ptr finals_aligned_model(new PointCloudDP);
		    transformPointCloud(*model,*finals_aligned_model,t_final);
		    pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/icp_aligned_model.pcd", *finals_aligned_model);
		    PointCloudDP complete_aligned_model_scene = *finals_aligned_model;

			ROS_INFO_STREAM("Frame: " << finals_aligned_model->header.frame_id);


		    complete_aligned_model_scene += *scene;
		    pcl::io::savePCDFile ("/home/ir/catkin_ws/build/registration_files/complete_final_aligned_model_scene.pcd", complete_aligned_model_scene);

		    PointCloudDP::Ptr complete_scene (new PointCloudDP);
		    ROS_INFO("======================================================================");
		    ROS_INFO(" 4. CLOUD VIEWER  please check the complete transformation works");
		    ROS_INFO("======================================================================");
		    pcl::io::loadPCDFile ("/home/ir/catkin_ws/build/registration_files/complete_final_aligned_model_scene.pcd", *complete_scene);
		    CloudViewer(complete_scene,"Visualizing final alignment");

		 std::cout << "camera to object_icp Final Transformation matrix:"<< std::endl << t_final << std::endl;
		 std::ofstream transf("/home/ir/catkin_ws/build/registration_files/cam2obj_final_Transformation.txt");
		 transf<<t_final<<std::endl;
//
//		 ROS_INFO("======================================================================");
//		 ROS_INFO(" 4. CLOUD VIEWER  FINAL ALIGNED");
//		 ROS_INFO("======================================================================");
//		 Multiple_CloudViewer(icp_aligned_model,scene,"ff");
//
//
//		  Eigen::Matrix3f finalrotation = t_final.block<3, 3>(0, 0);
//		  Eigen::Vector3f finaltranslation = t_final.block<3, 1>(0, 3);
//
//		                            		                		std::cout << " icp(parent) to camera frame Transformation matrix:" << std::endl << std::endl;
//		                            		                		printf("\t\t    | %6.3f %6.3f %6.3f | \n", finalrotation(0, 0), finalrotation(0, 1), finalrotation(0, 2));
//		                            		                		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", finalrotation(1, 0), finalrotation(1, 1), finalrotation(1, 2));
//		                            		                		printf("\t\t    | %6.3f %6.3f %6.3f | \n", finalrotation(2, 0), finalrotation(2, 1), finalrotation(2, 2));
//		                            		                		std::cout << std::endl;
//		                            		                		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", finaltranslation(0), finaltranslation(1), finaltranslation(2));
//
//		                            		                		Eigen::Matrix4f Tmm=t_final;
//		                            		                		tf::Vector3 oorigin;
//		                            		                			   tf::Matrix3x3 ttf3d;
//		                            		                			   tf::Quaternion ttfqt;
//		                            		                			   tf::Quaternion titfqt;
//		                            		                		ROS_INFO("Translation:");
//		                            		                	    oorigin.setValue(static_cast<double>(Tmm(0,3)),static_cast<double>(Tmm(1,3)),static_cast<double>(Tmm(2,3)));
//
//		                            		                	    ROS_INFO("Rotation:");
//		                            		                	    ttf3d.setValue(static_cast<double>(Tmm(0,0)), static_cast<double>(Tmm(0,1)), static_cast<double>(Tmm(0,2)),
//		                            		                	    			  static_cast<double>(Tmm(1,0)), static_cast<double>(Tmm(1,1)), static_cast<double>(Tmm(1,2)),
//		                            		                					  static_cast<double>(Tmm(2,0)), static_cast<double>(Tmm(2,1)), static_cast<double>(Tmm(2,2)));
//
//
//
//		                            		                	    ttf3d.getRotation(ttfqt);
//		                            		                	    ttf3d.transpose().getRotation(titfqt);
//		                            		                	    std::cout << ttfqt;
//
//		                            		                	                            ROS_INFO("Translation:");
//		                            		                	                            printf("\t\tt = < %0.6f, %0.6f, %0.6f>\n", oorigin.x(), oorigin.y(), oorigin.z());
//		                            		                	                    		ROS_INFO("object to camera Queternion values:");
//		                            		                	                    		printf("Camera(parent) to robot = < %0.6f, %0.6f, %0.6f, %0.6f>\n", ttfqt.x(), ttfqt.y(), ttfqt.z(), ttfqt.w());
//		                            		                	                    		ROS_INFO("titfqt camera  to object Queternion values:");
//		                            		                	                    		printf("Robot(parent) to camera = < %0.6f, %0.6f, %0.6f, %0.6f>\n", titfqt.x(), titfqt.y(), titfqt.z(), titfqt.w());
//		ROS_INFO("==========================Finished============================================");

		ros::spinOnce ();
}
