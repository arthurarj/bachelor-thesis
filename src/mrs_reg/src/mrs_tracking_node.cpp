#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>


#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>
#include <boost/lexical_cast.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

//TF
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//Custom message
#include <mrs_reg/gripper_data.h>

using namespace pcl::tracking;

//typedef pcl::PointXYZRGBA RefPointType;
typedef pcl::PointXYZ RefPointType;

typedef ParticleXYZRPY ParticleT;
//typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud_1;
CloudPtr target_cloud_2;

//Clouds for publishing
CloudPtr cloud_final_;
CloudPtr cloud_particles;

CloudPtr gripper_cloud;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_scene;
double downsampling_grid_size_model;
int counter;

double close_sim, open_sim;
int model_number;
bool new_hand_cloud;
int already, iteration_number;
Eigen::Affine3f last_transformation;

tf::Transform initial_pose_transform;
tf::Transform current_pose_transform;

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, CloudPtr &result)
{
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.4);
  pass.setKeepOrganized (true);
  pass.setInputCloud (cloud);
  pass.filter (*result);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.9, 0.25);
  pass.setInputCloud (result);
  pass.filter (*result);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.5, 0.5);
  pass.setInputCloud (result);
  pass.filter (*result);
}

void PlaneSAC_segmentation(Cloud::Ptr cloud, Cloud::Ptr output_cloud)
{
		//Inlier points
		pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

		//Output cloud
		*output_cloud = *cloud;

		// Object for storing the plane model coeff	icients.
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// Create the segmentation object.
		pcl::SACSegmentation<pcl::PointXYZ> segmentation;
		segmentation.setInputCloud(cloud);
		// Configure the object to look for a plane.
		segmentation.setModelType(pcl::SACMODEL_PLANE);
		// Use RANSAC method.
		segmentation.setMethodType(pcl::SAC_RANSAC);
		// Set the maximum allowed distance to the model.
		segmentation.setDistanceThreshold(0.01);
		// Enable model coefficient refinement (optional).
		segmentation.setOptimizeCoefficients(true);

		pcl::PointIndices inlierIndices;
		segmentation.segment(inlierIndices, *coefficients);

		if (inlierIndices.indices.size() == 0)
			std::cout << "Could not find any points that fitted the plane model." << std::endl;
		else
		{
		/*	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
					  << coefficients->values[1] << " "
					  << coefficients->values[2] << " "
					  << coefficients->values[3] << std::endl;*/

			// Copy all inliers of the model to another cloud.
			pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);
		}

		boost::shared_ptr<pcl::PointIndices> indices (new pcl::PointIndices);

		*indices = inlierIndices;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(output_cloud);
		extract.setIndices(indices);
		extract.setNegative(true);
		extract.filter(*output_cloud);

}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  //pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;

  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}


//Draw the current particles
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
    {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
	{
	  pcl::PointXYZ point;

	  point.x = particles->points[i].x;
	  point.y = particles->points[i].y;
	  point.z = particles->points[i].z;
	  particle_cloud->points.push_back (point);
	}

      //Draw red particles
      {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

	if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
	  viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}

//Draw model reference point cloud
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}

//visualization's callback function
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  boost::mutex::scoped_lock lock (mtx_);

  if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
//      boost::this_thread::sleep (boost::posix_time::milliseconds (500));
      return;
   }

  //Draw downsampled point cloud from sensor
  //if (new_cloud_ && cloud_pass_downsampled_)
  if (new_cloud_ && cloud_final_)
    {

      CloudPtr cloud_pass;

//      cloud_pass = cloud_pass_downsampled_;
      cloud_pass = cloud_final_;

      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
	{
	  viz.addPointCloud (cloud_pass, "cloudpass");
	  viz.resetCameraViewpoint ("cloudpass");
	}
      bool ret = drawParticles (viz);
      if (ret)
        drawResult (viz);
    }
  new_cloud_ = false;
}

void hand_passthrough(Cloud input_cloud){

    	//Reset used clouds
  	    gripper_cloud.reset (new Cloud);

  	    *gripper_cloud = input_cloud;

		Eigen::Affine3d hand_transform;
		tf::transformTFToEigen(initial_pose_transform*current_pose_transform, hand_transform);
		Eigen::Affine3f hand_transform_f = hand_transform.cast<float>();
		pcl::transformPointCloud<RefPointType>(*gripper_cloud,*gripper_cloud, hand_transform_f.inverse());

		pcl::PassThrough<pcl::PointXYZ> pass;

		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-0.15, 0.10);
		pass.setKeepOrganized (true);
		pass.setInputCloud (gripper_cloud);
		pass.filter (*gripper_cloud);

		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.15,0.15);
		pass.setInputCloud (gripper_cloud);
		pass.filter (*gripper_cloud);

		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.10,0.10);
		pass.setInputCloud (gripper_cloud);
		pass.filter (*gripper_cloud);

		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*gripper_cloud, *gripper_cloud, mapping);
		gripper_cloud->header.frame_id = "current_pose";
		gripper_cloud->width = 1;
		gripper_cloud->height = gripper_cloud->points.size();

		//if(already < iteration_number+1)
		new_hand_cloud = true;

	/*
	pcl::PCLPointCloud2 gripper_msg;
	pcl::toPCLPointCloud2(*gripper_cloud, gripper_msg);
	Eigen::Vector4f origin_ (current_pose_transform.getOrigin().getX(), current_pose_transform.getOrigin().getY(), current_pose_transform.getOrigin().getZ(), 0.0);

	Eigen::Quaterniond rot;
	tf::quaternionTFToEigen(current_pose_transform.getRotation(), rot);
	Eigen::Quaternionf rot_f = rot.cast<float>();
	pcl::io::savePCDFile ("tracking_files/hand_passthrough.pcd", gripper_msg, origin_, rot_f);*/
	if(gripper_cloud->points.size())
	pcl::io::savePCDFile ("/home/ir/catkin_ws/build/tracking_files/hand_passthrough.pcd", *gripper_cloud);


}

//ROS's cloud Callback function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){

  boost::mutex::scoped_lock lock (mtx_);

  //Reset used clouds
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);

  sensor_msgs::PointCloud2 cloudmsg = *input;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (cloudmsg,*cloud);

  //Apply filters
  filterPassThrough (cloud, cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_scene);
  PlaneSAC_segmentation(cloud_pass_downsampled_, cloud);

  if(counter < 10){
	counter++;
  }else{

	    //Reset used clouds
  	   // cloud_final_.reset (new Cloud);

  	//Track the object
		tracker_->setInputCloud (cloud);
		tracker_->compute ();
		new_cloud_ = true;

    //Prepare cloud to be published
		ParticleXYZRPY result = tracker_->getResult ();
		Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
		Eigen::Affine3d transf = transformation.cast<double>();
/*
		tf::Transform relative;
		tf::transformEigenToTF(transf,relative);	//Frame here is relative to camera_frame
		relative.mult(initial_pose_transform.inverse(),relative);
		relative.mult(relative, current_pose_transform.inverse());
		Eigen::Affine3d transf_after;
		tf::transformTFToEigen(relative,transf_after);
		Eigen::Affine3f transformation_after = transf_after.cast<float>();

		float x,y,z,roll,pitch,yaw;
		pcl::getTranslationAndEulerAngles(transformation_after,x,y,z,roll,pitch,yaw);

		std::cout << "NEW: " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
*/



/*
		Eigen::Affine3f temp;

		std::cout << "NEW: " << transformation.rotation().x() << " " << transformation.rotation().y() << " " << transformation.rotation().z() << " "
				transformation.rotation().w() << " " << endl;
*/
		/*temp = transformation last_transformation

		temp.*/

	//	last_transformation = transformation;

			   // Eigen::Matrix< simFloat, 3, 1> rpy = orientation.toRotationMatrix().eulerAngles(0,1,2);


	/*	tf::Quaternion q;
			q.setRPY(0.0,0.0,0.0);
			q*/

		//CloudPtr result_cloud (new Cloud ());
	//	if(already <= iteration_number){
	  	    cloud_final_.reset (new Cloud);

		pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *cloud_final_, transformation);
		cloud_final_->header = cloud->header;

		//		result_cloud->header = cloud->header;
//		*cloud_final_ = *result_cloud;


		tf::transformEigenToTF(transf,current_pose_transform);	//Frame here is relative to camera_frame
		current_pose_transform.mult(initial_pose_transform.inverse(),current_pose_transform);
		//}
		/*

		Eigen::Quaternion q = current_pose_transform.getRotation();
		std::cout << "current1: " << q. q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

		q.x() = 2*q.x();
		q.y() = 2*q.y();
		q.z() = 2*q.z();

		std::cout << "current2: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;*/

		hand_passthrough(*cloud_pass_);

	//Prepare particle filter points to be published
		ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
		if(particles){
			//Set pointCloud with particle's points

		//	if(already > iteration_number)	return;

			cloud_particles.reset (new Cloud);

			for (size_t i = 0; i < particles->points.size (); i++){
				pcl::PointXYZ point;
				point.x = particles->points[i].x;
				point.y = particles->points[i].y;
				point.z = particles->points[i].z;
				cloud_particles->points.push_back(point);
			}

			cloud_particles->width = 1;
			cloud_particles->height =cloud_particles->points.size();
			//cloud_particles->header = cloud_final_ ->header;
			cloud_particles->header.frame_id = "camera_rgb_optical_frame";

			if(cloud_particles->points.size())
			pcl::io::savePCDFile ("/home/ir/catkin_ws/build/tracking_files/particles.pcd",*cloud_particles);
			//ROS_INFO_STREAM("Iteration number: " << already);
		//	already++;
		}
  	}

    //*cloud_final_ += *result_cloud;

	/*//Draw blue model reracked_ference point cloud
	{
	  pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);
    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
     viz.addPointCloud (result_cloud, blue_color, "resultcloud");
   }*/

}
//typedef boost::shared_ptr< ::sensor_msgs::PointCloud2 const> PointCloud2ConstPtr;
//void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
//boost::shared_ptr< >sensor_



void gripper_cb(const mrs_reg::gripper_dataConstPtr& data){

	boost::mutex::scoped_lock lock (mtx_);

	if(model_number == 1 && data->model_n == 2){		// OPEN -> CLOSE
		tracker_->setReferenceCloud(target_cloud_2);
		model_number = 2;
	}
	if(model_number == 2 && data->model_n == 1){		// CLOSE -> OPEN
		tracker_->setReferenceCloud(target_cloud_1);
		model_number = 1;
	}

}


int
main (int argc, char** argv)
{

	//Initialize ROS
	ros::init (argc, argv, "mrs_tracking_node");
	ros::NodeHandle nh;

	//Read input from arguments
	/*if (argc < 3)
	{
	  PCL_WARN("Please set device_id pcd_filename(e.g. $ %s '#1' sample.pcd)\n", argv[0]);
	  exit (1);
	}

	//read pcd file
	target_cloud.reset(new Cloud());
	if(pcl::io::loadPCDFile (argv[2], *target_cloud) == -1){
	std::cout << "pcd file not found" << std::endl;
	exit(-1);
	}

	std::string device_id = std::string (argv[1]);
	*/

	//Read input from file
	target_cloud_1.reset(new Cloud());
	target_cloud_2.reset(new Cloud());
	pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/tracking_files/open_model.pcd",*target_cloud_1);
	pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ir/catkin_ws/build/tracking_files/close_model.pcd",*target_cloud_2);
	model_number = 1;	//start with open

	std::string device_id = "A70777703995337A";
	counter = 0;

	//Set parameters
	new_cloud_  = false;
	downsampling_grid_size_model =  0.02;
	downsampling_grid_size_scene =  0.03;

	gridSampleApprox (target_cloud_1, *target_cloud_1, downsampling_grid_size_model);
	gridSampleApprox (target_cloud_2, *target_cloud_2, downsampling_grid_size_model);



	std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);

/*  default_step_covariance[3] *= 100.0;
  default_step_covariance[4] *= 100.0;
  default_step_covariance[5] *= 100.0;
*/

	std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.01);// 0.00001);
	std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
	(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

	//already = 0;
	iteration_number = 25;

	ParticleT bin_size;
	bin_size.x = 0.01f;
	bin_size.y = 0.01f;
	bin_size.z = 0.01f;
	bin_size.roll = 0.01f;
	bin_size.pitch = 0.01f;
	bin_size.yaw = 0.01f;

	//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
	tracker->setMaximumParticleNum (1000);
	tracker->setDelta (0.99);
	tracker->setEpsilon (0.1);
	tracker->setBinSize (bin_size);

	//Set all parameters for  ParticleFilter
	tracker_ = tracker;
	tracker_->setTrans (Eigen::Affine3f::Identity ());
	tracker_->setStepNoiseCovariance (default_step_covariance);
	tracker_->setInitialNoiseCovariance (initial_noise_covariance);
	tracker_->setInitialNoiseMean (default_initial_mean);
	tracker_->setIterationNum (1);
	tracker_->setParticleNum (300);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal (false);

	//Setup coherence object for tracking
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
	(new ApproxNearestPairPointCloudCoherence<RefPointType> ());

	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
	= boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
	coherence->addPointCoherence (distance_coherence);

	boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);
	tracker_->setCloudCoherence (coherence);

	//prepare the model of tracker's target
	Eigen::Vector4f c;
	Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
	CloudPtr transed_ref (new Cloud);
	CloudPtr transed_ref_downsampled (new Cloud);

	//Reading transform from filealready
	Eigen::Matrix4f transform;
	std::ifstream file("/home/ir/catkin_ws/build/tracking_files/cam2obj_final_Transformation.txt");
	std::string line;
	if(file.is_open()){
	  int row = 0;
	  while(getline(file, line)){
		  std::vector<std::string> vec;
		  boost::algorithm::trim_all(line);
		  boost::split(vec, line, boost::is_any_of(" "));
		  for(int i = 0; i < vec.size(); i++){
			  float el = boost::lexical_cast<float>(vec.at(i));
			  transform(row,i) = el;
		  }

		  row++;
	  }
	  file.close();
	}

	Eigen::Affine3f initial_transform;
	initial_transform.matrix() = transform;

	tf::Vector3 origin;
	origin.setValue(static_cast<double>(transform(0,3)),static_cast<double>(transform(1,3)),static_cast<double>(transform(2,3)));
	tf::Matrix3x3 tf3d;
	tf3d.setValue(static_cast<double>(transform(0,0)), static_cast<double>(transform(0,1)), static_cast<double>(transform(0,2)),
	        	  static_cast<double>(transform(1,0)), static_cast<double>(transform(1,1)), static_cast<double>(transform(1,2)),
				  static_cast<double>(transform(2,0)), static_cast<double>(transform(2,1)), static_cast<double>(transform(2,2)));
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);


//	pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans);
	//  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
	//  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
	//  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());

	//set reference model and trans

	tracker_->setReferenceCloud (target_cloud_1);	//assume opened model
	tracker_->setTrans (initial_transform);

	//Setup TF frame broadcaster for initial position
	tf::TransformBroadcaster initial_pose_bc;
	initial_pose_transform.setOrigin(origin);
	initial_pose_transform.setRotation(tfqt);

	//Initialize as identity (with respect to initial pose)
	tf::Quaternion q;
	q.setRPY(0.0,0.0,0.0);
	current_pose_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	current_pose_transform.setRotation(q);

	tf::TransformBroadcaster current_pose_bc;

	//Subscribe to receive Kinect data
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);
	ros::Subscriber sub2 = nh.subscribe("gripper_state_node/gripper_data", 1, gripper_cb);




//FRAMEWORK TO PUBLISH CLOUD
  ros::Publisher  pub = nh.advertise<Cloud>("mrs_tracking_node/tracked_model_cloud",1);
  ros::Publisher  pub2 = nh.advertise<Cloud>("mrs_tracking_node/particles_cloud",1);
  ros::Publisher  pub3 = nh.advertise<Cloud>("mrs_tracking_node/tracked_scene_cloud",1);

  while (nh.ok())
      {
	  	initial_pose_bc.sendTransform(tf::StampedTransform(initial_pose_transform, ros::Time::now(), "camera_rgb_optical_frame", "initial_pose"));
    	current_pose_bc.sendTransform(tf::StampedTransform(current_pose_transform, ros::Time::now(), "initial_pose", "current_pose"));


	  	if(cloud_final_){
        	pcl_conversions::toPCL(ros::Time::now(), cloud_final_->header.stamp);

        	pub.publish (*cloud_final_);
        	if(cloud_final_->points.size())
			pcl::io::savePCDFile ("/home/ir/catkin_ws/build/tracking_files/final.pcd",*cloud_final_);

        }
        if(cloud_particles){
        	pcl_conversions::toPCL(ros::Time::now(), cloud_particles->header.stamp);
        	pub2.publish (*cloud_particles);
        }
        if(new_hand_cloud){
		 	pcl_conversions::toPCL(ros::Time::now(), gripper_cloud->header.stamp);
        	pub3.publish(*gripper_cloud);
        	new_hand_cloud = false;
        }


	    ros::spinOnce ();
      }


  return 0;

}
