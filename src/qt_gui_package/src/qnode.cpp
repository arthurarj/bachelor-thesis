/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "/home/ir/catkin_ws/src/qt_gui_package/include/qt_gui_package/qnode.hpp"



using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_gui_package {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::gripper_cb(const qt_gui_package::gripper_dataConstPtr& gripper_msg){


	open_s = gripper_msg->m1_sim;
	close_s = gripper_msg->m2_sim;
	open_t = gripper_msg->thres_1;
	close_t = gripper_msg->thres_2;
	obj_t = gripper_msg->thres_obj;
	model_number = gripper_msg->model_n;
	cluster_n = gripper_msg->cluster_n;
	obj_detect = gripper_msg->obj_detect;

	Q_EMIT new_data();
}

bool QNode::init() {

//	ROS_INFO("INITARGC " << init_argc);
//	ROS_INFO("INITARGV " << init_argv);




	ros::init(init_argc,init_argv,"qt_gui_package");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	gripper_data_sub = n.subscribe ("gripper_state_node/gripper_data", 1, &QNode::gripper_cb, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qt_gui_package");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

gripper_data_sub = n.subscribe ("gripper_state_node/gripper_data", 1, &QNode::gripper_cb, this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;

	while ( ros::ok() ) {

		/*std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);*/
		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


}  // namespace qt_gui_package
