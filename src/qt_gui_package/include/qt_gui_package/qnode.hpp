/**
 * @file /include/qt_gui_package/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_gui_package_QNODE_HPP_
#define qt_gui_package_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <qt_gui_package/gripper_data.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_gui_package {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();																//using environment variables
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	double open_s;
	double close_s;
	double open_t;
	double close_t;
	double obj_t;
	int model_number;
	int cluster_n;
	bool obj_detect;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	void gripper_cb(const qt_gui_package::gripper_dataConstPtr& gripper_msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void new_data();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Subscriber gripper_data_sub;

};

}  // namespace qt_gui_package

#endif /* qt_gui_package_QNODE_HPP_ */
