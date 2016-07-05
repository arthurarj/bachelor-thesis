/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <qdebug.h>
#include <QMessageBox>
#include <iostream>
#include "/home/ir/catkin_ws/src/qt_gui_package/include/qt_gui_package/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_gui_package {


using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    count = 0;

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	//ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
//    ui.view_logging->setModel(qnode.loggingModel());
//    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    //MODIFIED TO DISPLAY INFORMATION REGARDING THE STATES
    ui.view_logging->setModel(loggingModel());
    QObject::connect(this, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	//Assume opened model
    model_n = 1;
	ui.label_cur_model->setPixmap(QPixmap(":images/model1.bmp"));
	current_state = 0;
	previous_state = -1;

    ui.label_state_diag->setPixmap(QPixmap(":images/state_diag.png"));
    ui.label_object->setPixmap(QPixmap(":images/object_0.png"));
    //ui.label_ state_diag->setPixmap(QPixmap(":images/state_diag.png"));

    /*   ui.label_state_diag->setGeometry(ui.label_state_diag->geometry().x(), ui.label_state_diag->geometry().y(),
    		0.5*ui.verticalLayout->geometry().width(), ui.label_state_diag->geometry().height());
*/
	first_run = true;
	object = false;
    /* Connect right away*/
  //  ui.button_connect->click();

	ui.line1->setStyleSheet("#line1 { border: 2px solid black; }");
	ui.line2->setStyleSheet("#line2 { border: 2px solid black; }");
	ui.line1->setGeometry(ui.progressBar1->x() + 0.5*(ui.progressBar1->width() - ui.line1->width()),
					 	  ui.progressBar1->y() + 0.5*(ui.progressBar1->height() - ui.line1->height()), 2, ui.line1->height());
	ui.line2->setGeometry(ui.progressBar2->x() + 0.5*(ui.progressBar2->width() - ui.line2->width()),
						  ui.progressBar2->y() + 0.5*(ui.progressBar2->height() - ui.line2->height()), 2, ui.line2->height());

	//ui.checkbox_use_environment->setChecked(true);
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    ROS_INFO("TESTE");

	if ( ui.checkbox_use_environment->isChecked() ) {		//using environment variables
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);

			//Connect signal from NODE to SLOT in window
            QObject::connect(&qnode, SIGNAL((new_data)), this, SLOT(update_data()));
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {

			QObject::connect(&qnode, SIGNAL(new_data()), this, SLOT(update_data()));
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
        enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
bool MainWindow::grasp_stability_function(){

	bool out = false;
	if(qnode.cluster_n > 1){ grasp_stability_count = 0; return false;}
	grasp_stability_count++;	//number of inputs
	if(grasp_stability_count > 3)
		out = true;
	return out;
}

bool MainWindow::relea_stability_function(){

	bool out = false;
	if(qnode.cluster_n == 1){ relea_stability_count = 0; return false;}
	relea_stability_count++;	//number of inputs
	if(relea_stability_count > 5)
		out = true;
	return out;
}


void MainWindow::log(const QString &logging_model_msg){

		logging_model.insertRow(logging_model.rowCount());
		QModelIndex index = logging_model.index(logging_model.rowCount()-1);
		logging_model.setData(index, logging_model_msg);
		Q_EMIT loggingUpdated(); // used to readjust the scrollbar

}
void MainWindow::state_update(){

	//UPDATE STATE
	switch(current_state){
		case 0:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_0.png"));
												   log("State update: Fully opened gripper. Object detection is reliable.");
												   }
			   previous_state = current_state;
			   if(qnode.open_s > qnode.open_t){		    current_state = 0;														//fully_opened
			   	   	   	   	   	   	   	   	   	   	    if(qnode.open_s > qnode.obj_t) object = qnode.obj_detect;
			   	   	   	   	   	   	   	   	   	   	    if(object)
			   	   	   	   	   	   	   	   	   	   	    	ui.label_object->setPixmap(QPixmap(":images/object_1.png"));
			   	   	   	   	   	   	   	   	   	   	    else
			   	   	   	   	   	   	   	   	   	   	    	ui.label_object->setPixmap(QPixmap(":images/object_0.png"));
			   }
			   else if(!object)					   		current_state = 1;														//opened
			   	    else						   	   {current_state = 3;	grasp_stability_count = 0;}							//grasping
			   break;
		case 1:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_1.png"));
												   log("State update: Gripper in intermediate opening state. For object detection, open it fully.");}
		   	   previous_state = current_state;
			   if(qnode.open_s > qnode.open_t) 	   		current_state = 0;														//fully_opened
			   else if(qnode.close_s > qnode.close_t) 	current_state = 2;														//closed
				   	else								current_state = 1;														//opened
			   break;
		case 2:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_2.png"));
		log("State update: Fully closed gripper.");}
	   	       previous_state = current_state;
			   if(qnode.close_s > qnode.close_t)		current_state = 2;														//closed
			   else										current_state = 1;														//opened
			   break;
		case 3:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_3.png"));
		log("State update: Grasping started.");}
	   	       previous_state = current_state;
			   if(qnode.open_s > qnode.open_t)			current_state = 0;														//fully_opened
			   else if(!grasp_stability_function())		current_state = 3;														//grasping
						else 						   {current_state = 4; relea_stability_count = 0;}							//grasped
			   break;
		case 4:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_4.png"));
		log("State update: Grasping completed.");}

	       	   previous_state = current_state;
			   if(qnode.cluster_n == 1)					current_state = 4;														//grasped
			   else										current_state = 5;														//release
			   break;
		case 5:if(current_state != previous_state){ui.label_state_diag->setPixmap(QPixmap(":images/state_diag_5.png"));
			   log("State update: Object being released");}
	       	   previous_state = current_state;
	       	   object = false;
	       	   ui.label_object->setPixmap(QPixmap(":images/object_0.png"));
	       	   if(qnode.open_s > qnode.open_t) 	   		current_state = 0;														//fully_opened
		   	   else if(qnode.close_s > qnode.close_t) 	current_state = 2;														//closed
		   	   else										current_state = 5;														//release
		   	   break;
	}



}


void MainWindow::update_data(){
	ROS_INFO_STREAM("test");

	state_update();
	ui.label_6->setText(QString::number(100*qnode.open_s, 'd',0) + " %");
	ui.label_7->setText(QString::number(100*qnode.close_s,'d',0) + " %");

	if(model_n == 2 && qnode.model_number == 1){
		ui.label_cur_model->setPixmap(QPixmap(":images/model1.bmp"));
		model_n = 1;
	}
	if(model_n == 1 && qnode.model_number == 2){
		ui.label_cur_model->setPixmap(QPixmap(":images/model2.bmp"));
		model_n = 2;
	}
	ui.progressBar1->setValue(qnode.open_s*100);
	ui.progressBar2->setValue(qnode.close_s*100);

	if(first_run){
		ui.line1->setGeometry(ui.progressBar1->x() +  qnode.open_t*(ui.progressBar1->width() - ui.line1->width()), ui.line1->y(), ui.line1->width(), ui.line1->height());
		ui.line2->setGeometry(ui.progressBar2->x() + qnode.close_t*(ui.progressBar2->width() - ui.line2->width()), ui.line2->y(), ui.line2->width(), ui.line2->height());
		first_run = false;
	}
}



/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  //  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"))
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "name");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://localhost:11311")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_master->setCursorPosition(0);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qt_gui_package");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qt_gui_package

