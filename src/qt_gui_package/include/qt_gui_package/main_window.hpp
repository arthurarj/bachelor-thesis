/**
 * @file /include/qt_gui_package/main_window.hpp
 *
 * @brief Qt based gui for qt_gui_package.
 *
 * @date November 2010
 **/
#ifndef qt_gui_package_MAIN_WINDOW_H
#define qt_gui_package_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <qmainwindow.h>	//testing
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_gui_package {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

	void update_data();
	void state_update();
	bool grasp_stability_function();
	bool relea_stability_function();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void log(const QString &msg);

Q_SIGNALS:
	void loggingUpdated();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	int model_n;
	int grasp_stability_count, relea_stability_count;
	bool first_run, object;
	int current_state;
	int previous_state;
	int count;
	//STATE LOGGING MODEL
	QStringListModel* loggingModel() { return &logging_model; }
    QStringListModel logging_model;
};

}  // namespace qt_gui_package

#endif // qt_gui_package_MAIN_WINDOW_H
