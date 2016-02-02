/**
 * @file /include/qt_master/main_window.hpp
 *
 * @brief Qt based gui for qt_master.
 *
 * @date November 2010
 **/
#ifndef qt_master_MAIN_WINDOW_H
#define qt_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include "PointCloudManipulator.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_master {

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

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_button_connect_clicked(bool check);
    void on_button_refresh_topics_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/

Q_SIGNALS:



private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QVTKWidget *w;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudManipulator *manipulator;

    // Viewports
    int left;
    int right;

};

}  // namespace qt_master

#endif // qt_master_MAIN_WINDOW_H
