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
#include <QMessageBox>
#include <iostream>
#include "../include/qt_master/main_window.hpp"



namespace qt_master {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    // Initialize UI
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
	setWindowIcon(QIcon(":/images/icon.png"));
//	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    // Initialize ROS
    qnode.init();

    // Viewer setup
    w = new QVTKWidget();
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
    w->SetRenderWindow(viewer->getRenderWindow());
    left = 0;
    right = 1;
    viewer->createViewPort(0, 0, 0.5, 1, left);
    viewer->createViewPort(0.5, 0, 1, 1, right);
    w->update();
    viewer->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    //viewer->updateCamera();
    ui.groupBox_12->layout()->addWidget(w);

    // Set up filter/manipulator
    manipulator = new PointCloudManipulator();
    
    // Connections



}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/



/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/



void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::on_button_connect_clicked(bool check)
{
}

void MainWindow::on_button_refresh_topics_clicked(bool check)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(qnode.getTopics());
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    if(ui.comboBox->currentText().length() != 0){

    }
}

}  // namespace qt_master

