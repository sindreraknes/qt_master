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
    ui.setupUi(this);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Initialize ROS
    qnode.init();

    // Manipulator / Filter
    manipulator = new PointCloudManipulator();
    QObject::connect(manipulator, SIGNAL(sendNewIndexInfo(QStringList,QList<bool>)), this, SLOT(setNewIndexInfo(QStringList,QList<bool>)));

    // Initialize UI
    initializeUI();





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

void MainWindow::displayPointCloud(QString url)
{

    std::cout << url.toUtf8().constData() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(url.toUtf8().constData(), *tmpCloud) == -1){
        std::cout << "Could not load file" << std::endl;
        return;
    }
    displayCloud = tmpCloud;
    if(!viewer->updatePointCloud(displayCloud, "displayCloud")){
        viewer->addPointCloud(displayCloud, "displayCloud", left);
        w->update();
    }
    w->update();
}



void MainWindow::on_button_refresh_topics_clicked(bool check)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(qnode.getTopics());
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    ui.dock_status->setVisible(true);
    if(ui.comboBox->currentText().length() != 0){

    }
}

void MainWindow::on_button_filter_clicked(bool check)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
    manipulator->runFilter(ui.filter_box->currentIndex(), displayCloud, tmpCloud, ui.spinBox_1->value(), ui.spinBox_2->value(), ui.spinBox_3->value(), ui.filter_XYZ->currentText());
    filteredCloud = tmpCloud;
    if(!viewer->updatePointCloud(filteredCloud, "filteredCloud")){
        viewer->addPointCloud(filteredCloud, "filteredCloud", right);
        w->update();
    }
    w->update();
}

void MainWindow::on_button_add_cloud_clicked(bool check)
{
    QString fileName;
    //QString filters = "PointClouds (*.pcd);; PointClouds (*.PCD)";
    fileName = QFileDialog::getOpenFileName(this,tr("Choose a .pcd file to open"),"/home/",tr("PointClouds (*.pcd *.PCD)"));
    if(fileName.length() != 0){
        displayPointCloud(fileName);
    }
    else{
        std::cout << "Error while loading file" << std::endl;
    }


}

void MainWindow::on_slider_1_valueChanged(int i)
{
    double d = i/10.0;
    ui.spinBox_1->setValue(d);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }

}

void MainWindow::on_slider_2_valueChanged(int i)
{
    double d = i/10.0;
    ui.spinBox_2->setValue(d);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }

}

void MainWindow::on_slider_3_valueChanged(int i)
{
    double d = i/10.0;
    ui.spinBox_3->setValue(d);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_spinBox_1_valueChanged(double d)
{
    int i = d*10;
    ui.slider_1->setValue(i);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_spinBox_2_valueChanged(double d)
{
    int i = d*10;
    ui.slider_2->setValue(i);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_spinBox_3_valueChanged(double d)
{
    int i = d*10;
    ui.slider_3->setValue(i);
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_filter_box_currentIndexChanged(int i)
{
    manipulator->getNewIndexInfo(i);
}

void MainWindow::setNewIndexInfo(QStringList labels, QList<bool> show)
{
    ui.label_f_1->setText(labels.at(0));
    ui.label_f_2->setText(labels.at(1));
    ui.label_f_3->setText(labels.at(2));
    ui.label_f_1->setVisible(show.at(0));
    ui.label_f_2->setVisible(show.at(1));
    ui.label_f_3->setVisible(show.at(2));
    ui.spinBox_1->setVisible(show.at(0));
    ui.spinBox_2->setVisible(show.at(1));
    ui.spinBox_3->setVisible(show.at(2));
    ui.slider_1->setVisible(show.at(0));
    ui.slider_2->setVisible(show.at(1));
    ui.slider_3->setVisible(show.at(2));
    ui.auto_check->setVisible(show.at(3));
}

void MainWindow::initializeUI()
{
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
    ui.filter_box->addItems(manipulator->getFilters());
    QStringList xyz;
    xyz.append("x");
    xyz.append("y");
    xyz.append("z");
    ui.filter_XYZ->addItems(xyz);


}

}  // namespace qt_master

