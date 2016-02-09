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
    qRegisterMetaType<boost::shared_ptr<pcl::visualization::PCLVisualizer> >("boost::shared_ptr<pcl::visualization::PCLVisualizer>");
    // Initialize UI
    ui.setupUi(this);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Initialize ROS
    qnode.init();

    // Manipulator / Filter
    manipulator = new PointCloudManipulator();
    QObject::connect(manipulator, SIGNAL(sendNewIndexInfo(QStringList,QList<bool>, QList<double>)), this, SLOT(setNewIndexInfo(QStringList,QList<bool>, QList<double>)));
    QObject::connect(manipulator, SIGNAL(sendNewVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer>)), this, SLOT(setNewVis(boost::shared_ptr<pcl::visualization::PCLVisualizer>)));
    QObject::connect(manipulator, SIGNAL(sendNewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, QString)), this, SLOT(displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, QString)));

    // Initialize UI
    initializeUI();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::displayPointCloud(QString url, QString name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(url.toUtf8().constData(), *tmpCloud) == -1){
        std::cout << "Could not load file" << std::endl;
        return;
    }
    displayCloud = tmpCloud;
    if(!viewer1->updatePointCloud(displayCloud, name.toUtf8().constData())){
        viewer1->addPointCloud(displayCloud, name.toUtf8().constData());
    }
    w1->update();
}

void MainWindow::displayPointCloudLeft(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, QString name)
{
    displayCloud = cloud;
    if(!viewer1->updatePointCloud(displayCloud, name.toUtf8().constData())){
        viewer1->addPointCloud(displayCloud, name.toUtf8().constData());
    }
    w1->update();
}

void MainWindow::displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, QString name)
{
    filteredCloud = cloud;
    if(!viewer2->updatePointCloud(filteredCloud, name.toUtf8().constData())){
        viewer2->addPointCloud(filteredCloud, name.toUtf8().constData());
    }
    w2->update();
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
    if(changedFilter == true){
        manipulator->getNewVisualizer(ui.filter_box->currentIndex());
        changedFilter = false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
    manipulator->runFilter(ui.filter_box->currentIndex(), displayCloud, tmpCloud, ui.spinBox_1->value(), ui.spinBox_2->value(), ui.spinBox_3->value(), ui.filter_XYZ->currentText());
}

void MainWindow::on_button_add_cloud_clicked(bool check)
{
    QStringList fileNames;
    fileNames = QFileDialog::getOpenFileNames(this,tr("Choose a .pcd file(s) to open"),"/home/",tr("PointClouds (*.pcd *.PCD)"));

    for (int i = 0; i<fileNames.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(fileNames.at(i).toUtf8().constData(), *tmpCloud);
        if(i == 0){
            displayCloud = tmpCloud;
        }
        else{
            std::cout << i << std::endl;
            *displayCloud += *tmpCloud;
        }
    }
    displayPointCloudLeft(displayCloud, "displayCloud");

}

void MainWindow::on_button_reload_cloud_clicked(bool check)
{
    pcl::io::savePCDFileASCII("/home/minions/tmp_cloud.pcd", *filteredCloud);
    displayPointCloud("/home/minions/tmp_cloud.pcd", "displayCloud");

    ui.logBox->append(manipulator->getLastFiltered());

}

void MainWindow::on_button_transform_clicked(bool check)
{
    manipulator->translateCloud(displayCloud, ui.rotX->value(), ui.rotY->value(), ui.rotZ->value(), ui.translX->value(), ui.translY->value(), ui.translZ->value());
}

void MainWindow::on_slider_1_valueChanged(int i)
{
    double d = i/100.0;
    if(ui.spinBox_1->minimum() < 0.0){
        ui.spinBox_1->setValue(ui.spinBox_1->minimum()+(d*(abs(ui.spinBox_1->maximum())+abs(ui.spinBox_1->minimum()))));
    }
    else{
        ui.spinBox_1->setValue(ui.spinBox_1->minimum()+(d*(ui.spinBox_1->maximum()-ui.spinBox_1->minimum())));
    }
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);

    }

}

void MainWindow::on_slider_2_valueChanged(int i)
{
    double d = i/100.0;
    if(ui.spinBox_2->minimum() < 0.0){
        ui.spinBox_2->setValue(ui.spinBox_2->minimum()+(d*(abs(ui.spinBox_2->maximum())+abs(ui.spinBox_2->minimum()))));
    }
    else{
        ui.spinBox_2->setValue(ui.spinBox_2->minimum()+(d*(ui.spinBox_2->maximum()-ui.spinBox_2->minimum())));
    }
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }

}

void MainWindow::on_slider_3_valueChanged(int i)
{
    double d = i/100.0;
    if(ui.spinBox_3->minimum() < 0.0){
        ui.spinBox_3->setValue(ui.spinBox_3->minimum()+(d*(abs(ui.spinBox_3->maximum())+abs(ui.spinBox_3->minimum()))));
    }
    else{
        ui.spinBox_3->setValue(ui.spinBox_3->minimum()+(d*(abs(ui.spinBox_3->maximum())-abs(ui.spinBox_3->minimum()))));
    }

    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_srotX_valueChanged(int i)
{
    double d = i;
    ui.rotX->setValue(d);
}
void MainWindow::on_srotY_valueChanged(int i)
{
    double d = i;
    ui.rotY->setValue(d);
}
void MainWindow::on_srotZ_valueChanged(int i)
{
    double d = i;
    ui.rotZ->setValue(d);
}

void MainWindow::on_stranslX_valueChanged(int i)
{
    double d = i;
    ui.translX->setValue(d);
}
void MainWindow::on_stranslY_valueChanged(int i)
{
    double d = i;
    ui.translY->setValue(d);
}
void MainWindow::on_stranslZ_valueChanged(int i)
{
    double d = i;
    ui.translZ->setValue(d);
}

void MainWindow::on_spinBox_1_valueChanged(double d)
{
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_spinBox_2_valueChanged(double d)
{
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_spinBox_3_valueChanged(double d)
{
    if(ui.auto_check->isChecked()){
        Q_EMIT on_button_filter_clicked(true);
    }
}

void MainWindow::on_rotX_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}
void MainWindow::on_rotY_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}
void MainWindow::on_rotZ_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}

void MainWindow::on_translX_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}
void MainWindow::on_translY_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}
void MainWindow::on_translZ_valueChanged(double d)
{
    if(ui.auto_trans->isChecked()){
        Q_EMIT on_button_transform_clicked(true);
    }
}

void MainWindow::on_filter_box_currentIndexChanged(int i)
{
    changedFilter = true;
    ui.auto_check->setChecked(false);
    manipulator->getNewIndexInfo(i);
}

void MainWindow::setNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange)
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
    ui.spinBox_1->setSingleStep(stepsAndRange.at(0));
    ui.spinBox_2->setSingleStep(stepsAndRange.at(1));
    ui.spinBox_3->setSingleStep(stepsAndRange.at(2));
    ui.spinBox_1->setMinimum(stepsAndRange.at(3));
    ui.spinBox_1->setMaximum(stepsAndRange.at(4));
    ui.spinBox_2->setMinimum(stepsAndRange.at(5));
    ui.spinBox_2->setMaximum(stepsAndRange.at(6));
    ui.spinBox_3->setMinimum(stepsAndRange.at(7));
    ui.spinBox_3->setMaximum(stepsAndRange.at(8));
    ui.slider_1->setVisible(show.at(0));
    ui.slider_2->setVisible(show.at(1));
    ui.slider_3->setVisible(show.at(2));
    ui.filter_XYZ->setVisible(show.at(3));

    Q_EMIT on_slider_1_valueChanged(50);
    Q_EMIT on_slider_2_valueChanged(50);
    Q_EMIT on_slider_3_valueChanged(50);
}

void MainWindow::setNewVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
    viewer2->getCameras(cam);
    viewer2 = vis;
    w2->SetRenderWindow(viewer2->getRenderWindow());
    viewer2->setCameraPosition(cam[0].pos[0],cam[0].pos[1],cam[0].pos[2],cam[0].view[0],cam[0].view[1],cam[0].view[2]);
}

void MainWindow::initializeUI()
{
    // Viewer setup
    w1 = new QVTKWidget();
    w2 = new QVTKWidget();
    viewer1.reset(new pcl::visualization::PCLVisualizer ("viewer1", false));
    viewer2.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    w1->SetRenderWindow(viewer1->getRenderWindow());
    w2->SetRenderWindow(viewer2->getRenderWindow());
    w1->update();
    w2->update();
    viewer1->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    viewer2->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    QHBoxLayout *layout = new QHBoxLayout;
    ui.groupBox_12->setLayout(layout);
    ui.groupBox_12->layout()->addWidget(w1);
    ui.groupBox_12->layout()->addWidget(w2);

    // Set up filter/manipulator
    ui.filter_box->addItems(manipulator->getFilters());
    QStringList xyz;
    xyz.append("x");
    xyz.append("y");
    xyz.append("z");
    ui.filter_XYZ->addItems(xyz);
}

}  // namespace qt_master

