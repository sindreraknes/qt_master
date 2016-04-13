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
    QObject::connect(manipulator, SIGNAL(sendNewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, QString)), this, SLOT(displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, QString)));

    // Initialize UI
    initializeUI();
    //Q_EMIT on_button_tester_clicked(true);
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::displayPointCloud(QString url, QString name)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(url.toUtf8().constData(), *tmpCloud) == -1){
        std::cout << "Could not load file" << std::endl;
        return;
    }
    displayCloud = tmpCloud;
    if(!viewer1->updatePointCloud(displayCloud, name.toUtf8().constData())){
        viewer1->addPointCloud(displayCloud, name.toUtf8().constData());
    }
    w1->update();
}

void MainWindow::displayPointCloudLeft(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name)
{
    displayCloud = cloud;
    if(!viewer1->updatePointCloud(displayCloud, name.toUtf8().constData())){
        viewer1->addPointCloud(displayCloud, name.toUtf8().constData());
    }
    w1->update();
}

void MainWindow::displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name)
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


    manipulator->extractClusters(displayCloud, 0.02);
}

void MainWindow::on_button_save_filtered_cloud_clicked(bool check)
{
    QString selFilter="PCD files (*.pcd)";
    QString fileName = QFileDialog::getSaveFileName(this,"Save file",QDir::currentPath(),
        "PCD files (*.pcd)",&selFilter);
    if(!fileName.endsWith(".pcd",Qt::CaseInsensitive)){
        fileName.append(".pcd");
    }
    pcl::io::savePCDFileBinary(fileName.toUtf8().constData(), *filteredCloud);
}


void MainWindow::on_button_filter_clicked(bool check)
{
    if(changedFilter == true){
        manipulator->getNewVisualizer(ui.filter_box->currentIndex());
        changedFilter = false;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    manipulator->runFilter(ui.filter_box->currentIndex(), displayCloud, tmpCloud, ui.spinBox_1->value(), ui.spinBox_2->value(), ui.spinBox_3->value(), ui.filter_XYZ->currentText());
}

void MainWindow::on_button_add_cloud_clicked(bool check)
{
    QStringList fileNames;
    fileNames = QFileDialog::getOpenFileNames(this,tr("Choose a .pcd file(s) to open"),"/home/",tr("PointClouds (*.pcd *.PCD)"));
    if(ui.check_RGB->isChecked()){
        std::vector<float> zValues(217088);
        for (int i = 0; i<fileNames.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileNames.at(i).toUtf8().constData(), *tmpCloud);
            std::cout << tmpCloud->points.size() << std::endl;
            for(int k=0; k<tmpCloud->points.size(); k++){
                zValues[k] += tmpCloud->points[k].z*(1.0/fileNames.size());
            }
            if(i == fileNames.size()-1){
                for(int k=0; k<tmpCloud->points.size(); k++){
                    tmpCloud->points[k].z = zValues[k];
                    displayCloud = tmpCloud;
                }
            }
        }
        displayPointCloudLeft(displayCloud, "displayCloud");
    }
    else{
        for (int i = 0; i<fileNames.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileNames.at(i).toUtf8().constData(), *tmpCloud);
            if(i == 0){
                displayCloud = tmpCloud;
            }
            else{
                std::cout << i << std::endl;
                std::cout << tmpCloud->size() << std::endl;
                *displayCloud += *tmpCloud;
            }
        }
        displayPointCloudLeft(displayCloud, "displayCloud");
    }


}

void MainWindow::on_button_reload_cloud_clicked(bool check)
{
    pcl::io::savePCDFileBinary("/home/minions/tmp_cloud.pcd", *filteredCloud);
    displayPointCloud("/home/minions/tmp_cloud.pcd", "displayCloud");

    ui.logBox->append(manipulator->getLastFiltered());

}

void MainWindow::on_button_stl_clicked(bool check)
{
    QString fileName;
    fileName = QFileDialog::getOpenFileName(this,tr("Choose a .stl file to open"),"/home/",tr("STL File (*.stl *.STL)"));
    filteredCloud = manipulator->sampleSTL(fileName,300,1);
    //displayPointCloudLeft(filteredCloud, "filteredCloud");

}

void MainWindow::on_button_match_clicked(bool check)
{
    QString modelName;
    modelName = QFileDialog::getOpenFileName(this,tr("Choose MODEL cloud"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(modelName.toStdString(), *modelCloud);
    QString sceneName;
    sceneName = QFileDialog::getOpenFileName(this,tr("Choose SCENE cloud"),"/home/",tr("PCD File (*.pcd *.PCD)"));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(sceneName.toStdString(), *sceneCloud);

    manipulator->matchModelCloud(modelCloud, sceneCloud);

}

void MainWindow::on_button_open_gripper_clicked(bool check)
{
    if(ui.robot_selector->currentIndex() == 0){
        // Agilus1
        qnode.openGripper(0);
    }
    else if(ui.robot_selector->currentIndex() == 1){
        // Agilus2
        qnode.openGripper(1);
    }
}

void MainWindow::on_button_close_gripper_clicked(bool check)
{
    if(ui.robot_selector->currentIndex() == 0){
        // Agilus1
        qnode.closeGripper(0);
    }
    else if(ui.robot_selector->currentIndex() == 1){
        // Agilus2
        qnode.closeGripper(1);
    }
}





void MainWindow::on_button_tester_clicked(bool check)
{
    QStringList fileNames;
    fileNames = QFileDialog::getOpenFileNames(this,tr("Choose a .pcd file(s) to open"),"/home/minions/Downloads/",tr("PointClouds (*.pcd *.PCD)"));
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileNames.at(0).toUtf8().constData(), *cloud1);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileNames.at(1).toUtf8().constData(), *cloud2);
    //manipulator->tester2(cloud1, cloud2);

    //manipulator->alignClouds(fileNames);

    //manipulator->alignRobotCell(fileNames);

    manipulator->refineAlignment(fileNames);
    //manipulator->roflKinect(fileNames);
}

void MainWindow::on_button_plan_move_clicked(bool check)
{
    if(ui.robot_selector->currentIndex() == 0){
        // Agilus1
        std::cout << "SUPPOSED TO PLAN AG1" << std::endl;
        qnode.planPose(ui.x_pos->value(), ui.y_pos->value(), ui.z_pos->value(), ui.roll->value(), ui.pitch->value(), ui.yaw->value(),0);
    }
    else if(ui.robot_selector->currentIndex() == 1){
        // Agilus2
        std::cout << "SUPPOSED TO PLAN AG2" << std::endl;
        qnode.planPose(ui.x_pos->value(), ui.y_pos->value(), ui.z_pos->value(), ui.roll->value(), ui.pitch->value(), ui.yaw->value(),1);
    }

}

void MainWindow::on_button_move_clicked(bool check)
{

    if(ui.robot_selector->currentIndex() == 0){
        // Agilus1
        qnode.movePose(ui.x_pos->value(), ui.y_pos->value(), ui.z_pos->value(), ui.roll->value(), ui.pitch->value(), ui.yaw->value(),0);
    }
    else if(ui.robot_selector->currentIndex() == 1){
        // Agilus2
        qnode.movePose(ui.x_pos->value(), ui.y_pos->value(), ui.z_pos->value(), ui.roll->value(), ui.pitch->value(), ui.yaw->value(),1);
    }
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

