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

    //! metodebeskrivelse
    /*!
     *
     * \param *event
     * \return
     */
	void closeEvent(QCloseEvent *event); // Overloaded function
    void displayPointCloud(QString url, QString name);
    void displayPointCloudLeft(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    // Buttons
    void on_button_refresh_topics_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);
    void on_button_save_filtered_cloud_clicked(bool check);
    void on_button_filter_clicked(bool check);
    void on_button_add_cloud_clicked(bool check);
    void on_button_reload_cloud_clicked(bool check);
    void on_button_stl_clicked(bool check);
    void on_button_match_clicked(bool check);
    void on_button_open_gripper_clicked(bool check);
    void on_button_close_gripper_clicked(bool check);

    void on_button_emil_clicked(bool emil);

    void on_button_tester_clicked(bool check);

    void on_button_plan_move_clicked(bool check);
    void on_button_move_clicked(bool check);
    // Sliders
    // Filters
    void on_slider_1_valueChanged(int i);
    void on_slider_2_valueChanged(int i);
    void on_slider_3_valueChanged(int i);

    // SpinBoxes
    // Filters
    void on_spinBox_1_valueChanged(double d);
    void on_spinBox_2_valueChanged(double d);
    void on_spinBox_3_valueChanged(double d);
    // QSpinBoxes
    void on_filter_box_currentIndexChanged(int i);
    /******************************************
    ** Manual connections
    *******************************************/
    void setNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);
    void setNewVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
    void displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

Q_SIGNALS:



private:
	Ui::MainWindowDesign ui;
    void initializeUI();
	QNode qnode;
    QVTKWidget *w1;
    QVTKWidget *w2;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
    PointCloudManipulator *manipulator;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;
    std::vector<pcl::visualization::Camera> cam;
    bool changedFilter;

};

}  // namespace qt_master

#endif // qt_master_MAIN_WINDOW_H
