#ifndef qt_master_MAIN_WINDOW_H
#define qt_master_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include "PointCloudManipulator.hpp"
#include <ros/ros.h>
#include <ros/package.h>

namespace qt_master {
/*!
 * \brief The MainWindow class is the graphical user interface (GUI) class and
 * controls events, updates visualizer etc.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    /*!
     * \brief Constructor for MainWindow class.
     * \param argc Initialization argument
     * \param argv Initialization argument
     * \param parent parent graphics component
     */
    MainWindow(int argc, char** argv, QWidget *parent = 0);

    /*!
     * \brief Deconstructor for MainWindow class.
     */
    ~MainWindow();

    /*!
     * \brief Overloaded function that is called when the GUI is closed.
     * \param event the event that occured.
     */
    void closeEvent(QCloseEvent *event);

    /*!
     * \brief Loads a point cloud from file and displays it.
     * \param url path to file
     * \param name name of cloud (has to be unique if there already is one)
     */
    void displayPointCloud(QString url, QString name);

    /*!
     * \brief Displays a point cloud on the left visualizer.
     * \param cloud the point cloud object
     * \param name name of the cloud (has to be unique if there already is one)
     */
    void displayPointCloudLeft(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

public Q_SLOTS:
    /*!
     * \brief Refreshes the topic selector with new topic names.
     * \param check clicked event
     */
    void on_button_refresh_topics_clicked(bool check);

    /*!
     * \brief Subscribes to a selected topic.
     * \param check clicked event
     */
    void on_button_subscribe_topic_clicked(bool check);

    /*!
     * \brief Saves a filtered cloud.
     * \param check clicked event
     */
    void on_button_save_filtered_cloud_clicked(bool check);

    /*!
     * \brief Runs the selected filter.
     * \param check clicked event
     */
    void on_button_filter_clicked(bool check);

    /*!
     * \brief Adds a point cloud to the visualizer.
     * \param check clicked event
     */
    void on_button_add_cloud_clicked(bool check);

    /*!
     * \brief Saves the currently filtered cloud and reloads it
     * to the left visualizer. Used when using multiple filters
     * on the same point cloud.
     * \param check clicked event
     */
    void on_button_reload_cloud_clicked(bool check);

    /*!
     * \brief Loads a .STL file and converts it to a point cloud.
     * \param check clicked event
     */
    void on_button_stl_clicked(bool check);

    /*!
     * \brief Matches a scene with a model. Model and scene
     * is selected from saved point clouds.
     * \param check clicked event
     */
    void on_button_match_clicked(bool check);

    /*!
     * \brief Opens the gripper for the selected robot.
     * \param check clicked event
     */
    void on_button_open_gripper_clicked(bool check);

    /*!
     * \brief Closes the gripper for the selected robot.
     * \param check clicked event
     */
    void on_button_close_gripper_clicked(bool check);

    /*!
     * \brief Aquires 3 point clouds from camera1, camera2 and camera3,
     * merges them together, and perform object detection on the
     * selected model.
     * \param check clicked event
     */
    void on_button_align_match_clicked(bool check);

    /*!
     * \brief Tester button.
     * \param check clicked event
     */
    void on_button_tester_clicked(bool check);

    /*!
     * \brief Plans a pose for the selected robot.
     * \param check clicked event
     */
    void on_button_plan_move_clicked(bool check);

    /*!
     * \brief Moves the selected robot to a pose.
     * \param check clicked event
     */
    void on_button_move_clicked(bool check);

    /*!
     * \brief Runs if slider_1 changes value.
     * \param i the new value
     */
    void on_slider_1_valueChanged(int i);

    /*!
     * \brief Runs if slider_2 changes value.
     * \param i the new value
     */
    void on_slider_2_valueChanged(int i);

    /*!
     * \brief Runs if slider_3 changes value.
     * \param i the new value
     */
    void on_slider_3_valueChanged(int i);

    /*!
     * \brief Runs if spinBox_1 changes value.
     * \param d the new value
     */
    void on_spinBox_1_valueChanged(double d);

    /*!
     * \brief Runs if spinBox_2 changes value.
     * \param d the new value
     */
    void on_spinBox_2_valueChanged(double d);

    /*!
     * \brief Runs if spinBox_3 changes value.
     * \param d the new value
     */
    void on_spinBox_3_valueChanged(double d);

    /*!
     * \brief Runs if filter_box changes value.
     * \param i the new value
     */
    void on_filter_box_currentIndexChanged(int i);

    /*!
     * \brief Method that sets new parameters for the selected filter.
     * \param labels label names
     * \param show which label/spinbox/slider should show
     * \param stepsAndRange step and range for spinbox/slider
     */
    void setNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);

    /*!
     * \brief Sets a new visualizer object to the GUI.
     * \param vis the new visualizer
     */
    void setNewVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

    /*!
     * \brief Displays a point cloud object with a name
     * \param cloud the point cloud object
     * \param name name of the cloud (has to be unique if there already is one)
     */
    void displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

    /*!
     * \brief Recieves 3 point clouds from the ROS node.
     * \param clouds a list of 3 point clouds
     */
    void receive3Clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

Q_SIGNALS:

private:
    Ui::MainWindowDesign ui; //!< The UI object (contains the .ui file)
    void initializeUI(); //!< Initialization of the GUI.
    QNode qnode; //!< QNode object instance.
    QVTKWidget *w1; //!< Widget to contain the left visualizer.
    QVTKWidget *w2; //!< Widget to contain the right visualizer.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1; //!< The left visualizer.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2; //!< The right visualizer.
    PointCloudManipulator *manipulator; //!< PointCloudManipulator object instance.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud; //!< The point cloud on the left.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud; //!< The point cloud on the right.
    std::vector<pcl::visualization::Camera> cam; //!< Vector of camera positions.
    bool changedFilter; //!< Flag for changing filter in the GUI.
};
}
#endif
