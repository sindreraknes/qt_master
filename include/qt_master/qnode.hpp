#ifndef qt_master_QNODE_HPP_
#define qt_master_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <agilus_planner/Pose.h>
#include <kuka_rsi_hw_interface/write_8_outputs.h>
#include <math.h>

namespace qt_master {
/*!
 * \brief The QNode class is a ROS node that deals with the communication
 * between the robots, camera outputs etc.
 */
class QNode : public QThread {
    Q_OBJECT
public:
    /*!
     * \brief Constructor for QNode class.
     * \param argc Initialization argument
     * \param argv Initialization argument
     */
	QNode(int argc, char** argv );

    /*!
     * \brief Deconstructor for QNode class.
     */
	virtual ~QNode();

    /*!
     * \brief Initialization method for the ROS node.
     * \return Returns true if initialization is OK. Returns false if
     * initialization fails.
     */
    bool init();

    /*!
     * \brief Starts the ROS node and loops.
     */
	void run();

    /*!
     * \brief Callback method for point cloud from camera 1.
     * \param cloud_msg point cloud message from camera 1.
     */
    void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /*!
     * \brief Callback method for point cloud from camera 2.
     * \param cloud_msg point cloud message from camera 2.
     */
    void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /*!
     * \brief Callback method for point cloud from camera 3.
     * \param cloud_msg point cloud message from camera 3.
     */
    void cloudCallback3(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    /*!
     * \brief Asks the ROS master for every available point cloud topic name.
     * \return Returns a QStringList of available point cloud topics.
     */
    QStringList getTopics();

    /*!
     * \brief Sets the parameters to the pose object.
     * \param x position in x (meters)
     * \param y position in y (meters)
     * \param z position in z (meters)
     * \param roll orientation in roll (radians)
     * \param pitch orientation in pitch (radians)
     * \param yaw orientation in yaw (radians)
     */
    void setPose(double x, double y, double z, double roll, double pitch, double yaw);

    /*!
     * \brief Calls the service for planning a pose with given parameters.
     * \param x position in x (meters)
     * \param y position in y (meters)
     * \param z position in z (meters)
     * \param roll orientation in roll (radians)
     * \param pitch orientation in pitch (radians)
     * \param yaw orientation in yaw (radians)
     * \param robot selected robot, 0 is "Agilus 1" and 1 is "Agilus 2"
     */
    void planPose(double x, double y, double z, double roll, double pitch, double yaw, int robot);

    /*!
     * \brief Calls the service for moving to a pose with given parameters.
     * \param x position in x (meters)
     * \param y position in y (meters)
     * \param z position in z (meters)
     * \param roll orientation in roll (radians)
     * \param pitch orientation in pitch (radians)
     * \param yaw orientation in yaw (radians)
     * \param robot selected robot, 0 is "Agilus 1" and 1 is "Agilus 2"
     */
    void movePose(double x, double y, double z, double roll, double pitch, double yaw, int robot);

    /*!
     * \brief Opens the gripper at the end effector of the selected robot.
     * \param robot selected robot, 0 is "Agilus 1" and 1 is "Agilus 2"
     */
    void openGripper(int robot);

    /*!
     * \brief Closes the gripper at the end effector of the selected robot.
     * \param robot selected robot, 0 is "Agilus 1" and 1 is "Agilus 2"
     */
    void closeGripper(int robot);

    /*!
     * \brief Subscribes to all 3 point cloud topics (camera1, camera2 and camera3).
     */
    void subscribe3Clouds();


Q_SIGNALS:
    /*!
     * \brief Shuts down the ROS node in the correct manner.
     */
    void rosShutdown();

    /*!
     * \brief QtSignal for sending 3 aquired point clouds to another class (GUI class)
     * \param clouds a list of 3 point clouds
     */
    void send3Clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

public Q_SLOTS:

private:
    int init_argc; //!< Initialization arguments
    char** init_argv; //!< Initialization arguments
    ros::Subscriber pointCloudSub1; //!< ROS subscriber to the point cloud message from camera 1
    ros::Subscriber pointCloudSub2; //!< ROS subscriber to the point cloud message from camera 2
    ros::Subscriber pointCloudSub3; //!< ROS subscriber to the point cloud message from camera 3
    bool gotCloud1; //!< Flag to check if a point cloud from camera 1 is recieved
    bool gotCloud2; //!< Flag to check if a point cloud from camera 2 is recieved
    bool gotCloud3; //!< Flag to check if a point cloud from camera 3 is recieved
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1; //!< Point cloud for camera 1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2; //!< Point cloud for camera 2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3; //!< Point cloud for camera 3
    agilus_planner::Pose pose; //!< Pose object for the robot, contains xyz and orientation in rpy
    ros::ServiceClient planAg1; //!< ROS service for planning motion for Agilus 1
    ros::ServiceClient moveAg1; //!< ROS service for moving motion for Agilus 1
    ros::ServiceClient planAg2; //!< ROS service for planning motion for Agilus 2
    ros::ServiceClient moveAg2; //!< ROS service for moving motion for Agilus 2
    kuka_rsi_hw_interface::write_8_outputs gripperState; //!< Digital Output object to open/close gripper
    ros::ServiceClient gripperAg1; //!< ROS service for gripper handling for Agilus 1
    ros::ServiceClient gripperAg2; //!< ROS service for gripper handling for Agilus 2
};

}
#endif
