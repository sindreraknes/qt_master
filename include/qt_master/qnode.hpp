/**
 * @file /include/qt_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_master_QNODE_HPP_
#define qt_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

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


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_master {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
	void run();
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    QStringList getTopics();
    void subscribeToPointCloud2(QString string);
    void setPose(double x, double y, double z, double roll, double pitch, double yaw);
    void planPose(double x, double y, double z, double roll, double pitch, double yaw, int robot);
    void movePose(double x, double y, double z, double roll, double pitch, double yaw, int robot);

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Subscriber pointCloud2Sub;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    agilus_planner::Pose pose;
    ros::ServiceClient planAg1;
    ros::ServiceClient moveAg1;
    ros::ServiceClient planAg2;
    ros::ServiceClient moveAg2;
    kuka_rsi_hw_interface::write_8_outputs gripperState;
    ros::ServiceClient gripperAg1;
    ros::ServiceClient gripperAg2;

};

}  // namespace qt_master

#endif /* qt_master_QNODE_HPP_ */
