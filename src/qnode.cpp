/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qt_master/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_master {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_master");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    planAg1 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag1/plan_pose");
    moveAg1 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag1/go_to_pose");
    planAg2 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag2/plan_pose");
    moveAg2 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag2/go_to_pose");
    gripperAg1 = n.serviceClient<kuka_rsi_hw_interface::write_8_outputs>("/ag1/kuka_hardware_interface/write_8_digital_outputs");
    gripperAg2 = n.serviceClient<kuka_rsi_hw_interface::write_8_outputs>("/ag2/kuka_hardware_interface/write_8_digital_outputs");

    gotCloud1 = false;
    gotCloud2 = false;
    gotCloud3 = false;

	start();

	return true;
}



void QNode::run() {
    ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {

        if(gotCloud1 && gotCloud2 && gotCloud3){
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
            clouds.push_back(cloud1);
            clouds.push_back(cloud2);
            clouds.push_back(cloud3);
            Q_EMIT send3Clouds(clouds);
            pointCloudSub1.shutdown();
            pointCloudSub2.shutdown();
            pointCloudSub3.shutdown();
        }

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

QStringList QNode::getTopics()
{
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());
        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/PointCloud2", Qt::CaseInsensitive) == 0){
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    return list;
}


void QNode::setPose(double x, double y, double z, double roll, double pitch, double yaw)
{
    pose.request.header.frame_id = "/world";
    pose.request.set_position = true;
    pose.request.set_orientation = true;
    pose.request.position_x = x;
    pose.request.position_y = y;
    pose.request.position_z = z;
    pose.request.orientation_r = roll*M_PI/180.0;
    pose.request.orientation_p = pitch*M_PI/180.0;
    pose.request.orientation_y = yaw*M_PI/180.0;

}

void QNode::planPose(double x, double y, double z, double roll, double pitch, double yaw, int robot)
{
    setPose(x,y,z,roll,pitch,yaw);
    if(robot == 0){
        //Agilus1
        planAg1.call(pose);
    }
    else if(robot == 1){
        //Agilus2
        planAg2.call(pose);
    }
}

void QNode::movePose(double x, double y, double z, double roll, double pitch, double yaw, int robot)
{
    setPose(x,y,z,roll,pitch,yaw);
    if(robot == 0){
        //Agilus1
        moveAg1.call(pose);
    }
    else if(robot == 1){
        //Agilus2
        moveAg2.call(pose);
    }
}

void QNode::openGripper(int robot)
{
    gripperState.request.out1 = false;
    gripperState.request.out4 = true;
    // Tool changer
    gripperState.request.out2 = true;
    if(robot == 0){
        //Agilus1
        gripperAg1.call(gripperState);
    }
    else if(robot == 1){
        //Agilus2
        gripperAg2.call(gripperState);
    }
}

void QNode::closeGripper(int robot)
{
    gripperState.request.out1 = true;
    gripperState.request.out4 = false;
    // Tool changer
    gripperState.request.out2 = true;

    if(robot == 0){
        //Agilus1
        gripperAg1.call(gripperState);
    }
    else if(robot == 1){
        //Agilus2
        gripperAg2.call(gripperState);
    }
}

void QNode::subscribe3Clouds()
{
    ros::NodeHandle n;
    pointCloudSub1 = n.subscribe<sensor_msgs::PointCloud2, QNode>("/NUC1/sd/points", 10, &QNode::cloudCallback1, this);
    pointCloudSub2 = n.subscribe<sensor_msgs::PointCloud2, QNode>("/NUC2/sd/points", 10, &QNode::cloudCallback2, this);
    pointCloudSub3 = n.subscribe<sensor_msgs::PointCloud2, QNode>("/PC/sd/points", 10, &QNode::cloudCallback3, this);
}


void QNode::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
        pcl::fromROSMsg(*cloud_msg, *cloud1);
        gotCloud1 = true;
}
void QNode::cloudCallback2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
        pcl::fromROSMsg(*cloud_msg, *cloud2);
        gotCloud2 = true;
}
void QNode::cloudCallback3(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
        pcl::fromROSMsg(*cloud_msg, *cloud3);
        gotCloud3 = true;
}


}  // namespace qt_master
