
#ifndef qt_master_POINT_CLOUD_MANIPULATOR_H
#define qt_master_POINT_CLOUD_MANIPULATOR_H


#include <QObject>
#include <QStringList>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>


namespace qt_master {

class PointCloudManipulator : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudManipulator(QObject *parent = 0);
    ~PointCloudManipulator();
    QStringList getFilters();
    void runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double d1, double d2, double d3, QString xyz);
    void getNewIndexInfo(int selectedFilter);
    void filterPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double limitMin, double limitMax, QString field);


Q_SIGNALS:
    void sendNewIndexInfo(QStringList labels, QList<bool> show);

public Q_SLOTS:

private:
    QStringList filterList;
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;


};



}


#endif
