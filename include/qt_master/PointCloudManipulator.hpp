
#ifndef qt_master_POINT_CLOUD_MANIPULATOR_H
#define qt_master_POINT_CLOUD_MANIPULATOR_H

#include <QStringList>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/transforms.h>


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
    void filterVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double leafSize);
    void filterMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double windowSize, double maxMovement);
    void filterNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double radius, double nrToDisplay);
    void translateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ);
    void getNewVisualizer(int selectedFilter);
    QString getLastFiltered();


Q_SIGNALS:
    void sendNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);
    void sendNewVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
    void sendNewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, QString name);

public Q_SLOTS:

private:
    QStringList filterList;
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
    pcl::MedianFilter<pcl::PointXYZ> medianFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierFilter;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
    QString lastFiltered;




};

}


#endif
