
#include "../include/qt_master/PointCloudManipulator.hpp"
#include <iostream>


namespace qt_master {

// Constructor
PointCloudManipulator::PointCloudManipulator(QObject *parent) :
    QObject(parent)
{

}
// Destructor
PointCloudManipulator::~PointCloudManipulator(){}

QStringList PointCloudManipulator::getFilters()
{
    filterList.append("Filter1");
    filterList.append("Filter2");

    return filterList;
}

void PointCloudManipulator::runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double d1, double d2, double d3, QString xyz)
{
    switch(selectedFilter)
    {
    case 0:
        // CODE
        filterPassThrough(inCloud, outCloud, d1, d2, xyz);
        break;
    case 1:
        // CODE
        break;
    case 2:
        // CODE
        break;
    default:
        ;
        // SOMETHING WENT WRONG
    }
}

void PointCloudManipulator::getNewIndexInfo(int selectedFilter)
{
    QStringList labels;
    QList<bool> show;
    switch(selectedFilter)
    {
    case 0:
        // PASSTHROUGH FILTER
        labels.append("Minimum:");
        labels.append("Maximum");
        labels.append("");
        show.append(true);
        show.append(true);
        show.append(false);
        show.append(true);
        Q_EMIT sendNewIndexInfo(labels, show);
        break;
    case 1:
        // CODE
        break;
    case 2:
        // CODE
        break;
    default:
        ;
        // SOMETHING WENT WRONG
    }
}

void PointCloudManipulator::filterPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double limitMin, double limitMax, QString field)
{
    passThroughFilter.setInputCloud(inCloud);
    passThroughFilter.setFilterFieldName(field.toStdString());
    passThroughFilter.setFilterLimits(limitMin, limitMax);
    passThroughFilter.setKeepOrganized(true);
    passThroughFilter.filter(*outCloud);
}





}
