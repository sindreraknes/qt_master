#include "../include/qt_master/PointCloudManipulator.hpp"

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
    filterList.append("Passthrough");
    filterList.append("VoxelGrid");
    filterList.append("Median");
    filterList.append("Normals");
    filterList.append("Filter3");
    filterList.append("Filter3");

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
        filterVoxelGrid(inCloud, outCloud, d1);
        break;
    case 2:
        // CODE
        filterMedian(inCloud, outCloud, d1, d2);
        break;
    case 3:
        //CODE
        // Send new vis with normals filtered shit
        filterNormal(inCloud, d1, d2);
        break;
    default:
        ;
        // SOMETHING WENT WRONG
    }
}

void PointCloudManipulator::getNewIndexInfo(int selectedFilter)
{
    QList<QString> labels;
    labels.append("");
    labels.append("");
    labels.append("");
    QList<bool> show;
    show.append(false);
    show.append(false);
    show.append(false);
    show.append(false);
    QList<double> stepsAndRange;
    stepsAndRange.append(0.1);
    stepsAndRange.append(0.1);
    stepsAndRange.append(0.1);
    stepsAndRange.append(-5);
    stepsAndRange.append(5);
    stepsAndRange.append(-5);
    stepsAndRange.append(5);
    stepsAndRange.append(-5);
    stepsAndRange.append(5);

    switch(selectedFilter)
    {
    case 0:
        // PASSTHROUGH FILTER
        labels.replace(0, "Minimum:");
        labels.replace(1, "Maximum:");
        show.replace(0,true);
        show.replace(1,true);
        show.replace(3,true);
        Q_EMIT sendNewIndexInfo(labels, show, stepsAndRange);
        break;
    case 1:
        // VOXEL GRID FILTER
        labels.replace(0, "Leaf size:");
        show.replace(0, true);
        stepsAndRange.replace(0, 0.001);
        stepsAndRange.replace(3, 0.001);
        stepsAndRange.replace(4, 0.2);
        Q_EMIT sendNewIndexInfo(labels, show, stepsAndRange);
        break;
    case 2:
        // MEDIAN FILTER
        labels.replace(0, "Window size:");
        labels.replace(1, "Max movement:");
        show.replace(0, true);
        show.replace(1, true);
        stepsAndRange.replace(0, 1);
        stepsAndRange.replace(3, 0);
        stepsAndRange.replace(4, 100);
        stepsAndRange.replace(5, 0);
        stepsAndRange.replace(6, 10);
        Q_EMIT sendNewIndexInfo(labels, show, stepsAndRange);
        break;
    case 3:
        // NORMALS
        labels.replace(0, "Radius:");
        labels.replace(1, "Nr. Normals");
        show.replace(0, true);
        show.replace(1, true);
        stepsAndRange.replace(0, 0.001);
        stepsAndRange.replace(1, 1);
        stepsAndRange.replace(3, 0.001);
        stepsAndRange.replace(4, 0.5);
        stepsAndRange.replace(5, 1);
        stepsAndRange.replace(6, 10);
        Q_EMIT sendNewIndexInfo(labels, show, stepsAndRange);
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
    Q_EMIT sendNewPointCloud(outCloud);
}

void PointCloudManipulator::filterVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double leafSize)
{
    float leaf = leafSize;
    voxelGridFilter.setInputCloud(inCloud);
    voxelGridFilter.setLeafSize(leaf, leaf, leaf);
    voxelGridFilter.filter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud);
}

void PointCloudManipulator::filterMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double windowSize, double maxMovement)
{
    int windowSizeTmp = (int)windowSize;
    medianFilter.setInputCloud(inCloud);
    medianFilter.setWindowSize(windowSizeTmp);
    medianFilter.setMaxAllowedMovement(maxMovement);
    medianFilter.applyFilter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud);
}

void PointCloudManipulator::filterNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double radius, double nrToDisplay)
{
    int tmpDisplay = (int) nrToDisplay;
    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch (radius);
    norm_est.setInputCloud (inCloud);
    norm_est.compute (*normals_out);
    visualizer->addPointCloud<pcl::PointXYZ> (inCloud, "filteredCloud");
    visualizer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (inCloud, normals_out, tmpDisplay, 0.05, "normals");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,0.0,1.0, "normals");
    Q_EMIT sendNewVisualizer(visualizer);
}

void PointCloudManipulator::getNewVisualizer(int selectedFilter)
{
    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    Q_EMIT sendNewVisualizer(visualizer);
}

}
