
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

// ISS
#include <pcl/keypoints/iss_3d.h>

// NARF
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/narf_descriptor.h>

// SIFT
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/pfh.h"
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// DUNNO
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>

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

    // TESTING NEW SHIT
    void keyPointsISS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void keyPointsNARF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // This is connected
    void tester2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn2);
    pcl::PointCloud<pcl::Normal>::Ptr computeSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius);

    /**
    // This is connected
    void tester(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsIn, pcl::PointCloud<pcl::PointXYZ>::Ptr pointsIn2);
    void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, float normalRadius, pcl::PointCloud<pcl::Normal>::Ptr normal);
    void computeKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn,float minScale, int nrOctaves, int nrScalesPerOctaves, float minContrast,
                          pcl::PointCloud<pcl::PointWithScale>::Ptr keyPoints);
    void computePFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                    pcl::PointCloud<pcl::Normal>::Ptr normals,
                    pcl::PointCloud<pcl::PointWithScale>::Ptr keyPoints, float featureRadius,
                    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors);
    void computeFeatureCorrespondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr source_descriptors,
                                       pcl::PointCloud<pcl::PFHSignature125>::Ptr target_descriptors,
                                       std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);
    void visualizeCorrespondences( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                   const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                   const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                   const std::vector<int> &correspondences,
                                   const std::vector<float> &correspondence_scores);
                                   */


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

    // TESTING NEW SHIT
    // This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
    // http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);




};

}


#endif
