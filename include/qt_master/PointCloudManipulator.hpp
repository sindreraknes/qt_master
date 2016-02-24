
#ifndef qt_master_POINT_CLOUD_MANIPULATOR_H
#define qt_master_POINT_CLOUD_MANIPULATOR_H

#include <QStringList>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/shadowpoints.h>
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
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/pfh.h"
#include "pcl/features/fpfh.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/keypoints/iss_3d.h"
#include <pcl/registration/transforms.h>

// DUNNO
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/features/shot.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/elch.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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
    void filterBilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double sigmaS, float sigmaR);
    void translateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ);
    void getNewVisualizer(int selectedFilter);
    QString getLastFiltered();

    // TESTING NEW SHIT
    void keyPointsISS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void keyPointsNARF(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    // This is connected
    void tester2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn2);
    void alignClouds(QStringList fileNames);
    void alignRobotCell(QStringList fileNames);
    pcl::PointCloud<pcl::Normal>::Ptr computeSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius);
    pcl::PointCloud<pcl::PointNormal>::Ptr computeSurfacePointNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface, float radius);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, float minScale,
                                                           int nrOctaves, int nrScalesPerOctave, float minContrast);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double leafSize);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterShadowPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                             pcl::PointCloud<pcl::Normal>::Ptr normals, double threshold);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double limitMin, double limitMax, QString field);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeLocalDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints, float featureRadius);
    Eigen::Matrix4f computeInitialAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                            float minSampleDistance, float maxCorrespondenceDistance, int nrIterations);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius);

    struct PointCloudFeatures{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr localDescriptors;
    };

    PointCloudFeatures computeFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points);

    void findFeatureCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                    std::vector<int> &correspondencesOut, std::vector<float> &correspondenceScoresOut);

    void visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints1,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints2,
                                  std::vector<int> &correspondences, std::vector<float> &correspondenceScores, int maxToDisplay);



    pcl::CorrespondencesPtr findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors);

    pcl::CorrespondencesPtr rejectCorrespondencesDistance(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints, float maximumDistance);

    pcl::CorrespondencesPtr rejectCorrespondencesSampleConsensus(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints, float inlierTreshold, int maxIterations);

    pcl::CorrespondencesPtr rejectCorrespondencesOneToOne(pcl::CorrespondencesPtr correspondences);

    pcl::CorrespondencesPtr rejectCorrespondencesMedianDistance(pcl::CorrespondencesPtr correspondences, double meanDistance);

    void visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints,
                                  pcl::CorrespondencesPtr correspondences, pcl::CorrespondencesPtr goodCorrespondences);

    Eigen::Matrix4f estimateTransformationSVD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                              pcl::CorrespondencesPtr correspondences);
    Eigen::Matrix4f estimateTransformationLM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                             pcl::CorrespondencesPtr correspondences);

    void visualizeTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                 Eigen::Matrix4f transform);

Q_SIGNALS:
    void sendNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);
    void sendNewVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
    void sendNewPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, QString name);

public Q_SLOTS:

private:
    QStringList filterList;
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilterRGB;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilterRGB;
    pcl::ShadowPoints<pcl::PointXYZRGB, pcl::Normal> shadowPointsFilter;
    pcl::MedianFilter<pcl::PointXYZ> medianFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierFilter;
    pcl::FastBilateralFilter<pcl::PointXYZ> bilateralFilter;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
    QString lastFiltered;

    // TESTING NEW SHIT
    // This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
    // http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);




};

}


#endif
