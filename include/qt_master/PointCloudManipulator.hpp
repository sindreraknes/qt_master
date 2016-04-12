
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
#include <pcl/features/shot_omp.h>
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
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

namespace qt_master {

class PointCloudManipulator : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudManipulator(QObject *parent = 0);
    ~PointCloudManipulator();
    // GUI FILTERS
    QStringList getFilters();
    void runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double d1, double d2, double d3, QString xyz);
    void getNewIndexInfo(int selectedFilter);
    void filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double limitMin, double limitMax, QString field);
    void filterVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double leafSize);
    void filterMedian(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double windowSize, double maxMovement);
    void filterNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius, double nrToDisplay);
    void filterBilateral(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double sigmaS, float sigmaR);
    void translateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ);
    void getNewVisualizer(int selectedFilter);
    QString getLastFiltered();

    // Registration tester
    void tester2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsIn2);
    void alignClouds(QStringList fileNames);
    void alignRobotCell(QStringList fileNames);

    void refineAlignment(QStringList fileNames);

    void roflKinect(QStringList fileNames);
    // FILTERS, KEYPOINTS, DESCRIPTORS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double leafSize);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double limitMin, double limitMax, QString field);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterShadowPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                             pcl::PointCloud<pcl::Normal>::Ptr normals, double threshold);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    pcl::PointCloud<pcl::Normal>::Ptr computeSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius);
    pcl::PointCloud<pcl::PointNormal>::Ptr computeSurfacePointNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface, float radius);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double distance);
    // KEYPOINTS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectSIFTKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, float minScale,
                                                           int nrOctaves, int nrScalesPerOctave, float minContrast);
    // DESCRIPTORS

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeLocalDescriptorsFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints, float featureRadius);
    pcl::PointCloud<pcl::SHOT1344>::Ptr computeLocalDescriptorsSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints,float featureRadius);

    // ALIGNMENT AND REGISTRATION
    Eigen::Matrix4f computeInitialAlignmentFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                            float minSampleDistance, float maxCorrespondenceDistance, int nrIterations);

    Eigen::Matrix4f computeInitialAlignmentSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors,
                                            float minSampleDistance, float maxCorrespondenceDistance, int nrIterations);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius);

    // Struct with different information of each point cloud
    struct PointCloudFeatures{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr localDescriptorsFPFH;
        pcl::PointCloud<pcl::SHOT1344>::Ptr localDescriptorsSHOTColor;
    };
    PointCloudFeatures computeFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, QString keyPoints, QString descriptors);


    // CORRESPONDENCES
    void findFeatureCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                    std::vector<int> &correspondencesOut, std::vector<float> &correspondenceScoresOut);

    void visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints1,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints2,
                                  std::vector<int> &correspondences, std::vector<float> &correspondenceScores, int maxToDisplay);



    pcl::CorrespondencesPtr findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors);

    pcl::CorrespondencesPtr findCorrespondencesSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors);

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


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampleSTL(QString path, int resolution, int tess_level);
    void matchModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);


Q_SIGNALS:
    void sendNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);
    void sendNewVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
    void sendNewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

public Q_SLOTS:

private:
    QStringList filterList;
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilterRGB;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilter;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilterRGB;
    pcl::ShadowPoints<pcl::PointXYZRGB, pcl::Normal> shadowPointsFilter;
    pcl::MedianFilter<pcl::PointXYZRGB> medianFilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statOutlierFilter;
    pcl::FastBilateralFilter<pcl::PointXYZRGB> bilateralFilter;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
    QString lastFiltered;


};

}


#endif
