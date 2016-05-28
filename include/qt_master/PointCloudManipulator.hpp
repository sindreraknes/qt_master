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
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/narf_descriptor.h>
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

/*!
 * \brief The PointCloudManipulator class contains a number of filtering methods,
 * algorithms and matching methods.
 */
class PointCloudManipulator : public QObject
{
    Q_OBJECT
public:
    /*!
     * \brief Constructor for PointCloudManipulator class.
     * \param parent ui parent
     */
    explicit PointCloudManipulator(QObject *parent = 0);

    /*!
     * \brief Deconstructor for PointCloudManipulator class.
     */
    ~PointCloudManipulator();

    /*!
     * \brief Aquires names the available filters in the class.
     * \return a QStringList of available filters
     */
    QStringList getFilters();

    /*!
     * \brief Runs the selected filter on a point cloud.
     * \param selectedFilter the selected filter
     * \param inCloud input point cloud object
     * \param outCloud output point cloud object
     * \param d1 parameter 1
     * \param d2 parameter 2
     * \param d3 paramter 3
     * \param xyz parameter for passthrough filter
     */
    void runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double d1, double d2, double d3, QString xyz);

    /*!
     * \brief Creates the required indexes for the selected filter, and
     * sends them to the GUI. (label text, scaling etc)
     * \param selectedFilter the selected filter
     */
    void getNewIndexInfo(int selectedFilter);

    /*!
     * \brief Implementation of the PassThrough filter for a point cloud.
     * \param inCloud input point cloud object
     * \param outCloud output point cloud object
     * \param limitMin minimum limit (in meters)
     * \param limitMax maximum limit (in meters)
     * \param field the axis to be cut (x, y or z)
     */
    void filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double limitMin, double limitMax, QString field);

    /*!
     * \brief Implementation of the VoxelGrid filter for a point cloud.
     * \param inCloud input point cloud object
     * \param outCloud output point cloud object
     * \param leafSize the specified leaf size (volume) for the filter
     */
    void filterVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double leafSize);

    /*!
     * \brief Implementation of the median filter for a point cloud.
     * \param inCloud input point cloud object
     * \param outCloud output point cloud object
     * \param windowSize window size of the median
     * \param maxMovement maximum allowed movement for the median
     */
    void filterMedian(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double windowSize, double maxMovement);

    /*!
     * \brief Implementation of normal estimation for a point cloud.
     * \param inCloud input point cloud object
     * \param radius radius of search in normal estimation
     * \param nrToDisplay nr. of normals to return (for example 1/10)
     */
    void filterNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius, double nrToDisplay);

    /*!
     * \brief Implementation of the bilateral filter for a point cloud.
     * \param inCloud input point cloud object
     * \param sigmaS sigma S value in the filter
     * \param sigmaR sigma R value in the filter
     */
    void filterBilateral(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double sigmaS, float sigmaR);

    /*!
     * \brief Transforms a point cloud with given parameters.
     * \param inCloud input point cloud object to transform
     * \param rX rotation in x
     * \param rY rotation in y
     * \param rZ rotation in z
     * \param tX translation in x
     * \param tY translation in y
     * \param tZ translation in z
     */
    void translateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ);

    /*!
     * \brief Resets a visualizer and send it to the GUI.
     * \param selectedFilter selected filter to use
     */
    void getNewVisualizer(int selectedFilter);

    /*!
     * \brief Get the last used filter.
     * \return Returns a QString name of the last filter used
     */
    QString getLastFiltered();

    /*!
     * \brief Alignes point clouds using registration. Visualizes the process.
     * \param fileNames filenames of saved point clouds
     */
    void alignClouds(QStringList fileNames);

    /*!
     * \brief Alignes the robot cell at NTNU IPK using registration. Visualizes the process.
     * \param fileNames filenames of saved point clouds
     */
    void alignRobotCell(QStringList fileNames);

    /*!
     * \brief Refines the alignment of the robot cell at NTNU IPK using Iterative Closest Point.
     * \param fileNames filenames of saved point clouds
     */
    void refineAlignment(QStringList fileNames);

    /*!
     * \brief Implementation of the VoxelGrid filter for a point cloud.
     * \param inCloud input point cloud object
     * \param leafSize the specified leaf size (volume) for the filter
     * \return Returns a point cloud object that has been filtered
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double leafSize);

    /*!
     * \brief Implementation of the PassThrough filter for a point cloud.
     * \param inCloud input point cloud object
     * \param outCloud output point cloud object
     * \param limitMin minimum limit (in meters)
     * \param limitMax maximum limit (in meters)
     * \param field the axis to be cut (x, y or z)
     * \return Returns a point cloud object that has been filtered
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double limitMin, double limitMax, QString field);

    /*!
     * \brief Implementation of the ShadowPoint removal filter for a point cloud.
     * \param inCloud input point cloud
     * \param normals normals of the point cloud
     * \param threshold threshold to remove points
     * \return Returns a point cloud object that has been filtered
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterShadowPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                             pcl::PointCloud<pcl::Normal>::Ptr normals, double threshold);

    /*!
     * \brief Implementation of the Outlier removal filter for a point cloud.
     * \param inCloud input point cloud
     * \return Returns a point cloud object that has been filtered
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);

    /*!
     * \brief Implementation of normal estimation for a point cloud.
     * \param inCloud input point cloud object
     * \param radius radius of search in normal estimation
     * \param nrToDisplay nr. of normals to return (for example 1/10)
     * \return Returns a point cloud normal object containing the normals
     */
    pcl::PointCloud<pcl::Normal>::Ptr computeSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius);

    /*!
     * \brief Implementation of surface point normal estimation for a point cloud
     * \param input input point cloud object
     * \param surface surface of point cloud object
     * \param radius radius of search in normal estimation
     * \return Returns a point normal object containing the normals
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr computeSurfacePointNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface, float radius);

    /*!
     * \brief Implementation of cluster extraction.
     * \param input input point cloud object
     * \param distance maximum distance allowed to be outside a cluster
     * \return Returns a list of point cloud clusters that are found
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double distance);

    /*!
     * \brief Implementation of SIFT keypoint detector.
     * \param points input point cloud object
     * \param minScale minimum scale in SIFT
     * \param nrOctaves number of octaves in SIFT
     * \param nrScalesPerOctave number of scales per octave in SIFT
     * \param minContrast minimum allowed contrast in SIFT
     * \return Returns a point cloud object containing the SIFT keypoints
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectSIFTKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, float minScale,
                                                           int nrOctaves, int nrScalesPerOctave, float minContrast);

    /*!
     * \brief Implementation of FPFH feature descriptor estimation.
     * \param points input point cloud object
     * \param normals normals of input point cloud object
     * \param keyPoints keypoints of point cloud object
     * \param featureRadius radius to search in FPFH estimation
     * \return Returns a FPFHSignature33 histogram of the estimated FPFH feature descriptors
     */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeLocalDescriptorsFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints, float featureRadius);

    /*!
     * \brief Implementation of the SHOTColor feature descriptor estimation.
     * \param points input point cloud object
     * \param normals normals of input point cloud object
     * \param keyPoints keypoints of point cloud object
     * \param featureRadius radius to search in SHOT estimation
     * \return Returns a SHOT1344 histogram of the estimated SHOTColor feature descriptors
     */
    pcl::PointCloud<pcl::SHOT1344>::Ptr computeLocalDescriptorsSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints,float featureRadius);

    /*!
     * \brief Implementation of estimating an initial alignment of FPFH feature descriptors
     * \param sourcePoints input point cloud of cloud 1
     * \param sourceDescriptors input FPFH descriptors for cloud 1
     * \param targetPoints input point cloud of cloud 2
     * \param targetDescriptors input FPFH descriptors for cloud 2
     * \param minSampleDistance minimum sample distance in alignment estimation
     * \param maxCorrespondenceDistance maximum correspondence distance in alignment estimation
     * \param nrIterations nr of iterations to run before "giving up"
     * \return Returns a 4x4 transformation matrix of the estimated transformation
     */
    Eigen::Matrix4f computeInitialAlignmentFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                            float minSampleDistance, float maxCorrespondenceDistance, int nrIterations);

    /*!
     * \brief Implementation of estimating an initial alignment of SHOTColor feature descriptors
     * \param sourcePoints input point cloud of cloud 1
     * \param sourceDescriptors input SHOTColor descriptors for cloud 1
     * \param targetPoints input point cloud of cloud 2
     * \param targetDescriptors input SHOTColor descriptors for cloud 2
     * \param minSampleDistance minimum sample distance in alignment estimation
     * \param maxCorrespondenceDistance maximum correspondence distance in alignment estimation
     * \param nrIterations nr of iterations to run before "giving up"
     * \return Returns a 4x4 transformation matrix of the estimated transformation
     * \return
     */
    Eigen::Matrix4f computeInitialAlignmentSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors,
                                            float minSampleDistance, float maxCorrespondenceDistance, int nrIterations);

    /*!
     * \brief Implementation of plane segmentation.
     * \param inCloud input point cloud object
     * \param radius radius to search
     * \return Returns the point cloud without the plane
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius);

    /*!
     * \brief Implementation of plane segmentation.
     * \param inCloud input point cloud object
     * \param radius radius to search
     * \return Returns the plane
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPlaneReturnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius);

    /*!
     * \brief The PointCloudFeatures struct, contains the total points,
     * normals, keypoints and feature descriptor for one point cloud. Reduces
     * the amount of total code for each operation.
     */
    struct PointCloudFeatures{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr localDescriptorsFPFH;
        pcl::PointCloud<pcl::SHOT1344>::Ptr localDescriptorsSHOTColor;
    };

    /*!
     * \brief Computes the selected feature descriptor and keypoints for a point cloud
     * \param points input point cloud object
     * \param keyPoints name of the keypoint detector (for example "SIFT")
     * \param descriptors name of the feature descriptor estimator (for example "FPFH")
     * \return
     */
    PointCloudFeatures computeFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, QString keyPoints, QString descriptors);

    /*!
     * \brief Estimate correspondences between two feature descriptor histograms (FPFH).
     * \param sourceDescriptors source cloud feature descriptors
     * \param targetDescriptors target cloud feature descriptor
     * \param correspondencesOut output correspondences
     * \param correspondenceScoresOut output correspondences score
     */
    void findFeatureCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                    std::vector<int> &correspondencesOut, std::vector<float> &correspondenceScoresOut);

    /*!
     * \brief Visualizes the correspondences between two point clouds.
     * \param points1 input point cloud 1
     * \param keyPoints1 input keypoints from point cloud 1
     * \param points2 input point cloud 2
     * \param keyPoints2 input keypoints from point cloud 2
     * \param correspondences correspondences between feature descriptors
     * \param correspondenceScores correspondence scores between feature descriptors
     * \param maxToDisplay maximum correspondences to display
     */
    void visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints1,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints2,
                                  std::vector<int> &correspondences, std::vector<float> &correspondenceScores, int maxToDisplay);

    /*!
     * \brief Estimates correspondences between FPFH feature descriptors.
     * \param sourceDescriptors source FPFH feature descriptors
     * \param targetDescriptors target FPFH feature descriptors
     * \return Returns a Correspondences object with the correspondences found.
     */
    pcl::CorrespondencesPtr findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors);

    /*!
     * \brief Estimates correspondences between SHOT feature descriptors.
     * \param sourceDescriptors source SHOT feature descriptors
     * \param targetDescriptors target SHOT feature descriptors
     * \return Returns a Correspondences object with the correspondences found.
     */
    pcl::CorrespondencesPtr findCorrespondencesSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors);

    /*!
     * \brief Implementation of a correspondence rejector based on distance.
     * \param correspondences input correspondences to reject
     * \param sourceKeyPoints source cloud keypoints
     * \param targetKeyPoints target cloud keypoints
     * \param maximumDistance maximum allowed distance before rejection
     * \return Returns a Correspondences object with the good correspondences.
     */
    pcl::CorrespondencesPtr rejectCorrespondencesDistance(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints, float maximumDistance);
    /*!
     * \brief Implementation of a correspondence rejector based on RANSAC.
     * \param correspondences input correspondences to reject
     * \param sourceKeyPoints source cloud keypoints
     * \param targetKeyPoints target cloud keypoints
     * \param inlierTreshold treshold for inliers
     * \param maxIterations maximum allowed iterations
     * \return Returns a Correspondences object with the good correspondences.
     */
    pcl::CorrespondencesPtr rejectCorrespondencesSampleConsensus(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints, float inlierTreshold, int maxIterations);
    /*!
     * \brief Implementation of a correspondence rejector based on one-to-one.
     * \param correspondences input correspondences to reject
     * \return Returns a Correspondences object with the good correspondences.
     */
    pcl::CorrespondencesPtr rejectCorrespondencesOneToOne(pcl::CorrespondencesPtr correspondences);

    /*!
     * \brief Implementation of a correspondence rejector based on median distance.
     * \param correspondences input correspondences to reject
     * \param meanDistance mean distance to reject
     * \return Returns a Correspondences object with the good correspondences.
     */
    pcl::CorrespondencesPtr rejectCorrespondencesMedianDistance(pcl::CorrespondencesPtr correspondences, double meanDistance);

    /*!
     * \brief Visualizes the correspondences between two point clouds.
     * \param sourcePoints input source point cloud
     * \param targetPoints input target point cloud
     * \param sourceKeyPoints input source keypoints on point cloud
     * \param targetKeyPoints input target keypoints on point cloud
     * \param correspondences total correspondences
     * \param goodCorrespondences good correspondences after rejection
     */
    void visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints,
                                  pcl::CorrespondencesPtr correspondences, pcl::CorrespondencesPtr goodCorrespondences);

    /*!
     * \brief Implementation of transformation estimation based on Singular Value Decomposition.
     * \param sourcePoints input source point cloud
     * \param targetPoints input target point cloud
     * \param correspondences correspondences between the two point clouds
     * \return Returns a 4x4 transformation matrix with the estimated transformation
     */
    Eigen::Matrix4f estimateTransformationSVD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                              pcl::CorrespondencesPtr correspondences);

    /*!
     * \brief Implementation of transformation estimation based on Levenberg–Marquardt.
     * \param sourcePoints input source point cloud
     * \param targetPoints input target point cloud
     * \param correspondences correspondences between the two point clouds
     * \return Returns a 4x4 transformation matrix with the estimated transformation
     */
    Eigen::Matrix4f estimateTransformationLM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                             pcl::CorrespondencesPtr correspondences);

    /*!
     * \brief Visualizes a transformation between two point clouds.
     * \param sourcePoints input source point cloud
     * \param targetPoints input target point cloud
     * \param transform a 4x4 transformation matrix
     */
    void visualizeTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                 Eigen::Matrix4f transform);

    /*!
     * \brief Samples a .STL file to a point cloud.
     * \param path path to point cloud file
     * \param resolution resolution of sampling
     * \param tess_level tessalation level of sampling
     * \return Returns a point cloud object of the sampled .STL file
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampleSTL(QString path, int resolution, int tess_level);

    /*!
     * \brief Performs object recognition between a model and a scene.
     * \param model the model point cloud
     * \param scene the scene point cloud
     */
    void matchModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);

    /*!
     * \brief Automatic object recognition by subscribing to the point cloud topics.
     * \param clouds input point clouds, scene and model
     */
    void alignAndMatch(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

    /*!
     * \brief Refines the alignment of 3 point clouds in the robot cell.
     * \param cloudsIn list of 3 point clouds
     * \return Returns the reconstructed point cloud scene
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignCloudsRefined(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudsIn);


Q_SIGNALS:
    /*!
     * \brief Signal to send new information to the GUI
     * \param labels label text for the GUI
     * \param show which labels are showing in the GUI
     * \param stepsAndRange step and range for input box/slider
     */
    void sendNewIndexInfo(QStringList labels, QList<bool> show, QList<double> stepsAndRange);

    /*!
     * \brief Signal to send a new visualizer to the GUI.
     * \param vis the visualizer to send
     */
    void sendNewVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

    /*!
     * \brief Signal to send a new point cloud to the GUI.
     * \param cloud the cloud to send
     * \param name name of the cloud in the visualizer (needs to be unique)
     */
    void sendNewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString name);

public Q_SLOTS:

private:
    QStringList filterList; //!< List containing available filters
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter; //!< PassThrough filter object
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilterRGB; //!< PassThrough filter object
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilter; //!< VoxelGrid filter object
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilterRGB; //!< VoxelGrid filter object
    pcl::ShadowPoints<pcl::PointXYZRGB, pcl::Normal> shadowPointsFilter; //!< ShadowPoint removal filter object
    pcl::MedianFilter<pcl::PointXYZRGB> medianFilter; //!< Median filter object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statOutlierFilter; //!< Outlier removal filter object
    pcl::FastBilateralFilter<pcl::PointXYZRGB> bilateralFilter; //!< Bilateral filter object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer; //!< Visualizer object
    QString lastFiltered; //!< Last used filter
};

}

#endif
