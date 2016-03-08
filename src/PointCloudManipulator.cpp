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
    filterList.append("Translate");
    filterList.append("Bilateral");

    return filterList;

}

void PointCloudManipulator::runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double d1, double d2, double d3, QString xyz)
{
    switch(selectedFilter)
    {
    case 0:
        // PASSTHROUGH FILTER
        filterPassThrough(inCloud, outCloud, d1, d2, xyz);
        lastFiltered = "Passthrough filter, ";
        lastFiltered.append(" Min: ");
        lastFiltered.append(QString::number(d1));
        lastFiltered.append(" Max: ");
        lastFiltered.append(QString::number(d2));
        lastFiltered.append(" Field: ");
        lastFiltered.append(xyz);
        break;
    case 1:
        // VOXEL GRID FILTER
        filterVoxelGrid(inCloud, outCloud, d1);
        lastFiltered = "VoxelGrid filter, ";
        lastFiltered.append(" Leaf size : ");
        lastFiltered.append(QString::number(d1));
        break;
    case 2:
        // MEDIAN FILTER
        filterMedian(inCloud, outCloud, d1, d2);
        lastFiltered = "Median filter, ";
        lastFiltered.append(" Window size: ");
        lastFiltered.append(QString::number(d1));
        lastFiltered.append(" Max allowed movement: ");
        lastFiltered.append(QString::number(d2));
        break;
    case 3:
        // NEW VISUALIZER
        // Send new vis with normals filtered shit
        filterNormal(inCloud, d1, d2);
        lastFiltered = "Normals filter, ";
        lastFiltered.append(" Radius: ");
        lastFiltered.append(QString::number(d1));
        lastFiltered.append(" Nr. to display: ");
        lastFiltered.append(QString::number(d2));
        break;
    case 4:
        // TRANSLATION MATRIX
        //
        //translateCloud(inCloud, d1, d2, d3);
    case 5:
        filterBilateral(inCloud, d1, d2);
        lastFiltered = "Bilateral filter, ";
        lastFiltered.append(" SigmaS: ");
        lastFiltered.append(QString::number(d1));
        lastFiltered.append(" SigmaR: ");
        lastFiltered.append(QString::number(d2));
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
        break;
    case 4:
        // TRANSLATION
        labels.replace(0, "Transl. X: ");
        labels.replace(1, "Transl. Z: ");
        labels.replace(2, "Rotation Y: ");
        show.replace(0, true);
        show.replace(1, true);
        show.replace(2, true);
        stepsAndRange.replace(2, 1.0);
        stepsAndRange.replace(7, 0.0);
        stepsAndRange.replace(8, 360.0);

        Q_EMIT sendNewIndexInfo(labels, show, stepsAndRange);
        break;
    case 5:
        labels.replace(0, "Sigma S");
        labels.replace(1, "Sigma R");
        show.replace(0, true);
        show.replace(1,true);
        stepsAndRange.replace(0, 1);
        stepsAndRange.replace(1, 0.001);
        stepsAndRange.replace(3, 0);
        stepsAndRange.replace(4, 10);
        stepsAndRange.replace(5, 0);
        stepsAndRange.replace(6, 0.1);

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
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
}

void PointCloudManipulator::filterVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double leafSize)
{
    float leaf = leafSize;
    voxelGridFilter.setInputCloud(inCloud);
    voxelGridFilter.setLeafSize(leaf, leaf, leaf);
    voxelGridFilter.filter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
}

void PointCloudManipulator::filterMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double windowSize, double maxMovement)
{
    int windowSizeTmp = (int)windowSize;
    medianFilter.setInputCloud(inCloud);
    medianFilter.setWindowSize(windowSizeTmp);
    medianFilter.setMaxAllowedMovement(maxMovement);
    medianFilter.applyFilter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
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

void PointCloudManipulator::filterBilateral(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double sigmaS, float sigmaR)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    bilateralFilter.setInputCloud(inCloud);
    bilateralFilter.setSigmaS(sigmaS);
    bilateralFilter.setSigmaR(sigmaR);
    bilateralFilter.filter(*filteredCloud);
    Q_EMIT sendNewPointCloud(filteredCloud, "filteredCloud");
}

void PointCloudManipulator::translateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr translatedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    double x = M_PI/180*rX;
    double y = M_PI/180*rY;
    double z = M_PI/180*rZ;
    transform << cos(y) * cos(z), -cos(y) * sin(z), sin(y), tX,
            (cos(x) * sin(z)) + (cos(z) * sin(x) * sin(y)), (cos(x) * cos(z)) - (sin(x) * sin(y) * sin(z)), -cos(y) *
                                                                                                            sin(x), tY,
            (sin(x) * sin(z)) - (cos(x) * cos(z) * sin(y)), (cos(z) * sin(x)) + (cos(x) * sin(y) * sin(z)), cos(x) *
                                                                                                            cos(y), tZ,
            0.0, 0.0, 0.0, 1.0;

    pcl::transformPointCloud(*inCloud, *translatedCloud, transform);
    Q_EMIT sendNewPointCloud(translatedCloud, "translatedCloud");

}

void PointCloudManipulator::getNewVisualizer(int selectedFilter)
{
    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    Q_EMIT sendNewVisualizer(visualizer);
}

QString PointCloudManipulator::getLastFiltered()
{
    return lastFiltered;
}


pcl::PointCloud<pcl::Normal>::Ptr PointCloudManipulator::computeSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    return (normals);
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudManipulator::computeSurfacePointNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
                                                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface, float radius)
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    normal_estimation.setNumberOfThreads(8);
    normal_estimation.setSearchSurface(surface);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    return (normals);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::detectSIFTKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, float minScale,
                                                            int nrOctaves, int nrScalesPerOctave, float minContrast)
{
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> siftDetect;
    siftDetect.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    siftDetect.setScales(minScale,nrOctaves,nrScalesPerOctave);
    siftDetect.setMinimumContrast(minContrast);
    siftDetect.setInputCloud(points);
    pcl::PointCloud<pcl::PointWithScale> tmpKeyPoints;
    siftDetect.compute(tmpKeyPoints);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(tmpKeyPoints, *keyPoints);
    return keyPoints;

//    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeISS (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());


//    double iss_salient_radius_;
//    double iss_non_max_radius_;
//    double iss_normal_radius_;
//    double iss_border_radius_;
//    double iss_gamma_21_ (0.975);
//    double iss_gamma_32_ (0.975);
//    double iss_min_neighbors_ (5);
//    int iss_threads_ (4);

//    // Compute model_resolution
//    double model_resolution = 0.0;
//          int n_points = 0;
//          int nres;
//          std::vector<int> indices (2);
//          std::vector<float> sqr_distances (2);
//          pcl::search::KdTree<pcl::PointXYZRGB> tree;
//          tree.setInputCloud (points);

//          for (size_t i = 0; i < points->size (); ++i)
//          {
//            if (! pcl_isfinite ((*points)[i].x))
//            {
//              continue;
//            }
//            //Considering the second neighbor since the first is the point itself.
//            nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
//            if (nres == 2)
//            {
//              model_resolution += sqrt (sqr_distances[1]);
//              ++n_points;
//            }
//          }
//          if (n_points != 0)
//          {
//            model_resolution /= n_points;
//          }

//    iss_salient_radius_ = 6 * model_resolution;
//    iss_non_max_radius_ = 4 * model_resolution;
//    iss_normal_radius_ = 4 * model_resolution;
//    iss_border_radius_ = 1 * model_resolution;

//    iss_detector.setSearchMethod(treeISS);
//    iss_detector.setSalientRadius (iss_salient_radius_);
//    iss_detector.setNonMaxRadius (iss_non_max_radius_);

//    iss_detector.setNormalRadius (iss_normal_radius_);
//    iss_detector.setBorderRadius (iss_border_radius_);

//    iss_detector.setThreshold21 (iss_gamma_21_);
//    iss_detector.setThreshold32 (iss_gamma_32_);
//    iss_detector.setMinNeighbors (iss_min_neighbors_);
//    iss_detector.setNumberOfThreads (iss_threads_);
//    iss_detector.setInputCloud (points);
//    iss_detector.compute (*model_keypoints);

//    return (model_keypoints);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::filterVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double leafSize)
{
    float leaf = leafSize;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    voxelGridFilterRGB.setInputCloud(inCloud);
    voxelGridFilterRGB.setLeafSize(leaf, leaf, leaf);
    voxelGridFilterRGB.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::filterShadowPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                                              pcl::PointCloud<pcl::Normal>::Ptr normals, double threshold)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    shadowPointsFilter.setInputCloud(inCloud);
    shadowPointsFilter.setKeepOrganized(true);
    shadowPointsFilter.setNormals(normals);
    shadowPointsFilter.setThreshold(threshold);
    shadowPointsFilter.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::filterOutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());
    sor.setInputCloud (inCloud);
    sor.setMeanK (20);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double limitMin, double limitMax, QString field)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    passThroughFilterRGB.setInputCloud(inCloud);
    passThroughFilterRGB.setFilterLimits(limitMin, limitMax);
    passThroughFilterRGB.setFilterFieldName(field.toStdString());
    passThroughFilterRGB.setKeepOrganized(true);
    passThroughFilterRGB.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr PointCloudManipulator::computeLocalDescriptorsFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints, float featureRadius)
{
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
    fpfhEstimation.setNumberOfThreads(8);
    fpfhEstimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    fpfhEstimation.setRadiusSearch(featureRadius);
    fpfhEstimation.setSearchSurface(points);
    fpfhEstimation.setInputNormals(normals);
    fpfhEstimation.setInputCloud(keyPoints);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr localDescriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfhEstimation.compute(*localDescriptors);
    return localDescriptors;
}

pcl::PointCloud<pcl::SHOT1344>::Ptr PointCloudManipulator::computeLocalDescriptorsSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals ,
                                                                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints,float featureRadius)
{
    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot;
    pcl::PointCloud<pcl::SHOT1344>::Ptr localDescriptors (new pcl::PointCloud<pcl::SHOT1344>);
    shot.setNumberOfThreads(8);
    shot.setInputCloud(keyPoints);
    shot.setSearchSurface(points);
    shot.setInputNormals(normals);
    shot.setRadiusSearch(featureRadius);
    shot.compute(*localDescriptors);
    return (localDescriptors);
}

Eigen::Matrix4f PointCloudManipulator::computeInitialAlignmentFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors,
                                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                                               float minSampleDistance, float maxCorrespondenceDistance, int nrIterations)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sacInitAlign;
    sacInitAlign.setMinSampleDistance(minSampleDistance);
    sacInitAlign.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    sacInitAlign.setMaximumIterations(nrIterations);

    sacInitAlign.setInputSource(sourcePoints);
    sacInitAlign.setSourceFeatures(sourceDescriptors);
    sacInitAlign.setInputTarget(targetPoints);
    sacInitAlign.setTargetFeatures(targetDescriptors);

    pcl::PointCloud<pcl::PointXYZRGB> regOutput;
    sacInitAlign.align(regOutput);

    return (sacInitAlign.getFinalTransformation());
}

Eigen::Matrix4f PointCloudManipulator::computeInitialAlignmentSHOTColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors,
                                                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors,
                                                                        float minSampleDistance, float maxCorrespondenceDistance, int nrIterations)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::SHOT1344> sacInitAlign;
    sacInitAlign.setMinSampleDistance(minSampleDistance);
    sacInitAlign.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    sacInitAlign.setMaximumIterations(nrIterations);

    sacInitAlign.setInputSource(sourcePoints);
    sacInitAlign.setSourceFeatures(sourceDescriptors);
    sacInitAlign.setInputTarget(targetPoints);
    sacInitAlign.setTargetFeatures(targetDescriptors);

    pcl::PointCloud<pcl::PointXYZRGB> regOutput;
    sacInitAlign.align(regOutput);

    return (sacInitAlign.getFinalTransformation());
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (radius);
    seg.setInputCloud (inCloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (inCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    return (cloud_plane);
}

PointCloudManipulator::PointCloudFeatures PointCloudManipulator::computeFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, QString keyPoints, QString descriptors)
{
    PointCloudFeatures features;
    features.points = inCloud;

    // Normals
    features.normals = computeSurfaceNormals(features.points, 0.2);
    std::cout << "Found normals: " ;
    std::cout << features.normals->size() << std::endl;

    // Keypoints
    if(QString::compare(keyPoints, "SIFT", Qt::CaseInsensitive) == 0){
        // CONTRAST IS THE LAST PART (0.01, 3, 3, 0.2)
        features.keyPoints = detectSIFTKeyPoints(features.points, 0.01, 3, 3, 0.0);
        std::cout << "Found SIFT keypoints: " ;
        std::cout << features.keyPoints->size() << std::endl;
    }
    else if(QString::compare(keyPoints, "VOXEL", Qt::CaseInsensitive) == 0){
        features.keyPoints = filterVoxel(features.points, 0.08);
        std::cout << "Found VOXEL keypoints: " ;
        std::cout << features.keyPoints->size() << std::endl;
    }

    // Descriptors
    if(QString::compare(descriptors, "FPFH", Qt::CaseInsensitive) == 0){
        features.localDescriptorsFPFH = computeLocalDescriptorsFPFH(features.points, features.normals, features.keyPoints, 0.15);
        std::cout << "Found FPFH descriptors: " ;
        std::cout << features.localDescriptorsFPFH->size() << std::endl;
    }
    else if(QString::compare(descriptors, "SHOTCOLOR", Qt::CaseInsensitive) == 0){
        features.localDescriptorsSHOTColor = computeLocalDescriptorsSHOTColor(features.points, features.normals,features.keyPoints,0.15);
        std::cout << "Found SHOTColor descriptors: " ;
        std::cout << features.localDescriptorsSHOTColor->size() << std::endl;
    }




    return features;
}

void PointCloudManipulator::findFeatureCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors,
                                                       std::vector<int> &correspondencesOut, std::vector<float> &correspondenceScoresOut)
{
    correspondencesOut.resize(sourceDescriptors->size());
    correspondenceScoresOut.resize(sourceDescriptors->size());

    pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptorKDTree;
    descriptorKDTree.setInputCloud(targetDescriptors);

    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < sourceDescriptors->size(); i++){
        descriptorKDTree.nearestKSearch(*sourceDescriptors, i, k , k_indices, k_squared_distances);
        correspondencesOut[i] = k_indices[0];
        correspondenceScoresOut[i] = k_squared_distances[0];
    }
}

void PointCloudManipulator::visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints1,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoints2,
                                                     std::vector<int> &correspondences, std::vector<float> &correspondenceScores, int maxToDisplay)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keyPoints1, *keypoints_left, -translate, no_rotation);
    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keyPoints2, *keypoints_right, translate, no_rotation);

    // Add the clouds to the visualizer
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (points_left, "points_left");
    vis.addPointCloud (points_right, "points_right");

    // Compute the weakest correspondence score to display
    std::vector<float> temp (correspondenceScores);
    std::sort (temp.begin (), temp.end ());
    if (maxToDisplay >= temp.size ())
      maxToDisplay = temp.size () - 1;
    float threshold = temp[maxToDisplay];
    // Draw lines between the best corresponding points
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
      if (correspondenceScores[i] > threshold)
      {
        continue; // Don't draw weak correspondences
      }
      // Get the pair of points
      const pcl::PointXYZRGB & p_left = keypoints_left->points[i];
      const pcl::PointXYZRGB & p_right = keypoints_right->points[correspondences[i]];
      // Generate a random (bright) color
      double r = (rand() % 100);
      double g = (rand() % 100);
      double b = (rand() % 100);
      double max_channel = std::max (r, std::max (g, b));
      r /= max_channel;
      g /= max_channel;
      b /= max_channel;
      // Generate a unique string for each line
      std::stringstream ss ("line");
      ss << i;
      // Draw the line
      vis.addLine (p_left, p_right, r, g, b, ss.str ());
    }
    vis.resetCamera ();
    vis.spin ();
}

pcl::CorrespondencesPtr PointCloudManipulator::findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceDescriptors, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetDescriptors)
{
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(sourceDescriptors);
    est.setInputTarget(targetDescriptors);
    est.determineCorrespondences(*correspondences);
    return correspondences;
}

pcl::CorrespondencesPtr PointCloudManipulator::rejectCorrespondencesDistance(pcl::CorrespondencesPtr correspondences, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints, float maximumDistance)
{
    pcl::CorrespondencesPtr goodCorrespondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorDistance rej;
    rej.setInputSource<pcl::PointXYZRGB> (sourceKeyPoints);
    rej.setInputTarget<pcl::PointXYZRGB> (targetKeyPoints);
    rej.setMaximumDistance(maximumDistance); //meters
    rej.setInputCorrespondences(correspondences);
    rej.getCorrespondences(*goodCorrespondences);
    return goodCorrespondences;
}

pcl::CorrespondencesPtr PointCloudManipulator::rejectCorrespondencesSampleConsensus(pcl::CorrespondencesPtr correspondences,
                                                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints,
                                                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints,
                                                                                    float inlierTreshold, int maxIterations)
{
    pcl::CorrespondencesPtr goodCorrespondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rej;
    rej.setInputSource(sourceKeyPoints);
    rej.setInputTarget(targetKeyPoints);
    rej.setInlierThreshold(inlierTreshold);
    rej.setMaximumIterations(maxIterations);
    rej.setInputCorrespondences(correspondences);
    rej.getCorrespondences(*goodCorrespondences);
    return goodCorrespondences;

}

pcl::CorrespondencesPtr PointCloudManipulator::rejectCorrespondencesOneToOne(pcl::CorrespondencesPtr correspondences)
{
    pcl::CorrespondencesPtr goodCorrespondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorOneToOne rej;
    rej.setInputCorrespondences(correspondences);
    rej.getCorrespondences(*goodCorrespondences);
    return goodCorrespondences;
}

pcl::CorrespondencesPtr PointCloudManipulator::rejectCorrespondencesMedianDistance(pcl::CorrespondencesPtr correspondences, double meanDistance)
{
    pcl::CorrespondencesPtr corrRejectMed (new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorMedianDistance rejMed;
    rejMed.setInputCorrespondences(correspondences);
    rejMed.setMedianFactor(meanDistance);
    rejMed.getCorrespondences(*corrRejectMed);
    return corrRejectMed;
}

void PointCloudManipulator::visualizeCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceKeyPoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeyPoints,
                                                     pcl::CorrespondencesPtr correspondences, pcl::CorrespondencesPtr goodCorrespondences)
{
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr leftKey (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*sourcePoints, *left, -translate, no_rotation);
    pcl::transformPointCloud (*sourceKeyPoints, *leftKey, -translate, no_rotation);

    pcl::visualization::PCLVisualizer vis;
    int c = 0;
    int d = 1;
    vis.createViewPort(0, 0, 0.5, 1, c);
    vis.createViewPort(0.5, 0, 1, 1, d);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (left, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (targetPoints, 0, 0, 255);
    vis.addPointCloud(left, red, "cloud1",c);
    vis.addPointCloud(targetPoints, blue, "cloud2",c);
    vis.addCorrespondences<pcl::PointXYZRGB>(leftKey,targetKeyPoints,*correspondences,"Correspondences",c);
    vis.addPointCloud(left, red, "cloud1b",d);
    vis.addPointCloud(targetPoints, blue, "cloud2b", d);
    vis.addCorrespondences<pcl::PointXYZRGB>(leftKey,targetKeyPoints,*goodCorrespondences, "Good correspondences",d);
    vis.spin();

}

Eigen::Matrix4f PointCloudManipulator::estimateTransformationSVD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::CorrespondencesPtr correspondences)
{
    Eigen::Matrix4f transResult = Eigen::Matrix4f::Identity ();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> estTransSVD;
    estTransSVD.estimateRigidTransformation(*sourcePoints, *targetPoints, *correspondences, transResult);
    return transResult;
}

Eigen::Matrix4f PointCloudManipulator::estimateTransformationLM(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints, pcl::CorrespondencesPtr correspondences)
{
    Eigen::Matrix4f transResult = Eigen::Matrix4f::Identity ();
    pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB> estTransLM;
    estTransLM.estimateRigidTransformation(*sourcePoints, *targetPoints, *correspondences, transResult);
    return transResult;
}

void PointCloudManipulator::visualizeTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePoints, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPoints,
                                                    Eigen::Matrix4f transform)
{
    pcl::visualization::PCLVisualizer vis;
    int a = 0;
    int b = 1;
    vis.createViewPort(0, 0, 0.5, 1, a);
    vis.createViewPort(0.5, 0, 1, 1, b);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (sourcePoints, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (targetPoints, 0, 0, 255);
    vis.addPointCloud(sourcePoints, red, "source",a);
    vis.addPointCloud(targetPoints, blue, "target",a);

    vis.addPointCloud(sourcePoints, red, "source2",b);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*targetPoints, *tmp2, transform);
    vis.addPointCloud(tmp2, blue, "target2", b);
    vis.spin();
}


void PointCloudManipulator::tester2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1(new pcl::PointCloud<pcl::PointXYZRGB>);
    //Convert to XYZRGB point
    pcl::copyPointCloud(*cloudIn1,*points1);
    for (int i = 0; i< points1->points.size(); i++){
        points1->points[i].r = 255;
        points1->points[i].g = 255;
        points1->points[i].b = 255;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2(new pcl::PointCloud<pcl::PointXYZRGB>);
    //Convert to XYZRGB point
    pcl::copyPointCloud(*cloudIn2,*points2);
    for (int i = 0; i< points2->points.size(); i++){
        points2->points[i].r = 100;
        points2->points[i].g = 100;
        points2->points[i].b = 100;
    }

    PointCloudFeatures features1 = computeFeatures(points1, "SIFT", "FPFH");
    PointCloudFeatures features2 = computeFeatures(points2, "SIFT", "FPFH");
//    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity ();
//    tform = computeInitialAlignmentFPFH(features1.keyPoints, features1.localDescriptors, features2.keyPoints, features2.localDescriptors, 0.025, 0.01, 500);
//    std::cout << tform << std::endl;

//    pcl::visualization::PCLVisualizer vis;
//    int a = 0;
//    int b = 1;
//    vis.createViewPort(0, 0, 0.5, 1, a);
//    vis.createViewPort(0.5, 0, 1, 1, b);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (features2.points, 255, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (features1.points, 0, 0, 255);
//    vis.addPointCloud(features2.points, red, "cloud1",a);
//    vis.addPointCloud(features1.points, blue, "cloud2",a);

//    vis.addPointCloud(features2.points, red, "cloud1b",b);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::transformPointCloud(*features1.points, *tmp2, tform);
//    vis.addPointCloud(tmp2, blue, "cloud2b", b);
//    vis.spin();


    // CORRESPONDENCE USING VECTOR AND WEIRD KDTREE METHOD
//    std::vector<int> correspondences;
//    std::vector<float> correspondenceScores;
//    findFeatureCorrespondences(features1.localDescriptors, features2.localDescriptors, correspondences, correspondenceScores);
//    visualizeCorrespondences(features1.points,features1.keyPoints,features2.points,features2.keyPoints,correspondences,correspondenceScores,features1.keyPoints->size());
//    std::cout << "Vector<int> correspondences ALL: ";
//    std::cout << correspondences.size() << std::endl;

    // CORRESPONDENCE USING CORRESPONDENCEESTIMATION
    pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences);
    all_correspondences = findCorrespondences(features1.localDescriptorsFPFH, features2.localDescriptorsFPFH);
    std::cout << "CorrespondenceEstimation correspondences ALL: ";
    std::cout << all_correspondences->size() << std::endl;

    // CORRESPONDENCE REJECTION USING DISTANCE
//    pcl::CorrespondencesPtr corrRejectDistance (new pcl::Correspondences);
//    corrRejectDistance = rejectCorrespondencesDistance(all_correspondences, features1.keyPoints, features2.keyPoints, 0.5);
//    std::cout << "Rejected using distance, new amount is : ";
//    std::cout << corrRejectDistance->size() << std::endl;
//    visualizeCorrespondences(features1.points, features2.points,features1.keyPoints,features2.keyPoints,all_correspondences,corrRejectDistance);

    // CORRESPONDENCE REJECTION USING SAMPLE CONSENSUS
    pcl::CorrespondencesPtr corrRejectSampleConsensus (new pcl::Correspondences);
    corrRejectSampleConsensus = rejectCorrespondencesSampleConsensus(all_correspondences,features1.keyPoints,features2.keyPoints,0.25,1000);
    std::cout << "Rejected using sample consensus, new amount is : ";
    std::cout << corrRejectSampleConsensus->size() << std::endl;
    visualizeCorrespondences(features1.points, features2.points, features1.keyPoints, features2.keyPoints, all_correspondences, corrRejectSampleConsensus);

    // CORRESPONDENCE REJECTION USING ONE TO ONE
//    pcl::CorrespondencesPtr corrRejectOneToOne (new pcl::Correspondences);
//    corrRejectOneToOne = rejectCorrespondencesOneToOne(all_correspondences);
//    std::cout << "Rejected using one to one, new amount is : ";
//    std::cout << corrRejectOneToOne->size() << std::endl;
//    visualizeCorrespondences(features1.points, features2.points,features1.keyPoints,features2.keyPoints, all_correspondences, corrRejectOneToOne);

    // CORRESPONDENCE REJECTION USING ORGANIZED BOUNDARY
    // pcl::registration::CorrespondenceRejectionOrganizedBoundary rejOrg;

    // CORRESPONDENCE REJECTION USING MEDIAN DISTANCE
//    pcl::CorrespondencesPtr corrRejectMed (new pcl::Correspondences);
//    corrRejectMed = rejectCorrespondencesMedianDistance(all_correspondences, 0.5);
//    std::cout << "Rejected using median distance, new amount is : ";
//    std::cout << corrRejectMed->size() << std::endl;
//    visualizeCorrespondences(features1.points, features2.points, features1.keyPoints, features2.keyPoints, all_correspondences, corrRejectMed);

    Eigen::Matrix4f transSVD = Eigen::Matrix4f::Identity ();
    transSVD = estimateTransformationSVD(features1.keyPoints, features2.keyPoints, corrRejectSampleConsensus);
    std::cout << transSVD << std::endl;
    visualizeTransformation(features2.points, features1.points, transSVD);

    Eigen::Matrix4f transLM = Eigen::Matrix4f::Identity();
    transLM = estimateTransformationLM(features1.keyPoints, features2.keyPoints, corrRejectSampleConsensus);
    std::cout << transLM << std::endl;
    visualizeTransformation(features2.points, features1.points, transLM);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.001);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.0001);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp1 (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*features1.points,*tmp1,transSVD);
    icp.setInputSource(tmp1);
    icp.setInputTarget(features2.points);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());


    pcl::visualization::PCLVisualizer vis;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (tmp1, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (outCloud, 0, 0, 255);
    vis.addPointCloud(tmp1, red, "tmp1");
    vis.addPointCloud(outCloud, blue, "bluu");
    icp.align(*outCloud);

    while(!icp.hasConverged()){
        vis.updatePointCloud(tmp1, red, "tmp1");
        vis.updatePointCloud(outCloud, blue, "bluu");
        vis.spinOnce();
    }

    vis.updatePointCloud(tmp1, red, "tmp1");
    vis.updatePointCloud(outCloud, blue, "bluu");
    vis.spin();




}

void PointCloudManipulator::alignClouds(QStringList fileNames)
{
    std::vector<PointCloudFeatures> pointClouds;

    for(int i = 0; i<fileNames.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << fileNames.at(i).toStdString() << std::endl;
        pcl::io::loadPCDFile(fileNames.at(i).toUtf8().constData(), *tmpCloud);
        // BIG CLOUDS
//        tmpCloud = filterPassThrough(tmpCloud, 0.0, 6.5, "z");
//        tmpCloud = filterPassThrough(tmpCloud, -1.0, 2.7, "y");
        // TABLE
        tmpCloud = filterPassThrough(tmpCloud, 0.0, 2.3, "z");
        tmpCloud = filterPassThrough(tmpCloud, -0.5, 1.0, "y");
        tmpCloud = filterPassThrough(tmpCloud, -1.0, 1.0, "x");
        tmpCloud = filterVoxel(tmpCloud, 0.01);
//        for (int i = 0; i< tmpCloud->points.size(); i++){
//            tmpCloud->points[i].r = (i+1)*30;
//            tmpCloud->points[i].g = (i+1)*30;
//            tmpCloud->points[i].b = (i+1)*30;
//        }
        PointCloudFeatures tmpFeature = computeFeatures(tmpCloud, "SIFT", "FPFH");
        pointClouds.push_back(tmpFeature);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpAligned (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    *tmpAligned = *pointClouds.at(0).points;
    *icpCloud = *pointClouds.at(0).points;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.01);
    icp.setMaximumIterations(10000);
    //icp.setRANSACOutlierRejectionThreshold(0.1);
    icp.setRANSACIterations(0);
    icp.setTransformationEpsilon(1e-8 );
    icp.setEuclideanFitnessEpsilon(0.00001);

    Eigen::Matrix4f prevTrans = Eigen::Matrix4f::Identity();

    // TEST
//    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr icpNormCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//    *icpNormCloud = *pointClouds.at(0).pointsNormals;
//    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icpNorm;
//    icpNorm.setMaximumIterations(100);
//    icpNorm.setMaxCorrespondenceDistance(0.01);
//    icpNorm.setTransformationEpsilon(1e-8);
//    icpNorm.setEuclideanFitnessEpsilon(0.00001);


    for(int k = 0; k<pointClouds.size()-1; k++){
        pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences);
        all_correspondences = findCorrespondences(pointClouds.at(k).localDescriptorsFPFH, pointClouds.at(k+1).localDescriptorsFPFH);
        std::cout << "CorrespondenceEstimation correspondences ALL: ";
        std::cout << all_correspondences->size() << std::endl;

        pcl::CorrespondencesPtr corrRejectSampleConsensus (new pcl::Correspondences);
        corrRejectSampleConsensus = rejectCorrespondencesSampleConsensus(all_correspondences,pointClouds.at(k).keyPoints,pointClouds.at(k+1).keyPoints,0.25,1000);
        std::cout << "Rejected using sample consensus, new amount is : ";
        std::cout << corrRejectSampleConsensus->size() << std::endl;
        visualizeCorrespondences(pointClouds.at(k).points, pointClouds.at(k+1).points, pointClouds.at(k).keyPoints, pointClouds.at(k+1).keyPoints, all_correspondences, corrRejectSampleConsensus);

        Eigen::Matrix4f transSVD = Eigen::Matrix4f::Identity ();
        transSVD = estimateTransformationSVD(pointClouds.at(k).keyPoints, pointClouds.at(k+1).keyPoints, corrRejectSampleConsensus);
        std::cout << transSVD << std::endl;
        //visualizeTransformation(pointClouds.at(k+1).points, pointClouds.at(k).points, transSVD);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f trans = prevTrans*transSVD.inverse();
        pcl::transformPointCloud(*pointClouds.at(k+1).points,*tmp,trans);

        // ICP
        icp.setInputSource(tmp);
        icp.setInputTarget(pointClouds.at(k).points);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*outCloud);

        if(icp.hasConverged()){
            *icpCloud = *icpCloud + *outCloud;
            std::cout << "Converged! ";
            std::cout << k << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
        }


//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmpNorm (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//        pcl::transformPointCloudWithNormals(*pointClouds.at(k+1).pointsNormals, *tmpNorm, trans);
//        icpNorm.setInputSource(tmpNorm);
//        icpNorm.setInputTarget(pointClouds.at(k).pointsNormals);
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outCloudNorm (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//        icpNorm.align(*outCloudNorm);
//        if(icpNorm.hasConverged()){
//            *icpNormCloud = *icpNormCloud + *outCloudNorm;
//            std::cout << "Converged! ";
//            std::cout << k << std::endl;
//            std::cout << icpNorm.getFinalTransformation() << std::endl;
//        }

        *tmpAligned = *tmpAligned + *tmp;
        prevTrans = trans;
    }

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(tmpAligned, "aligned shit");
    vis.spin();

    pcl::visualization::PCLVisualizer vis2;
    vis2.addPointCloud(icpCloud, "icp shiiit shit");
    vis2.spin();

//    pcl::visualization::PCLVisualizer vis3;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
//    pcl::copyPointCloud(*icpNormCloud, *visCloud);
//    vis3.addPointCloud(visCloud);
//    vis3.spin();


}

void PointCloudManipulator::alignRobotCell(QStringList fileNames)
{
    std::vector<PointCloudFeatures> pointClouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> originalClouds;
    std::vector<Eigen::Matrix4f> cameraPositions;
    cameraPositions.push_back(Eigen::Matrix4f::Identity());


    for(int i = 0; i<fileNames.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << fileNames.at(i).toStdString() << std::endl;
        pcl::io::loadPCDFile(fileNames.at(i).toUtf8().constData(), *tmpCloud);
        originalClouds.push_back(tmpCloud);

//        switch(i){
//        case 0:
//            tmpCloud = filterPassThrough(tmpCloud, -1.3, 1.3, "x");
//            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.3, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 0.8, 2.1, "z");
//            break;
//        case 1:
//            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.6, "x");
//            tmpCloud = filterPassThrough(tmpCloud, -0.2, 0.5, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 1.1, 2.7, "z");
//            break;
//        case 2:
//            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.4, "x");
//            tmpCloud = filterPassThrough(tmpCloud, -0.6, 0.4, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 0.8, 4.0, "z");
//            break;
//        }


        // COLOR
//        switch(i){
//        case 0:
//            tmpCloud = filterPassThrough(tmpCloud, -0.8, 0.9, "x");
//            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.4, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 1.1, 1.8, "z");
//            break;
//        case 1:
//            tmpCloud = filterPassThrough(tmpCloud, -0.6, 0.4, "x");
//            tmpCloud = filterPassThrough(tmpCloud, -0.7, 0.4, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 0.7, 2.0, "z");
//            break;
//        case 2:
//            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.5, "x");
//            tmpCloud = filterPassThrough(tmpCloud, 0.0, 0.6, "y");
//            tmpCloud = filterPassThrough(tmpCloud, 1.1, 2.7, "z");
//            break;
//        }

        // ROBOT CELL 03.03
        switch(i){
        case 0:
            tmpCloud = filterPassThrough(tmpCloud, -0.8, 0.8, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.3, "y");
            tmpCloud = filterPassThrough(tmpCloud, 1.0, 1.7, "z");
            break;
        case 1:
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.5, "x");
            tmpCloud = filterPassThrough(tmpCloud, -1.0, 0.1, "y");
            tmpCloud = filterPassThrough(tmpCloud, 1.3, 2.6, "z");
            break;
        case 2:
            tmpCloud = filterPassThrough(tmpCloud, -0.6, 0.3, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.6, 0.4, "y");
            tmpCloud = filterPassThrough(tmpCloud, 1.2, 2.6, "z");
            break;
        }
        tmpCloud = filterVoxel(tmpCloud, 0.01);
        //tmpCloud = filterOutlier(tmpCloud);
        tmpCloud = extractPlane(tmpCloud, 0.1);

        PointCloudFeatures tmpFeature = computeFeatures(tmpCloud, "SIFT", "FPFH");
        //PointCloudFeatures tmpFeature = computeFeatures(tmpCloud, "VOXEL", "FPFH");
        //PointCloudFeatures tmpFeature = computeFeatures(tmpCloud, "SIFT", "SHOTCOLOR");
        pointClouds.push_back(tmpFeature);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpAligned (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalAligned (new pcl::PointCloud<pcl::PointXYZRGB>());
    *tmpAligned = *pointClouds.at(0).points;
    *icpCloud = *pointClouds.at(0).points;
    *originalAligned = *originalClouds.at(0);
    // ICP stuff
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.3);
    icp.setMaximumIterations(10000);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.0000001);


    for(int k = 0; k<pointClouds.size()-1; k++){
        // Initial aligment
        Eigen::Matrix4f initTrans = Eigen::Matrix4f::Identity ();
        Eigen::Matrix4f cameraPos = Eigen::Matrix4f::Identity();
        initTrans = computeInitialAlignmentFPFH(pointClouds.at(0).keyPoints,pointClouds.at(0).localDescriptorsFPFH,pointClouds.at(k+1).keyPoints, pointClouds.at(k+1).localDescriptorsFPFH,0.08,1.0,1000);
        //initTrans = computeInitialAlignmentSHOTColor(pointClouds.at(0).keyPoints,pointClouds.at(0).localDescriptorsSHOTColor,pointClouds.at(k+1).keyPoints, pointClouds.at(k+1).localDescriptorsSHOTColor,0.3,1.0,1000);
        std::cout << initTrans << std::endl;
        visualizeTransformation(pointClouds.at(k+1).points, pointClouds.at(0).points, initTrans);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f initTransInv = initTrans.inverse();
        pcl::transformPointCloud(*pointClouds.at(k+1).points,*tmp,initTransInv);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpk (new pcl::PointCloud<pcl::PointXYZRGB>());
        //pcl::transformPointCloud(*originalClouds.at(k+1),*tmpk,initTransInv);

        *tmpAligned = *tmpAligned + *tmp;

        cameraPos = initTrans.inverse();

        // ICP
        icp.setInputSource(tmp);
        icp.setInputTarget(pointClouds.at(0).points);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*outCloud);
        Eigen::Matrix4f transICP = Eigen::Matrix4f::Identity();
        if(icp.hasConverged()){
            *icpCloud = *icpCloud + *outCloud;
            std::cout << "Converged! ";
            std::cout << k << std::endl;
            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpOrig (new pcl::PointCloud<pcl::PointXYZRGB>());
            transICP = icp.getFinalTransformation();
            //pcl::transformPointCloud(*tmpk, *tmpOrig, transICP);
            //*originalAligned = *originalAligned + *tmpOrig;

            // Unsure about this one -__-
            //cameraPos = cameraPos*transICP;
            cameraPos = transICP*cameraPos;
            cameraPositions.push_back(cameraPos);
        }

    }

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(tmpAligned, "aligned shit");
    vis.spin();

    pcl::visualization::PCLVisualizer vis2;
    vis2.addPointCloud(icpCloud, "icp shit");
    vis2.spin();


    pcl::visualization::PCLVisualizer vis3;
    for(int i = 0; i<3; i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        QString s = "cloud";
        s.append(QString::number(i));
       // Eigen::Matrix4f tmp2 = Eigen::Matrix4f::Identity();
       // tmp2 = ;
        pcl::transformPointCloud(*originalClouds.at(i),*tmp,cameraPositions.at(i));
        std::cout << cameraPositions.at(i) << std::endl;
        vis3.addPointCloud(tmp,s.toStdString());
    }
    //vis3.addPointCloud(originalAligned, "orignial");
//    vis3.addCoordinateSystem(0.5);
//    Eigen::Affine3f A;
//    A = cameraPositions.at(1);
//    vis3.addCoordinateSystem(0.5, A);
//    std::cout << "Cam1: " << std::endl;
//    std::cout <<  cameraPositions.at(1) << std::endl;
//    A = cameraPositions.at(2);
//    std::cout << "Cam2: " << std::endl;
//    std::cout <<  cameraPositions.at(2) << std::endl;
//    vis3.addCoordinateSystem(0.5, A, 0);
    vis3.spin();
}

}
