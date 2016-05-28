#include "../include/qt_master/PointCloudManipulator.hpp"


namespace qt_master {

PointCloudManipulator::PointCloudManipulator(QObject *parent) :
    QObject(parent)
{

}

PointCloudManipulator::~PointCloudManipulator(){}

QStringList PointCloudManipulator::getFilters()
{
    filterList.append("Passthrough");
    filterList.append("VoxelGrid");
    filterList.append("Median");
    filterList.append("Normals");
    filterList.append("Plane extraction");
    filterList.append("Bilateral");
    return filterList;
}

void PointCloudManipulator::runFilter(int selectedFilter,pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double d1, double d2, double d3, QString xyz)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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
        // Plane extraction
        cloud = extractPlane(inCloud, d1);
        Q_EMIT sendNewPointCloud(cloud, "filteredCloud");
        lastFiltered = "Plane extraction, ";
        lastFiltered.append(" Radius: ");
        lastFiltered.append(QString::number(d1));
        break;
    case 5:
        filterBilateral(inCloud, d1, d2);
        lastFiltered = "Bilateral filter, ";
        lastFiltered.append(" SigmaS: ");
        lastFiltered.append(QString::number(d1));
        lastFiltered.append(" SigmaR: ");
        lastFiltered.append(QString::number(d2));
        break;
    default:
        ;
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
        labels.replace(0, "Radius: ");
        show.replace(0, true);
        stepsAndRange.replace(0, 0.01);
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
    }
}

void PointCloudManipulator::filterPassThrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double limitMin, double limitMax, QString field)
{
    passThroughFilter.setInputCloud(inCloud);
    passThroughFilter.setFilterFieldName(field.toStdString());
    passThroughFilter.setFilterLimits(limitMin, limitMax);
    passThroughFilter.setKeepOrganized(true);
    passThroughFilter.filter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
}

void PointCloudManipulator::filterVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double leafSize)
{
    float leaf = leafSize;
    voxelGridFilter.setInputCloud(inCloud);
    voxelGridFilter.setLeafSize(leaf, leaf, leaf);
    voxelGridFilter.filter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
}

void PointCloudManipulator::filterMedian(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, double windowSize, double maxMovement)
{
    int windowSizeTmp = (int)windowSize;
    medianFilter.setInputCloud(inCloud);
    medianFilter.setWindowSize(windowSizeTmp);
    medianFilter.setMaxAllowedMovement(maxMovement);
    medianFilter.applyFilter(*outCloud);
    Q_EMIT sendNewPointCloud(outCloud, "filteredCloud");
}

void PointCloudManipulator::filterNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius, double nrToDisplay)
{
    int tmpDisplay = (int) nrToDisplay;
    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(10);
    norm_est.setInputCloud (inCloud);
    norm_est.compute (*normals_out);
    visualizer->addPointCloud<pcl::PointXYZRGB> (inCloud, "filteredCloud");
    visualizer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (inCloud, normals_out, tmpDisplay, 0.05, "normals");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,0.0,1.0, "normals");
    Q_EMIT sendNewVisualizer(visualizer);
}

void PointCloudManipulator::filterBilateral(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double sigmaS, float sigmaR)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    bilateralFilter.setInputCloud(inCloud);
    bilateralFilter.setSigmaS(sigmaS);
    bilateralFilter.setSigmaR(sigmaR);
    bilateralFilter.filter(*filteredCloud);
    Q_EMIT sendNewPointCloud(filteredCloud, "filteredCloud");
}

void PointCloudManipulator::translateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  inCloud, double rX, double rY, double rZ, double tX, double tY, double tZ)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr translatedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    normal_estimation.setNumberOfThreads(8);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setKSearch(10);
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

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudManipulator::extractClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double distance)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>), outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*input,*incloud);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (distance);
    int i=0, nr_points = (int) incloud->points.size ();

    while (incloud->points.size () > 0.3 * nr_points)
    {
        seg.setInputCloud (incloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (incloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *incloud = *cloud_f;
    }
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (incloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02);
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (incloud);
    ec.extract (cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    for(int i = 0; i< cluster_indices.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*incloud,cluster_indices[i],*tmpcloud);
        clusters.push_back(tmpcloud);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i< clusters.size(); i++){
        *clusterCloud += *clusters.at(i);
    }

    Q_EMIT sendNewPointCloud(clusterCloud, "filteredCloud");

    return (clusters);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold (radius);
    seg.setInputCloud (inCloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (inCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_no_plane);

    return (cloud_no_plane);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::extractPlaneReturnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, double radius)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold (radius);
    seg.setInputCloud (inCloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (inCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);

    return (cloud_plane);
}

PointCloudManipulator::PointCloudFeatures PointCloudManipulator::computeFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, QString keyPoints, QString descriptors)
{
    PointCloudFeatures features;
    features.points = inCloud;

    features.normals = computeSurfaceNormals(features.points, 0.005);
    std::cout << "Found normals: " ;
    std::cout << features.normals->size() << std::endl;

    if(QString::compare(keyPoints, "SIFT", Qt::CaseInsensitive) == 0){
        // CONTRAST IS THE LAST PART (0.01, 3, 3, 0.2)
        features.keyPoints = detectSIFTKeyPoints(features.points, 0.01, 15, 15, 0.0);
        std::cout << "Found SIFT keypoints: " ;
        std::cout << features.keyPoints->size() << std::endl;
    }
    else if(QString::compare(keyPoints, "VOXEL", Qt::CaseInsensitive) == 0){
        features.keyPoints = filterVoxel(features.points, 0.001);
        std::cout << "Found VOXEL keypoints: " ;
        std::cout << features.keyPoints->size() << std::endl;
    }

    if(QString::compare(descriptors, "FPFH", Qt::CaseInsensitive) == 0){
        features.localDescriptorsFPFH = computeLocalDescriptorsFPFH(features.points, features.normals, features.keyPoints, 0.55); //0.15
        std::cout << "Found FPFH descriptors: " ;
        std::cout << features.localDescriptorsFPFH->size() << std::endl;
    }
    else if(QString::compare(descriptors, "SHOTCOLOR", Qt::CaseInsensitive) == 0){
        features.localDescriptorsSHOTColor = computeLocalDescriptorsSHOTColor(features.points, features.normals,features.keyPoints,0.55);
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
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keyPoints1, *keypoints_left, -translate, no_rotation);
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keyPoints2, *keypoints_right, translate, no_rotation);

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (points_left, "points_left");
    vis.addPointCloud (points_right, "points_right");

    std::vector<float> temp (correspondenceScores);
    std::sort (temp.begin (), temp.end ());
    if (maxToDisplay >= temp.size ())
        maxToDisplay = temp.size () - 1;
    float threshold = temp[maxToDisplay];
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
        if (correspondenceScores[i] > threshold)
        {
            continue;
        }
        const pcl::PointXYZRGB & p_left = keypoints_left->points[i];
        const pcl::PointXYZRGB & p_right = keypoints_right->points[correspondences[i]];
        double r = (rand() % 100);
        double g = (rand() % 100);
        double b = (rand() % 100);
        double max_channel = std::max (r, std::max (g, b));
        r /= max_channel;
        g /= max_channel;
        b /= max_channel;
        std::stringstream ss ("line");
        ss << i;
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

pcl::CorrespondencesPtr PointCloudManipulator::findCorrespondencesSHOT(pcl::PointCloud<pcl::SHOT1344>::Ptr sourceDescriptors, pcl::PointCloud<pcl::SHOT1344>::Ptr targetDescriptors){
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::sampleSTL(QString path, int resolution, int tess_level)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(path.toStdString(), mesh);
    pcl::PointCloud<pcl::PointXYZ> scaled_mesh;
    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
    scaleMatrix(0,0)=0.001f;
    scaleMatrix(1,1)=0.001f;
    scaleMatrix(2,2)=0.001f;
    pcl::fromPCLPointCloud2(mesh.cloud,scaled_mesh);
    pcl::transformPointCloud(scaled_mesh,scaled_mesh,scaleMatrix);
    pcl::toPCLPointCloud2(scaled_mesh, mesh.cloud);
    vtkSmartPointer<vtkPolyData> meshVTK;
    pcl::VTKUtils::convertToVTK(mesh, meshVTK);

    pcl::visualization::PCLVisualizer generator("Generating traces...");
    generator.addModelFromPolyData (meshVTK, "mesh", 0);
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > clouds;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> enthropies;
    generator.renderViewTesselatedSphere(resolution, resolution, clouds, poses, enthropies, tess_level);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmprgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<clouds.size(); i++){
        Eigen::Matrix4f tmpPose;
        tmpPose = poses.at(i).inverse();
        pcl::transformPointCloud(clouds.at(i), *tmp, tmpPose);
        pcl::copyPointCloud(*tmp, *tmprgb);
        *outCloud += *tmprgb;
    }

    for (int i = 0; i< outCloud->points.size(); i++){
        outCloud->points[i].r = 255;
        outCloud->points[i].g = 255;
        outCloud->points[i].b = 255;
    }
    return outCloud;
}

void PointCloudManipulator::matchModelCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullScene(new pcl::PointCloud<pcl::PointXYZRGB>());
    *fullScene = *scene;

    Eigen::Matrix4f cameraToTag = Eigen::Matrix4f::Identity();
    cameraToTag << -0.0324064,   0.999472, 0.00236665,  -0.236723,
            0.701194,  0.0244224,  -0.712552,  -0.589319,
            -0.712233, -0.0214317,  -0.701615,    1.82195,
            0,          0,          0,          1;
    Eigen::Matrix4f worldToTag = Eigen::Matrix4f::Identity();
    worldToTag <<  1,  0, 0, -0.084,
            0,  1, 0, -0.292,
            0,  0, 1, 0.87,
            0,  0, 0, 1;

    scene = filterPassThrough(scene, -0.4, 0.4, "x");
    scene = filterPassThrough(scene, -0.6, -0.04, "y");
    scene = filterPassThrough(scene, 1.1, 1.8, "z");
    model = filterVoxel(model, 0.005);
    scene = filterVoxel(scene, 0.001);
    scene = extractPlane(scene,0.02);

    PointCloudFeatures modelFeature = computeFeatures(model, "SIFT", "SHOTCOLOR");
    PointCloudFeatures sceneFeature = computeFeatures(scene, "SIFT", "SHOTCOLOR");

    pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences);
    all_correspondences = findCorrespondencesSHOT(modelFeature.localDescriptorsSHOTColor, sceneFeature.localDescriptorsSHOTColor);
    std::cout << "CorrespondenceEstimation correspondences ALL: ";
    std::cout << all_correspondences->size() << std::endl;

    pcl::CorrespondencesPtr corrRejectSampleConsensus (new pcl::Correspondences);
    corrRejectSampleConsensus = rejectCorrespondencesSampleConsensus(all_correspondences,modelFeature.keyPoints,sceneFeature.keyPoints,0.03,1000); //Was 0.10, 0.07, 0.05
    std::cout << "Rejected using sample consensus, new amount is : ";
    std::cout << corrRejectSampleConsensus->size() << std::endl;

    visualizeCorrespondences(modelFeature.points, fullScene, modelFeature.keyPoints, sceneFeature.keyPoints, all_correspondences, corrRejectSampleConsensus);

    Eigen::Matrix4f transSVD = Eigen::Matrix4f::Identity ();
    transSVD = estimateTransformationSVD(modelFeature.keyPoints, sceneFeature.keyPoints, corrRejectSampleConsensus);
    std::cout << "Initial transformation CAMERA to OBJECT: " << std::endl;
    std::cout << transSVD << std::endl;
    visualizeTransformation(sceneFeature.points, modelFeature.points, transSVD);
    visualizeTransformation(fullScene, modelFeature.points, transSVD);

    Eigen::Affine3f A;
    A = transSVD;
    Eigen::Affine3f B2;
    B2 = cameraToTag;
    pcl::visualization::PCLVisualizer vis;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (modelFeature.points, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (fullScene, 0, 0, 255);
    vis.addPointCloud(fullScene, "fullScene");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model2 (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*modelFeature.points,*model2,transSVD);
    vis.addPointCloud(modelFeature.points,red, "model");
    vis.addPointCloud(model2,red,"model2");
    vis.spin();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.01);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-8 );
    icp.setEuclideanFitnessEpsilon(0.00001);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4f aids = transSVD;
    pcl::transformPointCloud(*modelFeature.points,*tmp,aids);
    icp.setInputSource(tmp);
    icp.setInputTarget(sceneFeature.points);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    icp.align(*outCloud);

    Eigen::Matrix4f final;
    if(icp.hasConverged()){
        std::cout << "Converged! \n";
        final = icp.getFinalTransformation();
    }
    final = final*transSVD;

    std::cout << "FINAL transformation CAMERA to OBJECT: " << std::endl;
    std::cout << final << std::endl;

    std::cout << "CAMERA TO TAG MATRIX: " << std::endl;
    std::cout << cameraToTag << std::endl;

    Eigen::Matrix4f initialTable = cameraToTag.inverse()*transSVD;
    std::cout << "Initial table transform: " << std::endl;
    std::cout << initialTable << std::endl;
    Eigen::Matrix4f finalTable = cameraToTag.inverse()*final;
    std::cout << "Final table transform: " << std::endl;
    std::cout << finalTable << std::endl;

    vis.addPointCloud(tmp, "modelMoved");
    vis.addPointCloud(outCloud, "modelRefined");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*modelFeature.points,*finalPointCloud,final);
    vis.addPointCloud(finalPointCloud, "WHAT");

    Eigen::Matrix4f test = cameraToTag*worldToTag.inverse();
    Eigen::Affine3f C;
    C = test;
    vis.addCoordinateSystem(0.5, C);
    vis.spin();

    std::cout << "World coordinates" << std::endl;
    Eigen::Matrix4f posInWorld = worldToTag*finalTable;
    std::cout << posInWorld << std::endl;

    Eigen::Affine3f A2;
    float x, y, z, roll, pitch, yaw;
    A2 = posInWorld;
    pcl::getTranslationAndEulerAngles(A2, x, y, z, roll, pitch, yaw);
    std::cout << "X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
    std::cout << "Roll: " << roll*(180.0/3.14) << ", Pitch: " << pitch*(180.0/3.14) << ", yaw: " << yaw*(180.0/3.14) << std::endl;

    Eigen::Affine3f finalCoordinates;
    finalCoordinates = final;
    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    visualizer->addCoordinateSystem(0.2, B2);
    visualizer->addCoordinateSystem(0.2, finalCoordinates);
    visualizer->addCoordinateSystem(0.2, C);
    visualizer->addPointCloud(fullScene, "fullScene");
    visualizer->addPointCloud(heihoo, "model");

    Q_EMIT sendNewVisualizer(visualizer);

    pcl::visualization::PCLVisualizer tmpVis;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red2 (heihoo, 255, 0, 0);
    tmpVis.addPointCloud(heihoo,red2, "model");
    tmpVis.addPointCloud(fullScene, "fullScene");
    tmpVis.spin();
}

void PointCloudManipulator::alignAndMatch(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>());
    scene = alignCloudsRefined(clouds);
    std::cout << clouds.size() << std::endl;
    *model = *clouds.at(clouds.size()-1);
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(scene, "scene");
    vis.addPointCloud(model, "model");
    vis.spin();
    matchModelCloud(model, scene);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManipulator::alignCloudsRefined(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudsIn)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudsOriginal;
    std::vector<Eigen::Matrix4f> cameraPositions;
    Eigen::Matrix4f cam0 = Eigen::Matrix4f::Identity();
    cameraPositions.push_back(cam0);
    Eigen::Matrix4f cam1 = Eigen::Matrix4f::Identity();
    // This is the matrix from NUC2 (camera2) to table
    cam1 << -0.0369295,   -0.99916 , 0.0177825,   0.174928,
            -0.592998, 0.00758757 , -0.805168  , 0.016518,
            0.804357, -0.0402794 ,  -0.59278  , 0.945164,
            0,          0 ,         0   ,       1;
    cameraPositions.push_back(cam1);
    Eigen::Matrix4f cam2 = Eigen::Matrix4f::Identity();
    // This is the matrix from PC (camera3) to table
    cam2 <<   -0.999718 , -0.0224201 ,-0.00776418 ,   0.604796,
            -0.00404688 ,   0.483571  , -0.875296 ,  -0.256756,
            0.0233787 ,  -0.875018  , -0.483525  ,   2.14104,
            0 ,          0    ,       0     ,      1;
    cameraPositions.push_back(cam2);
    // This is the matrix from NUC1 (camera1) to table
    Eigen::Matrix4f cam3 = Eigen::Matrix4f::Identity();
    cam3 << -0.0324064,   0.999472, 0.00236665,  -0.236723,
            0.701194,  0.0244224,  -0.712552,  -0.589319,
            -0.712233, -0.0214317,  -0.701615,    1.82195,
            0,          0,          0,          1;

    cameraPositions.push_back(cam3);


    for(int i = 0; i<cloudsIn.size()-1; i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << "Loop nr: " << i << std::endl;
        *tmpCloud = *cloudsIn.at(i);

        cloudsOriginal.push_back(tmpCloud);
        tmpCloud = filterVoxel(tmpCloud, 0.01);

        switch(i){
        case 0:
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.39, "x");
            tmpCloud = filterPassThrough(tmpCloud, -1.1, -0.1, "y");
            break;
        case 1:
            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.3, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.7, 0.3, "y");
            break;
        case 2:
            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.39, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.1, "y");
            tmpCloud = filterPassThrough(tmpCloud, 0.8, 3.1, "z");
            break;
        }

        tmpCloud = filterVoxel(tmpCloud, 0.001);
        tmpCloud = extractPlane(tmpCloud,0.02);

        clouds.push_back(tmpCloud);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> roughClouds;
    roughClouds.push_back(clouds.at(0));
    std::vector<Eigen::Matrix4f> cameraPositions2;
    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    cameraPositions2.push_back(tmp);
    for(int i=1; i<clouds.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f tmpMat2 = cameraPositions.at(i);
        Eigen::Matrix4f tmpMat3 = cameraPositions.at(3);
        Eigen::Matrix4f final = tmpMat3*tmpMat2.inverse();
        cameraPositions2.push_back(final);
        pcl::transformPointCloud(*clouds.at(i),*tmp,final);
        roughClouds.push_back(tmp);
    }

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.03);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.0000001);
    icp.setInputTarget(roughClouds.at(0));

    for(int i=1; i<roughClouds.size(); i++){
        icp.setInputSource(roughClouds.at(i));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*outCloud);
        Eigen::Matrix4f transICP = Eigen::Matrix4f::Identity();
        if(icp.hasConverged()){
            std::cout << "Converged! ";
            std::cout << i << std::endl;
            transICP = icp.getFinalTransformation();
            cameraPositions2[i] = transICP*cameraPositions2[i];

        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr writeToFileCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i=0; i<roughClouds.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*cloudsOriginal[i], *tmp, cameraPositions2[i]);
        *writeToFileCloud += *tmp;
    }
    pcl::io::savePCDFileBinary("/home/minions/alignedWithObject.pcd", *writeToFileCloud);
    return (writeToFileCloud);
}


void PointCloudManipulator::alignClouds(QStringList fileNames)
{
    std::vector<PointCloudFeatures> pointClouds;

    for(int i = 0; i<fileNames.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << fileNames.at(i).toStdString() << std::endl;
        pcl::io::loadPCDFile(fileNames.at(i).toUtf8().constData(), *tmpCloud);
        tmpCloud = filterPassThrough(tmpCloud, 0.0, 2.3, "z");
        tmpCloud = filterPassThrough(tmpCloud, -0.5, 1.0, "y");
        tmpCloud = filterPassThrough(tmpCloud, -1.0, 1.0, "x");
        tmpCloud = filterVoxel(tmpCloud, 0.01);
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
    icp.setRANSACIterations(0);
    icp.setTransformationEpsilon(1e-8 );
    icp.setEuclideanFitnessEpsilon(0.00001);
    Eigen::Matrix4f prevTrans = Eigen::Matrix4f::Identity();

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
        visualizeTransformation(pointClouds.at(k+1).points, pointClouds.at(k).points, transSVD);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f trans = prevTrans*transSVD.inverse();
        pcl::transformPointCloud(*pointClouds.at(k+1).points,*tmp,trans);

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
        *tmpAligned = *tmpAligned + *tmp;
        prevTrans = trans;
    }

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(tmpAligned, "alignedcloud");
    vis.spin();

    pcl::visualization::PCLVisualizer vis2;
    vis2.addPointCloud(icpCloud, "icpalignedcloud");
    vis2.spin();
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
        tmpCloud = extractPlaneReturnPlane(tmpCloud, 0.1);

        PointCloudFeatures tmpFeature = computeFeatures(tmpCloud, "SIFT", "FPFH");
        pointClouds.push_back(tmpFeature);
    }

    Eigen::Matrix4f prevTrans = Eigen::Matrix4f::Identity();
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
        visualizeTransformation(pointClouds.at(k+1).points, pointClouds.at(k).points, transSVD);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f trans = prevTrans*transSVD.inverse();
        pcl::transformPointCloud(*pointClouds.at(k+1).points,*tmp,trans);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpAligned (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    *tmpAligned = *pointClouds.at(0).points;
    *icpCloud = *pointClouds.at(0).points;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.3);
    icp.setMaximumIterations(10000);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.0000001);

    for(int k = 0; k<pointClouds.size()-1; k++){
        Eigen::Matrix4f initTrans = Eigen::Matrix4f::Identity ();
        Eigen::Matrix4f cameraPos = Eigen::Matrix4f::Identity();
        initTrans = computeInitialAlignmentFPFH(pointClouds.at(0).keyPoints,pointClouds.at(0).localDescriptorsFPFH,pointClouds.at(k+1).keyPoints, pointClouds.at(k+1).localDescriptorsFPFH,0.08,1.0,1000);
        std::cout << initTrans << std::endl;
        visualizeTransformation(pointClouds.at(k+1).points, pointClouds.at(0).points, initTrans);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Matrix4f initTransInv = initTrans.inverse();
        pcl::transformPointCloud(*pointClouds.at(k+1).points,*tmp,initTransInv);

        *tmpAligned = *tmpAligned + *tmp;
        cameraPos = initTrans.inverse();
        icp.setInputSource(tmp);
        icp.setInputTarget(pointClouds.at(0).points);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*outCloud);
        Eigen::Matrix4f transICP = Eigen::Matrix4f::Identity();
        if(icp.hasConverged()){
            *icpCloud = *icpCloud + *outCloud;
            std::cout << "Converged! ";
            std::cout << k << std::endl;
            transICP = icp.getFinalTransformation();
            cameraPos = transICP*cameraPos;
            cameraPositions.push_back(cameraPos);
        }
    }

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(tmpAligned, "alignedcloud");
    vis.spin();

    pcl::visualization::PCLVisualizer vis2;
    vis2.addPointCloud(icpCloud, "icpalignedcloud");
    vis2.spin();

    pcl::visualization::PCLVisualizer vis3;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalAligned (new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i = 0; i<3; i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        QString s = "cloud";
        s.append(QString::number(i));
        pcl::transformPointCloud(*originalClouds.at(i),*tmp,cameraPositions.at(i));
        std::cout << cameraPositions.at(i) << std::endl;
        vis3.addPointCloud(tmp,s.toStdString());
        *originalAligned += *tmp;
    }
    vis3.addCoordinateSystem(0.5);
    Eigen::Affine3f A;
    A = cameraPositions.at(1);
    vis3.addCoordinateSystem(0.5, A);
    std::cout << "Cam1: " << std::endl;
    std::cout <<  cameraPositions.at(1) << std::endl;
    A = cameraPositions.at(2);
    std::cout << "Cam2: " << std::endl;
    std::cout <<  cameraPositions.at(2) << std::endl;
    vis3.addCoordinateSystem(0.5, A);
    vis3.spin();
    pcl::io::savePCDFileBinary("/home/minions/aligned.pcd", *originalAligned);
}

void PointCloudManipulator::refineAlignment(QStringList fileNames)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudsOriginal;
    std::vector<Eigen::Matrix4f> cameraPositions;
    Eigen::Matrix4f cam0 = Eigen::Matrix4f::Identity();
    cameraPositions.push_back(cam0);
    Eigen::Matrix4f cam1 = Eigen::Matrix4f::Identity();
    // This is the matrix from NUC2 (cam2) to table
    cam1 << -0.0369295,   -0.99916 , 0.0177825,   0.174928,
            -0.592998, 0.00758757 , -0.805168  , 0.016518,
            0.804357, -0.0402794 ,  -0.59278  , 0.945164,
            0,          0 ,         0   ,       1;
    cameraPositions.push_back(cam1);
    Eigen::Matrix4f cam2 = Eigen::Matrix4f::Identity();
    // This is the matrix from PC (cam3) to table
    cam2 <<   -0.999718 , -0.0224201 ,-0.00776418 ,   0.604796,
            -0.00404688 ,   0.483571  , -0.875296 ,  -0.256756,
            0.0233787 ,  -0.875018  , -0.483525  ,   2.14104,
            0 ,          0    ,       0     ,      1;
    cameraPositions.push_back(cam2);
    // This is the matrix from NUC1 (cam1) to table
    Eigen::Matrix4f cam3 = Eigen::Matrix4f::Identity();
    cam3 << -0.0324064,   0.999472, 0.00236665,  -0.236723,
            0.701194,  0.0244224,  -0.712552,  -0.589319,
            -0.712233, -0.0214317,  -0.701615,    1.82195,
            0,          0,          0,          1;
    cameraPositions.push_back(cam3);

    for(int i = 0; i<fileNames.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << fileNames.at(i).toStdString() << std::endl;
        pcl::io::loadPCDFile(fileNames.at(i).toUtf8().constData(), *tmpCloud);

        cloudsOriginal.push_back(tmpCloud);
        tmpCloud = filterVoxel(tmpCloud, 0.01);

        switch(i){
        case 0:
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.39, "x");
            tmpCloud = filterPassThrough(tmpCloud, -1.1, -0.1, "y");
            break;
        case 1:
            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.3, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.7, 0.3, "y");
            break;
        case 2:
            tmpCloud = filterPassThrough(tmpCloud, -0.5, 0.39, "x");
            tmpCloud = filterPassThrough(tmpCloud, -0.4, 0.1, "y");
            tmpCloud = filterPassThrough(tmpCloud, 0.8, 3.1, "z");
            break;
        }
        tmpCloud = filterVoxel(tmpCloud, 0.001);
        tmpCloud = extractPlane(tmpCloud,0.02);
        clouds.push_back(tmpCloud);
    }

    pcl::visualization::PCLVisualizer vis1;
    for(int i=0; i<clouds.size(); i++){
        QString s = "cloud";
        s.append(QString::number(i));
        vis1.addPointCloud(clouds.at(i),s.toStdString());
    }
    vis1.spin();

    pcl::visualization::PCLVisualizer vis2;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> roughClouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initAlignedSave (new pcl::PointCloud<pcl::PointXYZRGB>());
    roughClouds.push_back(clouds.at(0));
    *initAlignedSave = *cloudsOriginal.at(0);
    std::vector<Eigen::Matrix4f> cameraPositions2;
    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    cameraPositions2.push_back(tmp);
    vis2.addPointCloud(clouds.at(0), "cloud0");
    for(int i=1; i<clouds.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        QString s = "cloud";
        s.append(QString::number(i));
        Eigen::Matrix4f tmpMat2 = cameraPositions.at(i);
        Eigen::Matrix4f tmpMat3 = cameraPositions.at(3);
        Eigen::Matrix4f final = tmpMat3*tmpMat2.inverse();
        cameraPositions2.push_back(final);
        pcl::transformPointCloud(*clouds.at(i),*tmp,final);
        roughClouds.push_back(tmp);
        vis2.addPointCloud(tmp,s.toStdString());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*cloudsOriginal.at(i), *tmp2, final);
        *initAlignedSave += *tmp2;
    }
    pcl::io::savePCDFileBinary("/home/minions/initialAlignedWithObject.pcd", *initAlignedSave);
    vis2.spin();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(0.03); // 0.05
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.0000001);
    icp.setInputTarget(roughClouds.at(0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    *icpCloud += *roughClouds.at(0);

    for(int i=1; i<roughClouds.size(); i++){
        icp.setInputSource(roughClouds.at(i));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*outCloud);
        Eigen::Matrix4f transICP = Eigen::Matrix4f::Identity();
        if(icp.hasConverged()){
            *icpCloud = *icpCloud + *outCloud;
            std::cout << "Converged! ";
            std::cout << i << std::endl;
            transICP = icp.getFinalTransformation();
            cameraPositions2[i] = transICP*cameraPositions2[i];
        }
    }

    pcl::visualization::PCLVisualizer vis3;
    vis3.addPointCloud(icpCloud, "icp");
    vis3.spin();

    pcl::visualization::PCLVisualizer vis4;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr writeToFileCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i=0; i<roughClouds.size(); i++){
        Eigen::Affine3f A;
        A = cameraPositions2[i];
        vis4.addCoordinateSystem(0.5, A);
        QString s = "cloud";
        s.append(QString::number(i));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*cloudsOriginal[i], *tmp, cameraPositions2[i]);
        vis4.addPointCloud(tmp, s.toStdString());
        *writeToFileCloud += *tmp;
    }
    vis4.spin();
}

}
