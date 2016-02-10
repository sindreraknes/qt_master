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
    filterList.append("Filter3");

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

void PointCloudManipulator::keyPointsISS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

    // ISS keypoint detector object.
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setSalientRadius(6 * resolution);
    // Set the radius for the application of the non maxima supression algorithm.
    detector.setNonMaxRadius(4 * resolution);
    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    detector.setMinNeighbors(5);
    // Set the upper bound on the ratio between the second and the first eigenvalue.
    detector.setThreshold21(0.975);
    // Set the upper bound on the ratio between the third and the second eigenvalue.
    detector.setThreshold32(0.975);
    // Set the number of prpcessing threads to use. 0 sets it to automatic.
    detector.setNumberOfThreads(4);
    detector.compute(*keypoints);

    visualizer.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    visualizer->addPointCloud<pcl::PointXYZ> (cloud, "filteredCloud");
    visualizer->addPointCloud<pcl::PointXYZ> (keypoints, "keyPoints");
    visualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keyPoints");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,0.0,1.0, "keyPoints");

    Q_EMIT sendNewVisualizer(visualizer);
}

void PointCloudManipulator::keyPointsNARF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<int>::Ptr keypoints(new pcl::PointCloud<int>);

    // Convert the cloud to range image.
    int imageSizeX = 640, imageSizeY = 480;
    float centerX = (640.0f / 2.0f), centerY = (480.0f / 2.0f);
    float focalLengthX = 525.0f, focalLengthY = focalLengthX;
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                                 cloud->sensor_origin_[1],
                                                 cloud->sensor_origin_[2])) *
                                                 Eigen::Affine3f(cloud->sensor_orientation_);

    float noiseLevel = 0.0f, minimumRange = 0.0f;
    pcl::RangeImagePlanar rangeImage;
    rangeImage.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
                                                 centerX, centerY, focalLengthX, focalLengthX,
                                                 sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                 noiseLevel, minimumRange);

    pcl::RangeImageBorderExtractor borderExtractor;
    // Keypoint detection object.
    pcl::NarfKeypoint detector(&borderExtractor);
    detector.setRangeImage(&rangeImage);
    // The support size influences how big the surface of interest will be,
    // when finding keypoints from the border information.
    detector.getParameters().support_size = 0.2f;
    detector.compute(*keypoints);

    // Visualize the keypoints.
    pcl::visualization::RangeImageVisualizer viewer("NARF keypoints");
    viewer.showRangeImage(rangeImage);
    for (size_t i = 0; i < keypoints->points.size(); ++i)
    {
        viewer.markPoint(keypoints->points[i] % rangeImage.width,
                         keypoints->points[i] / rangeImage.width,
                         // Set the color of the pixel to red (the background
                         // circle is already that color). All other parameters
                         // are left untouched, check the API for more options.
                         pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));
    }



    /* Put the points in a cloud */
    // NOT IMPLEMENTED AND NOT TESTED

    /**
      this->keyPoints->points.resize(keypointIndices.points.size());
      for (size_t i=0; i<keypointIndices.points.size(); ++i)
      {
          this->keyPoints->points[i].getVector3fMap () = this->rangeImage->points[keypointIndices.points[i]].getVector3fMap();
          this->keyPoints->points[i].r = 0;
          this->keyPoints->points[i].g = 255;
          this->keyPoints->points[i].b = 0;
          //this->keyPoints->points[i].size
      }

      *this->cloud += *this->keyPoints;  */

    while (!viewer.wasStopped())
        {
            viewer.spinOnce();
            // Sleep 100ms to go easy on the CPU.
            pcl_sleep(0.1);
        }

    pcl::PointCloud<pcl::BorderDescription>::Ptr borders(new pcl::PointCloud<pcl::BorderDescription>);
    borderExtractor.compute(*borders);
    pcl::visualization::RangeImageVisualizer* viewer2 = NULL;
        viewer2 = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(rangeImage,
                 -std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 false, *borders, "Borders");

        while (!viewer2->wasStopped())
            {
                viewer2->spinOnce();
                // Sleep 100ms to go easy on the CPU.
                pcl_sleep(0.1);
            }

    // The NARF estimator needs the indices in a vector, not a cloud.
    pcl::PointCloud<pcl::Narf36>::Ptr descriptors(new pcl::PointCloud<pcl::Narf36>);
    std::vector<int> keypoints2;
    keypoints2.resize(keypoints->points.size());
    for (unsigned int i = 0; i < keypoints->size(); ++i)
        keypoints2[i] = keypoints->points[i];
    // NARF estimation object.
    pcl::NarfDescriptor narf(&rangeImage, &keypoints2);
    // Support size: choose the same value you used for keypoint extraction.
    narf.getParameters().support_size = 0.2f;
    // If true, the rotation invariant version of NARF will be used. The histogram
    // will be shifted according to the dominant orientation to provide robustness to
    // rotations around the normal.
    narf.getParameters().rotation_invariant = true;

    narf.compute(*descriptors);

    std::cout << "Extracted "<<descriptors->size()<<" descriptors for "
              <<keypoints->points.size()<< " keypoints.\n";
}

void PointCloudManipulator::tester(pcl::PointCloud<pcl::PointXYZ>::Ptr pointsIn, pcl::PointCloud<pcl::PointXYZ>::Ptr pointsIn2)
{
    /**
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2(new pcl::PointCloud<pcl::PointXYZRGB>);
    points1 = pointsIn;
    points2 = pointsIn2;
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1(new pcl::PointCloud<pcl::PointXYZRGB>);
    //Convert to XYZRGB point
    pcl::copyPointCloud(*pointsIn,*points1);
    for (int i = 0; i< points1->points.size(); i++){
           points1->points[i].r = 255;
           points1->points[i].g = 255;
           points1->points[i].b = 255;
       }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2(new pcl::PointCloud<pcl::PointXYZRGB>);
    //Convert to XYZRGB point
    pcl::copyPointCloud(*pointsIn2,*points2);
    for (int i = 0; i< points2->points.size(); i++){
           points2->points[i].r = 255;
           points2->points[i].g = 255;
           points2->points[i].b = 255;
       }


    // CLOUD 1 STUFF
    pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);
    // CLOUD 2 STUFF
    pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);
    // NORMALS
    const float normalRadius = 0.03;
    computeNormals(points1, normalRadius, normals1);
    computeNormals(points2, normalRadius, normals2);
    std::cout << "DONE NORMALS" << std::endl;
    // KEYPOINTS
    const float min_scale = 0.01;
    const int nr_octaves = 3;
    const int nr_octaves_per_scale = 3;
    // NEEDS TO BE 0 FOR NON RGB (CONTRAST)
    const float min_contrast = 0;
    computeKeyPoints(points1,min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints1);
    computeKeyPoints(points2,min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints2);
    std::cout << "DONE KEYPOINTS" << std::endl;
    // PFH
    const float feature_radius = 0.08;
    computePFH(points1,normals1,keypoints1,feature_radius,descriptors1);
    computePFH(points2,normals2,keypoints1,feature_radius,descriptors2);
    std::cout << "DONE DESCRIPTORS" << std::endl;
    // CORRESPONDENCES
    std::vector<int> correspondences;
    std::vector<float> correspondence_scores;
    computeFeatureCorrespondences(descriptors1, descriptors2, correspondences, correspondence_scores);
    std::cout << correspondences.size() << std::endl;
    std::cout << correspondence_scores.size() << std::endl;
    // Print out ( number of keypoints / number of points )
    std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
              << "out of " << points1->size () << " total points." << std::endl;
    std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
              << "out of " << points2->size () << " total points." << std::endl;

    //VISUALIZE CORRESPONDENCES
    visualizeCorrespondences(points1, keypoints1, points2, keypoints2, correspondences, correspondence_scores);

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::PFHSignature125> sac;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr key1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (*keypoints1, *key1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr key2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (*keypoints2, *key2);
    sac.setInputSource(key1);
    sac.setSourceFeatures(descriptors1);
    sac.setInputTarget(key2);
    sac.setTargetFeatures(descriptors2);
    sac.setMaxCorrespondenceDistance(0.05);
    sac.setNumberOfSamples(5);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
    sac.align(*source_aligned);
    Eigen::Matrix4f TT;
    TT = sac.getFinalTransformation();
}

void PointCloudManipulator::computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, float normalRadius, pcl::PointCloud<pcl::Normal>::Ptr normal)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    // Specify the size of the local neighborhood to use when computing the surface normals
    norm_est.setRadiusSearch (normalRadius);
    // Set the input points
    norm_est.setInputCloud (cloudIn);
    // Estimate the surface normals and store the result in "normals_out"
    norm_est.compute (*normal);

}

void PointCloudManipulator::computeKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn,float minScale, int nrOctaves, int nrScalesPerOctaves, float minContrast,
                                             pcl::PointCloud<pcl::PointWithScale>::Ptr keyPoints)
{
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
    // Use a FLANN-based KdTree to perform neighborhood searches
    sift_detect.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    // Set the detection parameters
    sift_detect.setScales (minScale, nrOctaves, nrScalesPerOctaves);
    sift_detect.setMinimumContrast (minContrast);
    // Set the input
    sift_detect.setInputCloud (cloudIn);
    // Detect the keypoints and store them in "keypoints_out"
    sift_detect.compute (*keyPoints);
}

void PointCloudManipulator::computePFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud,
                                       pcl::PointCloud<pcl::Normal>::Ptr normals,
                                       pcl::PointCloud<pcl::PointWithScale>::Ptr keyPoints, float featureRadius,
                                       pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch (featureRadius);

    /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
     * use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
     * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
     * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
     * values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (*keyPoints, *keypoints_xyzrgb);
    // Use all of the points for analyzing the local structure of the cloud
    pfh_est.setSearchSurface (inCloud);
    pfh_est.setInputNormals (normals);
    // But only compute features at the keypoints
    pfh_est.setInputCloud (keypoints_xyzrgb);
    // Compute the features
    pfh_est.compute (*descriptors);
}

void PointCloudManipulator::computeFeatureCorrespondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr source_descriptors,
                                                          pcl::PointCloud<pcl::PFHSignature125>::Ptr target_descriptors,
                                                          std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
    correspondences_out.resize(source_descriptors->size());
    correspondence_scores_out.resize(source_descriptors->size());

    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
      descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
      correspondences_out[i] = k_indices[0];
      correspondence_scores_out[i] = k_squared_distances[0];
    }


}

void PointCloudManipulator::visualizeCorrespondences( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                                      const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                                      const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                                      const std::vector<int> &correspondences,
                                                      const std::vector<float> &correspondence_scores)
{
    // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
    // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points


    // Create some new point clouds to hold our transformed data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);


    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points_left, "points_left");
    viz.addPointCloud (points_right, "points_right");


    // Compute the median correspondence score
    std::vector<float> temp (correspondence_scores);
    std::sort (temp.begin (), temp.end ());
    float median_score = temp[temp.size ()/2];



    // Draw lines between the best corresponding points
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
      if (correspondence_scores[i] > median_score)
      {
        continue; // Don't draw weak correspondences
      }

      // Get the pair of points
      const pcl::PointWithScale & p_left = keypoints_left->points[i];
      const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

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
      viz.addLine (p_left, p_right, r, g, b, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}

double PointCloudManipulator::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    double resolution = 0.0;
        int numberOfPoints = 0;
        int nres;
        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);
        pcl::search::KdTree<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (! pcl_isfinite((*cloud)[i].x))
                continue;

            // Considering the second neighbor since the first is the point itself.
            nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
            if (nres == 2)
            {
                resolution += sqrt(squaredDistances[1]);
                ++numberOfPoints;
            }
        }
        if (numberOfPoints != 0)
            resolution /= numberOfPoints;

        return resolution;
}

}
