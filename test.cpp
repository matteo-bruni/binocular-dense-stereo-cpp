
//void surfaceReconstruction(){
//
////////letto che poisson è pesa va usato convex hull
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    io::loadPCDFile ("./registrazione.pcd", *cloud);
//
//    MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
//    mls.setInputCloud (cloud);
//    mls.setSearchRadius (4);
//    mls.setPolynomialFit (true);
//    mls.setPolynomialOrder (1);
//    mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGB, PointXYZRGB>::SAMPLE_LOCAL_PLANE);
//    mls.setUpsamplingRadius (1);
//    mls.setUpsamplingStepSize (0.3);
//
//    PointCloud<PointXYZRGB>::Ptr cloud_smoothed (new PointCloud<PointXYZRGB> ());
//    mls.process (*cloud_smoothed);
//
//    NormalEstimationOMP<PointXYZRGB, Normal> ne;
//    ne.setNumberOfThreads (8);
//    ne.setInputCloud (cloud_smoothed);
//    ne.setRadiusSearch (0.8);
//    Eigen::Vector4f centroid;
//    compute3DCentroid (*cloud_smoothed, centroid);
//    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
//
//    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
//    ne.compute (*cloud_normals);
//    for (size_t i = 0; i < cloud_normals->size (); ++i)
//    {
//        cloud_normals->points[i].normal_x *= -1;
//        cloud_normals->points[i].normal_y *= -1;
//        cloud_normals->points[i].normal_z *= -1;
//    }
//    PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
//    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
//
//    Poisson<pcl::PointNormal> poisson;
//    poisson.setDepth (9);
//    poisson.setInputCloud  (cloud_smoothed_normals);
//    PolygonMesh mesh_poisson;
//    poisson.reconstruct (mesh_poisson);
//
//}