#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>


// local include
#include "viewer.hpp"


// Include logging facilities
#include "../logger/log.h"

namespace stereo {


    using pcl::visualization::PointCloudColorHandlerGenericField;
    using pcl::visualization::PointCloudColorHandlerCustom;


    void viewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr) {
        //Create visualizer
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = createVisualizer( point_cloud_ptr );

        viewer->resetCameraViewpoint ("reconstruction");
        //Main loop
        while ( !viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }



    void viewDoublePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr2) {
        //Create visualizer
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = createDoubleVisualizer(point_cloud_ptr, point_cloud_ptr2 );

        //Main loop
        while ( !viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0.3, 0.3, 0.3);

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
        //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
        viewer->addCoordinateSystem ( 1.0 );
        viewer->initCameraParameters ();
        //viewer->spin();
        return (viewer);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createDoubleVisualizer (
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2) {

        // --------------------------------------------------------
        // -----Open 3D viewer and add point cloud and normals-----
        // --------------------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v1);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb2, "sample cloud2", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
        viewer->addCoordinateSystem (1.0);


        return (viewer);

    }

}