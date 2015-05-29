//
// Created by hunter on 28/04/15.
//

#ifndef BINOCULAR_DENSE_STEREO_CUSTOM_TYPES_HPP
#define BINOCULAR_DENSE_STEREO_CUSTOM_TYPES_HPP


#include "../includes.h"

//convenient typedefs
//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointTRGB> PointCloudRGB;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


#endif //BINOCULAR_DENSE_STEREO_CUSTOM_TYPES_HPP
