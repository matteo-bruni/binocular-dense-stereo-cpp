/*
    *  stereo_match.cpp
    *  calibration
    *
    *  Created by Victor  Eruhimov on 1/18/10.
    *  Copyright 2010 Argus Corp. All rights reserved.
    *
    */
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "opencv2/photo/photo.hpp"


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <libconfig.h++>



#include <stdio.h>
#include <libconfig.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

// local includes
#include "stereo_matching.hpp"
#include "../utils/util.hpp"
#include "../dataset/msm_middlebury.hpp"
#include "../dataset/tsukuba_dataset.h"
#include "../dataset/dataset.hpp"


// logger
#include "../logger/log.h"


using namespace cv;

namespace stereo {


    /*
        input   :
                Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, scale
        output  :
                Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2,
     */
    void rectifyImages(Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2, float scale){

        Size img_size = img1.size();

        M1 *= scale;
        M2 *= scale;

        FILE_LOG(logDEBUG) << stereo_util::infoMatrix(T);

        // dopo Q: 0 o CV_CALIB_ZERO_DISPARITY
        int flags = 0;
        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, flags, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_CUBIC);
        remap(img2, img2r, map21, map22, INTER_CUBIC); // prima linear

        img1 = img1r;
        img2 = img2r;

    }


    void computeDisparityTsukuba(const int img_frame, Mat& img_left, Mat& img_right,Mat& disp,int alg,Rect & roi1,Rect &roi2){


        std::string tipo = "BM";
        Mat g1, g2;

        ///da provare
        cvtColor(img_left, g1, CV_BGR2GRAY);
        cvtColor(img_right, g2, CV_BGR2GRAY);

        StereoBM sbm;

        if (tipo == "BM")
        {


            int numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_left.rows/8) + 15) & -16;

            sbm.state->roi1 = roi1;
            sbm.state->roi2 = roi2;
            sbm.state->preFilterCap = 63;
            sbm.state->SADWindowSize = 5;
            sbm.state->minDisparity = 1;
            sbm.state->numberOfDisparities = 32;
            sbm.state->textureThreshold = 9;
            sbm.state->uniquenessRatio = 12;
            sbm.state->speckleWindowSize = 0;
            sbm.state->speckleRange = 0;
            sbm.state->disp12MaxDiff = 1;

        }
        else if (tipo == "SGBM")
        {
            StereoSGBM sbm;
            sbm.SADWindowSize = 5;
            sbm.numberOfDisparities = 112;
            sbm.preFilterCap = 63;
            sbm.minDisparity = 0;
            sbm.uniquenessRatio = 10;
            sbm.speckleWindowSize = 0;
            sbm.speckleRange = 0;
            sbm.disp12MaxDiff = 1;
            sbm.fullDP = false;
            sbm.P1 = 8*3*5*5;
            sbm.P2 = 8*3*5*5;
            sbm(g1, g2, disp);
        }

        sbm(g1, g2, disp, CV_32F);
        imwrite("disp_"+std::to_string(img_frame)+".png", disp);
    }

    void createPointCloudOpenCV (Mat& img1, Mat& img2,  Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr) {

        cv::reprojectImageTo3D(disp, recons3D, Q, true);
        FILE_LOG(logDEBUG) << "disp - " <<stereo_util::infoMatrix(disp);

        FILE_LOG(logDEBUG) << "reconst - " <<stereo_util::infoMatrix(recons3D) << " img - " << stereo_util::infoMatrix(img1);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for (int rows = 0; rows < recons3D.rows; ++rows) {

            for (int cols = 0; cols < recons3D.cols; ++cols) {

                cv::Point3f point = recons3D.at<cv::Point3f>(rows, cols);

                pcl::PointXYZ pcl_point(point.x, point.y, point.z); // normal PointCloud
                pcl::PointXYZRGB pcl_point_rgb;
                pcl_point_rgb.x = point.x;    // rgb PointCloud
                pcl_point_rgb.y = point.y;
                pcl_point_rgb.z = point.z;
                // image_left is the stereo rectified image used in stere reconstruction
                cv::Vec3b intensity = img1.at<cv::Vec3b>(rows, cols); //BGR

                uint32_t rgb = (static_cast<uint32_t>(intensity[2]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[0]));

                pcl_point_rgb.rgb = *reinterpret_cast<float *>(&rgb);

                // filter erroneus points
                if (pcl_point_rgb.z < 0)
                    point_cloud_ptr->push_back(pcl_point_rgb);
            }


        }

        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        FILE_LOG(logDEBUG) << "Esco..";
    }




    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloudTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, const int frame_num){

        FILE_LOG(logINFO) << "Loading data.. frame" << frame_num;

        // load images data
        Ptr<cv::datasets::tsukuba_datasetObj> data_stereo_img =
                static_cast< Ptr<cv::datasets::tsukuba_datasetObj> >  (dataset->getTrain()[frame_num]);


        FILE_LOG(logINFO) << "Loading images..";
        // load images
        cv::Mat img_left, img_right;
        cv::datasets::FramePair tuple_img = dataset->load_stereo_images(frame_num+1);
        img_left = tuple_img.frame_left;
        img_right = tuple_img.frame_right;

        // init
        Mat R1,R2,P1,P2,Q;
        // zero distiorsions
        Mat D_left = Mat::zeros(1, 5, CV_64F);
        Mat D_right = Mat::zeros(1, 5, CV_64F);

        // load K and R from dataset info
        Mat M_left = Mat(data_stereo_img->k);
        Mat M_right = Mat(data_stereo_img->k);

        // Left image
        Mat r_left = Mat(data_stereo_img->r);
        Mat t_left = Mat(3, 1, CV_64FC1, &data_stereo_img->tl);

        // Right image
        Mat r_right = Mat(data_stereo_img->r);
        Mat t_right = Mat(3, 1, CV_64FC1, &data_stereo_img->tr);

        // rotation between left and right
        // use ground truth rotation (img are already rectified
        cv::Mat R = Mat::eye(3,3, CV_64F); //r_right*r_left.inv();
        // translation between img2 and img1
//        cv::Mat T = t_left - (R.inv()*t_right );
        // use ground truth translation
        cv::Mat T = Mat::zeros(3, 1, CV_64F);
        T.at<double>(0,0) = 10.;

        FILE_LOG(logINFO) << "translation between cameras: " << T;

        FILE_LOG(logINFO) << "Rectifying images...";
        Rect roi1,roi2;
        stereo::rectifyImages(img_left, img_right, M_left, D_left, M_right, D_right, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);


        FILE_LOG(logINFO) << "Computing Disparity map Dense Stereo";
        Mat disp(img_left.size(), CV_32F);

        FILE_LOG(logDEBUG) << "imgsize " << stereo_util::infoMatrix(img_left);
        FILE_LOG(logDEBUG) << "dispsize " << stereo_util::infoMatrix(disp);

//        stereo::computeDisparityTsukuba(frame_num, img_left, img_right, disp,1,roi1,roi2);
        // load ground truth disparity
        disp = dataset->load_disparity(frame_num+1);

        FILE_LOG(logINFO) << "Creating point cloud..";
        Mat recons3D(disp.size(), CV_32FC3);
        FILE_LOG(logINFO) << "recons3Dsize " << stereo_util::infoMatrix(recons3D);
        FILE_LOG(logINFO) << "disp " << stereo_util::infoMatrix(disp);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        stereo::createPointCloudOpenCV(img_left, img_right, Q, disp, recons3D, point_cloud_ptr);
        return point_cloud_ptr;
    }



    void createAllCloudsTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, int first_frame,  int last_frame, int step){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        int frame_num;
        for (int i=first_frame; i<last_frame; i+=step){

            frame_num = i;

            cloud = generatePointCloudTsukuba(dataset, frame_num);

            if(!(*cloud).empty()){

                Eigen::Matrix4d transf = stereo_util::getTransformToWorldCoordinatesTsukuba(dataset, frame_num);
                // Executing the transformation
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
                // You can either apply transform_1 or transform_2; they are the same
                pcl::transformPointCloud (*cloud, *transformed_cloud, transf);
                clouds.push_back(transformed_cloud);
            }

        }
        FILE_LOG(logINFO) << "cloud size" <<clouds.size();

    }
}
