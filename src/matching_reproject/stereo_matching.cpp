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


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <stdio.h>

#include "stereo_matching.hpp"
#include "../utils/util.hpp"
#include "../logger/log.h"


using namespace cv;


//
//static void print_help()
//{
//    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
//    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
//            "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
//            "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
//}
//
//static void saveXYZ(const char* filename, const Mat& mat)
//{
//    const double max_z = 1.0e4;
//    FILE* fp = fopen(filename, "wt");
//    for(int y = 0; y < mat.rows; y++)
//    {
//        for(int x = 0; x < mat.cols; x++)
//        {
//            Vec3f point = mat.at<Vec3f>(y, x);
//            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
//            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
//        }
//    }
//    fclose(fp);
//}

namespace stereo {

    /*
        input : images number img1_num, img2_num
        output: cv:Mat matrices
     */
    void loadImages(const int img1_num, const int img2_num, Mat &img1, Mat &img2) {


        std::ostringstream ss;
        ss << std::setw(2) << std::setfill('0') << img1_num;
        std::string img1_path = "../dataset/dataset_templeRing/templeR00"+ ss.str() +".png";
        FILE_LOG(logINFO) << " loading " << img1_path;
        // clear string stream
        ss.str(std::string());
        ss << std::setw(2) << std::setfill('0') << img2_num;
        std::string img2_path = "../dataset/dataset_templeRing/templeR00"+ ss.str() +".png";
        FILE_LOG(logINFO) << " loading " << img2_path;

        int color_mode = -1; // = alg == STEREO_BM ? 0 : -1;

        img1 = imread(img1_path);
        img2 = imread(img2_path);

        float scale = 1.f; // TODO check
        if (1.f != scale) {
            Mat temp1, temp2;
            int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
            resize(img1, temp1, Size(), scale, scale, method);
            img1 = temp1;
            resize(img2, temp2, Size(), scale, scale, method);
            img2 = temp2;
        }

    }

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

        FILE_LOG(logINFO) << stereo_util::infoMatrix(T);


        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );


        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_CUBIC);
        remap(img2, img2r, map21, map22, INTER_CUBIC); // prima linear

        img1 = img1r;
        img2 = img2r;
    }



    void computeDisparity(const int img1_num, const int img2_num, Mat& img1, Mat& img2,Mat& disp,int alg,Rect & roi1,Rect &roi2){


        /////////////////


        std::string tipo = "BM";

        Mat g1, g2;


        cvtColor(img1, g1, CV_BGR2GRAY);
        cvtColor(img2, g2, CV_BGR2GRAY);

        if (img1_num < 32)
            stereo_util::rotate(g1, 90, g1);
        else
            stereo_util::rotate(g1, -90, g1);

        if (img2_num < 32)
            stereo_util::rotate(g2, 90, g2);
        else
            stereo_util::rotate(g2, -90, g2);

        if (tipo == "BM")
        {
//            StereoBM sbm;
//            sbm.state->SADWindowSize = 5;
//            sbm.state->numberOfDisparities = 160;
//            sbm.state->preFilterSize = 5;
//            sbm.state->preFilterCap = 11;
//            sbm.state->minDisparity = -68;
//            sbm.state->textureThreshold = 130;
//            sbm.state->uniquenessRatio = 0;
//            sbm.state->speckleWindowSize = 0;
//            sbm.state->speckleRange = 0;
//            sbm.state->disp12MaxDiff = 1;
            StereoBM sbm;
            sbm.state->SADWindowSize = 5;
            sbm.state->numberOfDisparities = 160;
            sbm.state->preFilterSize = 5;
            sbm.state->preFilterCap = 11;
            sbm.state->minDisparity = 6;
            sbm.state->textureThreshold = 173;
            sbm.state->uniquenessRatio = 0;
            sbm.state->speckleWindowSize = 0;
            sbm.state->speckleRange = 0;
//            sbm.state->disp12MaxDiff = 1;
            sbm(g1, g2, disp);
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


        normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

        if (img1_num < 32)
            stereo_util::rotate(disp, -90, disp);
        else
            stereo_util::rotate(disp, 90, disp);

        // APPLY OPENING
        cv::Mat const structure_elem = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(3, 3));
//        cv::Mat open_result;
        cv::morphologyEx(disp, disp,
                cv::MORPH_OPEN, structure_elem);



//        stereo::display(img1_num, img2_num, g1, g2, disp);
//        stereo::display(img1_num, img2_num, g1, disp, open_result);

    }

    void display(const int img1_num, const int img2_num, Mat& img1, Mat& img2,Mat& disp){

//        namedWindow("left ", 1);
        imshow("left "+std::to_string(img1_num), img1);
//        namedWindow("right", 1);
        imshow("right"+std::to_string(img2_num), img2);
//        namedWindow("disparity", 0);
        imshow("disparity", disp);
        printf("press any key to continue...");

        fflush(stdout);
        waitKey();
        destroyAllWindows();

        printf("\n");

    }

    void storePointCloud(Mat& disp, Mat& Q,/*const char* filename,*/ Mat& recons3D){

        FILE_LOG(logINFO) << "storing the point cloud..";

//        printf("storing the point cloud...");
//        fflush(stdout);

          reprojectImageTo3D(disp, recons3D, Q, true);
        //reprojectImageTo3D( disp, xyz, Q, false, CV_32F );

//        const double max_z = 1.0e4;
//        FILE* fp = fopen(filename, "wt");
//        for(int y = 0; y < recons3D.rows; y++)
//        {
//            for(int x = 0; x < recons3D.cols; x++)
//            {
//                Vec3f point = recons3D.at<Vec3f>(y, x);
//                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
//                fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
//            }
//        }
//        fclose(fp);
//        printf("\n");

    }


    void createPointCloud (Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr){


        Size img_size = img1.size();


        /// Q NAIVE
//        Mat Q1(4,4, CV_64FC1);
//        Q1.at<double>(0,0)=1;
//        Q1.at<double>(0,1)=0;
//        Q1.at<double>(0,2)=0;
//        Q1.at<double>(0,3)=-0.5*img_size.width;
//        Q1.at<double>(1,0)=0;
//        Q1.at<double>(1,1)=-1;
//        Q1.at<double>(1,2)=0;
//        Q1.at<double>(1,3)=0.5*img_size.height;
//        Q1.at<double>(2,0)=0;
//        Q1.at<double>(2,1)=0;
//        Q1.at<double>(2,2)=0;
//        Q1.at<double>(2,3)=0.8*img_size.width;
//        Q1.at<double>(3,0)=0;
//        Q1.at<double>(3,1)=0;
//        Q1.at<double>(3,2)=1;
//        Q1.at<double>(3,3)=0;
//        double Q03, Q13, Q23, Q32, Q33;
//        Q03 = Q1.at<double>(0,3);
//        Q13 = Q1.at<double>(1,3);
//        Q23 = Q1.at<double>(2,3);
//        Q32 = Q1.at<double>(3,2);
//        Q33 = Q1.at<double>(3,3);
//        reprojectImageTo3D(disp, recons3D, Q1, true);


        double Q03, Q13, Q23, Q32, Q33;
        Q03 = Q.at<double>(0,3);
        Q13 = Q.at<double>(1,3);
        Q23 = Q.at<double>(2,3);
        Q32 = Q.at<double>(3,2);
        Q33 = Q.at<double>(3,3);
        reprojectImageTo3D(disp, recons3D, Q, true);


        double px, py, pz;
        uchar pr, pg, pb;

        for (int i = 0; i < img1.rows; i++) {

            uchar* rgb_ptr = img1.ptr<uchar>(i);

            uchar* disp_ptr = disp.ptr<uchar>(i);

            double* recons_ptr = recons3D.ptr<double>(i);

            for (int j = 0; j < img1.cols; j++) {

                //Get 3D coordinates

                uchar d = disp_ptr[j];
                if ( d == 0 ) continue; //Discard bad pixels
                double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
                px = static_cast<double>(j) + Q03;
                py = static_cast<double>(i) + Q13;
                pz = Q23;

                px = px/pw;
                py = py/pw;
                pz = pz/pw;

                //Get RGB info
                pb = rgb_ptr[3*j];
                pg = rgb_ptr[3*j+1];
                pr = rgb_ptr[3*j+2];

                //Insert info into point cloud structure
                pcl::PointXYZRGB point;
                point.x = px;
                point.y = py;
                point.z = pz;

                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                        static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                point.rgb = *reinterpret_cast<float*>(&rgb);
                point_cloud_ptr->points.push_back (point);
            }
        }
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

 }
}
