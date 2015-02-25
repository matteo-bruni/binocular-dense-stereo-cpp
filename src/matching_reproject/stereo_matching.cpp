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



    void computeDisparity(const int img1_num, const int img2_num, Mat& img_left, Mat& img_right,Mat& disp,int alg,Rect & roi1,Rect &roi2){


        cv::Mat img1 = stereo_util::segmentation(img_left);
        cv::Mat img2 = stereo_util::segmentation(img_right);


        std::string tipo = "BM";

        Mat g1, g2;

        cvtColor(img1, g1, CV_BGR2GRAY);
        cvtColor(img2, g2, CV_BGR2GRAY);

        FILE_LOG(logINFO) << "prima img1 " << stereo_util::infoMatrix(g1);

        if (img1_num < 32)
            stereo_util::rotate_clockwise(g1, g1, false);
        else
            stereo_util::rotate_clockwise(g1, g1, true);

        FILE_LOG(logINFO) << "dopo img1 " << stereo_util::infoMatrix(g1);


        if (img2_num < 32)
            stereo_util::rotate_clockwise(g2, g2, false);
        else
            stereo_util::rotate_clockwise(g2, g2, true);
        
//        imshow("postsegme", g1);
//        imshow("postsegme2", g2);
//        imwrite("./g1.png",g1);
//        imwrite("./g2.png",g2);

        if (tipo == "BM")
        {
            
              // MICHI
//            StereoBM sbm;
//            sbm.state->SADWindowSize = 5;
//            sbm.state->numberOfDisparities = 192;
//            sbm.state->preFilterSize = 5;
//            sbm.state->preFilterCap = 51;
//            sbm.state->minDisparity = 25;
//            sbm.state->textureThreshold = 223;
//            sbm.state->uniquenessRatio = 0;
//            sbm.state->speckleWindowSize = 0;
//            sbm.state->speckleRange = 0;
//         //   sbm.state->disp12MaxDiff = 1;

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
//            StereoBM sbm;
//            sbm.state->SADWindowSize = 5;
//            sbm.state->numberOfDisparities = 160;
//            sbm.state->preFilterSize = 5;
//            sbm.state->preFilterCap = 11;
//            sbm.state->minDisparity = 6;
//            sbm.state->textureThreshold = 173;
//            sbm.state->uniquenessRatio = 0;
//            sbm.state->speckleWindowSize = 0;
//            sbm.state->speckleRange = 0;
//            sbm.state->disp12MaxDiff = 1;

            StereoBM sbm;
            sbm.state->SADWindowSize = 5;
            sbm.state->numberOfDisparities = 224;
            sbm.state->preFilterSize = 31;
            sbm.state->preFilterCap = 59;
            sbm.state->minDisparity = -4;
            sbm.state->textureThreshold = 182;
            sbm.state->uniquenessRatio = 0;
            sbm.state->speckleWindowSize = 0;
            sbm.state->speckleRange = 0;
            sbm.state->disp12MaxDiff = 1;           

            sbm(g1, g2, disp, CV_32F);
      
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

//            StereoSGBM sbm;
//            sbm.SADWindowSize = 3;
//            sbm.numberOfDisparities = 144;
//            sbm.preFilterCap = 63;
//            sbm.minDisparity = -39;
//            sbm.uniquenessRatio = 10;
//            sbm.speckleWindowSize = 100;
//            sbm.speckleRange = 32;
//            sbm.disp12MaxDiff = 1;
//            sbm.fullDP = false;
//            sbm.P1 = 216;
//            sbm.P2 = 864;
//            sbm(g1, g2, disp);
//            sbm(g1, g2, d\ispar);
        }

        FILE_LOG(logINFO) << "prima dispsize " << stereo_util::infoMatrix(disp);

        if (img1_num < 32)
            stereo_util::rotate_clockwise(disp, disp, true);
        else
            stereo_util::rotate_clockwise(disp, disp, false);

        FILE_LOG(logINFO) << "dopo dispsize " << stereo_util::infoMatrix(disp);


        Mat dispSGBMn, dispSGBMheat;
        normalize(disp, dispSGBMn, 0, 255, CV_MINMAX, CV_8U); // form 0-255
        equalizeHist(dispSGBMn, dispSGBMn);
        //imshow( "WindowDispSGBM", dispSGBMn );

        applyColorMap(dispSGBMn, dispSGBMheat, COLORMAP_JET);
        imshow( "WindowDispSGBMheat", dispSGBMheat );
        fflush(stdout);
        waitKey();
        destroyAllWindows();


        // APPLY OPENING
//        cv::Mat const structure_elem = cv::getStructuringElement(
//                cv::MORPH_RECT, cv::Size(3, 3));
//        cv::morphologyEx(disp, disp,
//                cv::MORPH_OPEN, structure_elem);

//        // normalize
//        normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
//        // equalize
//        equalizeHist(disp, disp);     // ausreisser nicht darstellen // remove outliers
//        Mat dispSGBMheat;
//        applyColorMap(disp, dispSGBMheat, COLORMAP_JET);
//
//        namedWindow( "WindowDispSGBMheat", WINDOW_AUTOSIZE );// Create a window for display.
//        imshow( "WindowDispSGBMheat", dispSGBMheat );
//        fflush(stdout);
//        waitKey();
//        destroyAllWindows();



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


    void createPointCloud (Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr) {


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


        // VERSIONE CUSTOM REPROJECT
//        double Q03, Q13, Q23, Q32, Q33;
//        Q03 = Q.at<double>(0, 3);
//        Q13 = Q.at<double>(1, 3);
//        Q23 = Q.at<double>(2, 3);
//        Q32 = Q.at<double>(3, 2);
//        Q33 = Q.at<double>(3, 3);
//
//        double px, py, pz;
//        uchar pr, pg, pb;
//
//        for (int i = 0; i < img1.rows; i++) {
//
//            uchar *rgb_ptr = img1.ptr<uchar>(i);
//
//            // VERSIONE CUSTOM REPROJECT
//            uchar *disp_ptr = disp.ptr<uchar>(i);
//
//
//            for (int j = 0; j < img1.cols; j++) {
//
//                //Get 3D coordinates
//                // VERSIONE CUSTOM REPROJECT
//                uchar d = disp_ptr[j];
//                if (d == 0) continue; //Discard bad pixels
//                double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
//                px = static_cast<double>(j) + Q03;
//                py = static_cast<double>(i) + Q13;
//                pz = Q23;
//
//                px = px / pw;
//                py = py / pw;
//                pz = pz / pw;
//
//                //Get RGB info
//                pb = rgb_ptr[3 * j];
//                pg = rgb_ptr[3 * j + 1];
//                pr = rgb_ptr[3 * j + 2];
//
//                //Insert info into point cloud structure
//                pcl::PointXYZRGB point;
//                point.x = static_cast<float>(px);
//                point.y = static_cast<float>(py);
//                point.z = static_cast<float>(pz);
//
//                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
//                        static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
//                point.rgb = *reinterpret_cast<float *>(&rgb);
//                point_cloud_ptr->points.push_back(point);
//            }
//        }
//        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
//        point_cloud_ptr->height = 1;





        reprojectImageTo3D(disp, recons3D, Q, true);
        FILE_LOG(logINFO) << "disp - " <<stereo_util::infoMatrix(disp);

        FILE_LOG(logINFO) << "reconst - " <<stereo_util::infoMatrix(recons3D) << " img - " << stereo_util::infoMatrix(img1);
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

                point_cloud_ptr->push_back(pcl_point_rgb);
            }


        }

        FILE_LOG(logINFO) << "Esco..";


//        cv::Mat_<float> vec(4,1);
//        for(int y=0; y<disp.rows; ++y) {
//            for(int x=0; x<disp.cols; ++x) {
//                vec(0)=x; vec(1)=y; vec(2)=disp.at<float>(y,x); vec(3)=1;
//                vec = Q*vec;
//                vec /= vec(3);
//                cv::Vec3f &point = recons3D.at<cv::Vec3f>(y,x);
////                point[0] = vec(0);
////                point[1] = vec(1);
////                point[2] = vec(2);
//                pcl::PointXYZRGB pcl_point_rgb;
//                cv::Vec3b intensity = img1.at<cv::Vec3b>(y, x); //BGR
//                uint32_t rgb = (static_cast<uint32_t>(intensity[2]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[0]));
//                pcl_point_rgb.rgb = *reinterpret_cast<float *>(&rgb);
//                pcl_point_rgb.x = vec(0);    // rgb PointCloud
//                pcl_point_rgb.y = vec(1);
//                pcl_point_rgb.z = vec(2);
//                point_cloud_ptr->push_back(pcl_point_rgb);
//
//
//            }
//        }



    }

}
