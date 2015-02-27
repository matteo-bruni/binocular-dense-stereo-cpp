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
#include <libconfig.h++>



#include <stdio.h>
#include <libconfig.h>

// local includes
#include "stereo_matching.hpp"
#include "../utils/util.hpp"
#include "../dataset/msm_middlebury.hpp"


// logger
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


        std::string tipo = "BM";

        Mat g1, g2;

        ///da provare
        cvtColor(img_left, g1, CV_BGR2GRAY);
        cvtColor(img_right, g2, CV_BGR2GRAY);

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





        libconfig::Config cfg;
        // Read the file. If there is an error, report it and exit.
        try
        {
            cfg.readFile("../config/config.cfg");
        }
        catch(const libconfig::FileIOException &fioex)
        {
            std::cerr << "I/O error while reading file." << std::endl;
            exit(EXIT_FAILURE);
        }
        catch(const libconfig::ParseException &pex)
        {
            std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                    << " - " << pex.getError() << std::endl;
            exit(EXIT_FAILURE);
        }


        if (tipo == "BM")
        {

            StereoBM sbm;

            // Get the store name.
            try
            {
                const libconfig::Setting & root = cfg.getRoot();
                const libconfig::Setting & StereoBMSettings  = root["StereoBM"];

                sbm.state->SADWindowSize = (int) StereoBMSettings["SADWindowSize"];
                sbm.state->numberOfDisparities = (int) StereoBMSettings["numberOfDisparities"];
                sbm.state->preFilterSize = (int) StereoBMSettings["preFilterSize"];
                sbm.state->preFilterCap = (int) StereoBMSettings["preFilterCap"];
                sbm.state->minDisparity = (int) StereoBMSettings["minDisparity"];
                sbm.state->textureThreshold = (int) StereoBMSettings["textureThreshold"];
                sbm.state->uniquenessRatio = (int) StereoBMSettings["uniquenessRatio"];
                sbm.state->speckleWindowSize = (int) StereoBMSettings["speckleWindowSize"];
                sbm.state->speckleRange = (int) StereoBMSettings["speckleRange"];
                sbm.state->disp12MaxDiff = (int) StereoBMSettings["disp12MaxDiff"];


            }
            catch(const libconfig::SettingNotFoundException &nfex)
            {
                sbm.state->SADWindowSize = 5;
                sbm.state->numberOfDisparities = 192;
                sbm.state->preFilterSize = 5;
                sbm.state->preFilterCap = 51;
                sbm.state->minDisparity = 25;
                sbm.state->textureThreshold = 223;
                sbm.state->uniquenessRatio = 0;
                sbm.state->speckleWindowSize = 0;
                sbm.state->speckleRange = 0;
                sbm.state->disp12MaxDiff = 0;

            }

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


        bool show_disparity_smooth = false;
        cv::Mat disp_smooth;

        try
        {
            const libconfig::Setting & root = cfg.getRoot();
            const libconfig::Setting & SmoothingSettings  = root["Smoothing"];

            cv::bilateralFilter ( disp, disp_smooth,
                    (int) SmoothingSettings["sigmaColor"],
                    (double) SmoothingSettings["sigmaSpace"],
                    (double) SmoothingSettings["borderType"] );

            show_disparity_smooth = (bool) SmoothingSettings["show"];

        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            cv::bilateralFilter ( disp, disp_smooth, 9, 60, 30 );
        }



        if (show_disparity_smooth){

            Mat dispSGBMn,dispSGBMnSmooth, dispSGBMheat, dispSGBMheatSmooth;

            // prepare disparity
            normalize(disp, dispSGBMn, 0, 255, CV_MINMAX, CV_8U); // form 0-255
            equalizeHist(dispSGBMn, dispSGBMn);
            applyColorMap(dispSGBMn, dispSGBMheat, COLORMAP_JET);

            // prepare disparity smoothed
            normalize(disp_smooth, dispSGBMnSmooth, 0, 255, CV_MINMAX, CV_8U); // form 0-255
            equalizeHist(dispSGBMnSmooth, dispSGBMnSmooth);
            applyColorMap(dispSGBMnSmooth, dispSGBMheatSmooth, COLORMAP_JET);

            // create single view
            Size sz1 = dispSGBMheat.size();
            Size sz2 = dispSGBMheatSmooth.size();
            Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
            Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
            dispSGBMheat.copyTo(left);
            Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
            dispSGBMheatSmooth.copyTo(right);
            imshow("Disparity - Disparity Smoothed", im3);

            fflush(stdout);
            waitKey();
            destroyAllWindows();
        }


        disp_smooth.copyTo(disp);


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


    void createPointCloudOpenCV (Mat& img1, Mat& img2, Mat img_1_segm, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr) {


        Size img_size = img1.size();


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

//                if (img1.at<uchar>(rows, cols) == 0)
                    point_cloud_ptr->push_back(pcl_point_rgb);
            }


        }

        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        FILE_LOG(logINFO) << "Esco..";




    }

    void createPointCloudCustom (Mat& img1, Mat& img2, Mat img_1_segm, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr) {

        //     VERSIONE CUSTOM REPROJECT
        double Q03, Q13, Q23, Q32, Q33;
        Q03 = Q.at<double>(0, 3);
        Q13 = Q.at<double>(1, 3);
        Q23 = Q.at<double>(2, 3);
        Q32 = Q.at<double>(3, 2);
        Q33 = Q.at<double>(3, 3);

        double px, py, pz;
        uchar pr, pg, pb;

        for (int i = 0; i < img1.rows; i++) {

            uchar *rgb_ptr = img1.ptr<uchar>(i);

            // VERSIONE CUSTOM REPROJECT
            uchar *disp_ptr = disp.ptr<uchar>(i);


            for (int j = 0; j < img1.cols; j++) {

                //Get 3D coordinates
                // VERSIONE CUSTOM REPROJECT
                uchar d = disp_ptr[j];
                if (d == 0) continue; //Discard bad pixels
                double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
                px = static_cast<double>(j) + Q03;
                py = static_cast<double>(i) + Q13;
                pz = Q23;

                px = px / pw;
                py = py / pw;
                pz = pz / pw;

                //Get RGB info
                pb = rgb_ptr[3 * j];
                pg = rgb_ptr[3 * j + 1];
                pr = rgb_ptr[3 * j + 2];

                //Insert info into point cloud structure
                pcl::PointXYZRGB point;
                point.x = static_cast<float>(px);
                point.y = static_cast<float>(py);
                point.z = static_cast<float>(pz);

                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                        static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                point.rgb = *reinterpret_cast<float *>(&rgb);

        //                    if (img1.at<uchar>(i, j) == 0)
                    point_cloud_ptr->points.push_back(point);
            }
        }
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num, bool opencv_rec){

        Mat img1;
        Mat img2;

        // images path

        // load images data
        Ptr<cv::datasets::MSM_middleburyObj> data_img1 =
                static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[img1_num]);
        Ptr<cv::datasets::MSM_middleburyObj> data_img2 =
                static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[img2_num]);

        // load images
        img1 = dataset->loadImage(img1_num);
        img2 = dataset->loadImage(img2_num);

        // init
        Mat R1,R2,P1,P2,Q;

        // zero distiorsions
        Mat D1 = Mat::zeros(1, 5, CV_64F);
        Mat D2 = Mat::zeros(1, 5, CV_64F);

        // load K and R from dataset info
        Mat M1 = Mat(data_img1->k);
        Mat M2 = Mat(data_img2->k);
        Mat r1 = Mat(data_img1->r);
        Mat r2 = Mat(data_img2->r);

        // init translation vectors from dataset
        Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->t);
        Mat t2 = Mat(3, 1, CV_64FC1, &data_img2->t);

        // rotation between img2 and img1
        Mat R = r2*r1.t();
        // translation between img2 and img1
        Mat T = t1 - (R.t()*t2 );

//    double tx = atan2 (R.at<double>(3,2), R.at<double>(3,3));
//    double ty = - asin(R.at<double>(3,1));
//    double tz = atan2 (R.at<double>(2,1), R.at<double>(1,1));
//    FILE_LOG(logDEBUG) << "ROTATION " << img1_num << "-" <<img2_num<< " tx="<< tx <<" ty=" << ty << "tz= " << tz;

//    theta_x = arctan(r_{3,2}/r_{3,3})
//    \theta_y = -arcsin(r_{3,1})
//    \theta_z = arctan(r_{2,1}/r_{1,1})

        cv:Mat img1_segm_mask;

        std::tuple<cv::Mat, cv::Mat> segm_tuple1 = stereo_util::segmentation(img1);
        img1 = std::get<0>(segm_tuple1);
        img1_segm_mask = std::get<1>(segm_tuple1);

        std::tuple<cv::Mat, cv::Mat> segm_tuple2 = stereo_util::segmentation(img2);
        img2 = std::get<0>(segm_tuple2);

        FILE_LOG(logINFO) << "Rectifying images...";
        Rect roi1,roi2;
        stereo::rectifyImages(img1, img2, M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);

//    cv::Mat img_roi(img1);
//    rectangle(img_roi, roi1.tl(), roi1.br(), CV_RGB(255, 0,0), 10, 8, 0);
//    imshow("rect", img_roi );

        FILE_LOG(logINFO) << "Computing Disparity map Dense Stereo";
        Mat disp(img1.size(), CV_32F);
        FILE_LOG(logINFO) << "imgsize " << stereo_util::infoMatrix(img1);
        FILE_LOG(logINFO) << "dispsize " << stereo_util::infoMatrix(disp);

        stereo::computeDisparity(img1_num, img2_num, img1, img2, disp,1,roi1,roi2);

//    stereo::display(img1, img2, disp);
//
        FILE_LOG(logINFO) << "Creating point cloud..";
        Mat recons3D(disp.size(), CV_32FC3);
        FILE_LOG(logINFO) << "recons3Dsize " << stereo_util::infoMatrix(recons3D);

        // stereo::storePointCloud(disp, Q, recons3D);

        //std::cout << "Creating Point Cloud..." <<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

        if (opencv_rec)
            stereo::createPointCloudOpenCV(img1, img2, img1_segm_mask, Q, disp, recons3D, point_cloud_ptr);
        else
            stereo::createPointCloudCustom(img1, img2, img1_segm_mask, Q, disp, recons3D, point_cloud_ptr);

        return point_cloud_ptr;

    }



    void createAllClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds){


        int img1_num=1;
        int img2_num=2;


        std::stringstream ss;
        std::string path;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generatePointCloud(dataset, img1_num, img2_num,true);
        clouds.push_back(cloud);

        pcl::io::savePCDFileASCII ("./cloud1.pcd", *cloud);
        unsigned int dataset_size = (unsigned int)dataset->getTrain().size();


        for(std::vector<std::tuple<int,int>>::iterator it = dataset->getAssociation().begin();
            it != dataset->getAssociation().end(); ++it) {
            img1_num = std::get<0>(*it);
            img2_num = std::get<1>(*it);
            cloud = generatePointCloud(dataset, img1_num, img2_num,true);
            if(!(*cloud).empty()){
                clouds.push_back(cloud);
                ss.str( std::string() );
                ss.clear();
                ss << img1_num<< "-" << img2_num ;
                path = "./cloud"+ ss.str() +".pcd";
                pcl::io::savePCDFileASCII (path, *cloud);
            }

            /* std::cout << *it; ... */
        }

        FILE_LOG(logINFO) << "cloud size" <<clouds.size();

    }

}
