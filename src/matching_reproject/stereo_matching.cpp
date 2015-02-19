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

        FILE_LOG(logINFO) << util::infoMatrix(T);


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



    void computeDisparity(Mat& img1, Mat& img2,Mat& disp,int alg,Rect & roi1,Rect &roi2){


        /////////////////


        std::string tipo = "SGBM";


        Mat g1, g2;

        cvtColor(img1, g1, CV_BGR2GRAY);
        cvtColor(img2, g2, CV_BGR2GRAY);

        if (tipo == "BM")
        {
            StereoBM sbm;
            sbm.state->SADWindowSize = 5;
            sbm.state->numberOfDisparities = 160;
            sbm.state->preFilterSize = 5;
            sbm.state->preFilterCap = 11;
            sbm.state->minDisparity = -68;
            sbm.state->textureThreshold = 130;
            sbm.state->uniquenessRatio = 0;
            sbm.state->speckleWindowSize = 0;
            sbm.state->speckleRange = 0;
            sbm.state->disp12MaxDiff = 1;
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

//        imshow("left", img1);
//        imshow("right", img2);
//        imshow("disp", disp);


        ///////////////
//        enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
//        StereoBM bm;
//        StereoSGBM sgbm;
//        StereoVar var;
//        int SADWindowSize = 0, numberOfDisparities = 0;
//        Size img_size = img1.size();
//
//        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
//
//        sgbm.preFilterCap = 60;//63;
//        sgbm.SADWindowSize = 5; //SADWindowSize > 0 ? SADWindowSize : 3;
//
//
//        bm.state->roi1 = roi1;
//        bm.state->roi2 = roi2;
//        bm.state->preFilterCap = 5;
//        bm.state->SADWindowSize = 5;
//        bm.state->minDisparity = -68;
//        bm.state->numberOfDisparities = 160;
//        bm.state->textureThreshold = 130;
//        bm.state->uniquenessRatio = 0;
//        bm.state->speckleWindowSize = 100;
//        bm.state->speckleRange = 0;
//        bm.state->disp12MaxDiff = 0;
//
//        int cn = img1.channels();
//
//        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//        sgbm.minDisparity = 1;//0;
//        sgbm.numberOfDisparities = 112;//numberOfDisparities;
//        sgbm.uniquenessRatio = 11;//10;
//        sgbm.speckleWindowSize = 0; //bm.state->speckleWindowSize;
//        sgbm.speckleRange = 0;//bm.state->speckleRange;
//        sgbm.disp12MaxDiff = 1;//
//        sgbm.fullDP = alg == STEREO_HH;// FALSE
//
////        var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
////        var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
////        var.nIt = 25;
////        var.minDisp = -numberOfDisparities;
////        var.maxDisp = 0;
////        var.poly_n = 3;
////        var.poly_sigma = 0.0;
////        var.fi = 15.0f;
////        var.lambda = 0.03f;
////        var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
////        var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
////        var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
//
//      //  Mat disp8;
//        //Mat img1p, img2p, dispp;
//        //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//        //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//
//        int64 t = getTickCount();
//
//        //sgbm(img1, img2, disp);
//
//        bm(img1, img2, disp,CV_16S);
//
//
//        t = getTickCount() - t;
//
//        FILE_LOG(logINFO) << "Time elapsed: " << t*1000/getTickFrequency()<< "ms";
//
//
//        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
//        if( alg != STEREO_VAR )
//            disp.convertTo(disp, CV_8U, 255/(numberOfDisparities*16.));
//        else
//            disp.convertTo(disp, CV_8U);


    }

    void display(Mat& img1, Mat& img2,Mat& disp){

        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp);
        printf("press any key to continue...");
        fflush(stdout);
        waitKey();
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


//        Mat Q1;
//
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

        reprojectImageTo3D(disp, recons3D, Q, true);


        double Q03, Q13, Q23, Q32, Q33;
        Q03 = Q.at<double>(0,3);
        Q13 = Q.at<double>(1,3);
        Q23 = Q.at<double>(2,3);
        Q32 = Q.at<double>(3,2);
        Q33 = Q.at<double>(3,3);

//    Q = np.float32([[1, 0, 0, -0.5*width],
//    [0,-1, 0,  0.5*height], # turn points 180 deg around x-axis,
//    [0, 0, 0,  0.8*width], # so that y-axis looks up
//    [0, 0, 1,   0]])
//

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
//
//int main(int argc, char** argv)
//{
//
//    /*
//
//
//    const char* algorithm_opt = "--algorithm=";
//    const char* maxdisp_opt = "--max-disparity=";
//    const char* blocksize_opt = "--blocksize=";
//    const char* nodisplay_opt = "--no-display";
//    const char* scale_opt = "--scale=";
//
//    if(argc < 3)
//    {
//        print_help();
//        return 0;

//    }
//    const char* img1_filename = 0;
//    const char* img2_filename = 0;
//    const char* intrinsic_filename = 0;
//    const char* extrinsic_filename = 0;
//    const char* disparity_filename = 0;
//    const char* point_cloud_filename = 0;
//
//    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
//    int alg = STEREO_SGBM;
//    int SADWindowSize = 0, numberOfDisparities = 0;
//    bool no_display = false;
//    float scale = 1.f;
//
//    StereoBM bm;
//    StereoSGBM sgbm;
//    StereoVar var;
//
//    for( int i = 1; i < argc; i++ )
//    {
//        if( argv[i][0] != '-' )
//        {
//            if( !img1_filename )
//                img1_filename = argv[i];
//            else
//                img2_filename = argv[i];
//        }
//        else if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
//        {
//            char* _alg = argv[i] + strlen(algorithm_opt);
//            alg = strcmp(_alg, "bm") == 0 ? STEREO_BM :
//                    strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
//                            strcmp(_alg, "hh") == 0 ? STEREO_HH :
//                                    strcmp(_alg, "var") == 0 ? STEREO_VAR : -1;
//            if( alg < 0 )
//            {
//                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
//                print_help();
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
//                    numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
//            {
//                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
//                print_help();
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
//                    SADWindowSize < 1 || SADWindowSize % 2 != 1 )
//            {
//                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
//            {
//                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
//                return -1;
//            }
//        }
//        else if( strcmp(argv[i], nodisplay_opt) == 0 )
//            no_display = true;
//        else if( strcmp(argv[i], "-i" ) == 0 )
//            intrinsic_filename = argv[++i];
//        else if( strcmp(argv[i], "-e" ) == 0 )
//            extrinsic_filename = argv[++i];
//        else if( strcmp(argv[i], "-o" ) == 0 )
//            disparity_filename = argv[++i];
//        else if( strcmp(argv[i], "-p" ) == 0 )
//            point_cloud_filename = argv[++i];
//        else
//        {
//            printf("Command-line parameter error: unknown option %s\n", argv[i]);
//            return -1;
//        }
//    }
//
//    if( !img1_filename || !img2_filename )
//    {
//        printf("Command-line parameter error: both left and right images must be specified\n");
//        return -1;
//    }
//
//    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
//    {
//        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
//        return -1;
//    }
//
//    if( extrinsic_filename == 0 && point_cloud_filename )
//    {
//        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
//        return -1;
//    }
//
//    int color_mode = alg == STEREO_BM ? 0 : -1;
//
//
//    */
//
//
//
//    Mat img1 = imread(img1_filename, color_mode);
//    Mat img2 = imread(img2_filename, color_mode);
//
//     if(1.f != scale)
//    {
//      Mat temp1, temp2;
//      int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//      resize(img1, temp1, Size(), scale, scale, method);
//      img1 = temp1;
//      resize(img2, temp2, Size(), scale, scale, method);
//      img2 = temp2;
//     }
//
//
//
//
//
//    ////////////// rectify, Q,p1 p2 , rimappaimg rettificate
//
//    Size img_size = img1.size();
//
//    Rect roi1, roi2;
//    Mat Q;
//
//    Mat M1, D1, M2, D2;
//
//    M1 *= scale;
//    M2 *= scale;
//
//
//    Mat R, T, R1, P1, R2, P2;
//
//
//    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
//
//    Mat map11, map12, map21, map22;
//    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
//
//    Mat img1r, img2r;
//    remap(img1, img1r, map11, map12, INTER_LINEAR);
//    remap(img2, img2r, map21, map22, INTER_LINEAR);
//
//
//    img1 = img1r;
//    img2 = img2r;
//
//    ///////////////
//
//    /////////////start disparity
//
//    //Get the interesting parameters from Q
//
//    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
//
//    bm.state->roi1 = roi1;
//    bm.state->roi2 = roi2;
//    bm.state->preFilterCap = 31;
//    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
//    bm.state->minDisparity = 0;
//    bm.state->numberOfDisparities = numberOfDisparities;
//    bm.state->textureThreshold = 10;
//    bm.state->uniquenessRatio = 15;
//    bm.state->speckleWindowSize = 100;
//    bm.state->speckleRange = 32;
//    bm.state->disp12MaxDiff = 1;
//
//    sgbm.preFilterCap = 63;
//    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
//
//    int cn = img1.channels();
//
//    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//    sgbm.minDisparity = 0;
//    sgbm.numberOfDisparities = numberOfDisparities;
//    sgbm.uniquenessRatio = 10;
//    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
//    sgbm.speckleRange = bm.state->speckleRange;
//    sgbm.disp12MaxDiff = 1;
//    sgbm.fullDP = alg == STEREO_HH;
//
//    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
//    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
//    var.nIt = 25;
//    var.minDisp = -numberOfDisparities;
//    var.maxDisp = 0;
//    var.poly_n = 3;
//    var.poly_sigma = 0.0;
//    var.fi = 15.0f;
//    var.lambda = 0.03f;
//    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
//    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
//    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
//
//    Mat disp, disp8;
//    //Mat img1p, img2p, dispp;
//    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//
//    int64 t = getTickCount();
//    if( alg == STEREO_BM )
//        bm(img1, img2, disp);
//    else if( alg == STEREO_VAR ) {
//        var(img1, img2, disp);
//    }
//    else if( alg == STEREO_SGBM || alg == STEREO_HH )
//        sgbm(img1, img2, disp);
//    t = getTickCount() - t;
//    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
//
//    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
//    if( alg != STEREO_VAR )
//        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
//    else
//        disp.convertTo(disp8, CV_8U);
//
//
//
//
//    //////////////////
//
//
//    if( !no_display )
//    {
//        namedWindow("left", 1);
//        imshow("left", img1);
//        namedWindow("right", 1);
//        imshow("right", img2);
//        namedWindow("disparity", 0);
//        imshow("disparity", disp8);
//        printf("press any key to continue...");
//        fflush(stdout);
//        waitKey();
//        printf("\n");
//    }
//
//
//
//
//    printf("storing the point cloud...");
//    fflush(stdout);
//    Mat xyz;
//    reprojectImageTo3D(disp, xyz, Q, true);
//    //reprojectImageTo3D( disp, xyz, Q, false, CV_32F );
//
//    saveXYZ(point_cloud_filename, xyz);
//    printf("\n");
//
//
//
//    return 0;
//}