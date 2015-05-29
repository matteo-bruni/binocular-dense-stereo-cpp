/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Itseez Inc or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "kitti_dataset.h"
#include "util.hpp"

namespace cv
{
    namespace datasets
    {

        using namespace std;

        class SLAM_kittiImp : public SLAM_kitti
        {
        public:
            SLAM_kittiImp() {}
            //SLAM_kittiImp(const string &path);
            virtual ~SLAM_kittiImp() {}

            virtual void load(const string &path);
            virtual cv::Mat loadImage(const int img_num);

            virtual FramePair load_stereo_images(const int num_img);

        private:
            void loadDataset(const string &path);
        };

/*SLAM_kittiImp::SLAM_kittiImp(const string &path)
{
    loadDataset(path);
}*/

        cv::Mat SLAM_kittiImp::loadImage(const int img_num) {
            cv::Mat img;

            return img;

        }

        void SLAM_kittiImp::load(const string &path)
        {
            loadDataset(path);
        }

        void SLAM_kittiImp::loadDataset(const string &path)
        {

            dataset_path = path;


            train.push_back(vector< Ptr<Object> >());
            test.push_back(vector< Ptr<Object> >());
            validation.push_back(vector< Ptr<Object> >());

            string pathSequence(path + "sequences/");
            vector<string> fileNames;
            getDirList(pathSequence, fileNames);
            for (vector<string>::iterator it=fileNames.begin(); it!=fileNames.end(); ++it) {
                Ptr<SLAM_kittiObj> curr(new SLAM_kittiObj);
                curr->name = *it;

                string currPath(pathSequence + curr->name);

                // loading velodyne
//                string pathVelodyne(currPath + "/velodyne/");
//                vector<string> velodyneNames;
//                getDirList(pathVelodyne, velodyneNames);
//                for (vector<string>::iterator itV=velodyneNames.begin(); itV!=velodyneNames.end(); ++itV)
//                {
//                    curr->velodyne.push_back(*itV);
//                }

                // loading gray & color images
                for (unsigned int i=0; i<=1; ++i)
                {
                    char tmp[2];
                    sprintf(tmp, "%u", i);
                    string pathImage(currPath + "/image_" + tmp + "/");
                    vector<string> imageNames;
                    getDirList(pathImage, imageNames);
                    for (vector<string>::iterator itImage=imageNames.begin(); itImage!=imageNames.end(); ++itImage)
                    {
                        curr->images[i].push_back(*itImage);
                    }
                }

                // loading times
//                ifstream infile((currPath + "/times.txt").c_str());
                string line;
//                while (getline(infile, line))
//                {
//                    curr->times.push_back(atof(line.c_str()));
//                }

                // loading calibration
                ifstream infile2((currPath + "/calib.txt").c_str());
                for (unsigned int i=0; i<2; ++i)
                {
                    getline(infile2, line);
                    vector<string> elems;
                    split(line, elems, ' ');
                    vector<string>::iterator itE=elems.begin();
                    for (++itE; itE!=elems.end(); ++itE)
                    {
                        curr->p[i].push_back(atof((*itE).c_str()));
                    }
                }

                // loading poses
                ifstream infile3((path + "poses/" + curr->name + ".txt").c_str());
                while (getline(infile3, line))
                {
                    pose p;

                    unsigned int i=0;
                    vector<string> elems;
                    split(line, elems, ' ');
                    for (vector<string>::iterator itE=elems.begin(); itE!=elems.end(); ++itE, ++i)
                    {
                        if (i>11)
                        {
                            break;
                        }
                        p.elem[i] = atof((*itE).c_str());
                    }
                    curr->posesArray.push_back(p);

                    image_datasetObj curr_calib;

                    // transform from i coord system to 0
                    curr_calib.transformMatrix = Mat::eye(4,4, CV_64F);

                    curr_calib.transformMatrix.at<double>(0,0) = p.elem[0];
                    curr_calib.transformMatrix.at<double>(0,1) = p.elem[1];
                    curr_calib.transformMatrix.at<double>(0,2) = p.elem[2];
                    curr_calib.transformMatrix.at<double>(0,3) = p.elem[3];

                    curr_calib.transformMatrix.at<double>(1,0) = p.elem[4];
                    curr_calib.transformMatrix.at<double>(1,1) = p.elem[5];
                    curr_calib.transformMatrix.at<double>(1,2) = p.elem[6];
                    curr_calib.transformMatrix.at<double>(1,3) = p.elem[7];

                    curr_calib.transformMatrix.at<double>(2,0) = p.elem[8];
                    curr_calib.transformMatrix.at<double>(2,1) = p.elem[9];
                    curr_calib.transformMatrix.at<double>(2,2) = p.elem[10];
                    curr_calib.transformMatrix.at<double>(2,3) = p.elem[11];
                    // NB last row 0 0 0 1
                    // invert to obtain H from 0 to coord system i
                    cv::Mat transformMatrix_inv = curr_calib.transformMatrix.inv();

                    // get calibration matrix from calib.txt
                    curr_calib.k(0, 0) = curr->p[0][0];
                    curr_calib.k(0, 1) = curr->p[0][1];
                    curr_calib.k(0, 2) = curr->p[0][2];
                    curr_calib.k(1, 0) = curr->p[0][4];
                    curr_calib.k(1, 1) = curr->p[0][5];
                    curr_calib.k(1, 2) = curr->p[0][6];
                    curr_calib.k(2, 0) = curr->p[0][8];
                    curr_calib.k(2, 1) = curr->p[0][8];
                    curr_calib.k(2, 2) = curr->p[0][10];

                    // get R from transformMatrix
                    curr_calib.r(0, 0) = transformMatrix_inv.at<double>(0,0);
                    curr_calib.r(0, 1) = transformMatrix_inv.at<double>(0,1);
                    curr_calib.r(0, 2) = transformMatrix_inv.at<double>(0,2);
                    curr_calib.r(1, 0) = transformMatrix_inv.at<double>(1,0);
                    curr_calib.r(1, 1) = transformMatrix_inv.at<double>(1,1);
                    curr_calib.r(1, 2) = transformMatrix_inv.at<double>(1,2);
                    curr_calib.r(2, 0) = transformMatrix_inv.at<double>(2,0);
                    curr_calib.r(2, 1) = transformMatrix_inv.at<double>(2,1);
                    curr_calib.r(2, 2) = transformMatrix_inv.at<double>(2,2);

                    // get T from transformMatrix
                    curr_calib.tl[0] = transformMatrix_inv.at<double>(0,3);
                    curr_calib.tl[1] = transformMatrix_inv.at<double>(1,3);
                    curr_calib.tl[2] = transformMatrix_inv.at<double>(2,3);



                    curr->calibArray.push_back(curr_calib);


                }


                train.back().push_back(curr);
            }
        }

        FramePair SLAM_kittiImp::load_stereo_images(const int img_num) {


            std::ostringstream ss;
            ss << std::setw(6) << std::setfill('0') << img_num;
//            ss << img_num;


//            std::string img_left_path = dataset_path + "daylight/left/frame_"+ ss.str() +".png";
            std::string img_left_path = dataset_path + "sequences/07/image_0/"+ ss.str() + ".png";
//            std::string img_right_path = dataset_path + "daylight/right/frame_"+ ss.str() +".png";
            std::string img_right_path = dataset_path + "sequences/07/image_1/"+ ss.str() + ".png";

            FILE_LOG(logINFO) << " loading " << img_left_path << " and " << img_right_path;


            cv::Mat img_left, img_right;

            img_left = imread(img_left_path);
            img_right = imread(img_right_path);

            FramePair pair;

            pair.frame_left = img_left;
            pair.frame_right = img_right;

            return pair;

        }

        Ptr<SLAM_kitti> SLAM_kitti::create()
        {
            return Ptr<SLAM_kittiImp>(new SLAM_kittiImp);
        }

    }
}