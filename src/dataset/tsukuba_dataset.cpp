//
// Created by hunter on 22/03/15.
//

#include "tsukuba_dataset.h"


#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"



// local includes
#include "util.hpp"
#include "../logger/log.h"


namespace cv {

    namespace datasets {

        using namespace std;

        class tsukuba_datasetImp : public tsukuba_dataset {

        public:
            tsukuba_datasetImp() {}

            virtual ~tsukuba_datasetImp() {}

            virtual void load(const string &path);

            virtual FramePair load_stereo_images(const int num_img);
            virtual cv::Mat loadImage(const int num_img);
            virtual cv::Mat load_disparity(const int img_num);


        private:
            void loadDataset(const string &path);
        };

        void tsukuba_datasetImp::load(const string &path) {
            loadDataset(path);
        }

        void tsukuba_datasetImp::loadDataset(const string &path) {

            dataset_path = path;

            train.push_back(vector<Ptr<Object> >());
            test.push_back(vector<Ptr<Object> >());
            validation.push_back(vector<Ptr<Object> >());

            string name(path.substr(0, path.length() - 1));
            size_t start = name.rfind('/');
            name = name.substr(start + 1, name.length() - start);

            string parName(path + "par.txt");

            FILE_LOG(logINFO) << "Loading dataset : " << name << " in path: " << path;

            ifstream infile(parName.c_str());
            string imageName;

            while (infile >> imageName) {
                Ptr<tsukuba_datasetObj> curr(new tsukuba_datasetObj);
                curr->imageName = imageName;

                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        infile >> curr->k(i, j);
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        infile >> curr->r(i, j);
                    }
                }
                for (int i = 0; i < 3; ++i) {

                    infile >> curr->tl[i];
                }
                for (int i = 0; i < 3; ++i) {

                    infile >> curr->tr[i];
                }

                train.back().push_back(curr);
            }
        }

        cv::Mat tsukuba_datasetImp::loadImage(const int img_num) {
            cv::Mat img;

            return img;

        }

        cv::Mat tsukuba_datasetImp::load_disparity(const int img_num) {

            std::ostringstream ss;
            ss << std::setw(5) << std::setfill('0') << img_num;
//            ss << img_num;
//            std::string img_path = dataset_path + "disparity_maps/left/frame_"+ ss.str() +".png";
            std::string img_path = dataset_path + "disparity_maps/left/tsukuba_disparity_L_"+ ss.str() +".png";

            return imread(img_path,0);
        }

            /*
                input : images number img1_num, img2_num
                output: cv:Mat matrices
            */
        FramePair tsukuba_datasetImp::load_stereo_images(const int img_num) {


            std::ostringstream ss;
            ss << std::setw(5) << std::setfill('0') << img_num;
//            ss << img_num;


//            std::string img_left_path = dataset_path + "daylight/left/frame_"+ ss.str() +".png";
            std::string img_left_path = dataset_path + "daylight/left/tsukuba_daylight_L_"+ ss.str() + ".png";
//            std::string img_right_path = dataset_path + "daylight/right/frame_"+ ss.str() +".png";
            std::string img_right_path = dataset_path + "daylight/right/tsukuba_daylight_R_"+ ss.str() +".png";

            FILE_LOG(logINFO) << " loading " << img_left_path << " and " << img_right_path;


            cv::Mat img_left, img_right;

            img_left = imread(img_left_path);
            img_right = imread(img_right_path);

            FramePair pair;

            pair.frame_left = img_left;
            pair.frame_right = img_right;

            return pair;

        }

        Ptr<tsukuba_dataset> tsukuba_dataset::create() {
            return Ptr<tsukuba_datasetImp>(new tsukuba_datasetImp);
        }

    }
}
