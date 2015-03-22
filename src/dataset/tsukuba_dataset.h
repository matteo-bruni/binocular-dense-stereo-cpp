//
// Created by hunter on 22/03/15.
//

#ifndef _BINOCULAR_DENSE_STEREO_TSUKUBA_DATASET_H_
#define _BINOCULAR_DENSE_STEREO_TSUKUBA_DATASET_H_


#include <string>
#include <vector>

#include "dataset.hpp"
#include <tuple>
#include <opencv2/core/core.hpp>

namespace cv {

    namespace datasets {

        class CV_EXPORTS tsukuba_dataset : public Dataset {

            public:
                virtual void load(const std::string &path) = 0;
                static Ptr<tsukuba_dataset> create();

                virtual cv::Mat loadImage(const int img_num) = 0;
                virtual FramePair load_stereo_images(const int img_num) = 0;

                virtual cv::Mat load_disparity(const int img_num) = 0;

        };

        struct tsukuba_datasetObj : public Object {

            std::string imageName;
            Matx33d k;
            Matx33d r;
            double tl[3];
            double tr[3];
        };

    }

}

#endif //_BINOCULAR_DENSE_STEREO_TSUKUBA_DATASET_H_

