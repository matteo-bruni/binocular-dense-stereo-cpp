#ifndef BINOCULAR_DENSE_STEREO_CONFIG_H
#define BINOCULAR_DENSE_STEREO_CONFIG_H

#include <libconfig.h++>
#include <iostream>

#include "../registration/registration.hpp"

namespace binocular_dense_stereo {

    struct middleburyPair {
        int left_image = 0;
        int right_image = 0;
    };

    struct configPars {

        bool load_clouds = true;
        bool incremental = false;
        bool save_generated_clouds = false;
        bool show_single_cloud = false;
        bool show_sum_cloud = false;
        int load_n_clouds = 20;
        int first_frame = 0;
        int last_frame = 200;
        int step = 10;
        int reg_cloud_1 = 0;
        int reg_cloud_2 = 1;
    };

    enum datasetType { TSUKUBA, MIDDLEBURY, KITTI};

    class ConfigLoader {

        private:

            ConfigLoader();
            virtual ~ConfigLoader(){}

    private:
        libconfig::Config cfg;

        public:

            static ConfigLoader& get_instance() {
                // l'unica istanza della classe viene creata alla prima chiamata di get_instance()
                // e verrà distrutta solo all'uscita dal programma
                static ConfigLoader instance;
                return instance;
            }

            configPars loadGeneralConfiguration(int dtype);
            registrationParams loadRegistrationParams(int dtype);
            std::vector<middleburyPair> loadMiddleburyAssociations();

        // C++ 11
            // =======
            // We can use the better technique of deleting the methods
            // we don't want.
            ConfigLoader(ConfigLoader const&)               = delete;
            void operator=(ConfigLoader const&)  = delete;
    };


}
#endif