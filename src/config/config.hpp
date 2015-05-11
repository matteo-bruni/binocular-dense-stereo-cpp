#ifndef BINOCULAR_DENSE_STEREO_CONFIG_H
#define BINOCULAR_DENSE_STEREO_CONFIG_H

#include <libconfig.h++>
#include <iostream>

namespace binocular_dense_stereo {

    struct configPars {

        bool load_clouds = true;
        bool incremental = false;
        int load_n_clouds = 20;
        int first_frame = 0;
        int last_frame = 200;
        int step = 10;
        int reg_cloud_1 = 0;
        int reg_cloud_2 = 1;
    };


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

            configPars loadGeneralConfiguration();

            // C++ 11
            // =======
            // We can use the better technique of deleting the methods
            // we don't want.
            ConfigLoader(ConfigLoader const&)               = delete;
            void operator=(ConfigLoader const&)  = delete;
    };


}
#endif