
#include "config.hpp"

namespace binocular_dense_stereo {

    // ecco il costruttore privato in modo che l'utente non possa istanziare direttamante
    ConfigLoader::ConfigLoader() {

//            libconfig::Config cfg;
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
            std::cout << "Settings loaded "<<std::endl;
    };


    configPars ConfigLoader::loadGeneralConfiguration() {
        configPars pars;
        try
        {
            const libconfig::Setting & root = cfg.getRoot();
            const libconfig::Setting & ConfigSettings  = root["Config"];

            pars.load_clouds = (bool) ConfigSettings["load_clouds"];
            pars.incremental = (bool) ConfigSettings["incremental"];
            pars.load_n_clouds = (int) ConfigSettings["load_clouds_n"];
            pars.first_frame = (int) ConfigSettings["generate_start_frame"];
            pars.last_frame = (int) ConfigSettings["generate_stop_frame"];
            pars.step = (int) ConfigSettings["generate_step"];
            pars.reg_cloud_1 = (int) ConfigSettings["reg_cloud_1"];;
            pars.reg_cloud_2 = (int) ConfigSettings["reg_cloud_2"];

        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            std::cout << nfex.what() << " Config" << std::endl;

        }
        return pars;
    }


    registrationParams ConfigLoader::loadRegistrationParams() {

        sacParams sacPars;
        try
        {
            const libconfig::Setting & root = cfg.getRoot();
            const libconfig::Setting & ConfigSettings  = root["sacParams"];

            sacPars.filter_limit = (double) ConfigSettings["filter_limit"];
            sacPars.max_sacia_iterations = (int) ConfigSettings["max_sacia_iterations"];
            sacPars.sac_max_correspondence_dist = (double) ConfigSettings["sac_max_correspondence_dist"];
            sacPars.sac_min_correspondence_dist = (double) ConfigSettings["sac_min_correspondence_dist"];

        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            std::cout << nfex.what() << "sacParams" << std::endl;

        }

        registrationParams pars;
        pars.sacPar = sacPars;

        try
        {
            const libconfig::Setting & root = cfg.getRoot();
            const libconfig::Setting & ConfigSettings  = root["registrationParams"];


            pars.leaf_size = (float) ConfigSettings["leaf_size"];
            pars.downsample_levels = (int) ConfigSettings["downsample_levels"];
            pars.downsample_decrease = (float) ConfigSettings["downsample_decrease"];
            pars.normals_radius = (double) ConfigSettings["normals_radius"];
            pars.features_radius = (double) ConfigSettings["features_radius"];


        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            std::cout << nfex.what() << "registrationParams" << std::endl;

        }

        binocular_dense_stereo::printRegistrationParams(pars);

        return pars;
    }

}

//
//int main() {
//    std::cout << singleton::get_instance().method() << std::endl;
//
//    return 0;
//
//}