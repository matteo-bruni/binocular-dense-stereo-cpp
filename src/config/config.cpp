#include <iostream>
#include <libconfig.h++>

#include "config.h"


class ConfigLoader {


    private:
        // ecco il costruttore privato in modo che l'utente non possa istanziare direttamante
        ConfigLoader() {

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



        };

    public:
        static ConfigLoader& get_instance() {
            // l'unica istanza della classe viene creata alla prima chiamata di get_instance()
            // e verrà distrutta solo all'uscita dal programma
            static ConfigLoader instance;


            return instance;
        }
        bool method() { return true; };




};

//
//int main() {
//    std::cout << singleton::get_instance().method() << std::endl;
//
//    return 0;
//
//}