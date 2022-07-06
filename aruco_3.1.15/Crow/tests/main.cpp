#include "crow.h"
//#include "crow_all.h"


int main()
{
    crow::SimpleApp app; //define your crow application
    
    //app.loglevel(crow::LogLevel::Warning); //Afficher différents niveaux de log
    //std::cout << std::system("pwd") << std::endl;//Check the directory of the executable (from linux command "pwd")
    
    //define your endpoint at the root directory
    CROW_ROUTE(app, "/")([](){ // 
    
        //Setting new template directory
        //crow::mustache::set_base("/home/rddoga/Desktop/WebSocket_test/jsmpeg");
        
        auto page = crow::mustache::load_text("view-stream.html"); // fancypage.html

       // crow::mustache::context ctx ({{"person", name}}); // 

        return page; //
    });
    
    //set the port, set the app to run on multiple threads, and run the app
    app.port(18080).multithreaded().run_async();

}
