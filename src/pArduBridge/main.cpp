/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                        */
/*    ORGN: MIT, Cambridge MA                                */
/*    FILE: main.cpp, Cambridge MA                           */
/*    DATE: December 29th, 1963                              */
/************************************************************/

#include <string>
#include <iostream>
#include <unistd.h>  // For getcwd
#include <limits.h>  // For PATH_MAX
#include "MBUtils.h"
#include "ColorParse.h"
#include "ArduBridge.h"
#include "ArduBridge_Info.h"

using namespace std;

#include <log.h>

int main(int argc, char *argv[])
{
    string mission_file;
    string run_command = argv[0];

    for(int i=1; i<argc; i++) {
        string argi = argv[i];
        if((argi=="-v") || (argi=="--version") || (argi=="-version"))
            showReleaseInfoAndExit();
        else if((argi=="-e") || (argi=="--example") || (argi=="-example"))
            showExampleConfigAndExit();
        else if((argi == "-h") || (argi == "--help") || (argi=="-help"))
            showHelpAndExit();
        else if((argi == "-i") || (argi == "--interface"))
            showInterfaceAndExit();
        else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
            mission_file = argv[i];
        else if(strBegins(argi, "--alias="))
            run_command = argi.substr(8);
        else if(i==2)
            run_command = argi;
    }

    if(mission_file == "")
        showHelpAndExit();

    cout << termColor("green");
    cout << "pArduBridge launching as " << run_command << endl;
    cout << termColor() << endl;

    ArduBridge ArduBridge;

    
    // Get the home directory from the environment variable
    const char* home_dir = getenv("HOME");
    if (home_dir == nullptr) {
        std::cerr << "Error: Could not get the home directory." << std::endl;
        return 1;
    }
    // Construct the full path for the log file
    std::string save_path = std::string(home_dir) + "/moos-ivp-uav/missions/MavlinkLog.log";
    std::cout << "Log file path: " << save_path << std::endl;
    
    // Set the log file with the full path
    mavsdk::set_log_file(save_path);

    // Example log output
    mavsdk::get_log_stream() << "This is a test log entry." << std::endl;
    std::cout << "This is a test log entry." << std::endl;

    
    ArduBridge.Run(run_command.c_str(), mission_file.c_str());

    return(0);
}
