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

    // Retrieve the current working directory using getcwd
    char current_path[PATH_MAX];
    if (getcwd(current_path, sizeof(current_path)) != NULL) {
        std::string current_path_str(current_path);

        // Construct the full path for the log file
        std::string log_file_path = current_path_str + "/output.log";

        // Set the log file with the full path
        mavsdk::set_log_file(log_file_path);

        // Example log output
        mavsdk::get_log_stream() << "This is a test log entry." << std::endl;
        std::cout << "This is a test log entry." << std::endl;

        // Ensure the output is flushed to the file
        // mavsdk::get_log_stream().flush();
    } else {
        std::cerr << "Error retrieving current working directory" << std::endl;
        return 1;
    }

    ArduBridge.Run(run_command.c_str(), mission_file.c_str());

    return(0);
}
