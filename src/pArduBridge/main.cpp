/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: main.cpp, Cambridge MA                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "ArduBridge.h"
#include "ArduBridge_Info.h"

using namespace std;


#include <log.h>
#include <filesystem>


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

  // Retrieve the current working directory
  std::filesystem::path current_path = std::filesystem::current_path();
  
  // Construct the full path for the log file
  std::filesystem::path log_file_path = current_path / "output.log";

  // Set the log file with the full path
  mavsdk::set_log_file(log_file_path.string());

  // Example log output
  mavsdk::get_log_stream() << "This is a test log entry." << std::endl;
  std::cout << "This is a test log entry." << std::endl;

  // Ensure the output is flushed to the file
  // mavsdk::get_log_stream().flush();

  ArduBridge.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}

