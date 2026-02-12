/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: main.cpp                                             */
/*    DATE: Feb 2025                                            */
/*****************************************************************/


#include <iostream>
#include "MBUtils.h"
#include "ColorParse.h"
#include "GridSearchViz.h"
#include "GridSearchViz_Info.h"

#include "../lib_common/Logger.h"


using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command  = argv[0];

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
  cout << "pGridSearchViz launching as " << run_command << endl;
  cout << termColor() << endl;

  
  Logger::enable();
  // Get the home directory from the environment variable
  auto home_dir = getenv("HOME");
  if (home_dir == nullptr)
  {
    Logger::error("Error: Could not get the home directory.");
    std::cerr << "Error: Could not get the home directory." << std::endl;
    return 1;
  }
  std::string save_path = std::string(home_dir) + "/moos-ivp-uav/missions/pGridSearchViz.log";
  Logger::configure(save_path);



  GridSearchViz search_grid;
  search_grid.Run(run_command.c_str(), mission_file.c_str(), argc, argv);
  
  return(0);
}










