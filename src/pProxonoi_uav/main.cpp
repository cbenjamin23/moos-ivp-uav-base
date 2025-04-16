/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: December 25th, 2019                                  */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "Proxonoi.h"
#include "Proxonoi_Info.h"


#include "Logger.h"


std::string extractVname(const std::string& missionFile) {
  size_t start = missionFile.find("targ_") + 5; // 5 = length of "targ_"
  size_t end = missionFile.find(".moos", start);

  // Extract the substring containing vehicle_name
  if (start != std::string::npos && end != std::string::npos) {
      return missionFile.substr(start, end - start);
  }

  return "";
}

using namespace std;
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
  cout << "pProxonoi launching as " << run_command << endl;
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
  std::string save_path = std::string(home_dir) + "/moos-ivp-uav/missions/pProxonoi_uav_" + extractVname(mission_file) + ".log";
  Logger::configure(save_path);

  Proxonoi Proxonoi;
  Proxonoi.Run(run_command.c_str(), mission_file.c_str());
  return(0);
}






