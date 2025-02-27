/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    FILE: main.cpp                                             */
/*    DATE: Feb 2022                                             */
/*                                                               */
/*****************************************************************/

#include <iostream>
#include "MBUtils.h"
#include "OpenURL.h"
#include "ColorParse.h"
#include "FireSim.h"
#include "FireSim_Info.h"

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
    else if((argi=="-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi=="-i") || (argi == "--interface"))
      showInterfaceAndExit();
    else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    // else if((argi == "-w") || (argi == "--web") || (argi == "-web"))
      // openURLX("https://oceanai.mit.edu/ivpman/apps/uFldSwimSensor");
    else if(i == 2)
      run_command = argi;
  }
  
  if(mission_file == "")
    showHelpAndExit();

  cout << termColor("green");
  cout << "uFldFireSim launching as " << run_command << endl;
  cout << termColor() << endl;

  FireSim firesim;
  firesim.Run(run_command.c_str(), mission_file.c_str(), argc, argv);
 
  return(0);
}










