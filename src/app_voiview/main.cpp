/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: main.cpp                                             */
/*    DATE: Dec 22nd, 2019                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
  
#include <cstdlib> 
#include <vector>
#include <iostream>
#include <FL/Fl.H>
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"
#include "VOI_GUI.h"

using namespace std; 
   
void showHelpAndExit();
 
//--------------------------------------------------------
// Procedure: idleProc() 

void idleProc(void *) 
{
  Fl::flush();
  millipause(10);
}
   
//--------------------------------------------------------
// Procedure: main()
  
int main(int argc, char *argv[])
{
  Fl::add_idle(idleProc);
 
  VOI_GUI  gui(1000, 800, "MIT Voronoi Viewer");

  for(int i=1; i<argc; i++) { 
    string argi  = argv[i];

    bool handled = true;
    if((argi == "-h") || (argi == "--help"))
      showHelpAndExit();
     
    else if((argi == "-v") || (argi == "--verbose"))
      gui.addConfigParam(argi);
    else if(strEnds(argi, ".tif"))
      gui.addConfigParam(argi);
    else if((argi == "--soj") || (argi == "--mit"))
      gui.addConfigParam(argi);
    else if(argi == "--sojd") 
      gui.addConfigParam(argi);
    else if(strBegins(argi, "--amt=")) 
      gui.addConfigParam(argi);

    else if(strBegins(argi, "--poly=")) 
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--dist="))  
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--poly_fill_color=")) 
      gui.addPostConfigParam(argi);      
    else if(strBegins(argi, "--poly_fill_trans="))
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--poly_vert_color=")) 
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--poly_vert_size="))
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--poly_edge_color=")) 
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--point_color=")) 
      gui.addPostConfigParam(argi);
    else if(strBegins(argi, "--point_size=")) 
      gui.addPostConfigParam(argi);

    if(!handled) {
      cout << "voiview: Bad Arg: " << argi << endl;
      exit(1);
    }      
  }

  gui.m_voi_viewer->handleConfigParams();
  gui.m_voi_viewer->handlePostConfigParams();
  
  gui.updateXY();

  cout << "Starting voiview..." << endl;
  return Fl::run();
}

 
//--------------------------------------------------------
// Procedure: showHelpAndExit() 

void showHelpAndExit()
{
  cout << "Usage                                               " << endl;
  cout << "  voiview [OPTIONS] image.tif                       " << endl;
  cout << "                                                    " << endl;
  cout << "Synopsis:                                           " << endl;
  cout << "  The voiview utility renders a convex polygon with " << endl;
  cout << "  several randomly placed interior vertices. It will" << endl;
  cout << "  render the Voronoi distribution of the vertices   " << endl;
  cout << "  and allow the user to step through variants of    " << endl;
  cout << "  Llyod's algorithm to move the vertices to a more  " << endl;
  cout << "  equitable distribution. The user may add or delete" << endl;
  cout << "  vertices and re-calculate the distribution. This  " << endl;
  cout << "  app exercises much of the Voronoi library code    " << endl;
  cout << "  used in the Voronoi based behaviors and apps, with" << endl;
  cout << "  smaller overhead than full vehicle simulations.   " << endl;
  cout << "                                                    " << endl;
  cout << "Options:                                            " << endl;
  cout << "  -h,--help      Displays this help message         " << endl;
  cout << "                                                    " << endl;
  cout << "  --osx=<num>    Ownship X-position                 " << endl;
  cout << "  --osy=<num>    Ownship Y-position                 " << endl;
  cout << "  --osh=<num>    Ownship Heading                    " << endl;
  cout << "  --osv=<num>    Ownship Velocity/Speed             " << endl;
  cout << "                                                    " << endl;
  cout << "  --poly<Polygon> The obstacle                      " << endl;
  cout << "                                                    " << endl;
  cout << "  --noimg        Do not open with a background image" << endl;
  cout << "                                                    " << endl;
  cout << "Examples:                                           " << endl;
  cout << "  voiview  --poly=format=radial,x=40,y=-40,radius=40,pts=6,snap=1,label=foo --amt=8 " << endl;
  cout << "  voiview    " << endl;
  exit(0);  
}








