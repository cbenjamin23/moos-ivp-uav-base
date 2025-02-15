/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VOI_Viewer.h                                         */
/*    DATE: Dec 22nd, 2019                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef VOI_VIEWER_HEADER
#define VOI_VIEWER_HEADER

#include <vector>
#include <string>
#include "FL/Fl.H"
#include "FL/fl_draw.H"
#include "BackImg.h"
#include "MarineViewer.h"
#include "XYPolygon.h"
#include "VoronoiField.h"
#include "VFieldEvaluator.h"
#include "XYFieldGenerator.h"

class VOI_GUI;

class VOI_Viewer : public MarineViewer
{
 friend class VOI_GUI;
 public:
  VOI_Viewer(int x, int y, int w, int h, const char *l=0);
  ~VOI_Viewer() {};

  // Pure virtuals that needs to be defined
  void   modColorScheme();

  // Virtuals defined
  void   draw();
  int    handle(int);
  void   handle_left_mouse(int, int);
  void   handle_right_mouse(int, int);

public:
  void   addPostConfigParam(std::string);
  void   addConfigParam(std::string);

  bool   handlePostConfigParams();
  bool   handleConfigParams();
  
  void   drawVoiPolys();
  void   drawVoiPoints();
  void   drawVoiCenterPoints();
  void   drawVoiCentroidPoints();
  void   drawVoiPushPoints();

  bool   addTiffFile(std::string);
  void   setAlgorithm(std::string alg) {m_algorithm=alg;}

  void   modSolutionSpeed(double amt);
  
  std::string getAlgorithm() const {return(m_algorithm);}

  void   generateVField();
  void   stepVField();
  
  VoronoiField getVField() const {return(m_vfield);}
  
private:
  void   initField();

  bool   setPolyFillColor(std::string);
  bool   setPolyFillTrans(std::string);
  bool   setPolyVertColor(std::string);
  bool   setPolyVertSize(std::string);
  bool   setPolyEdgeColor(std::string);
  bool   setPointColor(std::string);
  bool   setPointSize(std::string);
  
 protected: // Available as friend to VOI_GUI

  XYFieldGenerator m_fld_generator; 
  VFieldEvaluator  m_evaluator;
  VoronoiField     m_vfield;

 private: // State vars
  bool   m_field_initialized;
  
 private: // Config vars

  std::vector<std::string> m_config_params;
  std::vector<std::string> m_post_config_params;

  std::string  m_algorithm;
  double       m_solution_speed;
  
  double       m_poly_fill_trans;
  double       m_poly_vert_size;
  std::string  m_poly_fill_color;
  std::string  m_poly_vert_color;
  std::string  m_poly_edge_color;
  std::string  m_pt_color;
  double       m_pt_size;
};

#endif 





