/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VOI_GUI.h                                            */
/*    DATE: Dec 20th 2019                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef VOI_GUI_HEADER
#define VOI_GUI_HEADER

#include <vector>
#include <FL/Fl.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>
#include "XYPolygon.h"
#include "MarineGUI.h"
#include "VOI_Viewer.h"

class VOI_GUI : public MarineGUI {
public:
  VOI_GUI(int w, int h, const char *l=0);
  ~VOI_GUI() {};
  
  void  resize(int, int, int, int);
  void  updateXY();
  int   handle(int);

  bool  addPoly(std::string);

  void  addConfigParam(std::string);
  void  addPostConfigParam(std::string);
  
 protected:
  void  augmentMenu();
  void  initWidgets();
  void  resizeWidgetsShape();
  void  resizeWidgetsText();

 public:

  VOI_Viewer *m_voi_viewer;

 protected:

  // Panel - Column ONE 
  Fl_Output  *m_fld_tot_dist;
  Fl_Output  *m_fld_min_dist;
  Fl_Output  *m_fld_max_dist;
  Fl_Output  *m_fld_point_cnt;

  Fl_Output  *m_fld_poly;
  Fl_Output  *m_fld_neigh;
  Fl_Output  *m_fld_algorithm;
  
  // Panel - Column TWO
  Fl_Button  *m_but_reset;
  Fl_Button  *m_but_regen;

  // Panel - Column THREE 
  Fl_Button  *m_but_alg_center;
  Fl_Button  *m_but_alg_centroid;

  // Panel - Column FOUR
  Fl_Button  *m_but_alg_hybrid;
  Fl_Button  *m_but_alg_currpos;

  // Panel - Column SIX
  Fl_Button  *m_but_step_1;
  Fl_Button  *m_but_step_10;
  Fl_Button  *m_but_step_50;
  Fl_Button  *m_but_step_150;

  // Panel - Column SEVEN
  Fl_Output  *m_fld_avg_size;
  Fl_Output  *m_fld_max_size;
  Fl_Output  *m_fld_min_size;
  Fl_Output  *m_fld_std_size;
  
  //=================================================
  // BOTTOM Panel
  //=================================================

 private:
  inline void cb_CenterView_i();
  static void cb_CenterView(Fl_Widget*);
  
  inline void cb_RotatePoly_i(int);
  static void cb_RotatePoly(Fl_Widget*, int);

  inline void cb_ResizePoly_i(int);
  static void cb_ResizePoly(Fl_Widget*, int);

  inline void cb_AltPolyX_i(int);
  static void cb_AltPolyX(Fl_Widget*, int);

  inline void cb_AltPolyY_i(int);
  static void cb_AltPolyY(Fl_Widget*, int);

  // ------------------------------------------
  inline void cb_Reset_i();
  static void cb_Reset(Fl_Widget*);

  inline void cb_Regenerate_i();
  static void cb_Regenerate(Fl_Widget*);

  // ------------------------------------------
  inline void cb_ModAlgorithm_i(int);
  static void cb_ModAlgorithm(Fl_Widget*, int);

  // ------------------------------------------
  inline void cb_AltSolutionSpeed_i(int);
  static void cb_AltSolutionSpeed(Fl_Widget*, int);

  // ------------------------------------------
  inline void cb_Step_i(int);
  static void cb_Step(Fl_Widget*, int);

  int m_start_wid;
  int m_start_hgt;
};
#endif







