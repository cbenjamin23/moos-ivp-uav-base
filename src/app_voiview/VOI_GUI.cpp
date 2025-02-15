/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VOI_GUI.cpp                                          */
/*    DATE: Dec 22nd, 2019                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "VOI_GUI.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "VoronoiUtils.h"

using namespace std;

//----------------------------------------------------------------
// Constructor

VOI_GUI::VOI_GUI(int wid, int hgt, const char *label)
  : MarineGUI(wid, hgt, label) 
{
  this->user_data((void*)(this));
  this->when(FL_WHEN_CHANGED);
  this->begin();
  // size_range(minw,minh, maxw=0,maxh=0)
  this->size_range(800,800, 4500,2800);
 
  m_start_hgt = hgt;
  m_start_wid = wid;

  augmentMenu();
  setMenuItemColors();

  initWidgets();
  resizeWidgetsShape();
  resizeWidgetsText();
  
  this->end(); 
  this->resizable(this);
  this->show();
}

//----------------------------------------------------------------
// Procedure: addConfigParam()

void VOI_GUI::addConfigParam(string param)
{
  if(!m_voi_viewer)
    return;

  m_voi_viewer->addConfigParam(param);
}

//----------------------------------------------------------------
// Procedure: addPostConfigParam()

void VOI_GUI::addPostConfigParam(string param)
{
  if(!m_voi_viewer)
    return;

  m_voi_viewer->addPostConfigParam(param);
}

//--------------------------------------------------------------- 
// Procedure: initWidgets()     

void VOI_GUI::initWidgets()
{
  Fl_Color fcolor_blue  = fl_rgb_color(140, 140, 220);
  Fl_Color fcolor_beige = fl_rgb_color(223, 219, 191);
  //Fl_Color fcolor_green = fl_rgb_color(200, 230, 190);
  //Fl_Color fcolor_dark_goldenrod  = fl_rgb_color(184, 136, 11);
  //Fl_Color fcolor_dark_goldenrodx = fl_rgb_color(136, 136, 11);

  //=================================================
  // Main Voronoi Test Viewer
  //=================================================
  m_voi_viewer = new VOI_Viewer(1, 1, 1, 1);
  m_mviewer     = m_voi_viewer;

  //=================================================
  // Column One - Odometry

  m_fld_tot_dist = new Fl_Output(0, 0, 1, 1, "Tot Dist:");
  m_fld_tot_dist->color(fcolor_beige);
  m_fld_tot_dist->clear_visible_focus();

  m_fld_max_dist = new Fl_Output(0, 0, 1, 1, "Max Dist:");
  m_fld_max_dist->color(fcolor_beige);
  m_fld_max_dist->clear_visible_focus();

  m_fld_min_dist = new Fl_Output(0, 0, 1, 1, "Min Dist:");
  m_fld_min_dist->color(fcolor_beige);
  m_fld_min_dist->clear_visible_focus();

  m_fld_point_cnt = new Fl_Output(0, 0, 1, 1, "points:");
  m_fld_point_cnt->color(fcolor_blue);
  m_fld_point_cnt->clear_visible_focus();

  // Polygon field
  m_fld_poly = new Fl_Output(0, 0, 1, 1, "poly:");
  m_fld_poly->color(fcolor_beige);
  m_fld_poly->clear_visible_focus();

  //=================================================
  // Column Two - Generation

  // Neighbor field
  m_fld_algorithm = new Fl_Output(0, 0, 1, 1, "alg:");
  m_fld_algorithm->color(fcolor_beige);
  m_fld_algorithm->clear_visible_focus();

  // Reset button
  m_but_reset = new Fl_Button(0, 0, 1, 1, "Reset");
  m_but_reset->clear_visible_focus();
  m_but_reset->callback((Fl_Callback*)VOI_GUI::cb_Reset);
  m_but_reset->shortcut('r');
  m_but_reset->tooltip("Shortcut key is 'r'");

  // Regenerate button
  m_but_regen = new Fl_Button(0, 0, 1, 1, "Regen");
  m_but_regen->clear_visible_focus();
  m_but_regen->callback((Fl_Callback*)VOI_GUI::cb_Regenerate);
  m_but_regen->shortcut('r');
  m_but_regen->tooltip("Shortcut key is 'g'");

  //=================================================
  // Column Three - Mode TO-CENTER Buttons

  // Choose Algorithm "Center"
  m_but_alg_center = new Fl_Button(0, 0, 1, 1, "Center Alg");
  m_but_alg_center->clear_visible_focus();
  m_but_alg_center->callback((Fl_Callback*)VOI_GUI::cb_ModAlgorithm,
			     (void*)1);
  m_but_alg_center->shortcut('m');
  m_but_alg_center->tooltip("Shortcut key is 'm'");

  // Choose Algorithm "Centroid"
  m_but_alg_centroid = new Fl_Button(0, 0, 1, 1, "Centroid Alg");
  m_but_alg_centroid->clear_visible_focus();
  m_but_alg_centroid->callback((Fl_Callback*)VOI_GUI::cb_ModAlgorithm,
				   (void*)2);

  //=================================================
  // Column Four 

  // Choose Algorithm "Hybrid"
  m_but_alg_hybrid = new Fl_Button(0, 0, 1, 1, "Hybrid Alg");
  m_but_alg_hybrid->clear_visible_focus();
  m_but_alg_hybrid->callback((Fl_Callback*)VOI_GUI::cb_ModAlgorithm,
			     (void*)3);

  // Choose Algorithm "Area Balance"
  m_but_alg_currpos = new Fl_Button(0, 0, 1, 1, "Area Balance Alg");
  m_but_alg_currpos->clear_visible_focus();
  m_but_alg_currpos->callback((Fl_Callback*)VOI_GUI::cb_ModAlgorithm,
			      (void*)4);

  // Neighbor field
  m_fld_neigh = new Fl_Output(0, 0, 1, 1, "neigh:");
  m_fld_neigh->color(fcolor_beige);
  m_fld_neigh->clear_visible_focus();



  //=================================================
  // Column Six - Step Buttons

  m_but_step_1 = new Fl_Button(0, 0, 1, 1, "Step 1");
  m_but_step_1->clear_visible_focus();
  m_but_step_1->callback((Fl_Callback*)VOI_GUI::cb_Step, (void*)1);

  m_but_step_10 = new Fl_Button(0, 0, 1, 1, "step 10");
  m_but_step_10->clear_visible_focus();
  m_but_step_10->callback((Fl_Callback*)VOI_GUI::cb_Step, (void*)10);

  m_but_step_50 = new Fl_Button(0, 0, 1, 1, "Step 50");
  m_but_step_50->clear_visible_focus();
  m_but_step_50->callback((Fl_Callback*)VOI_GUI::cb_Step, (void*)50);

  m_but_step_150 = new Fl_Button(0, 0, 1, 1, "Step 150");
  m_but_step_150->clear_visible_focus();
  m_but_step_150->callback((Fl_Callback*)VOI_GUI::cb_Step, (void*)150);

  //=================================================
  // Column Seven - Proxonoi Size Fields

  m_fld_avg_size = new Fl_Output(0, 0, 1, 1, "Avg Size:");
  m_fld_avg_size->color(fcolor_beige);
  m_fld_avg_size->clear_visible_focus();

  m_fld_max_size = new Fl_Output(0, 0, 1, 1, "Max Size:");
  m_fld_max_size->color(fcolor_beige);
  m_fld_max_size->clear_visible_focus();

  m_fld_min_size = new Fl_Output(0, 0, 1, 1, "Min Size:");
  m_fld_min_size->color(fcolor_beige);
  m_fld_min_size->clear_visible_focus();

  m_fld_std_size = new Fl_Output(0, 0, 1, 1, "Std Size:");
  m_fld_std_size->color(fcolor_beige);
  m_fld_std_size->clear_visible_focus();

}

//---------------------------------------------------------------------- 
// Procedure: resizeWidgetsShape()     

void VOI_GUI::resizeWidgetsShape()
{
  int extra_wid = w() - m_start_wid;
  if(extra_wid < 0)
    extra_wid = 0;
  int field_hgt = 20;

  int row0 = h() - 165;
  int row1 = row0 + 25;
  int row2 = row1 + 25;
  int row3 = row2 + 25;
  int row4 = row3 + 25;
  int row5 = row4 + 30;

  int col1_pos = 60;
  int col1_wid = 50;

  int col2_pos = col1_pos + col1_wid + 20;
  int col2_wid = 70;

  int col3_pos = col2_pos + col2_wid + 20;
  int col3_wid = 130 + (extra_wid/4);

  int col4_pos = col3_pos + col3_wid + 20;
  int col4_wid = 130 + (extra_wid/4);

  int col5_pos = col4_pos + col4_wid + 20;
  int col5_wid = 180 + (extra_wid/4);

  int col6_pos = col5_pos + col5_wid + 20;
  int col6_wid = 90 + (extra_wid/4);
  
  int col7_pos = col6_pos + col6_wid + 70;
  int col7_wid = 50;
  
  //===================================================
  // Main Viewer
  //===================================================
  m_voi_viewer->resize(0, 30, w(), h()-200);

  // Column 1  --------------------------
  int tot_x = col1_pos;
  int tot_y = row0;
  int tot_wid = 50;
  m_fld_tot_dist->resize(tot_x, tot_y, tot_wid, field_hgt);

  int max_x = col1_pos;
  int max_y = row1;
  int max_wid = 50;
  m_fld_max_dist->resize(max_x, max_y, max_wid, field_hgt);

  int min_x = col1_pos;
  int min_y = row2;
  int min_wid = 50;
  m_fld_min_dist->resize(min_x, min_y, min_wid, field_hgt);

  int int_x = col1_pos;
  int int_y = row4;
  int int_wid = 50;
  m_fld_point_cnt->resize(int_x, int_y, int_wid, field_hgt);

  int poly_x = col1_pos;
  int poly_y = row5;
  int poly_wid = w()-70;
  m_fld_poly->resize(poly_x, poly_y, poly_wid, field_hgt);

  // Column 2 --------------------------
  int bres_x = col2_pos;
  int bres_y = row0;
  int bres_wid = col2_wid;
  m_but_reset->resize(bres_x, bres_y, bres_wid, field_hgt);

  int breg_x = col2_pos;
  int breg_y = row1;
  int breg_wid = col2_wid;
  m_but_regen->resize(breg_x, breg_y, breg_wid, field_hgt);
  
  int alg_x = col2_pos+30;
  int alg_y = row4;
  int alg_wid = col2_wid + 50;
  m_fld_algorithm->resize(alg_x, alg_y, alg_wid, field_hgt);

  // Column 3 --------------------------
  int bmod_x = col3_pos;
  int bmod_y = row0;
  int bmod_wid = col3_wid;
  m_but_alg_center->resize(bmod_x, bmod_y, bmod_wid, field_hgt);

  int droi_x = col3_pos;
  int droi_y = row1;
  int droi_wid = col3_wid;
  m_but_alg_centroid->resize(droi_x, droi_y, droi_wid, field_hgt);

  // Column 4 --------------------------
  int hyb_x = col4_pos;
  int hyb_y = row0;
  int hyb_wid = col4_wid;
  m_but_alg_hybrid->resize(hyb_x, hyb_y, hyb_wid, field_hgt);

  int pcurp_x = col4_pos;
  int pcurp_y = row1;
  int pcurp_wid = col4_wid;
  m_but_alg_currpos->resize(pcurp_x, pcurp_y, pcurp_wid, field_hgt);

  int neigh_x = col4_pos+30;
  int neigh_y = row4;
  int neigh_wid = w()-neigh_x-20;
  m_fld_neigh->resize(neigh_x, neigh_y, neigh_wid, field_hgt);

  // Column 6 --------------------------

  int step1_x = col6_pos;
  int step1_y = row0;
  int step1_wid = col6_wid;
  m_but_step_1->resize(step1_x, step1_y, step1_wid, field_hgt);

  int step10_x = col6_pos;
  int step10_y = row1;
  int step10_wid = col6_wid;
  m_but_step_10->resize(step10_x, step10_y, step10_wid, field_hgt);

  int step50_x = col6_pos;
  int step50_y = row2;
  int step50_wid = col6_wid;
  m_but_step_50->resize(step50_x, step50_y, step50_wid, field_hgt);

  int step150_x = col6_pos;
  int step150_y = row3;
  int step150_wid = col6_wid;
  m_but_step_150->resize(step150_x, step150_y, step150_wid, field_hgt);

  // Column 7  --------------------------
  int avg_sz_x = col7_pos;
  int avg_sz_y = row0;
  int avg_sz_wid = col7_wid;
  m_fld_avg_size->resize(avg_sz_x, avg_sz_y, avg_sz_wid, field_hgt);

  int max_sz_x = col7_pos;
  int max_sz_y = row1;
  int max_sz_wid = col7_wid;
  m_fld_max_size->resize(max_sz_x, max_sz_y, max_sz_wid, field_hgt);

  int min_sz_x = col7_pos;
  int min_sz_y = row2;
  int min_sz_wid = col7_wid;
  m_fld_min_size->resize(min_sz_x, min_sz_y, min_sz_wid, field_hgt);

  int std_sz_x = col7_pos;
  int std_sz_y = row3;
  int std_sz_wid = col7_wid;
  m_fld_std_size->resize(std_sz_x, std_sz_y, std_sz_wid, field_hgt);


  
}
  
//---------------------------------------------------------------------- 
// Procedure: resizeWidgetsText()

void VOI_GUI::resizeWidgetsText()
{
  int text_size  = 12;
  int label_size = 12;
  
  // Column One ------------------------
  m_fld_tot_dist->textsize(text_size);
  m_fld_tot_dist->labelsize(label_size);

  m_fld_max_dist->textsize(text_size);
  m_fld_max_dist->labelsize(label_size);

  m_fld_min_dist->textsize(text_size);
  m_fld_min_dist->labelsize(label_size);

  m_fld_point_cnt->textsize(text_size);
  m_fld_point_cnt->labelsize(label_size);

  m_fld_poly->textsize(text_size);
  m_fld_poly->labelsize(label_size);

  // Column Two ------------------------
  m_but_reset->labelsize(label_size);
  m_but_regen->labelsize(label_size);

  m_fld_algorithm->textsize(text_size);
  m_fld_algorithm->labelsize(label_size);

  // Column Three ------------------------
  m_but_alg_center->labelsize(label_size);
  m_but_alg_centroid->labelsize(label_size);

  // Column Four ------------------------
  m_but_alg_hybrid->labelsize(label_size);
  m_but_alg_currpos->labelsize(label_size);

  m_fld_neigh->textsize(text_size);
  m_fld_neigh->labelsize(label_size);

  // Column Six -------------------------
  m_but_step_1->labelsize(label_size);
  m_but_step_10->labelsize(label_size);
  m_but_step_50->labelsize(label_size);
  m_but_step_150->labelsize(label_size);

  // Column Seven ------------------------
  m_fld_avg_size->textsize(text_size);
  m_fld_avg_size->labelsize(label_size);

  m_fld_max_size->textsize(text_size);
  m_fld_max_size->labelsize(label_size);

  m_fld_min_size->textsize(text_size);
  m_fld_min_size->labelsize(label_size);

  m_fld_std_size->textsize(text_size);
  m_fld_std_size->labelsize(label_size);
}

//---------------------------------------------------------- 
// Procedure: resize   

void VOI_GUI::resize(int lx, int ly, int lw, int lh)
{
  Fl_Window::resize(lx, ly, lw, lh);
  resizeWidgetsShape();
  resizeWidgetsText();
}

//----------------------------------------------------------------
// Procedure: augmentMenu

void VOI_GUI::augmentMenu() 
{
  //==============================================================
  // The File SubMenu
  //==============================================================
  //m_menubar->add("File/dump cmdline args", 'd',
  //		 (Fl_Callback*)VOI_GUI::cb_DumpCmdLineArgs, (void*)0,
  //		 FL_MENU_DIVIDER);
  
  //==============================================================
  // The BackView SubMenu
  //==============================================================
  // First we remove some items at the superclass level so we can use the 
  // hot keys differently. 

  removeMenuItem("BackView/Zoom Reset");

  //removeMenuItem("BackView/Pan Up (slow) ");
  //removeMenuItem("BackView/Pan Down (slow) ");
  //removeMenuItem("BackView/Pan Left (slow) ");
  //removeMenuItem("BackView/Pan Right (slow)");

  removeMenuItem("BackView/Pan Up (v. slow) ");
  removeMenuItem("BackView/Pan Down (v. slow) ");
  removeMenuItem("BackView/Pan Left (v. slow) ");
  removeMenuItem("BackView/Pan Right (v. slow)");

  //====================================================================
  // The Obstacle SubMenu
  //====================================================================
  m_menubar->add("Poly/RotateLeft", '[',
		 (Fl_Callback*)VOI_GUI::cb_RotatePoly, (void*)-1, 0);
  m_menubar->add("Poly/RotateRight", ']',
		 (Fl_Callback*)VOI_GUI::cb_RotatePoly, (void*)1, 0);
  m_menubar->add("Poly/Smaller", '{',
		 (Fl_Callback*)VOI_GUI::cb_ResizePoly, (void*)-1, 0);
  m_menubar->add("Poly/Bigger", '}',
		 (Fl_Callback*)VOI_GUI::cb_ResizePoly, (void*)1,
		 FL_MENU_DIVIDER);

  m_menubar->add("Poly/Up", FL_SHIFT+FL_Up,
		 (Fl_Callback*)VOI_GUI::cb_AltPolyY, (void*)1, 0);
  m_menubar->add("Poly/Down", FL_SHIFT+FL_Down,
		 (Fl_Callback*)VOI_GUI::cb_AltPolyY, (void*)-1, 0);
  m_menubar->add("Poly/Right", FL_SHIFT+FL_Right,
		 (Fl_Callback*)VOI_GUI::cb_AltPolyX, (void*)1, 0);
  m_menubar->add("Poly/Left", FL_SHIFT+FL_Left,
		 (Fl_Callback*)VOI_GUI::cb_AltPolyX, (void*)-1, 0);

  m_menubar->add("Solve/Increase Solution Speed", ')',
		 (Fl_Callback*)VOI_GUI::cb_AltSolutionSpeed, (void*)1, 0);
  m_menubar->add("Solve/Decrease Solution Speed", '(',
		 (Fl_Callback*)VOI_GUI::cb_AltSolutionSpeed, (void*)2, 0);

}

//----------------------------------------------------------
// Procedure: handle

int VOI_GUI::handle(int event) 
{
  switch(event) {
  case FL_PUSH:
    Fl_Window::handle(event);
    updateXY();
    return(1);
    break;
  default:
    return(Fl_Window::handle(event));
  }
}

//----------------------------------------- RotatePoly
inline void VOI_GUI::cb_RotatePoly_i(int amt) {
  XYPolygon poly = m_voi_viewer->m_vfield.getRegionPoly();
  poly.rotate((double)(amt));
  m_voi_viewer->m_vfield.setRegionPoly(poly);
  m_voi_viewer->redraw();
  updateXY();
}
void VOI_GUI::cb_RotatePoly(Fl_Widget* o, int v) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_RotatePoly_i(v);
}

//----------------------------------------- ResizePoly
inline void VOI_GUI::cb_ResizePoly_i(int amt) {
  XYPolygon poly = m_voi_viewer->m_vfield.getRegionPoly();
  poly.grow_by_amt((double)(amt));
  m_voi_viewer->m_vfield.setRegionPoly(poly);
  m_voi_viewer->redraw();
  updateXY();
}
void VOI_GUI::cb_ResizePoly(Fl_Widget* o, int v) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_ResizePoly_i(v);
}

//----------------------------------------- AltPolyX
inline void VOI_GUI::cb_AltPolyX_i(int amt) {
  XYPolygon poly = m_voi_viewer->m_vfield.getRegionPoly();
  poly.shift_horz((double)(amt));
  m_voi_viewer->m_vfield.setRegionPoly(poly);
  m_voi_viewer->redraw();
  updateXY();
} 
void VOI_GUI::cb_AltPolyX(Fl_Widget* o, int v) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_AltPolyX_i(v);
}

//----------------------------------------- AltPolyY
inline void VOI_GUI::cb_AltPolyY_i(int amt) {
  XYPolygon poly = m_voi_viewer->m_vfield.getRegionPoly();
  poly.shift_vert((double)(amt));
  m_voi_viewer->m_vfield.setRegionPoly(poly);
  m_voi_viewer->redraw();
  updateXY();
}
void VOI_GUI::cb_AltPolyY(Fl_Widget* o, int v) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_AltPolyY_i(v);
}


//----------------------------------------- Reset Button
inline void VOI_GUI::cb_Reset_i()
{
  m_voi_viewer->generateVField();
  m_voi_viewer->redraw();
  updateXY();
}

void VOI_GUI::cb_Reset(Fl_Widget* o) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_Reset_i();
}

//----------------------------------------- Regenerate Button
inline void VOI_GUI::cb_Regenerate_i()
{
  m_voi_viewer->generateVField();
  m_voi_viewer->redraw();
  updateXY();
}

void VOI_GUI::cb_Regenerate(Fl_Widget* o) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_Regenerate_i();
}

//----------------------------------------- Mod Algorithm
inline void VOI_GUI::cb_ModAlgorithm_i(int val)
{
  string alg;
  if(val == 1)
    alg = "center";
  else if(val == 2)
    alg = "centroid";
  else if(val == 3)
    //    alg = "hybrid";
    alg = "churn";
  else if(val == 4)
    alg = "area_balance";

  m_voi_viewer->setAlgorithm(alg);
  updateXY();
}

void VOI_GUI::cb_ModAlgorithm(Fl_Widget* o, int i) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_ModAlgorithm_i(i);
}

//----------------------------------------- Alt Solution Speed
inline void VOI_GUI::cb_AltSolutionSpeed_i(int val)
{
  if(val == 1)
    m_voi_viewer->modSolutionSpeed(0.1);
  else if(val == 2)
    m_voi_viewer->modSolutionSpeed(-0.1);
}

void VOI_GUI::cb_AltSolutionSpeed(Fl_Widget* o, int i) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_AltSolutionSpeed_i(i);
}


//----------------------------------------- Step
inline void VOI_GUI::cb_Step_i(int iterations)
{
  unsigned int uiterations = 0;
  if(iterations > 0)
    uiterations = (unsigned int)(iterations);
  for(unsigned int i=0; i<uiterations; i++) {
    m_voi_viewer->stepVField();
    m_voi_viewer->redraw();
    updateXY();
    m_fld_tot_dist->redraw();
    Fl::flush();
  }
}

void VOI_GUI::cb_Step(Fl_Widget* o, int i) {
  ((VOI_GUI*)(o->parent()->user_data()))->cb_Step_i(i);
}


//------------------------------------------------------
// Procedure  UpdateXY()

void VOI_GUI::updateXY() 
{
  //double dval;
  string sval;

  // Column (1) Odometry Values  ------------------------
  double total_dist = m_voi_viewer->m_evaluator.getTotalOdometry();
  sval = doubleToStringX(total_dist,1);
  m_fld_tot_dist->value(sval.c_str());

  double max_dist = m_voi_viewer->m_evaluator.getMaxOdometry();
  sval = doubleToStringX(max_dist,1);
  m_fld_max_dist->value(sval.c_str());

  double min_dist = m_voi_viewer->m_evaluator.getMinOdometry();
  sval = doubleToStringX(min_dist,1);
  m_fld_min_dist->value(sval.c_str());

  unsigned int point_count = m_voi_viewer->m_vfield.size();
  sval = uintToString(point_count);
  m_fld_point_cnt->value(sval.c_str());

  XYPolygon poly = m_voi_viewer->m_vfield.getRegionPoly();
  sval = poly.get_spec();
  m_fld_poly->value(sval.c_str());

  sval = "disabled";
  m_fld_neigh->value(sval.c_str());
  
  sval = m_voi_viewer->getAlgorithm();
  m_fld_algorithm->value(sval.c_str());
  
  VoronoiField vfield = m_voi_viewer->getVField();

  // Column (7) Proxonoi Size Values  ------------------------
  double avg_size = m_voi_viewer->m_evaluator.getAvgProxArea();
  sval = doubleToStringX(avg_size,1);
  m_fld_avg_size->value(sval.c_str());
  
  double max_size = m_voi_viewer->m_evaluator.getMaxProxArea();
  sval = doubleToStringX(max_size,1);
  m_fld_max_size->value(sval.c_str());

  double min_size = m_voi_viewer->m_evaluator.getMinProxArea();
  sval = doubleToStringX(min_size,1);
  m_fld_min_size->value(sval.c_str());

  double std_size = m_voi_viewer->m_evaluator.getStdDevProxArea();
  sval = doubleToStringX(std_size,1);
  m_fld_std_size->value(sval.c_str());
}


