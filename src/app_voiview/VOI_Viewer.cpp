/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: VOI_Viewer.cpp                                       */
/*    DATE: Dec 22nd, 2019                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <unistd.h>
#include <iostream>
#include <cmath>
#include "VOI_Viewer.h"
#include "MBUtils.h"
#include "GeomUtils.h"
#include "AngleUtils.h"
#include "VoronoiUtils.h"
#include "XYFormatUtilsPoly.h"
#include "VoronoiSetPtMethods.h"
#include "XYFieldGenerator.h"

using namespace std;

//----------------------------------------------------------------
// Constructor

VOI_Viewer::VOI_Viewer(int x, int y, int wid, int hgt,
		       const char *label) :
  MarineViewer(x, y, wid, hgt, label)
{
  // Initialize book keeping state vars
  m_field_initialized = false;
  
  // Initialize config params
  m_poly_fill_color = "green";
  m_poly_fill_trans = 0.1;
  m_poly_vert_color = "green";
  m_poly_vert_size  = 1;
  m_poly_edge_color = "gray90";
  m_pt_color        = "yellow";
  m_pt_size         = 3;
  
  m_vshift_x = -80;
  m_vshift_y = -605;
  m_zoom     = 1.25;
  
  m_algorithm = "center";
  m_solution_speed = 0.1;
  
  //========================================================
  // Override some default values of MarineViewer superclass
  //========================================================
   
  setParam("tiff_view", "on");
  setParam("hash_viewable", "false");
  setParam("hash_shade", -1.0);
  setParam("hash_shade", 0.75);
  setParam("back_shade", -1.0);
  setParam("back_shade", 0.85);

  setParam("polygon_viewable_labels", "false");
  setParam("polygon_label_pos", "mid");
}

//-------------------------------------------------------------
// Procedure: handle()

int VOI_Viewer::handle(int event)
{
  int vx, vy;
  switch(event) {
  case FL_PUSH:
    vx = Fl::event_x();
    vy = h() - Fl::event_y();
    if(Fl_Window::handle(event) != 1) {
      if(Fl::event_button() == FL_LEFT_MOUSE)
	handle_left_mouse(vx, vy);
      if(Fl::event_button() == FL_RIGHT_MOUSE)
	handle_right_mouse(vx, vy);
    }
    return(1);
    break;  
  default:
    return(Fl_Gl_Window::handle(event));
  }
}

//-------------------------------------------------------------
// Procedure: draw()

void VOI_Viewer::draw()
{
  MarineViewer::draw();

  if(!m_field_initialized)
    initField();  

  drawVoiPolys();
  drawVoiCenterPoints();
  drawVoiCentroidPoints();
  drawVoiPushPoints();
  drawVoiPoints();
}

//-------------------------------------------------------------
// Procedure: drawVoiPolys()

void VOI_Viewer::drawVoiPolys()
{ 
  XYPolygon poly = m_vfield.getRegionPoly();
  poly.set_label("orig_poly");
  poly.set_color("edge", "dodger_blue");
  poly.set_color("vertex", "white");
  poly.set_color("fill", "invisible");
  poly.set_transparency(0.1);
  poly.set_label_color("invisible");
  drawPolygon(poly);

  vector<string> keys = m_vfield.getKeys();
  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];
    XYPolygon poly = m_vfield.getVPoly(key);
    poly.set_label("poly_" + key);
    poly.set_color("edge", m_poly_edge_color);
    poly.set_color("vertex", m_poly_vert_color);
    poly.set_color("fill", m_poly_fill_color);
    poly.set_label_color("yellow");
    
    double pct = m_evaluator.getRelativeArea(key);
    poly.set_transparency(pct);
    drawPolygon(poly);
  }
}

//-------------------------------------------------------------
// Procedure: drawVoiPoints()

void VOI_Viewer::drawVoiPoints()
{
  vector<string> keys = m_vfield.getKeys();
  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];
    XYPoint point = m_vfield.getVPoint(key);
    double area = m_vfield.getVArea(key);
    if(area > 10000)
      area /= 1000;
    unsigned int uint_area = (unsigned int)(area);

    string label = key + " (" + uintToCommaString(uint_area) + ")";
    point.set_label(label);
    point.set_label_color("white");
    point.set_color("vertex", m_pt_color);
    point.set_vertex_size(m_pt_size);
      
    drawPoint(point);
  }
} 

//-------------------------------------------------------------
// Procedure: drawVoiCenterPoints()

void VOI_Viewer::drawVoiCenterPoints()
{
  vector<string> keys = m_vfield.getKeys();
  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];

    XYPoint point = getSetPtCenter(m_vfield, key);

    string label = "c" + key;
    point.set_label(label);
    point.set_label_color("off");
    point.set_color("vertex", "magenta");
    point.set_vertex_size(10);    
    drawPoint(point);
  }
}

//-------------------------------------------------------------
// Procedure: drawVoiCentroidPoints()

void VOI_Viewer::drawVoiCentroidPoints()
{
  vector<string> keys = m_vfield.getKeys();
  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];

    XYPoint point = getSetPtCentroid(m_vfield, key);

    string label = "r" + key;
    point.set_label(label);
    point.set_label_color("off");
    point.set_color("vertex", "red");
    point.set_vertex_size(10);    
    drawPoint(point);
  }
}

//-------------------------------------------------------------
// Procedure: drawVoiPushPoints()

void VOI_Viewer::drawVoiPushPoints()
{
  vector<string> keys = m_vfield.getKeys();
  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];
    
    XYPoint point = getSetPtAreaBalance(m_vfield, key);

    string label = "r" + key;
    point.set_label(label);
    point.set_label_color("dodger_blue");
    point.set_color("vertex", "dodger_blue");
    point.set_vertex_size(10);    
    drawPoint(point);
  }
}

//-------------------------------------------------------------
// Procedure: handle_left_mouse()

void VOI_Viewer::handle_left_mouse(int vx, int vy)
{
  double ix = view2img('x', vx);
  double iy = view2img('y', vy);
  double mx = img2meters('x', ix);
  double my = img2meters('y', iy);
  double sx = snapToStep(mx, 0.1); 
  double sy = snapToStep(my, 0.1);
  
  //cout << "sx: " << doubleToStringX(sx,1) << ", sy: "
  //     << doubleToStringX(sy,1) << endl;
  if(Fl::event_state(FL_SHIFT)) {
    bool added = m_fld_generator.addPoint(sx, sy);
    if(added) {
      XYPoint newpt = m_fld_generator.getNewestPoint();
      string key = m_vfield.getUniqueKey("P");
      m_vfield.addProxPoint(key, newpt);
    }
  } 
  else {
    XYPoint rm_pt(sx,sy);
    string closest_key = m_vfield.getClosestPointKey(rm_pt);
    m_vfield.removePoint(closest_key);
  }

  redraw();
}



//-------------------------------------------------------------
// Procedure: handle_right_mouse()

void VOI_Viewer::handle_right_mouse(int vx, int vy)
{
  double ix = view2img('x', vx);
  double iy = view2img('y', vy);
  double mx = img2meters('x', ix);
  double my = img2meters('y', iy);
  double sx = snapToStep(mx, 0.1);
  double sy = snapToStep(my, 0.1);
  
  //cout << "rx: " << doubleToStringX(sx,1) 
  //  ", ry: " << doubleToStringX(sy,1) << endl;
  if(!Fl::event_state(FL_SHIFT)) {
    bool added = m_fld_generator.addPoint(sx, sy);
    if(added) {
      XYPoint newpt = m_fld_generator.getNewestPoint();
      string key = m_vfield.getUniqueKey("P");
      m_vfield.addProxPoint(key, newpt);
    }
  }
  m_evaluator.setVField(m_vfield);
  redraw();
}


//---------------------------------------------------------
// Procedure: modColorScheme()

void VOI_Viewer::modColorScheme()
{
  string tif_file = getTiffFileCurrent();

  tif_file = rbiteString(tif_file, '/');

  if(m_verbose)
    cout << "tif_file: [" << tif_file << "]" << endl;  
    
  if(tif_file == "sea_of_japan_09_cartodb_dark.tif") {
    setPolyFillColor("gray40");
    setPolyFillTrans("0.1");
    setPolyEdgeColor("gray40");
    setPolyVertColor("dark_blue");
    setPointColor("gray60");
    setPointSize("1");
    m_fld_generator.setBufferDist(25);
  }
  else if(tif_file == "sea_of_japan_09_open_streetmap_hot.tif") {
    setPolyFillColor("gray40");
    setPolyFillTrans("0.1");
    setPolyEdgeColor("dodger_blue");
    setPolyVertColor("gray40");
    setPointColor("white");
    setPointSize("2");
  }
}

//---------------------------------------------------------
// Procedure: addTiffFile()

bool VOI_Viewer::addTiffFile(string tif_file)
{
  if(m_verbose)
    cout << "ADDING TIFF FILE: [" << tif_file << "]" << endl;

  bool ok_tiff = setParam("tiff_file", tif_file);
  if(!ok_tiff)
    return(false);

  string tif_file_low = tolower(tif_file);
  
  if(tif_file == "sea_of_japan_09_cartodb_dark.tif") {
    setPolyFillColor("gray40");
    setPolyFillTrans("0.1");
    setPolyEdgeColor("gray40");
    setPolyVertColor("dark_blue");
    setPointColor("gray60");
    setPointSize("1");
    m_fld_generator.setBufferDist(25);
    
    string polystr = "-509838,10440: 174329,585000 : ";
    polystr += "427750,496100 : 283500,-88600 : -310500,-379600 ";
    XYPolygon poly = string2Poly(polystr);
    bool ok_poly = m_vfield.setRegionPoly(poly);
    if(!ok_poly) {
      cout << "Bad Poly:" << polystr << endl;
      return(false);
    }
    m_zoom = 0.14;
  }

  else if(strContains(tif_file_low, "mit")) {
    cout << "Using the MIT poly...." << endl;
    //else if(tif_file == "MIT_SP.tif") {
    m_fld_generator.setBufferDist(5);
    //string polystr = "0,-20 : 0,-70 : 60,-70 : 80,-45: 60,-20 ";
    string polystr = "format=radial, x=50, y=-50, radius=50, pts=8, snap=1";
    XYPolygon region_poly = string2Poly(polystr);
    region_poly.set_label("mit_sp");
    if(!region_poly.is_convex()) {
      cout << "Bad Poly:" << polystr << endl;
      return(false);
    }
    
    m_fld_generator.addPolygon(region_poly);
    m_vfield.setRegionPoly(region_poly);
    m_zoom = 1.25;    
  }
  else
    return(false);
  

  return(true);
}

//---------------------------------------------------------
// Procedure: modSolutionSpeed()

void VOI_Viewer::modSolutionSpeed(double dval)
{
  m_solution_speed += dval;
  if(m_solution_speed < 0.1)
    m_solution_speed = 0.1;
  if(m_solution_speed > 1)
    m_solution_speed = 1;
}

//---------------------------------------------------------
// Procedure: stepVField()

void VOI_Viewer::stepVField()
{
  if(m_verbose)
    m_vfield.print();

  vector<string> keys = m_vfield.getKeys();
  vector<XYPoint> targs;

  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];
    XYPoint targ_point = getSetPt(m_vfield, key, m_algorithm);
    targs.push_back(targ_point);
  }

  for(unsigned int i=0; i<keys.size(); i++) {
    string key = keys[i];
    XYPoint targ_point = targs[i];

    double cx = targ_point.x();
    double cy = targ_point.y();

    double px = m_vfield.getVPoint(key).x();
    double py = m_vfield.getVPoint(key).y();

    // By default the new point is exactly the target point if
    // solution speed is 1.
    double new_px = cx;
    double new_py = cy;
    
    double dist = hypot(px-cx, py-cy);
    if(m_solution_speed < 1) {
      double ang = relAng(px, py, cx, cy);
      dist *= m_solution_speed;
      projectPoint(ang, dist, px, py, new_px, new_py);
    }
    
    XYPoint newpt(new_px, new_py);
    m_vfield.modProxPoint(key, newpt);
  }

  m_vfield.updateProxPolys();
  m_evaluator.setVField(m_vfield);
}


//---------------------------------------------------------
// Procedure: setPolyFillColor()

bool VOI_Viewer::setPolyFillColor(string colorstr)
{
  if(colorstr == "")
    return(true);

  if(!isColor(colorstr))
    return(false);

  m_poly_fill_color = colorstr;
  
  return(true);
}


//---------------------------------------------------------
// Procedure: setPolyFillTrans()

bool VOI_Viewer::setPolyFillTrans(string trans)
{
  if(trans == "")
    return(true);

  double dtrans = atof(trans.c_str());

  if(dtrans < 0)
    dtrans = 0;
  if(dtrans > 1)
    dtrans = 1;

  m_poly_fill_trans = dtrans;

  return(true);
}


//---------------------------------------------------------
// Procedure: setPolyVertColor()

bool VOI_Viewer::setPolyVertColor(string colorstr)
{
  if(colorstr == "")
    return(true);

  if(!isColor(colorstr))
    return(false);

  m_poly_vert_color = colorstr;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: setPolyVertSize()

bool VOI_Viewer::setPolyVertSize(string vsize)
{
  if(vsize == "")
    return(true);

  double dval = atof(vsize.c_str());
  
  if(dval < 1)
    dval = 1;
  if(dval > 20)
    dval = 20;

  m_poly_vert_size = dval;

  return(true);
}


//---------------------------------------------------------
// Procedure: setPolyEdgeColor()

bool VOI_Viewer::setPolyEdgeColor(string colorstr)
{
  if(colorstr == "")
    return(true);

  if(!isColor(colorstr))
    return(false);

  m_poly_edge_color = colorstr;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: setPointColor()

bool VOI_Viewer::setPointColor(string colorstr)
{
  if(colorstr == "")
    return(true);

  if(!isColor(colorstr))
    return(false);

  m_pt_color = colorstr;
  
  return(true);
}

//---------------------------------------------------------
// Procedure: setPointSize()

bool VOI_Viewer::setPointSize(string psize)
{
  if(psize == "")
    return(true);

  double dval = atof(psize.c_str());

  if(dval < 1)
    dval = 1;
  if(dval > 20)
    dval = 20;

  m_pt_size = dval;

  return(true);
}

//----------------------------------------------------------------
// Procedure: addConfigParam()

void VOI_Viewer::addConfigParam(string str)
{
  m_config_params.push_back(str);
}

//----------------------------------------------------------------
// Procedure: addPostConfigParam()

void VOI_Viewer::addPostConfigParam(string str)
{
  m_post_config_params.push_back(str);
}


//----------------------------------------------------------------
// Procedure: handleConfigParams()

bool VOI_Viewer::handleConfigParams()
{
  if(vectorContains(m_config_params, "-v") ||
     vectorContains(m_config_params, "--verbose"))
    setVerbose(true);

  if(m_verbose)
    cout << "Config Params:" << endl;
  for(unsigned int i=0; i<m_config_params.size(); i++){
    string orig = m_config_params[i];
    string argi = m_config_params[i];
    string left = biteStringX(argi, '=');
    string val  = argi;
    if(m_verbose)
      cout << "orig: " << orig << endl;

    bool ok_param = true;
    if(strEnds(left, ".tif") || strEnds(left, ".tiff"))
      ok_param = addTiffFile(left);
    if(left == "--soj") 
      ok_param = addTiffFile("sea_of_japan_09_open_streetmap_hot.tif");
    else if(left == "--sojd") 
      ok_param = addTiffFile("sea_of_japan_09_cartodb_dark.tif");
    else if((left == "-v") || (left == "--verbose"))
      ok_param = true;
    else if(left == "--amt") 
      m_fld_generator.setTargAmt(atoi(val.c_str()));
      
    if(!ok_param) {
      cout << "Bad Config param: [" << orig << "]" << endl;
      return(false);
    } 
  }
  
  if(getTiffFileCount() == 0)
    addTiffFile("MIT_SP.tif");

  return(true);
}


//----------------------------------------------------------------
// Procedure: handlePostConfigParams()

bool VOI_Viewer::handlePostConfigParams()
{
  if(m_verbose)
    cout << "Post Config Params:" << endl;
  for(unsigned int i=0; i<m_post_config_params.size(); i++) {
    string orig = m_post_config_params[i];
    string argi = m_post_config_params[i];
    string left = biteStringX(argi, '=');
    string val  = argi;
    if(m_verbose)
      cout << "orig:: " << orig << endl;

    bool ok_param = true;
    if(left == "--poly_fill_color")
      ok_param = setPolyFillColor(val);
    else if(left == "--poly_fill_trans")
      ok_param = setPolyFillTrans(val);
    else if(left == "--poly_vert_color")
      ok_param = setPolyVertColor(val);
    else if(left == "--poly_edge_color")
      ok_param = setPolyEdgeColor(val);
    else if(left == "--point_color")
      ok_param = setPointColor(val);
    else if(left == "--point_size")
      ok_param = setPointSize(val);
    else if(left == "--poly") 
      ok_param = m_vfield.setRegionPoly(string2Poly(val));
    else if(left == "--dist")
      m_fld_generator.setBufferDist(atof(val.c_str())); 

    if(!ok_param) {
      cout << "Bad PostConfig param: [" << orig << "]" << endl;
      return(false);
    }
  }

  return(true);
}

//----------------------------------------------------------------
// Procedure: initField()

void VOI_Viewer::initField()
{
  if(!m_vfield.isValidRegionPoly()) {
    string spec = "format=radial,x=40,y=-40,radius=40,pts=6,snap=1,label=foo";
    m_vfield.setRegionPoly(string2Poly(spec));
    cout << "No region spec provided. Using a default region:" << endl;
    cout << "   " << spec << endl;
  }
  
  //=======================================================
  // Shift ctr of view to be at ct of Voronoi field polygon
  XYPolygon poly = m_vfield.getRegionPoly();
  
  double pos_x = poly.get_center_x();
  double pos_y = poly.get_center_y();

  // First determine how much we're off in terms of meters 
  double delta_x = pos_x - m_back_img.get_x_at_img_ctr();
  double delta_y = pos_y - m_back_img.get_y_at_img_ctr();

  // Next determine how much in terms of pixels          
  double pix_per_mtr_x = m_back_img.get_pix_per_mtr_x();
  double pix_per_mtr_y = m_back_img.get_pix_per_mtr_y();

  double x_pixels = pix_per_mtr_x * delta_x;
  double y_pixels = pix_per_mtr_y * delta_y;

  m_vshift_x = -x_pixels;
  m_vshift_y = -y_pixels;

  m_fld_generator.addPolygon(poly);
  
  generateVField();
  m_field_initialized = true;
}


//---------------------------------------------------------
// Procedure: generateVField()

void VOI_Viewer::generateVField()
{
  m_vfield.clear();
  m_fld_generator.generatePoints();

  vector<XYPoint> vpoints = m_fld_generator.getPoints();

  for(unsigned int i=0; i<vpoints.size(); i++) {
    string key = "P" + uintToString(i);
    m_vfield.addProxPoint(key, vpoints[i]);
  }

  if(m_verbose) {
    cout << "Field has been GENERATED:" << endl;
    m_vfield.print();
  }

  m_evaluator.setVField(m_vfield);
}

