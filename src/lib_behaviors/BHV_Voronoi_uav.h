/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_Voronoi.h                                        */
/*    DATE: Jan 6th 2020                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BHV_VORONOI_HEADER
#define BHV_VORONOI_HEADER

#include <string>
#include <list>
#include "XYPolygon.h"
#include "Odometer.h"
#include "IvPBehavior.h"

class IvPDomain;
class BHV_Voronoi : public IvPBehavior
{
public:
  BHV_Voronoi(IvPDomain);
  ~BHV_Voronoi() {}

  IvPFunction *onRunState();
  bool setParam(std::string, std::string);
  void onSetParamComplete();
  void onRunToIdleState();
  void onIdleState();
  void onHelmStart();

protected:
  bool updateOwnshipPosition();
  bool updateProxonoiPolys();
  bool updateSetPoint();

  double getRelevance();
  double getPriority();

  IvPFunction *buildOF();

  bool handleConfigOpRegion(std::string);
  bool handleConfigSetPointMethod(std::string);
  void handleVisualHint(std::string);
  void postViewables(unsigned int id = 0);
  void eraseViewables(unsigned int id = 0);
  void postViewableSetPoint();
  void postErasableSetPoint();
  bool setState(std::string);


  XYPoint calculateCircularSetPt();

private:
protected: // State vars
  XYPolygon m_proxonoi_poly;
  XYPolygon m_proxonoi_region;

  Odometer m_odometer;

  double m_osx;
  double m_osy;
  double m_set_x;
  double m_set_y;
  double m_set_x_prev;
  double m_set_y_prev;

  bool m_ownship_in_region;

  double m_total_activated_dist;

  std::string m_state;

private: // Config params
  double m_cruise_speed;

  double m_capture_radius;
  double m_activate_radius;

  double m_stale_nav_thresh;
  double m_stale_poly_thresh;
  double m_stale_searchcenter_thresh;

  XYPolygon m_op_region;

  bool m_setpt_viewable;
  bool m_allow_slipping;

  double m_hint_setpt_size;
  std::string m_hint_setpt_color;

  std::string m_setpt_method;
};

#ifdef WIN32
// Windows needs to explicitly specify functions to export from a dll
#define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
#define IVP_EXPORT_FUNCTION
#endif

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior *createBehavior(std::string name, IvPDomain domain)
  {
    return new BHV_Voronoi(domain);
  }
}

#endif
