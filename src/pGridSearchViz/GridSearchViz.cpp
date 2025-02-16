/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchViz.cpp                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <iterator>
#include "GridSearchViz.h"
#include "MBUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYGridUpdate.h"
#include "ACTable.h"

#include "XYCircle.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GridSearchViz::GridSearchViz()
{
  m_report_deltas = true;
  m_grid_label = "gsv";
  m_grid_var_name = "VIEW_GRID";
  m_sensor_radius_max = 10;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GridSearchViz::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;

    string key = msg.GetKey();
    string sval = msg.GetString();
    // double dval  = msg.GetDouble();
    // double mtime = msg.GetTime();
    // bool   mdbl  = msg.IsDouble();
    // bool   mstr  = msg.IsString();
    // string msrc  = msg.GetSource();
    string community = msg.GetCommunity();

    bool ok_community = m_filter_set.filterCheckVName(community);
    if (!ok_community)
      continue;

    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if (key == "GSV_RESET_GRID")
      m_grid.reset();
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GridSearchViz::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GridSearchViz::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_report_deltas)
    postGridUpdates();
  else
    postGrid();

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GridSearchViz::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  CMOOSApp::OnStartUp();

  string grid_config;

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {

    list<string>::reverse_iterator p;
    for (p = sParams.rbegin(); p != sParams.rend(); p++)
    {
      string orig = *p;
      string line = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      bool handled = false;
      if (param == "grid_config")
      {
        unsigned int len = grid_config.length();
        if ((len > 0) && (grid_config.at(len - 1) != ','))
          grid_config += ",";
        grid_config += value;
        handled = true;
      }
      else if (param == "report_deltas")
        handled = setBooleanOnString(m_report_deltas, value);
      else if (param == "ignore_name")
        handled = m_filter_set.addIgnoreName(value);
      else if (param == "match_name")
        handled = m_filter_set.addMatchName(value);
      else if (param == "grid_label")
        handled = setNonWhiteVarOnString(m_grid_label, value);
      else if (param == "grid_var_name")
        handled = setNonWhiteVarOnString(m_grid_var_name, toupper(value));
      else if (param == "sensor_radius")
        handled = setDoubleOnString(m_sensor_radius_max, value);

      if (!handled)
        reportUnhandledConfigWarning(orig);
    }
  }

  m_grid = string2ConvexGrid(grid_config);

  if (m_grid.size() == 0)
    reportConfigWarning("Unsuccessful ConvexGrid construction.");

  m_grid.set_label("gsv");

  postGrid();
  registerVariables();
  return (true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void GridSearchViz::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT_LOCAL", 0);
  Register("NODE_REPORT", 0);
  Register("GSV_RESET_GRID", 0);
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()

void GridSearchViz::handleMailNodeReport(string str)
{
  NodeRecord record = string2NodeRecord(str);
  if (!record.valid())
    return;

  double sensor_radius = m_sensor_radius_max;

  m_map_drone_sensor_radius.insert(std::pair<string, double>(record.getName(), sensor_radius));

  double posx = record.getX();
  double posy = record.getY();

  XYCircle sensorArea(posx, posy, sensor_radius); // Can be smart to correlate it with altitude

  // TODO: Take the radius as a config parameter
  // TODO: Paramter to configer radius to be proporsional to altitude
  // TODO: Add cell size, region, and sensor coverage parameter to the config file
  // TODO: The increments in grid values should happen after a minimum time interval (when same vehicle is incrementing)

  // TODO: Calculate statistics of coreverage percentage in buildreport. time for 40, 60, 90+ coverage, vise verca coverage, etc.
  // --->: Continues coverage percentage as afunction of time

  // TODO: Add decaying values to the grid cells after a long time, add as parameter

  // TODO: Ability to add or remove a region and then take that in consideration when calculating statistics.
  // --->: Removing a region is the same as it having max value. Increment wont change it.

  for (unsigned int ix = 0; ix < m_grid.size(); ix++)
  {

    const XYSquare &cell = m_grid.getElement(ix);

    // if the the center of the cell is inside the sensor area
    if (sensorArea.containsPoint(cell.getCenterX(), cell.getCenterY()))
    {
      m_map_deltas[ix] = m_map_deltas[ix] + 1;
      m_grid.incVal(ix, 1);
    }

    // bool contained = m_grid.ptIntersect(ix, posx, posy);
    // if(contained) {
    //   m_map_deltas[ix] = m_map_deltas[ix] + 1;
    //   m_grid.incVal(ix, 1);
    // }
  }

  // TODO: Post a view cirle to visualize the sensor area coverage
}

//------------------------------------------------------------
// Procedure: postGrid()

void GridSearchViz::postGrid()
{
  string spec = m_grid.get_spec();

  // By default m_grid_var_name="VIEW_GRID"
  Notify(m_grid_var_name, spec);
}

//------------------------------------------------------------
// Procedure: postGridUpdates()

void GridSearchViz::postGridUpdates()
{
  if (m_map_deltas.size() == 0)
    return;

  XYGridUpdate update(m_grid_label);

  map<unsigned int, double>::iterator p;
  for (p = m_map_deltas.begin(); p != m_map_deltas.end(); p++)
  {
    unsigned int ix = p->first;
    double delta = p->second;
    update.addUpdate(ix, "x", delta);
  }
  string msg = update.get_spec();

  m_map_deltas.clear();

  // By default m_grid_var_name="VIEW_GRID"
  Notify(m_grid_var_name + "_DELTA", msg);
}

//------------------------------------------------------------
// Procedure: buildReport()
//
//  Grid characteristics:
//        Cells: 1024
//    Cell size: 10
//
//             Initial  Min    Max    Min      Max      Cells
//    CellVar  Value    SoFar  SoFar  Limited  Limited  Written
//    -------  -------  -----  -----  -------  -------  -------
//    time           0      0      -  true     false    0
//    temp          70      -      -  false    false    172
//    confid.        0   -100    100  true     true     43
//
//  Reports Sent: 534
//  Report  Freq: 0.8

bool GridSearchViz::buildReport()
{
  unsigned int grid_cells = m_grid.size();
  double cell_sizex = 0;
  double cell_sizey = 0;
  if (grid_cells > 0)
  {
    cell_sizex = m_grid.getElement(0).getLengthX();
    cell_sizey = m_grid.getElement(0).getLengthY();
  }

  m_msgs << "Grid characteristics: " << endl;
  m_msgs << "      Cells: " << m_grid.size() << endl;
  m_msgs << "  Cell size: " << doubleToStringX(cell_sizex) << "x" << doubleToStringX(cell_sizey, 4) << endl
         << endl;

  ACTable actab(6, 2);
  actab.setColumnJustify(1, "right");
  actab.setColumnJustify(2, "right");
  actab.setColumnJustify(3, "right");
  actab << "        | Initial | Min   | Max   | Min     | Max     ";
  actab << "CellVar | Value   | SoFar | SoFar | Limited | Limited ";
  actab.addHeaderLines();

  unsigned int i, cell_var_cnt = m_grid.getCellVarCnt();
  for (i = 0; i < cell_var_cnt; i++)
  {
    string cell_var = m_grid.getVar(i);
    string init_val = doubleToStringX(m_grid.getInitVal(i), 5);
    string cell_min_sofar = doubleToStringX(m_grid.getMin(i), 5);
    string cell_max_sofar = doubleToStringX(m_grid.getMax(i), 5);
    bool cell_min_limited = m_grid.cellVarMinLimited(i);
    bool cell_max_limited = m_grid.cellVarMinLimited(i);
    string cell_min_limit = "-";
    string cell_max_limit = "-";
    if (cell_min_limited)
      cell_min_limit = doubleToStringX(m_grid.getMinLimit(i), 5);
    if (cell_max_limited)
      cell_max_limit = doubleToStringX(m_grid.getMaxLimit(i), 5);
    actab << cell_var << init_val << cell_min_sofar << cell_max_sofar << cell_min_limit << cell_max_limit;
  }
  m_msgs << actab.getFormattedString();



  m_msgs << "\n\nSensor data " << endl;
  m_msgs << "---------------------------------" << endl;
  m_msgs << "       sensor_radius : " << doubleToStringX(m_sensor_radius_max, 1) << endl;


  ACTable actab2(3,1);
  actab2.setColumnJustify(0, "left");
  actab2.setColumnJustify(1, "center");
  actab2.setColumnJustify(2, "center");
  actab2 << "Vehicle | Sensor radius | Max";
  actab2.addHeaderLines();
  for (const auto& [drone, radi] : m_map_drone_sensor_radius){
    actab2 << drone << doubleToStringX(radi,3) << m_sensor_radius_max;
  }
  m_msgs << actab2.getFormattedString();

  return (true);
}
