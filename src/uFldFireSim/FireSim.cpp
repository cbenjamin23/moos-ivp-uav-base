/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    FILE: FireSim.cpp                                          */
/*    DATE: Feb 2024                                        */
/*                                                               */
/*****************************************************************/

#include <iterator>
#include <cmath>
#include "FireSim.h"
#include "VarDataPairUtils.h"
#include "MacroUtils.h"
#include "NodeRecordUtils.h"
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "ColorParse.h"
#include "XYCircle.h"
#include "XYMarker.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "FileBuffer.h"
#include "XYFormatUtilsPoly.h"
#include "XYPolyExpander.h"
#include "XYRangePulse.h"

#include "Logger.h"
#include <filesystem>
//---------------------------------------------------------
// Constructor

FireSim::FireSim()
{
  // Config vars
  m_detect_rng_min = 25;
  m_detect_rng_max = 40;
  m_detect_rng_pd = 0.5;
  m_detect_alt_max = 25;
  m_detect_rng_fixed = true;

  m_scout_rng_transparency = 0.1;
  m_scout_rng_show = true;
  m_scouts_inplay = false;
  m_fire_color = "red";
  m_fire_color_from_vehicle = false;

  // State vars
  m_finished = false;
  m_total_discoverers = 0;
  m_last_broadcast = 0;
  m_vname_leader = "tie";

  m_mission_start_utc = 0;
  m_mission_duration_s = 0;
  m_mission_endtime_utc = 0;

  m_ac.setMaxEvents(20);

  m_mission_duration_s = 600; // Default to 10 minutes
  m_mission_start_utc = 0;

  m_imputeTime = false;

  m_planner_mode = Planner::PlannerMode::UNKNOWN_MODE;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool FireSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  static std::vector<std::string> warnings;

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    // p->Trace();
    CMOOSMsg msg = *p;

    std::string key = msg.GetKey();
    std::string sval = msg.GetString();
    double dval = msg.GetDouble();
    std::string comm = msg.GetCommunity();

    bool handled = false;
    std::string warning;
    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handled = handleMailNodeReport(sval);
    else if (key == "SCOUT_REQUEST")
      handled = handleMailScoutRequest(sval);
    else if ((key == "XFIRE_ALERT") && (comm == "shoreside"))
    {
      handled = m_fireset.fireAlert(sval, m_curr_time, warning);
      updateFinishStatus();
      postFireMarkers();
    }
    else if ((key == "XDISCOVERED_FIRE") && (comm == "shoreside"))
    {
      std::string xstr = tokStringParse(sval, "x");
      std::string ystr = tokStringParse(sval, "y");
      if ((xstr == "") || (ystr == ""))
        handled = false;
      else
      {
        double xval = atof(xstr.c_str());
        double yval = atof(ystr.c_str());
        std::string fname = m_fireset.getNameClosestFire(xval, yval);
        declareDiscoveredFire("nature", fname);
        postFireMarkers();
        handled = true;
      }
    }
    else if (key == "MISSION_START_TIME")
    {
      try 
      { 
        m_mission_scorer.setDeadline(m_mission_duration_s);
        m_mission_scorer.setAlgorithmName(Planner::modeToString(m_planner_mode));
        m_mission_scorer.setIgnoredRegionCount(m_ignoredRegionset.size());
        m_mission_scorer.setSpawnedIgnoredRegionCount(m_ignoredRegionset.spawnsize());
        m_mission_scorer.setDroneCount(m_map_node_records.size());

        m_mission_start_utc = dval;
        m_fireset.setMissionStartTimeOnFires(dval);
        m_ignoredRegionset.setMissionStartTimeOnRegions(dval);
        trySpawnFire();
        handled = true;
        retractRunWarnings(warnings);

      }catch (std::exception &e)
      {
        handled = false;
        warning = "Failed to set mission start time. Exception: " + std::string(e.what());
        Logger::error("OnNewMail:" + warning);
        reportRunWarning(warning);
      }
    }
    else if (key == "GSV_COVERAGE_PERCENTAGE")
    {
      if (isMissionRunning())
        m_mission_scorer.setCoveragePercentage(dval);
      handled = true;
    }
    else if (key == "GSV_VISUALIZE_SENSOR_AREA")
      handled = handleMailVisualizeSensorArea(sval);
    else if (key == "IGNORED_REGION")
      handled = handleMailIgnoredRegion(sval);
    else if (key == "XDISABLE_RESET_MISSION")
      handled = handleMailDisableResetMission(warning);
    else if (key == "CHANGE_PLANNER_MODEX")
    {
      MOOSToUpper(sval);
      try
      {
        m_planner_mode = Planner::stringToMode(sval);
        handled = true;
      }
      catch (std::exception &e)
      {
        std::string msg = "Failed to set planner mode. Exception: " + std::string(e.what());
        Logger::error("OnNewMail:" + msg);
        reportRunWarning(msg);
        handled = false;
      }
    }
    
    if(!warning.empty())
      warnings.push_back(warning);
     
    
    if (!handled)
    {
      if (warning.empty())
        reportRunWarning("Unhandled Mail: " + key);
      else
        reportRunWarning(warning);
    }
  }
  return (true);
}

void FireSim::retractRunWarnings(std::vector<std::string>& warnings)
{
  for (const auto &warning : warnings)
    retractRunWarning(warning);

  warnings.clear();
}

bool FireSim::handleMailDisableResetMission(std::string &warning)
{

  const std::string warningMessage = "Failed Mail: Mission is already disabled or not started.";
  if (m_mission_start_utc == 0 && !m_finished)
  {
    warning = warningMessage;
    return false;
  }

  notifyUnregIgnoredRegions();

  m_fireset.reset(m_curr_time);
  auto fire_points = m_fireset.getFirePoints();
  m_ignoredRegionset.reset(m_curr_time, fire_points);

  postFireMarkers();
  postIgnoredRegions();

  // This ends the mission and calculates a score
  // auto duration_s = m_curr_time - m_mission_start_utc;
  // m_mission_scorer.setDeadline(duration_s);
  // calculateMissionScore(m_imputeTime);

  // Notify("MISSION_FINISHED_TIME", doubleToString(m_mission_endtime_utc));

  m_finished = false;
  m_mission_endtime_utc = 0;
  m_mission_start_utc = 0;

  m_mission_scorer.setIgnoredRegionCount(0);
  m_mission_scorer.setSpawnedIgnoredRegionCount(0);

  retractRunWarning(warningMessage);
  return true;
}

bool FireSim::handleMailVisualizeSensorArea(std::string str)
{
  bool bval;
  bool ok = setBooleanOnString(bval, str);
  m_scout_rng_show = !bval;

  if (!m_scout_rng_show)
  {
    for (const auto &[vname, _] : m_map_node_scout_reqs)
    {
      postRangePolys(vname, false);
    }
  }

  return ok;
}

bool FireSim::handleMailIgnoredRegion(std::string str)
{
  str = stripBlankEnds(str);

  bool doRegister = !strContains(str, "unreg::");

  if (doRegister)
    registerRemoveIgnoredRegion(str.substr(5), doRegister);
  else
    registerRemoveIgnoredRegion(str.substr(7), doRegister);

  Logger::info("Received Command to (un)reg region: " + str);

  return true;
}

// format: x=1,y=4
void FireSim::registerRemoveIgnoredRegion(std::string pos_str, bool doRegister)
{

  double x = tokDoubleParse(pos_str, "x");
  double y = tokDoubleParse(pos_str, "y");

  if (doRegister)
  {

    std::vector<XYPoint> fire_pos = m_fireset.getFirePoints();
    auto name = m_ignoredRegionset.spawnIgnoreRegion(x, y, fire_pos);
    trySpawnIgnoredRegion();
    Logger::info("Registering ignored region: " + name);
  }
  else
  { // unregister
    std::string rname = m_ignoredRegionset.getNameOfIgnoredRegionContaining(x, y);
    if (rname.empty())
      return;
    auto ignoredRegion = m_ignoredRegionset.getIgnoredRegion(rname);
    ignoredRegion.setState(IgnoredRegion::RegionState::UNDISCOVERED);

    auto marker = ignoredRegion.getMarker();
    marker.set_active(false);

    Logger::info("Unregistering ignored region: " + rname);
    Logger::info("Marker spec: " + marker.get_spec());

    ignoredRegion.setMarker(marker);
    m_ignoredRegionset.modIgnoredRegion(ignoredRegion);

    postIgnoredRegions();
    m_ignoredRegionset.removeIgnoreRegion(rname);

    std::string alert_spec = "unreg::" + rname;
    Notify("IGNORED_REGION_ALERT", alert_spec);
  }
}


void FireSim::notifyUnregIgnoredRegions(){
  auto unreg_regions = m_ignoredRegionset.getIgnoredRegionNames();
  for (const auto &region : unreg_regions)
    Notify("IGNORED_REGION_ALERT", "unreg::" + region);
  
}
//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool FireSim::OnConnectToServer()
{
  registerVariables();

  Notify("XREQUEST_PLANNER_MODE", "true");
  return (true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void FireSim::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register("XFIRE_ALERT", 0);
  Register("XDISCOVERED_FIRE", 0);
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("SCOUT_REQUEST", 0);
  Register("MISSION_START_TIME", 0);

  Register("GSV_VISUALIZE_SENSOR_AREA", 0);
  Register("GSV_COVERAGE_PERCENTAGE", 0);

  Register("IGNORED_REGION", 0);
  Register("CHANGE_PLANNER_MODEX", 0);

  Register("XDISABLE_RESET_MISSION", 0);
  
}

//---------------------------------------------------------
// Procedure: Iterate()

bool FireSim::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (isMissionRunning())
  { // Mission is running
    tryScouts();
    trySpawnFire();
    trySpawnIgnoredRegion();
    updateFinishStatus();
  }

  postScoutRngPolys();

  // periodically broadcast fire info to all vehicles
  // if ((m_curr_time - m_last_broadcast) > 15)
  // {
  //   broadcastFires();
  //   m_last_broadcast = m_curr_time;
  // }

  AppCastingMOOSApp::PostReport();
  return (true);
}

void FireSim::trySpawnFire()
{
  auto spawned_fires = m_fireset.tryAddSpawnableFire(m_mission_start_utc, m_curr_time);
  if (spawned_fires.empty())
    return;

  postFireMarkers();
  for (const auto &fire : spawned_fires)
    postFirePulseMessage(fire, m_curr_time);
}

void FireSim::trySpawnIgnoredRegion()
{
  auto spawned_regions = m_ignoredRegionset.tryAddSpawnableRegion(m_mission_start_utc, m_curr_time);
  if (spawned_regions.empty())
    return;

  postIgnoredRegions();
  for (const auto &region : spawned_regions)
    postIgnoredRegionPulseMessage(region, m_curr_time);
}
//---------------------------------------------------------
// Procedure: OnStartUp()

bool FireSim::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  std::string fire_config;
  std::string ignoredRegion_config;

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    std::string orig = *p;
    std::string line = *p;
    std::string param = biteStringX(line, '=');
    std::string value = line;

    bool handled = false;
    std::string warning;
    if (param == "fire_config")
    {
      unsigned int len = fire_config.length();
      if ((len > 0) && (fire_config.at(len - 1) != ','))
        fire_config += ",";
      fire_config += value;
      handled = true;
    }
    else if (param == "ignoredRegion_config")
    {
      unsigned int len = ignoredRegion_config.length();
      if ((len > 0) && (ignoredRegion_config.at(len - 1) != ','))
        ignoredRegion_config += ",";
      ignoredRegion_config += value;
      handled = true;
    }
    else if (param == "mission_score_save_path")
      handled = handleMissionScoreSavePath(value);
    else if (param == "show_detect_rng")
      handled = setBooleanOnString(m_scout_rng_show, value);
    else if (param == "detect_rng_min")
      handled = handleConfigDetectRangeMin(value);
    else if (param == "detect_rng_max")
      handled = handleConfigDetectRangeMax(value);
    else if (param == "detect_rng_pd")
      handled = handleConfigDetectRangePd(value);
    else if (param == "detect_alt_max")
      handled = setDoubleOnString(m_detect_alt_max, value);
    else if (param == "detect_rng_fixed")
      handled = setBooleanOnString(m_detect_rng_fixed, value);
    else if ((param == "scout_rng_transparency") && isNumber(value))
      handled = setNonNegDoubleOnString(m_scout_rng_transparency, value);
    else if (param == "mission_duration")
      handled = setDoubleOnString(m_mission_duration_s, value);
    else if (param == "winner_flag")
      handled = addVarDataPairOnString(m_winner_flags, value);
    else if (param == "leader_flag")
      handled = addVarDataPairOnString(m_leader_flags, value);
    else if (param == "finish_flag")
      handled = addVarDataPairOnString(m_finish_flags, value);
    else if (param == "fire_color_from_vehicle") // Deprecated (dont use explicitly)
      handled = setBooleanOnString(m_fire_color_from_vehicle, value);
    else if (param == "fire_color")
    {
      if (value == "vehicle")
      {
        m_fire_color_from_vehicle = true;
        handled = true;
      }
      else
        handled = setColorOnString(m_fire_color, value);
    }
    else if (param == "impute_time")
      handled = setBooleanOnString(m_imputeTime, value);

    if (!handled)
    {
      reportUnhandledConfigWarning(orig);
      if (warning != "")
        reportUnhandledConfigWarning(warning);
    }
  }

  Logger::info("FireSim::OnStartUp: Fire Config: " + fire_config);
  Logger::info("FireSim::OnStartUp: IgnoredRegion Config: " + ignoredRegion_config);

  std::string warning;
  bool ok = m_fireset.handleFireConfig(fire_config, m_curr_time, warning);
  if (!ok)
    reportUnhandledConfigWarning(warning);

  warning.clear();

  auto fire_points = m_fireset.getFirePoints();
  ok = m_ignoredRegionset.handleIgnoredRegionConfig(ignoredRegion_config, m_curr_time, warning, fire_points);
  if (!ok)
    reportUnhandledConfigWarning(warning);

  Notify("PLOGGER_CMD", "COPY_FILE_REQUEST=" + m_fireset.getFireFile());
  Notify("PLOGGER_CMD", "COPY_FILE_REQUEST=" + m_ignoredRegionset.getRegionFile());

  updateFinishStatus();

  postFireMarkers();
  postIgnoredRegions();

  srand(time(NULL));

  registerVariables();

  // Initialize the mission scorer
  m_mission_scorer.init(m_fireset.size(), m_mission_duration_s, 0.0);

  return (true);
}



bool FireSim::handleMissionScoreSavePath(std::string path)
{
  if (path.empty())
    return false;

  m_mission_score_save_path = getenv("HOME");
  m_mission_score_save_path += "/moos-ivp-uav-base/" + path;
  
  // ensure directory exists
  if (!std::filesystem::exists(m_mission_score_save_path))
  {
    std::filesystem::create_directories(m_mission_score_save_path);
    if (!std::filesystem::exists(m_mission_score_save_path))
    {
      Logger::error("Failed to create directory: " + m_mission_score_save_path);
      return false;
    }
  }
  
  return true;
}
//------------------------------------------------------------
// Procedure: handleConfigDetectRangeMin()

bool FireSim::handleConfigDetectRangeMin(std::string str)
{
  if (!isNumber(str))
    return (false);

  m_detect_rng_min = atof(str.c_str());

  if (m_detect_rng_min < 0)
    m_detect_rng_min = 0;
  if (m_detect_rng_max <= m_detect_rng_min)
    m_detect_rng_max = m_detect_rng_min + 1;

  return (true);
}

//------------------------------------------------------------
// Procedure: handleConfigDetectRangeMax()

bool FireSim::handleConfigDetectRangeMax(std::string str)
{
  if (!isNumber(str))
    return (false);

  m_detect_rng_max = atof(str.c_str());

  if (m_detect_rng_max < 0)
    m_detect_rng_max = 0;
  if (m_detect_rng_min >= m_detect_rng_max)
    m_detect_rng_min = m_detect_rng_max * 0.9;

  return (true);
}

//------------------------------------------------------------
// Procedure: handleConfigDetectRangePd()

bool FireSim::handleConfigDetectRangePd(std::string str)
{
  if (!isNumber(str))
    return (false);

  m_detect_rng_pd = atof(str.c_str());

  if (m_detect_rng_pd < 0)
    m_detect_rng_pd = 0;
  if (m_detect_rng_pd > 1)
    m_detect_rng_pd = 1;

  return (true);
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49,LAT=43.825089, LON=-70.330030,
//            SPD=2.00, HDG=119.06,YAW=119.05677,DEPTH=0.00,
//            LENGTH=4.0,MODE=ENGAGED

bool FireSim::handleMailNodeReport(const std::string &node_report_str)
{
  NodeRecord new_record = string2NodeRecord(node_report_str);

  if (!new_record.valid())
  {
    Notify("FS_DEBUG", "Invalid incoming node report");
    reportRunWarning("ERROR: Unhandled node record");
    return (false);
  }

  // In case there is an outstanding RunWarning indicating the lack
  // of a node report for a given vehicle, retract it here. This is
  // mostly a startup timing issue. Sometimes a sensor request is
  // received before a node report. Only a problem if the node report
  // never comes. Once we get one, it's no longer a problem.
  std::string vname = new_record.getName();
  retractRunWarning("No NODE_REPORT received for " + vname);

  // if (m_map_node_records.count(vname) == 0)
  //    broadcastFires();

  m_map_node_records[vname] = new_record;

  return (true);
}

//---------------------------------------------------------
// Procedure: tryScouts()

void FireSim::tryScouts()
{

  // Logger::info("Trying to Scout for all vehicles");
  // For each vehicle, check if pending scout actions are to be applied
  for (const auto &[vname, _] : m_map_node_records)
    tryScoutsVName(vname);
}

//---------------------------------------------------------
// Procedure: tryScoutsVName()

void FireSim::tryScoutsVName(std::string vname)
{
  // Logger::info("Trying to scout for vehicle with Vname given");
  // If vehicle has not posted a scout request recently, then the
  // scout ability is off for this vehicle.
  double elapsed_req = m_curr_time - m_map_node_last_scout_req[vname];
  if (elapsed_req > 5)
  {
    Logger::warning("Elapsed request time > 5 ");
    return;
  }

  double elapsed_try = m_curr_time - m_map_node_last_scout_try[vname];
  if (elapsed_try < 1) // Allow 1 seconds between scout tries
  {
    // Logger::warning("Elapsed try time < 1 ");
    return;
  }
  m_map_node_last_scout_try[vname] = m_curr_time;
  m_map_node_scout_tries[vname]++;

  std::set<std::string> fire_names = m_fireset.getFireNames();
  for (const auto &fname : fire_names)
    tryScoutsVNameFire(vname, fname);

  std::set<std::string> ignoredRegion_names = m_ignoredRegionset.getIgnoredRegionNames();
  for (const auto &rname : ignoredRegion_names)
    tryScoutsVNameIgnoredRegion(vname, rname);
}

//---------------------------------------------------------
// Procedure: tryScoutsVNameFire()

void FireSim::tryScoutsVNameFire(std::string vname, std::string fname)
{
  // Logger::info("Trying to scout for vehicle with Vname and fire name given");

  Fire fire = m_fireset.getFire(fname);

  bool discovered = rollDiceFire(vname, fname);
  if (discovered)
  {
    fire.incDiscoverCnt();
    m_fireset.modFire(fire);

    if (fire.isDiscovered())
      return;

    declareDiscoveredFire(vname, fname);
  }
}

//---------------------------------------------------------
// Procedure: tryScoutsVNameIgnoredRegion()

void FireSim::tryScoutsVNameIgnoredRegion(std::string vname, std::string rname)
{
  IgnoredRegion ignoredRegion = m_ignoredRegionset.getIgnoredRegion(rname);

  bool discovered = rollDiceIgnoredRegion(vname, rname);
  if (discovered && !ignoredRegion.isDiscovered())
    declareDiscoveredIgnoredRegion(vname, rname);
}

double FireSim::altScaledRange(double range_limit, std::string vname) const
{
  double altitude = m_map_node_records.at(vname).getAltitude();
  double range_scaling = (altitude > range_limit || m_detect_rng_fixed) ? 1.0 : (altitude / range_limit);
  double rng = range_limit * range_scaling;
  return (rng < 0) ? 0 : rng;
}

//------------------------------------------------------------
// Procedure: updateLeaderStatus()

void FireSim::updateLeaderStatus()
{
  // Part 1: Note prev leader to detect a lead change
  std::string prev_leader = m_vname_leader;

  // Part 2: Calc highest number of discoveries over any vehicle
  unsigned int highest_discover_count = 0;
  std::map<std::string, unsigned int>::iterator p;
  for (p = m_map_node_discoveries.begin(); p != m_map_node_discoveries.end(); p++)
  {
    unsigned int discovers = p->second;
    if (discovers > highest_discover_count)
      highest_discover_count = discovers;
  }

  // Part 3: Calc vector of vnames having highest discover count
  std::vector<std::string> leader_vnames;
  for (p = m_map_node_discoveries.begin(); p != m_map_node_discoveries.end(); p++)
  {
    std::string vname = p->first;
    unsigned int discovers = p->second;
    if (discovers == highest_discover_count)
      leader_vnames.push_back(vname);
  }

  // Part 4: Set the new leader or update leader to tie status
  if (leader_vnames.size() == 1)
    m_vname_leader = leader_vnames[0];
  else
    m_vname_leader = "tie";

  // Part 5: If no change, we're done. Otherwise make postings
  if (m_vname_leader == prev_leader)
    return;

  Notify("UFFS_LEADER", m_vname_leader);
  postFlags(m_leader_flags);
}

//------------------------------------------------------------
// Procedure: updateWinnerStatus()

void FireSim::updateWinnerStatus(bool finished)
{
  // Once a winner always a winner
  if (m_vname_winner != "")
    return;

  // Part 2: Determine the threshold for winning
  unsigned int known_fire_cnt = m_fireset.size();
  double win_thresh = (double)(known_fire_cnt) / 2;

  // Part 3: Calc vector of vnames having reached the win threshold
  // Possibly >1 winner for now. Will handle tie-breaker afterwards.
  std::vector<std::string> winner_vnames;

  for (const auto &[vname, discoveries] : m_map_node_discoveries)
  {
    if (discoveries >= win_thresh)
      winner_vnames.push_back(vname);
  }

  // Part 4: If no winners then done for now
  if (winner_vnames.size() == 0)
  {
    Notify("UFFS_WINNER", "pending");
    return;
  }

  std::string would_be_winner;

  // Part 5: If one winner, set the winner
  if (winner_vnames.size() == 1)
    would_be_winner = winner_vnames[0];

  // Part 6: If multiple vnames meeting win threshold, do tiebreaker
  if (winner_vnames.size() > 1)
  {
    std::string first_winner;
    double first_winner_utc = 0;
    for (const auto &[vname, utc] : m_map_node_last_discover_utc)
    {
      if (utc >= win_thresh)
      {
        if ((first_winner == "") || (utc < first_winner_utc))
        {
          first_winner = vname;
          first_winner_utc = utc;
        }
      }
    }
    would_be_winner = first_winner;
  }

  // If scouting in play, and we're not yet
  // finished, then hold off on declaring a winner.
  if (m_scouts_inplay && !finished)
    return;

  m_vname_winner = would_be_winner;
  //Notify("UFFS_WINNER", m_vname_winner);
  postFlags(m_winner_flags);
}

//------------------------------------------------------------
// Procedure: updateFinishStatus()
//   Purpose: Completion is when all fires have been
//            discovered.

void FireSim::updateFinishStatus()
{
  // Mission is not running
  if (!isMissionRunning())
    return;

  std::set<std::string> fnames = m_fireset.getFireNames();
  if (fnames.size() == 0)
    return;

  // Are all fires discovered?
  bool finished = false;

  // First and most general criteria for finishing is when all
  // fires have been discovered
  if (m_fireset.allFiresDiscovered())
    finished = true;

  // Second criteria if misson deadline has passed .
  if (isMissionDeadlineReached())
    finished = true;

  // log the moostime, deadline, and if all fires are discovered

  // Logger::info("uPdateFinishStatus: Time: " + doubleToString(MOOSTime() - m_mission_start_utc));
  // Logger::info("uPdateFinishStatus: Mission duration: " + doubleToString(m_mission_duration_s));
  // Logger::info("uPdateFinishStatus: Deadline reached: " + boolToString(isMissionDeadlineReached()));
  // Logger::info("uPdateFinishStatus: All fires discovered: " + boolToString(m_fireset.allFiresDiscovered()));

  if (!finished)
    return;

  m_finished = true;
  m_mission_endtime_utc = m_curr_time;

  Notify("MISSION_FINISHED_TIME", doubleToString(m_mission_endtime_utc));
  Notify("UFFS_FINISHED", boolToString(m_finished));
  postFlags(m_finish_flags);

  updateWinnerStatus(m_finished);

  // Calculate and publish the mission score
  calculateMissionScore(m_imputeTime);
}

//------------------------------------------------------------
// Procedure: rollDiceFire()
//
//
// 1.0 ^       sensor_rng_min       sensor_rng_max
//     |
//     |            |                 |
// Pd  |------------o                 |
//     |            |  \              |
//     |            |     \           |
//     |            |        \        |
//     |            |           \     |
//     |            |              \  |
//     o------------------------------o----------------------------->
//         range from fire to ownship
//

bool FireSim::rollDiceFire(std::string vname, std::string fname)
{
  // Part 1: Sanity checking
  if (!m_fireset.hasFire(fname))
    return (false);
  if (m_map_node_records.count(vname) == 0)
    return (false);

  Fire fire = m_fireset.getFire(fname);

  // Part 2: Calculated the range to fire
  double vx = m_map_node_records[vname].getX();
  double vy = m_map_node_records[vname].getY();
  double fx = fire.getCurrX();
  double fy = fire.getCurrY();
  double range_to_fire = hypot((vx - fx), (vy - fy));

  // Part 3: Calculate Pd threshold modified by range to fire
  int rand_int = rand() % 10000;
  double dice_roll = (double)(rand_int) / 10000;

  double range_max = altScaledRange(m_detect_rng_max, vname);
  double range_min = altScaledRange(m_detect_rng_min, vname);

  double pd = m_detect_rng_pd;
  if (range_to_fire >= range_max)
    pd = 0;
  else if (range_to_fire >= range_min)
  {
    double pct = range_max - range_to_fire;
    pct = pct / (range_max - range_min);
    pd = pct * pd;
  }

  if (range_to_fire <= range_max)
  {
    fire.incScoutTries();
  }

  m_fireset.modFire(fire);

  // Apply the dice role to the Pd
  if (dice_roll >= pd)
    return (false);

  return (true);
}

//------------------------------------------------------------
// Procedure: rollDiceIgnoredRegion()
//
//
// 1.0 ^       sensor_rng_min       sensor_rng_max
//     |
//     |            |                 |
// Pd  |------------o                 |
//     |            |  \              |
//     |            |     \           |
//     |            |        \        |
//     |            |           \     |
//     |            |              \  |
//     o------------------------------o----------------------------->
//         range from fire to ownship
//

bool FireSim::rollDiceIgnoredRegion(std::string vname, std::string rname)
{
  // Part 1: Sanity checking
  if (!m_ignoredRegionset.hasIgnoredRegion(rname))
    return (false);
  if (m_map_node_records.count(vname) == 0)
    return (false);

  IgnoredRegion ignoredRegion = m_ignoredRegionset.getIgnoredRegion(rname);

  // Part 2: Calculated the range to fire
  double vx = m_map_node_records[vname].getX();
  double vy = m_map_node_records[vname].getY();
  double rx = ignoredRegion.getMarker().get_vx();
  double ry = ignoredRegion.getMarker().get_vy();
  double range_to_ignoredRegion = hypot((vx - rx), (vy - ry));

  // Part 3: Calculate Pd threshold modified by range to fire
  int rand_int = rand() % 10000;
  double dice_roll = (double)(rand_int) / 10000;

  double range_max = altScaledRange(m_detect_rng_max, vname);
  double range_min = altScaledRange(m_detect_rng_min, vname);

  double pd = m_detect_rng_pd;
  if (range_to_ignoredRegion >= range_max)
    pd = 0;
  else if (range_to_ignoredRegion >= range_min)
  {
    double pct = range_max - range_to_ignoredRegion;
    pct = pct / (range_max - range_min);
    pd = pct * pd;
  }

  if (range_to_ignoredRegion <= range_max)
  {
    ignoredRegion.incScoutTries();
  }

  m_ignoredRegionset.modIgnoredRegion(ignoredRegion);

  // Apply the dice role to the Pd
  if (dice_roll >= pd)
    return (false);

  return (true);
}

//---------------------------------------------------------
// Procedure: handleMailScoutRequest()
//   Example: vname=cal

bool FireSim::handleMailScoutRequest(std::string request)
{
  std::string vname = tokStringParse(request, "vname");

  // Sanity Check 1: check for empty vname or tname
  if (vname == "")
    return (reportRunWarning("Scout request with no vname"));

  m_scouts_inplay = true;

  m_map_node_scout_reqs[vname]++;
  m_map_node_last_scout_req[vname] = MOOSTime();
  return (true);
}

//------------------------------------------------------------
// Procedure: declareDiscoveredIgnoredRegion()
//     Notes: Example postings:
//            DISCOVERED_IGNORED_REGION = id=r1, ... , format=rectangle; ... ;..., discoverer=abe

void FireSim::declareDiscoveredIgnoredRegion(std::string vname, std::string rname)
{
  // Part 1: Sanity check
  if (!m_ignoredRegionset.hasIgnoredRegion(rname))
    return;

  IgnoredRegion ignoredRegion = m_ignoredRegionset.getIgnoredRegion(rname);
  ignoredRegion.setState(IgnoredRegion::RegionState::DISCOVERED);
  ignoredRegion.setDiscoverer(vname);
  ignoredRegion.setTimeDiscovered(MOOSTime());
  m_ignoredRegionset.modIgnoredRegion(ignoredRegion);

  reportEvent("Ignored Region " + rname + " has been discovered by " + vname + "!");

  postIgnoredRegions();
  postIgnoredRegionPulseMessage(ignoredRegion, m_curr_time, vname);

  std::string alert_spec = "reg::" + ignoredRegion.getSpec();
  Notify("IGNORED_REGION_ALERT", alert_spec);


  // if fire in ignored region, then declare it discovered
  std::set<std::string> fire_names = m_fireset.getFireNames();
  for (const auto &fname : fire_names)
  {
    Fire fire = m_fireset.getFire(fname);
    
    if (fire.isDiscovered())
      continue;

    if (ignoredRegion.contains(fire.getCurrX(), fire.getCurrY()))
    {
      fire.incDiscoverCnt();
      m_fireset.modFire(fire);
      declareDiscoveredFire(vname, fname);
    }
  }

  // Send the fire's position and weight to the discoverer as their "own region"
  double fire_x = fire.getCurrX();
  double fire_y = fire.getCurrY();
  
  // Send OWN_REGION_X to discoverer
  std::string nmsg_x = "src_node=shoreside,dest_node=" + vname;
  nmsg_x += ",var_name=OWN_REGION_X,string_val=" + doubleToStringX(fire_x, 2);
  Notify("NODE_MESSAGE_LOCAL", nmsg_x);
  
  // Send OWN_REGION_Y to discoverer
  std::string nmsg_y = "src_node=shoreside,dest_node=" + vname;
  nmsg_y += ",var_name=OWN_REGION_Y,string_val=" + doubleToStringX(fire_y, 2);
  Notify("NODE_MESSAGE_LOCAL", nmsg_y);
  
  // Send OWN_REGION_WEIGHT to discoverer
  // For now, use 1.0 as default weight. Later you can add per-fire weights.
  std::string nmsg_w = "src_node=shoreside,dest_node=" + vname;
  nmsg_w += ",var_name=OWN_REGION_WEIGHT,string_val=1.0";
  Notify("NODE_MESSAGE_LOCAL", nmsg_w);

  
}

//------------------------------------------------------------
// Procedure: declareDiscoveredFire()
//     Notes: Example postings:
//            DISCOVERED_FIRE = id=f1, finder=abe

void FireSim::declareDiscoveredFire(std::string vname, std::string fname)
{
  // Part 1: Sanity check
  if (!m_fireset.hasFire(fname))
    return;

  // Part 2: Update the notables data structures to support calc
  // of leader differentials
  addNotable(vname, fname);

  // Part 3: Update the fire status, mark the discoverer. Note the
  // check for fire being not yet discovered was done earlier
  Fire fire = m_fireset.getFire(fname);
  fire.setState(Fire::FireState::DISCOVERED);
  fire.setDiscoverer(vname);
  fire.setTimeDiscovered(MOOSTime());
  m_fireset.modFire(fire);

  // Part 4: Update the discover stats for this vehicle
  m_map_node_discoveries[vname]++;
  m_map_node_last_discover_utc[vname] = m_curr_time;

  // Part 5: Update the leader, winner and finish status
  updateLeaderStatus();
  updateWinnerStatus();
  updateFinishStatus();

  // Part 6: Generate postings, visuals and events
  reportEvent("Fire " + fname + " has been discovered by " + vname + "!");

  postFireMarkers();
  postFirePulseMessage(fire, m_curr_time, vname);

  std::string idstr = m_fireset.getFire(fname).getID();
  idstr = findReplace(idstr, "id", "");
  std::string msg = "id=" + idstr + ", finder=" + vname;
  Notify("DISCOVERED_FIRE", msg);
}

//------------------------------------------------------------
// Procedure: postScoutRngPolys()

void FireSim::postScoutRngPolys()
{
  if (!m_scout_rng_show)
    return;

  std::map<std::string, double>::iterator p;
  for (const auto [vname, last_req] : m_map_node_last_scout_req)
  {
    double elapsed = m_curr_time - last_req;
    bool active = elapsed < 3;
    postRangePolys(vname, active);
  }
}

//------------------------------------------------------------
// Procedure: postRangePolys()

void FireSim::postRangePolys(std::string vname, bool active)
{
  if (m_map_node_records.count(vname) == 0)
    return;

  double x = m_map_node_records[vname].getX();
  double y = m_map_node_records[vname].getY();

  XYCircle circ(x, y, altScaledRange(m_detect_rng_max, vname));
  circ.set_label("sensor_max_" + vname);
  circ.set_active(active);
  circ.set_vertex_color("off");
  circ.set_label_color("off");
  circ.set_edge_color("off");
  circ.set_color("fill", "white");
  circ.set_transparency(m_scout_rng_transparency);

  std::string spec1 = circ.get_spec();
  Notify("VIEW_CIRCLE", spec1);

  circ.set_label("sensor_min_" + vname);
  circ.setRad(altScaledRange(m_detect_rng_min, vname));
  std::string spec2 = circ.get_spec();
  Notify("VIEW_CIRCLE", spec2);
}

//------------------------------------------------------------
// Procedure: broadcastFires()
//   Example: FIRE_ALERT = x=34, y=85, id=21

void FireSim::broadcastFires()
{
  for (const auto &[vname, _] : m_map_node_records)
  {
    std::string var = "FIRE_ALERT_" + toupper(vname);
    std::set<std::string> fires = m_fireset.getFireNames();

    for (const auto &fname : fires)
    {
      Fire fire = m_fireset.getFire(fname);
      std::string id_str = fire.getID();
      id_str = findReplace(id_str, "id", "");
      std::string msg = "x=" + doubleToStringX(fire.getCurrX(), 1);
      msg += ", y=" + doubleToStringX(fire.getCurrY(), 1);
      msg += ", id=" + id_str;
      Notify(var, msg);
    }
  }
}

//------------------------------------------------------------
// Procedure: postFireMarkers()

void FireSim::postFireMarkers()
{
  std::set<std::string> fire_names = m_fireset.getFireNames();

  for (const auto &fname : fire_names)
    postFireMarker(fname);

  XYPolygon poly = m_fireset.getSearchRegion();
  if (poly.is_convex())
  {
    Notify("VIEW_POLYGON", poly.get_spec());
    Notify("SEARCH_REGION", poly.get_spec());
  }
}

//------------------------------------------------------------
// Procedure: postFireMarker()

void FireSim::postFireMarker(std::string fname)
{
  if (!m_fireset.hasFire(fname))
    return;

  Fire fire = m_fireset.getFire(fname);
  std::string discoverer = fire.getDiscoverer();

  bool notable = isNotable(fname);

  XYMarker marker;
  marker.set_label(fname);
  marker.set_type("diamond");
  marker.set_vx(fire.getCurrX());
  marker.set_vy(fire.getCurrY());
  marker.set_width(FIREMARKER_WIDTH);
  marker.set_edge_color("green");
  marker.set_transparency(FIREMARKER_TRANSPARENCY_UNDISC);

  if ((discoverer != "")) // Fire is discovered by vehicle or GCS
  {
    std::string marker_color = m_fire_color;

    if (discoverer == "nature") // Fire is discovered by nature/GCS
      marker.set_color("primary_color", m_fire_color);
    else // Fire is discovered by vehicle
    {
      if (m_fire_color_from_vehicle)
      {
        marker_color = m_map_node_records[discoverer].getColor();
        marker.set_type("efield");
        marker.set_color("secondary_color", m_fire_color);
      }

      marker.set_color("primary_color", marker_color);

      if (!notable)
        marker.set_transparency(FIREMARKER_TRANSPARENCY_DISC_NOTABLE);
      else
        marker.set_transparency(FIREMARKER_TRANSPARENCY_DISC);
    }
  }
  else // Fire is undiscovered
  {
    std::string gray = "gray50";
    marker.set_type("triangle");
    marker.set_color("primary_color", gray);
  }

  Notify("VIEW_MARKER", marker.get_spec());
}

void FireSim::postIgnoredRegions()
{
  std::set<std::string> ignoredRegion_names = m_ignoredRegionset.getIgnoredRegionNames();

  for (const auto &rname : ignoredRegion_names)
    postIgnoredRegion(rname);
}

void FireSim::postIgnoredRegion(std::string rname)
{
  if (!m_ignoredRegionset.hasIgnoredRegion(rname))
    return;

  IgnoredRegion ignoredRegion = m_ignoredRegionset.getIgnoredRegion(rname);

  XYPolygon poly = ignoredRegion.getPoly();
  poly.set_transparency(IGNORED_REGION_MARKER_TRANSPARENCY_UNDISC);
  XYMarker marker = ignoredRegion.getMarker();
  marker.set_transparency(IGNORED_REGION_MARKER_TRANSPARENCY_UNDISC);

  if (ignoredRegion.isDiscovered())
  {
    marker.set_active(true);
    poly.set_active(true);
    marker.set_transparency(IGNORED_REGION_MARKER_TRANSPARENCY_DISC);
    poly.set_transparency(IGNORED_REGION_MARKER_TRANSPARENCY_DISC);
    marker.set_color("primary_color", "yellow");
    marker.set_color("secondary_color", "green");
  }
  else
  {
    poly.set_active(false);
    marker.set_color("primary_color", "white");
    marker.set_color("secondary_color", "gray50");
  }

  ignoredRegion.setMarker(marker);
  ignoredRegion.setRegion(poly);
  m_ignoredRegionset.modIgnoredRegion(ignoredRegion);

  Notify("VIEW_POLYGON", poly.get_spec());
  Notify("VIEW_MARKER", marker.get_spec());
}

//------------------------------------------------------------
// Procedure: postPulseMessage()

void FireSim::postFirePulseMessage(Fire fire, double time, std::string discoverer)
{
  XYRangePulse pulse;
  pulse.set_x(fire.getCurrX());
  pulse.set_y(fire.getCurrY());

  pulse.set_label("pulse_" + fire.getID());
  pulse.set_label_color("off");

  pulse.set_rad(FIRE_PULSE_RANGE);
  pulse.set_time(time);

  std::string color = m_fire_color;
  if (!discoverer.empty() && discoverer != "nature")
    color = m_map_node_records[discoverer].getColor();

  pulse.set_color("edge", color);
  pulse.set_color("fill", m_fire_color);

  pulse.set_duration(PULSE_DURATION);

  std::string spec = pulse.get_spec();
  Notify("VIEW_RANGE_PULSE", spec);
}

//------------------------------------------------------------
// Procedure: postPulseMessage()

void FireSim::postIgnoredRegionPulseMessage(IgnoredRegion ignoredRegion, double time, std::string discoverer)
{

  XYMarker marker = ignoredRegion.getMarker();

  XYRangePulse pulse;

  pulse.set_x(marker.get_vx());
  pulse.set_y(marker.get_vy());

  pulse.set_label("pulse_" + ignoredRegion.getID());
  pulse.set_label_color("off");

  pulse.set_rad(IGNORED_REGION_PULSE_RANGE);
  pulse.set_time(time);

  std::string color = "white";
  if (!discoverer.empty() && discoverer != "nature")
    color = m_map_node_records[discoverer].getColor();

  pulse.set_color("edge", color);
  pulse.set_color("fill", m_fire_color);

  pulse.set_duration(PULSE_DURATION);

  std::string spec = pulse.get_spec();
  Notify("VIEW_RANGE_PULSE", spec);
}

//------------------------------------------------------------
// Procedure: postFlags()

void FireSim::postFlags(const std::vector<VarDataPair> &flags)
{
  for (unsigned int i = 0; i < flags.size(); i++)
  {
    VarDataPair pair = flags[i];
    std::string moosvar = pair.get_var();

    // If posting is a double, just post. No macro expansion
    if (!pair.is_string())
    {
      double dval = pair.get_ddata();
      Notify(moosvar, dval);
    }
    // Otherwise if string posting, handle macro expansion
    else
    {
      std::string sval = pair.get_sdata();
      sval = macroExpand(sval, "LEADER", m_vname_leader);
      sval = macroExpand(sval, "WINNER", m_vname_winner);

      Notify(moosvar, sval);
    }
  }
}

//------------------------------------------------------------
// Procedure: addNotable()
//   Purpose: The notables map is where we keep track of (a)
//            the most recent fires discovered for each vehicle
//            When we have data for all vehicles, we use this
//            map to pop-off equal amounts of fires for each
//            vehicle until some vehicle is an empty list. This
//            way the remaining fires are the "notable" once
//            since they represent the most recent fires that
//            provide the leading vehicle with the lead.

void FireSim::addNotable(std::string vname, std::string fname)
{
  if (vname == "nature")
    return;

  m_map_notables[vname].push_front(fname);

  bool some_empty = false;
  for (const auto &[_, fires] : m_map_notables)
    if (fires.size() == 0)
      some_empty = true;

  if (some_empty ||
      (m_map_notables.size() < m_total_discoverers) ||
      (m_map_notables.size() == 1))
    return;

  for (auto &[_, fires] : m_map_notables)
    fires.pop_back();
}

//------------------------------------------------------------
// Procedure: isNotable()

bool FireSim::isNotable(std::string fname)
{
  for (const auto &[_, fires] : m_map_notables)
    if (listContains(fires, fname))
      return (true);

  return (false);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool FireSim::buildReport()
{
  std::string str_rng_min = doubleToStringX(m_detect_rng_min, 1);
  std::string str_rng_max = doubleToStringX(m_detect_rng_max, 1);
  std::string str_rng_pd = doubleToStringX(m_detect_rng_pd, 2);
  std::string str_trans = doubleToString(m_scout_rng_transparency, 2);

  m_msgs << "======================================" << std::endl;
  m_msgs << "FireSim Configuration " << std::endl;
  m_msgs << "======================================" << std::endl;
  m_msgs << "detect_rng_min   : " << str_rng_min << std::endl;
  m_msgs << "detect_rng_max   : " << str_rng_max << std::endl;
  m_msgs << "detect_rng_pd    : " << str_rng_pd << std::endl;
  m_msgs << "detect_rng_show  : " << boolToString(m_scout_rng_show) << std::endl;
  m_msgs << "detect_alt_max   : " << doubleToString(m_detect_alt_max, 1) << std::endl;
  m_msgs << "detect_rng_fixed : " << boolToString(m_detect_rng_fixed) << std::endl;
  m_msgs << "      fire_color : " << m_fire_color << std::endl;
  m_msgs << "fire_transparency: " << str_trans << std::endl;
  m_msgs << "        fire_file: " << m_fireset.getFireFile() << std::endl;
  m_msgs << "     planner mode: " << Planner::modeToString(m_planner_mode) << std::endl;
  m_msgs << std::endl;

  m_msgs << "======================================" << std::endl;
  m_msgs << "Mission Summary " << std::endl;
  m_msgs << "======================================" << std::endl;

  auto undiscovered = m_fireset.size() - m_fireset.getTotalFiresDiscovered();
  std::string finished_str = boolToString(m_finished);
  finished_str += " (" + uintToString(undiscovered) + " remaining)";
  std::string running = boolToString(isMissionRunning());

  m_msgs << "     Total Ignored Regions: " << m_ignoredRegionset.size() << std::endl;
  m_msgs << " Spawnable Ignored Regions: " << m_ignoredRegionset.spawnsize() << std::endl;
  m_msgs << "       Total Fires: " << m_fireset.size() << std::endl;
  m_msgs << "   Spawnable Fires: " << m_fireset.spawnsize() << std::endl;
  m_msgs << "Scorer Impute Time: " << boolToString(m_imputeTime) << std::endl;
  m_msgs << "Mission Running (" << running << ")";
  if (m_mission_start_utc != 0)
  {
    m_msgs << std::endl;
    m_msgs << "     Start time: " << doubleToString(m_mission_start_utc, 1) << " / 0s" << std::endl;
    m_msgs << "       Duration: " << doubleToString(m_mission_duration_s, 1) << "s" << std::endl;
    if (!m_finished)
    {
      m_msgs << "   Elapsed time: " << doubleToString(m_curr_time - m_mission_start_utc, 3) << std::endl;
      m_msgs << " Time remaining: " << doubleToString(m_mission_duration_s - (m_curr_time - m_mission_start_utc), 0) << std::endl;
    }
    else
    {
      m_msgs << "   Finished time: " << doubleToString(m_mission_endtime_utc, 1) << " / " << doubleToString(m_mission_endtime_utc - m_mission_start_utc, 1) << "s" << std::endl;
    }
    m_msgs << "Mission Finished: " << finished_str << std::endl;
  }
  m_msgs << std::endl;

  m_msgs << "======================================" << std::endl;
  m_msgs << "Vehicle Discover Summary " << std::endl;
  m_msgs << "======================================" << std::endl;

  m_msgs << "Total vehicles: " << m_map_node_records.size() << std::endl;
  m_msgs << "Leader vehicle: " << m_vname_leader << std::endl;
  m_msgs << "Winner vehicle: " << ((m_vname_winner.empty()) ? "-" : m_vname_winner) << std::endl;

  ACTable actab = ACTable(4);
  actab << "Vehi | Fires       | Scout | Scout ";
  actab << "Name | Discovered  | Reqs  | Tries  ";
  actab.addHeaderLines();

  for (const auto &[vname, _] : m_map_node_records)
  {
    std::string discoveries = uintToString(m_map_node_discoveries[vname]);
    std::string sc_reqs = uintToString(m_map_node_scout_reqs[vname]);
    std::string sc_tries = uintToString(m_map_node_scout_tries[vname]);
    actab << vname << discoveries << sc_reqs << sc_tries;
  }
  m_msgs << actab.getFormattedString();
  m_msgs << std::endl
         << std::endl;

  m_msgs << "======================================" << std::endl;
  m_msgs << "Fire Summary " << std::endl;
  m_msgs << "======================================" << std::endl;
  actab = ACTable(9);
  actab << "Name | ID | Pos| State | Discoveries | Discoverer| Tries | Scouts | Time ";
  actab.addHeaderLines();

  std::set<std::string> fire_names = m_fireset.getFireNames();

  std::set<std::string>::iterator p;
  for (p = fire_names.begin(); p != fire_names.end(); p++)
  {
    std::string fname = *p;
    Fire fire = m_fireset.getFire(fname);
    std::string id = fire.getID();
    std::string state = FireStateToString(fire.getState());
    std::string tries = uintToString(fire.getScoutTries());

    std::string discoveries = uintToString(fire.getDiscoverCnt());
    // Logger::info("FireSim::buildReport: DiscCount: " + discoveries);

    double xpos = fire.getCurrX();
    double ypos = fire.getCurrY();
    std::string pos = doubleToStringX(xpos, 0) + "," + doubleToStringX(ypos, 0);

    std::string discoverer = "-";
    double duration = m_curr_time - fire.getTimeEnter();
    if (state == FireStateToString(Fire::DISCOVERED))
    {
      duration = fire.getTimeDiscovered() - fire.getTimeEnter();
      discoverer = fire.getDiscoverer();
    }

    std::set<std::string> scout_set = fire.getScoutSet();
    std::string scouts = stringSetToString(scout_set);

    std::string dur_str = doubleToStringX(duration, 1);

    actab << fname << id << pos << state << discoveries;
    actab << discoverer << tries << scouts << dur_str;
  }
  m_msgs << actab.getFormattedString();
  m_msgs << std::endl
         << std::endl;

  // Logger::info("\n");

  return (true);
}

//------------------------------------------------------------
// Procedure: calculateMissionScore()

void FireSim::calculateMissionScore(bool imputeTime)
{
  if (m_mission_scorer.isScoreCalculated())
    return;

  Logger::info("Calculating mission score");
  // Calculate score using the FireSet
  double score = m_mission_scorer.calculateScoreFromFireSet(m_fireset, imputeTime);

  // Publish score information to the MOOSDB
  std::function<void(std::string, std::string)> reportFnc = [&, this](std::string key, std::string value)
  {
    this->Notify(key, value);
  };
  m_mission_scorer.PublishScore(reportFnc);

  // Save score to file
  auto totalFires = m_fireset.size();
  auto min_sep = m_fireset.getMinSeparation() * MOOSDIST2METERS;

  std::string sep_str = (min_sep) ? "_sep" + uintToString(min_sep, 0) : "";

  // Get the current date and time
  std::time_t now = std::time(nullptr);
  std::tm *tm = std::localtime(&now);
  char buffer[80];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm);
  std::string date_str(buffer);
  // date_str = "_" + date_str.substr(0, 10) + "_" + date_str.substr(11, 5);

  std::string score_filename = "mission_score_f" + uintToString(totalFires, 0) + "_" + date_str + ".txt";

  std::string file_path = m_mission_score_save_path + score_filename;

  m_mission_scorer.SaveScoreToFile(file_path);
  Notify("PLOGGER_CMD", "COPY_FILE_REQUEST=" + file_path);

  // Send score summary to info_buffer for appcast
  reportEvent("Mission Score: " + doubleToStringX(score, 2) + "/100");
  reportEvent("Score details saved to: " + file_path);

  Logger::info("Mission Score: " + doubleToStringX(score, 2) + "/100");
  Logger::info("Score details saved to: " + file_path);
}
