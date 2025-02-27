/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA             */
/*    FILE: FireSet.cpp                                          */
/*    DATE: February 2022                                        */
/*****************************************************************/


#include <iterator>
#include <algorithm>
#include <cmath>
#include "FireSet.h"
#include "ColorParse.h"
#include "MBUtils.h"
#include "FileBuffer.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

FireSet::FireSet()
{
  m_total_reg = 0;
  m_total_unreg = 0;
  m_max_size = 99;
  shuffleIDs();
}

bool FireSet::handleFireFile(string str, double curr_time, string& warning)
{
  vector<string> lines = fileBuffer(str);
  if(lines.size() == 0) {
    warning = "File not found, or empty: " + str;
    return(false);
  }
  
  for(unsigned int i=0; i<lines.size(); i++) {
    string orig = lines[i];
    string line = stripBlankEnds(stripComment(lines[i], "//"));
    if(line == "")
      continue;
    
    string param = biteStringX(line, '=');
    string value = line;
    
    if(param == "fire") {
      Fire fire = stringToFire(value);
      string fname = fire.getName();
      if(m_map_fires.count(fname) != 0) {
        warning = "Bad FireFile Line: " + orig;
        return(false);
      }
      if(fire.getType() == "reg")
        m_total_reg++;
      else
        m_total_unreg++;
      
      fire.setTimeEnter(curr_time);
      tagFireID(fire);
      m_map_fires[fname] = fire;
    }
    else if((param == "region") || (param == "poly")) {
      m_search_region = string2Poly(value);
      m_search_region.set_color("edge", "gray90");
      m_search_region.set_color("vertex", "dodger_blue");
      m_search_region.set_vertex_size(5);
      if(!m_search_region.is_convex()) {
        warning = "Bad FireFile Line: " + orig;
        return(false);
      }
    }
  }

  m_fire_file = str;
  return(true);
}

vector<Fire> FireSet::getFires() const
{
  vector<Fire> fvector;

  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    Fire fire = p->second;
    fvector.push_back(fire);
  }

  return(fvector);
}

vector<string> FireSet::getFireFileSpec() const
{
  vector<string> svector;

  svector.push_back("poly = " + m_search_region.get_spec_pts());

  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    string name = p->first;
    Fire fire = p->second;
    double start_x = fire.getStartX();
    double start_y = fire.getStartX();
    string x_str = doubleToStringX(start_x);
    string y_str = doubleToStringX(start_y);

    string line = "fire = name=" + name;
    line += ", x=" + x_str + ", y=" + y_str;
    svector.push_back(line);
  }

  return(svector);
}

//---------------------------------------------------------
// Procedure: fireAlert()
//   Example: type=reg, x=2, y=3, name=joe
//            type=unreg, x=2, y=3
//      Note: XFIRE_ALERT 

bool FireSet::fireAlert(string str, double curr_time, string& warning)
{
  string fname = tokStringParse(str, "name");
  string ftype = tokStringParse(str, "type");
  double x = tokDoubleParse(str, "x");
  double y = tokDoubleParse(str, "y");  

  return(addFire(fname, ftype, x, y, curr_time, warning));
}

bool FireSet::addFire(string fname, string ftype,
                      double x, double y,
                      double curr_time,
                      string& warning)
{
  if((ftype != "reg") && (ftype != "unreg")) {
    warning = "Fire Alert with unknown type:[" + fname + "]";
    return(false);
  }
  if((fname != "") && (m_map_fires.count(fname))) {
    warning = "Fire Alert with duplicate name:" + fname;
    return(false);
  }
  
  if(fname == "") {
    if(ftype == "reg") {
      m_total_reg++;
      fname = "f" + uintToString(m_total_reg, 2);
    }
    else {
      m_total_unreg++;
      fname = "h" + uintToString(m_total_unreg, 2);
    }
  }
  
  Fire new_fire(fname);
  new_fire.initXY(x, y);
  new_fire.setType(ftype);
  new_fire.setTimeEnter(curr_time);
  tagFireID(new_fire);
  
  m_map_fires[fname] = new_fire;
  return(true);
}

string FireSet::getNameClosestFire(double x, double y, double min_range)
{
  if(m_map_fires.size() == 0)
    return("");
  
  string closest_fname;
  double closest_range = -1;
  
  map<string, Fire>::iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    string fname = p->first;
    Fire fire = p->second;
    if(fire.getState() == "discovered")
      continue;

    double curr_x = fire.getCurrX();
    double curr_y = fire.getCurrY();
    double range = hypot(x-curr_x, y-curr_y);
    if(range > min_range)
      continue;
    
    if((closest_range < 0) || (range < closest_range)) {
      closest_fname = fname;
      closest_range = range;
    }
  }

  return(closest_fname);
}

set<string> FireSet::getFireNames() const
{
  set<string> fire_names;
  
  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    string fname = p->first;
    fire_names.insert(fname);
  }
  return(fire_names);
}

bool FireSet::modFire(Fire fire)
{
  string fname = fire.getName();
  if(m_map_fires.count(fname) == 0)
    return(false);

  m_map_fires[fname] = fire;
  return(true);
}
  
bool FireSet::hasFire(string fname) const
{
  if(m_map_fires.count(fname) == 0)
    return(false);
  return(true);
}
  
Fire FireSet::getFire(string fname) const
{
  Fire null_fire;
  if(m_map_fires.count(fname) == 0)
    return(null_fire);
  
  return(m_map_fires.at(fname));
}

bool FireSet::hasFireByID(string id) const
{
  if(m_map_fire_ids.count(id) == 0)
    return(false);
  return(true);
}
  
Fire FireSet::getFireByID(string id) const
{
  Fire null_fire;
  if(m_map_fire_ids.count(id) == 0)
    return(null_fire);

  string fname = m_map_fire_ids.at(id);
  if(m_map_fires.count(fname) == 0)
    return(null_fire);

  return(m_map_fires.at(fname));
}

void FireSet::shuffleIDs()
{
  m_shuffled_ids.clear();
  for(unsigned int i=0; i<m_max_size; i++)
    m_shuffled_ids.push_back(i);

  random_shuffle(m_shuffled_ids.begin(), m_shuffled_ids.end());
}

void FireSet::tagFireID(Fire& fire)
{
  string new_id;
  if(m_map_fires.size() >= m_shuffled_ids.size())
    new_id = "id" + uintToString(m_map_fires.size(), 2);
  else {
    unsigned int next_ix = m_map_fires.size();
    new_id = "id" + uintToString(m_shuffled_ids[next_ix], 2);
  }
  
  fire.setID(new_id);
  m_map_fire_ids[new_id] = fire.getName();
}

unsigned int FireSet::getDiscoveredFireCnt(string vname) const
{
  if(vname == "")
    return(0);

  vname = tolower(vname);
  
  unsigned int total = 0;
  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    Fire fire = p->second;
    if(vname == tolower(fire.getDiscoverer()))
      total++;
  }
  
  return(total);
}
//------------------------------------------------------------
// Procedure: getKnownFireCnt()
//      Note: Known fires are fires that were either of
//            type "reg" originally, or "unreg" fires that
//            were scouted by one or more vehicles.
//      Note: This criteria may be used to help determine the
//            end of the mission. 

unsigned int FireSet::getKnownFireCnt() const
{
  unsigned int total = 0;

  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    Fire fire = p->second;
    if(fire.getType() == "reg")
      total++;
    else if(fire.getScoutSet().size() != 0)
      total++;
  }
  
  return(total);
}

//------------------------------------------------------------
// Procedure: allFiresDiscovered() 
//   Purpose: Determine if all fires, registered or not, have
//            been discovered. 

bool FireSet::allFiresDiscovered() const
{
  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    Fire fire = p->second;
    if(fire.getDiscoverer() == "")
      return(false);
  }
  
  return(true);
}

//------------------------------------------------------------
// Procedure: allRegFiresDiscovered()
//   Purpose: Determine if all known fires, e.g., all registered
//            fires, have been rescued. 

bool FireSet::allRegFiresDiscovered() const
{
  map<string, Fire>::const_iterator p;
  for(p=m_map_fires.begin(); p!=m_map_fires.end(); p++) {
    Fire fire = p->second;
    if(fire.getType() == "reg") {
      if(fire.getDiscoverer() == "")
        return(false);
    }
  }
  
  return(true);
}
