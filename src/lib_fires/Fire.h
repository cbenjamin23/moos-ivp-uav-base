/************************************************************/
/*    NAME: Michael Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Fire.h                                          */
/*    DATE: Apr 2nd, 2022                                   */
/************************************************************/

#ifndef FIRE_HEADER
#define FIRE_HEADER

#include <string>
#include <list>
#include <set>

class Fire
{
 public:
  Fire(std::string fname="");
  ~Fire() {}; 

  void   initXY(double, double);
  
  enum FireState {UNDISCOVERED, DISCOVERED, UNKNOWN};


 public: // Setters
  void   setStartX(double v) {m_start_x=v;}
  void   setStartY(double v) {m_start_y=v;}
  void   setCurrX(double v)  {m_curr_x=v;}
  void   setCurrY(double v)  {m_curr_y=v;}
    
  void   setTimeEnter(double v)   {m_time_enter=v;}
  void   setTimeDiscovered(double v) {m_time_discovered=v;}

  bool   setStateFromString(std::string s);
  bool   setState(FireState s);
  void   setDiscoverer(std::string s)   {m_discoverer=s;}
  void   setName(std::string s)         {m_name=s;}
  void   setID(std::string s)           {m_id=s;}
  void   setScoutTries(unsigned int v)  {m_scout_tries=v;}
  void   addScouted(std::string s)      {m_set_scouted.insert(s);}
  void   incDiscoverCnt()               {m_discoverCnt++;}
  void   incScoutTries()                {m_scout_tries++;}
  
 public: // Getters
  double getStartX() const  {return(m_start_x);}
  double getStartY() const  {return(m_start_y);}
  double getCurrX() const   {return(m_curr_x);}
  double getCurrY() const   {return(m_curr_y);}

  double getTimeEnter() const {return(m_time_enter);}
  double getTimeDiscovered() const {return(m_time_discovered);}

  FireState getState() const  {return(m_state);}
  std::string  getDiscoverer() const {return(m_discoverer);}
  std::string  getName() const   {return(m_name);}
  std::string  getID() const     {return(m_id);}
  unsigned int getDiscoverCnt() const {return(m_discoverCnt);}
  unsigned int getScoutTries() const   {return(m_scout_tries);}

  std::set<std::string> getScoutSet() {return(m_set_scouted);}
  bool hasBeenScouted(std::string vname="") const;
  
  std::string getSpec() const;
  
 private:  
  double       m_start_x;
  double       m_start_y;
  double       m_curr_x;
  double       m_curr_y;
  double       m_time_enter;   // time fire started
  double       m_time_discovered; // time discovered
  FireState    m_state;        // undiscovered or discovered
  std::string  m_discoverer;   // who discovered
  std::string  m_name;         // key identifier
  std::string  m_id;       
  
  unsigned int m_discoverCnt;  // number of times discovered
  std::set<std::string> m_set_scouted;
  unsigned int m_scout_tries;
};

Fire stringToFire(std::string);

std::string FireStateToString(Fire::FireState state);

Fire::FireState stringToFireState(std::string state);

#endif
