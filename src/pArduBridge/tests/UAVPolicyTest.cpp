#include "UAV_Model.h"

namespace
{
UAV_Model::LandPolicyInputs airborneIn(mavsdk::Telemetry::FlightMode flight_mode)
{
  return {flight_mode, true, 0.1, mavsdk::Telemetry::LandedState::InAir};
}

bool expectDecision(const UAV_Model::PolicyDecision &decision,
                    UAV_Model::PolicyAction action,
                    const std::string &reason)
{
  return decision.action == action && decision.reason == reason;
}
}

int main()
{
  using Action = UAV_Model::PolicyAction;
  using Mode = mavsdk::Telemetry::FlightMode;

  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Offboard)), Action::Submit, "READY"))
    return 1;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Guided)), Action::Submit, "READY"))
    return 2;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Mission)), Action::Submit, "READY"))
    return 3;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Hold)), Action::Submit, "READY"))
    return 4;

  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::ReturnToLaunch)), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 5;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Stabilized)), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 6;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(airborneIn(Mode::Unknown)), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 7;

  auto on_ground = airborneIn(Mode::Offboard);
  on_ground.landed_state = mavsdk::Telemetry::LandedState::OnGround;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(on_ground), Action::NoOp, "ALREADY_ON_GROUND"))
    return 8;

  auto landing = airborneIn(Mode::Offboard);
  landing.landed_state = mavsdk::Telemetry::LandedState::Landing;
  if (!expectDecision(UAV_Model::evaluateLandPolicy(landing), Action::NoOp, "ALREADY_LANDING"))
    return 9;

  const auto precision = [](bool copter, bool enable, bool armed,
                            bool enter_loiter, Mode mode,
                            bool enabled_available = true,
                            bool enabled = true,
                            bool type_available = true,
                            int32_t type = 1)
  {
    return UAV_Model::evaluatePrecisionLoiterPolicy(
        {copter, enable, armed, enter_loiter, mode,
         enabled_available, enabled, type_available, type});
  };

  if (!expectDecision(precision(true, true, true, true, Mode::Guided), Action::Submit, "READY"))
    return 10;
  if (!expectDecision(precision(true, true, true, true, Mode::Offboard), Action::Submit, "READY"))
    return 11;
  if (!expectDecision(precision(true, true, true, false, Mode::Hold), Action::Submit, "READY"))
    return 12;
  if (!expectDecision(precision(true, true, false, true, Mode::Guided), Action::Reject, "NOT_ARMED"))
    return 13;
  if (!expectDecision(precision(true, true, true, false, Mode::Guided), Action::Reject, "FC_LOITER_REQUIRED"))
    return 14;
  if (!expectDecision(precision(true, true, true, true, Mode::ReturnToLaunch), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 15;
  if (!expectDecision(precision(false, true, true, true, Mode::Guided), Action::Reject, "COPTER_ONLY"))
    return 16;
  if (!expectDecision(precision(true, false, false, false, Mode::ReturnToLaunch), Action::Submit, "READY"))
    return 17;
  if (!expectDecision(precision(false, false, false, false, Mode::Unknown), Action::Reject, "COPTER_ONLY"))
    return 18;
  if (!expectDecision(precision(true, true, true, true, Mode::Guided, false), Action::Reject, "PLND_ENABLED_UNAVAILABLE"))
    return 19;
  if (!expectDecision(precision(true, true, true, true, Mode::Guided, true, false), Action::Reject, "PLND_DISABLED"))
    return 20;
  if (!expectDecision(precision(true, true, true, true, Mode::Guided, true, true, false), Action::Reject, "PLND_TYPE_UNAVAILABLE"))
    return 21;
  if (!expectDecision(precision(true, true, true, true, Mode::Guided, true, true, true, 0), Action::Reject, "PLND_TYPE_NONE"))
    return 22;
  if (!expectDecision(precision(true, true, true, true, Mode::Mission), Action::Submit, "READY"))
    return 23;
  if (!expectDecision(precision(true, true, true, true, Mode::Land), Action::Submit, "READY"))
    return 24;
  if (!expectDecision(precision(true, true, true, true, Mode::Stabilized), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 25;
  if (!expectDecision(precision(true, true, true, true, Mode::Manual), Action::Reject, "FLIGHT_MODE_NOT_ALLOWED"))
    return 26;
}
