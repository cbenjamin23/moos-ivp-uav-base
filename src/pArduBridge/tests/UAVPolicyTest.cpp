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
}
