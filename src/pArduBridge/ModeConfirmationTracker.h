#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

class ModeConfirmationTracker
{
public:
  using Clock = std::chrono::steady_clock;

  enum class OutcomeStatus
  {
    None,
    Confirmed,
    TimedOut
  };

  struct Outcome
  {
    OutcomeStatus status = OutcomeStatus::None;
    uint64_t command_id = 0;
  };

  void accept(uint64_t command_id, Clock::time_point accepted_at = Clock::now())
  {
    m_pending = Pending{command_id, accepted_at, std::nullopt};
  }

  Outcome evaluate(bool expected_mode_active,
                   Clock::time_point now = Clock::now(),
                   double timeout_s = 5.0,
                   double confirmation_dwell_s = 0.5)
  {
    if (!m_pending)
      return {};

    const uint64_t command_id = m_pending->command_id;
    if (expected_mode_active)
    {
      if (!m_pending->first_match_at)
        m_pending->first_match_at = now;

      // ArduPilot may briefly expose an intermediate requested mode while a
      // rapid transition continues. Require sustained telemetry so CONFIRMED
      // means the mode was established, not merely observed for one sample.
      const double matched_s = std::chrono::duration<double>(
          now - *m_pending->first_match_at).count();
      if (matched_s >= confirmation_dwell_s)
      {
        m_pending.reset();
        return {OutcomeStatus::Confirmed, command_id};
      }
    }
    else
      m_pending->first_match_at.reset();

    const double age_s = std::chrono::duration<double>(now - m_pending->accepted_at).count();
    if (age_s >= timeout_s)
    {
      m_pending.reset();
      return {OutcomeStatus::TimedOut, command_id};
    }

    return {};
  }

private:
  struct Pending
  {
    uint64_t command_id;
    Clock::time_point accepted_at;
    std::optional<Clock::time_point> first_match_at;
  };

  std::optional<Pending> m_pending;
};
