#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

class TakeoffConfirmationTracker
{
public:
  using Clock = std::chrono::steady_clock;

  enum class TimeoutPhase
  {
    None,
    Activation,
    Completion
  };

  struct Outcome
  {
    uint64_t command_id = 0;
    bool confirmed = false;
    bool completed = false;
    bool aborted = false;
    TimeoutPhase timeout_phase = TimeoutPhase::None;
  };

  void accept(uint64_t command_id,
              double target_altitude_m,
              bool completion_required,
              Clock::time_point accepted_at = Clock::now())
  {
    m_pending = Pending{command_id, target_altitude_m, completion_required,
                        false, accepted_at};
  }

  bool hasPending() const { return m_pending.has_value(); }

  double targetAltitudeM() const
  {
    return m_pending ? m_pending->target_altitude_m : 0.0;
  }

  Outcome evaluate(bool takeoff_active,
                   bool target_reached,
                   bool aborted,
                   Clock::time_point now = Clock::now(),
                   double activation_timeout_s = 5.0,
                   double completion_timeout_s = 60.0)
  {
    if (!m_pending)
      return {};

    Outcome outcome;
    outcome.command_id = m_pending->command_id;

    if (!m_pending->confirmed && takeoff_active)
    {
      m_pending->confirmed = true;
      outcome.confirmed = true;
    }

    if (m_pending->confirmed && m_pending->completion_required && target_reached)
    {
      outcome.completed = true;
      m_pending.reset();
      return outcome;
    }

    if (m_pending->confirmed && !m_pending->completion_required)
    {
      m_pending.reset();
      return outcome;
    }

    if (aborted)
    {
      outcome.aborted = true;
      m_pending.reset();
      return outcome;
    }

    const double age_s =
        std::chrono::duration<double>(now - m_pending->accepted_at).count();
    if (!m_pending->confirmed && age_s >= activation_timeout_s)
    {
      outcome.timeout_phase = TimeoutPhase::Activation;
      m_pending.reset();
    }
    else if (m_pending->confirmed && age_s >= completion_timeout_s)
    {
      outcome.timeout_phase = TimeoutPhase::Completion;
      m_pending.reset();
    }

    return outcome;
  }

private:
  struct Pending
  {
    uint64_t command_id;
    double target_altitude_m;
    bool completion_required;
    bool confirmed;
    Clock::time_point accepted_at;
  };

  std::optional<Pending> m_pending;
};
