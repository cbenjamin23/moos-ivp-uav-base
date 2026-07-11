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
    m_pending = Pending{command_id, accepted_at};
  }

  Outcome evaluate(bool expected_mode_active,
                   Clock::time_point now = Clock::now(),
                   double timeout_s = 5.0)
  {
    if (!m_pending)
      return {};

    const uint64_t command_id = m_pending->command_id;
    // A matching observation at the deadline is stronger evidence than timeout.
    if (expected_mode_active)
    {
      m_pending.reset();
      return {OutcomeStatus::Confirmed, command_id};
    }

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
  };

  std::optional<Pending> m_pending;
};
