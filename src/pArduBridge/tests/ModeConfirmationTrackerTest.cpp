#include "ModeConfirmationTracker.h"

#include <chrono>

int main()
{
  using namespace std::chrono_literals;
  using Status = ModeConfirmationTracker::OutcomeStatus;

  const auto start = ModeConfirmationTracker::Clock::now();
  ModeConfirmationTracker tracker;

  if (tracker.evaluate(false, start).status != Status::None)
    return 1;

  tracker.accept(17, start);
  if (tracker.evaluate(false, start + 4999ms).status != Status::None)
    return 2;

  const auto confirmed = tracker.evaluate(true, start + 4999ms);
  if (confirmed.status != Status::Confirmed || confirmed.command_id != 17)
    return 3;
  if (tracker.evaluate(true, start + 5s).status != Status::None)
    return 4;

  tracker.accept(23, start);
  const auto timed_out = tracker.evaluate(false, start + 5s);
  if (timed_out.status != Status::TimedOut || timed_out.command_id != 23)
    return 5;
  if (tracker.evaluate(false, start + 6s).status != Status::None)
    return 6;

  tracker.accept(31, start);
  const auto confirmed_at_deadline = tracker.evaluate(true, start + 5s);
  if (confirmed_at_deadline.status != Status::Confirmed || confirmed_at_deadline.command_id != 31)
    return 7;
}
