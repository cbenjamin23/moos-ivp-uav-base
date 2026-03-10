## LocalAuctionReservation

`LocalAuctionReservation` is a tiny process-local helper for spawned task
behaviors that should not bid on multiple auctions at once on the same vehicle.

It provides one shared reservation per process:

- `heldByOther(task_hash, now)`
- `claim(task_hash, now)`
- `release(task_hash)`
- `maintainForState(task_hash, auction_state, now)`

In the current refuel/replace flow, the reservation value is the task hash.
The library now names it that way directly because that is how it is actually
used at every call site today.

### What It Does

- Lets the first feasible spawned behavior claim a short-lived reservation.
- Lets that same behavior refresh the reservation while it remains in
  `bidding` or `bidwon`.
- Makes sibling behaviors on the same helm abstain while the reservation is
  held by a different task hash.
- Releases the reservation on `bidlost` or `abstain`.
- Expires stale reservations after 20 seconds so a dead behavior does not
  wedge the process forever.

### What It Does Not Do

- It is not a fleet-wide lock.
- It is not a mission-execution lock.
- It does not know who won the team auction.
- It does not coordinate across MOOS apps.
- It is not thread-safe.
- It only tracks one reservation at a time.

The longer-lived "I actually won this task and am busy executing it" lock still
belongs in the mission app, such as `pRefuelReplace`.

### How The Reservation Exists

This helper is just a tiny shared library loaded into the same `pHelmIvP`
process as the spawned task behaviors. The file-scope variables in
`LocalAuctionReservation.cpp` are created once when that library is loaded:

- `g_reserved_task_hash`
- `g_last_refresh_time`

Because the spawned behaviors are running inside that same process, they all
call into the same functions and see the same two variables. There is no MOOS
message needed to create the reservation. The first call to `claim()` simply
stores a task hash in that shared process-local state.

### What Happens If Everything Crashes

This reservation lives only in process memory.

That means:

- if `pHelmIvP` exits normally, the reservation disappears
- if `pHelmIvP` crashes, the reservation disappears
- if the whole mission stack or computer restarts, the reservation disappears

There is no file, database, or MOOS variable that persists this state across a
crash. On restart, the helper begins again with:

- `g_reserved_task_hash = ""`
- `g_last_refresh_time = 0.0`

So crash recovery is simple: the local reservation is automatically reset
because the process memory that held it no longer exists.

### Where It Gets Loaded

In the current mission, `pHelmIvP` is pointed at a behavior file with:

- `behaviors = targ_$(VNAME).bhv`

That behavior file contains blocks such as:

- `Behavior = BHV_TaskRefuelReplaceTarget`
- `Behavior = BHV_TaskRefuelReplaceBasic`

Those behavior plugins are linked against `local_auction_reservation`, so when
`pHelmIvP` loads the behavior plugin, the dynamic loader also loads
`liblocal_auction_reservation`.

In this mission specifically, the reason the reservation library exists even
before any task behavior has claimed it is:

- `pHelmIvP` is configured to use a `.bhv` file
- that `.bhv` file contains the refuel behavior plugin types
- those behavior plugins are linked against `liblocal_auction_reservation`
- when the behavior plugin is loaded, that dependency is loaded too

So the reservation module is already present in the `pHelmIvP` process, with an
empty reservation, before the first spawned task behavior actually uses it.

That means:

- the behaviors and this helper all live in the same `pHelmIvP` process
- the helper's file-scope variables exist once for that process
- every spawned behavior instance on that vehicle sees the same reservation

This is why the local reservation works without any MOOS publication or
subscription of its own.

### What `maintainForState()` Does

`maintainForState(task_hash, auction_state, now)` is a small lifecycle helper:

- if the task is `bidding` or `bidwon`, refresh the reservation for that hash
- if the task is `bidlost` or `abstain`, release the reservation for that hash
- otherwise, just expire stale reservations if the timeout has passed

This is useful because the behaviors already track auction state every cycle.
Without `maintainForState()`, each call site would have to repeat the same
"refresh while active, release when finished, expire if stale" logic by hand.
The helper keeps that policy in one place so the two refuel behaviors stay
consistent.

### Where `auction_state` Comes From

`maintainForState()` does not discover the current state on its own. The caller
passes the state string in.

In the current behaviors, the call looks like:

- `maintainForState(m_task_hash, m_task_state, now)`

Here:

- `m_task_hash` is the current task's hash
- `m_task_state` is the current auction/task state already tracked by the task
  behavior base class

The helper then runs `normalizeState(auction_state)` internally. That function
does not fetch state from anywhere else. It just trims whitespace and converts
the passed-in string to lower case so comparisons like `bidding`, `bidwon`,
`bidlost`, and `abstain` are reliable.

### Why It Still Refreshes On `bidwon`

This local reservation and the mission-app lock serve different timing windows.

- The local reservation lives inside `pHelmIvP`.
- The longer mission-execution lock lives in the mission app, such as
  `pRefuelReplace`.

After a behavior becomes `bidwon`, the mission app still needs a short amount of
time to receive and process `TASK_STATE`, set its own active-task lock, publish
its busy variable, and have sibling behaviors observe that busy variable.

Refreshing the local reservation during `bidwon` keeps the local guard in place
during that handoff window. In other words, the two locks overlap briefly on
purpose so the vehicle does not momentarily become eligible to bid again while
the mission app lock is catching up.

If a future mission wants the local reservation to be strictly pre-award only,
this helper could be changed to refresh only on `bidding`. The current version
chooses safety over that sharper separation.

### How To Reuse This In Another Mission

To reimplement the same pattern in another mission:

1. Build one or more spawned task behaviors that may compete locally on the same
   vehicle.
2. Link those behavior plugins against `local_auction_reservation` in the
   behavior library `CMakeLists.txt` with `TARGET_LINK_LIBRARIES(...)`.
   In this repo, both refuel behaviors do exactly that.
3. Give each spawned task instance a stable per-task identifier, typically the
   task hash.
4. In each behavior's run path, call
   `maintainForState(task_hash, task_state, now)`.
5. In each behavior's feasibility check, call `heldByOther(task_hash, now)` and
   abstain if it returns true.
6. When a behavior is feasible and actively participating in the auction, call
   `claim(task_hash, now)` so sibling behaviors in the same process see the
   reservation immediately.
7. Pair this local reservation with a separate mission-side lock if the vehicle
   should remain busy after it actually wins.

The key design rule is:

- use `LocalAuctionReservation` to prevent local double-bids before and during
  auction resolution
- use a mission-side lock to prevent new work after a task is actually won

If another mission has only one spawned auction behavior per vehicle, this
helper may not be needed at all. Its value appears when multiple local behaviors
could otherwise race each other on the same helm.

### Why The Structure Is Simple

- The state is a single reserved task hash plus a last-refresh timestamp.
- The API is four functions and no object lifecycle.
- The behavior code can use it without additional MOOS messages or config.

### Potential Future Improvements

- Add configurable timeout support instead of keeping the 20 second timeout
  hard-coded.
- Replace raw state strings with a stronger typed interface if the task-behavior
  stack ever makes that practical.
- Add optional lock groups so different families of local auctions could share
  or isolate reservations intentionally. For example, one mission might want a
  single global reservation for all auxiliary tasks, while another might want
  separate groups for replacement, payload-swap, or recovery tasks.
- Wrap the state in a small class if stricter testability or multiple
  independent reservation instances ever become necessary.
- Keep pairing this helper with a separate mission-side commitment lock whenever
  a vehicle must remain busy after actually winning a task.

For the current use case, this tradeoff is reasonable: it stays light, works
with the existing spawned behavior flow, and avoids introducing another MOOS
protocol just to prevent local double-bids.
