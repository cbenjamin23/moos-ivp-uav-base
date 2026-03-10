# pRefuelReplace

`pRefuelReplace` decides when this vehicle should ask for a replacement, posts the replacement task, and tracks whether this vehicle is already committed to executing one.

It is not the full auction owner. The task manager and spawned task behaviors handle bidding and auction resolution. This app only:

- posts `MISSION_TASK` replacement requests
- keeps enough task metadata to interpret later hash-based task mail
- watches `TASK_STATE` to see whether this vehicle won
- holds one local execution lock while this vehicle is busy carrying out a replacement
- publishes `REFUEL_TRANSIT_BUSY` while that lock is held

## High-Level Model

There are two kinds of state in this app:

1. `m_pending_tasks`

This is a thin metadata map keyed by task hash. It is mainly for tasks that have been announced but are not yet the active commitment on this vehicle.

Each entry stores only the fields that later mission logic may need:

- `id`
- `requester`
- `task_type`
- target location, if any
- last seen time

The most important reason this map still exists is future-proofing for multiple overlapping pending tasks on one vehicle. If several relief requests are in flight at once, the vehicle needs a stable place to remember which hash corresponds to which requester/target before the auction outcome is known.

It also helps with:

- thin hash-based `TASK_STATE` mail that does not repeat full metadata
- late task metadata updates for a hash already seen
- possible future `update/cancel`-style messages keyed by hash

2. `m_active_lock`

This is the one real execution lock. Once a task is won, the active lock becomes self-contained and carries everything needed for post-win mission behavior:

- which task is active
- requester identity
- task type
- target coordinates, if any
- acquisition and refresh times
- whether handoff has happened
- whether return has started
- whether `ODOMETRY_RESET` has been seen

The key simplification in the current structure is:

- the pending map exists for pre-win metadata
- the active lock owns post-win mission execution

After `bidwon`, handoff and release logic no longer depend on looking the task up in the pending map.

## Trigger Paths

This app can post a replacement request in two ways.

### 1. Threshold-triggered

If `ODOMETRY_DIST >= refuel_threshold`, the app posts one replacement task and then latches `m_task_sent=true` so it does not keep reposting the same request.

That latch is only cleared after an `ODOMETRY_RESET` and then a later low-odometry reading. This is handled by:

- `m_waiting_for_odom_reset`
- `maybeRelatchAfterOdometryReset()`

This preserves the original one-shot-per-flight-leg behavior.

### 2. Discovery override

If `REFUEL_DISCOVERY_REQUEST` arrives, the app queues one pending fire id and handles it later in `Iterate()`. This keeps mail handling light and lets posting decisions use current nav, odom, and lock state together.

This path is not a “remember this and post later when fuel gets low” mechanism. It is only an immediate override for the case where a vehicle has just discovered a fire, been reassigned to loiter on it, and is already at or above the normal refuel threshold.

Discovery override requests are:

- dropped if they get too old
- deduped by `fire_id`
- dropped immediately if the vehicle is below threshold
- suppressed if the vehicle already holds an active replacement lock

If a discovery override is dropped because the vehicle is still below threshold, any later replacement auction will come from the normal threshold path, not from the original discovery request.

## Inputs

Main subscribed inputs:

- `NAV_X`, `NAV_Y`
- `ODOMETRY_DIST`
- `RETURN`
- `ODOMETRY_RESET`
- `TASK_RESET`
- `TARGET_RESET`
- `REFUEL_DISCOVERY_REQUEST`
- `OWN_TARGET_X`, `OWN_TARGET_Y`, `OWN_TARGET_WEIGHT`
- `TASK_REFUEL_TARGET`, `TASK_REFUEL_BASIC`
- `TASK_STATE`

## Outputs

Main outputs:

- `FUEL_DISTANCE_REMAINING`
- `MISSION_TASK`
- `NODE_MESSAGE_LOCAL`
- `REFUEL_TRANSIT_BUSY`
- `REFUEL_HANDOFF`
- requester-directed `DEPLOY`, `DO_SURVEY`, `LOITER`, `RETURN`, `TARGET_RESET`

## Task Posting

All task posting goes through `postReplacementTask()`.

That function:

- checks readiness and threshold policy
- creates a task id and hash
- posts either `refuelreplace_target` or `refuelreplace_basic`
- republishes the same task over `NODE_MESSAGE_LOCAL`
- inserts the new task into `m_pending_tasks`
- for a basic task, immediately commands the requester to return

Target task:

- posted when this vehicle still has a target to hand off
- completion is based on the winning replacement vehicle reaching the target handoff radius

Basic task:

- posted when this vehicle no longer has a target to hand off
- completion is based on the winning replacement vehicle finishing its return-reset cycle

## Lock Acquisition And Release

`processTaskState()` is the only place that acquires or releases the active lock from auction outcome mail.

### Acquire

The lock is acquired only when:

- `TASK_STATE` says `state=bidwon`
- the parsed winner matches `m_host_community`
- no other active lock is already held

At that moment, the app copies metadata for that hash from `m_pending_tasks` into `m_active_lock`:

- requester
- task type
- target coordinates and `target_set`

That is the handoff point between the pre-win metadata map and the post-win execution lock.

### Refresh

If another `bidwon` arrives for the same active task, the lock timestamp is refreshed.

If task metadata for the same hash arrives late, `upsertTaskInfo()` patches the active lock directly.

### Release

The lock is released when any of these conditions occur:

- `TASK_STATE` says `bidlost`
- `TASK_STATE` says `abstain`
- `TASK_STATE` says `bidwon` for the same hash but some other winner
- handoff has completed for a target task
- return has started and `ODOMETRY_RESET` has been seen for a basic-like task
- the lock times out

While the lock is engaged, the app publishes:

- `REFUEL_TRANSIT_BUSY=true`

That is the app-side signal that tells sibling task behaviors this vehicle is already committed.

## Why The Pending Map Is Still Needed

The pending map is intentionally much smaller than the old general cache. It exists because `TASK_STATE` is hash-centric and not self-contained.

The strongest reasons to keep it are:

1. Multiple overlapping pending tasks on one vehicle

This is the top likely future need. If several relief auctions are in flight at once, the vehicle needs a keyed place to remember pre-win metadata for each hash until the auction outcome is known.

2. Thin hash-based lifecycle messages

`TASK_STATE` usually tells the app the task hash and state, but not all of:

- requester
- task type
- target coordinates

The app still needs those fields if it later wins that task.

3. Possible future `update/cancel`-style task messages

A keyed metadata map makes it straightforward to support future messages like:

- `TASK_UPDATE hash=...`
- `TASK_CANCEL hash=...`

Without forcing every later message to repeat the full task spec.

### What The Pending Map Is Not For

Recent mission logs did not show terminal `TASK_STATE` arriving before `TASK_REFUEL_*` on the vehicle logs that were inspected. So the pending map is not mainly there because that race appears common in practice.

It is also not the authority on active mission execution. Once a task is won, the active lock should have everything it needs.

## Pending-Task Pruning

Cleanup happens in two stages:

- normal cleanup is event-driven
- fallback cleanup is time-based

Normal cleanup happens immediately when the app sees a terminal task outcome or closes an active lock:

- `bidwon`
- `bidlost`
- `abstain`
- active-lock clear

That means most pending entries should disappear without waiting for the TTL path at all.

The time-based path in `prunePendingTasks()` is only a fallback for orphaned pending entries if closeout mail never arrives.

Current behavior:

- prune age is `120` seconds
- the active task hash is skipped if it still appears in the map
- only stale, non-active entries are removed

This keeps the pending map bounded while making the real lifecycle cleanup happen on task events instead of on timers.

## Iterate Flow

`Iterate()` is split into small helpers, in the same order the app reasons about the mission.

1. `updateFuelDistanceRemaining()`

Publishes `FUEL_DISTANCE_REMAINING = total_range - ODOMETRY_DIST`.

2. `maybeRelatchAfterOdometryReset()`

Re-arms threshold posting only after odometry has actually dropped low again.

3. `maybeHandleImmediateDiscoveryReplacement(have_nav, have_odom)`

Handles the queued discovery override. If the vehicle is already at or above threshold, it may post an immediate one-shot replacement. If the vehicle is still below threshold, the request is discarded rather than remembered.

4. `maybePostThresholdReplacement(have_nav, have_odom)`

Posts the standard threshold-triggered task once per reset cycle.

5. `maybeSendActiveHandoff(have_nav)`

Uses the target coordinates and requester stored directly in `m_active_lock` to decide when the active target-task winner has reached the handoff radius and should tell the requester to return.

6. `maybeMaintainActiveLock()`

Advances the active lock toward release:

- timeout
- target-task handoff completion
- basic-task return-reset completion

7. `prunePendingTasks(now)`

Drops stale non-active pending-task entries.

8. publish `REFUEL_TRANSIT_BUSY`

The final busy output is simply whether `m_active_lock` is engaged.

## Helper Breakdown

The smaller helpers exist for readability, not to change policy.

- `normalizeTaskSpec()`: converts hash-delimited and comma-delimited task strings into one parseable format
- `inferRequesterFromId()`: extracts the requester vehicle from the generated task id when the requester field is absent
- `parseTaskStateWinner()`: looks for several known winner field names in variant `TASK_STATE` payloads
- `activeLockEngaged()`: checks whether the vehicle is already committed to a replacement
- `activeTaskUsesReturnReset()`: classifies tasks that should release on return-reset rather than handoff
- `maybeHandleImmediateDiscoveryReplacement()`: handles the fire-discovery override that only posts immediate relief when the vehicle is already at or above threshold
- `prunePendingTasks()`: keeps the pre-win metadata map bounded
- `sendNodeMessage()`: convenience wrapper for building and posting `NODE_MESSAGE_LOCAL`
- `notifyRequesterReturn()`: sends the requester the control messages that complete a target handoff
- `clearActiveReplacementLock()`: clears the active lock and records why it ended

## Design Notes

The current structure tries to be both simple and behavior-safe:

- one thin pending-task map for pre-win metadata
- one self-contained active lock for post-win execution
- one place that owns win/loss transitions
- one iterate path that advances the active task toward release

This keeps the useful part of the old cache idea while removing the unnecessary dependency of active mission behavior on repeated cache lookups.

## Extensibility

This pattern should transfer well to other “one active commitment, many pending hashes” apps and missions, for example:

- payload swap or payload relay missions
- station takeover or guard-post replacement
- rescue, escort, or assist tasks
- any auctioned mission where vehicles may hear several pending task hashes before they win one

The reusable structure is:

- thin pending metadata keyed by task hash
- one self-contained active lock
- one function that owns win/loss transitions
- mission-specific completion rules

For larger fleets or faster auction rates, the current event-driven cleanup plus short TTL fallback is a good starting point, but the expiry path is the first thing to scale up. The natural next step would be:

- keep immediate event-driven removal
- keep a short TTL fallback
- replace full-map iterate scanning with an expiry queue or min-heap keyed by `expires_at`

That preserves the same model while making stale-entry cleanup proportional to actual expirations instead of total pending-task count.
