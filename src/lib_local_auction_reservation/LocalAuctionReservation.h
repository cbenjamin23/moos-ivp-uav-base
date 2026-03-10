#ifndef LOCAL_AUCTION_RESERVATION_HEADER
#define LOCAL_AUCTION_RESERVATION_HEADER

#include <string>

namespace LocalAuctionReservation {

// Returns true if another task hash currently holds the reservation.
bool heldByOther(const std::string& task_hash, double now);

// Attempts to claim (or refresh) reservation ownership for task_hash.
// Returns true if reservation is now held by task_hash.
bool claim(const std::string& task_hash, double now);

// Releases reservation if task_hash is the current owner.
void release(const std::string& task_hash);

// Convenience lifecycle helper:
// - claim on bidding/bidwon
// - release on bidlost/abstain
void maintainForState(const std::string& task_hash,
                      const std::string& auction_state,
                      double now);

} // namespace LocalAuctionReservation

#endif
