#include "LocalAuctionReservation.h"

#include <algorithm>
#include <cctype>

namespace {
std::string g_reserved_task_hash = "";
double g_last_refresh_time = 0.0;
const double g_reservation_timeout = 20.0; // seconds

void clearReservation()
{
  g_reserved_task_hash = "";
  g_last_refresh_time = 0.0;
}

void expireIfStale(double now)
{
  if(g_reserved_task_hash == "")
    return;
  if((now - g_last_refresh_time) > g_reservation_timeout)
    clearReservation();
}

std::string normalizeState(const std::string& s)
{
  std::string out = s;
  out.erase(out.begin(),
            std::find_if(out.begin(), out.end(),
                         [](unsigned char ch) { return !std::isspace(ch); }));
  out.erase(std::find_if(out.rbegin(), out.rend(),
                         [](unsigned char ch) { return !std::isspace(ch); }).base(),
            out.end());
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}
}

namespace LocalAuctionReservation {

bool heldByOther(const std::string& task_hash, double now)
{
  expireIfStale(now);
  return((g_reserved_task_hash != "") && (g_reserved_task_hash != task_hash));
}

bool claim(const std::string& task_hash, double now)
{
  if(task_hash == "")
    return(false);

  expireIfStale(now);
  if((g_reserved_task_hash == "") || (g_reserved_task_hash == task_hash)) {
    g_reserved_task_hash = task_hash;
    g_last_refresh_time = now;
    return(true);
  }
  return(false);
}

void release(const std::string& task_hash)
{
  if((task_hash != "") && (task_hash == g_reserved_task_hash))
    clearReservation();
}

void maintainForState(const std::string& task_hash,
                      const std::string& auction_state,
                      double now)
{
  const std::string state = normalizeState(auction_state);
  if((state == "bidding") || (state == "bidwon")) {
    claim(task_hash, now);
  }
  else if((state == "bidlost") || (state == "abstain")) {
    release(task_hash);
  }
  else {
    expireIfStale(now);
  }
}

} // namespace LocalAuctionReservation
