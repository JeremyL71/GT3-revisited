#pragma once

// Players' countries for the relay's log (gt3r-relay --geoip; docs/dev/ONLINE.md "Logs"): a CSV
// of "first IP,last IP,country" ranges, IPv4 and IPv6, as in DB-IP's free "IP to Country Lite"
// (https://db-ip.com, CC BY 4.0; tools/gt3r-relay/update-geoip.sh fetches it monthly).

#include "base/bytes.h"
#include "net/socket.h"

#include <string>
#include <string_view>
#include <vector>

namespace gt3r::net {

class GeoIp {
public:
    // Replaces the table. False (the table unchanged): unreadable, or not one range in it.
    bool load(const std::string& path, std::string* error = nullptr);
    bool parse(std::string_view csv, std::string* error = nullptr);
    // Two capital letters as the list has them ("ZZ": reserved); "" when no range holds `a`.
    std::string country(const Address& a) const;
    size_t ranges() const { return v4_.size() + v6_.size(); }

private:
    struct V4 {
        u32 first, last;
        char cc[2];
    };
    struct V6 { // by /64 (one source, as in the relay's per-IP limits)
        u64 first, last;
        char cc[2];
    };
    static bool add(std::string_view line, std::vector<V4>& v4, std::vector<V6>& v6);
    bool finish(std::vector<V4>& v4, std::vector<V6>& v6, size_t bad, std::string* error);
    std::vector<V4> v4_;
    std::vector<V6> v6_;
};

} // namespace gt3r::net
