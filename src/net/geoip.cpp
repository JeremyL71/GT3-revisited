#include "net/geoip.h"

#include <algorithm>
#include <cstring>
#include <fstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

namespace gt3r::net {

namespace {

constexpr u8 kMapped[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF};

u64 be(const u8* p, int n) {
    u64 v = 0;
    for (int i = 0; i < n; ++i) v = v << 8 | p[i];
    return v;
}

// One field: surrounding spaces and quotes dropped.
std::string_view field(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '"')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '"' || s.back() == '\r')) s.remove_suffix(1);
    return s;
}

// family 4: out[0..3]; 6: out[0..15]. 0 when `s` is neither.
int parse_ip(std::string_view s, u8 out[16]) {
    char buf[64];
    if (s.empty() || s.size() >= sizeof buf) return 0;
    std::memcpy(buf, s.data(), s.size());
    buf[s.size()] = 0;
    if (s.find(':') == std::string_view::npos) return inet_pton(AF_INET, buf, out) == 1 ? 4 : 0;
    return inet_pton(AF_INET6, buf, out) == 1 ? 6 : 0;
}

// Sorted without overlaps (the same on every platform): a range overlapping the one before keeps
// only what lies past it, as when a /64 the list splits between countries goes to the first;
// neighbours of one country become one range.
template <class R> void merge(std::vector<R>& v) {
    std::sort(v.begin(), v.end(), [](const R& a, const R& b) {
        if (a.first != b.first) return a.first < b.first;
        if (a.last != b.last) return a.last < b.last;
        return std::memcmp(a.cc, b.cc, 2) < 0;
    });
    size_t n = 0;
    for (R r : v) {
        if (n) {
            R& prev = v[n - 1];
            if (r.first <= prev.last) {
                if (r.last <= prev.last) continue;
                r.first = prev.last + 1;
            }
            if (prev.last + 1 == r.first && !std::memcmp(prev.cc, r.cc, 2)) {
                prev.last = r.last;
                continue;
            }
        }
        v[n++] = r;
    }
    v.resize(n);
    v.shrink_to_fit();
}

template <class R, class K> const R* find(const std::vector<R>& v, K key) {
    auto it = std::upper_bound(v.begin(), v.end(), key, [](K k, const R& r) { return k < r.first; });
    if (it == v.begin()) return nullptr;
    --it;
    return key <= it->last ? &*it : nullptr;
}

} // namespace

// One "first IP,last IP,country" line into v4 / v6. False: unreadable (blank and # lines are fine).
bool GeoIp::add(std::string_view line, std::vector<V4>& v4, std::vector<V6>& v6) {
    if (field(line).empty() || field(line).front() == '#') return true;
    size_t c1 = line.find(','), c2 = c1 == std::string_view::npos ? c1 : line.find(',', c1 + 1);
    if (c2 == std::string_view::npos) return false;
    size_t c3 = line.find(',', c2 + 1); // any further columns are ignored
    std::string_view cc = field(line.substr(c2 + 1, c3 == std::string_view::npos ? c3 : c3 - c2 - 1));
    u8 a[16], b[16];
    int fa = parse_ip(field(line.substr(0, c1)), a), fb = parse_ip(field(line.substr(c1 + 1, c2 - c1 - 1)), b);
    if (!fa || fa != fb || cc.size() != 2 || cc[0] < 'A' || cc[0] > 'Z' || cc[1] < 'A' || cc[1] > 'Z') return false;
    if (fa == 6 && !std::memcmp(a, kMapped, 12) && !std::memcmp(b, kMapped, 12)) { // ::ffff:a.b.c.d
        std::memmove(a, a + 12, 4);
        std::memmove(b, b + 12, 4);
        fa = 4;
    }
    if (fa == 4) {
        V4 r{u32(be(a, 4)), u32(be(b, 4)), {cc[0], cc[1]}};
        if (r.first > r.last) return false;
        v4.push_back(r);
    } else {
        V6 r{be(a, 8), be(b, 8), {cc[0], cc[1]}};
        if (r.first > r.last) return false;
        v6.push_back(r);
    }
    return true;
}

bool GeoIp::finish(std::vector<V4>& v4, std::vector<V6>& v6, size_t bad, std::string* error) {
    if (v4.empty() && v6.empty()) {
        if (error) *error = "no IP ranges (" + std::to_string(bad) + " lines unreadable)";
        return false;
    }
    merge(v4);
    merge(v6);
    v4_ = std::move(v4);
    v6_ = std::move(v6);
    return true;
}

bool GeoIp::load(const std::string& path, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = path + " not found";
        return false;
    }
    std::vector<V4> v4;
    std::vector<V6> v6;
    size_t bad = 0;
    for (std::string line; std::getline(f, line);) bad += !add(line, v4, v6); // line by line: the list is ~30 MB
    std::string err;
    if (!finish(v4, v6, bad, &err)) {
        if (error) *error = path + ": " + err;
        return false;
    }
    return true;
}

bool GeoIp::parse(std::string_view csv, std::string* error) {
    std::vector<V4> v4;
    std::vector<V6> v6;
    size_t bad = 0;
    while (!csv.empty()) {
        size_t eol = csv.find('\n');
        bad += !add(csv.substr(0, eol), v4, v6);
        csv.remove_prefix(eol == std::string_view::npos ? csv.size() : eol + 1);
    }
    return finish(v4, v6, bad, error);
}

std::string GeoIp::country(const Address& a) const {
    if (!a.valid()) return "";
    u8 ip[16];
    a.ip16(ip);
    if (!std::memcmp(ip, kMapped, 12)) {
        const V4* r = find(v4_, u32(be(ip + 12, 4)));
        return r ? std::string(r->cc, 2) : "";
    }
    const V6* r = find(v6_, be(ip, 8));
    return r ? std::string(r->cc, 2) : "";
}

} // namespace gt3r::net
