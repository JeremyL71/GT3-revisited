#pragma once

// Non-blocking UDP sockets (BSD sockets / Winsock) for online play.
//
//   net::UdpSocket s;
//   s.open(47630);                         // host: bind the port (0 = any free port)
//   net::Address a; net::resolve("10.0.0.20", 47630, a);
//   s.send_to(a, bytes);
//   while (auto n = s.recv_from(from, buf)) ...  // returns 0 when nothing is waiting
//
// A host socket is dual-stack IPv6 (IPv4 peers arrive as ::ffff:a.b.c.d) when the
// platform allows it, IPv4 otherwise; a client socket takes the family of the
// address it talks to. Addresses compare by family, address and port.

#include "base/bytes.h"

#include <string>
#include <vector>

namespace gt3r::net {

constexpr u16 kDefaultPort = 47630;

struct Address {
    u8 storage[28] = {}; // sockaddr_in / sockaddr_in6
    u32 length = 0;      // 0 = unset
    bool valid() const { return length != 0; }
    int family() const;
    u16 port() const;
    std::string to_string() const; // "1.2.3.4:5" / "[::1]:5"
    // The IP as 16 bytes (IPv4 as ::ffff:a.b.c.d, so both forms of one peer match).
    void ip16(u8 out[16]) const;
    bool operator==(const Address& o) const;
    bool operator!=(const Address& o) const { return !(*this == o); }
};

// "host", "host:port", "[v6]:port", "1.2.3.4". Blocking DNS lookup.
bool resolve(const std::string& text, u16 default_port, Address& out, std::string* error = nullptr);

class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket() { close(); }
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Host: bind `port` on every interface (dual-stack when possible).
    bool open(u16 port, std::string* error = nullptr);
    // Client: an ephemeral port in `peer`'s family.
    bool open_for(const Address& peer, std::string* error = nullptr);
    void close();
    bool is_open() const { return fd_ != kInvalid; }
    u16 local_port() const;
    // Asks for larger kernel send / receive buffers (best effort; the OS caps it, e.g. Linux at
    // net.core.rmem_max). Returns the receive buffer the OS reports.
    int set_buffers(int bytes);

    bool send_to(const Address& to, ByteSpan data);
    // Bytes received (0 = nothing waiting; errors such as ICMP port-unreachable are skipped).
    size_t recv_from(Address& from, u8* buf, size_t cap);
    // Blocks up to `timeout_ms` until a datagram is waiting (false: timed out / closed).
    bool wait_readable(int timeout_ms) const;

private:
#if defined(_WIN32)
    using Fd = unsigned long long;
    static constexpr Fd kInvalid = ~0ull;
#else
    using Fd = int;
    static constexpr Fd kInvalid = -1;
#endif
    bool setup(int family, u16 port, std::string* error);
    Fd fd_ = kInvalid;
};

// Local IPv4 addresses of this machine (for "tell your friend to join ..."); best effort.
std::vector<std::string> local_addresses();

// Monotonic milliseconds.
double now_ms();
// Tests: now_ms() returns `clock()` instead, process-wide (sessions, their receive threads, the
// relay server), so their timers run on a simulated clock the test advances. nullptr: the real one.
void set_clock(double (*clock)());

} // namespace gt3r::net
