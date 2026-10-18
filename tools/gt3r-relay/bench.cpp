// gt3r-relay-bench: a load test for gt3r-relay (docs/dev/ONLINE.md "Benchmark").
//
// Simulates racing lobbies against a relay with the real protocol. A lobby is six players,
// one UDP socket each: the host registers (cookie challenge and all), five players look the
// code up, every pair binds its relay channel (5 join channels + 10 player pairs = 15 links,
// a fully relayed six-player race), and the host reserves the links. Then every player sends
// one car-state-sized datagram on each of its channels at 60 Hz, as the game does, with a
// sequence number and a timestamp inside, so the receiving end measures loss and the time
// the relay added (loopback: almost all of it is the relay and the kernel).
//
//   gt3r-relay-bench [--server 127.0.0.1:47631] [--links 300 | --ramp 50,100,200,300,400,600]
//                    [--duration 10] [--pid <relay pid>] [--size 126] [--hz 60]
//
// The relay must run with --bench (loopback skips its per-IP limits) and enough slots
// (--relay-links 10000). With --pid, the relay's CPU time during each step is read (Linux:
// /proc/<pid>/stat; macOS: proc_pid_rusage) and turned into CPU seconds per million relayed
// datagrams and the datagram rate / link count one core would carry at 70 %.
//
// Single threaded: poll() over every socket; on Linux, sendmmsg / recvmmsg batch the syscalls.
// No SDL, no game code: builds with tools/gt3r-relay/CMakeLists.txt. POSIX only.

#include "net/rendezvous.h"
#include "net/socket.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <libproc.h>
#include <mach/mach_time.h>
#endif

using namespace gt3r;

namespace {

constexpr size_t kPayloadMin = net::kRelayHeader + 4 + 4 + 1 + 4 + 8; // header, "GT3N", link, dir, seq, time
constexpr double kHeaderBytes = 28;                                    // IPv4 + UDP, for wire rates

struct Player {
    int fd = -1;
    int lobby = 0;
    u64 cookie = 0;
    std::vector<std::pair<int, u8>> out; // (link, direction) it sends on
    double next_due = 0;
    u32 seq = 0;
};

struct Link {
    u64 key = 0;
    int a = 0, b = 0; // players; direction 0 is a -> b
};

struct Lobby {
    std::string code;
    u64 token = 0;
    int host = 0;      // player index
    int links = 0;
    bool granted = false;
    int position = 0;  // in the relay's queue
    double keepalive = 0;
};

struct Window { // one ramp step's measurement
    double from = 1e300, to = 1e300;
    u64 sent = 0, recv = 0;
    double sent_bytes = 0, recv_bytes = 0;
    double late_sum = 0, late_max = 0;
    u64 late_n = 0;
    std::vector<u64> hist = std::vector<u64>(10001, 0); // 10 us buckets up to 100 ms, then overflow
    bool counting(double t) const { return t >= from && t < to; }
    double percentile(double p) const {
        u64 total = 0;
        for (u64 h : hist) total += h;
        if (!total) return -1;
        u64 want = u64(std::ceil(p * double(total))), acc = 0;
        for (size_t i = 0; i < hist.size(); ++i)
            if ((acc += hist[i]) >= want) return double(i) * 0.01; // ms
        return 100;
    }
};

std::vector<Player> g_players;
std::vector<Link> g_links;
std::vector<Lobby> g_lobbies;
net::Address g_server;
size_t g_size = 126;
double g_hz = 60;
Window g_win;

double cpu_of(int pid) { // seconds of CPU time, < 0: unknown
    if (pid <= 0) return -1;
#if defined(__linux__)
    char path[64];
    std::snprintf(path, sizeof path, "/proc/%d/stat", pid);
    FILE* f = std::fopen(path, "r");
    if (!f) return -1;
    char buf[1024];
    size_t n = std::fread(buf, 1, sizeof buf - 1, f);
    std::fclose(f);
    buf[n] = 0;
    const char* p = std::strrchr(buf, ')'); // the name may hold spaces
    if (!p) return -1;
    unsigned long long utime = 0, stime = 0;
    // After ")": state ppid pgrp session tty tpgid flags minflt cminflt majflt cmajflt utime stime
    if (std::sscanf(p + 2, "%*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu", &utime, &stime) != 2) return -1;
    return double(utime + stime) / double(sysconf(_SC_CLK_TCK));
#elif defined(__APPLE__)
    rusage_info_v2 ri{};
    if (proc_pid_rusage(pid, RUSAGE_INFO_V2, reinterpret_cast<rusage_info_t*>(&ri)) != 0) return -1;
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb); // Apple silicon reports mach ticks, not ns
    return double(ri.ri_user_time + ri.ri_system_time) * tb.numer / tb.denom / 1e9;
#else
    return -1;
#endif
}

double self_cpu() {
    rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return double(ru.ru_utime.tv_sec + ru.ru_stime.tv_sec) + double(ru.ru_utime.tv_usec + ru.ru_stime.tv_usec) / 1e6;
}

int open_socket() {
    int fd = ::socket(g_server.family(), SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int buf = 1 << 20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buf, sizeof buf);
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buf, sizeof buf);
    // Connected: the relay is the only peer (send / recv, and the kernel filters the rest).
    if (::connect(fd, reinterpret_cast<const sockaddr*>(g_server.storage), socklen_t(g_server.length)) != 0) {
        ::close(fd);
        return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    return fd;
}

void handle_data(const u8* p, size_t n, double now);
void handle_control(int player, const u8* p, size_t n);

// Everything waiting on one socket (control replies are returned when `want` matches).
bool drain(int player, net::Rv want, Bytes* reply, double now) {
    bool got = false;
    u8 buf[2048];
    for (;;) {
        ssize_t n = ::recv(g_players[size_t(player)].fd, buf, sizeof buf, 0);
        if (n <= 0) break;
        if (size_t(n) >= 4 && std::memcmp(buf, net::kRelayMagic, 4) == 0) {
            handle_data(buf, size_t(n), now);
            continue;
        }
        net::Rv type;
        net::Reader r(ByteSpan(buf, size_t(n)));
        if (!net::rv_parse(ByteSpan(buf, size_t(n)), type, r)) continue;
        if (reply && !got && (type == want || type == net::Rv::Error || type == net::Rv::Challenge)) {
            reply->assign(buf, buf + n);
            got = true;
            continue;
        }
        handle_control(player, buf, size_t(n));
    }
    return got;
}

// A request with the cookie handshake; the reply of type `want` (false: an Error, or no answer).
bool request(int player, net::Rv type, const std::function<void(net::Writer&)>& fill, net::Rv want, Bytes& reply,
             std::string* why = nullptr) {
    Player& p = g_players[size_t(player)];
    for (int attempt = 0; attempt < 6; ++attempt) {
        net::Writer w = net::rv_request(type, p.cookie);
        fill(w);
        net::rv_pad(w);
        ::send(p.fd, w.data.data(), w.data.size(), 0);
        double until = net::now_ms() + 300;
        bool got = false;
        while (!got && net::now_ms() < until) {
            pollfd pf{p.fd, POLLIN, 0};
            ::poll(&pf, 1, 20);
            got = drain(player, want, &reply, net::now_ms());
        }
        if (!got) continue; // lost (or rate limited): again
        net::Rv t;
        net::Reader r(reply);
        net::rv_parse(reply, t, r);
        if (t == net::Rv::Challenge) {
            r.u8_();
            p.cookie = net::get_u64(r);
            continue;
        }
        if (t == net::Rv::Error) {
            r.u8_();
            if (why) *why = r.str();
            return false;
        }
        return true;
    }
    if (why) *why = "no answer";
    return false;
}

void handle_control(int player, const u8* p, size_t n) { // Reservation news for a lobby's host
    net::Rv type;
    net::Reader r(ByteSpan(p, n));
    if (!net::rv_parse(ByteSpan(p, n), type, r) || type != net::Rv::Reservation) return;
    Lobby& l = g_lobbies[size_t(g_players[size_t(player)].lobby)];
    if (l.host != player) return;
    net::RvSlot st = net::RvSlot(r.u8_());
    r.u8_();
    u16 pos = r.u16_();
    if (!r.ok()) return;
    if (st == net::RvSlot::Granted && !l.granted) {
        l.granted = true;
        double now = net::now_ms();
        for (size_t e = 0; e < g_players.size(); ++e) { // its players start racing, each at its own phase
            Player& p = g_players[e];
            if (p.lobby == g_players[size_t(player)].lobby && p.next_due == 0 && !p.out.empty())
                p.next_due = now + std::fmod(double(e) * 7.3, 1000 / g_hz);
        }
    }
    if (st == net::RvSlot::Queued) l.position = pos;
}

void handle_data(const u8* p, size_t n, double now) {
    if (n < kPayloadMin) return;
    double sent;
    std::memcpy(&sent, p + kPayloadMin - 8, 8);
    if (!g_win.counting(sent)) return;
    ++g_win.recv;
    g_win.recv_bytes += double(n) + kHeaderBytes;
    double ms = now - sent;
    size_t b = ms < 0 ? 0 : std::min<size_t>(size_t(ms * 100), g_win.hist.size() - 1);
    ++g_win.hist[b];
}

// One lobby of `links` relayed links (1..15). False: the relay refused something (printed).
bool add_lobby(int links) {
    int players = 2;
    while (players * (players - 1) / 2 < links) ++players;
    Lobby l;
    l.links = links;
    l.host = int(g_players.size());
    l.token = (u64(std::rand()) << 32 | u64(std::rand())) | 1;
    int index = int(g_lobbies.size());
    for (int i = 0; i < players; ++i) {
        Player p;
        p.fd = open_socket();
        p.lobby = index;
        if (p.fd < 0) {
            std::fprintf(stderr, "bench: cannot open a socket (%s)\n", std::strerror(errno));
            return false;
        }
        g_players.push_back(p);
    }
    g_lobbies.push_back(l);
    Lobby& lob = g_lobbies.back();
    Bytes reply;
    std::string why;
    if (!request(lob.host, net::Rv::Register, [&](net::Writer& w) {
            net::put_u64(w, lob.token);
            net::put_endpoints(w, {});
        }, net::Rv::Registered, reply, &why)) {
        std::fprintf(stderr, "bench: register: %s (is the relay running with --bench?)\n", why.c_str());
        return false;
    }
    {
        net::Rv t;
        net::Reader r(reply);
        net::rv_parse(reply, t, r);
        lob.code = r.str();
    }
    std::vector<u64> nonce(size_t(players), 0);
    for (int i = 1; i < players; ++i) {
        if (!request(lob.host + i, net::Rv::Lookup, [&](net::Writer& w) {
                w.str(lob.code);
                w.u16_(net::kProtocolVersion);
                net::put_endpoints(w, {});
            }, net::Rv::HostInfo, reply, &why)) {
            std::fprintf(stderr, "bench: lookup: %s\n", why.c_str());
            return false;
        }
        net::Rv t;
        net::Reader r(reply);
        net::rv_parse(reply, t, r);
        nonce[size_t(i)] = net::get_u64(r);
    }
    drain(lob.host, net::Rv::Incoming, nullptr, net::now_ms()); // the host's Incoming notices
    int made = 0;
    for (int a = 0; a < players && made < links; ++a)
        for (int b = a + 1; b < players && made < links; ++b, ++made) {
            Link k;
            k.a = lob.host + a, k.b = lob.host + b;
            bool join = a == 0; // the host and a player: the join's nonce channel
            k.key = join ? nonce[size_t(b)] : net::pair_key(lob.token, u8(a), u8(b));
            for (int e : {k.a, k.b})
                if (!request(e, net::Rv::Bind, [&](net::Writer& w) {
                        net::put_u64(w, k.key);
                        w.str(lob.code);
                        net::put_u64(w, join ? 0 : lob.token);
                        w.u8_(join ? 0 : u8(a)), w.u8_(join ? 0 : u8(b));
                    }, net::Rv::Bound, reply, &why)) {
                    std::fprintf(stderr, "bench: bind: %s\n", why.c_str());
                    return false;
                }
            g_players[size_t(k.a)].out.push_back({int(g_links.size()), 0});
            g_players[size_t(k.b)].out.push_back({int(g_links.size()), 1});
            g_links.push_back(k);
        }
    if (!request(lob.host, net::Rv::Reserve, [&](net::Writer& w) {
            w.str(lob.code);
            net::put_u64(w, lob.token);
            w.u8_(u8(links));
        }, net::Rv::Reservation, reply, &why)) {
        std::fprintf(stderr, "bench: reserve: %s\n", why.c_str());
        return false;
    }
    lob.keepalive = net::now_ms();
    handle_control(lob.host, reply.data(), reply.size()); // granted (players start) or queued
    return true;
}

void send_tick(Player& p, double now) {
    u8 bufs[16][1500];
    size_t n = 0;
    for (const auto& [link, dir] : p.out) {
        if (n == 16) break;
        u8* b = bufs[n++];
        std::memset(b, 0, g_size);
        std::memcpy(b, net::kRelayMagic, 4);
        u64 key = g_links[size_t(link)].key;
        for (int i = 0; i < 8; ++i) b[4 + i] = u8(key >> (8 * i));
        std::memcpy(b + 12, "GT3N", 4);
        u32 l = u32(link), seq = p.seq;
        std::memcpy(b + 16, &l, 4);
        b[20] = dir;
        std::memcpy(b + 21, &seq, 4);
        std::memcpy(b + 25, &now, 8);
    }
    ++p.seq;
#if defined(__linux__)
    mmsghdr msgs[16]{};
    iovec iov[16];
    for (size_t i = 0; i < n; ++i) {
        iov[i] = {bufs[i], g_size};
        msgs[i].msg_hdr.msg_iov = &iov[i];
        msgs[i].msg_hdr.msg_iovlen = 1;
    }
    int sent = ::sendmmsg(p.fd, msgs, unsigned(n), 0);
    size_t ok = sent > 0 ? size_t(sent) : 0;
#else
    size_t ok = 0;
    for (size_t i = 0; i < n; ++i) ok += ::send(p.fd, bufs[i], g_size, 0) == ssize_t(g_size) ? 1 : 0;
#endif
    if (g_win.counting(now)) {
        g_win.sent += ok;
        g_win.sent_bytes += double(ok) * (double(g_size) + kHeaderBytes);
    }
}

void receive_ready(const std::vector<pollfd>& fds, const std::vector<int>& who, double now) {
    for (size_t i = 0; i < fds.size(); ++i) {
        if (!(fds[i].revents & POLLIN)) continue;
#if defined(__linux__)
        static u8 bufs[64][2048];
        mmsghdr msgs[64]{};
        iovec iov[64];
        for (int k = 0; k < 64; ++k) {
            iov[k] = {bufs[k], sizeof bufs[k]};
            msgs[k].msg_hdr.msg_iov = &iov[k];
            msgs[k].msg_hdr.msg_iovlen = 1;
        }
        for (;;) {
            int got = ::recvmmsg(fds[i].fd, msgs, 64, MSG_DONTWAIT, nullptr);
            if (got <= 0) break;
            for (int k = 0; k < got; ++k) {
                const u8* b = bufs[k];
                size_t n = msgs[k].msg_len;
                if (n >= 4 && std::memcmp(b, net::kRelayMagic, 4) == 0) handle_data(b, n, now);
                else handle_control(who[i], b, n);
            }
            if (got < 64) break;
        }
#else
        u8 buf[2048];
        for (;;) {
            ssize_t n = ::recv(fds[i].fd, buf, sizeof buf, 0);
            if (n <= 0) break;
            if (n >= 4 && std::memcmp(buf, net::kRelayMagic, 4) == 0) handle_data(buf, size_t(n), now);
            else handle_control(who[i], buf, size_t(n));
        }
#endif
    }
}

// Runs the traffic (and keepalives) until `until`.
void run(double until) {
    std::vector<pollfd> fds;
    std::vector<int> who;
    for (size_t i = 0; i < g_players.size(); ++i) {
        fds.push_back({g_players[i].fd, POLLIN, 0});
        who.push_back(int(i));
    }
    const double period = 1000 / g_hz;
    for (double now = net::now_ms(); now < until; now = net::now_ms()) {
        double next = until;
        for (Player& p : g_players) {
            if (p.next_due == 0) continue; // its lobby waits for slots
            if (now >= p.next_due) {
                double late = now - p.next_due;
                if (g_win.counting(now)) {
                    g_win.late_sum += late, g_win.late_max = std::max(g_win.late_max, late);
                    ++g_win.late_n;
                }
                send_tick(p, now);
                p.next_due += period;
                if (p.next_due < now) p.next_due = now + period; // far behind: skip ticks, as a stalled game would not
            }
            next = std::min(next, p.next_due);
        }
        for (Lobby& l : g_lobbies) // keepalives (a queued lobby keeps its place with them)
            if (now - l.keepalive > (l.granted ? 5000 : 2000)) {
                l.keepalive = now;
                Player& h = g_players[size_t(l.host)];
                net::Writer w = net::rv_request(net::Rv::Keepalive, h.cookie);
                w.str(l.code);
                net::put_u64(w, l.token);
                net::rv_pad(w);
                ::send(h.fd, w.data.data(), w.data.size(), 0);
            }
        int wait = int(std::clamp(std::ceil(next - net::now_ms()), 0.0, 5.0));
        if (::poll(fds.data(), nfds_t(fds.size()), wait) > 0) receive_ready(fds, who, net::now_ms());
    }
}

std::vector<int> parse_list(const char* s) {
    std::vector<int> v;
    while (*s) {
        v.push_back(std::atoi(s));
        const char* c = std::strchr(s, ',');
        if (!c) break;
        s = c + 1;
    }
    return v;
}

void usage() {
    std::fprintf(stderr,
                 "usage: gt3r-relay-bench [--server 127.0.0.1:47631] [--links 300 | --ramp 50,100,200,300,400,600]\n"
                 "                        [--duration 10] [--pid <relay pid>] [--size 126] [--hz 60]\n"
                 "Run the relay with --bench --relay-links 10000 on the same machine first.\n");
}

} // namespace

int main(int argc, char** argv) {
    std::string server = "127.0.0.1:" + std::to_string(net::kDefaultRelayPort);
    std::vector<int> steps{300};
    double duration = 10;
    int pid = 0;
    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* name) { return !std::strcmp(argv[i], name) && i + 1 < argc; };
        if (arg("--server")) server = argv[++i];
        else if (arg("--links")) steps = {std::atoi(argv[++i])};
        else if (arg("--ramp")) steps = parse_list(argv[++i]);
        else if (arg("--duration")) duration = std::atof(argv[++i]);
        else if (arg("--pid")) pid = std::atoi(argv[++i]);
        else if (arg("--size")) g_size = size_t(std::atoi(argv[++i]));
        else if (arg("--hz")) g_hz = std::atof(argv[++i]);
        else {
            usage();
            return 2;
        }
    }
    std::sort(steps.begin(), steps.end());
    if (steps.empty() || steps.front() <= 0 || duration <= 0 || g_hz <= 0 || g_size < kPayloadMin || g_size > 1400) {
        usage();
        return 2;
    }
    std::string err;
    if (!net::resolve(server, net::kDefaultRelayPort, g_server, &err)) {
        std::fprintf(stderr, "bench: %s\n", err.c_str());
        return 1;
    }
    // One socket per simulated player: raise the descriptor limit as far as allowed.
    rlimit rl{};
    if (getrlimit(RLIMIT_NOFILE, &rl) == 0) {
        rlim_t want = std::min<rlim_t>(rl.rlim_max, 65536);
#if defined(__APPLE__)
        want = std::min<rlim_t>(want, OPEN_MAX);
#endif
        if (rl.rlim_cur < want) rl.rlim_cur = want, setrlimit(RLIMIT_NOFILE, &rl);
    }
    u8 ip[16];
    g_server.ip16(ip);
    bool local = (ip[10] == 0xFF && ip[11] == 0xFF && ip[12] == 127) || (ip[15] == 1 && !ip[0] && !ip[14]);
    std::srand(unsigned(net::now_ms()));
    std::printf("gt3r-relay-bench: %s, %zu-byte datagrams at %.0f Hz per link and direction, %.0f s per step%s\n",
                g_server.to_string().c_str(), g_size, g_hz, duration, pid ? "" : " (no --pid: no relay CPU figures)");
    if (local)
        std::printf("note: the generator and the relay share this machine's CPU; on a 1 vCPU box both compete for it\n"
                    "      (see 'bench CPU'), so the relay's own figures are the ones to extrapolate from.\n");
    std::printf("Mbit/s include 28 bytes of IP/UDP header per datagram. Latency: send -> relay -> receive.\n\n");
    std::printf("%6s %6s %5s | %8s %8s | %7s %7s | %6s | %6s %6s | %5s %9s %9s %7s | %5s\n", "links", "racing", "queue",
                "sent/s", "recv/s", "Mb in", "Mb out", "loss%", "p50ms", "p99ms", "relay", "CPUs/Mpkt", "pkt/s@70%",
                "links", "bench");
    int total = 0;
    for (int target : steps) {
        while (total < target) { // more lobbies, up to 15 links each
            int n = std::min(15, target - total);
            if (!add_lobby(n)) return 1;
            total += n;
        }
        run(net::now_ms() + 1500); // warm up (newly granted lobbies start, queues settle)
        g_win = Window{};
        double c0 = cpu_of(pid), s0 = self_cpu();
        g_win.from = net::now_ms();
        g_win.to = 1e300;
        run(g_win.from + duration * 1000);
        g_win.to = net::now_ms();
        double c1 = cpu_of(pid), s1 = self_cpu();
        run(g_win.to + 300); // late arrivals of the window's datagrams
        double secs = (g_win.to - g_win.from) / 1000;
        int racing = 0, queued = 0;
        for (const Lobby& l : g_lobbies) (l.granted ? racing : queued) += l.granted ? l.links : 1;
        double loss = g_win.sent ? 100.0 * (1.0 - double(g_win.recv) / double(g_win.sent)) : 0;
        char relay[16] = "-", per_m[16] = "-", pps70[16] = "-", links70[16] = "-";
        if (c0 >= 0 && c1 >= 0 && g_win.sent) {
            double cpu = c1 - c0;
            double per_pkt = cpu / double(g_win.sent); // one relayed datagram: received and forwarded
            std::snprintf(relay, sizeof relay, "%.0f%%", 100 * cpu / secs);
            std::snprintf(per_m, sizeof per_m, "%.2f", per_pkt * 1e6);
            if (per_pkt > 0) {
                std::snprintf(pps70, sizeof pps70, "%.0f", 0.7 / per_pkt);
                std::snprintf(links70, sizeof links70, "%.0f", 0.7 / per_pkt / (2 * g_hz));
            }
        }
        auto ms = [](double v) {
            char b[16];
            if (v < 0) std::snprintf(b, sizeof b, "-");
            else if (v >= 100) std::snprintf(b, sizeof b, ">100");
            else std::snprintf(b, sizeof b, "%.3f", v);
            return std::string(b);
        };
        std::printf("%6d %6d %5d | %8.0f %8.0f | %7.2f %7.2f | %6.2f | %6s %6s | %5s %9s %9s %7s | %4.0f%%\n", total,
                    racing, queued, double(g_win.sent) / secs, double(g_win.recv) / secs, g_win.sent_bytes * 8 / 1e6 / secs,
                    g_win.recv_bytes * 8 / 1e6 / secs, loss, ms(g_win.percentile(0.5)).c_str(),
                    ms(g_win.percentile(0.99)).c_str(), relay, per_m, pps70, links70, 100 * (s1 - s0) / secs);
        if (loss > 1)
            std::printf("       (saturated: datagrams were lost before the relay read them; the extrapolation above "
                        "only holds for unsaturated steps)\n");
        if (g_win.late_n && g_win.late_sum / double(g_win.late_n) > 2)
            std::printf("       (the generator ran %.1f ms late on average, %.0f ms at worst: it is a bottleneck here)\n",
                        g_win.late_sum / double(g_win.late_n), g_win.late_max);
        std::fflush(stdout);
    }
    // Leave nothing behind: the sessions (and their slots) go at once.
    for (const Lobby& l : g_lobbies) {
        net::Writer w = net::rv_request(net::Rv::Unregister, 0);
        w.str(l.code);
        net::put_u64(w, l.token);
        net::rv_pad(w);
        ::send(g_players[size_t(l.host)].fd, w.data.data(), w.data.size(), 0);
    }
    for (const Player& p : g_players) ::close(p.fd);
    std::printf("\nlinks: relayed player pairs (a full six-player lobby is 15); racing: links with granted slots;\n"
                "queue: lobbies waiting for slots. sent/s, recv/s: datagrams into / out of the relay (Mb: Mbit/s).\n"
                "relay: its CPU during the step (one core = 100%%); CPUs/Mpkt: its CPU seconds per million relayed\n"
                "datagrams; pkt/s@70%% and links: what 70%% of one such core would relay. bench: this generator's CPU.\n");
    return 0;
}
