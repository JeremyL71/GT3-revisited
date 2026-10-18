#pragma once

// Online play wire format (docs/dev/ONLINE.md). Little endian.
//
// Datagram: "GT3N" u16 protocol version, u16 ack (the next reliable sequence number the
// sender expects from the receiver: every lower one arrived), u8 reliable count, then
// {u16 seq, u16 length, message} each, u8 unreliable count, then {u16 length, message}.
// A message is u8 type + body. Reliable messages are delivered once, in order.

#include "base/bytes.h"
#include "sim/sim_types.h"

#include <string>
#include <vector>

namespace gt3r::net {

constexpr u16 kProtocolVersion = 3;
constexpr int kMaxPlayers = 6; // as the original's i.LINK battle
constexpr size_t kMaxDatagram = 1200;

enum class Msg : u8 {
    Hello = 1,    // C->H  u16 version, str name, str car, u8 paint, str build
    Welcome = 2,  // H->C  u8 your id
    Reject = 3,   // H->C  str reason
    Roster = 4,   // H->*  u8 n, {u8 id, str name, str car, u8 paint, u8 flags (1 ready, 2 loaded), u16 ping ms}
    Settings = 5, // H->*  str course, u8 laps
    Car = 6,      // C->H  str car, u8 paint, u8 ready
    Load = 7,     // H->*  str course, u8 laps, u8 n, {u8 id} grid order (pole first)
    Loaded = 8,   // C->H  (the race scene is ready)
    Go = 9,       // H->*  u32 delay ms (start the countdown that long after the host sent it)
    State = 10,   // *     CarSnapshot (unreliable; the host relays clients' states)
    Finish = 11,  // *     u8 id, f64 race time, f64 best lap (reliable; relayed)
    Ping = 12,    // *     f64 sender time (unreliable)
    Pong = 13,    // *     f64 echoed time (unreliable)
    Bye = 14,     // *     (unreliable, sent a few times)
    Lobby = 15,   // H->*  back to the lobby after a race
    Retire = 16,  // *     u8 id (left the race; reliable; relayed)
    CarInfo = 17, // C->H  bytes (game::OnlineCar); H->* u8 id, bytes
    PkgRequest = 18, // *  str sha256: send me that mod car package (docs/dev/ONLINE.md "Mod cars")
    PkgChunk = 19,   // *  str sha256, u32 total, u32 offset, bytes
    PkgError = 20,   // *  str sha256, str reason
    Contact = 21,    // *  u8 target, u8 from, u32 tick, f32 point x, z, f32 impulse x, z (reliable; relayed)
    Flag = 22,       // C->H u8 suspect, u8 FlagKind, f32 value (reliable); H->C u8 suspect, u8 kind, f32 value, u8 reporter
    Kick = 23,       // H->C str reason (the host removed us)
};

class Writer {
public:
    Bytes data;
    void u8_(u8 v) { data.push_back(v); }
    void u16_(u16 v) { u8_(u8(v)), u8_(u8(v >> 8)); }
    void u32_(u32 v) { u16_(u16(v)), u16_(u16(v >> 16)); }
    void f32_(float v);
    void f64_(double v);
    void str(const std::string& s); // u8 length (truncated to 255) + bytes
    void bytes(ByteSpan b) { data.insert(data.end(), b.begin(), b.end()); }
};

class Reader {
public:
    explicit Reader(ByteSpan d) : d_(d) {}
    bool ok() const { return ok_; }
    bool at_end() const { return pos_ >= d_.size(); }
    size_t remaining() const { return ok_ ? d_.size() - pos_ : 0; }
    u8 u8_();
    u16 u16_();
    u32 u32_();
    float f32_();
    double f64_();
    std::string str();
    ByteSpan bytes(size_t n);

private:
    bool need(size_t n);
    ByteSpan d_;
    size_t pos_ = 0;
    bool ok_ = true;
};

// A driver's inputs for one tick, quantised (steer -1..1 in 1/127, pedals in 1/255).
struct InputFrame {
    s8 steer = 0;
    u8 throttle = 0, brake = 0;
    u8 flags = 0; // 1 handbrake, 2 shift up, 4 shift down, 8 automatic, 16 reverse request, bits 5-6 controller kind
    bool operator==(const InputFrame&) const = default;
};
constexpr int kInputHistory = 6; // inputs of the last ticks in every state (a lost datagram costs none)

// One car's state at the end of a tick, as its owner simulated it. On the wire it is
// quantised to ~100 bytes (docs/dev/ONLINE.md "State"): position in mm, orientation as
// "smallest three", velocities in cm/s and mrad/s, wheels in a few bits each. Wheel
// contact points, loads, speed, the CoG (body_x/z) and race_time are not sent: the
// receiver derives them from the car's setup and the tick (race::complete_snapshot).
struct CarSnapshot {
    u8 player = 0;
    u32 tick = 0;          // owner's race tick (sequence; newer replaces older)
    float race_time = 0;   // owner's race clock after the tick (s; < 0 in the countdown) [derived]
    sim::VehicleState state;
    // Collision pose (sim::collide::Body: CoG x/z [derived], heading, yaw rate).
    float body_x = 0, body_z = 0, heading = 0, yaw_rate = 0;
    float brake = 0;       // lamps (the brake pedal: inputs[0])
    u8 laps = 0;           // laps completed
    bool respawned = false; // the owner put the car back on the track lately (a jump is no teleport)
    // inputs[0] = this tick's, [1] the tick before, ... (input_count valid).
    InputFrame inputs[kInputHistory];
    u8 input_count = 0;
};

// Plausibility checks a player's machine reports about another player's car (the host sees them).
enum class FlagKind : u8 {
    Speed = 1,      // faster than the car's geared top speed allows (value: km/h)
    Teleport = 2,   // moved further than its speed allows, without a respawn (value: metres)
    Accel = 3,      // sustained acceleration beyond what a car can do (value: g)
    FinishTime = 4, // claimed a finish time others did not see (value: seconds off)
    Motion = 5,     // moves unlike its own car under its own inputs (value: metres)
};
const char* flag_text(FlagKind k);

// Quantisation helpers (exposed for tests).
u32 pack_quat(const sim::Quat& q);   // smallest three, 10 bits each
sim::Quat unpack_quat(u32 v);
InputFrame pack_input(const sim::ControlInputs& in, bool reverse = false);
sim::ControlInputs unpack_input(const InputFrame& f);

void write_snapshot(Writer& w, const CarSnapshot& s);
bool read_snapshot(Reader& r, CarSnapshot& s);

} // namespace gt3r::net
