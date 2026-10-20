# Online battle (i.LINK Battle over IP)

The original's arcade **i.LINK Battle** linked up to six PS2s with FireWire
cables, one car per console. This reimplementation does not run the original
program, so there is no i.LINK traffic to tunnel. The mode is rebuilt on our own
netcode over UDP instead. Choosing i.LINK Battle in the arcade menu opens the
online lobby (`src/app/screen_online.cpp`; also `gt3r --screen online`).

## Model: every player owns their car

- Each player simulates **only their own car** with the full physics, at the
  usual 60 Hz fixed tick, and sends that car's state after every tick.
- The other players' cars are opponents in the `RaceField` (no AI). They are
  **kinematic** cars in `sim::World` (`World::set_kinematic`,
  `Vehicle::set_remote_pose`). They are never stepped. They are posed before
  each step, and a car–car contact is solved with both bodies as usual, but
  only the local car keeps the result. The remote player's machine solves the
  same contact for its own car, so both sides feel the hit.
- `race::RemoteCar` predicts a remote car "now". It dead-reckons the newest
  snapshot along its velocity and spin, capped at 0.25 s. When a new snapshot
  lands, the jump between the shown car and the new prediction is blended out
  over about 0.1 s, so cars glide instead of popping. Collisions use the
  unblended prediction.

Why not lockstep or rollback with only inputs exchanged? The physics is
deterministic only on the same build (float results differ between compilers
and CPUs: desktop x86, Apple and Android ARM), so a lockstep game would desync
across platforms. Lockstep also adds input delay equal to the round trip, and
rollback would have to snapshot and re-simulate every car several ticks per
frame. Owner authority tolerates different builds and platforms and packet
loss, and it scales to six cars. The cost is that contacts are approximate
(each side sees the other car up to one-way latency late) and nothing stops a
modified client from lying about its car.

## State (protocol 2)

A car's state is 98 bytes on the wire, including six ticks of input history.
A race datagram carrying it is 114 bytes (126 through the relay, plus 28 bytes
of IP/UDP header), sent 60 times a second: about 61 kbit/s per stream, 65 kbit/s
at the busiest second (measured in `net_relay_traffic_per_link`, with pings and
contacts).

| field | encoding |
|---|---|
| tick | u32 |
| position | 3 × 24-bit, mm (±8.4 km) |
| orientation | "smallest three": 2-bit index + 3 × 10 bits (worst 0.22°) |
| velocity / angular velocity | 3 × s16 each, cm/s / mrad/s |
| heading, yaw rate | u16 (2π/65536), s16 mrad/s |
| rpm, gear, rev limiter, laps, throttle | u16, s8, u8, u8, u8 |
| wheels | steer s8 (5 mrad), spin speed s16 (0.1 rad/s), suspension s8 (2 mm), slip ratio / angle s8, surface u16, grounded bits |
| inputs | count + up to 6 × {steer s8, throttle u8, brake u8, flags u8}, newest first |

Not sent; the receiver rebuilds them (`race::complete_snapshot`) from the car's
own setup:
- the race clock (tick / 60 − countdown);
- the CoG for collisions (position + orientation · cog_offset);
- wheel contact points (wheel centre from the suspension travel, minus the
  radius);
- loads (grounded wheels share the weight);
- speed;
- wheel spin angles (integrated from the spin speed).

The input history feeds the physics-based prediction ("Feel and fairness"
below). It also means a lost datagram loses no input.

## Session (`src/net/`)

- `socket.*`: non-blocking UDP (BSD sockets / Winsock). The host socket is
  dual-stack IPv6. Name resolution goes through `getaddrinfo`, preferring IPv4.
- `protocol.*`: the datagram format and messages (`Msg`). Every datagram is
  `"GT3N"`, protocol version, cumulative ack, then reliable messages
  (sequence numbered, resent until acked, delivered once in order) and
  unreliable ones (states, pings).
- `session.*`:
  - **Lobby (star).** The host is player 0; up to five clients join.
    Hello → Welcome (with a random session token) / Reject (full, racing,
    other protocol), Roster, Settings, Car (each player's car, colour,
    ready). The host's Load carries course, laps and grid. Each race screen
    sends Loaded once its scene is built. Go carries the countdown start in
    the host's clock.
  - **Settings:** course, laps, *mod cars allowed* (default on), *ghost
    above* a round trip (default 150 ms; 0 = always contact).
  - **Race traffic (mesh).** Each Hello lists the client's LAN endpoints. The
    roster gives every player's endpoints: the address the host sees, then
    the LAN ones. Clients send "punch" datagrams (`"GT3P"` + session token) to
    every candidate of every other client at once, which opens both NAT
    mappings (UDP hole punching). Once both directions are heard, the pair
    has a direct link and reports it to the host. The host relays states
    only for pairs without one. A pair that fails within 6 s stays relayed
    and retries every 30 s. Lobby, finish and retire messages always go
    through the host.
  - **Receive thread.** Datagrams are read and timestamped on arrival, not
    when the game loop next polls. Two 60 Hz loops otherwise bias every
    measurement by up to a frame.
  - **Clock sync.** Ping/Pong carry four timestamps (NTP). The round trip
    excludes the other side's hold time. The offset comes from the
    minimum-round-trip sample of the last 16, taken every 250 ms. Over
    loopback with a 5 s skew: offset error 0.01 ms, start instants 0.01 ms
    apart. The race screen re-reads the start time every frame while the
    estimate improves.
- Peers time out after 8 s of silence. Bye is sent on close.
- `net::online()` is the session shared by the lobby and the race screen.

## Cars

Each player's car travels as a `game::OnlineCar` descriptor (`src/game/net_car.*`;
`CarInfo` messages, relayed by the host, also to players who join later):

- **Disc car:** its database label.
- **Garage car (GT Mode):** the base car, its 27 fitted part ids, the user
  settings, the engine condition and the wheel (~480 bytes). Every machine
  rebuilds the exact spec from its own disc (`game::online_car_spec`, the same
  path as GT Mode's `car_spec`). The lobby lists the GT Mode garage first. A
  garage car keeps its own paint. Remote cars use their real spec, since
  weight and size count in contact.
- **Mod car:** its label plus the SHA-256 and size of its package (below).

## Pickers (`src/app/online_pickers.cpp`)

Cross on the lobby's Car row opens the car page, on the Course row (host) the course
page; Left / Right on the rows still step through the lists. Both follow GT3's arcade
pickers with the disc's own art, loaded from the menu sets:

- Makers: GT Mode's dealers (`cd/cd` -> `cd/cd_<region>` in gt_mode.mbl; each `dealer<n>`
  item links `cd/cd_<region>_<maker>`, its layer holds the maker's `logo<n>`, the region
  page its `flag`), in the dealers' region order; the garage first. Makers without a
  dealer use the tune shop's logo of the same name (ts/ts), else their name. Groups are
  keyed by the maker name without spaces (the database has "CHEVROLET" two ways).
- Car page: the `<car>_mini` strip of `ar/select_car` (250 icons; mod cars in arcade mode
  get theirs from the mod patches), the car turning under `menu/showroom_single.fbi`'s
  lights (render/showroom.h, shared with the GT-mode menus) with the page's reflection
  map and a floor reflection, loaded once the choice rests a few frames; the paint column
  with each paint's colour and carcolor.sdb name; specs; the lobby's rule warning.
- Course page: the course's still (stream/course.ipic) behind, `<course>_logo` and the
  `<course>` map of `ar/select_course`, the `<course>_mini` strip, the length from the
  arcade course lists. Courses without arcade art show their names.
- Lobby: the course still beside the settings, the car's icon and paint chip in their
  rows, a paint chip per player.

Scripted screenshots: `GT3R_ONLINE="host,page=cars|makers|courses,..."` opens a page.

## Mod cars

A mod car is shared as a **package** (`src/game/mod_package.*`). That is its
`cars/<label>.ini` plus the files it references (glTF body, pictures), with
mod-relative paths, as one blob whose SHA-256 is its identity. The owner's
lobby builds and validates its own package when the car is chosen (a car
that would not pass is shown to the others as a stand-in, with a note to its
owner).

**Download.** When the lobby allows mod cars and another player races one this
machine lacks, the lobby asks once per package: "Download it? It is checked,
then added to your mods" (Cross / Circle; Circle = a stand-in car).
- The request goes to the host, which serves the package if it has it, or
  fetches it from the owner, verifies it, and keeps a copy for the next player.
- Chunks are 1 KB on the reliable channel, at most 24 in flight per
  connection. They only flow in the lobby, never during a race.
- On arrival: hash check, `parse_mod_package` and `validate_mod_package`
  against this machine's database. Then `install_mod_package` writes
  `mods/dl-<12 hash digits>/` (mod.ini with priority −100, so the user's own
  mods win), and the lobby rescans mods and reloads its car list. The race
  screen loads its own database, so the car is there with its real body,
  rim and physics.
- Opponents (online players and the AI alike) now draw a mod car's own body
  (`body_mod_for_car`), as the player's car always did.

**Validation** (everything checked before anything is written or loaded):
- **Structure:** magic, at most 8 files and 32 MB, per-type caps (`.ini`
  64 KB, `.glb` 16 MB, `.png` 4 MB).
- **Paths:** plain relative (letters, digits, `. _ -`, at most 3 levels, no
  `..`, absolute or hidden parts, no duplicates even by case).
- **Allowed files:** exactly one `cars/<label>.ini`, otherwise only `.glb`
  and `.png`. A package can never replace a game file.
- **The definition:** it parses and names the advertised label, and its donor
  is on this disc. Its values are in plausible ranges: weight 300–5000 kg,
  dimensions, power up to 1500 kW, torque up to 3000 N·m, rpm, gear ratios
  0.2–20 (EV reductions are ~9), aero, grip 50–150 %, at most 64 raw pokes
  (whose offsets `add_mod_car` already bounds-checks).
- **References:** the files it references are exactly the package's other
  files.
- **Content:** PNGs at most 2048 × 2048, checked from the header before
  decoding, then decoded. The GLB must load, with embedded textures at most
  2048 × 2048 and at most 500,000 vertices.
- **Hardened readers:**
  - glTF sizes, offsets and counts are range-checked (a huge accessor count
    used to overflow the bounds check);
  - out-of-range JSON numbers no longer convert to `int` (undefined
    behaviour);
  - `decode_png` takes a size cap.
- **Real cars must pass:** `GT3R_MODS_DIR=<mods folder>` runs the
  `net_mod_packages_of_installed_mods` test, which builds and validates every
  installed mod car. The EV pack: all 5 pass, 0.3–4.5 MB each.

## Fuzzing

`tests/fuzz/fuzz.cpp` has a fuzz target for every reader that sees untrusted
bytes:
- **Mod car files:** `png`, `glb`, `json`, `mod_ini`, `package`.
- **Network input:** `snapshot`, `online_car`, `rendezvous` (including
  VersionInfo and Error(UpdateRequired)), `relay_server` (datagrams into a
  `RelayServer` with min_online set, optionally with a valid cookie filled
  in; VersionQuery included), `versions` (version strings and the relay's
  versions file).

Build it in a separate tree with sanitizers:

```bash
cmake -S . -B build-fuzz -DGT3R_FUZZ=ON -DGT3R_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all" \
      -DCMAKE_C_FLAGS="-fsanitize=address,undefined"
cmake --build build-fuzz --target gt3r_fuzz
build-fuzz/gt3r_fuzz all 60      # every target for 60 s (built-in mutation fuzzer, works with Apple clang)
```

- **Built-in fuzzer:** it mutates valid seeds (bit flips, interesting
  integers, truncation, splices, huge numbers in text). The current input is
  kept in `fuzz-last-input.bin`, so a sanitizer report can be replayed with
  `gt3r_fuzz <target> 0 fuzz-last-input.bin`.
- **libFuzzer (coverage guided):** on Linux, add `-DGT3R_LIBFUZZER=ON` and
  set `GT3R_FUZZ_TARGET=<target>`. Apple's clang does not ship libFuzzer.

First results (Apple clang, ASan + UBSan, built-in fuzzer):
- **Found and fixed:** the car definition parser converted numbers like
  `2.4e12` to `int` (undefined behaviour). Numbers are now limited to ±1e9 and
  must be finite.
- **Clean:** every target then ran 0.5–1.2 M inputs (30–40 s each) without a
  sanitizer report. That includes the glTF reader after its hardening.

## Feel and fairness

### Physics-based prediction
Each remote car has a **shadow**: a vehicle built from its owner's spec, running
on our course (walls included). On each state the shadow is synced
(`Vehicle::sync_to`):
- **Set from the state:** pose, velocities, wheel speeds, steer and
  suspension, engine speed, gear.
- **Kept:** clutch, differentials, turbo and tyre history.

It then catches up to "now" with the owner's newest input (at most 15 ticks)
and steps once per tick after that. Corrections are blended over 0.1 s as
before. `GT3R_NET_PREDICT=dead` switches back to dead reckoning.

Measured (`net_physics_prediction_beats_dead_reckoning`: a scripted launch,
hard braking, weave and corner; states through the wire format):

| latency | dead reckoning rms / max | physics rms / max |
|---|---|---|
| 100 ms | 3.7 cm / 5 cm | 1.5 cm / 8 cm |
| 150 ms | 7.9 cm / 11 cm | 3.2 cm / 18 cm |

The worst moments are the latency window right after the driver changes input
(the shadow still applies the old one), the known trade of input-based
prediction. A physics tick costs 0.66 µs, so 5 remote cars catching up
16 ticks each is 0.05 ms per frame.

### Contacts
A hit our machine sees between our car and a remote car goes to that car's
owner (`Contact`: point and the impulse the remote car felt). It travels over
the direct link, or through the host. The owner applies it with
`Vehicle::apply_impulse` only when its own machine saw no contact with us
within 0.2 s and the car is not a ghost to it. Impulses are capped at
60,000 N·s.

### Ghosting
A remote car is a ghost (drawn at 50 %, no contact) when:
- its round trip is above the lobby's limit;
- it is within the first 3 s after the start signal;
- it is lapping us or being lapped (race distances more than 0.9 laps apart).

### Lobby car rules
- **Car rules:** any car, or stock only (no tuned garage cars).
- **Power limit:** off, or 200–700 PS at the crank, from the car's own spec
  (the R34 V-spec rates 336 PS).
- **Who checks:** each player's lobby refuses Ready for a car that breaks
  them. The host checks every car it can rebuild before the start (a mod car
  the host lacks cannot be checked there).

### Cheat warnings
Every machine runs `race::Plausibility` on every remote car:
- **Speed:** above its geared top speed × 1.15 + 4 m/s for 0.5 s.
- **Teleport:** a jump its motion does not explain (more than 6 m). States
  carry a respawn flag for 0.5 s after a reset.
- **Acceleration:** a speed gain over 2.5 g in both quarters of a half
  second (engines ramp; a shove is one step, and contacts with us are
  excluded).
- **Motion:** physics disagreeing by more than 5 m for 2 s.
- **Finish time:** a claimed finish more than 2 s from when we saw it cross
  the line.

Each kind fires once per car per race and goes to the host (`Flag`). The host
gets an on-screen warning naming who reported it, and a "!" next to the player
in the lobby. The **Remove a player** row lists each player's warnings and
removes them (Cross twice; they are told "removed by the host"). Flag, don't
ban.

False positives:
- `net_plausibility_honest_drive` (the scripted drive, 100 ms late, with
  physics prediction) raises nothing.
- A 2:50 two-instance race on Trial Mountain through the public relay raised
  nothing.

## Race screen (online mode)

- The tick count comes from the wall clock since the synchronised start, not
  the frame accumulator, so a stalled frame catches up instead of leaving the
  car behind everyone's race clock. Snapshot age is then simply local race
  time minus the snapshot's race time.
- Pause opens GT3's race window (CONTINUE / EXIT, "Exit race?") with a note
  that the race goes on. The car keeps running (no input) and the camera keeps
  following it: the race does not stop for anyone. EXIT retires from the race
  (back to the lobby). Select's longer list is Resume, Camera, Gearbox and
  Retire; restart and replays are not offered. Respawn is disabled (the Select button and `[race] respawn =
  all`); only a car that falls out of the world is put back on the road.
- Each player's finish time is the owner's (reliable Finish message). A
  finished car turns into a ghost (no contact). So does a car whose round trip
  to us is above the lobby's ghost threshold. It is drawn at 50 % alpha and
  turns solid again when the round trip drops. The results use player names.
  After the results everyone returns to the lobby (the host's return sends
  Lobby, which ends the race for anyone still in it).
- A car this machine lacks and does not download (or that fails validation) is
  shown as your own car (a stand-in); physics stay the owner's.

## Server (`tools/gt3r-relay`)

A small rendezvous / relay server makes internet play work without port
forwarding. The server is `net::RelayServer` (`src/net/relay_server.*`); its
wire format is in `src/net/rendezvous.*`.

- **Session codes.** "Create a session" registers the host and gets an
  8-character code (alphabet without 0/O/1/I/L), shown as `ABCD-EFGH`. Players
  type it under "Join with a code", with or without the dash.
- **Endpoint exchange.** On a join the server sends the client the host's
  addresses (public as the server sees it, plus LAN) and the host the
  client's, with a random join nonce. Both punch with the nonce at once. If
  the client hears the host's acknowledgement it sends Hello straight to it.
- **Relay fallback (as TURN).** After 2.5 s without a direct path, the client
  binds the nonce's relay channel and connects through it. (The host binds
  every announced join right away and releases the binding when the player
  came in directly.) Two players who fail to punch each other bind
  `pair_key(session token, a, b)`. The server only accepts that key with the
  token the host registered. A relayed connection is an ordinary connection
  whose datagrams travel as `"GT3R" + key + datagram` via the server.
- **Link slots:** a race whose pairs go through the relay reserves them when
  it starts; a full relay queues the lobby ("Link slots and the queue" below).
- **Limits:** see "Abuse protection" below.
- **The host** sends a keepalive every 5 s, which also keeps its NAT mapping
  open. Bindings are refreshed every 20 s.
- **The lobby** shows the code in large type for the host to read out. Each
  player's link is shown as *direct*, *relay* (through the server) or *via
  host* (direct-IP sessions only).
- `GT3R_NET_FORCE_RELAY=1` disables punching (as behind a symmetric NAT), for
  testing the relay path.

Direct-IP sessions ("Host by address" / "Join by address") still work with no
server: on a LAN, or with UDP 47630 forwarded. In those sessions the host
relays for pairs that cannot punch.

### Protocol (rendezvous version 3)

The message list is in `src/net/rendezvous.h`. Control datagrams start with
`"GT3S"`, u16 version, u8 type. Relayed data is `"GT3R"`, u64 key, then one
game datagram.

- **Requests** (Register, Keepalive, Lookup, Bind, Unregister, Reserve, Release) carry a u64
  cookie right after the header, then the game's version (u16 major, minor,
  patch), and are zero-padded to 64 bytes. The server drops shorter ones.
- **VersionQuery / VersionInfo** (the update check, "Versions and updates"
  below): the game sends its version and a platform tag, padded to 160 bytes;
  the server answers the latest release, min_online and the release page URL
  (at most 128 characters, so at most 148 bytes).
- **Cookie.** SipHash-2-4 over (source IP, source port, 30 s time window),
  keyed with a random per-process secret. It is valid for the current and the
  previous window. A request with a missing or stale cookie gets only a
  16-byte Challenge (request type, cookie), and the game resends the request
  with it. The first request of a session costs one extra round trip, and so
  does the first request after the cookie expires (30–60 s). Unregister and Release are not
  checked: they create nothing and send nothing, and the session token authorizes them.
  Neither is VersionQuery: it creates nothing, and its answer is smaller than it.
- **Slots** (below): Reserve (host: code, token, number of links) is answered
  with Reservation: granted, queued (position, estimated wait in seconds or
  unknown), released, or refused (with a fixed reason). The server also answers
  a Keepalive with Reservation while the session holds or waits for slots, and
  sends one unasked when queued slots are granted. Release frees the slots or
  leaves the queue.
- **Version.** A request with another version gets Error(Version) in the
  requester's own header (the Error layout is unchanged since version 1). It
  is sent only if it is no larger than the request. Older games (versions 1
  and 2) show
  "please update the game to play online" (or a shorter "update the game"
  when the request was short). A newer game talking to an older
  server says "the relay server is out of date", and an older game talking to
  a newer server says "the relay server needs a newer version of the game".
- Version 2 changes: cookies, padding, Challenge, 8-character codes, and link
  slots (Reserve, Reservation, Release).
- Version 3 changes: the game's version in every request, VersionQuery /
  VersionInfo, and Error(UpdateRequired) (code 6: fixed text, then the latest
  version and the release page).
- **Why a new version rather than extending 2.** Nothing public speaks
  version 2. A version 2 request has no version field: its zero padding would
  read as 0.0.0, and long requests none at all. With version 3 every request
  has the field at a fixed place, and version 2 games already get Error(Version),
  which they show as "please update the game to play online": the same
  outcome min_online gives them. Accepting version 2 for a while would have
  needed replies in each requester's header version, which the server does not
  do. So there is no grace window. Deploy the relay together with the first
  version 3 release (the owner's version 2 builds stop playing online until
  updated).

### Link slots and the queue

The relay's capacity is counted in **links**. A link is one relay channel that
carries a race between two players: a player's join channel to the host, or
the channel of a player pair that could not punch. The server carries at most
`--relay-links` (default 300) racing links at once. Pairs that connect directly
never use one.

- **Only while racing.** Channels are still bound in the lobby (a player who
  cannot reach the host directly has the whole lobby on its join channel), but
  a channel outside a reservation is a *lobby* channel: no slot, and a tight
  rate limit (256 kbit/s each way, 20 Mbit/s for all lobby channels together).
  That carries lobby messages, and mod car downloads at about 30 s per MB.
- **Reservation at the start.** When the host starts the race, it counts the
  links its lobby needs (`Session::relay_links_needed`): players connected to it
  through the relay, plus every pair of players that do not both report a
  direct link. A pair still punching counts; players report a pair through a
  relay channel as relayed. A lobby that needs none loads at once and never
  queues. Otherwise the host sends Reserve and the Load goes out when the slots
  are granted, usually one round trip later.
- **Which channels.** After the grant, each of the session's channels takes a
  slot when it first carries data, up to the granted number, and gets the
  racing flood guard (0.5 Mbit/s each way, about twice the busiest legitimate
  channel: a host still forwarding four players' states, 0.26 Mbit/s). A
  channel beyond the reservation (a pair that lost its direct link mid-race)
  keeps the lobby rate, which still carries a race link.
- **Release.** The host sends Release when the race ends (back to the lobby).
  The server also frees the slots when the session closes or expires, and when
  the reservation's channels carry nothing for 10 s. A grant the host no longer
  wants (it cancelled, and that Release was lost) is released again when its
  keepalive's answer reports it.
- **Queue.** A Reserve that does not fit queues the lobby, first come first
  served. A smaller request may start ahead of the head only if the head would
  still start within 60 s. That is estimated from the running reservations'
  expected ends (start plus the typical race length: the average slot hold
  time, 4 min until races have measured it). When that is unknown (every race
  that would free enough slots is overdue), smaller requests may go first only
  while the head has waited less than 60 s.
- **Waiting.** A queued host sends its keepalive every 2 s (instead of 5) and
  must keep doing so: 15 s without one loses the place. Nobody waits more than
  10 min, and at most 100 lobbies wait (`--max-queue`). Beyond that, or for a
  race larger than the whole relay, Reserve is refused ("the relay is busy: try
  again in a few minutes"). Each keepalive's answer carries the position and an
  estimated wait (the same estimate, in order).
- **Lobby.** The host's Start race row then reads, for example, "Relay busy:
  2nd in line, about 3 min - the race starts by itself", and Cross cancels
  (Release). The roster carries the waiting state, so players see the same
  line. The wait is shown in minutes only when it is at least a minute and
  known. The race starts by itself when the grant arrives. A player who stops
  being ready cancels the wait. If the server never answers the Reserve (10 s),
  the race starts anyway (its channels then run at the lobby rate).
- **Bounded:** reservations (one per session, freed as above), queue entries
  (capped, expire). The per-session caps on channels still apply. Reserve needs
  the cookie, like every request that creates state.

### Abuse protection

The server is public, so it must not be usable to aim traffic at anyone, and
no table may grow without bound.

- **Return routability.** Every request that creates state or makes the server
  send to someone else (Register, Keepalive, Lookup, Bind) needs a valid
  cookie. A spoofed source gets one Challenge per request, never larger than
  the request. Both ends of a relay channel have therefore proved their
  address, so a channel cannot be pointed at a victim.
- **Replies to unverified sources.** These are only Challenge, Error(Version)
  and VersionInfo, each no larger than the request (a VersionQuery shorter
  than 160 bytes gets nothing). Every other Error comes after the cookie check
  and carries fixed text: it never echoes the request. Error(UpdateRequired)
  adds the configured latest version and release page, so it can be larger
  than the request, but only goes to a source whose cookie proved it receives
  there. Unknown datagrams get no reply.
- **Rate limits.** Defaults are in `RelayServer::Limits`. Most can be changed
  with `gt3r-relay` flags:

  | Limit | Default | Flag |
  |---|---|---|
  | Control datagrams per source IP (IPv6: per /64; keyed-hash buckets, fixed memory) | 10/s, burst 40 | `--ip-rate`, `--ip-burst` |
  | Control datagrams, all sources (a CPU guard) | 20000/s | |
  | Lookups of unknown codes per source IP | 10/min | `--misses-per-min` |
  | Lookups of unknown codes, all sources | 200/s | `--misses-per-s` |
  | Joins (Incoming to the host) per session | burst 10, then 1 per 2 s | |
  | Pending joins (nonces): per session / total | 16 / 8192 | `--max-pending` |
  | Sessions: total / per source IP | 4000 / 8 | `--max-sessions`, `--sessions-per-ip` |
  | Relay channels: total / per session | 20000 / 15 pairs + its joins | `--max-channels` |
  | Racing links (link slots) / lobbies waiting for them | 300 / 100 | `--relay-links`, `--max-queue` |
  | Racing channel, per direction (a flood guard) | 0.5 Mbit/s | `--channel-mbit` |
  | Lobby channel, per direction / all lobby channels | 256 kbit/s / 20 Mbit/s | `--idle-channel-kbit`, `--idle-mbit` |

  `--bench` (load testing only) lets loopback sources (127.0.0.0/8, ::1) past
  the per-IP limits. Nothing else changes, and it is never on by default.

  When a lookup limit is hit, every lookup from that source is refused,
  including one with the right code. The refusal happens before the lookup, so
  it reveals nothing about which codes exist.
- **Code guessing.** There are 31^8 ≈ 8.5·10^11 codes. Even at the global miss
  limit, with the server full (4000 sessions), a guess hits about once in 12
  days. That rate also needs over 1000 source addresses. With 6 characters it
  was once every 20 minutes. A hit only reveals the host's address and lets
  the guesser into the lobby, where the host can remove them.
- **Bounded state.** Sessions, joins and channels have caps and time out
  (60 s without keepalive, 120 s, 60 s idle). So do reservations and the
  queue (above). Per-IP session counts follow the
  sessions. The rate buckets are a fixed array. Codes and nonces come from
  SipHash in counter mode, so earlier ones do not predict later ones.
- **Logs.** stdout (journald) gets one line when a session is registered,
  closed or expires (code, and the host's IP on registration), one line per
  join (code and the joining player's IP, no port), one when a race's slots
  are queued, granted or freed, and a stats line every minute. Version checks
  and refusals for an old version are only counted in that stats line (per
  platform), never logged one by one. Nothing else is recorded, and nothing goes to disk. Retention is the
  host system's journald setting.

### Versions and updates

The game's version has one source: `project(GT3Revisited VERSION x.y.z)` in
`CMakeLists.txt`. CMake writes it to `gt3r_version.h` (the window and app
metadata, the main menu, Settings, the online checks, the macOS bundle).
Android's `build.gradle` reads the same line: versionName `x.y.z`, versionCode
x·10000 + y·100 + z. A local build may add `-DGT3R_VERSION_SUFFIX=-dev`.

**Comparison rules** (`net::GameVersion`, tested in `net_version_compare`):
- Online play needs x.y.z ≥ the relay's **min_online**. The suffix does not
  count: a 0.2.0-dev build plays where 0.2.0 does. A min_online of 0.0.0 (not
  set) allows every version.
- The **update notice** shows when the relay's **latest** is newer: latest >
  x.y.z for a release, latest ≥ x.y.z for a suffixed build (0.2.0-dev is older
  than the 0.2.0 release).

**The update check** (`net::VersionCheck`, `src/app/updates.*`):
- When: at startup unless `[online] update_check = off` (Settings → Online),
  and on Settings → Online → Check now.
- How: one VersionQuery to `[online] server` on a background thread (the name
  lookup can block). Up to 3 tries a second apart, then it gives up. It never
  holds up the menus. What is sent: the game's version and platform (windows,
  macos, linux, android), nothing else.
- Cache: the answer is kept in config.ini (`update_latest`,
  `update_min_online`, `update_url`, `update_checked`), so a start without a
  network still knows. `--shot` runs (screenshots, scripted tests) use only
  the cache unless `GT3R_UPDATE_CHECK=1`, so they never reach a live server.

**The update dialog** (`src/app/updates.*`; drawn by App over any screen,
modal; pad, keyboard, mouse and touch):
- **Update available** (latest is newer): "GT3 Revisited <latest> is
  available. You have <ours>."
- **Update required to play online** (below min_online): "Online play needs
  GT3 Revisited <min_online> or newer. You have <ours>. Offline play is not
  affected."
- Buttons: **Download update** (default focus) opens this platform's package
  (below), **Later** closes, and a **Release notes** link opens the release
  page. Circle / Triangle / Esc mean Later.
- **At startup** it shows once per release, over the main menu, the first-run
  screen or the game's title page (`Screen::update_prompts`). Later stores the
  release in `[online] update_dismissed`; it shows again only for a newer one
  (`net::show_startup_prompt`, tested in `net_update_prompt_rules`).
- **Online battle** opens the "required" dialog every time it is entered while
  this game is below min_online, when Create a session / Join with a code is
  chosen anyway, and when the relay answers UpdateRequired. After Later the
  lobby stays open for Host / Join by address.
- **Afterwards:** a small badge beside the version in the main menu's title bar
  ("Update available" / "Update required for online") opens the dialog (tap,
  click, or Up from the top row and Cross). Settings → Online has **Download
  update** (the dialog) and **Release notes**.

**Downloads.** Every GitHub Release carries its packages under
stable names, so `https://github.com/<owner>/<repo>/releases/latest/download/<name>`
is always the newest:

| Platform | Stable asset |
|---|---|
| Windows x64 | `gt3r-windows-x64.zip` |
| macOS arm64 | `gt3r-macos-arm64.zip` |
| Linux x86_64 | `gt3r-linux-x86_64.tar.gz` |
| Android (arm64, x86_64) | `gt3r-android.apk` (only when the APK was built) |
| Meta Quest (the VR build, `GT3R_XR`) | `gt3r-quest.apk` (likewise) |

The relay's `release_url` may hold an `{asset}` placeholder
(`net::update_links`, `net::release_asset`; tested in
`net_release_asset_per_platform` and `net_update_links`):
- **Download update** opens the URL with `{asset}` replaced by this
  platform's name.
- **Release notes** opens the URL cut at `/download/`, i.e.
  `.../releases/latest` on GitHub.
- A platform without a package (Windows arm64, Intel macOS, Linux arm64, iOS)
  gets the release page for both. So does a URL without the placeholder.
- **Caveat:** if a release lacks a platform's stable asset, that platform's
  direct link answers 404. Without the Android signing key there is no APK, so
  set the key up before the first release that players update to.

**Enforcement on the relay** (`net_relay_min_online_enforced`):
- Requests that create state from a game below min_online get
  Error(UpdateRequired): Register, Lookup, Reserve, and a Bind that would add a
  channel or a channel end.
- This happens after the cookie check, like every other error. The reply
  carries fixed text, the latest version and the release page.
- Not refused (they create nothing new): Keepalive, Unregister, Release, and a
  Bind that refreshes an end the sender already holds.
- So a lobby made before min_online was raised keeps running and finishes its
  race. Its next race start (Reserve) is refused: the host's lobby shows
  "Relay server: update required to play online: version x.y.z" and stays open.
- A game refused while hosting or joining ends that attempt with the same
  text. The lobby then switches to its "update required" state, with the
  release page from the error.

**Direct-IP sessions** (Host / Join by address, no relay) need the same x.y.z
on both sides (the suffix does not count), on top of the game protocol check.
- The host refuses a different version in Hello. The joiner's message says
  who must update: "update required: the host has version 0.2.0 (you have
  0.1.0)", or "the host has an older version (…): the host needs to update".
- Welcome carries the host's version (older games ignore the extra string). A
  joiner leaves a host whose version differs, or a host too old to send one.
- Through a relay, the relay's min_online decides instead (`net_direct_ip_needs_same_version`).

**Relay configuration:**
- Flags `--latest`, `--min-online`, `--release-url`, or a versions file
  (`--versions-file`; the systemd unit uses `/etc/gt3r-relay/versions.conf`).
  The file holds `latest = x.y.z`, `min_online = x.y.z` and
  `release_url = https://...` lines (at most 128 characters; `{asset}`
  allowed, see "Downloads"), which win over the flags.
- `systemctl reload gt3r-relay` (SIGHUP) re-reads the file. Sessions,
  channels and reservations stay. A file that does not parse (bad version,
  URL, unknown key, min_online above latest) is refused and the previous
  values stay; the log says so.
- Unconfigured (no flags, no file), every version may play and the check
  reports no release.

**Announcing a release from your relay:**
1. Publish the release with its stable-name packages (the workflow's
   `release` job attaches them).
2. If the rendezvous protocol or the relay changed, deploy the relay first:
   `tools/gt3r-relay/deploy.sh root@<server>`. An update keeps
   `/etc/gt3r-relay/versions.conf`.
3. `tools/gt3r-relay/set-versions.sh root@<server> <x.y.z> [min_online] [url]`.
   min_online defaults to the new version. The URL is kept when left out; the
   first run needs it, e.g.
   `https://github.com/<owner>/<repo>/releases/latest/download/{asset}`.
   The script writes the file, reloads the relay, and prints its "reload:" log
   line with the new values. Games older than min_online can no longer create
   or join sessions; they show the update dialog.

### Running your own relay

The game's default server is `gt3r.gameversions.com` (`kDefaultServer` in
`src/app/screen_online.h`). Anyone can run a relay of their own: players
reach it by entering its name or address under "Relay server" in the lobby
(`[online] server`), and a build of your own can make it the default by
changing `kDefaultServer`. Give it a DNS name that points straight at the
server (UDP 47631), not through an HTTP proxy, since the relay is plain UDP.
Deploy a new relay before releasing a game build with a new rendezvous
version: older games then get "please update the game".

For players spread over a continent, run one server near its middle: in the
US, coast-to-coast players then see roughly 15–35 ms extra on
a relayed pair, and nothing on direct pairs. Any small Linux VPS works (1 vCPU
/ 512 MB). Relay traffic is only for pairs that cannot connect directly.

**Capacity and cost, rule of thumb** (measured; IP/UDP headers included):

- One link (a relayed pair, both directions): 60 datagrams/s each way,
  0.15 Mbit/s into the relay and 0.15 Mbit/s out.
- A fully relayed 2-player race is 1 link: 0.15 Mbit/s each way. A fully
  relayed 6-player race is 15 links: 2.2 Mbit/s each way.
- 300 slots in use: 45 Mbit/s each way, 36,000 datagrams/s relayed.
- Traffic: each slot in use around the clock is about 100 GB a month (in
  plus out; half of it each way): about $1 at a typical $0.01/GB beyond a
  plan's included transfer. Real use is far lower: slots are only held while racing.
- CPU: measure the server with the benchmark below. On an M-series Mac the
  relay used about 8.4 µs of CPU per relayed datagram, so 300 links took
  ~30 % of a core (indicative only: macOS syscalls differ from Linux, and a
  VPS core is slower).

Use a server with an IPv4 address (IPv6-only servers can't be reached by
IPv4-only players), e.g. Debian or Ubuntu. Then, from this repository:

```bash
tools/gt3r-relay/deploy.sh root@<server IP>   # builds and installs over SSH; run again to update
```

The script copies only the relay's own sources (20 KB, no game code or
data), installs g++ / cmake, and builds on the server. It installs
`/usr/local/bin/gt3r-relay` and the hardened systemd unit, opens UDP 47631 in
ufw if ufw is active, and starts the service. Alternatives:

```bash
# Docker (from the repository root)
docker build -f tools/gt3r-relay/Dockerfile -t gt3r-relay .
docker run -d --name gt3r-relay --restart unless-stopped -p 47631:47631/udp gt3r-relay

# or plain: build without SDL or the game, install with systemd
cmake -S tools/gt3r-relay -B build-relay -DCMAKE_BUILD_TYPE=Release && cmake --build build-relay
sudo cp build-relay/gt3r-relay /usr/local/bin/
sudo cp tools/gt3r-relay/gt3r-relay.service /etc/systemd/system/
sudo systemctl enable --now gt3r-relay
```

- Open UDP 47631 in the VPS firewall.
- Point a DNS name at the server (e.g. `relay.example.org`).
- Either set `kDefaultServer` in `src/app/screen_online.h` before building
  the game, or have players enter it under "Relay server" (`[online] server`).
  The update check asks the same server.
- The server logs one line per session event, plus a stats line every minute
  (sessions, channels, pending joins, links in use / capacity, racing and
  queued lobbies, relayed / dropped, the busiest channel, challenges, rate
  limited, the typical race length, version checks per platform, requests
  refused as outdated). At startup and on each reload it logs the versions in
  force. Limits are flags (`gt3r-relay --help`); the systemd unit passes the
  main ones.
- It asks for 4 MB socket buffers (`--buffers-kb`). Linux caps that at
  `net.core.rmem_max` / `wmem_max` (often 208 KB); the startup line shows what
  it got. Raise them for bursts:
  `sysctl -w net.core.rmem_max=4194304 net.core.wmem_max=4194304`.

### Benchmark

`gt3r-relay-bench` (`tools/gt3r-relay/bench.cpp`, built by the standalone relay
CMake and installed by `deploy.sh`; no SDL) runs simulated relayed races
against a relay. It uses the real protocol (cookies, register, lookups, binds,
reservations, keepalives). A lobby is six sockets with 15 links, and every
player sends a 126-byte datagram on each of its channels at 60 Hz, like a
fully relayed six-player race. Each step reports:
- links racing and lobbies queued;
- datagrams/s and Mbit/s into and out of the relay;
- loss;
- relay-added latency p50 / p99 (send → relay → receive, timestamps in the
  payload);
- with `--pid`, the relay's CPU time during the step (Linux `/proc/<pid>/stat`,
  macOS `proc_pid_rusage`): CPU seconds per million relayed datagrams, and the
  datagram rate and link count 70 % of one such core would carry;
- the generator's own CPU.

When both run on one machine they share its CPU (the output says so). On
Linux the generator batches with `sendmmsg` / `recvmmsg`. The relay must run
with `--bench`, and with enough slots for the ramp. Never use `--bench` on the
public port.

```bash
# on the server (a second relay on another port; the live one keeps running,
# but shares the CPU: run it at a quiet time, or stop the service first)
gt3r-relay --port 47700 --bench --relay-links 10000 --quiet &
gt3r-relay-bench --server 127.0.0.1:47700 --ramp 50,100,200,300,400,600 --duration 10 --pid $!
kill %1
```

Local run (M-series Mac, macOS, relay and generator on the same machine;
indicative only):

| links | datagrams/s in = out | Mbit/s each way | loss | p50 / p99 ms | relay CPU | CPU s per M datagrams | links at 70 % of a core |
|---|---|---|---|---|---|---|---|
| 50 | 6,000 | 7.4 | 0 | 0.11 / 0.67 | 6 % | 10.7 | 545 |
| 100 | 12,000 | 14.8 | 0 | 0.14 / 5.4 | 12 % | 9.9 | 589 |
| 200 | 23,800 | 29.3 | 0 | 2.9 / 33 | 23 % | 9.6 | 607 |
| 300 | 36,000 | 44.4 | 0 | 1.8 / 3.7 | 30 % | 8.4 | 695 |
| 400 | 48,000 | 59.1 | 0 | 1.6 / 3.9 | 40 % | 8.4 | 698 |
| 600 | 72,000 | 88.7 | 0 | 1.2 / 5.1 | 58 % | 8.1 | 725 |

An earlier run went further: 1000 links used 94 % of the relay's core (no
loss, p50 25 ms), and 1500 links saturated it (33 % loss). The latency spikes
at 100–200 links are the busy desktop, not the relay.

## Testing

- Unit tests: `net_*` (wire format, loopback sessions with and without 30 %
  packet loss, rejection, prediction) and `sim_world_kinematic_remote_car`.
  The `net_relay_*` tests cover the server: code joins, relay fallback, cookie
  challenges and rollover, spoofed binds, rate limits, join caps, and old
  protocol versions. They also cover link slots:
  - grant and release (Release, unregister, 10 s without traffic);
  - queue order and the 60 s skip rule (known and unknown estimates);
  - the queue cap, expiry, and keepalives to hold a place;
  - `--bench` (loopback only);
  - a direct-only lobby never queues;
  - a lobby that waits for a full relay, starts by itself, cancels;
  - per-link traffic of a fully relayed six-player race
    (`net_relay_traffic_per_link`, the figures above).
- Versions: `net_version_compare` (the rules), `net_relay_version_query`
  (round trip, reply no larger than the query, no cookie, counted not logged,
  rate limited), `net_relay_min_online_enforced` (each state-creating request
  below / at / above min_online), `net_relay_versions_reload` (the file,
  reload keeps sessions), `net_relay_unconfigured_allows_all`,
  `net_relay_update_required_client` and `..._at_race_start` (the game's
  handling), `net_version_check_roundtrip` (the background check, and its
  time-out), `net_direct_ip_needs_same_version`.
- The loopback tests do not depend on how fast the machine runs them (they
  used to fail now and then while other builds loaded the CPU):
  - Simulated clock: `net::set_clock` swaps `now_ms()` for the sessions, their
    receive threads and the relay server. The tests' `FakeClock` moves 5 ms per
    pump step (with a real 5 ms sleep), so timers such as resends, punch
    give-up, relay fallback, time-outs, rate limits and cookie windows count
    pump steps rather than wall time.
  - Link slot tests: the server runs on `receive` / `tick` with the test's
    `now`. Its replies go to in-memory mailboxes (`RelayServer::set_send_hook`),
    because loopback UDP is delivered asynchronously on macOS.
  - Waiting for datagrams uses bounded loops that stop as soon as the
    condition holds, not fixed sleeps.
- Two instances on one machine:

```bash
GT3R_RACE="auto=1,cam=1" GT3R_ONLINE="host,players=2,course=trialmt,laps=2,car=r34_gtr_v_99" \
  gt3r disc.iso --screen online
GT3R_RACE="auto=1,cam=1" GT3R_ONLINE="join=127.0.0.1,car=supra_rz_97,name=Two" gt3r disc.iso --screen online
```

## Plan

1. ~~Netcode core: compact state with inputs, client mesh with hole punching
   and relay fallback, receive thread, continuous clock sync, ghost
   threshold.~~ (done)
2. ~~Rendezvous / relay server (`tools/gt3r-relay`): session codes, endpoint
   exchange (no port forwarding), per-pair relay fallback.~~ (done; deploy it:
   "Running the server")
3. ~~Cars: garage cars as descriptors; mod cars as validated, content-addressed
   packages offered for download.~~ (done; fuzz targets: "Fuzzing" above)
   - **Package validation:**
     - allowed file types only (`.ini`, `.glb`/`.gltf`/`.bin`, `.png`);
     - a package may not replace game files;
     - relative, plain paths only;
     - size and count caps;
     - strict parsing with limits (image size, glTF counts and bounds);
     - `.ini` keys and physics values checked against plausible ranges;
     - fuzz targets for the PNG / glTF / JSON / ini readers under
       AddressSanitizer.
4. ~~Feel and fairness: physics-based prediction, contact messages, ghosting at
   the start and for lapped cars, lobby car rules, cheat warnings and removing
   players.~~ (done)

## Known issues

- Remote players' engine notes use their car's opponent engine sound; turbo /
  whine extras are not sent.
- No online replays yet (they would need the remote snapshots recorded too).
- No 2P split screen combined with online players.
- Players' IP addresses are visible to the other players (inherent to peer
  links; `GT3R_NET_FORCE_RELAY=1` sends everything through the server instead).
- The relay is built and run on Linux (Debian, g++) and macOS (clang); the
  Windows build compiles but is not run as a server.
