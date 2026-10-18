#pragma once

// GT3's background-music layer. The game addresses
// all BGM with one id:
//   0x00..0x11  sequenced menu music: the music.inf entry with that name in
//               the game's table (alphabetical: 0 arcade01 ... 17 wheelshop01)
//   0x12..0x22  streamed music: bgm/ads.inf group id - 0x12; the track is
//               `index` within the group, or a random one for index -1;
//               index | 0x1000 plays the intro file ("bgm/i_<file>")
// Two entry points: play() always (re)starts; play_if_different() keeps the
// current music when it is the same piece (same sequence, or same stream
// file), as the menus do. Stopping is immediate (no fades).

#include "formats/sound/mads.h"

#include <random>
#include <string>

namespace gt3r {

class AudioEngine;
class Vfs;

namespace bgm {

// Sequenced ids (music.inf names).
constexpr int kArcade = 0x00, kBattle2P = 0x01, kDealer = 0x02, kFree = 0x03, kGorace = 0x04, kGtMenu = 0x05,
              kHome = 0x06, kLicense1 = 0x07, kLicense2 = 0x08, kMachTest = 0x09, kMain = 0x0A, kMini = 0x0B,
              kOption = 0x0C, kTheater = 0x0D, kTimeTrial = 0x0E, kTune = 0x0F, kWash = 0x10, kWheelShop = 0x11;
// Streamed ids (ads.inf groups; working titles).
constexpr int kStartLong = 0x12;     // str01a "jg11": standing start held 10 s (start type 2)
constexpr int kFinish = 0x13;        // str02a "jg12": the player finishes (2P: the winner)
constexpr int kRace = 0x14;          // race_a01..25: in-race music
constexpr int kQuickMenu = 0x15;     // str10a "shortmenu-2": pre-race / time trial / license quick menus
constexpr int kResult = 0x16;        // str05a "result": race result quick menus
constexpr int kMiniPrize = 0x17;     // str14a "jg05": arcade trophy, GT-mode trophy, consolation prize
constexpr int kTrophy = 0x18;        // str12a "champ-3": championship trophy presentation
constexpr int kCarGet = 0x19;        // str13a "getacar": prize car presentation (plays once)
constexpr int kOpening = 0x1A;       // str91a "Are You Gonna Go My Way" (opening)
constexpr int kStartShort = 0x1B;    // str03a "jg04e": standing start held 4 s (start type 1)
constexpr int kJg01 = 0x1C;          // str04a (no caller found)
constexpr int kLicenseGold = 0x1D;   // str06a "jg06"
constexpr int kLicenseSilver = 0x1E; // str07a "jg05"
constexpr int kLicenseBronze = 0x1F; // str08a "bronze"
constexpr int kFail = 0x20;          // str09a "fail": license failed / time trial failed
constexpr int kLicenseGet = 0x21;    // str11a "champ-2": license card presentation
constexpr int kDemo = 0x22;          // demo_a01: attract-mode race

constexpr int kIntro = 0x1000; // index flag: the intro variant of a stream

const char* sequence_name(int id); // music.inf name for 0x00..0x11, else nullptr
bool is_sequence(int id);
bool is_stream(int id);

class Player {
public:
    // Starts `id` (restarting it if it already plays). loop: streams repeat
    // (menus, presentations) or play once (race tracks, jingles; the game's
    // play flag). Sequences always loop. Returns false if nothing could play.
    bool play(AudioEngine& audio, Vfs& vfs, int id, int index = -1, bool loop = true);
    // Like play(), but keeps the current music when it is the same piece.
    bool play_if_different(AudioEngine& audio, Vfs& vfs, int id, int index = -1, bool loop = true);
    void stop(AudioEngine& audio);

    int current() const { return id_; } // -1 when stopped
    // True while the current piece plays (a non-looping stream has ended
    // when this turns false).
    bool playing(const AudioEngine& audio) const;

private:
    bool load(Vfs& vfs);
    // Identity of what (id, index) would play: sequence index or stream file.
    std::string identity(AudioEngine& audio, Vfs& vfs, int id, int index);

    bool loaded_ = false;
    sound::MadsIndex mads_;
    int id_ = -1, index_ = -1;
    std::string what_;
    std::mt19937 rng_{std::random_device{}()};
};

} // namespace bgm
} // namespace gt3r
