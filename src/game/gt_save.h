#pragma once

// GT Mode saves (docs/dev/GT_MODE.md "Save format").
//
// Native format: a versioned UTF-8 text file, one record per line
// ("key value..." tokens separated by spaces; strings never contain
// spaces). Written atomically (temporary file + rename); the saves live in
// <user data dir>/saves/ (save_library.h). Unknown keys are ignored when
// loading, so newer versions can add records.
//
// Import: the original PS2 memory-card file
// BASCUS-97102GAMEDATA/BASCUS-97102GAMEDATA (save version 7.1: 0x40-byte
// header, payload bytes inverted, zlib CRC-32), see docs/FORMATS.md "PS2 memory-card saves".

#include "game/gt_mode.h"

#include <optional>
#include <string>

namespace gt3r::game {

constexpr int kSaveVersion = 1;

std::string save_to_text(const GameState& s);
// Throws FormatError on malformed input or an unsupported version.
GameState load_from_text(const std::string& text);

// The user data folder (with a trailing separator): SDL_GetPrefPath, or the executable's
// folder in portable mode (portable.ini beside it); the app keeps mods and config there.
std::string user_data_dir();
bool write_save(const std::string& path, const GameState& s, std::string* error = nullptr);
std::optional<GameState> read_save(const std::string& path, std::string* error = nullptr);

// Decodes an original GT3 (NTSC-U) GAMEDATA file. Needs the disc data for
// the race-code / prize-key orders and the part tables. Throws FormatError.
GameState import_ps2_save(ByteSpan file, const GtData& data);
// The inverse, for tests and for moving progress back to the PS2: builds a
// GAMEDATA file from a state (fields this reimplementation does not track
// are written as a new game has them).
Bytes export_ps2_save(const GameState& s, const GtData& data);

// The original's memory-card save directory for the game file (spec §12):
// BASCUS-97102GAMEDATA/ {BASCUS-97102GAMEDATA, icon.sys, icon0.ico, thumbnail}.
constexpr const char* kPs2GameDir = "BASCUS-97102GAMEDATA";
struct Ps2SaveFile {
    std::string name;
    Bytes data;
};
// Every file of the directory, in the order the original writes them (icon0.ico,
// icon.sys, the data file, thumbnail). The icon files come from the disc
// (menu/icon/icon.sys is the template the game fills in, menu/icon/ICON_a_spec.ico
// the icon); `read` reads GT3.VOL paths. Throws FormatError when they are missing.
std::vector<Ps2SaveFile> export_ps2_save_files(const GameState& s, std::shared_ptr<const GtData> data,
                                               const GtData::Reader& read);
// File-level import / export for players (gt3tool save-import / save-export; the
// game's file picker). Supported containers: the data file itself
// (BASCUS-97102GAMEDATA), its save directory, a PCSX2 folder memory card (the
// folder holding BASCUS-97102GAMEDATA/), an EMS / uLaunchELF .psu and a raw 8 MB card
// image (PCSX2 .ps2 / .mcd, with or without ECC). Throws FormatError.
GameState import_ps2_save_path(const std::string& path, const GtData& data);
// `out` ending in .psu: a .psu; in .ps2 / .mcd: a new card image (with ECC) holding
// only this save; otherwise a folder: <out>/BASCUS-97102GAMEDATA/... (e.g. a PCSX2
// folder memory card). Throws FormatError.
void export_ps2_save_path(const GameState& s, std::shared_ptr<const GtData> data, const GtData::Reader& read,
                          const std::string& out);

// The same as one file: a .psu (`card` false) or a new 8 MB card image with ECC (true),
// e.g. for a path from a save dialog that has no extension. Throws FormatError.
Bytes export_ps2_save_container(const GameState& s, std::shared_ptr<const GtData> data, const GtData::Reader& read,
                                bool card);

// The 0x60-byte thumbnail the file list shows without loading the data file.
Bytes ps2_thumbnail(const GameState& s, std::shared_ptr<const GtData> data);

// The original's car setup file (memory card BASCUS-97102S…: version
// 1.0x827, payload = one 0x1D0-byte car setup record, same header,
// inversion and CRC as the game data). Throws FormatError.
SettingFile import_ps2_setting(ByteSpan file, const GtData& data);
// One car setup record (0x1D0 bytes; also the head of the car records in the game's replay
// saves, e.g. the disc's demo runs): the car label, its fitted parts and user settings.
void decode_car_equipments(ByteSpan r, const data::CarDatabase& db, std::string& car, std::array<u64, 27>& parts,
                           sim::CarSettings& settings, int& ref_final);
Bytes export_ps2_setting(const SettingFile& f, const GtData& data);

} // namespace gt3r::game
