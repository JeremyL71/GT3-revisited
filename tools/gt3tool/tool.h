#pragma once

// gt3tool command registry. Each cmd_*.cpp file registers its commands:
//
//   GT3TOOL_COMMAND(tex, "tex", "<vol-path> <outdir>", "decode Tex1 sets to PNGs", 2,
//                   [](Disc& disc, const Args& a) { ...; return 0; });
//
// `min_args` counts arguments after the disc path.

#include "disc/disc.h"

#include <functional>
#include <string>
#include <vector>

namespace gt3r::tool {

using Args = std::vector<std::string>;

struct Command {
    const char* name;
    const char* usage;
    const char* help;
    size_t min_args;
    std::function<int(Disc&, const Args&)> run;
};

std::vector<Command>& commands();

struct CommandRegistrar {
    explicit CommandRegistrar(Command c) { commands().push_back(std::move(c)); }
};

// Writes a host file, creating parent directories. Prints an error on failure.
bool save(const std::string& path, ByteSpan data);
// Looks up a GT3.VOL entry or throws a FormatError naming it.
const Volume::Entry& need(Volume& vol, const std::string& path);

} // namespace gt3r::tool

#define GT3TOOL_COMMAND(var, name, usage, help, min_args, fn) \
    static ::gt3r::tool::CommandRegistrar gt3tool_cmd_##var { ::gt3r::tool::Command{name, usage, help, min_args, fn} }
