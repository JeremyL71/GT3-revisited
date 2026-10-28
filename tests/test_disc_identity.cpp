// The NTSC-U releases are one game (docs/COMPATIBILITY.md): US 1.00 (SCUS_971.02) and 1.10 (PBPX_955.03) ship the same
// CORE.GT3 and GT3.VOL; only their boot files and disc headers differ. The disc tests run on 1.10; on 1.00 this one
// checks that the disc opens as the same release:
//
//   GT3R_DISC=<US 1.00> gt3r_tests disc_identity
//
// Either NTSC-U disc in GT3R_DISC opens as a verified NTSC-U/C release on the NTSC-U data set and program. The byte
// comparison is a one-off check, for when disc support changes or another printing turns up, and runs only when asked
// for: with the other NTSC-U release in GT3R_DISC_IDENTITY, CORE.GT3 is the same and so is GT3.VOL, file for file and
// byte for byte as stored.
//
//   GT3R_DISC=<US 1.00> GT3R_DISC_IDENTITY=<US 1.10> gt3r_tests disc_identity

#include "test_framework.h"

#include "disc/core_image.h"
#include "disc/disc.h"
#include "disc/region.h"
#include "sim/exact/program_data.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

using namespace gt3r;
namespace ex = gt3r::sim::exact;

namespace {

bool is_us_100(const DiscInfo& i) { return i.serial == "SCUS_971.02" && i.version == "1.00"; }
bool is_us_110(const DiscInfo& i) { return i.serial == "PBPX_955.03" && i.version == "1.10"; }

// What makes a disc the NTSC-U game: a release in the known list, its data set, its program.
void check_ntsc_u_release(Disc& disc) {
    const DiscInfo& i = disc.info();
    std::printf("  %s v%s: %s, %s data\n", i.serial.c_str(), i.version.c_str(), i.description.c_str(),
                region_name(i.region));
    CHECK(is_us_100(i) || is_us_110(i));
    CHECK(i.verified && !i.reported && i.problem.empty());
    CHECK(i.description == "Gran Turismo 3 A-Spec (NTSC-U/C)");
    CHECK(i.region == Region::America && disc.volume().region() == Region::America);
    CHECK(region_of_serial(i.serial) == Region::America);
    const ex::ProgramData d = ex::ProgramData::load(CoreImage::decode(disc.read_core_gt3()));
    CHECK(d.known() && d.issues().empty() && d.program() == ex::Program::NtscU);
    CHECK(d.behaviour() == (ex::Behaviour{ex::Program::NtscU, 60}));
}

} // namespace

TEST(disc_identity_ntsc_u) {
    Disc* disc = test::test_disc();
    if (!disc) SKIP("needs GT3R_DISC (an NTSC-U disc)");
    check_ntsc_u_release(*disc);
    const char* other_path = std::getenv("GT3R_DISC_IDENTITY");
    if (!other_path || !*other_path) return; // the byte comparison only when asked for
    std::unique_ptr<Disc> other = Disc::open(other_path);
    check_ntsc_u_release(*other);
    CHECK(is_us_100(disc->info()) != is_us_100(other->info())); // one of each

    CHECK(disc->read_core_gt3() == other->read_core_gt3());
    const std::vector<Volume::Entry>& a = disc->volume().entries();
    const std::vector<Volume::Entry>& b = other->volume().entries();
    CHECK(a.size() == b.size() && !a.empty());
    if (a.size() != b.size()) return;
    size_t differ = 0;
    u64 bytes = 0;
    for (size_t k = 0; k < a.size(); ++k) {
        const Volume::Entry &x = a[k], &y = b[k];
        const bool same = x.path == y.path && x.sector == y.sector && x.size == y.size &&
                          x.stored_size == y.stored_size && x.compressed == y.compressed && x.part == y.part &&
                          disc->volume().read_raw(x) == other->volume().read_raw(y);
        if (!same && differ++ < 10) std::printf("  differs: %s\n", x.path.c_str());
        bytes += x.stored_size;
    }
    std::printf("  GT3.VOL: %zu files (%.2f GB as stored), %zu differ\n", a.size(), double(bytes) / 1e9, differ);
    CHECK(differ == 0);
}
