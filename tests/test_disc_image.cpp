// A disc image shorter than its volume descriptor says (a dump that stopped early), disc-free: a synthetic image of a
// primary volume descriptor and nothing else reads its volume size, and the player's message names both sizes.

#include "base/file.h"
#include "base/gzip.h"
#include "disc/disc.h"
#include "disc/iso9660.h"
#include "disc/volume.h"
#include "test_framework.h"

#include <cstring>
#include <filesystem>
#include <limits>

#include <zlib.h>

using namespace gt3r;

TEST(disc_image_incomplete) {
    // 17 sectors on disk; the descriptor says 1,791,776 sectors of 2048 bytes (3.67 GB).
    std::vector<u8> img(17 * 2048, 0);
    u8* pvd = img.data() + 16 * 2048;
    pvd[0] = 1;
    std::memcpy(pvd + 1, "CD001", 5);
    const u32 sectors = 1791776;
    for (int k = 0; k < 4; ++k) pvd[80 + k] = u8(sectors >> (8 * k)), pvd[84 + 3 - k] = u8(sectors >> (8 * k));
    pvd[128] = 0x00, pvd[129] = 0x08, pvd[130] = 0x08, pvd[131] = 0x00; // 2048, both byte orders
    const std::string path = test::temp_path("incomplete.iso").string();
    CHECK(write_host_file(path, ByteSpan(img.data(), img.size())));
    std::unique_ptr<File> f = File::open(path);
    CHECK(f != nullptr);
    if (!f) return;
    const Iso9660 iso(*f);
    CHECK(iso.volume_bytes() == u64(sectors) * 2048);
    CHECK(incomplete_image(f->size(), iso.volume_bytes()) ==
          "This disc image is incomplete (0.00 of 3.67 GB). Make the image again from your disc.");
    CHECK(incomplete_image(2470000000ull, iso.volume_bytes()) ==
          "This disc image is incomplete (2.47 of 3.67 GB). Make the image again from your disc.");
    CHECK(incomplete_image(iso.volume_bytes(), iso.volume_bytes()).empty() && incomplete_image(f->size(), 0).empty());
    f.reset();
    std::filesystem::remove(path);
}

namespace {

constexpr u32 kSector = 2048;

void put16(std::vector<u8>& b, size_t at, u32 v) { b[at] = u8(v), b[at + 1] = u8(v >> 8); }
void put32(std::vector<u8>& b, size_t at, u32 v) {
    for (int k = 0; k < 4; ++k) b[at + size_t(k)] = u8(v >> (8 * k));
}
void put32be(std::vector<u8>& b, size_t at, u32 v) {
    for (int k = 0; k < 4; ++k) b[at + size_t(k)] = u8(v >> (8 * (3 - k)));
}

// A directory record (ISO9660 9.1) at `at`: its length.
size_t dir_record(std::vector<u8>& b, size_t at, const std::string& name, u32 lba, u32 size, bool dir) {
    const size_t len = 33 + name.size() + (name.size() % 2 == 0 ? 1 : 0);
    b[at] = u8(len);
    put32(b, at + 2, lba), put32be(b, at + 6, lba), put32(b, at + 10, size), put32be(b, at + 14, size);
    b[at + 25] = dir ? 2 : 0;
    b[at + 32] = u8(name.size());
    for (size_t k = 0; k < name.size(); ++k) b[at + 33 + k] = u8(name[k]);
    return len;
}

// A file system whose sector 0 is `base`: its descriptor (`sectors` long) and a root directory at its sector 18 with
// these files.
struct IsoFile {
    std::string name;
    u32 lba;
    Bytes data;
};
void file_system(std::vector<u8>& img, u32 base, u32 sectors, const std::vector<IsoFile>& files) {
    const size_t pvd = size_t(base + 16) * kSector;
    img[pvd] = 1;
    std::memcpy(&img[pvd + 1], "CD001", 5);
    put32(img, pvd + 80, sectors), put32be(img, pvd + 84, sectors);
    put16(img, pvd + 128, kSector), img[pvd + 130] = 0x08, img[pvd + 131] = 0x00;
    dir_record(img, pvd + 156, std::string(1, '\0'), 18, kSector, true);
    size_t at = size_t(base + 18) * kSector;
    at += dir_record(img, at, std::string(1, '\0'), 18, kSector, true);
    at += dir_record(img, at, std::string(1, '\1'), 18, kSector, true);
    for (const IsoFile& f : files) {
        at += dir_record(img, at, f.name + ";1", f.lba, u32(f.data.size()), false);
        std::memcpy(&img[size_t(base + f.lba) * kSector], f.data.data(), f.data.size());
    }
}

// A GT3.VOL with these files (each in a sector of its own after the index).
Bytes vol(const std::vector<std::pair<std::string, Bytes>>& files) {
    const u32 n = u32(files.size()), root = 0x14, nodes = root + 8 + 4 * (n + 1);
    std::vector<u8> toc(nodes + 12 * n, 0);
    std::vector<u8> pool{0xFF}; // the root's name
    std::vector<u32> names;
    for (const auto& f : files) {
        names.push_back(u32(pool.size()));
        for (char c : f.first) pool.push_back(u8(~u8(c)));
        pool.push_back(0xFF);
    }
    const u32 pool_at = u32(toc.size());
    put32(toc, 0, Volume::kMagic), put16(toc, 4, 2), put16(toc, 6, 2);
    put32(toc, root, (1u << 24) | pool_at), put32(toc, root + 4, n + 1), put32(toc, root + 8, 0);
    for (u32 i = 0; i < n; ++i) {
        const u32 node = nodes + 12 * i;
        put32(toc, root + 12 + 4 * i, node);
        put32(toc, node, pool_at + names[i]), put32(toc, node + 4, 1 + i), put32(toc, node + 8, u32(files[i].second.size()));
    }
    toc.insert(toc.end(), pool.begin(), pool.end());
    put32(toc, 8, u32(toc.size())), put32(toc, 0xC, pool_at);
    Bytes v((n + 1) * kSector, 0);
    std::memcpy(v.data(), toc.data(), toc.size());
    for (u32 i = 0; i < n; ++i) std::memcpy(&v[(1 + i) * kSector], files[i].second.data(), files[i].second.size());
    return v;
}

Bytes ascii_bytes(const std::string& s) { return Bytes(s.begin(), s.end()); }

} // namespace

// A dual-layer image (a file system per layer, the second's descriptor right after the first's volume): both layers are
// found, CORE.GT3 and the game volumes come from the second, and two *.VOL files read as one volume set (the one with the
// larger index first; a path both have stays its).
TEST(disc_image_two_layers) {
    std::vector<u8> img(size_t(24 + 40) * kSector, 0);
    file_system(img, 0, 40, {{"SYSTEM.CNF", 19, ascii_bytes("BOOT2 = cdrom0:\\TEST_000.00;1\nVER = 1.00\n")}});
    const Bytes main = vol({{"a.bin", Bytes{1, 2, 3}}, {"same.bin", Bytes{7}}}), media = vol({{"b.bin", Bytes{4, 5}}, {"same.bin", Bytes{9}}});
    file_system(img, 24, 40,
                {{"CORE.GT3", 19, Bytes{1, 1, 0, 0, 0, 0}}, {"GTC1.VOL", 20, main}, {"GTC0.VOL", 20 + u32(main.size() / kSector), media}});
    const std::string path = test::temp_path("two_layers.iso").string();
    CHECK(write_host_file(path, ByteSpan(img.data(), img.size())));
    {
        std::unique_ptr<File> f = File::open(path);
        std::vector<std::unique_ptr<Iso9660>> layers = iso_layers(*f);
        CHECK(layers.size() == 2 && layers[0]->find("SYSTEM.CNF") && !layers[0]->find("CORE.GT3"));
        if (layers.size() == 2) CHECK(layers[1]->base_sector() == 24 && layers[1]->find("CORE.GT3") && layers[1]->find("GTC1.VOL"));
        std::unique_ptr<Disc> disc;
        try {
            disc = Disc::open(path);
        } catch (const std::exception& e) {
            std::printf("  %s\n", e.what());
        }
        CHECK(disc != nullptr);
        if (disc) {
            Volume& v = disc->volume();
            CHECK(v.entries().size() == 3 && v.find("a.bin") && v.find("b.bin") && v.find("same.bin"));
            if (v.find("a.bin") && v.find("b.bin") && v.find("same.bin")) {
                CHECK(v.read(*v.find("a.bin")) == (Bytes{1, 2, 3}) && v.read(*v.find("b.bin")) == (Bytes{4, 5}));
                CHECK(v.read(*v.find("same.bin")) == Bytes{7}); // the main volume's
            }
            CHECK(disc->info().serial == "TEST_000.00" && disc->info().problem.empty());
            CHECK(disc->read_core_gt3() == (Bytes{1, 1, 0, 0, 0, 0}));
        }
    }
    std::filesystem::remove(path);
}

// A damaged directory: a record whose name runs past its own end (and past the directory's last byte) ends the listing
// without reading beyond it, and a directory whose size runs past the image fails to read rather than allocating it.
TEST(disc_image_bad_directory_records) {
    std::vector<u8> img(size_t(24) * kSector, 0);
    file_system(img, 0, 24, {{"A.BIN", 19, Bytes{1, 2}}});
    size_t at = size_t(18) * kSector + 34 + 34 + 40; // after ".", ".." and "A.BIN;1"
    at += dir_record(img, at, "SUB", 20, 0xFFFFF000u, true);
    const size_t bad = at;
    at += dir_record(img, at, "", 19, 2, false);
    img[bad + 32] = 255; // a 255-byte name in a 34-byte record, the directory's last
    const u32 root_size = u32(at - size_t(18) * kSector);
    put32(img, 16 * kSector + 156 + 10, root_size), put32be(img, 16 * kSector + 156 + 14, root_size);
    const std::string path = (std::filesystem::temp_directory_path() / "gt3r_test_bad_records.iso").string();
    CHECK(write_host_file(path, ByteSpan(img.data(), img.size())));
    {
        std::unique_ptr<File> f = File::open(path);
        const Iso9660 iso(*f);
        const std::vector<Iso9660::Entry> root = iso.list("");
        CHECK(root.size() == 2 && root[0].name == "A.BIN" && root[1].name == "SUB");
        bool threw = false;
        try {
            iso.list("SUB");
        } catch (const FormatError&) {
            threw = true;
        }
        CHECK(threw);
    }
    std::filesystem::remove(path);
}

namespace {
Bytes gzip_member(const Bytes& data) {
    z_stream zs{};
    deflateInit2(&zs, 9, Z_DEFLATED, MAX_WBITS + 16, 8, Z_DEFAULT_STRATEGY);
    Bytes out(deflateBound(&zs, uLong(data.size())));
    zs.next_in = const_cast<Bytef*>(data.data());
    zs.avail_in = uInt(data.size());
    zs.next_out = out.data();
    zs.avail_out = uInt(out.size());
    deflate(&zs, Z_FINISH);
    out.resize(zs.total_out);
    deflateEnd(&zs);
    return out;
}

bool inflate_fails(ByteSpan gz, size_t size) {
    try {
        inflate_gzip(gz, size);
    } catch (const FormatError&) {
        return true;
    }
    return false;
}
} // namespace

// A GT3.VOL entry's unpacked size comes from the disc: a wrong one fails without allocating it (an entry saying 4 GB; a
// size no machine could allocate), and the right one reads the member whether it fits the first output guess or not
// (1 MB of one byte packs ~1000:1). A cut-off member fails too.
TEST(inflate_untrusted_size) {
    Bytes text(5000), ones(1 << 20, 0x5A), empty;
    for (size_t i = 0; i < text.size(); ++i) text[i] = u8("GT3 revisited "[i % 14] ^ (i / 97));
    for (const Bytes* data : {&text, &ones, &empty}) {
        const Bytes gz = gzip_member(*data);
        CHECK(inflate_gzip(gz, data->size()) == *data);
        CHECK(inflate_fails(gz, data->size() + 1) && inflate_fails(gz, 0xFFFFFFFFu) &&
              inflate_fails(gz, std::numeric_limits<size_t>::max() / 2));
        if (!data->empty())
            CHECK(inflate_fails(gz, data->size() - 1) && inflate_fails(ByteSpan(gz).first(gz.size() - 9), data->size()));
    }
}
