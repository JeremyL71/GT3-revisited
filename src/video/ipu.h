#pragma once

// PS2 IPU pictures and the "ipua" archive of course preview stills
// (stream/course.ipic). See docs/formats/pss.md.
//
// ipum (one picture file, as produced by Sony's tools):
//   0x00 "ipum"  0x04 u32 data size  0x08 u16 width  0x0A u16 height
//   0x0C u32 frame count  0x10 IPU bitstream
// IPU bitstream: u8 flags (bits 0-1 intra_dc_precision, 0x04 dct_type bits
// present, 0x10 alternate scan, 0x20 intra_vlc_format, 0x40 q_scale_type,
// 0x80 MPEG-1 style blocks), then raster-order MPEG-2 intra macroblocks with
// no slices: [address increment '1' except for the first] type '1' (intra) or
// '01' (intra + 5-bit quantiser_scale_code), [dct_type], six intra blocks.
// DC predictors run across the whole picture; default intra matrix.
//
// ipua archive:
//   0x00 "ipua"  0x04 u32 count  0x08 count x { u32 offset; char name[28] }
// Each offset points at an ipum; entries are 64-byte aligned.

#include "video/mpeg2_decoder.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gt3r {
class Vfs;
}

namespace gt3r::video {

struct IpuHeader {
    u32 data_size = 0;
    int width = 0, height = 0;
    u32 frames = 0;
    static constexpr size_t kBytes = 16;
    static IpuHeader parse(ByteSpan d); // throws FormatError
};

// Decodes the first picture of an ipum file. Throws FormatError.
Frame decode_ipum(ByteSpan ipum);

struct IpuArchiveEntry {
    std::string name;
    u32 offset = 0, size = 0; // size: up to the next entry / end of file
};
std::vector<IpuArchiveEntry> parse_ipua(ByteSpan data); // throws FormatError

struct RgbaImage {
    int width = 0, height = 0;
    std::vector<u32> rgba; // R,G,B,A in memory order
};

// Course preview stills from stream/course.ipic, looked up by course name
// ("akasaka", "r_route5_dry", ...). Decoded on demand and cached.
class CoursePreviews {
public:
    static constexpr const char* kPath = "stream/course.ipic";

    bool load(Vfs& vfs); // false if the archive is missing or malformed
    const std::vector<IpuArchiveEntry>& entries() const { return entries_; }
    // nullptr if unknown or undecodable. Field-split stills are returned as
    // stored (two 640x224 fields stacked) unless `deinterlace` is set, which
    // weaves the two halves into one full-height frame.
    const RgbaImage* image(const std::string& course, bool deinterlace = true);

private:
    Bytes data_;
    std::vector<IpuArchiveEntry> entries_;
    std::map<std::string, RgbaImage> cache_;
};

// Interleaves the two stacked fields of a field-split frame into one
// full-height progressive frame (top half -> even lines, bottom -> odd).
Frame weave_fields(const Frame& f);

} // namespace gt3r::video
