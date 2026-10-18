#include "video/ipu.h"

#include "video/field_layout.h"
#include "video/yuv.h"
#include "vfs/vfs.h"

#include <algorithm>
#include <cstring>

namespace gt3r::video {

IpuHeader IpuHeader::parse(ByteSpan d) {
    if (!has_magic(d, "ipum") || d.size() < kBytes) throw FormatError("not an IPU picture (ipum)");
    IpuHeader h;
    h.data_size = rd32(d, 4);
    h.width = rd16(d, 8);
    h.height = rd16(d, 10);
    h.frames = rd32(d, 12);
    if (h.width == 0 || h.height == 0 || h.width > 4096 || h.height > 4096) throw FormatError("IPU: bad size");
    return h;
}

Frame decode_ipum(ByteSpan d) {
    IpuHeader h = IpuHeader::parse(d);
    size_t len = std::min<size_t>(d.size() - IpuHeader::kBytes, h.data_size);
    Mpeg2Decoder dec;
    dec.decode_ipu_picture(d.data() + IpuHeader::kBytes, len, h.width, h.height);
    Frame f;
    if (!dec.pop(f)) throw FormatError("IPU: no picture decoded");
    return f;
}

std::vector<IpuArchiveEntry> parse_ipua(ByteSpan d) {
    if (!has_magic(d, "ipua")) throw FormatError("not an IPU archive (ipua)");
    u32 count = rd32(d, 4);
    if (count > 4096 || 8 + size_t(count) * 32 > d.size()) throw FormatError("ipua: bad entry count");
    std::vector<IpuArchiveEntry> out(count);
    for (u32 i = 0; i < count; ++i) {
        size_t rec = 8 + size_t(i) * 32;
        out[i].offset = rd32(d, rec);
        const char* name = reinterpret_cast<const char*>(d.data() + rec + 4);
        out[i].name.assign(name, strnlen(name, 28));
        if (out[i].offset >= d.size()) throw FormatError("ipua: entry offset past end");
    }
    for (u32 i = 0; i < count; ++i) {
        u32 end = u32(d.size());
        for (const auto& e : out)
            if (e.offset > out[i].offset) end = std::min(end, e.offset);
        out[i].size = end - out[i].offset;
    }
    return out;
}

Frame weave_fields(const Frame& f) {
    Frame o = f;
    const int half = f.height / 2, chalf = f.chroma_height() / 2, cw = f.chroma_width();
    for (int y = 0; y < half; ++y) {
        std::memcpy(&o.y[size_t(2 * y) * f.width], &f.y[size_t(y) * f.width], f.width);
        std::memcpy(&o.y[size_t(2 * y + 1) * f.width], &f.y[size_t(y + half) * f.width], f.width);
    }
    for (int y = 0; y < chalf; ++y)
        for (const auto* plane : {&f.u, &f.v}) {
            auto& dst = plane == &f.u ? o.u : o.v;
            std::memcpy(&dst[size_t(2 * y) * cw], &(*plane)[size_t(y) * cw], cw);
            std::memcpy(&dst[size_t(2 * y + 1) * cw], &(*plane)[size_t(y + chalf) * cw], cw);
        }
    return o;
}

bool CoursePreviews::load(Vfs& vfs) {
    entries_.clear();
    cache_.clear();
    auto bytes = vfs.read(kPath);
    if (!bytes) return false;
    try {
        entries_ = parse_ipua(*bytes);
    } catch (const FormatError&) {
        return false;
    }
    data_ = std::move(*bytes);
    return true;
}

const RgbaImage* CoursePreviews::image(const std::string& course, bool deinterlace) {
    std::string key = course + (deinterlace ? "#w" : "");
    if (auto it = cache_.find(key); it != cache_.end()) return &it->second;
    for (const IpuArchiveEntry& e : entries_) {
        if (e.name != course) continue;
        try {
            Frame f = decode_ipum(ByteSpan(data_).subspan(e.offset, e.size));
            if (deinterlace && seam_ratio(f) > FieldLayoutDetector::kSeamThreshold) f = weave_fields(f);
            return &(cache_[key] = RgbaImage{f.width, f.height, frame_to_rgba(f)});
        } catch (const FormatError&) {
            return nullptr;
        }
    }
    return nullptr;
}

} // namespace gt3r::video
