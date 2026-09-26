#pragma once

// E5-2 deterministic outline admission. SYTO/1 is a deliberately small,
// pre-resolved local resource format: it contains glyph or emblem outlines,
// never a platform font name, URL, XML/SVG, or provider payload. Successful
// admission returns complete canonical C1 records and no geometry.
#include "BoundedCurveCodec.hxx"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace core3d::text_relief {
inline constexpr std::size_t MaximumResourceBytes = 1'048'576;
inline constexpr std::size_t MaximumTextBytes = 256;
inline constexpr std::size_t MaximumGlyphs = 64;
inline constexpr std::size_t MaximumContours = 256;
inline constexpr std::size_t MaximumContoursPerComponent = 32;
inline constexpr std::size_t MaximumCurves = 128;
inline constexpr std::uint32_t MaximumItems = 65'535;
inline constexpr const char* ResolverVersion = "shapeyard-syto-1";

enum class ResourceKind : std::uint8_t { FontOutline = 1, EmblemVector = 2 };
enum class SegmentKind : std::uint8_t { Line = 1, Quadratic = 2, Cubic = 3 };
enum class Refusal : std::uint8_t {
    None = 0, ResourceUnavailable, ResourceTooLarge, ResourceHashMismatch,
    UnsupportedVersion, UnsupportedKind, MalformedResource, ResourceNotCanonical,
    InvalidText, GlyphLimit, GlyphUnsupported, ContourLimit,
    ComponentContourLimit, CurveLimit, InvalidContour, InvalidLayout,
    IdentityCollision, C1Refused
};

struct Point { std::int32_t x = 0, y = 0; };
inline bool operator==(const Point& a, const Point& b) noexcept {
    return a.x == b.x && a.y == b.y;
}

struct Segment {
    SegmentKind kind = SegmentKind::Line;
    Point control1{}, control2{}, end{};
};
struct Contour { Point start{}; std::vector<Segment> segments; };
struct Item {
    std::uint32_t scalar = 0;
    std::uint32_t glyphID = 0;
    std::int32_t advance = 0;
    std::vector<Contour> contours;
};
struct Resource {
    ResourceKind kind = ResourceKind::FontOutline;
    std::uint32_t unitsPerEm = 0;
    std::vector<Item> items;
};
struct Request {
    ResourceKind kind = ResourceKind::FontOutline;
    std::vector<std::uint8_t> rawResourceBytes;
    bounded_curve::Digest expectedResourceSHA256{};
    std::string textUTF8;
    bounded_curve::Frame frame;
    bounded_curve::UUID identitySeed{};
    double heightMM = 0;
    double trackingMM = 0;
};
struct CurveRecord {
    std::size_t component = 0, contour = 0, segment = 0;
    bounded_curve::Value value;
    std::vector<std::uint8_t> canonicalBytes;
};
struct ContourRecord {
    std::size_t component = 0, contour = 0, firstCurve = 0, curveCount = 0;
};
struct Result {
    bounded_curve::Digest resourceSHA256{};
    std::vector<std::uint32_t> scalarToGlyph;
    std::vector<CurveRecord> curves;
    std::vector<ContourRecord> contours;
};

class Reader final {
public:
    explicit Reader(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}
    bool integer(unsigned width, std::uint64_t& value) noexcept {
        value = 0;
        if (!ok_ || width == 0 || width > 8 || cursor_ > bytes_.size()
            || width > bytes_.size() - cursor_) { ok_ = false; return false; }
        for (unsigned i = 0; i < width; ++i)
            value |= std::uint64_t(bytes_[cursor_++]) << (8 * i);
        return true;
    }
    bool signed32(std::int32_t& value) noexcept {
        std::uint64_t raw = 0; if (!integer(4, raw)) return false;
        const std::uint32_t bits = std::uint32_t(raw); std::memcpy(&value, &bits, 4); return true;
    }
    bool prefix(const char* value, std::size_t count) noexcept {
        if (!ok_ || cursor_ > bytes_.size() || count > bytes_.size() - cursor_
            || std::memcmp(bytes_.data() + cursor_, value, count) != 0) {
            ok_ = false; return false;
        }
        cursor_ += count; return true;
    }
    std::size_t remaining() const noexcept {
        return ok_ && cursor_ <= bytes_.size() ? bytes_.size() - cursor_ : 0;
    }
    bool complete() const noexcept { return ok_ && cursor_ == bytes_.size(); }
private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t cursor_ = 0;
    bool ok_ = true;
};

inline bool ReadPoint(Reader& reader, Point& point) noexcept {
    return reader.signed32(point.x) && reader.signed32(point.y);
}

inline Refusal DecodeResource(const std::vector<std::uint8_t>& bytes,
                              Resource& output) noexcept {
    output = {};
    try {
        if (bytes.empty() || bytes.size() > MaximumResourceBytes) return Refusal::ResourceTooLarge;
        Reader reader(bytes); std::uint64_t raw = 0, reserved = 0, itemCount = 0;
        Resource staged;
        if (!reader.prefix("SYTO", 4) || !reader.integer(1, raw) || raw != 1)
            return Refusal::UnsupportedVersion;
        if (!reader.integer(1, raw) || (raw != 1 && raw != 2)) return Refusal::UnsupportedKind;
        staged.kind = ResourceKind(raw);
        if (!reader.integer(2, reserved) || reserved != 0 || !reader.integer(4, raw)
            || raw == 0 || raw > 1'000'000 || !reader.integer(2, itemCount)
            || itemCount == 0 || itemCount > MaximumItems || !reader.integer(2, reserved)
            || reserved != 0 || itemCount > reader.remaining() / 16) return Refusal::MalformedResource;
        staged.unitsPerEm = std::uint32_t(raw); staged.items.reserve(std::size_t(itemCount));
        std::uint32_t previousScalar = 0;
        std::set<std::uint32_t> glyphIDs;
        for (std::size_t itemIndex = 0; itemIndex < itemCount; ++itemIndex) {
            Item item; std::uint64_t contourCount = 0;
            if (!reader.integer(4, raw)) return Refusal::MalformedResource;
            item.scalar = std::uint32_t(raw);
            if (!reader.integer(4, raw)) return Refusal::MalformedResource;
            item.glyphID = std::uint32_t(raw);
            if (!reader.signed32(item.advance) || !reader.integer(2, contourCount)
                || !reader.integer(2, reserved) || reserved != 0)
                return Refusal::MalformedResource;
            if (contourCount > MaximumContoursPerComponent) return Refusal::ComponentContourLimit;
            if (item.glyphID == 0 || !glyphIDs.insert(item.glyphID).second
                || (staged.kind == ResourceKind::EmblemVector && (itemCount != 1 || item.scalar != 0))
                || (staged.kind == ResourceKind::FontOutline
                    && (item.scalar == 0 || item.scalar > 0x10ffff
                        || (item.scalar >= 0xd800 && item.scalar <= 0xdfff)
                        || (itemIndex != 0 && item.scalar <= previousScalar))))
                return Refusal::ResourceNotCanonical;
            previousScalar = item.scalar;
            item.contours.reserve(std::size_t(contourCount));
            for (std::size_t contourIndex = 0; contourIndex < contourCount; ++contourIndex) {
                Contour contour; std::uint64_t segmentCount = 0;
                if (!reader.integer(2, segmentCount) || !reader.integer(2, reserved)
                    || reserved != 0 || segmentCount == 0
                    || segmentCount > reader.remaining() / 12 || !ReadPoint(reader, contour.start))
                    return Refusal::MalformedResource;
                if (segmentCount > MaximumCurves) return Refusal::CurveLimit;
                contour.segments.reserve(std::size_t(segmentCount)); Point current = contour.start;
                for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
                    Segment segment;
                    if (!reader.integer(1, raw) || raw < 1 || raw > 3
                        || !reader.integer(3, reserved) || reserved != 0)
                        return Refusal::MalformedResource;
                    segment.kind = SegmentKind(raw);
                    if (segment.kind == SegmentKind::Line) {
                        if (!ReadPoint(reader, segment.end)) return Refusal::MalformedResource;
                    } else if (segment.kind == SegmentKind::Quadratic) {
                        if (!ReadPoint(reader, segment.control1) || !ReadPoint(reader, segment.end))
                            return Refusal::MalformedResource;
                    } else if (!ReadPoint(reader, segment.control1)
                               || !ReadPoint(reader, segment.control2)
                               || !ReadPoint(reader, segment.end)) return Refusal::MalformedResource;
                    if (segment.end == current) return Refusal::InvalidContour;
                    current = segment.end; contour.segments.push_back(segment);
                }
                if (!(current == contour.start)) return Refusal::InvalidContour;
                item.contours.push_back(std::move(contour));
            }
            staged.items.push_back(std::move(item));
        }
        if (!reader.complete()) return Refusal::ResourceNotCanonical;
        output = std::move(staged); return Refusal::None;
    } catch (...) { output = {}; return Refusal::MalformedResource; }
}

inline bool DecodeUTF8(const std::string& text, std::vector<std::uint32_t>& output) noexcept {
    output.clear();
    try {
        if (text.empty() || text.size() > MaximumTextBytes) return false;
        for (std::size_t i = 0; i < text.size();) {
            const auto first = std::uint8_t(text[i++]); std::uint32_t scalar = 0; unsigned trailing = 0;
            if (first <= 0x7f) scalar = first;
            else if (first >= 0xc2 && first <= 0xdf) { scalar = first & 0x1f; trailing = 1; }
            else if (first >= 0xe0 && first <= 0xef) { scalar = first & 0x0f; trailing = 2; }
            else if (first >= 0xf0 && first <= 0xf4) { scalar = first & 0x07; trailing = 3; }
            else return false;
            if (trailing > text.size() - i) return false;
            for (unsigned j = 0; j < trailing; ++j) {
                const auto next = std::uint8_t(text[i++]); if ((next & 0xc0) != 0x80) return false;
                scalar = (scalar << 6) | (next & 0x3f);
            }
            if ((trailing == 1 && scalar < 0x80) || (trailing == 2 && scalar < 0x800)
                || (trailing == 3 && scalar < 0x10000) || scalar > 0x10ffff
                || (scalar >= 0xd800 && scalar <= 0xdfff)
                || scalar < 0x20 || (scalar >= 0x7f && scalar <= 0x9f)) return false;
            if (output.size() == MaximumGlyphs) return false;
            output.push_back(scalar);
        }
        return !output.empty();
    } catch (...) { output.clear(); return false; }
}

inline void Append64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
    for (unsigned i = 0; i < 8; ++i) bytes.push_back(std::uint8_t(value >> (8 * i)));
}
inline bounded_curve::UUID DerivedID(const bounded_curve::UUID& seed,
                                     const bounded_curve::Digest& resourceDigest,
                                     std::size_t component, std::size_t contour,
                                     std::size_t segment, std::size_t point,
                                     std::uint8_t role) {
    std::vector<std::uint8_t> material; material.reserve(16 + 32 + 41);
    material.insert(material.end(), seed.begin(), seed.end());
    material.insert(material.end(), resourceDigest.begin(), resourceDigest.end());
    Append64(material, component); Append64(material, contour); Append64(material, segment);
    Append64(material, point); material.push_back(role);
    bounded_curve::Digest digest{}; bounded_curve::Hash(material, 128, digest);
    bounded_curve::UUID result{}; std::copy_n(digest.begin(), result.size(), result.begin());
    result[6] = std::uint8_t((result[6] & 0x0f) | 0x50);
    result[8] = std::uint8_t((result[8] & 0x3f) | 0x80); return result;
}

inline Refusal Admit(const Request& request, Result& output) noexcept {
    output = {};
    try {
        if (request.rawResourceBytes.empty()) return Refusal::ResourceUnavailable;
        if (request.rawResourceBytes.size() > MaximumResourceBytes) return Refusal::ResourceTooLarge;
        bounded_curve::Digest digest{};
        if (!bounded_curve::Hash(request.rawResourceBytes, MaximumResourceBytes, digest)
            || digest != request.expectedResourceSHA256) return Refusal::ResourceHashMismatch;
        if (!bounded_curve::Nonzero(request.identitySeed)
            || bounded_curve::ValidateFrame(request.frame) != bounded_curve::Refusal::None
            || !std::isfinite(request.heightMM) || request.heightMM < 0.1 || request.heightMM > 100'000
            || !std::isfinite(request.trackingMM) || request.trackingMM < -10'000
            || request.trackingMM > 10'000) return Refusal::InvalidLayout;
        Resource resource; const Refusal decoded = DecodeResource(request.rawResourceBytes, resource);
        if (decoded != Refusal::None) return decoded;
        if (resource.kind != request.kind) return Refusal::UnsupportedKind;
        std::vector<std::uint32_t> scalars;
        if (request.kind == ResourceKind::FontOutline) {
            if (!DecodeUTF8(request.textUTF8, scalars))
                return request.textUTF8.size() > MaximumTextBytes
                    || scalars.size() == MaximumGlyphs ? Refusal::GlyphLimit : Refusal::InvalidText;
            if (scalars.size() > MaximumGlyphs) return Refusal::GlyphLimit;
        } else {
            if (!request.textUTF8.empty()) return Refusal::InvalidText;
            scalars.push_back(0);
        }
        std::vector<const Item*> items; items.reserve(scalars.size());
        std::size_t contours = 0, curves = 0;
        for (const std::uint32_t scalar : scalars) {
            const auto found = std::lower_bound(resource.items.begin(), resource.items.end(), scalar,
                [](const Item& item, std::uint32_t value) { return item.scalar < value; });
            if (found == resource.items.end() || found->scalar != scalar) return Refusal::GlyphUnsupported;
            if (found->contours.size() > MaximumContoursPerComponent) return Refusal::ComponentContourLimit;
            contours += found->contours.size();
            for (const Contour& contour : found->contours) {
                curves += contour.segments.size();
            }
            items.push_back(&*found);
        }
        if (contours > MaximumContours) return Refusal::ContourLimit;
        if (curves > MaximumCurves) return Refusal::CurveLimit;
        Result staged; staged.resourceSHA256 = digest;
        staged.scalarToGlyph.reserve(items.size());
        for (const Item* item : items) staged.scalarToGlyph.push_back(item->glyphID);
        staged.curves.reserve(curves); staged.contours.reserve(contours);
        const double scale = request.heightMM / double(resource.unitsPerEm);
        if (!std::isfinite(scale) || scale <= 0) return Refusal::InvalidLayout;
        double cursor = 0; std::set<bounded_curve::UUID> identifiers;
        for (std::size_t component = 0; component < items.size(); ++component) {
            const Item& item = *items[component];
            for (std::size_t contourIndex = 0; contourIndex < item.contours.size(); ++contourIndex) {
                const Contour& contour = item.contours[contourIndex];
                ContourRecord contourRecord{component, contourIndex, staged.curves.size(), contour.segments.size()};
                Point current = contour.start;
                for (std::size_t segmentIndex = 0; segmentIndex < contour.segments.size(); ++segmentIndex) {
                    const Segment& segment = contour.segments[segmentIndex];
                    auto local = [&](const Point& point) {
                        return std::array<double, 3>{{cursor + double(point.x) * scale,
                            double(point.y) * scale, 0}};
                    };
                    std::vector<std::array<double, 3>> poles;
                    if (segment.kind == SegmentKind::Line) poles = {local(current), local(segment.end)};
                    else if (segment.kind == SegmentKind::Quadratic) {
                        const auto start = local(current), control = local(segment.control1), end = local(segment.end);
                        std::array<double, 3> first{}, second{};
                        for (int axis = 0; axis < 3; ++axis) {
                            first[axis] = start[axis] + (control[axis] - start[axis]) * (2.0 / 3.0);
                            second[axis] = end[axis] + (control[axis] - end[axis]) * (2.0 / 3.0);
                        }
                        poles = {start, first, second, end};
                    } else poles = {local(current), local(segment.control1),
                                    local(segment.control2), local(segment.end)};
                    bounded_curve::Value value;
                    value.feature = DerivedID(request.identitySeed, digest, component,
                                              contourIndex, segmentIndex, 0, 1);
                    value.definition.domain = bounded_curve::Domain::Sketch2D;
                    value.definition.frame = request.frame;
                    value.definition.degree = std::uint8_t(poles.size() == 2 ? 1 : 3);
                    for (std::size_t point = 0; point < poles.size(); ++point) {
                        bounded_curve::ControlPoint control;
                        control.identifier = DerivedID(request.identitySeed, digest, component,
                                                       contourIndex, segmentIndex, point, 2);
                        control.local = poles[point]; value.definition.controlPoints.push_back(control);
                        if (!identifiers.insert(control.identifier).second) return Refusal::IdentityCollision;
                    }
                    if (!identifiers.insert(value.feature).second) return Refusal::IdentityCollision;
                    const auto multiplicity = std::uint8_t(value.definition.degree + 1);
                    value.definition.knots = {{0, multiplicity}, {1, multiplicity}};
                    std::vector<std::uint8_t> bytes;
                    if (!bounded_curve::Encode(value, bytes)) return Refusal::C1Refused;
                    staged.curves.push_back({component, contourIndex, segmentIndex,
                                             std::move(value), std::move(bytes)});
                    current = segment.end;
                }
                staged.contours.push_back(contourRecord);
            }
            const double advance = double(item.advance) * scale;
            if (!std::isfinite(advance) || !std::isfinite(cursor + advance + request.trackingMM))
                return Refusal::InvalidLayout;
            cursor += advance;
            if (component + 1 != items.size()) cursor += request.trackingMM;
            if (!std::isfinite(cursor) || std::abs(cursor) > bounded_curve::CoordinateLimit)
                return Refusal::InvalidLayout;
        }
        output = std::move(staged); return Refusal::None;
    } catch (...) { output = {}; return Refusal::MalformedResource; }
}
} // namespace core3d::text_relief
