#include "TextReliefResourceAdmission.hxx"
#include <cassert>
#include <iostream>

using namespace core3d;
using namespace core3d::text_relief;

namespace {
void u(std::vector<std::uint8_t>& out, std::uint64_t value, unsigned width) {
    for (unsigned i = 0; i < width; ++i) out.push_back(std::uint8_t(value >> (8 * i)));
}
void s32(std::vector<std::uint8_t>& out, std::int32_t value) {
    std::uint32_t bits = 0; std::memcpy(&bits, &value, 4); u(out, bits, 4);
}
void point(std::vector<std::uint8_t>& out, std::int32_t x, std::int32_t y) {
    s32(out, x); s32(out, y);
}
void line(std::vector<std::uint8_t>& out, std::int32_t x, std::int32_t y) {
    u(out, 1, 1); u(out, 0, 3); point(out, x, y);
}

struct Glyph { std::uint32_t scalar, glyph; std::uint16_t contours, segments; };
std::vector<std::uint8_t> font(std::initializer_list<Glyph> glyphs) {
    std::vector<std::uint8_t> out{'S','Y','T','O',1,1,0,0};
    u(out, 1000, 4); u(out, glyphs.size(), 2); u(out, 0, 2);
    for (const Glyph value : glyphs) {
        u(out, value.scalar, 4); u(out, value.glyph, 4); s32(out, 1000);
        u(out, value.contours, 2); u(out, 0, 2);
        for (std::size_t contour = 0; contour < value.contours; ++contour) {
            u(out, value.segments, 2); u(out, 0, 2); point(out, 0, 0);
            for (std::size_t segment = 0; segment < value.segments; ++segment)
                line(out, segment % 2 == 0 ? 1000 : 0, 0);
        }
    }
    return out;
}

bounded_curve::UUID identifier(std::uint8_t seed) {
    bounded_curve::UUID value{};
    for (std::size_t i = 0; i < value.size(); ++i) value[i] = std::uint8_t(seed + i);
    return value;
}
Request request(std::vector<std::uint8_t> bytes, std::string text) {
    Request value; value.kind = ResourceKind::FontOutline;
    value.rawResourceBytes = std::move(bytes); value.textUTF8 = std::move(text);
    assert(bounded_curve::Hash(value.rawResourceBytes, MaximumResourceBytes,
                               value.expectedResourceSHA256));
    value.identitySeed = identifier(10); value.frame.identifier = identifier(40);
    value.frame.revision = 1; value.heightMM = 10; return value;
}

void testE5TextOutlineProducesByteIdenticalCanonicalC1Records() {
    const auto bytes = font({{'A', 7, 1, 4}}); const auto admitted = request(bytes, "A");
    Result first, second;
    assert(Admit(admitted, first) == Refusal::None);
    assert(Admit(admitted, second) == Refusal::None);
    assert(first.resourceSHA256 == second.resourceSHA256);
    assert(first.scalarToGlyph == std::vector<std::uint32_t>{7});
    assert(first.curves.size() == 4 && second.curves.size() == 4);
    for (std::size_t i = 0; i < first.curves.size(); ++i) {
        assert(first.curves[i].canonicalBytes == second.curves[i].canonicalBytes);
        bounded_curve::Value decoded;
        assert(bounded_curve::Decode(first.curves[i].canonicalBytes, decoded));
    }
}

void testE5TextOutlineFailsClosedAt64GlyphAnd128CurveCaps() {
    Result output;
    assert(Admit(request(font({{' ', 1, 0, 0}}), std::string(64, ' ')), output) == Refusal::None);
    assert(output.curves.empty() && output.scalarToGlyph.size() == 64);
    assert(Admit(request(font({{' ', 1, 0, 0}}), std::string(65, ' ')), output)
           == Refusal::GlyphLimit && output.curves.empty());
    assert(Admit(request(font({{'A', 2, 1, 128}}), "A"), output) == Refusal::None);
    assert(output.curves.size() == 128);
    assert(Admit(request(font({{'A', 2, 1, 130}}), "A"), output)
           == Refusal::CurveLimit && output.curves.empty());
}

void testE5TextOutlineFailsClosedAtResourceAndContourCaps() {
    Result output; auto oversized = request(font({{'A', 2, 1, 4}}), "A");
    oversized.rawResourceBytes.resize(MaximumResourceBytes + 1);
    assert(Admit(oversized, output) == Refusal::ResourceTooLarge && output.curves.empty());
    assert(Admit(request(font({{'A', 2, 33, 2}}), "A"), output)
           == Refusal::ComponentContourLimit && output.curves.empty());
    const auto many = font({{'A',1,32,2},{'B',2,32,2},{'C',3,32,2},
        {'D',4,32,2},{'E',5,32,2},{'F',6,32,2},{'G',7,32,2},
        {'H',8,32,2},{'I',9,32,2}});
    assert(Admit(request(many, "ABCDEFGHI"), output) == Refusal::ContourLimit
           && output.curves.empty());
    auto mismatch = request(font({{'A', 2, 1, 4}}), "A"); mismatch.expectedResourceSHA256[0] ^= 1;
    assert(Admit(mismatch, output) == Refusal::ResourceHashMismatch && output.curves.empty());
}
}

int main() {
    testE5TextOutlineProducesByteIdenticalCanonicalC1Records();
    testE5TextOutlineFailsClosedAt64GlyphAnd128CurveCaps();
    testE5TextOutlineFailsClosedAtResourceAndContourCaps();
    std::cout << "TextReliefResourceAdmissionTests: PASS\n";
}
