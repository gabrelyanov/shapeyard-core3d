#pragma once
// Isolated D1 prerequisite; not registered with OCAF and not an authority API.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <istream>
#include <limits>
#include <functional>
#include <streambuf>

namespace core3d::persistence::receipt_framing {

// Limits must be supplied by the shared load policy, not reconstructed per record.
struct LoadBudget {
    std::uint64_t maximumWireBytes = 0;
    std::uint64_t chargedWireBytes = 0;
    bool seenReceipt = false;
    bool rejected = false;
    std::function<bool()> continueReading;
    bool refuse() noexcept { rejected = true; return false; }
    bool more() noexcept {
        if (rejected) return false;
        try { if (continueReading && !continueReading()) return refuse(); }
        catch (...) { return refuse(); }
        return !rejected;
    }
};

inline std::uint32_t Decode32(const unsigned char* p, bool inverse) noexcept {
    std::uint32_t n = 0; std::memcpy(&n, p, sizeof n);
    if (inverse) n = ((n & 0xffU) << 24) | ((n & 0xff00U) << 8) |
                     ((n & 0xff0000U) >> 8) | ((n & 0xff000000U) >> 24);
    return n;
}
inline std::uint64_t Decode64(const unsigned char* p, bool inverse) noexcept {
    std::uint64_t n = 0; std::memcpy(&n, p, sizeof n);
    if (inverse) {
        n = ((n & UINT64_C(0x00ff00ff00ff00ff)) << 8) |
            ((n >> 8) & UINT64_C(0x00ff00ff00ff00ff));
        n = ((n & UINT64_C(0x0000ffff0000ffff)) << 16) |
            ((n >> 16) & UINT64_C(0x0000ffff0000ffff));
        n = (n << 32) | (n >> 32);
    }
    return n;
}
inline std::int32_t Signed32(std::uint32_t bits) noexcept {
    std::int32_t n = 0; std::memcpy(&n, &bits, sizeof n); return n;
}

// Captured once by the future ReadSubTree header reader, before BinObjMgt::Read.
// Receipt's zero buffered prefix is deliberate: all V3 grammar is in its frame.
class ValidatedHeader final {
public:
    static bool Parse(const std::array<unsigned char, 12>& raw,
                      std::int32_t expectedType, bool inverse,
                      LoadBudget& budget, ValidatedHeader& out) noexcept {
        if (budget.rejected || expectedType <= 0 ||
            Signed32(Decode32(raw.data(), inverse)) != expectedType)
            return budget.refuse();
        const auto id = Signed32(Decode32(raw.data() + 4, inverse));
        const auto length = Signed32(Decode32(raw.data() + 8, inverse));
        if (id >= 0 || id == std::numeric_limits<std::int32_t>::min() || length != 0)
            return budget.refuse();
        out.raw_ = raw; out.typeID_ = expectedType; out.objectID_ = -id; out.valid_ = true;
        return true;
    }
    bool valid() const noexcept { return valid_; }
    std::int32_t typeID() const noexcept { return typeID_; }
    std::int32_t objectID() const noexcept { return objectID_; }
    const std::array<unsigned char, 12>& bytes() const noexcept { return raw_; }
private:
    std::array<unsigned char, 12> raw_{};
    std::int32_t objectID_ = 0;
    std::int32_t typeID_ = 0;
    bool valid_ = false;
};

// Stack-only view. The eight already-read extent bytes are replayed from cache;
// payload bytes are consumed once from the original stream. Never wraps OCCT's
// quick-part shape reader, which legitimately uses absolute shared references.
class DirectBuffer final : public std::streambuf {
public:
    DirectBuffer(std::istream& original, const std::array<unsigned char, 8>& size,
                 std::uint64_t extent, LoadBudget& budget) noexcept
        : original_(original), size_(size), extent_(extent), budget_(budget) {}
    bool complete() const noexcept { return !budget_.rejected && position_ == extent_; }
    std::uint64_t position() const noexcept { return position_; }
protected:
    std::streamsize xsgetn(char* destination, std::streamsize count) override {
        if (!budget_.more() || count < 0) { budget_.refuse(); return 0; }
        const auto wanted = static_cast<std::uint64_t>(count);
        if (wanted > extent_ - position_) { budget_.refuse(); return 0; }
        std::streamsize done = 0;
        while (done < count && position_ < size_.size()) {
            destination[done++] = static_cast<char>(size_[static_cast<std::size_t>(position_++)]);
        }
        // Fixed blocks also avoid count conversions overflowing the source API.
        while (done < count) {
            if (!budget_.more()) break;
            const auto n = std::min<std::streamsize>(count - done, 4096);
            original_.read(destination + done, n);
            const auto got = original_.gcount();
            if (got > 0) { done += got; position_ += static_cast<std::uint64_t>(got); }
            if (got != n || !original_) { budget_.refuse(); break; }
        }
        return done;
    }
    int_type underflow() override {
        if (!budget_.more() || position_ >= extent_) {
            budget_.refuse(); return traits_type::eof();
        }
        if (position_ < size_.size())
            return traits_type::to_int_type(static_cast<char>(size_[static_cast<std::size_t>(position_)]));
        const auto c = original_.peek();
        if (traits_type::eq_int_type(c, traits_type::eof())) budget_.refuse();
        return c;
    }
    int_type uflow() override {
        char c = 0;
        return xsgetn(&c, 1) == 1 ? traits_type::to_int_type(c) : traits_type::eof();
    }
    std::streamsize showmanyc() override {
        if (budget_.rejected) return -1;
        const auto n = extent_ - position_;
        const auto maximum = static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max());
        return static_cast<std::streamsize>(std::min(n, maximum));
    }
    pos_type seekoff(off_type offset, std::ios_base::seekdir direction,
                     std::ios_base::openmode mode) override {
        if (budget_.rejected || mode != std::ios_base::in ||
            direction != std::ios_base::cur || offset < 0 ||
            static_cast<std::uint64_t>(offset) > extent_ - position_) {
            budget_.refuse(); return pos_type(off_type(-1));
        }
        // GetIStream's seekg(+8,cur) consumes only the cached extent prefix.
        // Other forward seeks are bounded consumes, never raw source seeks.
        std::array<char, 256> scratch{};
        auto remaining = static_cast<std::uint64_t>(offset);
        while (remaining != 0) {
            const auto n = static_cast<std::streamsize>(std::min<std::uint64_t>(remaining, scratch.size()));
            if (xsgetn(scratch.data(), n) != n) return pos_type(off_type(-1));
            remaining -= static_cast<std::uint64_t>(n);
        }
        return pos_type(static_cast<off_type>(position_));
    }
    pos_type seekpos(pos_type, std::ios_base::openmode) override {
        budget_.refuse(); return pos_type(off_type(-1));
    }
    int_type pbackfail(int_type = traits_type::eof()) override {
        budget_.refuse(); return traits_type::eof();
    }
private:
    std::istream& original_;
    const std::array<unsigned char, 8> size_;
    const std::uint64_t extent_;
    LoadBudget& budget_;
    std::uint64_t position_ = 0;
};

// Caller supplies the actual remaining bytes in both the current label and file
// after the cached 12-byte header, before the direct uint64 size field. The
// unimplemented traversal must derive these from checked positions/label frames.
// No body-count allocation occurs here; the future tree decoder charges memory.
template<class Decoder>
bool ReadDirectFrame(std::istream& original, const ValidatedHeader& header,
                     bool inverse, std::uint64_t enclosingRemaining,
                     LoadBudget& budget, Decoder&& decode) {
    if (!header.valid() || budget.rejected || budget.seenReceipt || enclosingRemaining < 8)
        return budget.refuse();
    std::array<unsigned char, 8> rawSize{};
    try {
        original.read(reinterpret_cast<char*>(rawSize.data()), rawSize.size());
        if (!original || original.gcount() != static_cast<std::streamsize>(rawSize.size()))
            return budget.refuse();
        const auto extent = Decode64(rawSize.data(), inverse);
        // An off_type is used only for safe bounded tell/forward positions.
        const auto maxOffset = static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max());
        if (extent < 8 || extent > enclosingRemaining || extent > maxOffset ||
            budget.chargedWireBytes > budget.maximumWireBytes ||
            budget.maximumWireBytes - budget.chargedWireBytes < 12 ||
            extent > budget.maximumWireBytes - budget.chargedWireBytes - 12)
            return budget.refuse();
        budget.chargedWireBytes += extent + 12; // cached header plus inclusive direct extent
        budget.seenReceipt = true; // before callback; failure never resets this load.
        DirectBuffer buffer(original, rawSize, extent, budget);
        std::istream view(&buffer);
        if (!decode(header, view, extent) || !view || !buffer.complete())
            return budget.refuse();
        return true;
    } catch (...) {
        budget.refuse();
        return false;
    }
}
} // namespace core3d::persistence::receipt_framing
