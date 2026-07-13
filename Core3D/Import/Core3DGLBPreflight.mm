//
//  Core3DGLBPreflight.mm
//  Core3D
//

#import <Foundation/Foundation.h>

#include "Core3DGLBPreflight.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <new>
#include <string>
#include <sys/stat.h>
#include <unordered_set>
#include <unistd.h>
#include <utility>
#include <vector>

namespace core3d::gltf {
namespace {

constexpr std::uint64_t kMaximumSourceBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumJSONBytes = 4ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumJSONDepth = 64;
constexpr std::size_t kMaximumJSONStringBytes = 256ULL * 1024ULL;
constexpr std::size_t kMaximumJSONNumberBytes = 128;
constexpr std::uint64_t kMaximumJSONStructuralNodes = 250'000;
constexpr std::uint64_t kMaximumObjects = 2'048;
constexpr std::uint64_t kMaximumVertices = 1'500'000;
constexpr std::uint64_t kMaximumIndices = 4'500'000;
constexpr std::uint64_t kMaximumDefinitionRecords = 100'000;
constexpr std::uint64_t kMaximumMaterials = 2'048;
constexpr std::uint64_t kGLBMagic = 0x46546C67U;
constexpr std::uint32_t kJSONChunkType = 0x4E4F534AU;
constexpr std::uint32_t kBINChunkType = 0x004E4942U;

bool IsCancelled(const std::atomic_bool *cancelled) noexcept {
    return cancelled != nullptr
        && cancelled->load(std::memory_order_acquire);
}

bool AddWouldOverflow(
    const std::uint64_t lhs,
    const std::uint64_t rhs) noexcept {
    return rhs > std::numeric_limits<std::uint64_t>::max() - lhs;
}

bool MultiplyWouldOverflow(
    const std::uint64_t lhs,
    const std::uint64_t rhs) noexcept {
    return lhs != 0
        && rhs > std::numeric_limits<std::uint64_t>::max() / lhs;
}

std::uint32_t ReadLE32(const std::uint8_t *bytes) noexcept {
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

bool PreadExactly(
    const int descriptor,
    void *destination,
    const std::size_t length,
    const std::uint64_t offset,
    const std::atomic_bool *cancelled,
    PreflightResult& result) {
    if (length == 0) {
        return true;
    }
    if (offset > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())
        || length > static_cast<std::size_t>(
            std::numeric_limits<off_t>::max())) {
        result.status = PreflightStatus::IOFailure;
        result.message = "GLB read range cannot be represented by pread.";
        return false;
    }

    auto *output = static_cast<std::uint8_t *>(destination);
    std::size_t completed = 0;
    while (completed < length) {
        if (IsCancelled(cancelled)) {
            result.status = PreflightStatus::Cancelled;
            result.message = "GLB preflight was cancelled.";
            return false;
        }
        constexpr std::size_t kReadQuantum = 64ULL * 1024ULL;
        const std::size_t requested = std::min(
            length - completed,
            kReadQuantum);
        const std::uint64_t readOffset = offset + completed;
        const ssize_t count = pread(
            descriptor,
            output + completed,
            requested,
            static_cast<off_t>(readOffset));
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            result.status = PreflightStatus::IOFailure;
            result.message = "Could not read the pinned GLB descriptor.";
            return false;
        }
        if (count == 0) {
            result.status = PreflightStatus::IOFailure;
            result.message = "The pinned GLB ended before its declared length.";
            return false;
        }
        completed += static_cast<std::size_t>(count);
    }
    return true;
}

bool SamePinnedFile(const struct stat& before, const struct stat& after) {
    if (before.st_dev != after.st_dev
        || before.st_ino != after.st_ino
        || before.st_size != after.st_size) {
        return false;
    }
#if defined(__APPLE__)
    return before.st_mtimespec.tv_sec == after.st_mtimespec.tv_sec
        && before.st_mtimespec.tv_nsec == after.st_mtimespec.tv_nsec
        && before.st_ctimespec.tv_sec == after.st_ctimespec.tv_sec
        && before.st_ctimespec.tv_nsec == after.st_ctimespec.tv_nsec;
#else
    return before.st_mtim.tv_sec == after.st_mtim.tv_sec
        && before.st_mtim.tv_nsec == after.st_mtim.tv_nsec
        && before.st_ctim.tv_sec == after.st_ctim.tv_sec
        && before.st_ctim.tv_nsec == after.st_ctim.tv_nsec;
#endif
}

bool IsHexDigit(const std::uint8_t value) noexcept {
    return (value >= '0' && value <= '9')
        || (value >= 'a' && value <= 'f')
        || (value >= 'A' && value <= 'F');
}

std::uint16_t HexQuad(const std::uint8_t *bytes) noexcept {
    std::uint16_t value = 0;
    for (std::size_t index = 0; index < 4; ++index) {
        const std::uint8_t digit = bytes[index];
        value = static_cast<std::uint16_t>(value << 4U);
        if (digit >= '0' && digit <= '9') {
            value = static_cast<std::uint16_t>(value + digit - '0');
        } else if (digit >= 'a' && digit <= 'f') {
            value = static_cast<std::uint16_t>(value + digit - 'a' + 10U);
        } else {
            value = static_cast<std::uint16_t>(value + digit - 'A' + 10U);
        }
    }
    return value;
}

bool ConsumeUTF8Scalar(
    const std::uint8_t *bytes,
    const std::size_t length,
    std::size_t& cursor) noexcept {
    const std::uint8_t first = bytes[cursor];
    if (first < 0x80U) {
        ++cursor;
        return first >= 0x20U;
    }

    std::size_t count = 0;
    std::uint32_t scalar = 0;
    std::uint32_t minimum = 0;
    if (first >= 0xC2U && first <= 0xDFU) {
        count = 2;
        scalar = first & 0x1FU;
        minimum = 0x80U;
    } else if (first >= 0xE0U && first <= 0xEFU) {
        count = 3;
        scalar = first & 0x0FU;
        minimum = 0x800U;
    } else if (first >= 0xF0U && first <= 0xF4U) {
        count = 4;
        scalar = first & 0x07U;
        minimum = 0x10000U;
    } else {
        return false;
    }
    if (cursor + count > length) {
        return false;
    }
    for (std::size_t index = 1; index < count; ++index) {
        const std::uint8_t continuation = bytes[cursor + index];
        if ((continuation & 0xC0U) != 0x80U) {
            return false;
        }
        scalar = (scalar << 6U) | (continuation & 0x3FU);
    }
    if (scalar < minimum
        || scalar > 0x10FFFFU
        || (scalar >= 0xD800U && scalar <= 0xDFFFU)) {
        return false;
    }
    cursor += count;
    return true;
}

bool LexicallyValidateJSON(
    const std::vector<std::uint8_t>& bytes,
    const std::atomic_bool *cancelled,
    PreflightResult& result) {
    std::vector<std::uint8_t> containers;
    containers.reserve(kMaximumJSONDepth);
    std::uint64_t structuralNodes = 0;
    std::size_t cursor = 0;

    const auto countNode = [&]() -> bool {
        ++structuralNodes;
        if (structuralNodes > kMaximumJSONStructuralNodes) {
            result.status = PreflightStatus::ResourceLimit;
            result.message = "GLB JSON exceeds the structural-node budget.";
            return false;
        }
        return true;
    };

    while (cursor < bytes.size()) {
        if ((cursor & 0x3FFFU) == 0 && IsCancelled(cancelled)) {
            result.status = PreflightStatus::Cancelled;
            result.message = "GLB preflight was cancelled.";
            return false;
        }
        const std::uint8_t current = bytes[cursor];
        if (current == ' ' || current == '\t'
            || current == '\n' || current == '\r') {
            ++cursor;
            continue;
        }
        if (current == '{' || current == '[') {
            if (!countNode()) {
                return false;
            }
            containers.push_back(current);
            if (containers.size() > kMaximumJSONDepth) {
                result.status = PreflightStatus::ResourceLimit;
                result.message = "GLB JSON exceeds the nesting-depth budget.";
                return false;
            }
            ++cursor;
            continue;
        }
        if (current == '}' || current == ']') {
            const std::uint8_t expected = current == '}' ? '{' : '[';
            if (containers.empty() || containers.back() != expected) {
                result.status = PreflightStatus::Invalid;
                result.message = "GLB JSON contains mismatched containers.";
                return false;
            }
            containers.pop_back();
            ++cursor;
            continue;
        }
        if (current == ':' || current == ',') {
            ++cursor;
            continue;
        }
        if (current == '"') {
            if (!countNode()) {
                return false;
            }
            const std::size_t tokenStart = cursor++;
            bool closed = false;
            while (cursor < bytes.size()) {
                if (cursor - tokenStart > kMaximumJSONStringBytes) {
                    result.status = PreflightStatus::ResourceLimit;
                    result.message = "GLB JSON contains an oversized string token.";
                    return false;
                }
                const std::uint8_t character = bytes[cursor];
                if (character == '"') {
                    ++cursor;
                    closed = true;
                    break;
                }
                if (character == '\\') {
                    ++cursor;
                    if (cursor >= bytes.size()) {
                        break;
                    }
                    const std::uint8_t escaped = bytes[cursor++];
                    if (escaped == '"' || escaped == '\\' || escaped == '/'
                        || escaped == 'b' || escaped == 'f'
                        || escaped == 'n' || escaped == 'r'
                        || escaped == 't') {
                        continue;
                    }
                    if (escaped != 'u' || cursor + 4 > bytes.size()
                        || !IsHexDigit(bytes[cursor])
                        || !IsHexDigit(bytes[cursor + 1])
                        || !IsHexDigit(bytes[cursor + 2])
                        || !IsHexDigit(bytes[cursor + 3])) {
                        result.status = PreflightStatus::Invalid;
                        result.message = "GLB JSON contains an invalid string escape.";
                        return false;
                    }
                    const std::uint16_t firstUnit = HexQuad(bytes.data() + cursor);
                    cursor += 4;
                    if (firstUnit >= 0xD800U && firstUnit <= 0xDBFFU) {
                        if (cursor + 6 > bytes.size()
                            || bytes[cursor] != '\\'
                            || bytes[cursor + 1] != 'u'
                            || !IsHexDigit(bytes[cursor + 2])
                            || !IsHexDigit(bytes[cursor + 3])
                            || !IsHexDigit(bytes[cursor + 4])
                            || !IsHexDigit(bytes[cursor + 5])) {
                            result.status = PreflightStatus::Invalid;
                            result.message = "GLB JSON contains an unpaired surrogate escape.";
                            return false;
                        }
                        const std::uint16_t secondUnit = HexQuad(
                            bytes.data() + cursor + 2);
                        if (secondUnit < 0xDC00U || secondUnit > 0xDFFFU) {
                            result.status = PreflightStatus::Invalid;
                            result.message = "GLB JSON contains an unpaired surrogate escape.";
                            return false;
                        }
                        cursor += 6;
                    } else if (firstUnit >= 0xDC00U && firstUnit <= 0xDFFFU) {
                        result.status = PreflightStatus::Invalid;
                        result.message = "GLB JSON contains an unpaired surrogate escape.";
                        return false;
                    }
                    continue;
                }
                if (!ConsumeUTF8Scalar(bytes.data(), bytes.size(), cursor)) {
                    result.status = PreflightStatus::Invalid;
                    result.message = "GLB JSON is not strict UTF-8.";
                    return false;
                }
            }
            if (!closed) {
                result.status = PreflightStatus::Invalid;
                result.message = "GLB JSON contains an unterminated string.";
                return false;
            }
            continue;
        }
        if (current == '-' || (current >= '0' && current <= '9')) {
            if (!countNode()) {
                return false;
            }
            const std::size_t tokenStart = cursor;
            while (cursor < bytes.size()) {
                const std::uint8_t character = bytes[cursor];
                if ((character >= '0' && character <= '9')
                    || character == '-' || character == '+'
                    || character == '.' || character == 'e'
                    || character == 'E') {
                    ++cursor;
                } else {
                    break;
                }
                if (cursor - tokenStart > kMaximumJSONNumberBytes) {
                    result.status = PreflightStatus::ResourceLimit;
                    result.message = "GLB JSON contains an oversized number token.";
                    return false;
                }
            }
            continue;
        }
        const auto hasLiteral = [&](const char *literal, const std::size_t count) {
            return cursor + count <= bytes.size()
                && std::memcmp(bytes.data() + cursor, literal, count) == 0;
        };
        if (hasLiteral("true", 4)) {
            if (!countNode()) {
                return false;
            }
            cursor += 4;
            continue;
        }
        if (hasLiteral("false", 5)) {
            if (!countNode()) {
                return false;
            }
            cursor += 5;
            continue;
        }
        if (hasLiteral("null", 4)) {
            if (!countNode()) {
                return false;
            }
            cursor += 4;
            continue;
        }
        result.status = PreflightStatus::Invalid;
        result.message = "GLB JSON contains an invalid lexical token.";
        return false;
    }
    if (!containers.empty()) {
        result.status = PreflightStatus::Invalid;
        result.message = "GLB JSON contains an unterminated container.";
        return false;
    }
    return true;
}

bool DecodeUTF8Scalar(
    const std::uint8_t *bytes,
    const std::size_t length,
    std::size_t& cursor,
    char32_t& output) noexcept {
    const std::size_t start = cursor;
    if (!ConsumeUTF8Scalar(bytes, length, cursor)) {
        return false;
    }
    const std::uint8_t first = bytes[start];
    if (first < 0x80U) {
        output = first;
        return true;
    }
    const std::size_t count = cursor - start;
    std::uint32_t scalar = count == 2
        ? first & 0x1FU
        : (count == 3 ? first & 0x0FU : first & 0x07U);
    for (std::size_t index = 1; index < count; ++index) {
        scalar = (scalar << 6U) | (bytes[start + index] & 0x3FU);
    }
    output = static_cast<char32_t>(scalar);
    return true;
}

class JSONShapeValidator final {
public:
    JSONShapeValidator(
        const std::vector<std::uint8_t>& bytes,
        const std::atomic_bool *cancelled,
        PreflightResult& result)
    : myBytes(bytes), myCancelled(cancelled), myResult(result) {
    }

    bool Validate() {
        SkipWhitespace();
        if (!ParseValue(0)) {
            return false;
        }
        SkipWhitespace();
        if (myCursor != myBytes.size()) {
            return Fail("GLB JSON contains more than one top-level value.");
        }
        return true;
    }

private:
    bool Fail(const char *message) {
        myResult.status = PreflightStatus::Invalid;
        myResult.message = message;
        return false;
    }

    bool CheckCancelled() {
        if (!IsCancelled(myCancelled)) {
            return true;
        }
        myResult.status = PreflightStatus::Cancelled;
        myResult.message = "GLB preflight was cancelled.";
        return false;
    }

    void SkipWhitespace() {
        while (myCursor < myBytes.size()) {
            const std::uint8_t value = myBytes[myCursor];
            if (value != ' ' && value != '\t' && value != '\n' && value != '\r') {
                break;
            }
            ++myCursor;
        }
    }

    bool ParseValue(const std::size_t depth) {
        if (!CheckCancelled() || depth > kMaximumJSONDepth) {
            return false;
        }
        SkipWhitespace();
        if (myCursor >= myBytes.size()) {
            return Fail("GLB JSON value is truncated.");
        }
        const std::uint8_t value = myBytes[myCursor];
        if (value == '{') {
            return ParseObject(depth + 1);
        }
        if (value == '[') {
            return ParseArray(depth + 1);
        }
        if (value == '"') {
            return ParseString(nullptr);
        }
        if (value == '-' || (value >= '0' && value <= '9')) {
            return ParseNumber();
        }
        if (ConsumeLiteral("true", 4)
            || ConsumeLiteral("false", 5)
            || ConsumeLiteral("null", 4)) {
            return true;
        }
        return Fail("GLB JSON value syntax is invalid.");
    }

    bool ParseObject(const std::size_t depth) {
        ++myCursor;
        SkipWhitespace();
        if (myCursor < myBytes.size() && myBytes[myCursor] == '}') {
            ++myCursor;
            return true;
        }
        std::unordered_set<std::u32string> keys;
        while (myCursor < myBytes.size()) {
            if (!CheckCancelled()) {
                return false;
            }
            std::u32string key;
            if (!ParseString(&key)) {
                return false;
            }
            if (!keys.insert(std::move(key)).second) {
                return Fail("GLB JSON contains a duplicate object member name.");
            }
            SkipWhitespace();
            if (myCursor >= myBytes.size() || myBytes[myCursor] != ':') {
                return Fail("GLB JSON object member is missing a colon.");
            }
            ++myCursor;
            if (!ParseValue(depth)) {
                return false;
            }
            SkipWhitespace();
            if (myCursor < myBytes.size() && myBytes[myCursor] == '}') {
                ++myCursor;
                return true;
            }
            if (myCursor >= myBytes.size() || myBytes[myCursor] != ',') {
                return Fail("GLB JSON object syntax is invalid.");
            }
            ++myCursor;
            SkipWhitespace();
        }
        return Fail("GLB JSON object is truncated.");
    }

    bool ParseArray(const std::size_t depth) {
        ++myCursor;
        SkipWhitespace();
        if (myCursor < myBytes.size() && myBytes[myCursor] == ']') {
            ++myCursor;
            return true;
        }
        while (myCursor < myBytes.size()) {
            if (!ParseValue(depth)) {
                return false;
            }
            SkipWhitespace();
            if (myCursor < myBytes.size() && myBytes[myCursor] == ']') {
                ++myCursor;
                return true;
            }
            if (myCursor >= myBytes.size() || myBytes[myCursor] != ',') {
                return Fail("GLB JSON array syntax is invalid.");
            }
            ++myCursor;
            SkipWhitespace();
        }
        return Fail("GLB JSON array is truncated.");
    }

    bool ParseString(std::u32string *decoded) {
        SkipWhitespace();
        if (myCursor >= myBytes.size() || myBytes[myCursor] != '"') {
            return Fail("GLB JSON object key is not a string.");
        }
        ++myCursor;
        while (myCursor < myBytes.size()) {
            const std::uint8_t value = myBytes[myCursor];
            if (value == '"') {
                ++myCursor;
                return true;
            }
            char32_t scalar = 0;
            if (value == '\\') {
                ++myCursor;
                if (myCursor >= myBytes.size()) {
                    return Fail("GLB JSON string escape is truncated.");
                }
                const std::uint8_t escaped = myBytes[myCursor++];
                switch (escaped) {
                    case '"': scalar = U'"'; break;
                    case '\\': scalar = U'\\'; break;
                    case '/': scalar = U'/'; break;
                    case 'b': scalar = U'\b'; break;
                    case 'f': scalar = U'\f'; break;
                    case 'n': scalar = U'\n'; break;
                    case 'r': scalar = U'\r'; break;
                    case 't': scalar = U'\t'; break;
                    case 'u': {
                        const std::uint16_t first = HexQuad(
                            myBytes.data() + myCursor);
                        myCursor += 4;
                        if (first >= 0xD800U && first <= 0xDBFFU) {
                            myCursor += 2; // The lexical pass proved "\\u".
                            const std::uint16_t second = HexQuad(
                                myBytes.data() + myCursor);
                            myCursor += 4;
                            scalar = static_cast<char32_t>(0x10000U
                                + ((first - 0xD800U) << 10U)
                                + (second - 0xDC00U));
                        } else {
                            scalar = static_cast<char32_t>(first);
                        }
                        break;
                    }
                    default:
                        return Fail("GLB JSON string escape is invalid.");
                }
            } else if (!DecodeUTF8Scalar(
                myBytes.data(), myBytes.size(), myCursor, scalar)) {
                return Fail("GLB JSON string is not strict UTF-8.");
            }
            if (decoded != nullptr) {
                decoded->push_back(scalar);
            }
        }
        return Fail("GLB JSON string is truncated.");
    }

    bool ParseNumber() {
        if (myBytes[myCursor] == '-') {
            ++myCursor;
        }
        if (myCursor >= myBytes.size()) {
            return Fail("GLB JSON number is truncated.");
        }
        if (myBytes[myCursor] == '0') {
            ++myCursor;
        } else if (myBytes[myCursor] >= '1' && myBytes[myCursor] <= '9') {
            while (myCursor < myBytes.size()
                && myBytes[myCursor] >= '0' && myBytes[myCursor] <= '9') {
                ++myCursor;
            }
        } else {
            return Fail("GLB JSON number integer part is invalid.");
        }
        if (myCursor < myBytes.size() && myBytes[myCursor] == '.') {
            ++myCursor;
            const std::size_t fractionStart = myCursor;
            while (myCursor < myBytes.size()
                && myBytes[myCursor] >= '0' && myBytes[myCursor] <= '9') {
                ++myCursor;
            }
            if (fractionStart == myCursor) {
                return Fail("GLB JSON number fraction is invalid.");
            }
        }
        if (myCursor < myBytes.size()
            && (myBytes[myCursor] == 'e' || myBytes[myCursor] == 'E')) {
            ++myCursor;
            if (myCursor < myBytes.size()
                && (myBytes[myCursor] == '+' || myBytes[myCursor] == '-')) {
                ++myCursor;
            }
            const std::size_t exponentStart = myCursor;
            while (myCursor < myBytes.size()
                && myBytes[myCursor] >= '0' && myBytes[myCursor] <= '9') {
                ++myCursor;
            }
            if (exponentStart == myCursor) {
                return Fail("GLB JSON number exponent is invalid.");
            }
        }
        return true;
    }

    bool ConsumeLiteral(const char *literal, const std::size_t count) {
        if (myCursor + count > myBytes.size()
            || std::memcmp(myBytes.data() + myCursor, literal, count) != 0) {
            return false;
        }
        myCursor += count;
        return true;
    }

    const std::vector<std::uint8_t>& myBytes;
    const std::atomic_bool *myCancelled;
    PreflightResult& myResult;
    std::size_t myCursor = 0;
};

NSDictionary *Dictionary(id value) {
    return [value isKindOfClass:[NSDictionary class]]
        ? static_cast<NSDictionary *>(value)
        : nil;
}

NSArray *Array(id value) {
    return [value isKindOfClass:[NSArray class]]
        ? static_cast<NSArray *>(value)
        : nil;
}

NSString *String(id value) {
    return [value isKindOfClass:[NSString class]]
        ? static_cast<NSString *>(value)
        : nil;
}

bool IsBoolean(id value) {
    return value != nil
        && CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID();
}

bool UInt64(id value, std::uint64_t& output) {
    if (![value isKindOfClass:[NSNumber class]] || IsBoolean(value)) {
        return false;
    }
    // Match the pinned RapidJSON consumer's integer-token contract. Accepting
    // an integral-valued JSON real here (for example, `byteStride: 16.0`)
    // would let preflight validate one layout while OCCT loads another.
    if (CFNumberIsFloatType((__bridge CFNumberRef)value)) {
        return false;
    }
    // Every unsigned integer in glTF 2.0 is a JSON integer backed by a
    // 32-bit schema field. Constraining conversion here also avoids lossy
    // NSNumber double-to-uint64 conversions at the 64-bit boundary.
    const double candidate = static_cast<NSNumber *>(value).doubleValue;
    if (!std::isfinite(candidate)
        || candidate < 0.0
        || candidate > static_cast<double>(
            std::numeric_limits<std::uint32_t>::max())
        || std::trunc(candidate) != candidate) {
        return false;
    }
    output = static_cast<std::uint32_t>(candidate);
    return true;
}

bool FiniteDouble(id value, double& output) {
    if (![value isKindOfClass:[NSNumber class]] || IsBoolean(value)) {
        return false;
    }
    output = static_cast<NSNumber *>(value).doubleValue;
    return std::isfinite(output);
}

struct BufferViewRecord {
    std::uint64_t offset = 0;
    std::uint64_t length = 0;
    std::uint64_t stride = 0;
    std::uint64_t target = 0;
    bool hasTarget = false;
};

struct AccessorRecord {
    std::uint64_t bufferView = 0;
    std::uint64_t byteOffset = 0;
    std::uint64_t count = 0;
    std::uint64_t componentType = 0;
    std::string type;
    std::uint64_t componentBytes = 0;
    std::uint64_t componentCount = 0;
    std::uint64_t elementBytes = 0;
};

struct MeshRecord {
    std::uint64_t primitiveCount = 0;
    std::uint64_t vertices = 0;
    std::uint64_t indices = 0;
    std::vector<std::uint64_t> positionAccessors;
};

struct PositionBounds {
    std::array<double, 3> minimum{};
    std::array<double, 3> maximum{};
    bool valid = false;
};

struct SimilarityTransform {
    std::array<std::array<double, 3>, 3> linear{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}},
    }};
    std::array<double, 3> translation{{0.0, 0.0, 0.0}};
};

class SemanticValidator final {
public:
    SemanticValidator(
        const int descriptor,
        const ByteRange binaryChunk,
        const std::atomic_bool *cancelled,
        PreflightResult& result)
    : myDescriptor(descriptor),
      myBinaryChunk(binaryChunk),
      myCancelled(cancelled),
      myResult(result) {
    }

    bool Validate(NSDictionary *root) {
        if (!CheckCancelled()) {
            return false;
        }
        if (!RejectExtensions(root, "GLB root")) {
            return false;
        }
        if (!ValidateEmptyExtensionLists(root)
            || !RejectRootFeatures(root)
            || !ValidateAsset(root)
            || !ValidateBuffer(root)
            || !ValidateBufferViews(root)
            || !ValidateAccessors(root)
            || !ValidateImages(root)
            || !ValidateSamplers(root)
            || !ValidateTextures(root)
            || !ValidateMaterials(root)
            || !ValidateMeshes(root)
            || !ValidateNodesAndScenes(root)) {
            return false;
        }
        if (myResult.objectOccurrenceEstimate == 0) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB contains no scene-instantiated triangle geometry.");
        }
        return CheckCancelled();
    }

private:
    bool Fail(const PreflightStatus status, const char *message) {
        myResult.status = status;
        myResult.message = message;
        return false;
    }

    bool CheckCancelled() {
        return !IsCancelled(myCancelled)
            || Fail(PreflightStatus::Cancelled, "GLB preflight was cancelled.");
    }

    bool RejectExtensions(NSDictionary *dictionary, const char *context) {
        if (dictionary[@"extensions"] == nil) {
            return true;
        }
        std::string message = context;
        message += " uses unsupported extensions.";
        myResult.status = PreflightStatus::Unsupported;
        myResult.message = std::move(message);
        return false;
    }

    bool ValidateEmptyExtensionLists(NSDictionary *root) {
        for (NSString *key in @[@"extensionsUsed", @"extensionsRequired"]) {
            id value = root[key];
            if (value == nil) {
                continue;
            }
            NSArray *items = Array(value);
            if (items == nil || items.count != 0) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB extension lists must be absent or empty.");
            }
        }
        return true;
    }

    bool RejectRootFeatures(NSDictionary *root) {
        for (NSString *key in @[@"skins", @"animations", @"cameras"]) {
            if (root[key] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB skins, animations, and cameras are unsupported.");
            }
        }
        return true;
    }

    bool ValidateAsset(NSDictionary *root) {
        NSDictionary *asset = Dictionary(root[@"asset"]);
        if (asset == nil || !RejectExtensions(asset, "GLB asset")) {
            return asset != nil
                ? false
                : Fail(PreflightStatus::Invalid, "GLB asset metadata is missing.");
        }
        NSString *version = String(asset[@"version"]);
        if (version == nil || ![version isEqualToString:@"2.0"]) {
            return Fail(
                PreflightStatus::Unsupported,
                "Only glTF asset version 2.0 is supported.");
        }
        id minVersionValue = asset[@"minVersion"];
        if (minVersionValue != nil) {
            NSString *minVersion = String(minVersionValue);
            if (minVersion == nil || ![minVersion isEqualToString:@"2.0"]) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "The GLB minimum-version requirement is unsupported.");
            }
        }
        return true;
    }

    bool ValidateBuffer(NSDictionary *root) {
        NSArray *buffers = Array(root[@"buffers"]);
        if (buffers == nil || buffers.count != 1) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB must contain exactly one embedded BIN buffer.");
        }
        NSDictionary *buffer = Dictionary(buffers[0]);
        if (buffer == nil || !RejectExtensions(buffer, "GLB buffer")) {
            return buffer != nil
                ? false
                : Fail(PreflightStatus::Invalid, "GLB buffer metadata is invalid.");
        }
        if (buffer[@"uri"] != nil) {
            return Fail(
                PreflightStatus::Unsupported,
                "External and data-URI GLB buffers are unsupported.");
        }
        if (!UInt64(buffer[@"byteLength"], myDeclaredBufferLength)
            || myDeclaredBufferLength == 0
            || myDeclaredBufferLength > myBinaryChunk.length
            || myBinaryChunk.length - myDeclaredBufferLength > 3) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB BIN length does not match its buffer declaration.");
        }
        return true;
    }

    bool ValidateBufferViews(NSDictionary *root) {
        NSArray *views = Array(root[@"bufferViews"]);
        if (views == nil || views.count == 0) {
            return Fail(PreflightStatus::Invalid, "GLB has no buffer views.");
        }
        if (views.count > kMaximumDefinitionRecords) {
            return Fail(
                PreflightStatus::ResourceLimit,
                "GLB exceeds the buffer-view budget.");
        }
        myBufferViews.reserve(views.count);
        for (NSUInteger index = 0; index < views.count; ++index) {
            if ((index & 0xFFU) == 0 && !CheckCancelled()) {
                return false;
            }
            NSDictionary *view = Dictionary(views[index]);
            if (view == nil || !RejectExtensions(view, "GLB buffer view")) {
                return view != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB buffer view is invalid.");
            }
            BufferViewRecord record;
            std::uint64_t bufferIndex = 0;
            if (!UInt64(view[@"buffer"], bufferIndex) || bufferIndex != 0
                || !UInt64(view[@"byteLength"], record.length)
                || record.length == 0) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB buffer view has invalid buffer or length metadata.");
            }
            id offsetValue = view[@"byteOffset"];
            if (offsetValue != nil && !UInt64(offsetValue, record.offset)) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB buffer view has an invalid byte offset.");
            }
            if (AddWouldOverflow(record.offset, record.length)
                || record.offset + record.length > myDeclaredBufferLength) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB buffer view exceeds the embedded buffer.");
            }
            id strideValue = view[@"byteStride"];
            if (strideValue != nil) {
                if (!UInt64(strideValue, record.stride)
                    || record.stride < 4 || record.stride > 252
                    || (record.stride & 3U) != 0) {
                    return Fail(
                        PreflightStatus::Unsupported,
                        "GLB buffer view uses an unsupported byte stride.");
                }
            }
            id targetValue = view[@"target"];
            if (targetValue != nil) {
                record.hasTarget = true;
                if (!UInt64(targetValue, record.target)
                    || (record.target != 34962 && record.target != 34963)) {
                    return Fail(
                        PreflightStatus::Unsupported,
                        "GLB buffer view uses an unsupported target.");
                }
            }
            myBufferViews.push_back(record);
        }
        return true;
    }

    bool ValidateAccessors(NSDictionary *root) {
        NSArray *accessors = Array(root[@"accessors"]);
        if (accessors == nil || accessors.count == 0) {
            return Fail(PreflightStatus::Invalid, "GLB has no accessors.");
        }
        if (accessors.count > kMaximumDefinitionRecords) {
            return Fail(
                PreflightStatus::ResourceLimit,
                "GLB exceeds the accessor budget.");
        }
        myAccessors.reserve(accessors.count);
        myAccessorBufferViews.assign(myBufferViews.size(), false);
        for (NSUInteger index = 0; index < accessors.count; ++index) {
            if ((index & 0xFFU) == 0 && !CheckCancelled()) {
                return false;
            }
            NSDictionary *accessor = Dictionary(accessors[index]);
            if (accessor == nil || !RejectExtensions(accessor, "GLB accessor")) {
                return accessor != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB accessor is invalid.");
            }
            if (accessor[@"sparse"] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "Sparse GLB accessors are unsupported.");
            }
            AccessorRecord record;
            if (!UInt64(accessor[@"bufferView"], record.bufferView)
                || record.bufferView >= myBufferViews.size()
                || !UInt64(accessor[@"count"], record.count)
                || record.count == 0
                || record.count > kMaximumIndices
                || !UInt64(accessor[@"componentType"], record.componentType)) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB accessor has invalid range metadata.");
            }
            id offsetValue = accessor[@"byteOffset"];
            if (offsetValue != nil && !UInt64(offsetValue, record.byteOffset)) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB accessor has an invalid byte offset.");
            }
            NSString *type = String(accessor[@"type"]);
            if (type == nil) {
                return Fail(PreflightStatus::Invalid, "GLB accessor type is missing.");
            }
            record.type = type.UTF8String ?: "";
            if (record.componentType == 5121) {
                record.componentBytes = 1;
            } else if (record.componentType == 5123) {
                record.componentBytes = 2;
            } else if (record.componentType == 5125
                || record.componentType == 5126) {
                record.componentBytes = 4;
            } else {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB accessor component type is unsupported.");
            }
            if (record.type == "SCALAR") {
                record.componentCount = 1;
            } else if (record.type == "VEC2") {
                record.componentCount = 2;
            } else if (record.type == "VEC3") {
                record.componentCount = 3;
            } else {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB accessor layout is unsupported.");
            }
            const bool supportedLayout =
                (record.type == "SCALAR"
                    && (record.componentType == 5121
                        || record.componentType == 5123
                        || record.componentType == 5125))
                || ((record.type == "VEC2" || record.type == "VEC3")
                    && record.componentType == 5126);
            if (!supportedLayout || accessor[@"normalized"] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB accessor uses an unsupported normalized or packed layout.");
            }
            record.elementBytes = record.componentBytes * record.componentCount;
            const BufferViewRecord& view = myBufferViews[record.bufferView];
            const std::uint64_t stride = view.stride == 0
                ? record.elementBytes
                : view.stride;
            if (stride < record.elementBytes
                || (stride % record.componentBytes) != 0
                || (record.byteOffset % record.componentBytes) != 0
                || ((view.offset + record.byteOffset) % record.componentBytes) != 0
                || record.byteOffset > view.length) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB accessor alignment or stride is invalid.");
            }
            const std::uint64_t steps = record.count - 1;
            if (MultiplyWouldOverflow(steps, stride)
                || AddWouldOverflow(steps * stride, record.elementBytes)
                || steps * stride + record.elementBytes > view.length - record.byteOffset) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB accessor exceeds its buffer view.");
            }
            if (!ValidateAccessorBounds(accessor, record)) {
                return false;
            }
            myAccessorBufferViews[record.bufferView] = true;
            myAccessors.push_back(std::move(record));
        }
        myAccessorPayloadFlags.assign(myAccessors.size(), 0);
        myAccessorIndexMaximum.assign(myAccessors.size(), 0);
        myPositionBounds.resize(myAccessors.size());
        return true;
    }

    bool ValidateAccessorBounds(
        NSDictionary *accessor,
        const AccessorRecord& record) {
        std::array<double, 3> minimum{};
        std::array<double, 3> maximum{};
        bool hasMinimum = false;
        bool hasMaximum = false;
        for (NSString *key in @[@"min", @"max"]) {
            id value = accessor[key];
            if (value == nil) {
                continue;
            }
            NSArray *items = Array(value);
            if (items == nil || items.count != record.componentCount) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB accessor bounds have an invalid layout.");
            }
            for (NSUInteger index = 0; index < items.count; ++index) {
                double number = 0.0;
                if (!FiniteDouble(items[index], number)) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB accessor bounds must contain finite numbers.");
                }
                if ([key isEqualToString:@"min"]) {
                    minimum[index] = number;
                    hasMinimum = true;
                } else {
                    maximum[index] = number;
                    hasMaximum = true;
                }
            }
        }
        if (hasMinimum && hasMaximum) {
            for (std::size_t index = 0; index < record.componentCount; ++index) {
                if (minimum[index] > maximum[index]) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB accessor minimum exceeds its maximum.");
                }
            }
        }
        return true;
    }

    bool ValidateImages(NSDictionary *root) {
        id imagesValue = root[@"images"];
        if (imagesValue == nil) {
            return true;
        }
        NSArray *images = Array(imagesValue);
        if (images == nil || images.count > kMaximumMaterials) {
            return Fail(
                images == nil ? PreflightStatus::Invalid : PreflightStatus::ResourceLimit,
                "GLB image table is invalid or too large.");
        }
        myImageCount = images.count;
        myResult.embeddedImages.reserve(images.count);
        for (NSUInteger index = 0; index < images.count; ++index) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *image = Dictionary(images[index]);
            if (image == nil || !RejectExtensions(image, "GLB image")) {
                return image != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB image is invalid.");
            }
            if (image[@"uri"] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "External and data-URI GLB images are unsupported.");
            }
            std::uint64_t viewIndex = 0;
            if (!UInt64(image[@"bufferView"], viewIndex)
                || viewIndex >= myBufferViews.size()) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB image buffer view is missing or invalid.");
            }
            NSString *mime = String(image[@"mimeType"]);
            const bool isPNG = [mime isEqualToString:@"image/png"];
            const bool isJPEG = [mime isEqualToString:@"image/jpeg"];
            if (!isPNG && !isJPEG) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "Only embedded PNG and JPEG GLB images are supported.");
            }
            const BufferViewRecord& view = myBufferViews[viewIndex];
            if (view.stride != 0 || view.hasTarget
                || myAccessorBufferViews[viewIndex]
                || AddWouldOverflow(myBinaryChunk.offset, view.offset)) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB image buffer view has invalid metadata.");
            }
            EmbeddedImage output;
            output.imageIndex = index;
            output.bytes = {
                myBinaryChunk.offset + view.offset,
                view.length,
            };
            output.mimeType = isPNG ? "image/png" : "image/jpeg";
            if (!ValidateImageSignature(output, isPNG)) {
                return false;
            }
            myResult.embeddedImages.push_back(std::move(output));
        }
        return true;
    }

    bool ValidateImageSignature(
        const EmbeddedImage& image,
        const bool isPNG) {
        if (isPNG) {
            constexpr std::array<std::uint8_t, 8> signature = {
                0x89U, 'P', 'N', 'G', 0x0DU, 0x0AU, 0x1AU, 0x0AU,
            };
            std::array<std::uint8_t, 8> actual{};
            if (image.bytes.length < actual.size()
                || !PreadExactly(
                    myDescriptor,
                    actual.data(),
                    actual.size(),
                    image.bytes.offset,
                    myCancelled,
                    myResult)) {
                return image.bytes.length >= actual.size() ? false : Fail(
                    PreflightStatus::Invalid,
                    "Embedded GLB PNG is truncated.");
            }
            if (actual != signature) {
                return Fail(
                    PreflightStatus::Invalid,
                    "Embedded GLB PNG signature does not match its MIME type.");
            }
            return true;
        }

        std::array<std::uint8_t, 3> beginning{};
        std::array<std::uint8_t, 2> ending{};
        if (image.bytes.length < 4) {
            return Fail(PreflightStatus::Invalid, "Embedded GLB JPEG is truncated.");
        }
        if (!PreadExactly(
                myDescriptor,
                beginning.data(),
                beginning.size(),
                image.bytes.offset,
                myCancelled,
                myResult)
            || !PreadExactly(
                myDescriptor,
                ending.data(),
                ending.size(),
                image.bytes.offset + image.bytes.length - ending.size(),
                myCancelled,
                myResult)) {
            return false;
        }
        if (beginning[0] != 0xFFU || beginning[1] != 0xD8U
            || beginning[2] != 0xFFU
            || ending[0] != 0xFFU || ending[1] != 0xD9U) {
            return Fail(
                PreflightStatus::Invalid,
                "Embedded GLB JPEG markers do not match its MIME type.");
        }
        return true;
    }

    bool ValidateSamplers(NSDictionary *root) {
        id value = root[@"samplers"];
        if (value == nil) {
            return true;
        }
        NSArray *samplers = Array(value);
        if (samplers == nil || samplers.count > kMaximumMaterials) {
            return Fail(
                samplers == nil ? PreflightStatus::Invalid : PreflightStatus::ResourceLimit,
                "GLB sampler table is invalid or too large.");
        }
        mySamplerCount = samplers.count;
        for (NSDictionary *samplerValue in samplers) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *sampler = Dictionary(samplerValue);
            if (sampler == nil || !RejectExtensions(sampler, "GLB sampler")) {
                return sampler != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB sampler is invalid.");
            }
            // OCCT 7.8 does not preserve sampler state. Admit only the
            // renderer's canonical linear/trilinear repeat behavior so import
            // cannot silently change authored texture semantics.
            if (!ValidateOptionalEnum(sampler, @"magFilter", {9729})
                || !ValidateOptionalEnum(sampler, @"minFilter", {9987})
                || !ValidateOptionalEnum(sampler, @"wrapS", {10497})
                || !ValidateOptionalEnum(sampler, @"wrapT", {10497})) {
                return false;
            }
        }
        return true;
    }

    bool ValidateOptionalEnum(
        NSDictionary *dictionary,
        NSString *key,
        std::initializer_list<std::uint64_t> allowed) {
        id value = dictionary[key];
        if (value == nil) {
            return true;
        }
        std::uint64_t integer = 0;
        if (!UInt64(value, integer)
            || std::find(allowed.begin(), allowed.end(), integer) == allowed.end()) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB sampler uses an unsupported filter or wrapping mode.");
        }
        return true;
    }

    bool ValidateTextures(NSDictionary *root) {
        id value = root[@"textures"];
        if (value == nil) {
            return true;
        }
        NSArray *textures = Array(value);
        if (textures == nil || textures.count > kMaximumMaterials) {
            return Fail(
                textures == nil ? PreflightStatus::Invalid : PreflightStatus::ResourceLimit,
                "GLB texture table is invalid or too large.");
        }
        myTextureCount = textures.count;
        for (NSDictionary *textureValue in textures) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *texture = Dictionary(textureValue);
            if (texture == nil || !RejectExtensions(texture, "GLB texture")) {
                return texture != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB texture is invalid.");
            }
            std::uint64_t source = 0;
            if (!UInt64(texture[@"source"], source) || source >= myImageCount) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB texture has no valid embedded image source.");
            }
            id samplerValue = texture[@"sampler"];
            std::uint64_t sampler = 0;
            if (samplerValue != nil
                && (!UInt64(samplerValue, sampler) || sampler >= mySamplerCount)) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB texture references an invalid sampler.");
            }
        }
        return true;
    }

    bool ValidateMaterials(NSDictionary *root) {
        id value = root[@"materials"];
        if (value == nil) {
            return true;
        }
        NSArray *materials = Array(value);
        if (materials == nil || materials.count > kMaximumMaterials) {
            return Fail(
                materials == nil ? PreflightStatus::Invalid : PreflightStatus::ResourceLimit,
                "GLB material table is invalid or too large.");
        }
        myMaterialCount = materials.count;
        myMaterialUsesTexture.reserve(materials.count);
        for (NSDictionary *materialValue in materials) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *material = Dictionary(materialValue);
            if (material == nil || !RejectExtensions(material, "GLB material")) {
                return material != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB material is invalid.");
            }
            if (!ValidateMaterial(material)) {
                return false;
            }
            NSDictionary *pbr = Dictionary(material[@"pbrMetallicRoughness"]);
            const bool usesTexture = material[@"emissiveTexture"] != nil
                || (pbr != nil && pbr[@"baseColorTexture"] != nil);
            myMaterialUsesTexture.push_back(usesTexture);
        }
        return true;
    }

    bool ValidateMaterial(NSDictionary *material) {
        if (material[@"values"] != nil || material[@"technique"] != nil) {
            return Fail(
                PreflightStatus::Unsupported,
                "Legacy glTF material values and techniques are unsupported.");
        }
        id pbrValue = material[@"pbrMetallicRoughness"];
        NSDictionary *pbr = Dictionary(pbrValue);
        if (pbr == nil || !RejectExtensions(pbr, "GLB PBR material")) {
            return pbr != nil
                ? false
                : Fail(
                    PreflightStatus::Unsupported,
                    "Every GLB material must provide PBR metallic-roughness metadata.");
        }
        if (pbr[@"metallicRoughnessTexture"] != nil
            || material[@"normalTexture"] != nil
            || material[@"occlusionTexture"] != nil) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB normal, occlusion, and metallic-roughness texture maps are unsupported.");
        }
        if (!ValidateFactorArray(pbr, @"baseColorFactor", 4, 0.0, 1.0)
            || !ValidateFactor(pbr, @"metallicFactor", 0.0, 1.0)
            || !ValidateFactor(pbr, @"roughnessFactor", 0.0, 1.0)
            || !ValidateTextureInfo(pbr, @"baseColorTexture", false, false)) {
            return false;
        }
        if (!ValidateFactorArray(material, @"emissiveFactor", 3, 0.0, 1.0)
            || !ValidateTextureInfo(material, @"emissiveTexture", false, false)) {
            return false;
        }
        id alphaModeValue = material[@"alphaMode"];
        if (alphaModeValue != nil) {
            NSString *alphaMode = String(alphaModeValue);
            if (alphaMode == nil
                || (![alphaMode isEqualToString:@"OPAQUE"]
                    && ![alphaMode isEqualToString:@"MASK"]
                    && ![alphaMode isEqualToString:@"BLEND"])) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB material uses an unsupported alpha mode.");
            }
        }
        if (!ValidateFactor(material, @"alphaCutoff", 0.0, 1.0)) {
            return false;
        }
        id doubleSided = material[@"doubleSided"];
        if (doubleSided != nil && !IsBoolean(doubleSided)) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB material doubleSided must be Boolean.");
        }
        return true;
    }

    bool ValidateFactorArray(
        NSDictionary *dictionary,
        NSString *key,
        const NSUInteger requiredCount,
        const double minimum,
        const double maximum) {
        id value = dictionary[key];
        if (value == nil) {
            return true;
        }
        NSArray *items = Array(value);
        if (items == nil || items.count != requiredCount) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB material factor has an invalid layout.");
        }
        for (id item in items) {
            double number = 0.0;
            if (!FiniteDouble(item, number) || number < minimum || number > maximum) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB material factor is outside its supported range.");
            }
        }
        return true;
    }

    bool ValidateFactor(
        NSDictionary *dictionary,
        NSString *key,
        const double minimum,
        const double maximum) {
        id value = dictionary[key];
        if (value == nil) {
            return true;
        }
        double number = 0.0;
        if (!FiniteDouble(value, number) || number < minimum || number > maximum) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB material scalar is outside its supported range.");
        }
        return true;
    }

    bool ValidateTextureInfo(
        NSDictionary *owner,
        NSString *key,
        const bool allowScale,
        const bool allowStrength) {
        id value = owner[key];
        if (value == nil) {
            return true;
        }
        NSDictionary *info = Dictionary(value);
        if (info == nil || !RejectExtensions(info, "GLB texture info")) {
            return info != nil
                ? false
                : Fail(PreflightStatus::Invalid, "GLB texture info is invalid.");
        }
        std::uint64_t texture = 0;
        if (!UInt64(info[@"index"], texture) || texture >= myTextureCount) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB material references an invalid texture.");
        }
        id texCoordValue = info[@"texCoord"];
        std::uint64_t texCoord = 0;
        if (texCoordValue != nil
            && (!UInt64(texCoordValue, texCoord) || texCoord != 0)) {
            return Fail(
                PreflightStatus::Unsupported,
                "Only GLB texture coordinate set zero is supported.");
        }
        if (info[@"scale"] != nil) {
            double scale = 0.0;
            if (!allowScale || !FiniteDouble(info[@"scale"], scale)
                || scale != 1.0) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "Only the default GLB normal texture scale is supported.");
            }
        }
        if (info[@"strength"] != nil) {
            double strength = 0.0;
            if (!allowStrength || !FiniteDouble(info[@"strength"], strength)
                || strength != 1.0) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "Only the default GLB occlusion texture strength is supported.");
            }
        }
        return true;
    }

    bool ValidateMeshes(NSDictionary *root) {
        NSArray *meshes = Array(root[@"meshes"]);
        if (meshes == nil || meshes.count == 0) {
            return Fail(PreflightStatus::Invalid, "GLB has no meshes.");
        }
        if (meshes.count > kMaximumObjects) {
            return Fail(PreflightStatus::ResourceLimit, "GLB exceeds the mesh budget.");
        }
        myMeshes.reserve(meshes.count);
        std::uint64_t definitionPrimitives = 0;
        std::uint64_t definitionVertices = 0;
        std::uint64_t definitionIndices = 0;
        for (NSUInteger meshIndex = 0; meshIndex < meshes.count; ++meshIndex) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *mesh = Dictionary(meshes[meshIndex]);
            if (mesh == nil || !RejectExtensions(mesh, "GLB mesh")) {
                return mesh != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB mesh is invalid.");
            }
            if (mesh[@"weights"] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB morph weights are unsupported.");
            }
            NSArray *primitives = Array(mesh[@"primitives"]);
            if (primitives == nil || primitives.count == 0) {
                return Fail(PreflightStatus::Invalid, "GLB mesh has no primitives.");
            }
            MeshRecord meshRecord;
            meshRecord.primitiveCount = primitives.count;
            if (!Accumulate(definitionPrimitives, meshRecord.primitiveCount, kMaximumObjects)) {
                return Fail(
                    PreflightStatus::ResourceLimit,
                    "GLB exceeds the mesh-primitive definition budget.");
            }
            for (NSDictionary *primitiveValue in primitives) {
                NSDictionary *primitive = Dictionary(primitiveValue);
                if (primitive == nil || !ValidatePrimitive(primitive, meshRecord)) {
                    return primitive != nil
                        ? false
                        : Fail(PreflightStatus::Invalid, "GLB mesh primitive is invalid.");
                }
            }
            if (!Accumulate(definitionVertices, meshRecord.vertices, kMaximumVertices)
                || !Accumulate(definitionIndices, meshRecord.indices, kMaximumIndices)) {
                return Fail(
                    PreflightStatus::ResourceLimit,
                    "GLB unique mesh definitions exceed geometry budgets.");
            }
            myMeshes.push_back(meshRecord);
        }
        return true;
    }

    bool ValidatePrimitive(NSDictionary *primitive, MeshRecord& mesh) {
        if (!RejectExtensions(primitive, "GLB mesh primitive")) {
            return false;
        }
        if (primitive[@"targets"] != nil) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB morph targets are unsupported.");
        }
        id modeValue = primitive[@"mode"];
        std::uint64_t mode = 4;
        if (modeValue != nil && (!UInt64(modeValue, mode) || mode != 4)) {
            return Fail(
                PreflightStatus::Unsupported,
                "Only indexed TRIANGLES GLB primitives are supported.");
        }
        NSDictionary *attributes = Dictionary(primitive[@"attributes"]);
        if (attributes == nil || !RejectExtensions(attributes, "GLB attributes")) {
            return attributes != nil
                ? false
                : Fail(PreflightStatus::Invalid, "GLB primitive attributes are missing.");
        }
        for (id keyValue in attributes) {
            NSString *key = String(keyValue);
            if (key == nil
                || (![key isEqualToString:@"POSITION"]
                    && ![key isEqualToString:@"NORMAL"]
                    && ![key isEqualToString:@"TEXCOORD_0"])) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB primitive contains an unsupported vertex attribute.");
            }
        }
        const AccessorRecord *position = AccessorAt(attributes[@"POSITION"]);
        if (position == nullptr
            || position->type != "VEC3"
            || position->componentType != 5126
            || !RequireVertexBuffer(*position)) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB POSITION must be an aligned floating-point VEC3 accessor.");
        }
        if (!ValidateFloatingPayload(*position, PayloadRole::Position)) {
            return false;
        }
        id normalValue = attributes[@"NORMAL"];
        if (normalValue != nil) {
            const AccessorRecord *normal = AccessorAt(normalValue);
            if (normal == nullptr || normal->type != "VEC3"
                || normal->componentType != 5126
                || normal->count != position->count
                || !RequireVertexBuffer(*normal)) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB NORMAL must match POSITION as floating-point VEC3.");
            }
            if (!ValidateFloatingPayload(*normal, PayloadRole::Normal)) {
                return false;
            }
        }
        id texCoordValue = attributes[@"TEXCOORD_0"];
        if (texCoordValue != nil) {
            const AccessorRecord *texCoord = AccessorAt(texCoordValue);
            if (texCoord == nullptr || texCoord->type != "VEC2"
                || texCoord->componentType != 5126
                || texCoord->count != position->count
                || !RequireVertexBuffer(*texCoord)) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB TEXCOORD_0 must match POSITION as floating-point VEC2.");
            }
            if (!ValidateFloatingPayload(*texCoord, PayloadRole::TexCoord)) {
                return false;
            }
        }
        const AccessorRecord *indices = AccessorAt(primitive[@"indices"]);
        if (indices == nullptr || indices->type != "SCALAR"
            || (indices->componentType != 5121
                && indices->componentType != 5123
                && indices->componentType != 5125)
            || indices->count < 3 || indices->count % 3 != 0) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB primitive must use an unsigned indexed triangle accessor.");
        }
        const BufferViewRecord& indexView = myBufferViews[indices->bufferView];
        if (indexView.stride != 0
            || (indexView.hasTarget && indexView.target != 34963)) {
            return Fail(
                PreflightStatus::Unsupported,
                "GLB index buffer view layout is unsupported.");
        }
        std::uint64_t maximumIndex = 0;
        if (!ValidateIndexPayload(*indices, maximumIndex)) {
            return false;
        }
        if (maximumIndex >= position->count) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB primitive index refers beyond its POSITION accessor.");
        }
        id materialValue = primitive[@"material"];
        std::uint64_t material = 0;
        if (materialValue != nil
            && (!UInt64(materialValue, material) || material >= myMaterialCount)) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB primitive references an invalid material.");
        }
        if (materialValue != nil && myMaterialUsesTexture[material]
            && texCoordValue == nil) {
            return Fail(
                PreflightStatus::Invalid,
                "Textured GLB primitives must provide TEXCOORD_0.");
        }
        const std::uint64_t positionIndex = static_cast<std::uint64_t>(
            position - myAccessors.data());
        mesh.positionAccessors.push_back(positionIndex);
        return Accumulate(mesh.vertices, position->count, kMaximumVertices)
            && Accumulate(mesh.indices, indices->count, kMaximumIndices)
            ? true
            : Fail(
                PreflightStatus::ResourceLimit,
                "GLB mesh exceeds geometry budgets.");
    }

    enum class PayloadRole : std::uint8_t {
        Position = 1U,
        Normal = 2U,
        TexCoord = 4U,
    };

    bool ValidateFloatingPayload(
        const AccessorRecord& accessor,
        const PayloadRole role) {
        const std::size_t accessorIndex = static_cast<std::size_t>(
            &accessor - myAccessors.data());
        const std::uint8_t roleBit = static_cast<std::uint8_t>(role);
        if ((myAccessorPayloadFlags[accessorIndex] & roleBit) != 0) {
            return true;
        }

        const BufferViewRecord& view = myBufferViews[accessor.bufferView];
        const std::uint64_t stride = view.stride == 0
            ? accessor.elementBytes
            : view.stride;
        if (AddWouldOverflow(myBinaryChunk.offset, view.offset)
            || AddWouldOverflow(
                myBinaryChunk.offset + view.offset,
                accessor.byteOffset)) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB vertex payload offset overflowed.");
        }
        const std::uint64_t sourceStart =
            myBinaryChunk.offset + view.offset + accessor.byteOffset;
        PositionBounds bounds;
        bounds.minimum.fill(std::numeric_limits<double>::infinity());
        bounds.maximum.fill(-std::numeric_limits<double>::infinity());

        constexpr std::uint64_t kElementsPerRead = 4'096;
        for (std::uint64_t first = 0; first < accessor.count;) {
            if (!CheckCancelled()) {
                return false;
            }
            const std::uint64_t count = std::min(
                kElementsPerRead,
                accessor.count - first);
            const std::uint64_t span =
                (count - 1) * stride + accessor.elementBytes;
            std::vector<std::uint8_t> block(static_cast<std::size_t>(span));
            if (!PreadExactly(
                    myDescriptor,
                    block.data(),
                    block.size(),
                    sourceStart + first * stride,
                    myCancelled,
                    myResult)) {
                return false;
            }
            for (std::uint64_t item = 0; item < count; ++item) {
                double normSquared = 0.0;
                const std::uint8_t *element = block.data() + item * stride;
                for (std::uint64_t component = 0;
                     component < accessor.componentCount;
                     ++component) {
                    const std::uint32_t bits = ReadLE32(
                        element + component * sizeof(float));
                    float value = 0.0F;
                    std::memcpy(&value, &bits, sizeof(value));
                    const double number = value;
                    if (!std::isfinite(number)
                        || std::abs(number) > 1'000'000.0) {
                        return Fail(
                            PreflightStatus::Invalid,
                            "GLB vertex payload contains non-finite or unbounded values.");
                    }
                    normSquared += number * number;
                    if (role == PayloadRole::Position) {
                        bounds.minimum[component] = std::min(
                            bounds.minimum[component],
                            number);
                        bounds.maximum[component] = std::max(
                            bounds.maximum[component],
                            number);
                    }
                }
                if (role == PayloadRole::Normal
                    && (!std::isfinite(normSquared) || normSquared < 1.0e-12)) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB NORMAL payload contains a zero or invalid vector.");
                }
            }
            first += count;
        }
        if (role == PayloadRole::Position) {
            bounds.valid = true;
            myPositionBounds[accessorIndex] = bounds;
        }
        myAccessorPayloadFlags[accessorIndex] |= roleBit;
        return true;
    }

    bool ValidateIndexPayload(
        const AccessorRecord& accessor,
        std::uint64_t& maximum) {
        const std::size_t accessorIndex = static_cast<std::size_t>(
            &accessor - myAccessors.data());
        constexpr std::uint8_t kIndexValidated = 8U;
        if ((myAccessorPayloadFlags[accessorIndex] & kIndexValidated) != 0) {
            maximum = myAccessorIndexMaximum[accessorIndex];
            return true;
        }
        const BufferViewRecord& view = myBufferViews[accessor.bufferView];
        if (AddWouldOverflow(myBinaryChunk.offset, view.offset)
            || AddWouldOverflow(
                myBinaryChunk.offset + view.offset,
                accessor.byteOffset)) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB index payload offset overflowed.");
        }
        const std::uint64_t sourceStart =
            myBinaryChunk.offset + view.offset + accessor.byteOffset;
        constexpr std::uint64_t kElementsPerRead = 16'384;
        maximum = 0;
        for (std::uint64_t first = 0; first < accessor.count;) {
            if (!CheckCancelled()) {
                return false;
            }
            const std::uint64_t count = std::min(
                kElementsPerRead,
                accessor.count - first);
            const std::uint64_t byteCount = count * accessor.componentBytes;
            std::vector<std::uint8_t> block(static_cast<std::size_t>(byteCount));
            if (!PreadExactly(
                    myDescriptor,
                    block.data(),
                    block.size(),
                    sourceStart + first * accessor.componentBytes,
                    myCancelled,
                    myResult)) {
                return false;
            }
            for (std::uint64_t item = 0; item < count; ++item) {
                const std::uint8_t *element =
                    block.data() + item * accessor.componentBytes;
                std::uint64_t indexValue = element[0];
                if (accessor.componentBytes >= 2) {
                    indexValue = static_cast<std::uint64_t>(element[0])
                        | (static_cast<std::uint64_t>(element[1]) << 8U);
                }
                if (accessor.componentBytes == 4) {
                    indexValue |= static_cast<std::uint64_t>(element[2]) << 16U;
                    indexValue |= static_cast<std::uint64_t>(element[3]) << 24U;
                }
                maximum = std::max(maximum, indexValue);
            }
            first += count;
        }
        myAccessorIndexMaximum[accessorIndex] = maximum;
        myAccessorPayloadFlags[accessorIndex] |= kIndexValidated;
        return true;
    }

    const AccessorRecord *AccessorAt(id value) const {
        std::uint64_t index = 0;
        return UInt64(value, index) && index < myAccessors.size()
            ? &myAccessors[index]
            : nullptr;
    }

    bool RequireVertexBuffer(const AccessorRecord& accessor) const {
        const BufferViewRecord& view = myBufferViews[accessor.bufferView];
        return !view.hasTarget || view.target == 34962;
    }

    bool ValidateNodesAndScenes(NSDictionary *root) {
        NSArray *nodes = Array(root[@"nodes"]);
        if (nodes == nil || nodes.count == 0) {
            return Fail(PreflightStatus::Invalid, "GLB has no scene nodes.");
        }
        if (nodes.count > kMaximumObjects) {
            return Fail(PreflightStatus::ResourceLimit, "GLB exceeds the node budget.");
        }
        myNodeChildren.resize(nodes.count);
        myNodeParents.assign(nodes.count, 0);
        myNodeMeshes.assign(nodes.count, -1);
        myNodeTransforms.resize(nodes.count);
        for (NSUInteger index = 0; index < nodes.count; ++index) {
            if (!CheckCancelled()) {
                return false;
            }
            NSDictionary *node = Dictionary(nodes[index]);
            if (node == nil || !RejectExtensions(node, "GLB node")) {
                return node != nil
                    ? false
                    : Fail(PreflightStatus::Invalid, "GLB node is invalid.");
            }
            if (node[@"camera"] != nil || node[@"skin"] != nil
                || node[@"weights"] != nil) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB camera, skin, and morph node features are unsupported.");
            }
            if (!ValidateNodeTransform(node, myNodeTransforms[index])) {
                return false;
            }
            id meshValue = node[@"mesh"];
            if (meshValue != nil) {
                std::uint64_t meshIndex = 0;
                if (!UInt64(meshValue, meshIndex) || meshIndex >= myMeshes.size()) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB node references an invalid mesh.");
                }
                myNodeMeshes[index] = static_cast<std::int64_t>(meshIndex);
            }
            id childrenValue = node[@"children"];
            if (childrenValue == nil) {
                continue;
            }
            NSArray *children = Array(childrenValue);
            if (children == nil) {
                return Fail(PreflightStatus::Invalid, "GLB node children are invalid.");
            }
            myNodeChildren[index].reserve(children.count);
            for (id childValue in children) {
                std::uint64_t child = 0;
                if (!UInt64(childValue, child) || child >= nodes.count
                    || child == index
                    || std::find(
                        myNodeChildren[index].begin(),
                        myNodeChildren[index].end(),
                        child) != myNodeChildren[index].end()) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB node has an invalid or duplicate child.");
                }
                myNodeChildren[index].push_back(child);
                ++myNodeParents[child];
                if (myNodeParents[child] > 1) {
                    return Fail(
                        PreflightStatus::Unsupported,
                        "GLB node graphs must have at most one parent per node.");
                }
            }
        }
        if (!ValidateAcyclicGraph()) {
            return false;
        }
        return ValidateScenes(root);
    }

    bool ValidateNodeTransform(
        NSDictionary *node,
        SimilarityTransform& output) {
        const bool hasMatrix = node[@"matrix"] != nil;
        const bool hasTRS = node[@"translation"] != nil
            || node[@"rotation"] != nil || node[@"scale"] != nil;
        if (hasMatrix && hasTRS) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB node cannot combine matrix and TRS transforms.");
        }
        if (hasMatrix) {
            std::vector<double> matrix;
            if (!ReadTransformArray(node[@"matrix"], 16, matrix)) {
                return false;
            }
            constexpr double kAffineTolerance = 1.0e-6;
            if (std::abs(matrix[3]) > kAffineTolerance
                || std::abs(matrix[7]) > kAffineTolerance
                || std::abs(matrix[11]) > kAffineTolerance
                || std::abs(matrix[15] - 1.0) > kAffineTolerance) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node matrix must be affine.");
            }
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    output.linear[row][column] = matrix[column * 4 + row];
                }
            }
            output.translation = {{matrix[12], matrix[13], matrix[14]}};

            std::array<double, 3> lengthSquared{};
            for (std::size_t column = 0; column < 3; ++column) {
                for (std::size_t row = 0; row < 3; ++row) {
                    const double value = output.linear[row][column];
                    lengthSquared[column] += value * value;
                }
            }
            const double scaleSquared = lengthSquared[0];
            if (!std::isfinite(scaleSquared)
                || scaleSquared < 1.0e-18
                || scaleSquared > 1.0e12) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node matrix scale is singular or unbounded.");
            }
            const double metricTolerance = std::max(
                1.0e-18,
                scaleSquared * 1.0e-6);
            if (std::abs(lengthSquared[1] - scaleSquared) > metricTolerance
                || std::abs(lengthSquared[2] - scaleSquared) > metricTolerance) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node matrix must use uniform scale.");
            }
            for (std::size_t first = 0; first < 3; ++first) {
                for (std::size_t second = first + 1; second < 3; ++second) {
                    double dot = 0.0;
                    for (std::size_t row = 0; row < 3; ++row) {
                        dot += output.linear[row][first]
                            * output.linear[row][second];
                    }
                    if (std::abs(dot) > metricTolerance) {
                        return Fail(
                            PreflightStatus::Unsupported,
                            "GLB node matrix shear is unsupported.");
                    }
                }
            }
            const double determinant =
                output.linear[0][0]
                    * (output.linear[1][1] * output.linear[2][2]
                        - output.linear[1][2] * output.linear[2][1])
                - output.linear[0][1]
                    * (output.linear[1][0] * output.linear[2][2]
                        - output.linear[1][2] * output.linear[2][0])
                + output.linear[0][2]
                    * (output.linear[1][0] * output.linear[2][1]
                        - output.linear[1][1] * output.linear[2][0]);
            if (!std::isfinite(determinant) || determinant <= 0.0) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node matrix reflection is unsupported.");
            }
            return true;
        }

        std::vector<double> translation;
        if (node[@"translation"] != nil) {
            if (!ReadTransformArray(node[@"translation"], 3, translation)) {
                return false;
            }
            output.translation = {{
                translation[0], translation[1], translation[2],
            }};
        }

        double uniformScale = 1.0;
        if (node[@"scale"] != nil) {
            std::vector<double> scale;
            if (!ReadTransformArray(node[@"scale"], 3, scale)) {
                return false;
            }
            const double scaleTolerance = std::max(
                1.0e-15,
                scale[0] * 1.0e-6);
            if (scale[0] <= 1.0e-9
                || std::abs(scale[1] - scale[0]) > scaleTolerance
                || std::abs(scale[2] - scale[0]) > scaleTolerance) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node TRS must use positive uniform scale.");
            }
            uniformScale = scale[0];
        }

        if (node[@"rotation"] != nil) {
            std::vector<double> rotation;
            if (!ReadTransformArray(node[@"rotation"], 4, rotation)) {
                return false;
            }
            const double x = rotation[0];
            const double y = rotation[1];
            const double z = rotation[2];
            const double w = rotation[3];
            const double normSquared = x * x + y * y + z * z + w * w;
            if (!std::isfinite(normSquared)
                || std::abs(normSquared - 1.0) > 1.0e-3) {
                return Fail(
                    PreflightStatus::Unsupported,
                    "GLB node rotation quaternion must be normalized.");
            }
            const double inverseNorm = 1.0 / std::sqrt(normSquared);
            const double nx = x * inverseNorm;
            const double ny = y * inverseNorm;
            const double nz = z * inverseNorm;
            const double nw = w * inverseNorm;
            output.linear = {{
                {{1.0 - 2.0 * (ny * ny + nz * nz),
                  2.0 * (nx * ny - nz * nw),
                  2.0 * (nx * nz + ny * nw)}},
                {{2.0 * (nx * ny + nz * nw),
                  1.0 - 2.0 * (nx * nx + nz * nz),
                  2.0 * (ny * nz - nx * nw)}},
                {{2.0 * (nx * nz - ny * nw),
                  2.0 * (ny * nz + nx * nw),
                  1.0 - 2.0 * (nx * nx + ny * ny)}},
            }};
        }
        for (auto& row : output.linear) {
            for (double& value : row) {
                value *= uniformScale;
            }
        }
        return true;
    }

    bool ReadTransformArray(
        id value,
        const NSUInteger requiredCount,
        std::vector<double>& output) {
        NSArray *items = Array(value);
        if (items == nil || items.count != requiredCount) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB node transform has an invalid layout.");
        }
        output.clear();
        output.reserve(requiredCount);
        for (id item in items) {
            double number = 0.0;
            if (!FiniteDouble(item, number) || std::abs(number) > 1'000'000.0) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB node transform must contain bounded finite numbers.");
            }
            output.push_back(number);
        }
        return true;
    }

    bool ValidateAcyclicGraph() {
        std::vector<std::uint8_t> color(myNodeChildren.size(), 0);
        struct Frame {
            std::uint64_t node = 0;
            std::size_t nextChild = 0;
        };
        std::vector<Frame> stack;
        stack.reserve(myNodeChildren.size());
        for (std::uint64_t start = 0; start < myNodeChildren.size(); ++start) {
            if (color[start] != 0) {
                continue;
            }
            stack.push_back({start, 0});
            color[start] = 1;
            while (!stack.empty()) {
                if (!CheckCancelled()) {
                    return false;
                }
                Frame& frame = stack.back();
                if (frame.nextChild == myNodeChildren[frame.node].size()) {
                    color[frame.node] = 2;
                    stack.pop_back();
                    continue;
                }
                const std::uint64_t child =
                    myNodeChildren[frame.node][frame.nextChild++];
                if (color[child] == 1) {
                    return Fail(
                        PreflightStatus::Invalid,
                        "GLB node graph contains a cycle.");
                }
                if (color[child] == 0) {
                    color[child] = 1;
                    stack.push_back({child, 0});
                }
            }
        }
        return true;
    }

    bool ValidateScenes(NSDictionary *root) {
        NSArray *scenes = Array(root[@"scenes"]);
        if (scenes == nil || scenes.count != 1) {
            return Fail(
                PreflightStatus::Unsupported,
                "The mobile GLB subset requires exactly one scene.");
        }
        id defaultSceneValue = root[@"scene"];
        if (defaultSceneValue != nil) {
            std::uint64_t defaultScene = 0;
            if (!UInt64(defaultSceneValue, defaultScene) || defaultScene != 0) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB default scene index is invalid.");
            }
        }

        NSDictionary *scene = Dictionary(scenes[0]);
        if (scene == nil || !RejectExtensions(scene, "GLB scene")) {
            return scene != nil
                ? false
                : Fail(PreflightStatus::Invalid, "GLB scene is invalid.");
        }
        NSArray *roots = Array(scene[@"nodes"]);
        if (roots == nil || roots.count == 0) {
            return Fail(
                PreflightStatus::Invalid,
                "The GLB scene must contain at least one root node.");
        }

        std::vector<std::uint8_t> reached(myNodeChildren.size(), 0);
        struct TraversalEntry {
            std::uint64_t node = 0;
            std::uint64_t depth = 0;
            SimilarityTransform parentTransform;
        };
        std::vector<TraversalEntry> traversal;
        traversal.reserve(myNodeChildren.size());
        std::vector<std::uint8_t> rootMembership(myNodeChildren.size(), 0);
        for (id rootValue in roots) {
            std::uint64_t rootNode = 0;
            if (!UInt64(rootValue, rootNode)
                || rootNode >= myNodeChildren.size()
                || myNodeParents[rootNode] != 0
                || rootMembership[rootNode] != 0) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB scene has an invalid, parented, or repeated root.");
            }
            rootMembership[rootNode] = 1;
            traversal.push_back({rootNode, 1, SimilarityTransform{}});
        }
        while (!traversal.empty()) {
            if (!CheckCancelled()) {
                return false;
            }
            TraversalEntry entry = std::move(traversal.back());
            traversal.pop_back();
            if (entry.depth > 128) {
                return Fail(
                    PreflightStatus::ResourceLimit,
                    "GLB node graph exceeds the occurrence-depth budget.");
            }
            if (reached[entry.node] != 0) {
                return Fail(
                    PreflightStatus::Invalid,
                    "GLB node occurs more than once in its scene.");
            }
            reached[entry.node] = 1;
            SimilarityTransform world;
            if (!ComposeTransforms(
                    entry.parentTransform,
                    myNodeTransforms[entry.node],
                    world)) {
                return false;
            }

            const std::int64_t meshIndex = myNodeMeshes[entry.node];
            if (meshIndex >= 0) {
                const MeshRecord& mesh = myMeshes[
                    static_cast<std::size_t>(meshIndex)];
                if (!Accumulate(
                        myResult.objectOccurrenceEstimate,
                        mesh.primitiveCount,
                        kMaximumObjects)
                    || !Accumulate(
                        myResult.vertexEstimate,
                        mesh.vertices,
                        kMaximumVertices)
                    || !Accumulate(
                        myResult.indexEstimate,
                        mesh.indices,
                        kMaximumIndices)) {
                    return Fail(
                        PreflightStatus::ResourceLimit,
                        "Flattened GLB occurrences exceed mobile geometry budgets.");
                }
                for (const std::uint64_t positionAccessor
                     : mesh.positionAccessors) {
                    if (!ValidateTransformedPositionBounds(
                            myPositionBounds[positionAccessor],
                            world)) {
                        return false;
                    }
                }
            }
            for (const std::uint64_t child : myNodeChildren[entry.node]) {
                traversal.push_back({child, entry.depth + 1, world});
            }
        }
        if (std::find(reached.begin(), reached.end(), 0) != reached.end()) {
            return Fail(
                PreflightStatus::Unsupported,
                "Every admitted GLB node must belong to exactly one scene tree.");
        }
        return true;
    }

    bool ComposeTransforms(
        const SimilarityTransform& parent,
        const SimilarityTransform& local,
        SimilarityTransform& output) {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                output.linear[row][column] = 0.0;
                for (std::size_t inner = 0; inner < 3; ++inner) {
                    output.linear[row][column] += parent.linear[row][inner]
                        * local.linear[inner][column];
                }
            }
            output.translation[row] = parent.translation[row];
            for (std::size_t inner = 0; inner < 3; ++inner) {
                output.translation[row] += parent.linear[row][inner]
                    * local.translation[inner];
            }
        }
        double cumulativeScaleSquared = 0.0;
        for (std::size_t row = 0; row < 3; ++row) {
            cumulativeScaleSquared += output.linear[row][0]
                * output.linear[row][0];
            if (!std::isfinite(output.translation[row])
                || std::abs(output.translation[row]) > 1'000.0) {
                return Fail(
                    PreflightStatus::ResourceLimit,
                    "GLB cumulative translation exceeds the document bound.");
            }
            for (const double value : output.linear[row]) {
                if (!std::isfinite(value)) {
                    return Fail(
                        PreflightStatus::ResourceLimit,
                        "GLB cumulative transform overflowed.");
                }
            }
        }
        if (!std::isfinite(cumulativeScaleSquared)
            || cumulativeScaleSquared < 1.0e-18
            || cumulativeScaleSquared > 1.0e12) {
            return Fail(
                PreflightStatus::ResourceLimit,
                "GLB cumulative scale is singular or exceeds its budget.");
        }
        return true;
    }

    bool ValidateTransformedPositionBounds(
        const PositionBounds& bounds,
        const SimilarityTransform& transform) {
        if (!bounds.valid) {
            return Fail(
                PreflightStatus::Invalid,
                "GLB POSITION payload bounds were not validated.");
        }
        // OCCT imports glTF metres into the document's millimetre system. A
        // 1,000 m coordinate therefore reaches Core's 1,000,000 mm ceiling.
        constexpr double kMaximumGLTFCoordinateMetres = 1'000.0;
        for (std::uint8_t corner = 0; corner < 8; ++corner) {
            std::array<double, 3> point{};
            for (std::size_t axis = 0; axis < 3; ++axis) {
                point[axis] = (corner & (1U << axis)) != 0
                    ? bounds.maximum[axis]
                    : bounds.minimum[axis];
            }
            for (std::size_t row = 0; row < 3; ++row) {
                double transformed = transform.translation[row];
                for (std::size_t column = 0; column < 3; ++column) {
                    transformed += transform.linear[row][column] * point[column];
                }
                if (!std::isfinite(transformed)
                    || std::abs(transformed) > kMaximumGLTFCoordinateMetres) {
                    return Fail(
                        PreflightStatus::ResourceLimit,
                        "Transformed GLB POSITION exceeds the document coordinate bound.");
                }
            }
        }
        return true;
    }

    static bool Accumulate(
        std::uint64_t& total,
        const std::uint64_t increment,
        const std::uint64_t ceiling) {
        if (AddWouldOverflow(total, increment) || total + increment > ceiling) {
            return false;
        }
        total += increment;
        return true;
    }

    int myDescriptor;
    ByteRange myBinaryChunk;
    const std::atomic_bool *myCancelled;
    PreflightResult& myResult;
    std::uint64_t myDeclaredBufferLength = 0;
    std::uint64_t myImageCount = 0;
    std::uint64_t mySamplerCount = 0;
    std::uint64_t myTextureCount = 0;
    std::uint64_t myMaterialCount = 0;
    std::vector<bool> myMaterialUsesTexture;
    std::vector<BufferViewRecord> myBufferViews;
    std::vector<AccessorRecord> myAccessors;
    std::vector<bool> myAccessorBufferViews;
    std::vector<std::uint8_t> myAccessorPayloadFlags;
    std::vector<std::uint64_t> myAccessorIndexMaximum;
    std::vector<PositionBounds> myPositionBounds;
    std::vector<MeshRecord> myMeshes;
    std::vector<std::vector<std::uint64_t>> myNodeChildren;
    std::vector<std::uint64_t> myNodeParents;
    std::vector<std::int64_t> myNodeMeshes;
    std::vector<SimilarityTransform> myNodeTransforms;
};

bool ParseChunkTable(
    const int descriptor,
    const std::uint64_t expectedSize,
    const std::atomic_bool *cancelled,
    PreflightResult& result) {
    std::array<std::uint8_t, 12> header{};
    if (!PreadExactly(
            descriptor,
            header.data(),
            header.size(),
            0,
            cancelled,
            result)) {
        return false;
    }
    const std::uint32_t magic = ReadLE32(header.data());
    const std::uint32_t version = ReadLE32(header.data() + 4);
    const std::uint32_t declaredLength = ReadLE32(header.data() + 8);
    if (magic != kGLBMagic || version != 2) {
        result.status = PreflightStatus::Unsupported;
        result.message = "Source is not a supported GLB 2.0 container.";
        return false;
    }
    if (declaredLength != expectedSize) {
        result.status = PreflightStatus::Invalid;
        result.message = "GLB header length does not match the pinned file.";
        return false;
    }

    std::uint64_t cursor = header.size();
    std::size_t chunkIndex = 0;
    while (cursor < expectedSize) {
        if (IsCancelled(cancelled)) {
            result.status = PreflightStatus::Cancelled;
            result.message = "GLB preflight was cancelled.";
            return false;
        }
        if (expectedSize - cursor < 8) {
            result.status = PreflightStatus::Invalid;
            result.message = "GLB has a truncated chunk header.";
            return false;
        }
        std::array<std::uint8_t, 8> chunkHeader{};
        if (!PreadExactly(
                descriptor,
                chunkHeader.data(),
                chunkHeader.size(),
                cursor,
                cancelled,
                result)) {
            return false;
        }
        const std::uint64_t chunkLength = ReadLE32(chunkHeader.data());
        const std::uint32_t chunkType = ReadLE32(chunkHeader.data() + 4);
        cursor += chunkHeader.size();
        if (chunkLength == 0 || (chunkLength & 3U) != 0
            || chunkLength > expectedSize - cursor) {
            result.status = PreflightStatus::Invalid;
            result.message = "GLB chunk length is zero, unaligned, or out of range.";
            return false;
        }
        if (chunkIndex == 0) {
            if (chunkType != kJSONChunkType) {
                result.status = PreflightStatus::Invalid;
                result.message = "GLB first chunk is not JSON.";
                return false;
            }
            if (chunkLength > kMaximumJSONBytes) {
                result.status = PreflightStatus::ResourceLimit;
                result.message = "GLB JSON chunk exceeds 4 MiB.";
                return false;
            }
            result.jsonChunk = {cursor, chunkLength};
        } else if (chunkIndex == 1) {
            if (chunkType != kBINChunkType) {
                result.status = PreflightStatus::Unsupported;
                result.message = "GLB second chunk is not the embedded BIN chunk.";
                return false;
            }
            result.hasBinaryChunk = true;
            result.binaryChunk = {cursor, chunkLength};
        } else {
            result.status = PreflightStatus::Unsupported;
            result.message = "GLB contains more than the JSON and one BIN chunk.";
            return false;
        }
        cursor += chunkLength;
        ++chunkIndex;
    }
    if (cursor != expectedSize || chunkIndex == 0) {
        result.status = PreflightStatus::Invalid;
        result.message = "GLB chunks do not terminate exactly at the declared end.";
        return false;
    }
    if (!result.hasBinaryChunk) {
        result.status = PreflightStatus::Unsupported;
        result.message = "Self-contained GLB import requires one BIN chunk.";
        return false;
    }
    return true;
}

} // namespace

PreflightResult PreflightPinnedGLB(
    const int descriptor,
    const std::uint64_t expectedSize,
    const std::atomic_bool *cancelled) noexcept {
    PreflightResult result;
    @autoreleasepool {
        @try {
          try {
            if (IsCancelled(cancelled)) {
                result.status = PreflightStatus::Cancelled;
                result.message = "GLB preflight was cancelled.";
                return result;
            }
            if (descriptor < 0) {
                result.status = PreflightStatus::IOFailure;
                result.message = "GLB descriptor is invalid.";
                return result;
            }
            if (expectedSize < 20 || expectedSize > kMaximumSourceBytes) {
                result.status = expectedSize > kMaximumSourceBytes
                    ? PreflightStatus::ResourceLimit
                    : PreflightStatus::Invalid;
                result.message = expectedSize > kMaximumSourceBytes
                    ? "GLB source exceeds 32 MiB."
                    : "GLB source is too short.";
                return result;
            }
            struct stat initialStatus {};
            if (fstat(descriptor, &initialStatus) != 0
                || !S_ISREG(initialStatus.st_mode)
                || initialStatus.st_size < 0
                || static_cast<std::uint64_t>(initialStatus.st_size) != expectedSize) {
                result.status = PreflightStatus::IOFailure;
                result.message = "GLB descriptor is not the expected pinned regular file.";
                return result;
            }
            if (!ParseChunkTable(descriptor, expectedSize, cancelled, result)) {
                return result;
            }

            std::vector<std::uint8_t> jsonBytes(
                static_cast<std::size_t>(result.jsonChunk.length));
            if (!PreadExactly(
                    descriptor,
                    jsonBytes.data(),
                    jsonBytes.size(),
                    result.jsonChunk.offset,
                    cancelled,
                    result)
                || !LexicallyValidateJSON(jsonBytes, cancelled, result)) {
                return result;
            }
            JSONShapeValidator shapeValidator(jsonBytes, cancelled, result);
            if (!shapeValidator.Validate()) {
                return result;
            }

            NSData *jsonData = [NSData dataWithBytes:jsonBytes.data()
                                              length:jsonBytes.size()];
            NSError *jsonError = nil;
            id jsonObject = [NSJSONSerialization JSONObjectWithData:jsonData
                                                             options:0
                                                               error:&jsonError];
            NSDictionary *root = Dictionary(jsonObject);
            if (root == nil || jsonError != nil) {
                result.status = PreflightStatus::Invalid;
                result.message = "GLB JSON is not a valid top-level object.";
                return result;
            }
            if (IsCancelled(cancelled)) {
                result.status = PreflightStatus::Cancelled;
                result.message = "GLB preflight was cancelled.";
                return result;
            }

            SemanticValidator validator(
                descriptor,
                result.binaryChunk,
                cancelled,
                result);
            if (!validator.Validate(root)) {
                return result;
            }

            struct stat finalStatus {};
            if (fstat(descriptor, &finalStatus) != 0
                || !SamePinnedFile(initialStatus, finalStatus)) {
                result.status = PreflightStatus::IOFailure;
                result.message = "Pinned GLB changed during preflight.";
                return result;
            }
            result.status = PreflightStatus::Valid;
            result.message = "GLB 2.0 preflight passed.";
            return result;
          } catch (const std::bad_alloc&) {
            result.status = PreflightStatus::ResourceLimit;
            result.message = "GLB preflight could not allocate bounded working memory.";
            return result;
          } catch (...) {
            result.status = PreflightStatus::Invalid;
            result.message = "GLB preflight failed safely.";
            return result;
          }
        } @catch (NSException *exception) {
            (void)exception;
            result.status = PreflightStatus::Invalid;
            result.message = "GLB preflight rejected an exceptional JSON object.";
            return result;
        }
    }
}

} // namespace core3d::gltf
