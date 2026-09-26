#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace shapeyard::text_relief_interface {

constexpr std::size_t MaximumResourceBytes = 1'048'576;
constexpr std::size_t MaximumTextBytes = 256;
constexpr std::size_t MaximumGlyphs = 64;
constexpr std::size_t MaximumContours = 256;
constexpr std::size_t MaximumContoursPerComponent = 32;
constexpr std::size_t MaximumCurves = 128;

enum class ResourceKind : std::uint8_t { FontOutlineV1 = 1, EmblemVectorV1 = 2 };
enum class Alignment : std::uint8_t { Left = 1, Center = 2, Right = 3 };

struct ResourceReference final {
  std::string handle;
  std::string sha256;
  ResourceKind kind = ResourceKind::FontOutlineV1;
  std::uint64_t encodedBytes = 0;
};

struct TextCommand final {
  ResourceReference resource;
  std::string textUTF8;
  std::string script;
  std::string planeFrameReceipt;
  double heightMM = 0.0;
  double trackingMM = 0.0;
  Alignment alignment = Alignment::Left;
  double depthMM = 0.0;
};

struct EmblemCommand final {
  ResourceReference resource;
  std::string emblemRoot;
  std::string planeFrameReceipt;
  double heightMM = 0.0;
  double depthMM = 0.0;
};

inline bool IsIdentifier(const std::string& value) noexcept {
  if (value.empty() || value.size() > 128) return false;
  for (const unsigned char c : value) {
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '.' || c == '_' || c == ':' || c == '-')) return false;
  }
  return true;
}

inline bool IsDigest(const std::string& value) noexcept {
  if (value.size() != 64) return false;
  for (const unsigned char c : value) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

inline bool IsPinnedResource(const ResourceReference& value, ResourceKind expected) noexcept {
  return value.kind == expected && IsIdentifier(value.handle) && IsDigest(value.sha256) &&
         value.encodedBytes > 0 && value.encodedBytes <= MaximumResourceBytes;
}

inline bool IsBoundedText(const std::string& value) noexcept {
  if (value.empty() || value.size() > MaximumTextBytes) return false;
  std::size_t scalars = 0;
  for (std::size_t i = 0; i < value.size();) {
    const auto c = static_cast<unsigned char>(value[i]);
    if (c < 0x20 || (c >= 0x7f && c <= 0x9f)) return false;
    std::size_t width = 0;
    if (c <= 0x7f) width = 1;
    else if (c >= 0xc2 && c <= 0xdf) width = 2;
    else if (c >= 0xe0 && c <= 0xef) width = 3;
    else if (c >= 0xf0 && c <= 0xf4) width = 4;
    else return false;
    if (i + width > value.size()) return false;
    for (std::size_t j = 1; j < width; ++j) {
      const auto continuation = static_cast<unsigned char>(value[i + j]);
      if ((continuation & 0xc0) != 0x80) return false;
    }
    if ((width == 3 && c == 0xe0 && static_cast<unsigned char>(value[i + 1]) < 0xa0) ||
        (width == 3 && c == 0xed && static_cast<unsigned char>(value[i + 1]) >= 0xa0) ||
        (width == 4 && c == 0xf0 && static_cast<unsigned char>(value[i + 1]) < 0x90) ||
        (width == 4 && c == 0xf4 && static_cast<unsigned char>(value[i + 1]) >= 0x90)) return false;
    i += width;
    if (++scalars > MaximumGlyphs) return false;
  }
  return true;
}

inline bool IsHeight(double value) noexcept {
  return std::isfinite(value) && value >= 0.1 && value <= 100'000.0;
}
inline bool IsDepth(double value) noexcept {
  return std::isfinite(value) && value >= 0.01 && value <= 10'000.0;
}

inline bool Admit(const TextCommand& value) noexcept {
  return IsPinnedResource(value.resource, ResourceKind::FontOutlineV1) &&
         IsBoundedText(value.textUTF8) && value.script == "latin" &&
         IsIdentifier(value.planeFrameReceipt) && IsHeight(value.heightMM) &&
         std::isfinite(value.trackingMM) && value.trackingMM >= -10'000.0 &&
         value.trackingMM <= 10'000.0 && IsDepth(value.depthMM);
}

inline bool Admit(const EmblemCommand& value) noexcept {
  return IsPinnedResource(value.resource, ResourceKind::EmblemVectorV1) &&
         IsIdentifier(value.emblemRoot) && IsIdentifier(value.planeFrameReceipt) &&
         IsHeight(value.heightMM) && IsDepth(value.depthMM);
}

}  // namespace shapeyard::text_relief_interface
