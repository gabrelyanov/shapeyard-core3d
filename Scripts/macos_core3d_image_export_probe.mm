#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>

#include "Core3DMacImageExport.hxx"

#include <Image_PixMap.hxx>
#include <Standard_Failure.hxx>

#include <array>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

namespace
{
constexpr std::size_t kWidth = 2;
constexpr std::size_t kHeight = 3;
constexpr std::size_t kStride = 8; // Six RGB bytes plus two sentinel padding bytes.

struct Pixel
{
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
  std::uint8_t a;
};

using SourceBuffer = std::array<std::uint8_t, kStride * kHeight>;
using DecodedBuffer = std::array<Pixel, kWidth * kHeight>;

constexpr std::array<Pixel, kWidth * kHeight> kExpected = {{
  {255,   0,   0, 255}, {  0, 255,   0, 255},
  {  0,   0, 255, 255}, {128,  96,  64, 255},
  {255,   0, 255, 255}, {  0, 255, 255, 255},
}};

void require(bool theCondition, const char* theMessage)
{
  if (!theCondition)
  {
    throw std::runtime_error(theMessage);
  }
}

SourceBuffer makeSource(const bool theTopDown)
{
  SourceBuffer aBytes;
  aBytes.fill(0xA5); // Padding must never become a decoded pixel channel.
  for (std::size_t aMemoryRow = 0; aMemoryRow < kHeight; ++aMemoryRow)
  {
    const std::size_t aVisualRow = theTopDown
      ? aMemoryRow
      : (kHeight - 1 - aMemoryRow);
    for (std::size_t aColumn = 0; aColumn < kWidth; ++aColumn)
    {
      const Pixel& aPixel = kExpected[aVisualRow * kWidth + aColumn];
      std::uint8_t* aDestination =
        aBytes.data() + aMemoryRow * kStride + aColumn * 3;
      aDestination[0] = aPixel.r;
      aDestination[1] = aPixel.g;
      aDestination[2] = aPixel.b;
    }
  }
  return aBytes;
}

DecodedBuffer decodePNG(NSData* theData)
{
  require(theData != nil && theData.length >= 8, "PNG output is empty");
  const std::uint8_t* aSignature =
    static_cast<const std::uint8_t*>(theData.bytes);
  require(aSignature[0] == 0x89 && aSignature[1] == 'P'
          && aSignature[2] == 'N' && aSignature[3] == 'G'
          && aSignature[4] == 0x0D && aSignature[5] == 0x0A
          && aSignature[6] == 0x1A && aSignature[7] == 0x0A,
          "output does not have a PNG signature");

  NSDictionary* aSourceOptions = @{
    (__bridge NSString*)kCGImageSourceShouldCache: @NO,
  };
  CGImageSourceRef aSource = CGImageSourceCreateWithData(
    (__bridge CFDataRef)theData,
    (__bridge CFDictionaryRef)aSourceOptions);
  require(aSource != nullptr, "ImageIO could not create a PNG source");
  const bool aComplete = CGImageSourceGetCount(aSource) == 1
    && CGImageSourceGetStatus(aSource) == kCGImageStatusComplete
    && CGImageSourceGetStatusAtIndex(aSource, 0) == kCGImageStatusComplete;
  require(aComplete, "ImageIO reports an incomplete PNG");

  CGImageRef anImage = CGImageSourceCreateImageAtIndex(aSource, 0, nullptr);
  CFRelease(aSource);
  require(anImage != nullptr, "ImageIO could not decode the PNG");
  const bool hasExpectedDimensions = CGImageGetWidth(anImage) == kWidth
    && CGImageGetHeight(anImage) == kHeight;
  require(hasExpectedDimensions, "decoded PNG dimensions differ");

  DecodedBuffer aPixels = {};
  CGColorSpaceRef aColorSpace =
    CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
  require(aColorSpace != nullptr, "sRGB color-space creation failed");
  const CGBitmapInfo aBitmapInfo = static_cast<CGBitmapInfo>(
    kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
  CGContextRef aContext = CGBitmapContextCreate(
    aPixels.data(), kWidth, kHeight, 8, kWidth * sizeof(Pixel),
    aColorSpace, aBitmapInfo);
  CGColorSpaceRelease(aColorSpace);
  require(aContext != nullptr, "RGBA decode context creation failed");
  CGContextSetBlendMode(aContext, kCGBlendModeCopy);
  // This matches the renderer texture ImageIO path: its first destination
  // scanline is treated as the visual top row and marked top-down for OCCT.
  CGContextDrawImage(aContext, CGRectMake(0, 0, kWidth, kHeight), anImage);
  CGContextRelease(aContext);
  CGImageRelease(anImage);
  return aPixels;
}

void requireExact(const DecodedBuffer& thePixels, const char* theMessage)
{
  for (std::size_t anIndex = 0; anIndex < kExpected.size(); ++anIndex)
  {
    const Pixel& anActual = thePixels[anIndex];
    const Pixel& anExpected = kExpected[anIndex];
    if (anActual.r != anExpected.r || anActual.g != anExpected.g
        || anActual.b != anExpected.b || anActual.a != anExpected.a)
    {
      throw std::runtime_error(theMessage);
    }
  }
}

void requireGammaDirection(const DecodedBuffer& theBaseline,
                           const DecodedBuffer& theGamma)
{
  const Pixel& aBaseline = theBaseline[3]; // Interior RGB sample: 128,96,64.
  const Pixel& aGamma = theGamma[3];
  // CIGammaAdjust(inputPower=2) should darken each non-extreme sRGB channel.
  // A four-code-value minimum is deliberately directional rather than a
  // platform-specific transfer-function oracle; it tolerates CI rounding and
  // color-management implementation differences without accepting a no-op.
  require(aGamma.r + 4 <= aBaseline.r
          && aGamma.g + 4 <= aBaseline.g
          && aGamma.b + 4 <= aBaseline.b,
          "gamma-enabled PNG did not directionally darken the interior sample");

  const Pixel& aRed = theGamma[0];
  require(aRed.r >= 254 && aRed.g <= 1 && aRed.b <= 1 && aRed.a == 255,
          "gamma conversion changed an extreme primary outside tolerance");
}

DecodedBuffer encodeFixture(const bool theTopDown,
                            const bool theAdjustGamma)
{
  SourceBuffer aSource = makeSource(theTopDown);
  Image_PixMap anImage;
  require(anImage.InitWrapper(Image_Format_RGB, aSource.data(),
                              kWidth, kHeight, kStride),
          "OCCT RGB wrapper initialization failed");
  anImage.SetTopDown(theTopDown);
  require(anImage.SizeRowBytes() == kStride,
          "OCCT wrapper did not preserve row padding");
  require(anImage.IsTopDown() == theTopDown,
          "OCCT wrapper row orientation differs");
  NSData* aPNG = Core3DMacPNGData(anImage, theAdjustGamma);
  return decodePNG(aPNG);
}

int runProbe()
{
  Image_PixMap anEmpty;
  require(Core3DMacPNGData(anEmpty, false) == nil,
          "empty OCCT image was accepted");

  Image_PixMap anUnsupported;
  require(anUnsupported.InitZero(Image_Format_RGBA, 1, 1, 4),
          "unsupported-format fixture initialization failed");
  require(Core3DMacPNGData(anUnsupported, false) == nil,
          "non-RGB OCCT image was accepted");

  const DecodedBuffer aTopDown = encodeFixture(true, false);
  const DecodedBuffer aBottomUp = encodeFixture(false, false);
  requireExact(aTopDown,
               "top-down gamma-disabled PNG changed position or RGB channels");
  requireExact(aBottomUp,
               "bottom-up gamma-disabled PNG changed position or RGB channels");

  const DecodedBuffer aGammaTopDown = encodeFixture(true, true);
  const DecodedBuffer aGammaBottomUp = encodeFixture(false, true);
  requireGammaDirection(aTopDown, aGammaTopDown);
  requireGammaDirection(aBottomUp, aGammaBottomUp);
  for (std::size_t anIndex = 0; anIndex < aGammaTopDown.size(); ++anIndex)
  {
    const Pixel& a = aGammaTopDown[anIndex];
    const Pixel& b = aGammaBottomUp[anIndex];
    require(a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a,
            "gamma output depends on OCCT row storage orientation");
  }

  std::puts("PASS: macOS PNG export orientation, padding, channels and gamma direction");
  return 0;
}
} // namespace

int main()
{
  @autoreleasepool
  {
    try
    {
      return runProbe();
    }
    catch (const Standard_Failure& theFailure)
    {
      std::fprintf(stderr, "FAIL (OCCT): %s\n", theFailure.GetMessageString());
    }
    catch (const std::exception& theFailure)
    {
      std::fprintf(stderr, "FAIL: %s\n", theFailure.what());
    }
    return 1;
  }
}

