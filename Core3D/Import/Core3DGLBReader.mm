//
//  Core3DGLBReader.mm
//  Core3D
//

#include "Core3DGLBReader.hpp"
#include "Core3DGLBSourceAdapter.hpp"
#include "../OCCTKit/NativeAuthoredFrameGeometry.hxx"

#include "../Common/Core3DMobileResourceLimits.h"
#include "../OCCTKit/OcctDocument.h"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Message_ProgressIndicator.hxx>
#include <NCollection_DataMap.hxx>
#include <OSD_FileSystem.hxx>
#include <OSD_StreamBuffer.hxx>
#include <Poly_ListOfTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <RWGltf_CafReader.hxx>
#include <RWGltf_GltfLatePrimitiveArray.hxx>
#include <RWGltf_GltfPrimArrayData.hxx>
#if DEBUG
#include <TDF_Tool.hxx>
#endif
#include <RWGltf_TriangulationReader.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDataStd_Name.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ios>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <sstream>
#include <streambuf>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace core3d::gltf {
namespace {

constexpr std::uint64_t kMaximumSourceBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumObjects =
    static_cast<std::uint64_t>(limits::kMaximumLeafPresentations);
constexpr std::uint64_t kMaximumVerticesPerObject = 1'000'000ULL;
constexpr std::uint64_t kMaximumVerticesPerDocument = 1'500'000ULL;
constexpr std::uint64_t kMaximumIndicesPerObject = 3'000'000ULL;
constexpr std::uint64_t kMaximumIndicesPerDocument = 4'500'000ULL;
constexpr std::uint64_t kMaximumTraversalNodes = 32'768ULL;
constexpr std::uint64_t kMaximumTraversalDepth = 128ULL;
constexpr std::size_t kMaximumMaterialDefinitions = 2'048;
constexpr Standard_Size kMaximumSerializedTextureOccurrenceBytes =
    128ULL * 1024ULL * 1024ULL;
constexpr Standard_Size kMaximumDecodedTextureResourceBytes =
    128ULL * 1024ULL * 1024ULL;
constexpr Standard_Integer kMaximumImportedNameCharacters = 4'096;
constexpr Standard_Integer kMaximumImportedNameUTF8Bytes =
    kMaximumImportedNameCharacters * 4;
constexpr Standard_Real kMaximumCoordinateMagnitude = 1.0e6;
constexpr std::size_t kReadBufferBytes = 16 * 1024;
constexpr const char *kPinnedToken = "core3d-pinned-glb";

bool IsCancelled(const std::atomic_bool *cancelled) noexcept {
    return cancelled != nullptr
        && cancelled->load(std::memory_order_acquire);
}

struct DescriptorReadState {
    std::shared_ptr<const GLBReaderSource> source;
    const std::atomic_bool *cancelled = nullptr;
    bool ioFailure = false;
};

class PreadStreamBuffer final : public std::streambuf {
public:
    PreadStreamBuffer(
        int descriptor,
        std::uint64_t size,
        std::uint64_t offset,
        const std::shared_ptr<DescriptorReadState>& state)
    : myDescriptor(descriptor),
      mySize(size),
      myBufferStart(offset),
      myState(state) {
        setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
    }

protected:
    int_type underflow() override {
        if (gptr() != nullptr && gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        const std::uint64_t position = LogicalPosition();
        if (position >= mySize || IsCancelled(myState->cancelled)) {
            myBufferStart = std::min(position, mySize);
            setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
            return traits_type::eof();
        }
        const std::uint64_t remaining = mySize - position;
        const std::size_t request = static_cast<std::size_t>(
            std::min<std::uint64_t>(remaining, myBuffer.size()));
        ssize_t count = -1;
        if (myState->source) {
            count = myState->source->Read(position,myBuffer.data(),request,
                myState->cancelled,myState->ioFailure) ? ssize_t(request) : -1;
        } else {
            do {
                count = ::pread(myDescriptor,myBuffer.data(),request,static_cast<off_t>(position));
            } while (count < 0 && errno == EINTR && !IsCancelled(myState->cancelled));
        }
        if (count <= 0) {
            if (!IsCancelled(myState->cancelled)) {
                myState->ioFailure = true;
            }
            myBufferStart = position;
            setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
            return traits_type::eof();
        }
        myBufferStart = position;
        setg(
            myBuffer.data(),
            myBuffer.data(),
            myBuffer.data() + count);
        return traits_type::to_int_type(*gptr());
    }

    std::streamsize xsgetn(char_type *destination, std::streamsize count) override {
        if (destination == nullptr || count <= 0) {
            return 0;
        }
        std::streamsize copied = 0;
        while (copied < count) {
            if (gptr() == nullptr || gptr() == egptr()) {
                if (traits_type::eq_int_type(underflow(), traits_type::eof())) {
                    break;
                }
            }
            const std::streamsize available = egptr() - gptr();
            const std::streamsize chunk = std::min(available, count - copied);
            std::memcpy(destination + copied, gptr(), static_cast<std::size_t>(chunk));
            gbump(static_cast<int>(chunk));
            copied += chunk;
        }
        return copied;
    }

    pos_type seekoff(
        off_type offset,
        std::ios_base::seekdir direction,
        std::ios_base::openmode mode) override {
        if ((mode & std::ios_base::in) == 0
            || (mode & std::ios_base::out) != 0) {
            return pos_type(off_type(-1));
        }
        const std::int64_t base = direction == std::ios_base::beg
            ? 0
            : direction == std::ios_base::cur
                ? static_cast<std::int64_t>(LogicalPosition())
                : direction == std::ios_base::end
                    ? static_cast<std::int64_t>(mySize)
                    : -1;
        if (base < 0) {
            return pos_type(off_type(-1));
        }
        const __int128 destination = static_cast<__int128>(base)
            + static_cast<__int128>(offset);
        if (destination < 0
            || destination > static_cast<__int128>(mySize)) {
            return pos_type(off_type(-1));
        }
        myBufferStart = static_cast<std::uint64_t>(destination);
        setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
        return pos_type(static_cast<off_type>(myBufferStart));
    }

    pos_type seekpos(pos_type position, std::ios_base::openmode mode) override {
        return seekoff(
            static_cast<off_type>(position),
            std::ios_base::beg,
            mode);
    }

    std::streamsize showmanyc() override {
        const std::uint64_t position = LogicalPosition();
        const std::uint64_t remaining = position < mySize ? mySize - position : 0;
        return static_cast<std::streamsize>(std::min<std::uint64_t>(
            remaining,
            static_cast<std::uint64_t>(
                std::numeric_limits<std::streamsize>::max())));
    }

private:
    std::uint64_t LogicalPosition() const noexcept {
        if (eback() == nullptr || gptr() == nullptr) {
            return myBufferStart;
        }
        return myBufferStart
            + static_cast<std::uint64_t>(gptr() - eback());
    }

    int myDescriptor;
    std::uint64_t mySize;
    std::uint64_t myBufferStart;
    std::shared_ptr<DescriptorReadState> myState;
    std::array<char, kReadBufferBytes> myBuffer{};
};

class PinnedGLBFileSystem final : public OSD_FileSystem {
    DEFINE_STANDARD_RTTI_INLINE(PinnedGLBFileSystem, OSD_FileSystem)

public:
    PinnedGLBFileSystem(
        int descriptor,
        std::uint64_t size,
        const std::shared_ptr<DescriptorReadState>& state)
    : myDescriptor(descriptor), mySize(size), myState(state) {
    }

    Standard_Boolean IsSupportedPath(
        const TCollection_AsciiString& url) const override {
        return url.IsEqual(kPinnedToken);
    }

    Standard_Boolean IsOpenIStream(
        const std::shared_ptr<std::istream>& stream) const override {
        const std::shared_ptr<OSD_IStreamBuffer> pinned =
            std::dynamic_pointer_cast<OSD_IStreamBuffer>(stream);
        return pinned != nullptr
            && pinned->Url() == kPinnedToken
            && !pinned->bad();
    }

    Standard_Boolean IsOpenOStream(
        const std::shared_ptr<std::ostream>&) const override {
        return Standard_False;
    }

    std::shared_ptr<std::istream> OpenIStream(
        const TCollection_AsciiString& url,
        const std::ios_base::openmode mode,
        const int64_t offset = 0,
        const std::shared_ptr<std::istream>& =
            std::shared_ptr<std::istream>()) override {
        if (!IsSupportedPath(url)
            || offset < 0
            || static_cast<std::uint64_t>(offset) > mySize
            || (mode & std::ios_base::out) != 0) {
            return {};
        }
        const std::shared_ptr<std::streambuf> buffer = OpenStreamBuffer(
            url, mode | std::ios_base::in, offset, nullptr);
        if (buffer == nullptr) {
            return {};
        }
        return std::make_shared<OSD_IStreamBuffer>(kPinnedToken, buffer);
    }

    std::shared_ptr<std::ostream> OpenOStream(
        const TCollection_AsciiString&,
        const std::ios_base::openmode) override {
        return {};
    }

    std::shared_ptr<std::streambuf> OpenStreamBuffer(
        const TCollection_AsciiString& url,
        const std::ios_base::openmode mode,
        const int64_t offset = 0,
        int64_t *outSize = nullptr) override {
        if (!IsSupportedPath(url)
            || offset < 0
            || static_cast<std::uint64_t>(offset) > mySize
            || (mode & std::ios_base::out) != 0
            || IsCancelled(myState->cancelled)) {
            return {};
        }
        if (outSize != nullptr) {
            *outSize = static_cast<int64_t>(mySize);
        }
        return std::make_shared<PreadStreamBuffer>(
            myDescriptor,
            mySize,
            static_cast<std::uint64_t>(offset),
            myState);
    }

private:
    int myDescriptor;
    std::uint64_t mySize;
    std::shared_ptr<DescriptorReadState> myState;
};

bool ReadExact(
    int descriptor,
    std::uint64_t offset,
    Standard_Byte *destination,
    std::size_t count,
    const std::atomic_bool *cancelled) noexcept {
    std::size_t completed = 0;
    while (completed < count) {
        if (IsCancelled(cancelled)) {
            return false;
        }
        const ssize_t readCount = ::pread(
            descriptor,
            destination + completed,
            count - completed,
            static_cast<off_t>(offset + completed));
        if (readCount < 0 && errno == EINTR) {
            continue;
        }
        if (readCount <= 0) {
            return false;
        }
        completed += static_cast<std::size_t>(readCount);
    }
    return true;
}

bool AddWithin(
    std::uint64_t& value,
    std::uint64_t increment,
    std::uint64_t maximum) noexcept {
    if (increment > maximum || value > maximum - increment) {
        return false;
    }
    value += increment;
    return true;
}

bool IsFiniteTransform(const gp_Trsf& transform) noexcept {
    for (Standard_Integer row = 1; row <= 3; ++row) {
        for (Standard_Integer column = 1; column <= 4; ++column) {
            if (!std::isfinite(transform.Value(row, column))) {
                return false;
            }
        }
    }
    return true;
}

bool IsFiniteBounded(Standard_Real value) noexcept {
    return std::isfinite(value)
        && std::abs(value) <= kMaximumCoordinateMagnitude;
}

TCollection_AsciiString BoundedAsciiName(
    const TCollection_AsciiString& name,
    const Standard_Integer maximumBytes = kMaximumImportedNameUTF8Bytes) {
    if (maximumBytes <= 0) {
        return {};
    }
    if (name.Length() <= maximumBytes) {
        return name;
    }
    const unsigned char *bytes = reinterpret_cast<const unsigned char *>(
        name.ToCString());
    Standard_Integer end = maximumBytes;
    while (end > 0
           && (bytes[end] & 0xC0U) == 0x80U) {
        --end;
    }
    return end > 0 ? name.SubString(1, end) : TCollection_AsciiString();
}

TCollection_ExtendedString PersistentName(const TCollection_AsciiString& name) {
    TCollection_ExtendedString persistent(name, Standard_True);
    if (persistent.Length() > kMaximumImportedNameCharacters) {
        persistent.Trunc(kMaximumImportedNameCharacters);
    }
    return persistent;
}

struct TextureRangeKey {
    std::uint64_t offset = 0;
    std::uint64_t length = 0;

    bool operator<(const TextureRangeKey& other) const noexcept {
        return offset < other.offset
            || (offset == other.offset && length < other.length);
    }
};

class PinnedCafReader final : public RWGltf_CafReader {
public:
    PinnedCafReader(
        int descriptor,
        std::uint64_t size,
        const PreflightResult& preflight,
        const PreflightResult& originalPreflight,
        std::uint64_t originalSize,
        const std::atomic_bool *cancelled,
        const std::shared_ptr<DescriptorReadState>& streamState
#if DEBUG
        , GLBDebugTrace* debugTrace
#endif
        )
    : myDescriptor(descriptor),
      mySize(size),
      myPreflight(preflight),
      myOriginalPreflight(originalPreflight),
      myOriginalSize(originalSize),
      myCancelled(cancelled),
      myStreamState(streamState),
      myFileSystem(new PinnedGLBFileSystem(descriptor, size, streamState)) {
#if DEBUG
        myDebugTrace = debugTrace;
#endif
        SetParallel(false);
        SetFillIncompleteDocument(Standard_False);
        SetSkipEmptyNodes(true);
        SetLoadAllScenes(false);
        SetMeshNameAsFallback(true);
        SetDoublePrecision(false);
        SetToSkipLateDataLoading(false);
        SetToKeepLateData(false);
        SetSystemCoordinateSystem(RWMesh_CoordinateSystem_Zup);
    }

    bool Succeeded() const noexcept {
        return myStatus == GLBReadStatus::Imported;
    }

    GLBReadStatus Status() const noexcept {
        return myStatus;
    }

    const std::string& FailureMessage() const noexcept {
        return myMessage;
    }

    std::uint64_t ObjectCount() const noexcept { return myObjectCount; }
    std::uint64_t VertexCount() const noexcept { return myVertexCount; }
    std::uint64_t IndexCount() const noexcept { return myIndexCount; }
    std::vector<GLBImportedFrames> TakeAuthoredFrames() { return std::move(myImportedFrames); }

protected:
    Standard_Boolean readLateData(
        NCollection_Vector<TopoDS_Face>& faces,
        const TCollection_AsciiString& file,
        const Message_ProgressRange& progress) override {
        if (!file.IsEqual(kPinnedToken)) {
            Fail(GLBReadStatus::InternalFailure, "OCCT changed the pinned GLB token.");
            return Standard_False;
        }
        if (IsCancelled(myCancelled) || progress.UserBreak()) {
            Fail(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
            return Standard_False;
        }
        Handle(RWGltf_TriangulationReader) reader =
            Handle(RWGltf_TriangulationReader)::DownCast(
                createMeshReaderContext());
        if (reader.IsNull()) {
            Fail(GLBReadStatus::InternalFailure, "The GLB mesh reader is unavailable.");
            return Standard_False;
        }
        reader->SetFileName(kPinnedToken);
        updateLateDataReader(faces, reader);

        BRep_Builder builder;
        for (NCollection_Vector<TopoDS_Face>::Iterator iterator(faces);
             iterator.More(); iterator.Next()) {
            if (IsCancelled(myCancelled) || progress.UserBreak()) {
                Fail(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
                return Standard_False;
            }
            const TopoDS_Face& face = iterator.Value();
            TopLoc_Location location;
            const Poly_ListOfTriangulation& triangulations =
                BRep_Tool::Triangulations(face, location);
            if (triangulations.Size() != 1) {
                Fail(GLBReadStatus::Unsupported,
                     "Each admitted GLB primitive must have one triangulation.");
                return Standard_False;
            }
            const Handle(RWGltf_GltfLatePrimitiveArray) deferred =
                Handle(RWGltf_GltfLatePrimitiveArray)::DownCast(
                    triangulations.First());
            if (deferred.IsNull()
                || !deferred->HasDeferredData()
                || deferred->NbDeferredNodes() <= 0
                || deferred->NbDeferredTriangles() <= 0) {
                Fail(GLBReadStatus::Invalid,
                     "OCCT produced an invalid deferred GLB primitive.");
                return Standard_False;
            }
#if DEBUG
            if (myDebugTrace != nullptr) {
                if (myDebugTrace->primitives.size() >= 32 || deferred->Data().Length() > 8
                    || deferred->NbDeferredNodes() > 64 || deferred->NbDeferredTriangles() > 64) {
                    Fail(GLBReadStatus::ResourceLimit, "The private ownership probe exceeded its bound.");
                    return Standard_False;
                }
                GLBDebugPrimitive primitive;
                primitive.meshID = deferred->Id().ToCString();
                for (NCollection_Sequence<RWGltf_GltfPrimArrayData>::Iterator data(deferred->Data()); data.More(); data.Next()) {
                    const auto& value = data.Value();
                    primitive.streams.push_back({int(value.Type), value.Accessor.Id,
                        value.StreamOffset, value.StreamLength, value.Accessor.ByteOffset,
                        value.Accessor.Count, value.Accessor.ByteStride, value.StreamUri.IsEqual(kPinnedToken)});
                }
                myDebugTrace->primitives.push_back(std::move(primitive));
                myDebugSourceFaces.push_back(face);
            }
#endif
            std::size_t sourcePrimitive=0;
            if (myStreamState->source->IsAdapted() && !MatchReaderSource(deferred,sourcePrimitive)) {
                Fail(GLBReadStatus::Invalid,"An adapted GLB primitive lost its exact source-stream ownership.");
                return Standard_False;
            }
            const Handle(Poly_Triangulation) loaded =
                deferred->DetachedLoadDeferredData(myFileSystem);
            if (loaded.IsNull()
                || loaded->HasDeferredData()
                || !loaded->HasGeometry()
                || loaded->NbNodes() != deferred->NbDeferredNodes()
                || loaded->NbTriangles() != deferred->NbDeferredTriangles()) {
                Fail(
                    myStreamState->ioFailure
                        ? GLBReadStatus::IOFailure
                        : GLBReadStatus::Invalid,
                    "The pinned GLB primitive data could not be loaded exactly.");
                return Standard_False;
            }
            if (myStreamState->source->IsAdapted() && !CaptureSourceFace(face,sourcePrimitive,loaded)) {
                Fail(IsCancelled(myCancelled)?GLBReadStatus::Cancelled:GLBReadStatus::Invalid,
                    "The loaded GLB geometry or supplied basis does not match its pinned source.");
                return Standard_False;
            }
            builder.UpdateFace(face, loaded, Standard_True);
        }
        return Standard_True;
    }

    void fillDocument() override {
        if (!Succeeded() || myXdeDoc.IsNull() || myRootShapes.IsEmpty()) {
            if (Succeeded()) {
                Fail(GLBReadStatus::Invalid, "The GLB contains no supported geometry.");
            }
            return;
        }
        if (!CanonicalizeMaterials()) {
            return;
        }

        CafDocumentTools tools;
        tools.ShapeTool = XCAFDoc_DocumentTool::ShapeTool(myXdeDoc->Main());
        tools.ColorTool = XCAFDoc_DocumentTool::ColorTool(myXdeDoc->Main());
        tools.VisMaterialTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myXdeDoc->Main());
        if (tools.ShapeTool.IsNull()
            || tools.ColorTool.IsNull()
            || tools.VisMaterialTool.IsNull()) {
            Fail(GLBReadStatus::InternalFailure,
                 "The private GLB document tools are unavailable.");
            return;
        }

        for (TopTools_SequenceOfShape::Iterator iterator(myRootShapes);
             iterator.More() && Succeeded(); iterator.Next()) {
            const XCAFPrs_Style emptyStyle;
            if (!FlattenShape(
                    iterator.Value(),
                    TopLoc_Location(),
                    TCollection_AsciiString(),
                    emptyStyle,
                    tools,
                    0)) {
                break;
            }
        }
        if (!Succeeded()) {
            return;
        }
        tools.ShapeTool->UpdateAssemblies();

        if (myObjectCount != myPreflight.objectOccurrenceEstimate
            || myVertexCount != myPreflight.vertexEstimate
            || myIndexCount != myPreflight.indexEstimate
            || myAuthoredFrameBytes != myOriginalPreflight.suppliedFrameResidentEstimate) {
            Fail(GLBReadStatus::Invalid,
                 "The transferred GLB geometry does not match its bounded preflight.");
        }
    }

private:
    struct SourceFace {
        TopoDS_Face face;
        std::size_t primitive=0;
        std::vector<core3d::scene::Float4> axisFrames;
    };

    bool MatchReaderSource(const Handle(RWGltf_GltfLatePrimitiveArray)& deferred,
                           std::size_t& primitive) {
        primitive=std::numeric_limits<std::size_t>::max();
        const RWGltf_GltfPrimArrayData* position=nullptr;
        for (NCollection_Sequence<RWGltf_GltfPrimArrayData>::Iterator it(deferred->Data());it.More();it.Next())
            if (it.Value().Type==RWGltf_GltfArrayType_Position) {
                if (position) return false;
                position=&it.Value();
            }
        if (!position || position->StreamOffset<0) return false;
        for (std::size_t i=0;i<myPreflight.primitiveSources.size();++i)
            if (myPreflight.primitiveSources[i].position.offset==std::uint64_t(position->StreamOffset)) {
                if (primitive!=std::numeric_limits<std::size_t>::max()) return false;
                primitive=i;
            }
        if (primitive==std::numeric_limits<std::size_t>::max()) return false;
        const auto& expected=myPreflight.primitiveSources[primitive];
        unsigned mask=0;
        for (NCollection_Sequence<RWGltf_GltfPrimArrayData>::Iterator it(deferred->Data());it.More();it.Next()) {
            const auto& actual=it.Value(); const GLBAccessorSource* record=nullptr;
            RWGltf_GltfAccessorLayout layout=RWGltf_GltfAccessorLayout_UNKNOWN;unsigned bit=0;
            switch (actual.Type) {
                case RWGltf_GltfArrayType_Position:record=&expected.position;layout=RWGltf_GltfAccessorLayout_Vec3;bit=1;break;
                case RWGltf_GltfArrayType_Normal:record=&expected.normal;layout=RWGltf_GltfAccessorLayout_Vec3;bit=2;break;
                case RWGltf_GltfArrayType_TCoord0:record=&expected.uv;layout=RWGltf_GltfAccessorLayout_Vec2;bit=4;break;
                case RWGltf_GltfArrayType_Indices:record=&expected.indices;layout=RWGltf_GltfAccessorLayout_Scalar;bit=8;break;
                default:return false;
            }
            if (!record->IsPresent() || (mask&bit) || !actual.StreamUri.IsEqual(kPinnedToken)
                || actual.StreamOffset<0 || std::uint64_t(actual.StreamOffset)!=record->offset
                || actual.StreamLength<0 || std::uint64_t(actual.StreamLength)!=record->view.length
                || actual.Accessor.Count<0 || std::uint64_t(actual.Accessor.Count)!=record->count
                || actual.Accessor.ByteOffset<0 || std::uint64_t(actual.Accessor.ByteOffset)!=record->offset-record->view.offset
                || actual.Accessor.ByteStride<0
                || std::uint64_t(actual.Accessor.ByteStride?actual.Accessor.ByteStride:record->elementBytes)!=record->stride
                || actual.Accessor.Type!=layout || std::uint64_t(actual.Accessor.ComponentType)!=record->componentType
                || actual.Accessor.IsCompressed) return false;
            mask|=bit;
        }
        return mask==(1U|8U|(expected.normal.IsPresent()?2U:0U)|(expected.uv.IsPresent()?4U:0U))
            && std::uint64_t(deferred->NbDeferredNodes())==expected.position.count
            && std::uint64_t(deferred->NbDeferredTriangles())*3==expected.indices.count;
    }

    bool ReadSourcePayload(const GLBAccessorSource& source,std::vector<std::uint8_t>& bytes) {
        bytes.clear();
        if (!source.IsPresent() || source.count>core3d::scene::kMaximumTangentVertices
            || source.stride<source.elementBytes || source.stride>252
            || source.elementBytes==0 || source.elementBytes>16
            || source.view.offset>myOriginalSize || source.view.length>myOriginalSize-source.view.offset
            || source.offset<source.view.offset || source.offset-source.view.offset>source.view.length) return false;
        const auto span=(source.count-1)*source.stride+source.elementBytes;
        if (span>source.view.length-(source.offset-source.view.offset)) return false;
        bytes.resize(span);
        return ReadExact(myDescriptor,source.offset,bytes.data(),bytes.size(),myCancelled);
    }

    static bool SameSourceFloat(double actual,double expected) {
        const float rounded=float(expected);
        if (!std::isfinite(actual) || !std::isfinite(expected) || !std::isfinite(rounded)) return false;
        const double up=std::nextafter(rounded,std::numeric_limits<float>::infinity());
        const double down=std::nextafter(rounded,-std::numeric_limits<float>::infinity());
        const double tolerance=2*std::max(std::abs(up-rounded),std::abs(double(rounded)-down));
        return std::abs(actual-double(rounded))<=tolerance;
    }

    bool CaptureSourceFace(const TopoDS_Face& face,std::size_t primitive,
                           const Handle(Poly_Triangulation)& loaded) {
        if (mySourceFaces.size()>=2048 || primitive>=myOriginalPreflight.primitiveSources.size()) return false;
        const auto& source=myOriginalPreflight.primitiveSources[primitive];
        SourceFace captured{face,primitive,{}};
        if (source.tangent.IsPresent()) {
            if (!loaded->HasNormals() || !loaded->HasUVNodes()
                || source.position.count>core3d::scene::kMaximumTangentVertices
                || source.indices.count>core3d::scene::authored::kMaximumFrames) return false;
            std::vector<std::uint8_t> p,n,uv,t,ix;
            if (!ReadSourcePayload(source.position,p) || !ReadSourcePayload(source.normal,n)
                || !ReadSourcePayload(source.uv,uv) || !ReadSourcePayload(source.tangent,t)
                || !ReadSourcePayload(source.indices,ix)) return false;
            Graphic3d_Vec3 axisX(1,0,0),axisY(0,1,0),axisZ(0,0,1);
            const auto& converter=CoordinateSystemConverter();
            converter.TransformNormal(axisX);converter.TransformNormal(axisY);converter.TransformNormal(axisZ);
            const gp_Vec x(axisX.x(),axisX.y(),axisX.z()),y(axisY.x(),axisY.y(),axisY.z()),z(axisZ.x(),axisZ.y(),axisZ.z());
            const double determinant=x.Crossed(y).Dot(z);
            if (!std::isfinite(determinant) || std::abs(std::abs(determinant)-1)>1.e-4) return false;
            const float axisSign=determinant<0?-1.0f:1.0f;
            auto read=[](const std::vector<std::uint8_t>& bytes,const GLBAccessorSource& a,std::size_t i,int c) {
                return core3d::scene::authored::ReadFloat(bytes.data()+i*a.stride+c*4);
            };
            captured.axisFrames.reserve(source.position.count);
            for (std::size_t i=0;i<source.position.count;++i) {
                if ((i&255U)==0 && IsCancelled(myCancelled)) return false;
                gp_XYZ point(read(p,source.position,i,0),read(p,source.position,i,1),read(p,source.position,i,2));
                converter.TransformPosition(point);
                const auto actual=loaded->Node(int(i)+1);
                if (!SameSourceFloat(actual.X(),point.X()) || !SameSourceFloat(actual.Y(),point.Y())
                    || !SameSourceFloat(actual.Z(),point.Z())) return false;
                Graphic3d_Vec3 normal(read(n,source.normal,i,0),read(n,source.normal,i,1),read(n,source.normal,i,2));
                converter.TransformNormal(normal);gp_Dir expectedNormal(normal.x(),normal.y(),normal.z());
                const auto actualNormal=loaded->Normal(int(i)+1);
                if (!SameSourceFloat(actualNormal.X(),expectedNormal.X()) || !SameSourceFloat(actualNormal.Y(),expectedNormal.Y())
                    || !SameSourceFloat(actualNormal.Z(),expectedNormal.Z())) return false;
                const auto actualUV=loaded->UVNode(int(i)+1);
                if (!SameSourceFloat(actualUV.X(),read(uv,source.uv,i,0))
                    || !SameSourceFloat(actualUV.Y(),1.0-double(read(uv,source.uv,i,1)))) return false;
                Graphic3d_Vec3 tangent(read(t,source.tangent,i,0),read(t,source.tangent,i,1),read(t,source.tangent,i,2));
                converter.TransformNormal(tangent);gp_Dir direction(tangent.x(),tangent.y(),tangent.z());
                core3d::scene::Float4 frame{float(direction.X()),float(direction.Y()),float(direction.Z()),read(t,source.tangent,i,3)*axisSign};
                if (!core3d::scene::authored::ValidFrame(frame)
                    || std::abs(actualNormal.X()*frame.x+actualNormal.Y()*frame.y+actualNormal.Z()*frame.z)>1.e-4) return false;
                captured.axisFrames.push_back(frame);
            }
            for (int triangle=1;triangle<=loaded->NbTriangles();++triangle) {
                int nodes[3];loaded->Triangle(triangle).Get(nodes[0],nodes[1],nodes[2]);
                for (int corner=0;corner<3;++corner) {
                    const auto* value=ix.data()+(std::size_t(triangle-1)*3+corner)*source.indices.stride;
                    std::uint32_t index=value[0];
                    if (source.indices.elementBytes>=2) index|=std::uint32_t(value[1])<<8;
                    if (source.indices.elementBytes==4) index|=(std::uint32_t(value[2])<<16)|(std::uint32_t(value[3])<<24);
                    if (nodes[corner]<1 || std::uint64_t(nodes[corner]-1)!=index) return false;
                }
            }
        }
        mySourceFaces.push_back(std::move(captured));return true;
    }

    bool RecordImportedFrames(const TopoDS_Face& located,const TopoDS_Face& independent,const TDF_Label& label) {
        if (!myStreamState->source->IsAdapted()) return true;
        const SourceFace* source=nullptr;
        for (const auto& value:mySourceFaces) if (value.face.IsPartner(located)) {
            if (source) return false;source=&value;
        }
        if (!source) return false;
        if (source->axisFrames.empty()) return true;
        TopLoc_Location location;const auto& mesh=BRep_Tool::Triangulation(independent,location);
        if (mesh.IsNull() || !location.IsIdentity() || mesh->NbTriangles()>int(core3d::scene::kMaximumTangentTriangles)) return false;
        const auto transform=located.Location().Transformation();
        std::vector<core3d::scene::Float4> frames;frames.reserve(std::size_t(mesh->NbTriangles())*3);
        for (int triangle=1;triangle<=mesh->NbTriangles();++triangle) {
            if ((triangle&255)==0 && IsCancelled(myCancelled)) return false;
            int nodes[3];mesh->Triangle(triangle).Get(nodes[0],nodes[1],nodes[2]);
            for (int node:nodes) {
                if (node<1 || std::size_t(node)>source->axisFrames.size()) return false;
                const auto& original=source->axisFrames[std::size_t(node-1)];
                gp_Dir tangent(original.x,original.y,original.z);tangent.Transform(transform);
                core3d::scene::Float4 frame{float(tangent.X()),float(tangent.Y()),float(tangent.Z()),
                    original.w*(transform.IsNegative()?-1.0f:1.0f)};
                const auto normal=mesh->Normal(node);
                if (!core3d::scene::authored::ValidFrame(frame)
                    || std::abs(normal.X()*frame.x+normal.Y()*frame.y+normal.Z()*frame.z)>1.e-4) return false;
                frames.push_back(frame);
            }
        }
        core3d::scene::authored::GeometryIdentity identity;
        GLBImportedFrames record;record.label=label;
        if (!core3d::persistence::NativeAuthoredGeometryIdentity(independent,identity)
            || core3d::scene::authored::Encode(frames,identity,record.archive)!=core3d::scene::authored::ArchiveStatus::Valid)
            return false;
        std::vector<core3d::scene::Float4> verified;
        if (!core3d::persistence::DecodeNativeAuthoredFrames(independent,record.archive.data(),record.archive.size(),verified)
            || !AddWithin(myAuthoredFrameBytes,record.archive.size()+64U*frames.size(),64ULL*1024ULL*1024ULL)) return false;
        myImportedFrames.push_back(std::move(record));return true;
    }

    void Fail(GLBReadStatus status, const std::string& message) noexcept {
        if (myStatus == GLBReadStatus::Imported) {
            myStatus = status;
            myMessage = message;
        }
    }

    const EmbeddedImage *FindImage(
        std::uint64_t offset,
        std::uint64_t length) const noexcept {
        for (const EmbeddedImage& image : myPreflight.embeddedImages) {
            if (image.bytes.offset == offset && image.bytes.length == length) {
                return &image;
            }
        }
        return nullptr;
    }

    bool CanonicalizeTexture(Handle(Image_Texture)& texture) {
        if (texture.IsNull()) {
            return true;
        }
        if (!texture->DataBuffer().IsNull()
            || !texture->FilePath().IsEqual(kPinnedToken)
            || texture->FileOffset() < 0
            || texture->FileLength() <= 0) {
            Fail(GLBReadStatus::Unsupported,
                 "GLB textures must be embedded PNG or JPEG buffer views.");
            return false;
        }
        const std::uint64_t offset =
            static_cast<std::uint64_t>(texture->FileOffset());
        const std::uint64_t length =
            static_cast<std::uint64_t>(texture->FileLength());
        if (offset > mySize || length > mySize - offset) {
            Fail(GLBReadStatus::Invalid, "A GLB texture range is outside the pinned file.");
            return false;
        }
        const EmbeddedImage *image = FindImage(offset, length);
        if (image == nullptr
            || (image->mimeType != "image/png"
                && image->mimeType != "image/jpeg")) {
            Fail(GLBReadStatus::Unsupported,
                 "A GLB texture was not admitted by the semantic preflight.");
            return false;
        }

        const TextureRangeKey key{offset, length};
        const auto cached = myCanonicalTextures.find(key);
        if (cached != myCanonicalTextures.end()) {
            texture = cached->second;
            return true;
        }
        if (length > static_cast<std::uint64_t>(
                std::numeric_limits<std::size_t>::max())) {
            Fail(GLBReadStatus::ResourceLimit, "A GLB texture is too large.");
            return false;
        }
        std::vector<Standard_Byte> bytes(static_cast<std::size_t>(length));
        if (!myStreamState->source->Read(offset,bytes.data(),bytes.size(),
                myCancelled,myStreamState->ioFailure)) {
            Fail(
                IsCancelled(myCancelled)
                    ? GLBReadStatus::Cancelled
                    : GLBReadStatus::IOFailure,
                IsCancelled(myCancelled)
                    ? "The GLB import was cancelled."
                    : "An embedded GLB texture could not be read.");
            return false;
        }
        Handle(Image_Texture) authored;
        if (!Core3DCreateAuthoredTexture(
                bytes.data(),
                bytes.size(),
                image->mimeType,
                authored)
            || authored.IsNull()
            || !Core3DValidateAuthoredTexture(authored)) {
            Fail(GLBReadStatus::Invalid,
                 "An embedded GLB texture is not a valid bounded PNG or JPEG.");
            return false;
        }
        myCanonicalTextures.emplace(key, authored);
        texture = authored;
        return true;
    }

    bool ChargeTexture(const Handle(Image_Texture)& texture) {
        if (texture.IsNull()) {
            return true;
        }
        if (!Core3DAccumulateEmbeddedTextureBudget(
                texture,
                myTextureBudget,
                kMaximumSerializedTextureOccurrenceBytes,
                kMaximumDecodedTextureResourceBytes)) {
            Fail(GLBReadStatus::ResourceLimit,
                 "The GLB textures exceed mobile project limits.");
            return false;
        }
        return true;
    }

    bool CanonicalizeMaterial(const Handle(XCAFDoc_VisMaterial)& material) {
        if (material.IsNull()) {
            return true;
        }
        if (!material->RawName().IsNull()) {
            const TCollection_AsciiString boundedRawName = BoundedAsciiName(
                material->RawName()->String(),
                kMaximumImportedNameCharacters);
            material->SetRawName(
                boundedRawName.IsEmpty()
                    ? Handle(TCollection_HAsciiString)()
                    : new TCollection_HAsciiString(boundedRawName));
        }
        if (material->HasPbrMaterial()) {
            XCAFDoc_VisMaterialPBR pbr = material->PbrMaterial();
            if (!CanonicalizeTexture(pbr.BaseColorTexture)
                || !CanonicalizeTexture(pbr.MetallicRoughnessTexture)
                || !CanonicalizeTexture(pbr.EmissiveTexture)
                || !CanonicalizeTexture(pbr.OcclusionTexture)
                || !CanonicalizeTexture(pbr.NormalTexture)
                || !ChargeTexture(pbr.BaseColorTexture)
                || !ChargeTexture(pbr.MetallicRoughnessTexture)
                || !ChargeTexture(pbr.EmissiveTexture)
                || !ChargeTexture(pbr.OcclusionTexture)
                || !ChargeTexture(pbr.NormalTexture)) {
                return false;
            }
            for (const Handle(Image_Texture)& texture : {pbr.MetallicRoughnessTexture, pbr.OcclusionTexture}) {
                if (!texture.IsNull() && myValidatedNumericTextures.insert(texture.get()).second
                    && !Core3DValidateNumericTexture(texture)) {
                    Fail(GLBReadStatus::Unsupported,
                         "Data maps require upright opaque 8-bit RGB or grayscale PNG images.");
                    return false;
                }
            }
            material->SetPbrMaterial(pbr);
        }
        if (material->HasCommonMaterial()) {
            XCAFDoc_VisMaterialCommon common = material->CommonMaterial();
            if (!CanonicalizeTexture(common.DiffuseTexture)
                || !ChargeTexture(common.DiffuseTexture)) {
                return false;
            }
            material->SetCommonMaterial(common);
        }
        return true;
    }

    bool CanonicalizeMaterials() {
        std::set<const XCAFDoc_VisMaterial *> materials;
        for (RWMesh_NodeAttributeMap::Iterator iterator(myAttribMap);
             iterator.More(); iterator.Next()) {
            if (IsCancelled(myCancelled)) {
                Fail(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
                return false;
            }
            const Handle(XCAFDoc_VisMaterial)& material =
                iterator.Value().Style.Material();
            if (material.IsNull() || !materials.insert(material.get()).second) {
                continue;
            }
            if (materials.size() > kMaximumMaterialDefinitions) {
                Fail(GLBReadStatus::ResourceLimit,
                     "The GLB has too many material definitions.");
                return false;
            }
            if (!CanonicalizeMaterial(material)) {
                return false;
            }
        }
        return true;
    }

    RWMesh_NodeAttributes AttributesFor(
        const TopoDS_Shape& shape,
        const TCollection_AsciiString& inheritedName,
        const XCAFPrs_Style& inheritedStyle) const {
        RWMesh_NodeAttributes effective;
        effective.Name = BoundedAsciiName(inheritedName);
        effective.Style = inheritedStyle;

        RWMesh_NodeAttributes definition;
        if (myAttribMap.Find(shape.Located(TopLoc_Location()), definition)) {
            if (!definition.Name.IsEmpty()) {
                effective.Name = BoundedAsciiName(definition.Name);
            }
            if (!definition.Style.IsEmpty()) {
                effective.Style = definition.Style;
            }
        }
        RWMesh_NodeAttributes occurrence;
        if (!shape.Location().IsIdentity()
            && myAttribMap.Find(shape, occurrence)) {
            if (!occurrence.Name.IsEmpty()) {
                effective.Name = BoundedAsciiName(occurrence.Name);
            }
            if (!occurrence.Style.IsEmpty()) {
                effective.Style = occurrence.Style;
            }
        }
        return effective;
    }

    bool ValidateAndBake(
        const TopoDS_Face& face,
        Handle(Poly_Triangulation)& baked) {
        if (face.Orientation() != TopAbs_FORWARD
            && face.Orientation() != TopAbs_REVERSED) {
            Fail(GLBReadStatus::Unsupported,
                 "The GLB contains an unsupported primitive orientation.");
            return false;
        }
        TopLoc_Location location;
        const Handle(Poly_Triangulation)& source =
            BRep_Tool::Triangulation(face, location);
        if (source.IsNull()
            || source->HasDeferredData()
            || !source->HasGeometry()
            || source->NbNodes() <= 0
            || source->NbTriangles() <= 0) {
            Fail(GLBReadStatus::Invalid,
                 "The GLB contains an invalid loaded triangulation.");
            return false;
        }
        const std::uint64_t vertices =
            static_cast<std::uint64_t>(source->NbNodes());
        const std::uint64_t triangles =
            static_cast<std::uint64_t>(source->NbTriangles());
        if (vertices > kMaximumVerticesPerObject
            || triangles > kMaximumIndicesPerObject / 3ULL) {
            Fail(GLBReadStatus::ResourceLimit,
                 "A GLB primitive exceeds mobile mesh limits.");
            return false;
        }
        const gp_Trsf transform = location.Transformation();
        if (!IsFiniteTransform(transform)) {
            Fail(GLBReadStatus::Invalid, "A GLB node transform is not finite.");
            return false;
        }
        baked = source->Copy();
        if (baked.IsNull() || baked->HasDeferredData()) {
            Fail(GLBReadStatus::InternalFailure,
                 "The GLB triangulation could not be detached.");
            return false;
        }

        for (Standard_Integer index = 1; index <= baked->NbNodes(); ++index) {
            gp_Pnt point = baked->Node(index);
            point.Transform(transform);
            if (!IsFiniteBounded(point.X())
                || !IsFiniteBounded(point.Y())
                || !IsFiniteBounded(point.Z())) {
                Fail(GLBReadStatus::ResourceLimit,
                     "The GLB contains out-of-range mesh coordinates.");
                return false;
            }
            baked->SetNode(index, point);
            if (baked->HasNormals()) {
                gp_Dir normal = baked->Normal(index);
                normal.Transform(transform);
                if (!std::isfinite(normal.X())
                    || !std::isfinite(normal.Y())
                    || !std::isfinite(normal.Z())) {
                    Fail(GLBReadStatus::Invalid,
                         "The GLB contains invalid vertex normals.");
                    return false;
                }
                baked->SetNormal(index, normal);
            }
            if (baked->HasUVNodes()) {
                const gp_Pnt2d uv = baked->UVNode(index);
                if (!std::isfinite(uv.X()) || !std::isfinite(uv.Y())) {
                    Fail(GLBReadStatus::Invalid,
                         "The GLB contains invalid texture coordinates.");
                    return false;
                }
            }
        }
        // Poly_Triangulation::Copy() preserves the source bounding cache, while
        // SetNode() deliberately does not invalidate it. The cumulative GLB
        // occurrence transform has now been baked into every point, so refresh
        // the cache before this detached triangulation is accepted by BRep/XDE.
        // Normal and UV mutations do not participate in Poly_Triangulation's
        // cached spatial range and require no additional invalidation.
        baked->UpdateCachedMinMax();
        for (Standard_Integer index = 1;
             index <= baked->NbTriangles(); ++index) {
            Standard_Integer first = 0;
            Standard_Integer second = 0;
            Standard_Integer third = 0;
            baked->Triangle(index).Get(first, second, third);
            if (first < 1 || first > baked->NbNodes()
                || second < 1 || second > baked->NbNodes()
                || third < 1 || third > baked->NbNodes()
                || first == second || second == third || third == first) {
                Fail(GLBReadStatus::Invalid,
                     "The GLB contains invalid triangle indices.");
                return false;
            }
            // Removing a reflected occurrence location changes handedness.
            // Reverse the detached mesh once, using the cumulative transform;
            // two nested reflections therefore correctly cancel each other.
            if (transform.IsNegative()) {
                baked->SetTriangle(index, Poly_Triangle(first, third, second));
            }
            const gp_Pnt p0 = baked->Node(first);
            const gp_Pnt p1 = baked->Node(second);
            const gp_Pnt p2 = baked->Node(third);
            const gp_Vec u(p0, p1);
            const gp_Vec v(p0, p2);
            const Standard_Real squaredArea = u.Crossed(v).SquareMagnitude();
            if (!std::isfinite(squaredArea) || squaredArea <= 0.0) {
                Fail(GLBReadStatus::Invalid,
                     "The GLB contains a degenerate triangle.");
                return false;
            }
        }
        return true;
    }

    bool FlattenShape(
        const TopoDS_Shape& shape,
        const TopLoc_Location& parentLocation,
        const TCollection_AsciiString& inheritedName,
        const XCAFPrs_Style& inheritedStyle,
        CafDocumentTools& tools,
        std::uint64_t depth) {
        if (IsCancelled(myCancelled)) {
            Fail(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
            return false;
        }
        if (shape.IsNull()
            || depth > kMaximumTraversalDepth
            || ++myTraversalNodes > kMaximumTraversalNodes) {
            Fail(GLBReadStatus::ResourceLimit,
                 "The GLB scene hierarchy exceeds mobile limits.");
            return false;
        }
        const RWMesh_NodeAttributes attributes = AttributesFor(
            shape, inheritedName, inheritedStyle);
        const TopLoc_Location cumulativeLocation =
            parentLocation * shape.Location();

        if (shape.ShapeType() == TopAbs_FACE) {
            if (myObjectCount >= kMaximumObjects) {
                Fail(GLBReadStatus::ResourceLimit,
                     "The GLB has too many editable objects.");
                return false;
            }
            TopoDS_Face located = TopoDS::Face(shape);
            located.Location(cumulativeLocation, Standard_False);
            Handle(Poly_Triangulation) baked;
            if (!ValidateAndBake(located, baked)) {
                return false;
            }
            const std::uint64_t vertices =
                static_cast<std::uint64_t>(baked->NbNodes());
            const std::uint64_t indices =
                static_cast<std::uint64_t>(baked->NbTriangles()) * 3ULL;
            if (!AddWithin(
                    myVertexCount,
                    vertices,
                    kMaximumVerticesPerDocument)
                || !AddWithin(
                    myIndexCount,
                    indices,
                    kMaximumIndicesPerDocument)) {
                Fail(GLBReadStatus::ResourceLimit,
                     "The GLB mesh data exceeds mobile project limits.");
                return false;
            }
            BRep_Builder builder;
            TopoDS_Face independent;
            builder.MakeFace(independent, baked);
            independent.Orientation(located.Orientation());
            const TDF_Label label = tools.ShapeTool->AddShape(
                independent, Standard_False);
            if (label.IsNull()) {
                Fail(GLBReadStatus::InternalFailure,
                     "An editable GLB object could not be created.");
                return false;
            }
#if DEBUG
            if (myDebugTrace != nullptr) {
                const auto source = std::find_if(myDebugSourceFaces.begin(), myDebugSourceFaces.end(),
                    [&](const auto& value) { return value.IsPartner(located); });
                if (source == myDebugSourceFaces.end() || myDebugTrace->occurrences.size() >= 32) {
                    Fail(GLBReadStatus::Invalid, "A private probe occurrence lost its native source face.");
                    return false;
                }
                GLBDebugOccurrence occurrence;
                occurrence.primitive = std::uint64_t(source - myDebugSourceFaces.begin());
                TCollection_AsciiString entry; TDF_Tool::Entry(label, entry); occurrence.label = entry.ToCString();
                for (int i = 1; i <= baked->NbNodes(); ++i) {
                    const auto p = baked->Node(i); occurrence.positions.insert(occurrence.positions.end(), {p.X(),p.Y(),p.Z()});
                    if (baked->HasNormals()) { const auto n = baked->Normal(i); occurrence.normals.insert(occurrence.normals.end(), {n.X(),n.Y(),n.Z()}); }
                    if (baked->HasUVNodes()) { const auto uv = baked->UVNode(i); occurrence.uvs.insert(occurrence.uvs.end(), {uv.X(),uv.Y()}); }
                }
                for (int i = 1; i <= baked->NbTriangles(); ++i) {
                    int a=0,b=0,c=0; baked->Triangle(i).Get(a,b,c); occurrence.indices.insert(occurrence.indices.end(), {a,b,c});
                }
                myDebugTrace->occurrences.push_back(std::move(occurrence));
            }
#endif
            TCollection_AsciiString name = attributes.Name;
            if (name.IsEmpty()) {
                std::ostringstream generated;
                generated << "GLB Primitive " << (myObjectCount + 1ULL);
                name = generated.str().c_str();
            }
            TDataStd_Name::Set(label, PersistentName(name));
            setShapeStyle(tools, label, attributes.Style);
            if (!RecordImportedFrames(located,independent,label)) {
                Fail(IsCancelled(myCancelled)?GLBReadStatus::Cancelled:GLBReadStatus::Invalid,
                    "Supplied GLB frames cannot be bound to the exact final native geometry.");
                return false;
            }
            ++myObjectCount;
            return true;
        }
        if (shape.ShapeType() != TopAbs_COMPOUND) {
            Fail(GLBReadStatus::Unsupported,
                 "The GLB scene contains unsupported non-mesh topology.");
            return false;
        }
        for (TopoDS_Iterator iterator(shape, Standard_True, Standard_False);
             iterator.More(); iterator.Next()) {
            if (!FlattenShape(
                    iterator.Value(),
                    cumulativeLocation,
                    attributes.Name,
                    attributes.Style,
                    tools,
                    depth + 1ULL)) {
                return false;
            }
        }
        return true;
    }

#if DEBUG
    GLBDebugTrace* myDebugTrace = nullptr;
    std::vector<TopoDS_Face> myDebugSourceFaces;
#endif
    int myDescriptor;
    std::uint64_t mySize;
    const PreflightResult& myPreflight;
    const PreflightResult& myOriginalPreflight;
    std::uint64_t myOriginalSize;
    std::vector<SourceFace> mySourceFaces;
    std::vector<GLBImportedFrames> myImportedFrames;
    std::uint64_t myAuthoredFrameBytes=0;
    const std::atomic_bool *myCancelled;
    std::shared_ptr<DescriptorReadState> myStreamState;
    Handle(OSD_FileSystem) myFileSystem;
    GLBReadStatus myStatus = GLBReadStatus::Imported;
    std::string myMessage;
    std::uint64_t myObjectCount = 0;
    std::uint64_t myVertexCount = 0;
    std::uint64_t myIndexCount = 0;
    std::uint64_t myTraversalNodes = 0;
    std::set<const Image_Texture*> myValidatedNumericTextures;
    std::map<TextureRangeKey, Handle(Image_Texture)> myCanonicalTextures;
    Core3DEmbeddedTextureBudgetState myTextureBudget;
};

bool IsRangeWithin(
    const ByteRange& range,
    std::uint64_t size) noexcept {
    return range.offset <= size && range.length <= size - range.offset;
}

GLBReadResult Failure(GLBReadStatus status, std::string message) {
    GLBReadResult result;
    result.status = status;
    result.message = std::move(message);
    return result;
}

} // namespace

GLBReadResult ImportPinnedGLB(
    int descriptor,
    std::uint64_t expectedSize,
    const PreflightResult& preflight,
    const Handle(TDocStd_Document)& document,
    const std::atomic_bool *cancelled,
    const Message_ProgressRange& progress
#if DEBUG
    , GLBDebugTrace* debugTrace
#endif
    ) noexcept {
    try {
#if DEBUG
        if (debugTrace != nullptr) *debugTrace = {};
#endif
        if (IsCancelled(cancelled) || progress.UserBreak()) {
            return Failure(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
        }
        if (descriptor < 0 || document.IsNull()) {
            return Failure(
                GLBReadStatus::InternalFailure,
                "The private GLB import target is unavailable.");
        }
        struct stat status{};
        if (::fstat(descriptor, &status) != 0
            || !S_ISREG(status.st_mode)
            || status.st_size < 0
            || static_cast<std::uint64_t>(status.st_size) != expectedSize) {
            return Failure(
                GLBReadStatus::IOFailure,
                "The pinned GLB descriptor changed before transfer.");
        }
        if (expectedSize < 20 || expectedSize > kMaximumSourceBytes) {
            return Failure(
                GLBReadStatus::ResourceLimit,
                "The GLB exceeds the mobile source-file limit.");
        }
        if (!preflight.IsValid()
            || !IsRangeWithin(preflight.jsonChunk, expectedSize)
            || !preflight.hasBinaryChunk
            || !IsRangeWithin(preflight.binaryChunk, expectedSize)
            || preflight.objectOccurrenceEstimate == 0
            || preflight.objectOccurrenceEstimate > kMaximumObjects
            || preflight.vertexEstimate == 0
            || preflight.vertexEstimate > kMaximumVerticesPerDocument
            || preflight.indexEstimate == 0
            || preflight.indexEstimate > kMaximumIndicesPerDocument) {
            return Failure(
                GLBReadStatus::Invalid,
                "The GLB transfer was not backed by a valid bounded preflight.");
        }
        for (const EmbeddedImage& image : preflight.embeddedImages) {
            if (!IsRangeWithin(image.bytes, expectedSize)
                || image.bytes.length == 0
                || (image.mimeType != "image/png"
                    && image.mimeType != "image/jpeg")) {
                return Failure(
                    GLBReadStatus::Invalid,
                    "The GLB preflight contains an invalid embedded image range.");
            }
        }
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const Handle(XCAFDoc_ColorTool) colorTool =
            XCAFDoc_DocumentTool::ColorTool(document->Main());
        const Handle(XCAFDoc_VisMaterialTool) materialTool =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        TDF_LabelSequence existingShapes;
        TDF_LabelSequence existingColors;
        TDF_LabelSequence existingMaterials;
        if (shapeTool.IsNull() || colorTool.IsNull() || materialTool.IsNull()) {
            return Failure(
                GLBReadStatus::InternalFailure,
                "The private GLB document tables are unavailable.");
        }
        shapeTool->GetShapes(existingShapes);
        colorTool->GetColors(existingColors);
        materialTool->GetMaterials(existingMaterials);
        if (!existingShapes.IsEmpty()
            || !existingColors.IsEmpty()
            || !existingMaterials.IsEmpty()
            || document->HasOpenCommand()
            || document->GetAvailableUndos() != 0
            || document->GetAvailableRedos() != 0) {
            return Failure(
                GLBReadStatus::InternalFailure,
                "GLB import requires an isolated document with no edit history.");
        }

        PreparedGLBReader prepared;
        std::string adapterMessage;
        const auto adapter=PrepareGLBReaderSource(descriptor,expectedSize,preflight,cancelled,prepared,adapterMessage);
        if (adapter!=GLBAdapterStatus::Ready) {
            return Failure(adapter==GLBAdapterStatus::Cancelled?GLBReadStatus::Cancelled:
                adapter==GLBAdapterStatus::ResourceLimit?GLBReadStatus::ResourceLimit:
                adapter==GLBAdapterStatus::IOFailure?GLBReadStatus::IOFailure:GLBReadStatus::Invalid,adapterMessage);
        }
        const std::shared_ptr<DescriptorReadState> streamState =
            std::make_shared<DescriptorReadState>();
        streamState->cancelled = cancelled;
        streamState->source = prepared.bytes;
        const std::shared_ptr<std::streambuf> sourceBuffer =
            std::make_shared<PreadStreamBuffer>(
                descriptor, prepared.bytes->Size(), 0, streamState);
        std::istream source(sourceBuffer.get());

        PinnedCafReader reader(
            descriptor,
            prepared.bytes->Size(),
            prepared.preflight,
            preflight,
            expectedSize,
            cancelled,
            streamState
#if DEBUG
            , debugTrace
#endif
            );
        reader.SetDocument(document);
        const Standard_Boolean performed = reader.Perform(
            source,
            progress,
            TCollection_AsciiString(kPinnedToken));
        if (IsCancelled(cancelled) || progress.UserBreak()) {
            return Failure(GLBReadStatus::Cancelled, "The GLB import was cancelled.");
        }
        if (streamState->ioFailure) {
            return Failure(
                GLBReadStatus::IOFailure,
                "The pinned GLB descriptor could not be read completely.");
        }
        if (!reader.ExternalFiles().IsEmpty()) {
            return Failure(
                GLBReadStatus::Unsupported,
                "Linked GLB buffers and textures are not supported.");
        }
        if (!performed || !reader.Succeeded()) {
            return Failure(
                reader.Succeeded() ? GLBReadStatus::Invalid : reader.Status(),
                reader.FailureMessage().empty()
                    ? "OCCT rejected the preflight-approved GLB."
                    : reader.FailureMessage());
        }
        if (::fstat(descriptor, &status) != 0
            || !S_ISREG(status.st_mode)
            || status.st_size < 0
            || static_cast<std::uint64_t>(status.st_size) != expectedSize) {
            return Failure(
                GLBReadStatus::IOFailure,
                "The pinned GLB descriptor changed during transfer.");
        }
        GLBReadResult result;
        result.status = GLBReadStatus::Imported;
        result.message = "The GLB was transferred into flattened editable mesh objects.";
        result.flattenedObjectCount = reader.ObjectCount();
        result.vertexCount = reader.VertexCount();
        result.indexCount = reader.IndexCount();
        result.authoredFrames = reader.TakeAuthoredFrames();
        return result;
    } catch (const std::bad_alloc&) {
        return Failure(
            GLBReadStatus::ResourceLimit,
            "The GLB import exceeded the available mobile memory budget.");
    } catch (const Standard_Failure& failure) {
        return Failure(
            IsCancelled(cancelled)
                ? GLBReadStatus::Cancelled
                : GLBReadStatus::Invalid,
            IsCancelled(cancelled)
                ? "The GLB import was cancelled."
                : std::string("OCCT rejected the GLB transfer: ")
                    + failure.GetMessageString());
    } catch (const std::exception& exception) {
        return Failure(
            IsCancelled(cancelled)
                ? GLBReadStatus::Cancelled
                : GLBReadStatus::InternalFailure,
            IsCancelled(cancelled)
                ? "The GLB import was cancelled."
                : std::string("The GLB transfer failed: ") + exception.what());
    } catch (...) {
        return Failure(
            IsCancelled(cancelled)
                ? GLBReadStatus::Cancelled
                : GLBReadStatus::InternalFailure,
            IsCancelled(cancelled)
                ? "The GLB import was cancelled."
                : "The GLB transfer failed unexpectedly.");
    }
}

} // namespace core3d::gltf
