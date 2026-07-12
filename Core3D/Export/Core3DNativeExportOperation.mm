#import "Core3DNativeExportOperation+Private.h"

#include "../OCCTKit/Core3DSTEPExchangeLock.h"
#include "../OCCTKit/OcctDocument.h"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Image_Texture.hxx>
#include <Interface_CheckIterator.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <NCollection_Map.hxx>
#include <OSD_Path.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Drawer.hxx>
#include <RWMesh_FaceIterator.hxx>
#include <RWObj_CafWriter.hxx>
#include <RWStl.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <StepData_ConfParameters.hxx>
#include <StepData_StepModel.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS.hxx>
#include <UnitsMethods.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <XSControl_WorkSession.hxx>

#include <atomic>
#include <array>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <utility>
#include <vector>

NSErrorDomain const Core3DNativeExportErrorDomain =
    @"Core3DNativeExportErrorDomain";

namespace {

constexpr std::int64_t kMaximumSTLNodes = 1'500'000;
constexpr std::int64_t kMaximumSTLTriangles = 1'500'000;
constexpr std::uint64_t kMaximumSTLArtifactBytes =
    96ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumSTEPArtifactBytes =
    256ULL * 1024ULL * 1024ULL;

struct NativeExportState {
    std::string snapshotPath;
    std::string snapshotCleanupPath;
    std::string packageRootPath;
    std::string cleanupPath;
    std::string primaryPath;
    ExportType exportType = ExportTypeObj;
    Aspect_TypeOfDeflection deflectionType = Aspect_TOD_RELATIVE;
    Standard_Real deviationCoefficient = 0.001;
    Standard_Real deviationAngle = 20.0 * M_PI / 180.0;
    Standard_Real maximalChordialDeviation = 0.0001;
    std::atomic_bool cancelled{false};
#ifdef DEBUG
    std::atomic_bool debugOmitTextureReferences{false};
    std::atomic_bool debugCorruptSTLTriangleCount{false};
    std::atomic_bool debugCorruptSTEPTerminator{false};
    std::atomic_bool debugExceedSTLResourceLimit{false};
#endif
    std::mutex lifecycleMutex;
    bool started = false;
    bool finished = false;
};

struct NativeExportResult {
    bool succeeded = false;
    Core3DNativeExportErrorCode errorCode =
        Core3DNativeExportErrorInternalFailure;
    std::string message;
};

class NativeExportFailure final : public std::runtime_error {
public:
    NativeExportFailure(
        const Core3DNativeExportErrorCode code,
        const std::string& message)
    : std::runtime_error(message), myCode(code) {
    }

    Core3DNativeExportErrorCode Code() const noexcept {
        return myCode;
    }

private:
    Core3DNativeExportErrorCode myCode;
};

class NativeExportProgress final : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(
        NativeExportProgress,
        Message_ProgressIndicator)

public:
    explicit NativeExportProgress(const std::atomic_bool* cancelled)
    : myCancelled(cancelled) {
    }

protected:
    Standard_Boolean UserBreak() override {
        return myCancelled != nullptr
            && myCancelled->load(std::memory_order_acquire)
            ? Standard_True
            : Standard_False;
    }

    void Show(
        const Message_ProgressScope&,
        const Standard_Boolean) override {
    }

private:
    const std::atomic_bool* myCancelled;
};

//! Tracks exactly the effective textures encountered by OCCT's own OBJ
//! preflight traversal. The base writer calls addFaceInfo() for every nonempty
//! face it will later write, so this expectation cannot be weakened by a
//! texture-copy failure inside RWObj_ObjMaterialMap.
class ValidatedOBJWriter final : public RWObj_CafWriter {
public:
    explicit ValidatedOBJWriter(const TCollection_AsciiString& path)
    : RWObj_CafWriter(path) {
    }

    Standard_Integer ExpectedTextureCount() const {
        return myCreatesMaterialFile ? myTextures.Extent() : 0;
    }

protected:
    void addFaceInfo(
        const RWMesh_FaceIterator& face,
        Standard_Integer& nodeCount,
        Standard_Integer& elementCount,
        Standard_Real& progressSteps,
        Standard_Boolean& createsMaterialFile) override {
        const Handle(Image_Texture)& texture =
            face.FaceStyle().BaseColorTexture();
        if (!texture.IsNull()) {
            myTextures.Add(texture);
        }
        RWObj_CafWriter::addFaceInfo(
            face,
            nodeCount,
            elementCount,
            progressSteps,
            createsMaterialFile);
        myCreatesMaterialFile = createsMaterialFile;
    }

private:
    NCollection_Map<Handle(Image_Texture)> myTextures;
    Standard_Boolean myCreatesMaterialFile = Standard_False;
};

#ifdef DEBUG
std::mutex gDebugWorkerPauseMutex;
std::condition_variable gDebugWorkerPauseCondition;
bool gDebugWorkerPaused = false;

void WaitForDebugWorkerBarrier(
    const std::shared_ptr<NativeExportState>& state) {
    std::unique_lock<std::mutex> lock(gDebugWorkerPauseMutex);
    gDebugWorkerPauseCondition.wait(lock, [&] {
        return !gDebugWorkerPaused
            || state->cancelled.load(std::memory_order_acquire);
    });
}
#else
void WaitForDebugWorkerBarrier(
    const std::shared_ptr<NativeExportState>&) {
}
#endif

dispatch_queue_t NativeExportQueue() {
    static dispatch_queue_t queue;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        dispatch_queue_attr_t attributes =
            dispatch_queue_attr_make_with_qos_class(
                DISPATCH_QUEUE_SERIAL,
                QOS_CLASS_USER_INITIATED,
                0);
        queue = dispatch_queue_create(
            "com.shapeyard.core3d.native-export",
            attributes);
    });
    return queue;
}

void RemoveTreeNoThrow(const std::string& path) noexcept {
    if (path.empty()) {
        return;
    }
    std::error_code error;
    std::filesystem::remove_all(std::filesystem::path(path), error);
}

bool IsNonemptyRegularFile(const std::string& path) {
    struct stat info = {};
    return !path.empty()
        && ::stat(path.c_str(), &info) == 0
        && S_ISREG(info.st_mode)
        && info.st_size > 0;
}

std::string TrimASCIIWhitespace(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(whitespace);
    std::string trimmed = value.substr(first, last - first + 1);
    if (trimmed.size() >= 2
        && ((trimmed.front() == '"' && trimmed.back() == '"')
            || (trimmed.front() == '\'' && trimmed.back() == '\''))) {
        trimmed = trimmed.substr(1, trimmed.size() - 2);
    }
    return trimmed;
}

std::optional<std::string> DirectiveValue(
    const std::string& line,
    const std::string& directive) {
    if (line.size() <= directive.size()
        || line.compare(0, directive.size(), directive) != 0
        || (line[directive.size()] != ' '
            && line[directive.size()] != '\t')) {
        return std::nullopt;
    }
    return TrimASCIIWhitespace(line.substr(directive.size() + 1));
}

bool ResolveSafeArtifactReference(
    const std::filesystem::path& packageRoot,
    const std::filesystem::path& baseDirectory,
    const std::string& reference,
    std::filesystem::path& resolved) {
    if (reference.empty()) {
        return false;
    }
    const std::filesystem::path relative(reference);
    if (relative.is_absolute() || relative.has_root_name()) {
        return false;
    }
    for (const std::filesystem::path& component : relative) {
        if (component == "..") {
            return false;
        }
    }
    const std::filesystem::path normalizedRoot =
        packageRoot.lexically_normal();
    resolved = (baseDirectory / relative).lexically_normal();
    auto rootIterator = normalizedRoot.begin();
    auto resolvedIterator = resolved.begin();
    for (; rootIterator != normalizedRoot.end();
         ++rootIterator, ++resolvedIterator) {
        if (resolvedIterator == resolved.end()
            || *rootIterator != *resolvedIterator) {
            return false;
        }
    }
    return IsNonemptyRegularFile(resolved.string());
}

void ThrowIfCancelled(const std::shared_ptr<NativeExportState>& state) {
    if (state->cancelled.load(std::memory_order_acquire)) {
        throw NativeExportFailure(
            Core3DNativeExportErrorCancelled,
            "The native export was cancelled.");
    }
}

#ifdef DEBUG
bool OmitTextureReferencesForDebug(
    const std::shared_ptr<NativeExportState>& state) {
    std::error_code error;
    for (std::filesystem::recursive_directory_iterator iterator(
             std::filesystem::path(state->packageRootPath),
             std::filesystem::directory_options::skip_permission_denied,
             error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        if (!iterator->is_regular_file(error) || error
            || iterator->path().extension() != ".mtl") {
            if (error) {
                return false;
            }
            continue;
        }
        std::ifstream input(iterator->path());
        if (!input.is_open()) {
            return false;
        }
        std::vector<std::string> retainedLines;
        std::string line;
        while (std::getline(input, line)) {
            if (!DirectiveValue(line, "map_Kd").has_value()) {
                retainedLines.push_back(line);
            }
        }
        if (input.bad()) {
            return false;
        }
        input.close();
        std::ofstream output(iterator->path(), std::ios::trunc);
        if (!output.is_open()) {
            return false;
        }
        for (const std::string& retainedLine : retainedLines) {
            output << retainedLine << '\n';
        }
        if (!output.good()) {
            return false;
        }
    }
    return !error;
}
#endif

bool ValidateOBJArtifact(
    const std::shared_ptr<NativeExportState>& state,
    const Standard_Integer expectedTextureCount) {
    ThrowIfCancelled(state);
    if (!IsNonemptyRegularFile(state->primaryPath)) {
        return false;
    }
    const std::filesystem::path packageRoot(state->packageRootPath);
    std::error_code error;
    for (std::filesystem::recursive_directory_iterator iterator(
             packageRoot,
             std::filesystem::directory_options::skip_permission_denied,
             error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        ThrowIfCancelled(state);
        const std::filesystem::file_status status =
            iterator->symlink_status(error);
        if (error || std::filesystem::is_symlink(status)) {
            return false;
        }
        if (std::filesystem::is_regular_file(status)
            && !IsNonemptyRegularFile(iterator->path().string())) {
            return false;
        }
        if (!std::filesystem::is_directory(status)
            && !std::filesystem::is_regular_file(status)) {
            return false;
        }
    }
    if (error) {
        return false;
    }

    std::ifstream objectFile(state->primaryPath);
    if (!objectFile.is_open()) {
        return false;
    }
    std::vector<std::filesystem::path> materialFiles;
    std::string line;
    while (std::getline(objectFile, line)) {
        ThrowIfCancelled(state);
        const std::optional<std::string> materialReference =
            DirectiveValue(line, "mtllib");
        if (!materialReference.has_value()) {
            continue;
        }
        std::filesystem::path materialPath;
        if (!ResolveSafeArtifactReference(
                packageRoot,
                std::filesystem::path(state->primaryPath).parent_path(),
                *materialReference,
                materialPath)) {
            return false;
        }
        materialFiles.push_back(materialPath);
    }
    if (objectFile.bad()) {
        return false;
    }

    std::set<std::filesystem::path> textureFiles;
    for (const std::filesystem::path& materialPath : materialFiles) {
        ThrowIfCancelled(state);
        std::ifstream materialFile(materialPath);
        if (!materialFile.is_open()) {
            return false;
        }
        while (std::getline(materialFile, line)) {
            ThrowIfCancelled(state);
            const std::optional<std::string> textureReference =
                DirectiveValue(line, "map_Kd");
            if (!textureReference.has_value()) {
                continue;
            }
            std::filesystem::path texturePath;
            if (!ResolveSafeArtifactReference(
                    packageRoot,
                    materialPath.parent_path(),
                    *textureReference,
                    texturePath)) {
                return false;
            }
            textureFiles.insert(texturePath);
        }
        if (materialFile.bad()) {
            return false;
        }
    }
    return expectedTextureCount >= 0
        && textureFiles.size()
            == static_cast<std::size_t>(expectedTextureCount);
}

Handle(Poly_Triangulation) BuildBinarySTLMesh(
    const TopoDS_Shape& shape,
    const std::shared_ptr<NativeExportState>& state,
    const Message_ProgressRange& progress) {
    Message_ProgressScope buildScope(progress, "Flatten STL mesh", 2);
    std::int64_t nodeCount = 0;
    std::int64_t triangleCount = 0;
#ifdef DEBUG
    const bool useTestResourceLimit =
        state->debugExceedSTLResourceLimit.load(
            std::memory_order_acquire);
    const std::int64_t maximumNodes = useTestResourceLimit
        ? 1
        : kMaximumSTLNodes;
    const std::int64_t maximumTriangles = useTestResourceLimit
        ? 1
        : kMaximumSTLTriangles;
#else
    const std::int64_t maximumNodes = kMaximumSTLNodes;
    const std::int64_t maximumTriangles = kMaximumSTLTriangles;
#endif
    for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
         faceExplorer.More();
         faceExplorer.Next()) {
        ThrowIfCancelled(state);
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(
                TopoDS::Face(faceExplorer.Current()),
                location);
        if (triangulation.IsNull()
            || triangulation->NbNodes() <= 0
            || triangulation->NbTriangles() <= 0) {
            throw NativeExportFailure(
                Core3DNativeExportErrorMeshingFailed,
                "A committed face has no exportable triangulation.");
        }
        nodeCount += triangulation->NbNodes();
        triangleCount += triangulation->NbTriangles();
        const std::uint64_t artifactBytes =
            84ULL + 50ULL * static_cast<std::uint64_t>(triangleCount);
        if (nodeCount > maximumNodes
            || triangleCount > maximumTriangles
            || artifactBytes > kMaximumSTLArtifactBytes
            || nodeCount > std::numeric_limits<Standard_Integer>::max()
            || triangleCount
                > std::numeric_limits<Standard_Integer>::max()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorMeshingFailed,
                "The STL mesh exceeds the supported mobile export size.");
        }
    }
    buildScope.Next(1).Close();
    if (nodeCount <= 0 || triangleCount <= 0) {
        throw NativeExportFailure(
            Core3DNativeExportErrorNoGeometry,
            "There are no triangles to export.");
    }

    Handle(Poly_Triangulation) mesh = new Poly_Triangulation(
        static_cast<Standard_Integer>(nodeCount),
        static_cast<Standard_Integer>(triangleCount),
        Standard_False);
    Standard_Integer nodeOffset = 0;
    Standard_Integer triangleOffset = 0;
    for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
         faceExplorer.More();
         faceExplorer.Next()) {
        ThrowIfCancelled(state);
        const TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorMeshingFailed,
                "The STL triangulation changed during export.");
        }
        const gp_Trsf transform = location.Transformation();
        const bool reversesWinding =
            (face.Orientation() == TopAbs_REVERSED)
            != static_cast<bool>(transform.IsNegative());
        for (Standard_Integer nodeIndex = 1;
             nodeIndex <= triangulation->NbNodes();
             ++nodeIndex) {
            if ((nodeIndex & 4095) == 0) {
                ThrowIfCancelled(state);
            }
            gp_Pnt point = triangulation->Node(nodeIndex);
            point.Transform(transform);
            if (!std::isfinite(point.X())
                || !std::isfinite(point.Y())
                || !std::isfinite(point.Z())) {
                throw NativeExportFailure(
                    Core3DNativeExportErrorMeshingFailed,
                    "The STL mesh contains a non-finite vertex.");
            }
            mesh->SetNode(nodeOffset + nodeIndex, point);
        }
        for (Standard_Integer triangleIndex = 1;
             triangleIndex <= triangulation->NbTriangles();
             ++triangleIndex) {
            if ((triangleIndex & 4095) == 0) {
                ThrowIfCancelled(state);
            }
            Standard_Integer first = 0;
            Standard_Integer second = 0;
            Standard_Integer third = 0;
            triangulation->Triangle(triangleIndex).Get(
                first,
                second,
                third);
            if (first < 1 || first > triangulation->NbNodes()
                || second < 1 || second > triangulation->NbNodes()
                || third < 1 || third > triangulation->NbNodes()) {
                throw NativeExportFailure(
                    Core3DNativeExportErrorMeshingFailed,
                    "The STL mesh contains an invalid triangle index.");
            }
            if (reversesWinding) {
                std::swap(second, third);
            }
            mesh->SetTriangle(
                triangleOffset + triangleIndex,
                Poly_Triangle(
                    nodeOffset + first,
                    nodeOffset + second,
                    nodeOffset + third));
        }
        nodeOffset += triangulation->NbNodes();
        triangleOffset += triangulation->NbTriangles();
    }
    buildScope.Next(1).Close();
    ThrowIfCancelled(state);
    return mesh;
}

std::uint32_t ReadUInt32LittleEndian(const unsigned char *bytes) {
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8U)
        | (static_cast<std::uint32_t>(bytes[2]) << 16U)
        | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

#ifdef DEBUG
bool CorruptSTLTriangleCountForDebug(
    const std::shared_ptr<NativeExportState>& state) {
    std::fstream output(
        state->primaryPath,
        std::ios::binary | std::ios::in | std::ios::out);
    if (!output.is_open()) {
        return false;
    }
    const std::array<unsigned char, 4> invalidTriangleCount = {};
    output.seekp(80, std::ios::beg);
    output.write(
        reinterpret_cast<const char *>(invalidTriangleCount.data()),
        static_cast<std::streamsize>(invalidTriangleCount.size()));
    output.flush();
    return output.good();
}
#endif

bool ValidateBinarySTLArtifact(
    const std::shared_ptr<NativeExportState>& state,
    const Standard_Integer expectedTriangleCount) {
    ThrowIfCancelled(state);
    struct stat info = {};
    if (::stat(state->primaryPath.c_str(), &info) != 0
        || !S_ISREG(info.st_mode)
        || info.st_size < 84) {
        return false;
    }

    const std::filesystem::path packageRoot(state->packageRootPath);
    const std::filesystem::path primaryPath(state->primaryPath);
    std::error_code error;
    std::size_t artifactCount = 0;
    for (std::filesystem::directory_iterator iterator(packageRoot, error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        ThrowIfCancelled(state);
        const std::filesystem::file_status status =
            iterator->symlink_status(error);
        if (error
            || std::filesystem::is_symlink(status)
            || !std::filesystem::is_regular_file(status)
            || iterator->path().lexically_normal()
                != primaryPath.lexically_normal()) {
            return false;
        }
        ++artifactCount;
    }
    if (error || artifactCount != 1) {
        return false;
    }

    std::ifstream input(state->primaryPath, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }
    std::array<unsigned char, 84> header = {};
    input.read(
        reinterpret_cast<char *>(header.data()),
        static_cast<std::streamsize>(header.size()));
    if (input.gcount() != static_cast<std::streamsize>(header.size())) {
        return false;
    }
    const std::uint32_t triangleCount =
        ReadUInt32LittleEndian(header.data() + 80);
    const std::uint64_t expectedByteCount =
        84ULL + 50ULL * static_cast<std::uint64_t>(triangleCount);
    if (expectedTriangleCount <= 0
        || triangleCount
            != static_cast<std::uint32_t>(expectedTriangleCount)
        || static_cast<std::uint64_t>(info.st_size) != expectedByteCount) {
        return false;
    }

    std::array<unsigned char, 50> triangle = {};
    for (std::uint32_t triangleIndex = 0;
         triangleIndex < triangleCount;
         ++triangleIndex) {
        if ((triangleIndex & 1023U) == 0U) {
            ThrowIfCancelled(state);
        }
        input.read(
            reinterpret_cast<char *>(triangle.data()),
            static_cast<std::streamsize>(triangle.size()));
        if (input.gcount()
            != static_cast<std::streamsize>(triangle.size())) {
            return false;
        }
        for (std::size_t offset = 0; offset < 48; offset += 4) {
            const std::uint32_t bits =
                ReadUInt32LittleEndian(triangle.data() + offset);
            float value = 0.0f;
            std::memcpy(&value, &bits, sizeof(value));
            if (!std::isfinite(value)) {
                return false;
            }
        }
    }
    return input.peek() == std::char_traits<char>::eof()
        && !input.bad();
}

class StreamingTokenMatcher {
public:
    explicit StreamingTokenMatcher(std::string token)
    : myToken(std::move(token)) {
    }

    void Consume(const unsigned char byte) {
        if (myFound || myToken.empty()) {
            return;
        }
        if (byte == static_cast<unsigned char>(myToken[myMatched])) {
            ++myMatched;
            if (myMatched == myToken.size()) {
                myFound = true;
            }
            return;
        }
        myMatched = byte == static_cast<unsigned char>(myToken.front())
            ? 1
            : 0;
    }

    bool Found() const {
        return myFound;
    }

private:
    std::string myToken;
    std::size_t myMatched = 0;
    bool myFound = false;
};

std::string ExpectedSTEPUnitToken(const UnitsMethods_LengthUnit unit) {
    switch (unit) {
        case UnitsMethods_LengthUnit_Inch:
            return "CONVERSION_BASED_UNIT('INCH'";
        case UnitsMethods_LengthUnit_Millimeter:
            return "SI_UNIT(.MILLI.,.METRE.)";
        case UnitsMethods_LengthUnit_Foot:
            return "CONVERSION_BASED_UNIT('FOOT'";
        case UnitsMethods_LengthUnit_Mile:
            return "CONVERSION_BASED_UNIT('MILE'";
        case UnitsMethods_LengthUnit_Meter:
            return "SI_UNIT($,.METRE.)";
        case UnitsMethods_LengthUnit_Kilometer:
            return "SI_UNIT(.KILO.,.METRE.)";
        case UnitsMethods_LengthUnit_Mil:
            return "CONVERSION_BASED_UNIT('MIL'";
        case UnitsMethods_LengthUnit_Micron:
            return "SI_UNIT(.MICRO.,.METRE.)";
        case UnitsMethods_LengthUnit_Centimeter:
            return "SI_UNIT(.CENTI.,.METRE.)";
        case UnitsMethods_LengthUnit_Microinch:
            return "CONVERSION_BASED_UNIT('MICROINCH'";
        case UnitsMethods_LengthUnit_Undefined:
            return {};
    }
    return {};
}

bool HasExactSingleArtifact(
    const std::shared_ptr<NativeExportState>& state) {
    struct stat info = {};
    if (::lstat(state->primaryPath.c_str(), &info) != 0
        || !S_ISREG(info.st_mode)
        || info.st_size <= 0) {
        return false;
    }
    const std::filesystem::path packageRoot(state->packageRootPath);
    const std::filesystem::path primaryPath =
        std::filesystem::path(state->primaryPath).lexically_normal();
    std::error_code error;
    std::size_t artifactCount = 0;
    for (std::filesystem::directory_iterator iterator(packageRoot, error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        ThrowIfCancelled(state);
        const std::filesystem::file_status status =
            iterator->symlink_status(error);
        if (error
            || std::filesystem::is_symlink(status)
            || !std::filesystem::is_regular_file(status)
            || iterator->path().lexically_normal() != primaryPath) {
            return false;
        }
        ++artifactCount;
    }
    return !error && artifactCount == 1;
}

bool ValidateSTEPFileContents(
    const std::shared_ptr<NativeExportState>& state,
    const std::string& path,
    const UnitsMethods_LengthUnit outputUnit,
    const bool expectedColor) {
    ThrowIfCancelled(state);
    struct stat info = {};
    if (::lstat(path.c_str(), &info) != 0
        || !S_ISREG(info.st_mode)
        || info.st_size <= 0
        || static_cast<std::uint64_t>(info.st_size)
            > kMaximumSTEPArtifactBytes) {
        return false;
    }

    const std::string unitToken = ExpectedSTEPUnitToken(outputUnit);
    if (unitToken.empty()) {
        return false;
    }
    StreamingTokenMatcher schema(
        "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }");
    StreamingTokenMatcher shapeRepresentation(
        "SHAPE_DEFINITION_REPRESENTATION");
    StreamingTokenMatcher lengthUnit("LENGTH_UNIT()");
    StreamingTokenMatcher expectedUnit(unitToken);
    StreamingTokenMatcher rgbColor("COLOUR_RGB");
    StreamingTokenMatcher predefinedColor(
        "DRAUGHTING_PRE_DEFINED_COLOUR");
    const std::array<std::string, 4> structureTokens = {
        "HEADER;",
        "ENDSEC;",
        "DATA;",
        "ENDSEC;",
    };
    std::size_t structureStage = 0;
    std::size_t structureMatched = 0;
    enum class EntityState {
        Waiting,
        Hash,
        Digits,
        Whitespace,
    };
    EntityState entityState = EntityState::Waiting;
    bool hasEntity = false;
    const std::string requiredPrefix = "ISO-10303-21;";
    std::size_t prefixIndex = 0;
    bool prefixMatches = true;
    const std::string terminator = "END-ISO-10303-21;";
    std::string tail;
    tail.reserve(256);

    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }
    std::array<unsigned char, 64 * 1024> buffer = {};
    std::uint64_t byteCount = 0;
    while (input) {
        ThrowIfCancelled(state);
        input.read(
            reinterpret_cast<char *>(buffer.data()),
            static_cast<std::streamsize>(buffer.size()));
        const std::streamsize readCount = input.gcount();
        if (readCount <= 0) {
            break;
        }
        byteCount += static_cast<std::uint64_t>(readCount);
        if (byteCount > kMaximumSTEPArtifactBytes) {
            return false;
        }
        for (std::streamsize index = 0; index < readCount; ++index) {
            const unsigned char byte = buffer[static_cast<std::size_t>(index)];
            if (byte == 0) {
                return false;
            }
            if (prefixIndex < requiredPrefix.size()) {
                prefixMatches = prefixMatches
                    && byte == static_cast<unsigned char>(
                        requiredPrefix[prefixIndex]);
                ++prefixIndex;
            }
            if (structureStage < structureTokens.size()) {
                const std::string& token = structureTokens[structureStage];
                if (byte == static_cast<unsigned char>(
                        token[structureMatched])) {
                    ++structureMatched;
                    if (structureMatched == token.size()) {
                        ++structureStage;
                        structureMatched = 0;
                    }
                } else {
                    structureMatched =
                        byte == static_cast<unsigned char>(token.front())
                        ? 1
                        : 0;
                }
            }
            schema.Consume(byte);
            shapeRepresentation.Consume(byte);
            lengthUnit.Consume(byte);
            expectedUnit.Consume(byte);
            rgbColor.Consume(byte);
            predefinedColor.Consume(byte);

            if (!hasEntity) {
                if (entityState == EntityState::Waiting) {
                    if (byte == '#') {
                        entityState = EntityState::Hash;
                    }
                } else if (entityState == EntityState::Hash) {
                    if (std::isdigit(byte)) {
                        entityState = EntityState::Digits;
                    } else {
                        entityState = byte == '#'
                            ? EntityState::Hash
                            : EntityState::Waiting;
                    }
                } else if (entityState == EntityState::Digits) {
                    if (std::isdigit(byte)) {
                        continue;
                    }
                    if (byte == '=') {
                        hasEntity = true;
                    } else if (std::isspace(byte)) {
                        entityState = EntityState::Whitespace;
                    } else {
                        entityState = byte == '#'
                            ? EntityState::Hash
                            : EntityState::Waiting;
                    }
                } else {
                    if (byte == '=') {
                        hasEntity = true;
                    } else if (!std::isspace(byte)) {
                        entityState = byte == '#'
                            ? EntityState::Hash
                            : EntityState::Waiting;
                    }
                }
            }
        }
        tail.append(
            reinterpret_cast<const char *>(buffer.data()),
            static_cast<std::size_t>(readCount));
        if (tail.size() > 256) {
            tail.erase(0, tail.size() - 256);
        }
    }
    if (input.bad()
        || byteCount != static_cast<std::uint64_t>(info.st_size)
        || prefixIndex < requiredPrefix.size()
        || !prefixMatches) {
        return false;
    }

    const std::size_t lastNonWhitespace =
        tail.find_last_not_of(" \t\r\n");
    if (lastNonWhitespace == std::string::npos
        || lastNonWhitespace + 1 < terminator.size()
        || tail.compare(
            lastNonWhitespace + 1 - terminator.size(),
            terminator.size(),
            terminator) != 0) {
        return false;
    }
    return structureStage == structureTokens.size()
        && schema.Found()
        && shapeRepresentation.Found()
        && lengthUnit.Found()
        && expectedUnit.Found()
        && hasEntity
        && (!expectedColor
            || rgbColor.Found()
            || predefinedColor.Found());
}

#ifdef DEBUG
bool CorruptSTEPTerminatorForDebug(const std::string& path) {
    std::ofstream output(path, std::ios::binary | std::ios::app);
    if (!output.is_open()) {
        return false;
    }
    output << "CORRUPTED";
    output.flush();
    return output.good();
}
#endif

void BridgeLegacySTEPColors(
    const Handle(OcctDocument)& document,
    const Handle(TDocStd_Document)& ocafDocument,
    const TDF_LabelSequence& rootLabels) {
    const Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(ocafDocument->Main());
    if (colorTool.IsNull()) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInvalidState,
            "The STEP color table is unavailable.");
    }
    for (Standard_Integer index = 1;
         index <= rootLabels.Length();
         ++index) {
        const TDF_Label& label = rootLabels.Value(index);
        XCAFDoc_VisMaterialPBR localPBR;
        Quantity_NameOfColor legacyColor = Quantity_NOC_GRAY80;
        if (!document->TryPBRMaterialForLabel(label, localPBR)
            && document->TryColorNameForLabel(label, legacyColor)) {
            colorTool->SetColor(
                label,
                Quantity_Color(legacyColor),
                XCAFDoc_ColorSurf);
        }
    }
}

bool HasSTEPColorExpectation(
    const Handle(TDocStd_Document)& ocafDocument,
    const TDF_LabelSequence& rootLabels) {
    const Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(ocafDocument->Main());
    if (colorTool.IsNull()) {
        return false;
    }
    for (Standard_Integer index = 1;
         index <= rootLabels.Length();
         ++index) {
        const TDF_Label& label = rootLabels.Value(index);
        if (colorTool->IsSet(label, XCAFDoc_ColorGen)
            || colorTool->IsSet(label, XCAFDoc_ColorSurf)
            || colorTool->IsSet(label, XCAFDoc_ColorCurv)
            || !XCAFDoc_VisMaterialTool::GetShapeMaterial(label).IsNull()) {
            return true;
        }
    }
    return false;
}

UnitsMethods_LengthUnit ConfigureSTEPUnits(
    const Handle(TDocStd_Document)& ocafDocument,
    StepData_ConfParameters& parameters,
    const Handle(StepData_StepModel)& stepModel) {
    Standard_Real metersPerUnit = 0.001;
    const Standard_Boolean hasUnit =
        XCAFDoc_DocumentTool::GetLengthUnit(
            ocafDocument,
            metersPerUnit);
    if (hasUnit
        && (!std::isfinite(metersPerUnit) || metersPerUnit <= 0.0)) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInvalidState,
            "The STEP source unit is invalid.");
    }
    if (!hasUnit) {
        metersPerUnit = 0.001;
        XCAFDoc_DocumentTool::SetLengthUnit(
            ocafDocument,
            metersPerUnit);
    }
    UnitsMethods_LengthUnit outputUnit =
        UnitsMethods::GetLengthUnitByFactorValue(
            metersPerUnit,
            UnitsMethods_LengthUnit_Meter);
    if (outputUnit == UnitsMethods_LengthUnit_Undefined) {
        outputUnit = UnitsMethods_LengthUnit_Millimeter;
    }
    const Standard_Real outputUnitMillimeters =
        UnitsMethods::GetLengthUnitScale(
            outputUnit,
            UnitsMethods_LengthUnit_Millimeter);
    if (!std::isfinite(outputUnitMillimeters)
        || outputUnitMillimeters <= 0.0) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInvalidState,
            "The STEP output unit is invalid.");
    }
    parameters.WriteUnit = outputUnit;
    stepModel->SetWriteLengthUnit(outputUnitMillimeters);
    return outputUnit;
}

void WriteSTEP(
    const std::shared_ptr<NativeExportState>& state,
    const Handle(OcctDocument)& document,
    const Handle(TDocStd_Document)& ocafDocument,
    const TDF_LabelSequence& rootLabels,
    const Message_ProgressRange& progress) {
    ThrowIfCancelled(state);
    BridgeLegacySTEPColors(document, ocafDocument, rootLabels);
    const bool expectedColor =
        HasSTEPColorExpectation(ocafDocument, rootLabels);
    UnitsMethods_LengthUnit outputUnit =
        UnitsMethods_LengthUnit_Undefined;
    const std::string partialPath =
        (std::filesystem::path(state->packageRootPath)
            / ".model.step.partial").string();
    {
        std::lock_guard<std::mutex> exchangeLock(
            Core3DSTEPExchangeMutex());
        STEPCAFControl_Writer writer;
        writer.SetColorMode(Standard_True);
        writer.SetNameMode(Standard_True);
        writer.SetMaterialMode(Standard_True);
        writer.SetLayerMode(Standard_False);
        writer.SetPropsMode(Standard_False);
        writer.SetSHUOMode(Standard_False);
        writer.SetDimTolMode(Standard_False);

        StepData_ConfParameters parameters;
        parameters.WriteSchema =
            StepData_ConfParameters::WriteMode_StepSchema_AP214IS;
        parameters.WriteAssembly =
            StepData_ConfParameters::WriteMode_Assembly_Auto;
        parameters.WriteTessellated =
            StepData_ConfParameters::RWMode_Tessellated_Off;
        parameters.WriteProductName =
            TCollection_AsciiString("Shapeyard 3D");
        parameters.WriteColor = true;
        parameters.WriteName = true;
        parameters.WriteLayer = false;
        parameters.WriteProps = false;
        parameters.WriteSubshapeNames = false;
        parameters.WriteNonmanifold = false;
        parameters.WriteModelType = STEPControl_AsIs;

        const Handle(StepData_StepModel) stepModel =
            writer.ChangeWriter().Model(Standard_False);
        if (stepModel.IsNull()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorWriterFailed,
                "The STEP model could not be created.");
        }
        outputUnit = ConfigureSTEPUnits(
            ocafDocument,
            parameters,
            stepModel);
        const Standard_Boolean transferred = writer.Transfer(
            rootLabels,
            parameters,
            STEPControl_AsIs,
            nullptr,
            progress);
        ThrowIfCancelled(state);

        const Handle(XSControl_WorkSession) session =
            writer.ChangeWriter().WS();
        const Interface_CheckIterator checks = session.IsNull()
            ? Interface_CheckIterator()
            : session->TransferWriteCheckList();
        const Handle(StepData_StepModel) translatedModel =
            writer.ChangeWriter().Model(Standard_False);
        if (!transferred
            || session.IsNull()
            || translatedModel.IsNull()
            || translatedModel->NbEntities() <= 0
            || !checks.IsEmpty(Standard_True)) {
            throw NativeExportFailure(
                Core3DNativeExportErrorWriterFailed,
                "The committed geometry could not be translated to STEP.");
        }
        if (writer.Write(partialPath.c_str()) != IFSelect_RetDone) {
            throw NativeExportFailure(
                Core3DNativeExportErrorWriterFailed,
                "The STEP writer reported a failure.");
        }
    }
    ThrowIfCancelled(state);
#ifdef DEBUG
    if (state->debugCorruptSTEPTerminator.load(
            std::memory_order_acquire)
        && !CorruptSTEPTerminatorForDebug(partialPath)) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInternalFailure,
            "The STEP corruption test seam could not be applied.");
    }
#endif
    if (!ValidateSTEPFileContents(
            state,
            partialPath,
            outputUnit,
            expectedColor)) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInvalidArtifact,
            "The STEP writer produced an invalid artifact.");
    }
    std::error_code renameError;
    std::filesystem::rename(
        std::filesystem::path(partialPath),
        std::filesystem::path(state->primaryPath),
        renameError);
    if (renameError || !HasExactSingleArtifact(state)) {
        throw NativeExportFailure(
            Core3DNativeExportErrorInvalidArtifact,
            "The STEP artifact could not be finalized safely.");
    }
    ThrowIfCancelled(state);
}

NativeExportResult RunNativeExport(
    const std::shared_ptr<NativeExportState>& state) noexcept {
    NativeExportResult result;
    Handle(OcctDocument) document;
    try {
        OCC_CATCH_SIGNALS
        ThrowIfCancelled(state);

        const std::filesystem::path packageRoot(state->packageRootPath);
        std::error_code fileError;
        if (!std::filesystem::is_directory(packageRoot, fileError)
            || fileError
            || !std::filesystem::is_empty(packageRoot, fileError)
            || fileError) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The native export directory is not empty.");
        }

        Handle(Message_ProgressIndicator) progress =
            new NativeExportProgress(&state->cancelled);
        const char *progressName = state->exportType == ExportTypeStl
            ? "STL export"
            : (state->exportType == ExportTypeStep
                ? "STEP export"
                : "OBJ export");
        Message_ProgressScope whole(
            progress->Start(),
            progressName,
            11);

        document = new OcctDocument();
        if (!document->OpenPrivateExportSnapshot(
                state->snapshotPath,
                whole.Next(1))) {
            ThrowIfCancelled(state);
            throw NativeExportFailure(
                Core3DNativeExportErrorSnapshotOpenFailed,
                "The private export snapshot could not be opened.");
        }
        ThrowIfCancelled(state);

        const Handle(TDocStd_Document)& ocafDocument = document->Document();
        if (ocafDocument.IsNull()
            || ocafDocument->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                ocafDocument->Main())) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The private export document is incomplete.");
        }
        ocafDocument->SetUndoLimit(1);
        ocafDocument->NewCommand();
        if (!ocafDocument->HasOpenCommand()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The private export transaction could not be opened.");
        }
        if (!document->ApplyTransforms(whole.Next(1))) {
            ThrowIfCancelled(state);
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The committed transforms could not be applied.");
        }
        ThrowIfCancelled(state);

        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(ocafDocument->Main());
        TDF_LabelSequence freeLabels;
        shapeTool->GetFreeShapes(freeLabels);
        if (freeLabels.IsEmpty()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorNoGeometry,
                "There is no committed geometry to export.");
        }
        TDF_LabelSequence rootLabels;
        for (Standard_Integer index = 1;
             index <= freeLabels.Length();
             ++index) {
            ThrowIfCancelled(state);
            if (!shapeTool->GetShape(freeLabels.Value(index)).IsNull()) {
                rootLabels.Append(freeLabels.Value(index));
            }
        }
        if (rootLabels.IsEmpty()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorNoGeometry,
                "There is no valid committed geometry to export.");
        }

        if (state->exportType == ExportTypeStep) {
            WriteSTEP(
                state,
                document,
                ocafDocument,
                rootLabels,
                whole.Next(9));
        } else {
            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);
            for (Standard_Integer index = 1;
                 index <= rootLabels.Length();
                 ++index) {
                ThrowIfCancelled(state);
                const TopoDS_Shape shape = shapeTool->GetShape(
                    rootLabels.Value(index));
                builder.Add(compound, shape);
            }

            Handle(Prs3d_Drawer) drawer = new Prs3d_Drawer();
            drawer->SetTypeOfDeflection(state->deflectionType);
            drawer->SetDeviationCoefficient(state->deviationCoefficient);
            drawer->SetDeviationAngle(state->deviationAngle);
            drawer->SetMaximalChordialDeviation(
                state->maximalChordialDeviation);
            const Standard_Real deflection =
                StdPrs_ToolTriangulatedShape::GetDeflection(
                    compound,
                    drawer);
            if (!std::isfinite(deflection) || deflection <= 0.0) {
                throw NativeExportFailure(
                    Core3DNativeExportErrorMeshingFailed,
                    "The mesh deflection is invalid.");
            }

            if (!BRepTools::Triangulation(compound, deflection)) {
                BRepMesh_IncrementalMesh mesher;
                mesher.ChangeParameters().Deflection = deflection;
                mesher.ChangeParameters().Angle = state->deviationAngle;
                mesher.ChangeParameters().InParallel = Standard_True;
                mesher.SetShape(compound);
                mesher.Perform(whole.Next(4));
                ThrowIfCancelled(state);
                if (!mesher.IsDone()
                    || !BRepTools::Triangulation(compound, deflection)) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorMeshingFailed,
                        "The committed geometry could not be meshed.");
                }
            } else {
                whole.Next(4).Close();
            }
            ThrowIfCancelled(state);

            if (state->exportType == ExportTypeStl) {
                Message_ProgressScope stlScope(
                    whole.Next(5),
                    "Binary STL export",
                    2);
                const Handle(Poly_Triangulation) stlMesh =
                    BuildBinarySTLMesh(
                        compound,
                        state,
                        stlScope.Next(1));
                const OSD_Path outputPath(
                    TCollection_AsciiString(state->primaryPath.c_str()));
                const bool writerSucceeded = RWStl::WriteBinary(
                    stlMesh,
                    outputPath,
                    stlScope.Next(1));
                ThrowIfCancelled(state);
                if (!writerSucceeded) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorWriterFailed,
                        "The STL writer reported a failure.");
                }
#ifdef DEBUG
                if (state->debugCorruptSTLTriangleCount.load(
                        std::memory_order_acquire)
                    && !CorruptSTLTriangleCountForDebug(state)) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorInternalFailure,
                        "The STL corruption test seam could not be applied.");
                }
#endif
                if (!ValidateBinarySTLArtifact(
                        state,
                        stlMesh->NbTriangles())) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorInvalidArtifact,
                        "The STL writer produced an invalid binary artifact.");
                }
            } else if (state->exportType == ExportTypeObj) {
                TColStd_IndexedDataMapOfStringString fileInfo;
                fileInfo.Add("Author", "Shapeyard 3D");
                ValidatedOBJWriter writer(
                    TCollection_AsciiString(state->primaryPath.c_str()));
                const bool writerSucceeded = writer.Perform(
                    ocafDocument,
                    rootLabels,
                    nullptr,
                    fileInfo,
                    whole.Next(5));
                ThrowIfCancelled(state);
                if (!writerSucceeded) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorWriterFailed,
                        "The OBJ writer reported a failure.");
                }
#ifdef DEBUG
                if (state->debugOmitTextureReferences.load(
                        std::memory_order_acquire)
                    && !OmitTextureReferencesForDebug(state)) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorInternalFailure,
                        "The texture-omission test seam could not be applied.");
                }
#endif
                if (!ValidateOBJArtifact(
                        state,
                        writer.ExpectedTextureCount())) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorInvalidArtifact,
                        "The OBJ writer produced an incomplete artifact bundle.");
                }
            } else {
                throw NativeExportFailure(
                    Core3DNativeExportErrorInvalidState,
                    "The native export format is unsupported.");
            }
        }

        ThrowIfCancelled(state);
        result.succeeded = true;
    } catch (const NativeExportFailure& failure) {
        result.errorCode = failure.Code();
        result.message = failure.what();
    } catch (const Standard_Failure& failure) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeExportErrorCancelled
            : Core3DNativeExportErrorInternalFailure;
        result.message = failure.GetMessageString();
    } catch (const std::bad_alloc&) {
        result.errorCode = Core3DNativeExportErrorInternalFailure;
        result.message = "The native export ran out of memory.";
    } catch (const std::exception& exception) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeExportErrorCancelled
            : Core3DNativeExportErrorInternalFailure;
        result.message = exception.what();
    } catch (...) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeExportErrorCancelled
            : Core3DNativeExportErrorInternalFailure;
        result.message = "The native export failed unexpectedly.";
    }

    if (!document.IsNull()) {
        document->ClosePrivateExportSnapshot();
        document.Nullify();
    }
    RemoveTreeNoThrow(state->snapshotCleanupPath);
    if (!result.succeeded) {
        RemoveTreeNoThrow(state->cleanupPath);
    }
    return result;
}

NSString *ErrorDescription(const NativeExportResult& result) {
    if (!result.message.empty()) {
        return [NSString stringWithUTF8String:result.message.c_str()]
            ?: @"The native export failed.";
    }
    return @"The native export failed.";
}

} // namespace

@interface Core3DNativeExportArtifact ()

@property(nonatomic, readwrite, copy) NSURL *primaryURL;
@property(nonatomic, readwrite, copy) NSURL *packageRootURL;
@property(nonatomic, readwrite, copy) NSURL *cleanupURL;

- (instancetype)initWithPrimaryURL:(NSURL *)primaryURL
                     packageRootURL:(NSURL *)packageRootURL
                         cleanupURL:(NSURL *)cleanupURL;

@end

@implementation Core3DNativeExportArtifact

- (instancetype)initWithPrimaryURL:(NSURL *)primaryURL
                     packageRootURL:(NSURL *)packageRootURL
                         cleanupURL:(NSURL *)cleanupURL {
    self = [super init];
    if (self) {
        _primaryURL = [primaryURL copy];
        _packageRootURL = [packageRootURL copy];
        _cleanupURL = [cleanupURL copy];
    }
    return self;
}

@end

@interface Core3DNativeExportOperation () {
    std::shared_ptr<NativeExportState> _state;
}
@end

@implementation Core3DNativeExportOperation

- (instancetype)initWithSnapshotURL:(NSURL *)snapshotURL
                 snapshotCleanupURL:(NSURL *)snapshotCleanupURL
                      packageRootURL:(NSURL *)packageRootURL
                          cleanupURL:(NSURL *)cleanupURL
                          exportType:(ExportType)exportType
                      deflectionType:(NSInteger)deflectionType
                deviationCoefficient:(double)deviationCoefficient
                       deviationAngle:(double)deviationAngle
            maximalChordialDeviation:(double)maximalChordialDeviation {
    self = [super init];
    if (!self) {
        return nil;
    }
    if (!snapshotURL.isFileURL
        || !snapshotCleanupURL.isFileURL
        || !packageRootURL.isFileURL
        || !cleanupURL.isFileURL
        || (exportType != ExportTypeObj
            && exportType != ExportTypeStl
            && exportType != ExportTypeStep)
        || (deflectionType != Aspect_TOD_ABSOLUTE
            && deflectionType != Aspect_TOD_RELATIVE)
        || !std::isfinite(deviationCoefficient)
        || deviationCoefficient <= 0.0
        || !std::isfinite(deviationAngle)
        || deviationAngle <= 0.0
        || !std::isfinite(maximalChordialDeviation)
        || maximalChordialDeviation <= 0.0) {
        return nil;
    }

    const char *snapshotPath = snapshotURL.path.UTF8String;
    const char *snapshotCleanupPath = snapshotCleanupURL.path.UTF8String;
    const char *packageRootPath = packageRootURL.path.UTF8String;
    const char *cleanupPath = cleanupURL.path.UTF8String;
    if (snapshotPath == nullptr
        || snapshotCleanupPath == nullptr
        || packageRootPath == nullptr
        || cleanupPath == nullptr) {
        return nil;
    }

    _state = std::make_shared<NativeExportState>();
    _state->snapshotPath = snapshotPath;
    _state->snapshotCleanupPath = snapshotCleanupPath;
    _state->packageRootPath = packageRootPath;
    _state->cleanupPath = cleanupPath;
    _state->exportType = exportType;
    const char *primaryFilename = exportType == ExportTypeStl
        ? "model.stl"
        : (exportType == ExportTypeStep ? "model.step" : "model.obj");
    _state->primaryPath = (
        std::filesystem::path(packageRootPath)
        / primaryFilename
    ).string();
    _state->deflectionType =
        static_cast<Aspect_TypeOfDeflection>(deflectionType);
    _state->deviationCoefficient = deviationCoefficient;
    _state->deviationAngle = deviationAngle;
    _state->maximalChordialDeviation = maximalChordialDeviation;
    return self;
}

- (BOOL)isCancelled {
    return _state != nullptr
        && _state->cancelled.load(std::memory_order_acquire);
}

- (void)dealloc {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    bool shouldCleanup = false;
    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (!state->started) {
            state->started = true;
            state->finished = true;
            state->cancelled.store(true, std::memory_order_release);
            shouldCleanup = true;
        }
    }
    if (shouldCleanup) {
        dispatch_async(NativeExportQueue(), ^{
            RemoveTreeNoThrow(state->snapshotCleanupPath);
            RemoveTreeNoThrow(state->cleanupPath);
        });
    }
}

- (void)startWithCompletion:(Core3DNativeExportCompletion)completion {
    if (completion == nil) {
        return;
    }
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(nil, [NSError
                errorWithDomain:Core3DNativeExportErrorDomain
                code:Core3DNativeExportErrorInvalidState
                userInfo:@{
                    NSLocalizedDescriptionKey:
                        @"The native export operation is unavailable."
                }]);
        });
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (state->started) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil, [NSError
                    errorWithDomain:Core3DNativeExportErrorDomain
                    code:Core3DNativeExportErrorInvalidState
                    userInfo:@{
                        NSLocalizedDescriptionKey:
                            @"The native export operation was already started."
                    }]);
            });
            return;
        }
        state->started = true;
    }

    Core3DNativeExportCompletion completionCopy = [completion copy];
    dispatch_async(NativeExportQueue(), ^{
        WaitForDebugWorkerBarrier(state);
        const NativeExportResult result = RunNativeExport(state);
        {
            std::lock_guard<std::mutex> lock(state->lifecycleMutex);
            state->finished = true;
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            if (result.succeeded) {
                NSURL *packageRootURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->packageRootPath.c_str()]];
                NSURL *cleanupURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->cleanupPath.c_str()]];
                NSURL *primaryURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->primaryPath.c_str()]];
                Core3DNativeExportArtifact *artifact =
                    [[Core3DNativeExportArtifact alloc]
                        initWithPrimaryURL:primaryURL
                        packageRootURL:packageRootURL
                        cleanupURL:cleanupURL];
                completionCopy(artifact, nil);
                return;
            }
            NSLog(@"[NativeExport] Worker failed: code=%ld message=%@",
                  static_cast<long>(result.errorCode),
                  ErrorDescription(result));
            NSError *error = [NSError
                errorWithDomain:Core3DNativeExportErrorDomain
                code:result.errorCode
                userInfo:@{
                    NSLocalizedDescriptionKey: ErrorDescription(result)
                }];
            completionCopy(nil, error);
        });
    });
}

- (void)cancel {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (state->finished) {
            return;
        }
        state->cancelled.store(true, std::memory_order_release);
    }
#ifdef DEBUG
    gDebugWorkerPauseCondition.notify_all();
#endif
}

#ifdef DEBUG
+ (void)debugSetWorkerPaused:(BOOL)paused {
    {
        std::lock_guard<std::mutex> lock(gDebugWorkerPauseMutex);
        gDebugWorkerPaused = paused;
    }
    if (!paused) {
        gDebugWorkerPauseCondition.notify_all();
    }
}

- (void)debugSimulateTextureReferenceOmission {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state->lifecycleMutex);
    if (!state->started) {
        state->debugOmitTextureReferences.store(
            true,
            std::memory_order_release);
    }
}

- (void)debugSimulateSTLTriangleCountCorruption {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state->lifecycleMutex);
    if (!state->started && state->exportType == ExportTypeStl) {
        state->debugCorruptSTLTriangleCount.store(
            true,
            std::memory_order_release);
    }
}

- (void)debugSimulateSTEPTerminatorCorruption {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state->lifecycleMutex);
    if (!state->started && state->exportType == ExportTypeStep) {
        state->debugCorruptSTEPTerminator.store(
            true,
            std::memory_order_release);
    }
}

- (void)debugSimulateSTLResourceLimitExceeded {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(state->lifecycleMutex);
    if (!state->started && state->exportType == ExportTypeStl) {
        state->debugExceedSTLResourceLimit.store(
            true,
            std::memory_order_release);
    }
}
#endif

@end
