#import "Core3DNativeExportOperation+Private.h"

#include "../OCCTKit/Core3DSTEPExchangeLock.h"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/EnclosurePersistence.hxx"
#include "../Scene/OcctSceneSnapshotBuilder.hpp"
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepMesh_Context.hxx>
#include <BRepMesh_FaceDiscret.hxx>
#include <BRepMesh_MeshAlgoFactory.hxx>
#include <BRepMesh_DelaunayBaseMeshAlgo.hxx>
#include <BRepMesh_DelaunayDeflectionControlMeshAlgo.hxx>
#include <BRepMesh_NURBSRangeSplitter.hxx>
#include <Geom_BSplineSurface.hxx>
#include <algorithm>
#ifdef DEBUG
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepTools_ShapeSet.hxx>
#include <TopoDS_Iterator.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfInteger.hxx>
#endif
#include <BRepTools.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_Real.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Image_Texture.hxx>
#include <Interface_CheckIterator.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <NCollection_Map.hxx>
#include <OSD_Path.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <Prs3d_Drawer.hxx>
#include <RWMesh_FaceIterator.hxx>
#include <RWObj_CafWriter.hxx>
#include <RWObj_ObjWriterContext.hxx>
#include <XCAFDoc_VisMaterial.hxx>
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
#include <iomanip>
#include <locale>
#include <unordered_map>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
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
    Core3DExportMeshQuality meshQuality = Core3DExportMeshQualityViewport;
    Core3DOBJColorConvention objColorConvention = Core3DOBJColorConventionCurrent;
    core3d::scene::OcctSceneSnapshotBuilder::SnapshotPointer sourceScene;
    bool selectedObjectsOnly = false;
    bool usesSelectedRoots = false;
    std::set<std::string> selectedEntityIdentifiers;
    Aspect_TypeOfDeflection deflectionType = Aspect_TOD_RELATIVE;
    Standard_Real deviationCoefficient = 0.001;
    Standard_Real deviationAngle = 20.0 * M_PI / 180.0;
    Standard_Real maximalChordialDeviation = 0.0001;
    std::atomic_bool cancelled{false};
#ifdef DEBUG
    bool debugInitialInteriorControl = false;
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
    core3d::scene::OcctSceneSnapshotBuilder::SnapshotPointer scene;
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
    explicit ValidatedOBJWriter(const TCollection_AsciiString& path, Core3DOBJColorConvention convention)
    : RWObj_CafWriter(path), myConvention(convention) {
    }

    Standard_Integer ExpectedTextureCount() const {
        return myCreatesMaterialFile ? myTextures.Extent() : 0;
    }

    using PBRScalars = std::array<double, 6>; // RGB, metallic, roughness, PBR flag
    const std::unordered_map<std::string, PBRScalars>& MaterialScalars() const {
        return myMaterialScalars;
    }

protected:
    bool writePositions(RWObj_ObjWriterContext& writer,
                        Message_LazyProgressScope& progress,
                        const RWMesh_FaceIterator& face) override {
        // OCCT registers the effective face material before writing positions.
        // Use that exact key; material names cannot be reconstructed from labels.
        const auto& material = face.FaceStyle().Material();
        const bool hasPBR = !material.IsNull() && material->HasPbrMaterial();
        if (!writer.ActiveMaterial().IsEmpty()
            && (hasPBR || myConvention != Core3DOBJColorConventionCurrent)) {
            // Match the writer's effective face color, including occurrence and
            // face overrides, rather than deriving material names from labels.
            Quantity_Color color = face.HasFaceColor() ? face.FaceColor().GetRGB()
                : (!material.IsNull() ? material->BaseColor().GetRGB()
                                     : XCAFDoc_VisMaterialCommon().DiffuseColor);
            double r = color.Red(), g = color.Green(), b = color.Blue();
            if (myConvention == Core3DOBJColorConventionEncoded) {
                color.Values(r, g, b, Quantity_TOC_sRGB);
            }
            const PBRScalars values{r, g, b,
                hasPBR ? material->PbrMaterial().Metallic : 0.0,
                hasPBR ? material->PbrMaterial().Roughness : 0.0,
                hasPBR ? 1.0 : 0.0};
            for (double value : values) {
                if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
                    throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                        "An OBJ PBR material has invalid scalar values.");
                }
            }
            const std::string name(writer.ActiveMaterial().ToCString());
            const auto found = myMaterialScalars.find(name);
            if (found != myMaterialScalars.end() && found->second != values) {
                throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                    "An OBJ material name identifies conflicting PBR values.");
            }
            if (found == myMaterialScalars.end()) {
                if (myMaterialScalars.size() >= 100'000) {
                    throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                        "The OBJ material count exceeds the export budget.");
                }
                myMaterialScalars.emplace(name, values);
            }
        }
        return RWObj_CafWriter::writePositions(writer, progress, face);
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
    Core3DOBJColorConvention myConvention;
    std::unordered_map<std::string, PBRScalars> myMaterialScalars;
    NCollection_Map<Handle(Image_Texture)> myTextures;
    Standard_Boolean myCreatesMaterialFile = Standard_False;
};

//! Rewrite only writer-owned coefficients under the captured convention.
//! Current retains legacy common coefficients. Texture references, opacity and
//! non-color scalar values remain intact; explicit modes express opacity as d.
void PreserveOBJMaterialScalars(
    const std::shared_ptr<NativeExportState>& state,
    const ValidatedOBJWriter& writer) {
    const auto& materials = writer.MaterialScalars();
    if (materials.empty()) {
        return;
    }
    auto path = std::filesystem::path(state->primaryPath);
    path.replace_extension(".mtl");
    if (!std::filesystem::is_regular_file(std::filesystem::symlink_status(path))
        || std::filesystem::file_size(path) > 32ULL * 1024ULL * 1024ULL) {
        throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
            "The OBJ material library is missing or exceeds the export budget.");
    }
    const auto temporary = path.string() + ".pbr.tmp";
    std::ifstream input(path, std::ios::binary);
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.imbue(std::locale::classic());
    output << std::setprecision(9);
    std::set<std::string> sections, colors;
    std::string line, active;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        if ((++lineNumber % 256) == 0
            && state->cancelled.load(std::memory_order_acquire)) {
            throw NativeExportFailure(Core3DNativeExportErrorCancelled,
                "OBJ export was cancelled while retaining its materials.");
        }
        if (line.rfind("newmtl ", 0) == 0) {
            active = line.substr(7);
            output << line << '\n';
            const auto found = materials.find(active);
            if (found != materials.end()) {
                if (!sections.insert(active).second) {
                    throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                        "The OBJ material library repeats a PBR section.");
                }
                if (found->second[5] == 1.0) {
                    output << "Pm " << found->second[3] << '\n'
                           << "Pr " << found->second[4] << '\n';
                }
            }
        } else if (line.rfind("Kd ", 0) == 0 && materials.count(active) != 0) {
            if (!colors.insert(active).second) {
                throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                    "The OBJ material library repeats a PBR base color.");
            }
            const auto& values = materials.at(active);
            output << "Kd " << values[0] << ' ' << values[1] << ' ' << values[2] << '\n';
        } else if (state->objColorConvention != Core3DOBJColorConventionCurrent
                   && materials.count(active) != 0 && line.rfind("Tr ", 0) == 0) {
            // Preserve opacity using the widely consumed dissolve directive.
            // Blender ignores OCCT's Tr spelling. Current keeps legacy bytes;
            // explicit modes emit only d so consumers cannot apply both.
            std::istringstream values(line.substr(3));
            values.imbue(std::locale::classic());
            double transparency;
            if (!(values >> transparency) || !(values >> std::ws).eof()
                || !std::isfinite(transparency) || transparency < 0.0 || transparency > 1.0) {
                throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                    "An OBJ material opacity is malformed.");
            }
            output << "d " << (1.0 - transparency) << '\n';
        } else if (state->objColorConvention == Core3DOBJColorConventionLinear
                   && materials.count(active) != 0
                   && (line.rfind("Ka ", 0) == 0 || line.rfind("Ks ", 0) == 0)) {
            // OCCT emits these common fallback coefficients as sRGB. Convert
            // color values only: Ns, Tr, Pm, Pr and encoded images are scalars/data.
            std::istringstream values(line.substr(3));
            values.imbue(std::locale::classic());
            double rgb[3];
            if (!(values >> rgb[0] >> rgb[1] >> rgb[2]) || !(values >> std::ws).eof()) {
                throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                    "An OBJ material color is malformed.");
            }
            output << line.substr(0, 3);
            for (int channel = 0; channel < 3; ++channel) {
                const double value = rgb[channel];
                if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
                    throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
                        "An OBJ material color is outside its supported range.");
                }
                output << (channel == 0 ? "" : " ")
                       << (value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4));
            }
            output << '\n';
        } else {
            output << line << '\n';
        }
    }
    output.flush();
    if (!input.eof() || !output.good()
        || sections.size() != materials.size() || colors != sections) {
        throw NativeExportFailure(Core3DNativeExportErrorInvalidArtifact,
            "The OBJ material library could not retain every PBR material.");
    }
    output.close();
    input.close();
    if (!output.good()) {
        throw NativeExportFailure(Core3DNativeExportErrorWriterFailed,
            "The OBJ material library could not be written.");
    }
    // Both files belong to this private operation. Publish the complete MTL
    // before package validation; any error discards the whole private export.
    std::filesystem::rename(temporary, path);
}

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

bool ApplyPrivateExportTransforms(
    const Handle(OcctDocument)& document,
    const std::shared_ptr<NativeExportState>& state,
    const Message_ProgressRange& progress) {
    const auto& ocaf = document->Document();
    if (ocaf.IsNull() || !ocaf->HasOpenCommand()) { return false; }
    const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(ocaf->Main());
    if (shapeTool.IsNull()) { return false; }
    Message_ProgressScope phases(progress, "Apply export transforms", 2);
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    Message_ProgressScope scaling(phases.Next(), "Bake export scale", roots.Length());
    std::int64_t copiedNodes = 0, copiedTriangles = 0;
    for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
        ThrowIfCancelled(state);
        if (!scaling.More()) { return false; }
        const auto& root = roots.Value(index);
        const gp_Trsf stored = document->ObjectTransformForLabel(root);
        const double factor = stored.ScaleFactor();
        if (factor != 1.0) {
            // Object scale is separate from shape placement. OCCT locations
            // reject scale, so bake it into a detached copy in this private
            // export document before applying the remaining rigid transform.
            if (!std::isfinite(factor) || factor == 0.0
                || !XCAFDoc_ShapeTool::IsSimpleShape(root)
                || XCAFDoc_ShapeTool::IsAssembly(root)
                || XCAFDoc_ShapeTool::IsReference(root)) { return false; }
            const TopoDS_Shape original = shapeTool->GetShape(root);
            if (original.IsNull()) { return false; }
            const auto budgetCopiedMesh = [&](const TopoDS_Shape& shape) {
                for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
                    ThrowIfCancelled(state);
                    TopLoc_Location location;
                    const auto mesh = BRep_Tool::Triangulation(TopoDS::Face(faces.Current()), location);
                    if (!mesh.IsNull()) {
                        copiedNodes += mesh->NbNodes(); copiedTriangles += mesh->NbTriangles();
                        if (copiedNodes > kMaximumSTLNodes || copiedTriangles > kMaximumSTLTriangles) {
                            throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                                "Scaled geometry exceeds the supported mobile export size.");
                        }
                    }
                }
            };
            budgetCopiedMesh(original);
            core3d::profile::Record profile;
            core3d::enclosure::Record enclosure;
            if (!core3d::profile::Read(ocaf, root, profile)
                || !core3d::enclosure::Read(ocaf, root, enclosure)) { return false; }
            const TDF_Label featureLabel = !profile.label.IsNull() ? profile.label : enclosure.label;
            gp_Trsf scale;
            scale.SetScale(gp_Pnt(0, 0, 0), factor);
            // CopyMesh preserves authored UVs and handles negative scale's
            // triangle winding/normals. BRep surfaces stay analytic for STEP.
            BRepBuilderAPI_Transform transformed(original, scale, Standard_True, Standard_True);
            ThrowIfCancelled(state);
            if (!transformed.IsDone() || transformed.Shape().IsNull()) { return false; }
            std::vector<std::pair<TDF_Label, TopoDS_Shape>> replacements;
            const auto captureReplacement = [&](const TDF_Label& label) {
                Handle(TNaming_NamedShape) named;
                if (!label.FindAttribute(TNaming_NamedShape::GetID(), named)) { return true; }
                const bool isFeatureBinding = !featureLabel.IsNull() && label.IsEqual(featureLabel);
                const TopoDS_Shape previous = isFeatureBinding ? named->Get() : shapeTool->GetShape(label);
                if (previous.IsNull()) { return false; }
                TopoDS_Shape replacement;
                if (isFeatureBinding && !previous.IsEqual(original)) {
                    // A stale recipe can retain a different solid. Preserve its
                    // independent binding in this private export copy instead of
                    // asking the current root's modifier to resolve an old shape.
                    budgetCopiedMesh(previous);
                    BRepBuilderAPI_Transform retained(previous, scale, Standard_True, Standard_True);
                    ThrowIfCancelled(state);
                    if (!retained.IsDone()) { return false; }
                    replacement = retained.Shape();
                } else {
                    replacement = transformed.ModifiedShape(previous);
                }
                if (replacement.IsNull()) { return false; }
                replacements.emplace_back(label, replacement);
                return true;
            };
            if (!captureReplacement(root)) { return false; }
            std::size_t labelCount = 0;
            for (TDF_ChildIterator child(root, Standard_True); child.More(); child.Next()) {
                ThrowIfCancelled(state);
                if (++labelCount > 100'000 || !captureReplacement(child.Value())) { return false; }
            }
            // Replace only this root's labels. Global naming substitution can
            // also replace another occurrence sharing the original shape.
            for (const auto& replacement : replacements) {
                ThrowIfCancelled(state);
                if (!featureLabel.IsNull() && replacement.first.IsEqual(featureLabel)) {
                    // Recipe bindings admit their exact codec attributes only.
                    // ShapeTool::SetShape would add XCAFDoc_ShapeMapTool here,
                    // making subsequent private-document validation reject.
                    TNaming_Builder(replacement.first).Generated(replacement.second);
                } else {
                    shapeTool->SetShape(replacement.first, replacement.second);
                }
            }
            // Tag 8 is the persisted object-scale field read by
            // ObjectTransformForLabel. The source snapshot is never modified.
            TDataStd_Real::Set(root.FindChild(8, Standard_True), 1.0);
        }
        scaling.Next();
    }
    return document->ApplyTransforms(phases.Next());
}

Handle(Poly_Triangulation) BuildBinarySTLMesh(
    const TopoDS_Shape& shape,
    const double millimetersPerUnit,
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
            // STL carries no standard unit metadata. Normalize only this
            // detached export mesh; keep the editable document in its units.
            if (millimetersPerUnit != 1.0) {
                point.SetCoord(point.X() * millimetersPerUnit,
                               point.Y() * millimetersPerUnit,
                               point.Z() * millimetersPerUnit);
            }
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

// BRepLib::UpdateDeflection measures corresponding UV/3D midpoint distance.
// The standard Delaunay optimizer instead measures distance to a triangle plane
// or link. A bilinear planar trapezoid can therefore remain at two triangles,
// with a large *tangential* midpoint discrepancy, through every normal retry.
// This final STL-only retry inserts interior nodes on the ORIGINAL surface. It
// does not alter surfaces, pcurves, boundary polygons, deflection metadata or BRep.
using ExportParametricBase = BRepMesh_DelaunayDeflectionControlMeshAlgo<
    BRepMesh_NURBSRangeSplitter, BRepMesh_DelaunayBaseMeshAlgo>;

bool ExportBilinearSurface(const Handle(Geom_BSplineSurface)& spline) {
    return !spline.IsNull() && spline->UDegree()==1 && spline->VDegree()==1
        && spline->NbUPoles()==2 && spline->NbVPoles()==2
        && spline->NbUKnots()==2 && spline->NbVKnots()==2
        && !spline->IsURational() && !spline->IsVRational()
        && !spline->IsUPeriodic() && !spline->IsVPeriodic();
}

struct ExportParametricBudget {
    explicit ExportParametricBudget(unsigned limit=65536): maximumProposals(limit) {}
    const unsigned maximumProposals;
    std::atomic<unsigned> proposedNodes{0};
};

class ExportParametricMesh final : public ExportParametricBase {
public:
    ExportParametricMesh(double requested, std::shared_ptr<ExportParametricBudget> budget)
        : requested_(requested), budget_(std::move(budget)) {}
protected:
    void postProcessMesh(BRepMesh_Delaun& mesher, const Message_ProgressRange& range) override {
        Message_ProgressScope scope(range, "Refine corresponding surface points", 2);
        ExportParametricBase::postProcessMesh(mesher, scope.Next());
        if (!scope.More()) return;
        const auto surface = getDFace()->GetSurface();
        const auto spline = Handle(Geom_BSplineSurface)::DownCast(surface->Surface().Surface());
        // Bounded, non-rational single bilinear span only. No arbitrary spline,
        // rational surface, mesh-only face or UV-authoring route is admitted.
        if (!ExportBilinearSurface(spline) || !std::isfinite(requested_) || requested_<=0) return;
        const double threshold = requested_*0.5;
        const double minimum = getParameters().MinSize;
        if (!std::isfinite(threshold) || threshold<=0 || !std::isfinite(minimum) || minimum<=0) return;
        Message_ProgressScope passes(scope.Next(), "Corresponding point refinement", 10);
        for (int pass=0; pass<10 && passes.More(); ++pass) {
            const auto structure = getStructure();
            if (structure->ElementsOfDomain().Extent()>32768) return;
            Handle(IMeshData::ListOfPnt2d) nodes = new IMeshData::ListOfPnt2d(getAllocator());
            std::set<std::pair<int,int>> seenLinks;
            bool limited = false;
            const auto consider = [&](const gp_XY& uv, const gp_XYZ& linear,
                                      const std::array<gp_XYZ,3>& corners) {
                gp_Pnt exact; surface->D0(uv.X(),uv.Y(),exact);
                if (!std::isfinite(exact.X()) || !std::isfinite(exact.Y()) || !std::isfinite(exact.Z())) {
                    limited=true; return;
                }
                const double distance = (exact.XYZ()-linear).Modulus();
                if (!std::isfinite(distance)) { limited=true; return; }
                if (distance<=threshold) return;
                // Respect the existing mesher's minimum edge-length guard.
                for (const auto& corner : corners) if ((exact.XYZ()-corner).Modulus()<minimum) return;
                if (nodes->Size()>=8192 || budget_->proposedNodes.fetch_add(1)>=budget_->maximumProposals) {
                    limited=true; return;
                }
                nodes->Append(gp_Pnt2d(uv));
            };
            IMeshData::IteratorOfMapOfInteger triangles(structure->ElementsOfDomain());
            for (; triangles.More(); triangles.Next()) {
                if (!passes.More() || limited) return;
                const auto& triangle=structure->GetElement(triangles.Key());
                if (triangle.Movability()==BRepMesh_Deleted) continue;
                int indices[3]; structure->ElementNodes(triangle,indices);
                std::array<gp_XY,3> uv; std::array<gp_XYZ,3> xyz;
                for (int i=0;i<3;++i) {
                    const auto& vertex=structure->GetNode(indices[i]);
                    uv[i]=getRangeSplitter().Scale(vertex.Coord(),Standard_False).XY();
                    xyz[i]=getNodesMap()->Value(vertex.Location3d()).XYZ();
                }
                consider((uv[0]+uv[1]+uv[2])/3.0,(xyz[0]+xyz[1]+xyz[2])/3.0,xyz);
                for (int i=0;i<3 && !limited;++i) {
                    if (structure->GetLink(triangle.myEdges[i]).Movability()==BRepMesh_Frontier) continue;
                    const int j=(i+1)%3;
                    const auto key=std::minmax(indices[i],indices[j]);
                    if (!seenLinks.emplace(key.first,key.second).second) continue;
                    consider((uv[i]+uv[j])/2.0,(xyz[i]+xyz[j])/2.0,xyz);
                }
            }
            if (limited || nodes->IsEmpty()) return;
            // OCCT classifies candidates as TopAbs_IN and evaluates their actual
            // surface point before adding them with BRepMesh_Free authority.
            if (!insertNodes(nodes,mesher,passes.Next())) return;
        }
        // No success is inferred here. Standard OCCT postprocessing measures
        // the resulting mesh and the ORIGINAL requested-deflection gate decides.
    }
private:
    double requested_;
    std::shared_ptr<ExportParametricBudget> budget_;
};

class ExportParametricFactory final : public BRepMesh_MeshAlgoFactory {
public:
    explicit ExportParametricFactory(double requested, unsigned maximumProposals=65536)
        : requested_(requested), budget_(std::make_shared<ExportParametricBudget>(maximumProposals)) {}
    Handle(IMeshTools_MeshAlgo) GetAlgo(GeomAbs_SurfaceType type,
        const IMeshTools_Parameters& parameters) const override {
        if (type==GeomAbs_BSplineSurface) return new ExportParametricMesh(requested_,budget_);
        return BRepMesh_MeshAlgoFactory::GetAlgo(type,parameters);
    }
private:
    double requested_;
    std::shared_ptr<ExportParametricBudget> budget_;
};

void MeshPrivateExportSurfaces(
    const TopoDS_Shape& compound,
    const std::shared_ptr<NativeExportState>& state,
    const Message_ProgressRange& progress) {
    BRep_Builder builder;
    Message_ProgressScope whole(progress, "Prepare private export mesh", 4);
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

    // Only analytic faces are remeshed. Mesh-only faces carry
    // authored triangles/UVs and never enter the mesher or Clean.
    // Everything here belongs to the deserialized private document.
    TopoDS_Compound geometricFaces;
    builder.MakeCompound(geometricFaces);
    TopoDS_Shape meshingShape = compound;
    bool hasGeometricFaces = false;
    if (state->meshQuality != Core3DExportMeshQualityViewport) {
        Standard_Integer faceCount = 0;
        for (TopExp_Explorer faces(compound, TopAbs_FACE); faces.More(); faces.Next()) {
            ThrowIfCancelled(state);
            if (++faceCount > 4096) {
                throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                    "Mesh quality presets support up to 4,096 faces. Use viewport quality for this model.");
            }
            const TopoDS_Face face = TopoDS::Face(faces.Current());
            if (!BRep_Tool::Surface(face).IsNull()) {
                BRepTools::Clean(face);
                builder.Add(geometricFaces, face);
                hasGeometricFaces = true;
            }
        }
        meshingShape = geometricFaces;
    }
    const bool needsMeshing = state->meshQuality == Core3DExportMeshQualityViewport
        ? !BRepTools::Triangulation(compound, deflection) : hasGeometricFaces;
    if (needsMeshing) {
        const bool qualityPreset = state->meshQuality != Core3DExportMeshQualityViewport;
        // Four existing attempts stay byte-for-byte in order. Only STL may
        // make a fifth, bounded original-surface parameter refinement attempt.
        const bool parametricRetry = qualityPreset && state->exportType == ExportTypeStl;
        const int maximumAttempts = qualityPreset ? (parametricRetry ? 5 : 4) : 1;
        Message_ProgressScope meshScope(whole.Next(4), "Refine export mesh", maximumAttempts);
        bool validMesh = false;
        for (int attempt = 0; attempt < maximumAttempts; ++attempt) {
            ThrowIfCancelled(state);
            // Keep the original first refinement and final fallback. Before
            // the final fourfold refinement step, try a half-sized step: a
            // transformed curved face may only narrowly miss the same proof.
            // Every accepted mesh still satisfies the requested deflection.
            if (attempt == 4) {
                bool hasEligibleFailure = false;
                for (TopExp_Explorer faces(meshingShape, TopAbs_FACE); faces.More(); faces.Next()) {
                    ThrowIfCancelled(state);
                    const auto face = TopoDS::Face(faces.Current());
                    const auto spline = Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face));
                    if (!BRepTools::Triangulation(face, deflection) && ExportBilinearSurface(spline)) {
                        hasEligibleFailure = true; break;
                    }
                }
                if (!hasEligibleFailure) break;
            }
            if (attempt > 0) { BRepTools::Clean(meshingShape); }
            constexpr double refinementFactors[] = {1.0, 0.25, 0.125, 0.0625, 0.0625};
            const double target = attempt == 0 ? deflection
                : std::max(Precision::Confusion(), deflection * refinementFactors[attempt]);
            BRepMesh_IncrementalMesh mesher;
            mesher.ChangeParameters().Deflection = target;
            mesher.ChangeParameters().Angle = state->deviationAngle;
            mesher.ChangeParameters().InParallel = Standard_True;
            bool controlInterior = attempt > 0;
#ifdef DEBUG
            controlInterior = controlInterior || state->debugInitialInteriorControl;
#endif
            if (controlInterior) {
                mesher.ChangeParameters().DeflectionInterior = target;
                mesher.ChangeParameters().AngleInterior = state->deviationAngle;
                mesher.ChangeParameters().EnableControlSurfaceDeflectionAllSurfaces = Standard_True;
            }
            mesher.SetShape(meshingShape);
            if (attempt == 4) {
                Handle(IMeshTools_Context) context = new BRepMesh_Context();
                context->SetFaceDiscret(new BRepMesh_FaceDiscret(new ExportParametricFactory(deflection)));
                mesher.Perform(context, meshScope.Next(1));
            } else {
                mesher.Perform(meshScope.Next(1));
            }
            ThrowIfCancelled(state);
            if (qualityPreset) {
                std::int64_t nodes = 0, triangles = 0;
                for (TopExp_Explorer faces(meshingShape, TopAbs_FACE); faces.More(); faces.Next()) {
                    ThrowIfCancelled(state);
                    TopLoc_Location location;
                    const auto mesh = BRep_Tool::Triangulation(TopoDS::Face(faces.Current()), location);
                    if (!mesh.IsNull()) { nodes += mesh->NbNodes(); triangles += mesh->NbTriangles(); }
                    if (nodes > kMaximumSTLNodes || triangles > kMaximumSTLTriangles) {
                        throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                            "The export mesh is too detailed. Choose a coarser quality or export fewer objects.");
                    }
                }
            }
            validMesh = mesher.IsDone() && BRepTools::Triangulation(meshingShape, deflection);
#if DEBUG
            if (!validMesh || attempt > 0 || state->debugInitialInteriorControl) {
                NSLog(@"[NativeExportMeshDiagnostic] attempt=%d valid=%d done=%d flags=%d target=%.17g requested=%.17g",
                    attempt, validMesh, mesher.IsDone(), mesher.GetStatusFlags(), target, deflection);
                Standard_Integer diagnosticFace = 0;
                for (TopExp_Explorer faces(meshingShape, TopAbs_FACE); faces.More() && diagnosticFace < 16; faces.Next()) {
                    TopLoc_Location location;
                    const auto face = TopoDS::Face(faces.Current());
                    const auto mesh = BRep_Tool::Triangulation(face, location);
                    const auto diagnosticSpline=Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face));
                    NSLog(@"[NativeExportMeshDiagnostic] face=%d mesh=%d deflection=%.17g nodes=%d triangles=%d valid=%d bilinear=%d",
                        ++diagnosticFace, !mesh.IsNull(), mesh.IsNull() ? -1.0 : mesh->Deflection(),
                        mesh.IsNull() ? 0 : mesh->NbNodes(), mesh.IsNull() ? 0 : mesh->NbTriangles(),
                        BRepTools::Triangulation(face, deflection),ExportBilinearSurface(diagnosticSpline));
                }
            }
#endif
            if (validMesh) { break; }
        }
        if (!validMesh) {
            throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                "The committed geometry could not be meshed within the requested quality.");
        }
    } else {
        whole.Next(4).Close();
    }
    ThrowIfCancelled(state);

    if (state->meshQuality != Core3DExportMeshQualityViewport) {
        std::int64_t nodes = 0, triangles = 0;
        for (TopExp_Explorer faces(compound, TopAbs_FACE); faces.More(); faces.Next()) {
            ThrowIfCancelled(state);
            TopLoc_Location location;
            const Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(
                TopoDS::Face(faces.Current()), location);
            if (mesh.IsNull()) {
                throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                    "An export face has no mesh.");
            }
            nodes += mesh->NbNodes();
            triangles += mesh->NbTriangles();
            if (nodes > kMaximumSTLNodes || triangles > kMaximumSTLTriangles) {
                throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                    "The export mesh is too detailed. Choose a coarser quality or export fewer objects.");
            }
        }
    }

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

        if (state->sourceScene) {
            result.scene = state->meshQuality == Core3DExportMeshQualityViewport
                ? state->sourceScene
                : core3d::scene::OcctSceneSnapshotBuilder::BuildPrivateExportDerivative(
                document, *state->sourceScene, state->selectedObjectsOnly,
                [&](const TopoDS_Shape& shape) {
                    MeshPrivateExportSurfaces(shape, state, whole.Next(4));
                }, [&] { return state->cancelled.load(std::memory_order_acquire); });
            ThrowIfCancelled(state);
            if (!result.scene) {
                throw NativeExportFailure(Core3DNativeExportErrorMeshingFailed,
                    "The private GLB geometry could not be prepared safely.");
            }
        } else {
        const Handle(TDocStd_Document)& ocafDocument = document->Document();
        if (ocafDocument.IsNull()
            || ocafDocument->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                ocafDocument->Main())) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The private export document is incomplete.");
        }
		const OcctGeometryExportFormat geometryExportFormat =
			state->exportType == ExportTypeObj
				? OcctGeometryExportFormat::Obj
				: state->exportType == ExportTypeStl
					? OcctGeometryExportFormat::Stl
					: OcctGeometryExportFormat::Step;
		if (!document->CanExportGeometry(geometryExportFormat)
			&& !document->IsGeometryDocumentEmpty()) {
			throw NativeExportFailure(
				Core3DNativeExportErrorInvalidState,
				"The geometry representation cannot be exported in this format.");
		}
        ocafDocument->SetUndoLimit(1);
        ocafDocument->NewCommand();
        if (!ocafDocument->HasOpenCommand()) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The private export transaction could not be opened.");
        }
        if (!ApplyPrivateExportTransforms(document, state, whole.Next(1))) {
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
        std::set<std::string> matchedSelectedIdentifiers;
        for (Standard_Integer index = 1;
             index <= freeLabels.Length();
             ++index) {
            ThrowIfCancelled(state);
            const TDF_Label& label = freeLabels.Value(index);
            if (shapeTool->GetShape(label).IsNull()) {
                continue;
            }
            if (state->usesSelectedRoots) {
                const std::string identifier = document->EntityIdentifierForLabel(label);
                if (state->selectedEntityIdentifiers.count(identifier) == 0) {
                    continue;
                }
                if (!matchedSelectedIdentifiers.insert(identifier).second) {
                    throw NativeExportFailure(
                        Core3DNativeExportErrorInvalidState,
                        "A selected export identity matches more than one object.");
                }
            }
            rootLabels.Append(label);
        }
        if (state->usesSelectedRoots
            && matchedSelectedIdentifiers != state->selectedEntityIdentifiers) {
            throw NativeExportFailure(
                Core3DNativeExportErrorInvalidState,
                "The private document does not contain the exact selected objects.");
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

            MeshPrivateExportSurfaces(compound, state, whole.Next(4));

            if (state->exportType == ExportTypeStl) {
                Standard_Real metersPerUnit = 0.001;
                if (!XCAFDoc_DocumentTool::GetLengthUnit(ocafDocument, metersPerUnit)) {
                    metersPerUnit = 0.001;
                }
                const double millimetersPerUnit = metersPerUnit * 1000.0;
                if (!std::isfinite(millimetersPerUnit) || millimetersPerUnit <= 0.0) {
                    throw NativeExportFailure(Core3DNativeExportErrorInvalidState,
                        "The STL source unit cannot be represented in millimeters.");
                }
                Message_ProgressScope stlScope(
                    whole.Next(5),
                    "Binary STL export",
                    2);
                const Handle(Poly_Triangulation) stlMesh =
                    BuildBinarySTLMesh(
                        compound,
                        millimetersPerUnit,
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
                    TCollection_AsciiString(state->primaryPath.c_str()), state->objColorConvention);
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
                PreserveOBJMaterialScalars(state, writer);
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
    if (!result.succeeded || state->sourceScene) {
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


#ifdef DEBUG
// This is a fixed analytic-face test oracle, not a production persistence digest.
std::string DebugExportBRep(const TopoDS_Shape& shape) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream, Standard_False, Standard_False,
                    TopTools_FormatVersion_VERSION_3);
    if (!stream || stream.str().empty() || stream.str().size() > 128 * 1024) {
        throw std::runtime_error("Invalid bounded export-probe BRep");
    }
    return stream.str();
}

std::string DebugExportAnalyticGeometry(const TopoDS_Face& face) {
    BRepTools_ShapeSet shapes(Standard_False, Standard_False);
    shapes.SetFormatNb(TopTools_FormatVersion_VERSION_3);
    // Add recursively inserts children before the root (OCCT 7.8). This fixture
    // has exactly one face, one wire, four edges and four vertices.
    const auto count = shapes.Add(face);
    if (count != 10) throw std::runtime_error("Unexpected probe topology");
    std::ostringstream locations;
    locations.imbue(std::locale::classic());
    shapes.Locations().Write(locations);
    std::istringstream locationInput(locations.str());
    std::string label; int locationCount = -1;
    locationInput >> label >> locationCount;
    locationInput >> std::ws;
    // All geometric and topological locations of this fixed fixture are identity.
    // Refuse unexpected transforms instead of using the location writer's 15-digit
    // representation as an exact matrix proof.
    if (label != "Locations" || locationCount != 0 || !locationInput.eof()) {
        throw std::runtime_error("Unexpected probe location");
    }
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    // Native geometry tables include complete surfaces, 3D curves and pcurves.
    // OCCT's GeomTools writers use 17 digits. Per-shape geometry below retains
    // vertex points, tolerances, all curve ranges and edge geometric flags.
    shapes.WriteGeometry(stream);
    std::array<int, 8> counts{};
    for (int index = 1; index <= count; ++index) {
        const auto& shape = shapes.Shape(index);
        const int type = static_cast<int>(shape.ShapeType());
        if (type < 0 || type >= static_cast<int>(counts.size())
            || !shape.Location().IsIdentity()) {
            throw std::runtime_error("Invalid probe shape");
        }
        ++counts[type];
        stream << "shape " << index << ' ' << type << ' ';
        shapes.Write(shape, stream);
        stream << ' ' << shape.Orientable() << ' ' << shape.Closed()
               << ' ' << shape.Infinite() << ' ' << shape.Convex() << '\n';
        shapes.WriteGeometry(shape, stream);
        // Free/Modified/Checked are deliberately not a private-derivative geometry
        // invariant: OCCT cache updates write operational TShape flags. The original
        // source is separately compared using the complete, unfiltered BRep bytes.
        for (TopoDS_Iterator child(shape, Standard_False, Standard_False);
             child.More(); child.Next()) {
            shapes.Write(child.Value(), stream); // exact order, orientation and link
        }
        stream << "end\n";
    }
    if (counts[TopAbs_VERTEX] != 4 || counts[TopAbs_EDGE] != 4
        || counts[TopAbs_WIRE] != 1 || counts[TopAbs_FACE] != 1
        || !stream || stream.str().empty() || stream.str().size() > 128 * 1024) {
        throw std::runtime_error("Incomplete probe geometry proof");
    }
    return stream.str();
}
#endif

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
           selectedEntityIdentifiers:(NSArray<NSString *> *)selectedEntityIdentifiers
                         meshQuality:(Core3DExportMeshQuality)meshQuality
                  objColorConvention:(Core3DOBJColorConvention)objColorConvention
                      deflectionType:(NSInteger)deflectionType
                deviationCoefficient:(double)deviationCoefficient
                       deviationAngle:(double)deviationAngle
            maximalChordialDeviation:(double)maximalChordialDeviation
                         sourceScene:(core3d::scene::OcctSceneSnapshotBuilder::SnapshotPointer)sourceScene
                 selectedObjectsOnly:(BOOL)selectedObjectsOnly {
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
            && exportType != ExportTypeStep
            && exportType != ExportTypeGltf)
        || ((exportType == ExportTypeGltf) != (sourceScene != nullptr))
        || objColorConvention < Core3DOBJColorConventionCurrent
        || objColorConvention > Core3DOBJColorConventionEncoded
        || (exportType != ExportTypeObj && objColorConvention != Core3DOBJColorConventionCurrent)
        || meshQuality < Core3DExportMeshQualityViewport
        || meshQuality > Core3DExportMeshQualityFine
        || (meshQuality != Core3DExportMeshQualityViewport
            && exportType == ExportTypeStep)
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

    std::set<std::string> selectedIdentifiers;
    if (selectedEntityIdentifiers != nil) {
        if ((exportType != ExportTypeStl && exportType != ExportTypeObj && exportType != ExportTypeGltf)
            || selectedEntityIdentifiers.count == 0
            || selectedEntityIdentifiers.count > 50'000) {
            return nil;
        }
        for (NSString *identifier in selectedEntityIdentifiers) {
            NSData *bytes = [identifier dataUsingEncoding:NSUTF8StringEncoding
                                    allowLossyConversion:NO];
            if (bytes.length == 0 || bytes.length > 128
                || memchr(bytes.bytes, 0, bytes.length) != nullptr) {
                return nil;
            }
            const std::string value(static_cast<const char *>(bytes.bytes), bytes.length);
            if (!selectedIdentifiers.insert(value).second) {
                return nil;
            }
        }
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
    _state->sourceScene = std::move(sourceScene);
    _state->selectedObjectsOnly = selectedObjectsOnly;
    _state->meshQuality = meshQuality;
    _state->objColorConvention = objColorConvention;
    _state->usesSelectedRoots = selectedEntityIdentifiers != nil;
    _state->selectedEntityIdentifiers = std::move(selectedIdentifiers);
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

- (Core3DOBJColorConvention)objColorConvention {
    return _state == nullptr ? Core3DOBJColorConventionCurrent : _state->objColorConvention;
}

- (Core3DExportMeshQuality)meshQuality {
    return _state == nullptr ? Core3DExportMeshQualityViewport : _state->meshQuality;
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
    [self startWithArtifactCompletion:completion snapshotCompletion:nil];
}

- (void)startSceneSnapshotWithCompletion:(Core3DExportSceneCompletion)completion {
    [self startWithArtifactCompletion:nil snapshotCompletion:completion];
}

- (void)startWithArtifactCompletion:(Core3DNativeExportCompletion)artifactCompletion
                snapshotCompletion:(Core3DExportSceneCompletion)snapshotCompletion {
    Core3DNativeExportCompletion completion = artifactCompletion ?: ^(Core3DNativeExportArtifact *artifact, NSError *error) {
        if (snapshotCompletion) { snapshotCompletion(nil, error); }
    };
    if (artifactCompletion == nil && snapshotCompletion == nil) { return; }
    if (_state && ((_state->sourceScene != nullptr) != (snapshotCompletion != nil))) {
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(nil, [NSError errorWithDomain:Core3DNativeExportErrorDomain
                code:Core3DNativeExportErrorInvalidState
                userInfo:@{NSLocalizedDescriptionKey: @"Use the completion matching the prepared export format."}]);
        });
        return;
    }
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
            if (result.succeeded && result.scene) {
                if (state->cancelled.load(std::memory_order_acquire)) {
                    snapshotCompletion(nil, [NSError errorWithDomain:Core3DNativeExportErrorDomain
                        code:Core3DNativeExportErrorCancelled userInfo:nil]);
                    return;
                }
                Core3DSceneSnapshot *snapshot = Core3DCreateSceneSnapshotDTO(*result.scene);
                snapshotCompletion(snapshot, snapshot ? nil : [NSError errorWithDomain:Core3DNativeExportErrorDomain
                    code:Core3DNativeExportErrorInvalidArtifact userInfo:nil]);
                return;
            }
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
+ (NSDictionary<NSString *, id> *)debugParametricSTLRefinement:(double)metersPerUnit {
    NSString *stage = @"admission";
    if (![NSThread isMainThread] || (metersPerUnit!=0.001 && metersPerUnit!=1.0)) return @{@"failureStage":stage};
    try {
        const double scale=0.001/metersPerUnit, requested=0.12*scale;
        TColgp_Array2OfPnt poles(1,2,1,2);
        poles(1,1)=gp_Pnt(-10*scale,-6*scale,0); poles(2,1)=gp_Pnt(-10*scale,6*scale,0);
        poles(1,2)=gp_Pnt(-9*scale,-10*scale,60*scale); poles(2,2)=gp_Pnt(-9*scale,10*scale,60*scale);
        TColStd_Array1OfReal knots(1,2); knots(1)=0; knots(2)=1;
        TColStd_Array1OfInteger multiplicities(1,2); multiplicities(1)=2; multiplicities(2)=2;
        Handle(Geom_BSplineSurface) surface=new Geom_BSplineSurface(poles,knots,knots,multiplicities,multiplicities,1,1);
        stage = @"make-face";
        BRepBuilderAPI_MakeFace maker(surface,Precision::Confusion());
        if (!maker.IsDone()) return @{@"failureStage":stage};
        const TopoDS_Face source = maker.Face();
        stage = @"source-write";
        const auto sourceBefore = DebugExportBRep(source);
        // Production exports deserialize a separate saved OCAF document. This
        // bounded probe reproduces the geometry isolation through an actual native
        // serialize/read, rather than remeshing the object named "source".
        stage = @"private-read";
        TopoDS_Shape privateShape;
        std::istringstream snapshot(sourceBefore);
        snapshot.imbue(std::locale::classic());
        BRepTools::Read(privateShape, snapshot, BRep_Builder());
        if (snapshot.fail() || privateShape.IsNull() || privateShape.ShapeType() != TopAbs_FACE) return @{@"failureStage":stage};
        const TopoDS_Face face = TopoDS::Face(privateShape);
        stage = @"isolation";
        BRepTools_ShapeSet sourceShapes(Standard_False, Standard_False), privateShapes(Standard_False, Standard_False);
        const auto sourceCount = sourceShapes.Add(source), privateCount = privateShapes.Add(face);
        if (sourceCount != 10 || privateCount != 10) return @{@"failureStage":stage};
        bool isolated = BRep_Tool::Surface(source) != BRep_Tool::Surface(face);
        for (int a = 1; a <= sourceCount; ++a) {
            for (int b = 1; b <= privateCount; ++b) {
                isolated = isolated && !sourceShapes.Shape(a).IsPartner(privateShapes.Shape(b));
            }
        }
        if (!isolated) return @{@"failureStage":stage};
        stage = @"initial-geometry";
        const auto derivativeBefore = DebugExportAnalyticGeometry(face);
        const auto derivativeBRepBefore = DebugExportBRep(face);
        // Independent bilinear-corner interpolation error; this is not the
        // deflection of the standard mesher, which can already insert vertices.
        gp_Pnt center; surface->D0(0.5, 0.5, center);
        const gp_Pnt diagonalA((poles(1,1).XYZ() + poles(2,2).XYZ()) * 0.5);
        const gp_Pnt diagonalB((poles(2,1).XYZ() + poles(1,2).XYZ()) * 0.5);
        const double cornerGap = std::max(center.Distance(diagonalA), center.Distance(diagonalB));
        stage = @"baseline-mesh";
        BRepMesh_IncrementalMesh baseline(face,requested,Standard_False,0.1,Standard_False);
        TopLoc_Location location;
        const auto baselineMesh=BRep_Tool::Triangulation(face,location);
        if (baselineMesh.IsNull()) return @{@"failureStage":stage};
        const double baselineDeflection=baselineMesh->Deflection();
        const bool baselineRejected=!BRepTools::Triangulation(face,requested);
        stage = @"baseline-geometry";
        const auto baselineGeometry = DebugExportAnalyticGeometry(face);
        stage = @"limited-mesh";
        BRepTools::Clean(face);
        BRepMesh_IncrementalMesh limited;
        limited.SetShape(face);limited.ChangeParameters().Deflection=requested;
        limited.ChangeParameters().Angle=0.1;
        Handle(IMeshTools_Context) limitedContext=new BRepMesh_Context();
        limitedContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new ExportParametricFactory(requested,0)));
        limited.Perform(limitedContext);
        const bool budgetRefused=!BRepTools::Triangulation(face,requested);
        stage = @"limited-geometry";
        const auto limitedGeometry = DebugExportAnalyticGeometry(face);
        auto state=std::make_shared<NativeExportState>();
        state->meshQuality=Core3DExportMeshQualityFine; state->exportType=ExportTypeStl;
        state->deflectionType=Aspect_TOD_ABSOLUTE;state->maximalChordialDeviation=requested;
        state->deviationAngle=0.1;
        stage = @"refinement";
        MeshPrivateExportSurfaces(face,state,Message_ProgressRange());
        const auto refined=BRep_Tool::Triangulation(face,location);
        if(refined.IsNull())return @{@"failureStage":stage};
        stage = @"refined-geometry";
        const auto refinedGeometry = DebugExportAnalyticGeometry(face);
        stage = @"cancel";
        bool cancelled=false;state->cancelled.store(true);
        try {MeshPrivateExportSurfaces(face,state,Message_ProgressRange());}
        catch(const NativeExportFailure& failure){cancelled=failure.Code()==Core3DNativeExportErrorCancelled;}
        stage = @"final-source";
        const auto sourceAfter = DebugExportBRep(source);
        const bool unchanged = sourceBefore == sourceAfter;
        stage = @"cancelled-geometry";
        const auto cancelledGeometry = DebugExportAnalyticGeometry(face);
        const bool derivativeUnchanged = derivativeBefore == baselineGeometry
            && derivativeBefore == limitedGeometry && derivativeBefore == refinedGeometry
            && derivativeBefore == cancelledGeometry;
        stage = @"diagnostics";
        NSDictionary *failureDiagnostics = @{};
        if (!unchanged || !derivativeUnchanged) {
            const auto nativeText = [](const std::string& bytes) -> NSString * {
                return [[NSString alloc] initWithBytes:bytes.data() length:bytes.size()
                    encoding:NSUTF8StringEncoding];
            };
            failureDiagnostics = @{@"sourceBefore":nativeText(sourceBefore),
                @"sourceAfter":nativeText(sourceAfter),
                @"privateBRepBefore":nativeText(derivativeBRepBefore),
                @"privateBRepAfter":nativeText(DebugExportBRep(face)),
                @"geometryBefore":nativeText(derivativeBefore),
                @"geometryAfterBaseline":nativeText(baselineGeometry),
                @"geometryAfterLimited":nativeText(limitedGeometry),
                @"geometryAfterRefinement":nativeText(refinedGeometry),
                @"geometryAfterCancellation":nativeText(cancelledGeometry)};
        }
        stage = @"eligibility-controls";
        auto rational=Handle(Geom_BSplineSurface)::DownCast(surface->Copy());rational->SetWeight(1,1,2.0);
        auto higher=Handle(Geom_BSplineSurface)::DownCast(surface->Copy());higher->IncreaseDegree(2,2);
        return @{@"baselineDone":@(baseline.IsDone() && baseline.GetStatusFlags()==0),
            @"baselineQualityRejected":@(baselineRejected),@"baselineDeflection":@(baselineDeflection),
            @"requested":@(requested),@"refinedAccepted":@(BRepTools::Triangulation(face,requested)),
            @"refinedDeflection":@(refined->Deflection()),@"sourceBRepUnchanged":@(unchanged),
            @"privateSnapshotIsolated":@(isolated),@"derivativeGeometryUnchanged":@(derivativeUnchanged),
            @"bilinearCornerGap":@(cornerGap),@"failureDiagnostics":failureDiagnostics,
            @"budgetRefused":@(budgetRefused),@"cancelled":@(cancelled),
            @"rationalRefused":@(!ExportBilinearSurface(rational)),@"higherDegreeRefused":@(!ExportBilinearSurface(higher))};
    } catch (...) {return @{@"failureStage":stage};}
}

- (BOOL)debugUseAbsoluteChordMM:(double)chordMM
                   angleDegrees:(double)angleDegrees
         initialInteriorControl:(BOOL)initialInteriorControl {
    const std::shared_ptr<NativeExportState> state = _state;
    if (state == nullptr) {
        return NO;
    }
    std::lock_guard<std::mutex> lock(state->lifecycleMutex);
    if (state->started || state->finished ||
        state->cancelled.load(std::memory_order_acquire)) {
        return NO;
    }
    if (state->exportType != ExportTypeGltf ||
        state->meshQuality != Core3DExportMeshQualityStandard) {
        return NO;
    }
    if (state->sourceScene == nullptr) {
        return NO;
    }
    if (!std::isfinite(chordMM) || !std::isfinite(angleDegrees) ||
        chordMM < 0.01 || chordMM > 10.0 ||
        angleDegrees < 1.0 || angleDegrees > 45.0) {
        return NO;
    }
    const double metersPerUnit = state->sourceScene->metersPerUnit;
    if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0.0) {
        return NO;
    }
    const double chord = (chordMM / 1000.0) / metersPerUnit;
    if (!std::isfinite(chord) || chord <= 0.0) {
        return NO;
    }
    state->deflectionType = Aspect_TOD_ABSOLUTE;
    state->maximalChordialDeviation = chord;
    state->deviationAngle = angleDegrees * M_PI / 180.0;
    state->debugInitialInteriorControl = initialInteriorControl;
    return YES;
}

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
