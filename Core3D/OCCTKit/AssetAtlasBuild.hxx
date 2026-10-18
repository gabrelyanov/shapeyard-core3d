#pragma once

// E2a asset-atlas capture and build. All members are resolved and captured
// (row-268 producer route) before any detached chart work; the existing
// shapeyard::uv::curved chart kernels and the existing curveduv::pack packer
// are reused, and the packer runs exactly once per build over the merged
// chart set of all members, producing one globalTexelsPerMM. No second
// packer, no per-member packing, no member-shape mutation.
#include "AssetAtlasDefinition.hxx"
#include "AssetAtlasPersistence.hxx"
#include "DecalLayerBake.hxx"
#include "FaceImagePersistence.hxx"
#include "RetainedFinishingProducer.hxx"

#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <atomic>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace core3d::asset_atlas {

struct MemberCapture final {
    Member slot;                                 // fences; member UUID assigned at build
    retained_finishing::producer::Capture capture; // row-268 capture, verbatim
    retained_finishing::Definition finishing;      // fenced receipt read-back
    TDF_Label ownerLabel;
    bool painted = false;                          // image-backed painted content detected
};
struct Capture final { std::vector<MemberCapture> members; };

namespace build {

struct Settings final { int resolutionTexels = 2048; int gutterTexels = 2; };
enum class Status : std::uint8_t {
    Captured, Built, Refused, StaleSource, MissingMember, ForeignMember,
    OverBudget, PaintedRebakeRequired, UnsupportedSurface, OwnerMismatch,
    Busy, Malformed, PersistenceFailure
};
struct LayoutEvidence final {           // optional BuildAtlas out-param
    curveduv::PackResult packed;        // the one asset-wide pack result
    std::vector<curveduv::ChartInput> charts; // exact merged packer input
    std::vector<std::pair<std::size_t, std::uint64_t>> chartMembers;
    // The source face is structural provenance for later ordered/masked
    // sampling. It is carried beside member/ordinal without changing any
    // chart, packer, UV or sampling arithmetic in this portion.
    std::vector<std::size_t> chartFaces;
};

//! Required accounting state for private final-atlas production. The caller
//! supplies the one operation view and retains the layout/result tickets for
//! as long as the corresponding vectors are physically live. The chart and
//! packer scratch ticket is local to BuildFinalAtlas.
struct FinalAtlasAccounting final {
    decal_layer::bake::accounting::View operation;
    decal_layer::bake::accounting::Ticket layoutStorage;
    decal_layer::bake::accounting::Ticket resultStorage;
    std::size_t roleCount = 0;
    std::size_t sourceCount = 0;
    std::size_t maskCount = 0;
    std::size_t jobUnits = 0;
    bool sealAtlasJob = true;
#ifdef DEBUG
    bool denyLayoutReservation = false;
    std::size_t memberCount = 0;
    std::size_t triangleCount = 0;
    std::size_t cornerCount = 0;
    std::size_t layoutReservationAttempts = 0;
    std::size_t packerEntries = 0;
#endif
};

//! Detached final-representation input. The immutable member/source fences
//! are captured before disposable export work; only developed geometry comes
//! from the final representation. No label or mutable document authority is
//! carried across this boundary.
struct FinalMemberInput final {
    Member savedMember;
    retained_finishing::producer::Capture authority;
    std::vector<shapeyard::uv::Triangle> triangles;
    std::vector<shapeyard::uv::curved::FaceInput> faces;
};

#ifdef DEBUG
//! Borrowed passive observation hooks. The opaque context is owned by the
//! caller and cannot influence product control flow or accounting.
struct DebugLayoutObserver final {
    void* context = nullptr;
    void (*lineage)(void*, std::uint64_t, std::uint64_t, std::uint64_t,
        std::int32_t, std::int32_t, std::int32_t, std::int32_t) noexcept = nullptr;
    void (*island)(void*, std::int32_t, std::int32_t, std::uint64_t,
        std::uint64_t, std::uint64_t, const UUID&) noexcept = nullptr;
    void (*finish)(void*,
        const std::vector<std::vector<std::vector<int>>>&,
        const std::vector<curveduv::SharedEdge>&) noexcept = nullptr;
};
#endif

// The legacy entry points always use Reject. Preserve is consumed only by
// the separately named E2b opt-in bake after it captures both XCAF material
// slots and every SYFI binding/resource fence.
enum class PaintedAdmission : std::uint8_t { Reject = 0, Preserve = 1 };

namespace detail {

// Structural image-content check mirroring PrepareTriangleUVAtlas: any
// image-backed texture slot on the owner's visual material marks the member
// painted. 278c: the read-set must enumerate BOTH XCAF material slots and
// SYFI/1 binding records, so a present non-empty face-image binding record on
// the owner also marks the member painted. Detection only; no rebake is
// attempted here, and MemberCapture.painted is never cleared to obtain an
// admissible layout.
inline bool Painted(const Handle(TDocStd_Document)& document,
                    const TDF_Label& label) noexcept {
    try {
        if (document.IsNull() || label.IsNull()) return false;
        face_image::Definition bindings;
        if (face_image::persistence::bindings::Read(document, label, bindings)
                == face_image::persistence::bindings::ReadState::Present
            && !bindings.bindings.empty()) return true;
        TDF_Label materialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label, materialLabel);
        if (materialLabel.IsNull()) return false;
        const auto tool = XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (tool.IsNull()) return false;
        const auto material = tool->GetMaterial(materialLabel);
        if (material.IsNull()) return false;
        bool painted = material->HasCommonMaterial()
            && !material->CommonMaterial().DiffuseTexture.IsNull();
        if (material->HasPbrMaterial()) {
            const auto& pbr = material->PbrMaterial();
            painted = painted || !pbr.BaseColorTexture.IsNull() || !pbr.EmissiveTexture.IsNull()
                || !pbr.NormalTexture.IsNull() || !pbr.MetallicRoughnessTexture.IsNull()
                || !pbr.OcclusionTexture.IsNull();
        }
        return painted;
    } catch (...) { return false; }
}

// Stable member identity: UUIDFromDigest(Hash(atlas | entity | definition))
// with the row-271 member discriminator 0xA6 (0xA7 stays the frozen chart
// discriminator; 0xA6 is unused by any row-266/268 derivation).
inline UUID MemberUUID(const UUID& atlas, const OwnerKey& owner) noexcept {
    std::vector<std::uint8_t> bytes(atlas.begin(), atlas.end());
    bytes.insert(bytes.end(), owner.entity.begin(), owner.entity.end());
    bytes.insert(bytes.end(), owner.definition.begin(), owner.definition.end());
    Digest digest{};
    if (!retained_solid::Hash(bytes, digest)) return UUID{};
    return retained_finishing::producer::detail::UUIDFromDigest(digest, 0xA6);
}

// Stable chart identity: UUIDFromDigest(Hash(atlas | member |
// memberChartOrdinal | subChartIndex), 0xA7). memberChartOrdinal is
// member-LOCAL, so sibling membership edits never renumber survivors.
// subChartIndex is the fitted sub-chart's segment rank within its own
// kernel chart (the packer's fit-driven split order along the chart's long
// developed axis, reproduced by SegmentRanks), which keeps a survivor's
// chart identities stable across membership edits and regeneration whenever
// its face structure and split count are preserved.
inline UUID ChartUUID(const UUID& atlas, const UUID& member,
                      std::uint64_t memberChartOrdinal,
                      std::uint64_t subChartIndex) noexcept {
    std::vector<std::uint8_t> bytes(atlas.begin(), atlas.end());
    bytes.insert(bytes.end(), member.begin(), member.end());
    for (unsigned index = 0; index < 8; ++index)
        bytes.push_back(std::uint8_t(memberChartOrdinal >> (8 * index)));
    for (unsigned index = 0; index < 8; ++index)
        bytes.push_back(std::uint8_t(subChartIndex >> (8 * index)));
    Digest digest{};
    if (!retained_solid::Hash(bytes, digest)) return UUID{};
    return retained_finishing::producer::detail::UUIDFromDigest(digest, 0xA7);
}

// Reproduces the packer's fit-driven segment index for every (chartIndex,
// subChartIndex) pair in a pack output: within one input chart, sub-charts
// are emitted in centroid order along the chart's long developed axis
// (exactly the packer's strip-split segment order). Rank 0 is the first
// segment. Deterministic; independent of the packer's own claims.
inline std::map<std::pair<int, int>, std::uint64_t> SegmentRanks(
    const std::vector<curveduv::ChartInput>& charts,
    const curveduv::PackResult& packed) noexcept {
    std::map<std::pair<int, int>, std::uint64_t> ranks;
    try {
        struct GroupStat { double centroid = 0; long triangles = 0; };
        std::map<std::pair<int, int>, GroupStat> stats;
        for (const auto& placed : packed.triangles) {
            if (placed.chartIndex < 0 || std::size_t(placed.chartIndex) >= charts.size()
                || placed.triangleIndex < 0
                || std::size_t(placed.triangleIndex)
                    >= charts[std::size_t(placed.chartIndex)].triangles.size())
                return {};
            const auto& chart = charts[std::size_t(placed.chartIndex)];
            const bool longX = chart.rectMax.x - chart.rectMin.x
                >= chart.rectMax.y - chart.rectMin.y;
            const auto& dev = chart.triangles[std::size_t(placed.triangleIndex)];
            double mean = 0;
            for (int k = 0; k < 3; ++k) mean += longX ? dev.corner[k].x : dev.corner[k].y;
            auto& stat = stats[{placed.chartIndex, placed.subChartIndex}];
            stat.centroid += mean / 3.0; ++stat.triangles;
        }
        std::map<int, std::vector<std::pair<double, int>>> perChart;
        for (const auto& entry : stats) {
            if (entry.second.triangles <= 0) return {};
            perChart[entry.first.first].push_back(
                {entry.second.centroid / entry.second.triangles, entry.first.second});
        }
        for (auto& entry : perChart) {
            auto& subs = entry.second;
            std::sort(subs.begin(), subs.end(), [](const std::pair<double, int>& a,
                                                   const std::pair<double, int>& b) {
                return a.first != b.first ? a.first < b.first : a.second < b.second;
            });
            for (std::size_t rank = 0; rank < subs.size(); ++rank)
                ranks[{entry.first, subs[rank].second}] = std::uint64_t(rank);
        }
        return ranks;
    } catch (...) { return {}; }
}

enum class ChartRejection { Ok, Unsupported, Rejected };

// Per-member kernel chart generation matching the curved::unwrap product
// route exactly (same dispatch, branch normalization, tear checks, developed
// conversion and adjacency), but appending into ONE shared packer input so
// the asset-wide pack runs exactly once. No pack call happens here.
inline ChartRejection AppendMemberCharts(
    const std::vector<shapeyard::uv::Triangle>& triangles,
    const std::vector<shapeyard::uv::curved::FaceInput>& faces,
    const shapeyard::uv::Settings& settings,
    int& nextFaceId,
    int flatBase,
    curveduv::PackerInput& packer,
    std::vector<std::vector<int>>& originalIndices,
    std::vector<std::size_t>& chartFaces,
    std::string& reason) noexcept {
    reason.clear();
    try {
        using namespace shapeyard::uv;
        using namespace shapeyard::uv::detail;
        namespace curved = shapeyard::uv::curved;
        if (!settings.valid() || settings.gutterPixels > 8 || triangles.empty()
            || triangles.size() > 4096 || faces.empty() || faces.size() > 256) {
            reason = "invalid bounded unwrap input/settings"; return ChartRejection::Rejected;
        }
        int next = 0;
        for (const auto& f : faces) {
            if (f.firstTriangle != next || f.triangleCount <= 0
                || f.triangleCount > int(triangles.size()) - next) {
                reason = "provenance triangle partition mismatch"; return ChartRejection::Rejected;
            }
            next += f.triangleCount;
        }
        if (next != int(triangles.size())) {
            reason = "incomplete provenance partition"; return ChartRejection::Rejected;
        }
        curved::impl::Edges edges;
        if (!curved::impl::edgesFor(triangles, edges)) {
            reason = "nonmanifold or degenerate shared edge"; return ChartRejection::Rejected;
        }
        std::vector<int> owner(triangles.size(), -1), chartOf(triangles.size(), -1);
        std::vector<int> faceIds(faces.size(), -1);
        std::vector<UV> periods(faces.size(), UV{0, 0});
        std::vector<std::array<UV, 3>> raw(triangles.size());
        for (int fi = 0; fi < int(faces.size()); ++fi) {
            const auto& f = faces[fi];
            // surfaceType 3 (other) has only the fallback kernel: the member
            // is inadmissible (verified-chart receipts never cover it).
            if (f.surfaceType == 3) {
                reason = "unsupported analytic surface"; return ChartRejection::Unsupported;
            }
            std::vector<Triangle> input(triangles.begin() + f.firstTriangle,
                                        triangles.begin() + f.firstTriangle + f.triangleCount);
            Atlas atlas; bool accepted = false;
            const int expected = f.surfaceType == 0 ? 9 : f.surfaceType == 1 ? 10
                : f.surfaceType == 2 ? 11 : -1;
            if (expected < 0 || f.params.size() != std::size_t(expected)) {
                reason = "invalid analytic parameters"; return ChartRejection::Rejected;
            }
            bool finite = true;
            for (double p : f.params) if (!std::isfinite(p)) finite = false;
            if (!finite) { reason = "nonfinite analytic parameter"; return ChartRejection::Rejected; }
            curveduv::KernelTag kernel = curveduv::KernelTag::Planar;
            if (f.surfaceType == 1 || f.surfaceType == 2) {
                kernel = f.surfaceType == 1 ? curveduv::KernelTag::Cylinder
                                            : curveduv::KernelTag::Torus;
                if (std::isfinite(f.toleranceMM) && f.toleranceMM > 0) {
                    shapeyard::uv::Point center{f.params[0], f.params[1], f.params[2]};
                    shapeyard::uv::Point axis{f.params[3], f.params[4], f.params[5]};
                    const double length = magnitude(axis), xLength = magnitude(f.xDirection);
                    if (length > 1.e-12 && std::isfinite(xLength) && std::abs(xLength - 1) < 1.e-8
                        && std::abs(dot(axis, f.xDirection)) < length * 1.e-8) {
                        shapeyard::uv::Point u, v;
                        prototype::curved_detail::frameFor(divide(axis, length), u, v);
                        const double seam = std::atan2(dot(f.xDirection, v), dot(f.xDirection, u));
                        if (f.surfaceType == 1) {
                            prototype::CylinderBand surface;
                            surface.center = center; surface.axis = axis;
                            surface.radius = f.params[9];
                            surface.seamAngle = seam;
                            surface.fitToleranceModelUnits = f.toleranceMM;
                            const auto value = prototype::unwrapCylindricalBand(input, surface, settings);
                            accepted = value.ok(); atlas = value.atlas;
                            periods[fi] = {prototype::curved_detail::kTwoPi * surface.radius, 0};
                        } else {
                            prototype::TorusBand surface;
                            surface.center = center; surface.axis = axis;
                            surface.majorRadius = f.params[9]; surface.minorRadius = f.params[10];
                            surface.seamAngleU = seam;
                            surface.fitToleranceModelUnits = f.toleranceMM;
                            // Fixed analytic metric band, exactly as unwrap.
                            const double R = surface.majorRadius, r = surface.minorRadius;
                            prototype::DistortionBudgets budget{R / (R + r) - 1.e-10,
                                                                R / (R - r) + 1.e-10, 1.0};
                            const auto value = prototype::unwrapToroidalBand(input, surface, budget, settings);
                            accepted = value.ok(); atlas = value.atlas;
                            periods[fi] = {prototype::curved_detail::kTwoPi * R,
                                           prototype::curved_detail::kTwoPi * r};
                        }
                    }
                }
                // A verified-chart member whose curved kernel falls back at
                // atlas time contradicts its receipt quality: inadmissible.
                if (!accepted) {
                    reason = "curved kernel rejected a verified-chart member face";
                    return ChartRejection::Unsupported;
                }
            }
            if (f.surfaceType == 0) {
                if (!generate(input, settings, atlas)) {
                    reason = "per-face planar chart rejected geometry";
                    return ChartRejection::Rejected;
                }
            }
            if (accepted) {
                // Whole-turn branch choice identical to unwrap: kernels anchor
                // each triangle at corner zero; choose the whole-turn branch
                // from the minimum angle without changing any edge vector.
                shapeyard::uv::Point center{f.params[0], f.params[1], f.params[2]};
                shapeyard::uv::Point axis{f.params[3], f.params[4], f.params[5]}, u, v;
                axis = divide(axis, magnitude(axis));
                prototype::curved_detail::frameFor(axis, u, v);
                const auto delta = sub(input[0].points[0], center);
                const double height = dot(delta, axis);
                const auto radial = sub(delta, shapeyard::uv::Point{
                    axis[0] * height, axis[1] * height, axis[2] * height});
                const double seam = std::atan2(dot(f.xDirection, v), dot(f.xDirection, u));
                double angle[2] = {std::atan2(dot(radial, v), dot(radial, u)) - seam,
                    f.surfaceType == 2 ? std::atan2(height, magnitude(radial) - f.params[9]) : 0};
                for (int d = 0; d < 2; ++d) if (periods[fi][d] > 0) {
                    const double turn = prototype::curved_detail::kTwoPi;
                    angle[d] = std::fmod(angle[d], turn); if (angle[d] < 0) angle[d] += turn;
                    const double period = periods[fi][d];
                    const double origin = atlas.corners[0][0][d] / atlas.unitsPerMillimeter
                        - angle[d] * period / turn;
                    for (auto& corners : atlas.corners) {
                        double minimum = INFINITY;
                        for (const auto& uv : corners) minimum = std::min(minimum, uv[d] / atlas.unitsPerMillimeter - origin);
                        const double shift = std::floor((minimum + period * 1.e-12) / period) * period;
                        for (auto& uv : corners) uv[d] -= shift * atlas.unitsPerMillimeter;
                    }
                }
                // Branch tears refuse before strip splitting can disguise them.
                for (const auto& item : edges) {
                    const auto& uses = item.second; if (uses.size() != 2) continue;
                    const auto a = uses[0], b = uses[1];
                    const int ta = a.triangle - f.firstTriangle, tb = b.triangle - f.firstTriangle;
                    if (ta < 0 || tb < 0 || ta >= f.triangleCount || tb >= f.triangleCount) continue;
                    auto x = atlas.corners[ta], y = atlas.corners[tb];
                    for (auto* list : {&x, &y}) for (auto& uv : *list) for (double& d : uv) d /= atlas.unitsPerMillimeter;
                    bool same = true;
                    for (int d = 0; d < 2; ++d)
                        same = same && std::abs(x[a.a][d] - y[b.a][d]) < 1.e-8
                            && std::abs(x[a.b][d] - y[b.b][d]) < 1.e-8;
                    if (!same && !curved::impl::periodicSeam(x[a.a], x[a.b], y[b.a], y[b.b],
                                                             periods[fi][0], periods[fi][1])) {
                        reason = "kernel shared-edge tear"; return ChartRejection::Rejected;
                    }
                }
            }
            std::vector<std::vector<int>> indices;
            const int faceId = nextFaceId++;
            faceIds[std::size_t(fi)] = faceId;
            auto charts = curved::impl::developed(atlas, faceId, kernel, indices, f.firstTriangle);
            for (int c = 0; c < int(charts.size()); ++c) {
                for (int t = 0; t < int(indices[c].size()); ++t) {
                    const int original = indices[c][t];
                    owner[original] = fi;
                    chartOf[original] = int(packer.charts.size());
                    for (int k = 0; k < 3; ++k)
                        raw[original][k] = {charts[c].triangles[t].corner[k].x,
                                            charts[c].triangles[t].corner[k].y};
                }
                packer.charts.push_back(std::move(charts[c]));
                originalIndices.push_back(std::move(indices[c]));
                chartFaces.push_back(std::size_t(fi));
            }
        }
        // Global flattened triangle indices in chart order across the whole
        // merged packer input; adjacency stays face-internal like unwrap.
        std::vector<int> flat(triangles.size(), -1);
        int cursor = flatBase;
        for (const auto& indices : originalIndices)
            for (int original : indices) flat[std::size_t(original)] = cursor++;
        for (const auto& item : edges) {
            const auto& uses = item.second; if (uses.size() != 2) continue;
            const auto a = uses[0], b = uses[1];
            if (chartOf[a.triangle] != chartOf[b.triangle]) continue;
            const int fi = owner[a.triangle];
            const bool seam = curved::impl::periodicSeam(
                raw[a.triangle][a.a], raw[a.triangle][a.b],
                raw[b.triangle][b.a], raw[b.triangle][b.b],
                periods[fi][0], periods[fi][1]);
            packer.adjacency.push_back({faceIds[std::size_t(fi)], flat[std::size_t(a.triangle)],
                a.a, a.b, flat[std::size_t(b.triangle)], b.a, b.b, seam});
        }
        return ChartRejection::Ok;
    } catch (...) { reason = "asset atlas chart exception"; return ChartRejection::Rejected; }
}

} // namespace detail

// Admission classification for one requested owner. Captured means the owner
// resolves to exactly one free simple shape in this document carrying a
// valid verified-chart Planar/Cylindrical/Toroidal finishing receipt bound
// to the same OwnerKey.
inline Status ClassifyMember(const Handle(TDocStd_Document)& document,
                             const OwnerKey& key) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Valid(key)) return Status::Malformed;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID))
            return Status::Malformed;
        if (!(key.document == documentID)) return Status::ForeignMember;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return Status::Malformed;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        int matches = 0; TDF_Label found;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            UUID entity{}, definition{};
            const TDF_Label candidate = roots.Value(index);
            if (!XCAFDoc_ShapeTool::IsSimpleShape(candidate)
                || !retained_solid::ReadUUID(candidate,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(candidate,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || !(entity == key.entity) || !(definition == key.definition)) continue;
            if (++matches > 1) return Status::ForeignMember;
            found = candidate;
        }
        if (matches == 0) return Status::MissingMember;
        retained_finishing::Record receipt;
        if (!retained_finishing::Read(document, found, receipt) || !receipt.value)
            return Status::UnsupportedSurface;
        retained_finishing::Refusal refusal = retained_finishing::Refusal::None;
        const auto& definition = receipt.value->definition;
        if (!retained_finishing::Valid(definition, refusal)
            || definition.quality != retained_finishing::Quality::VerifiedChart
            || (definition.unwrap != retained_finishing::UnwrapPolicy::Planar
                && definition.unwrap != retained_finishing::UnwrapPolicy::Cylindrical
                && definition.unwrap != retained_finishing::UnwrapPolicy::Toroidal))
            return Status::UnsupportedSurface;
        if (!(definition.owner == key)) return Status::ForeignMember;
        return Status::Captured;
    } catch (...) { return Status::Malformed; }
}

// All-member capture: resolve, fence and read back every admitted member
// before any detached build work. Repeated OwnerKeys are deduplicated with
// admission order preserved. Any failure refuses the whole capture.
inline bool CaptureMembers(const Handle(TDocStd_Document)& document,
                           const std::vector<OwnerKey>& members,
                           Capture& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        std::vector<OwnerKey> keys;
        for (const auto& key : members) {
            bool duplicate = false;
            for (const auto& existing : keys) if (existing == key) { duplicate = true; break; }
            if (!duplicate) keys.push_back(key);
        }
        if (keys.empty() || keys.size() > kMaximumMembers) return false;
        for (const auto& key : keys) {
            if (ClassifyMember(document, key) != Status::Captured) return false;
            MemberCapture entry;
            TDF_Label label; retained_solid::Record retained;
            if (!retained_finishing::producer::detail::Resolve(document, key, label, retained))
                return false;
            retained_finishing::Record receipt;
            if (!retained_finishing::Read(document, label, receipt) || !receipt.value)
                return false;
            if (!retained_finishing::producer::CaptureSource(document, key, entry.capture))
                return false;
            entry.finishing = receipt.value->definition;
            entry.ownerLabel = label;
            entry.slot.owner = key;
            entry.slot.finishing = receipt.value->definition.finishing;
            entry.slot.source = entry.capture.source;
            entry.painted = detail::Painted(document, label);
            output.members.push_back(std::move(entry));
        }
        return true;
    } catch (...) { output = {}; return false; }
}

// Commit-time re-observation: rebuild the observed membership (current
// receipt UUID and SourceRevision per fenced member) without any mutation.
inline bool ObserveMembers(const Handle(TDocStd_Document)& document,
                           const std::vector<Member>& fenced,
                           std::vector<Member>& observed) noexcept {
    observed.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        for (const auto& member : fenced) {
            Member value;
            retained_finishing::Record receipt;
            TDF_Label label; retained_solid::Record retained;
            if (!retained_finishing::producer::detail::Resolve(
                    document, member.owner, label, retained)
                || !retained_finishing::Read(document, label, receipt) || !receipt.value)
                return false;
            value.member = member.member;
            value.owner = member.owner;
            value.finishing = receipt.value->definition.finishing;
            if (!retained_finishing::producer::detail::Source(
                    document, member.owner, value.source))
                return false;
            observed.push_back(value);
        }
        return true;
    } catch (...) { observed.clear(); return false; }
}

//! One asset-wide detached build over exact final triangles and developed
//! face inputs. Authority/resource fields remain the captured saved values;
//! this function cannot observe or mutate OCAF state.
inline Status BuildFinalAtlas(const Key& atlas,
                              const std::vector<FinalMemberInput>& inputs,
                              const Settings& settings,
                              Definition& candidate,
                              std::vector<MemberUVAssignment>& assignments,
                              std::string& diagnosis,
                              LayoutEvidence* layout = nullptr,
                              FinalAtlasAccounting* accounting = nullptr
#ifdef DEBUG
                              , const DebugLayoutObserver* observer = nullptr
#endif
                              ) noexcept {
    candidate = {}; assignments.clear(); diagnosis.clear();
    if (layout) *layout = {};
    if (accounting) {
        accounting->layoutStorage.reset();
        accounting->resultStorage.reset();
        accounting->jobUnits = 0;
#ifdef DEBUG
        accounting->memberCount = 0;
        accounting->triangleCount = 0;
        accounting->cornerCount = 0;
        accounting->layoutReservationAttempts = 0;
        accounting->packerEntries = 0;
#endif
    }
    struct AccountingReset final {
        FinalAtlasAccounting* value = nullptr;
        bool committed = false;
        ~AccountingReset() {
            if (value && !committed) {
                value->resultStorage.reset();
                value->layoutStorage.reset();
            }
        }
    } accountingReset{accounting};
    try {
        if (!retained_recipe::Nonzero(atlas.document)
            || !retained_recipe::Nonzero(atlas.atlas)
            || settings.resolutionTexels < 256
            || settings.resolutionTexels > 4096
            || (settings.resolutionTexels & (settings.resolutionTexels - 1)) != 0
            || settings.gutterTexels < 1 || settings.gutterTexels > 8
            || inputs.empty()) return Status::Malformed;
        if (inputs.size() > kMaximumMembers) return Status::OverBudget;

        using namespace decal_layer::bake::accounting;
        const auto checkedProduct = [&](std::size_t first,
                std::size_t second, std::size_t third,
                std::size_t& product, FailureSite site) {
            product = 0;
            if (accounting)
                return accounting->operation.CheckedProduct(
                    first, second, third, product, site);
            if ((first != 0
                    && second > std::numeric_limits<std::size_t>::max() / first)
                || (first * second != 0
                    && third > std::numeric_limits<std::size_t>::max()
                        / (first * second))) return false;
            product = first * second * third;
            return true;
        };
        const auto checkedAdd = [&](std::size_t value,
                std::size_t increment, std::size_t& result) {
            if (increment > std::numeric_limits<std::size_t>::max() - value) {
                if (accounting)
                    accounting->operation.Refuse(FailureSite::CheckedProduct);
                return false;
            }
            result = value + increment;
            return true;
        };

        std::size_t totalTriangles = 0;
        std::size_t totalFaces = 0;
        std::size_t totalResources = 0;
        std::size_t totalCorners = 0;
        for (const auto& input : inputs) {
            if (!retained_recipe::Valid(input.savedMember.owner)
                || input.triangles.empty() || input.faces.empty()
                || input.authority.resources.empty()) return Status::Malformed;
            for (const auto& face : input.faces)
                if (face.surfaceType == 3) return Status::UnsupportedSurface;
            std::size_t memberCorners = 0;
            if (!checkedProduct(input.triangles.size(), 3U, 1U,
                    memberCorners, FailureSite::CheckedProduct)
                || !checkedAdd(totalTriangles, input.triangles.size(),
                    totalTriangles)
                || !checkedAdd(totalFaces, input.faces.size(), totalFaces)
                || !checkedAdd(totalResources,
                    input.authority.resources.size(), totalResources)
                || !checkedAdd(totalCorners, memberCorners, totalCorners))
                return Status::OverBudget;
        }
        if (totalCorners > kMaximumFinalCorners
            || totalResources > kMaximumMaterials)
            return Status::OverBudget;
        if (accounting && (!accounting->operation.valid() || !layout
                || accounting->roleCount == 0
                || accounting->sourceCount == 0)) return Status::OverBudget;

        Ticket chartScratch;
        if (accounting) {
            std::size_t triangleScratch = 0, faceScratch = 0;
            std::size_t memberScratch = 0, chartScratchBytes = 0;
            if (!checkedProduct(totalTriangles,
                    sizeof(shapeyard::uv::Triangle)
                        + sizeof(std::array<std::array<double, 2>, 3>)
                        + 5U * sizeof(int),
                    1U, triangleScratch, FailureSite::ExportMemberStorage)
                || !checkedProduct(totalFaces,
                    sizeof(shapeyard::uv::curved::FaceInput)
                        + sizeof(std::array<double, 2>) + 2U * sizeof(int),
                    1U, faceScratch, FailureSite::ExportMemberStorage)
                || !checkedProduct(inputs.size(),
                    sizeof(std::vector<std::vector<int>>)
                        + sizeof(std::size_t),
                    1U, memberScratch, FailureSite::ExportMemberStorage)
                || !checkedAdd(triangleScratch, faceScratch,
                    chartScratchBytes)
                || !checkedAdd(chartScratchBytes, memberScratch,
                    chartScratchBytes)
                || !checkedAdd(chartScratchBytes,
                    64U * (sizeof(curveduv::ChartInput)
                        + sizeof(std::vector<int>)
                        + sizeof(std::pair<std::size_t, std::uint64_t>)
                        + sizeof(std::size_t)), chartScratchBytes)
                || !chartScratch.acquire(accounting->operation,
                    StorageDimension::PrivateStorage, chartScratchBytes,
                    Retention::Scratch, FailureSite::ExportMemberStorage)
                || !accounting->operation.EnterAllocation(
                    FailureSite::ExportMemberStorage))
                return Status::OverBudget;

            std::size_t pixels = 0, rolePixels = 0, gutterUnits = 0;
            std::size_t sourceUnits = 0, jobUnits = 0;
            if (!checkedProduct(std::size_t(settings.resolutionTexels),
                    std::size_t(settings.resolutionTexels), 1U, pixels,
                    FailureSite::ExportCanonicalStorage)
                || !checkedProduct(pixels, accounting->roleCount, 1U,
                    rolePixels, FailureSite::ExportCanonicalStorage)
                || !checkedProduct(totalCorners,
                    std::size_t(settings.gutterTexels), 1U, gutterUnits,
                    FailureSite::ExportCanonicalStorage)
                || !checkedAdd(accounting->sourceCount,
                    accounting->maskCount, sourceUnits)
                || !checkedAdd(rolePixels, gutterUnits, jobUnits)
                || !checkedAdd(jobUnits, totalTriangles, jobUnits)
                || !checkedAdd(jobUnits, sourceUnits, jobUnits)
                || (accounting->sealAtlasJob
                    && !accounting->operation.SealJob(Job::AtlasRoles,
                        jobUnits, FailureSite::ExportCanonicalStorage)))
                return Status::OverBudget;
            accounting->jobUnits = jobUnits;
#ifdef DEBUG
            accounting->memberCount = inputs.size();
            accounting->triangleCount = totalTriangles;
            accounting->cornerCount = totalCorners;
#endif
        }

        curveduv::PackerInput packer;
        packer.settings = {settings.resolutionTexels, settings.gutterTexels};
        const shapeyard::uv::Settings kernelSettings{
            settings.resolutionTexels, settings.gutterTexels};
        const std::size_t count = inputs.size();
        std::vector<std::vector<std::vector<int>>> memberCharts(count);
        std::vector<std::pair<std::size_t, std::uint64_t>> chartMembers;
        std::vector<std::size_t> chartFaces;
        std::vector<std::size_t> memberTriangles(count, 0);
        int nextFaceId = 0;
        for (std::size_t index = 0; index < count; ++index) {
            const auto& input = inputs[index];
            memberTriangles[index] = input.triangles.size();
            int flatBase = 0;
            for (const auto& chart : packer.charts)
                flatBase += int(chart.triangles.size());
            std::string reason;
            const auto rejection = detail::AppendMemberCharts(
                input.triangles, input.faces, kernelSettings, nextFaceId,
                flatBase, packer, memberCharts[index], chartFaces, reason);
            if (rejection != detail::ChartRejection::Ok) {
                diagnosis = reason;
                return rejection == detail::ChartRejection::Unsupported
                    ? Status::UnsupportedSurface : Status::Refused;
            }
            for (std::size_t ordinal = 0;
                 ordinal < memberCharts[index].size(); ++ordinal)
                chartMembers.push_back({index, std::uint64_t(ordinal)});
            if (packer.charts.size() > 64) return Status::OverBudget;
        }

        Ticket packerScratch;
        if (accounting) {
            std::size_t coverageBytes = 0, triangleBytes = 0;
            std::size_t chartBytes = 0, packerBytes = 0;
            if (!checkedProduct(std::size_t(settings.resolutionTexels),
                    std::size_t(settings.resolutionTexels), 2U,
                    coverageBytes, FailureSite::ExportObservedStorage)
                || !checkedProduct(totalTriangles,
                    sizeof(curveduv::TriangleUV)
                        + sizeof(curveduv::detail::PlacedTriangle)
                        + 5U * sizeof(int) + 2U * sizeof(double),
                    1U, triangleBytes, FailureSite::ExportObservedStorage)
                || !checkedProduct(packer.charts.size(),
                    65U * sizeof(double) + sizeof(std::vector<int>)
                        + sizeof(std::vector<curveduv::detail::PlacedTriangle>),
                    1U, chartBytes, FailureSite::ExportObservedStorage)
                || !checkedAdd(coverageBytes, triangleBytes, packerBytes)
                || !checkedAdd(packerBytes, chartBytes, packerBytes))
                return Status::OverBudget;
#ifdef DEBUG
            ++accounting->layoutReservationAttempts;
            if (accounting->denyLayoutReservation) {
                (void)accounting->operation.Reserve(
                    StorageDimension::PrivateStorage,
                    std::numeric_limits<std::size_t>::max(),
                    Retention::Scratch, FailureSite::ExportObservedStorage);
                return Status::OverBudget;
            }
#endif
            std::size_t layoutBytes = 0, part = 0;
            if (!checkedProduct(totalTriangles,
                    sizeof(curveduv::TriangleUV)
                        + sizeof(curveduv::ChartTriangle),
                    1U, layoutBytes, FailureSite::ExportAtlasStorage)
                || !checkedProduct(packer.charts.size(),
                    sizeof(curveduv::ChartInput)
                        + sizeof(std::pair<std::size_t, std::uint64_t>)
                        + sizeof(std::size_t),
                    1U, part, FailureSite::ExportAtlasStorage)
                || !checkedAdd(layoutBytes, part, layoutBytes)
                || !packerScratch.acquire(accounting->operation,
                    StorageDimension::PrivateStorage, packerBytes,
                    Retention::Scratch, FailureSite::ExportObservedStorage)
                || !accounting->layoutStorage.acquire(accounting->operation,
                    StorageDimension::PrivateStorage, layoutBytes,
                    Retention::Retained, FailureSite::ExportAtlasStorage)
                || !accounting->operation.EnterAllocation(
                    FailureSite::ExportObservedStorage))
                return Status::OverBudget;
#ifdef DEBUG
            ++accounting->packerEntries;
#endif
        }
        const auto packed = curveduv::pack(packer);
        if (!packed.ok) { diagnosis = packed.reason; return Status::Refused; }
        if (packed.summary.subChartCount <= 0
            || packed.summary.subChartCount > int(kMaximumCharts))
            return Status::OverBudget;

        if (accounting) {
            std::size_t resultBytes = 0, part = 0;
            if (!checkedProduct(totalResources,
                    sizeof(retained_finishing::MaterialResource), 1U,
                    resultBytes, FailureSite::ExportBakeStorage)
                || !checkedProduct(count,
                    sizeof(Member) + sizeof(MemberUVAssignment), 1U,
                    part, FailureSite::ExportBakeStorage)
                || !checkedAdd(resultBytes, part, resultBytes)
                || !checkedProduct(std::size_t(packed.summary.subChartCount),
                    sizeof(Chart), 1U, part,
                    FailureSite::ExportBakeStorage)
                || !checkedAdd(resultBytes, part, resultBytes)
                || !checkedProduct(totalCorners,
                    sizeof(std::array<double, 2>), 1U, part,
                    FailureSite::ExportBakeStorage)
                || !checkedAdd(resultBytes, part, resultBytes)
                || !accounting->resultStorage.acquire(accounting->operation,
                    StorageDimension::PrivateStorage, resultBytes,
                    Retention::Retained, FailureSite::ExportBakeStorage)
                || !accounting->operation.EnterAllocation(
                    FailureSite::ExportBakeStorage))
                return Status::OverBudget;
        }

        std::vector<retained_finishing::MaterialResource> resources;
        std::vector<UUID> memberMaterial(count);
        for (std::size_t index = 0; index < count; ++index) {
            const auto& captured = inputs[index].authority.resources;
            memberMaterial[index] = captured.front().identity;
            for (const auto& resource : captured) {
                bool present = false;
                for (const auto& existing : resources)
                    if (existing.identity == resource.identity) {
                        present = true; break;
                    }
                if (!present) resources.push_back(resource);
            }
            if (resources.size() > kMaximumMaterials)
                return Status::OverBudget;
        }

        candidate.key = atlas;
        candidate.resolutionTexels = settings.resolutionTexels;
        candidate.gutterTexels = settings.gutterTexels;
        candidate.globalTexelsPerMM = packed.summary.globalTexelsPerMM;
        candidate.resources = resources;
        candidate.members.resize(count);
        for (std::size_t index = 0; index < count; ++index) {
            candidate.members[index].member = detail::MemberUUID(
                atlas.atlas, inputs[index].savedMember.owner);
            if (!retained_recipe::Nonzero(candidate.members[index].member))
                return Status::Malformed;
            candidate.members[index].owner = inputs[index].savedMember.owner;
            candidate.members[index].finishing =
                inputs[index].savedMember.finishing;
            candidate.members[index].source = inputs[index].savedMember.source;
        }

        assignments.resize(count);
        for (std::size_t index = 0; index < count; ++index) {
            assignments[index].member = candidate.members[index].member;
            assignments[index].triangleCount =
                std::uint32_t(memberTriangles[index]);
            std::size_t memberCorners = 0;
            if (!checkedProduct(memberTriangles[index], 3U, 1U,
                    memberCorners, FailureSite::CheckedProduct))
                return Status::OverBudget;
            assignments[index].corners.resize(memberCorners);
        }
        struct ChartAccum {
            std::uint32_t triangles = 0;
            double area = 0;
            double loU = INFINITY, loV = INFINITY;
            double hiU = -INFINITY, hiV = -INFINITY;
        };
        std::map<std::pair<int, int>, ChartAccum> groups;
#ifdef DEBUG
        std::uint64_t observationOrdinal = 0;
#endif
        for (const auto& placed : packed.triangles) {
            if (placed.chartIndex < 0
                || std::size_t(placed.chartIndex) >= chartMembers.size()
                || placed.triangleIndex < 0) return Status::Malformed;
            const auto& [memberIndex, ordinal] =
                chartMembers[std::size_t(placed.chartIndex)];
            const auto& memberChart = memberCharts[memberIndex][ordinal];
            if (std::size_t(placed.triangleIndex) >= memberChart.size())
                return Status::Malformed;
            const int original = memberChart[std::size_t(placed.triangleIndex)];
            if (original < 0
                || std::size_t(original) >= memberTriangles[memberIndex])
                return Status::Malformed;
            auto& slot = assignments[memberIndex].corners[
                3 * std::size_t(original)];
            std::array<double, 2>* target = &slot;
            for (int corner = 0; corner < 3; ++corner)
                target[corner] = {
                    placed.uv[corner].x, placed.uv[corner].y};
#ifdef DEBUG
            if (observer && observer->lineage)
                observer->lineage(observer->context, observationOrdinal,
                    std::uint64_t(memberIndex), ordinal,
                    std::int32_t(placed.chartIndex),
                    std::int32_t(placed.triangleIndex),
                    std::int32_t(placed.subChartIndex),
                    std::int32_t(original));
            ++observationOrdinal;
#endif
            auto& accum = groups[{placed.chartIndex, placed.subChartIndex}];
            ++accum.triangles;
            if (std::size_t(placed.triangleIndex)
                >= packer.charts[std::size_t(placed.chartIndex)]
                    .triangles.size()) return Status::Malformed;
            const auto& developed =
                packer.charts[std::size_t(placed.chartIndex)]
                    .triangles[std::size_t(placed.triangleIndex)];
            const double cross =
                (developed.corner[1].x - developed.corner[0].x)
                    * (developed.corner[2].y - developed.corner[0].y)
                - (developed.corner[1].y - developed.corner[0].y)
                    * (developed.corner[2].x - developed.corner[0].x);
            accum.area += 0.5 * std::fabs(cross);
            for (int corner = 0; corner < 3; ++corner) {
                accum.loU = std::min(accum.loU, placed.uv[corner].x);
                accum.loV = std::min(accum.loV, placed.uv[corner].y);
                accum.hiU = std::max(accum.hiU, placed.uv[corner].x);
                accum.hiV = std::max(accum.hiV, placed.uv[corner].y);
            }
        }
        if (groups.empty() || groups.size() > kMaximumCharts)
            return Status::OverBudget;
        const auto ranks = detail::SegmentRanks(packer.charts, packed);
        if (ranks.size() != groups.size()) return Status::Malformed;
        for (const auto& [group, accum] : groups) {
            const auto& [memberIndex, ordinal] =
                chartMembers[std::size_t(group.first)];
            const auto rank = ranks.find(group);
            if (rank == ranks.end()) return Status::Malformed;
            Chart chart;
            chart.member = candidate.members[memberIndex].member;
            chart.chart = detail::ChartUUID(
                atlas.atlas, chart.member, ordinal, rank->second);
            if (!retained_recipe::Nonzero(chart.chart))
                return Status::Malformed;
            chart.material = memberMaterial[memberIndex];
            chart.triangleCount = accum.triangles;
            chart.developedAreaMM2 = accum.area;
            chart.rectUV = {
                accum.loU, accum.loV, accum.hiU, accum.hiV};
            candidate.charts.push_back(chart);
#ifdef DEBUG
            if (observer && observer->island)
                observer->island(observer->context,
                    std::int32_t(group.first), std::int32_t(group.second),
                    ordinal, rank->second,
                    std::uint64_t(candidate.charts.size() - 1), chart.chart);
#endif
        }
        if (!BindLayoutProof(candidate, assignments))
            return Status::Malformed;
        Refusal refusal = Refusal::None;
        if (!Valid(candidate, refusal)) return Status::Malformed;
        Ticket encodeScratch;
        if (accounting
            && (!encodeScratch.acquire(accounting->operation,
                    StorageDimension::PrivateStorage, kMaximumBytes,
                    Retention::Scratch, FailureSite::ExportCanonicalStorage)
                || !accounting->operation.EnterAllocation(
                    FailureSite::ExportCanonicalStorage)))
            return Status::OverBudget;
        std::vector<std::uint8_t> encoded;
        if (!Encode(candidate, encoded)) return Status::OverBudget;
        if (layout) {
            layout->packed = packed;
            layout->charts = packer.charts;
            layout->chartMembers = chartMembers;
            layout->chartFaces = chartFaces;
        }
#ifdef DEBUG
        if (observer && observer->finish)
            observer->finish(observer->context, memberCharts,
                packer.adjacency);
#endif
        accountingReset.committed = true;
        return Status::Built;
    } catch (...) {
        candidate = {}; assignments.clear(); diagnosis.clear();
        if (layout) *layout = {};
        return Status::Malformed;
    }
}

// One asset-wide build. Per member the row-268 tessellation/face-input route
// and the existing curved kernels produce developed charts; curveduv::pack
// runs exactly once over the merged chart set; the single
// summary.globalTexelsPerMM is recorded on the candidate.
inline Status BuildAtlas(const Handle(TDocStd_Document)& document,
                         const Key& atlas, const Capture& capture,
                         const Settings& settings, Definition& candidate,
                         std::vector<MemberUVAssignment>& assignments,
                         std::string& diagnosis,
                         LayoutEvidence* layout = nullptr,
                         PaintedAdmission paintedAdmission = PaintedAdmission::Reject) noexcept {
    candidate = {}; assignments.clear(); diagnosis.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Nonzero(atlas.document)
            || !retained_recipe::Nonzero(atlas.atlas)) return Status::Malformed;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID))
            return Status::Malformed;
        if (!(atlas.document == documentID)) return Status::OwnerMismatch;
        if (settings.resolutionTexels < 256 || settings.resolutionTexels > 4096
            || (settings.resolutionTexels & (settings.resolutionTexels - 1)) != 0
            || settings.gutterTexels < 1 || settings.gutterTexels > 8)
            return Status::Malformed;
        const std::size_t count = capture.members.size();
        if (count == 0) return Status::Malformed;
        if (count > kMaximumMembers) return Status::OverBudget;

        // Admission re-observation: every member must still resolve, still
        // carry its fenced receipt and revision, and must not be fenced by a
        // different atlas record.
        std::vector<persistence::Record> records;
        if (!persistence::ReadAll(document, records)) return Status::Malformed;
        std::vector<retained_solid::Record> retained(count);
        for (std::size_t index = 0; index < count; ++index) {
            const auto& entry = capture.members[index];
            const Status admission = ClassifyMember(document, entry.slot.owner);
            if (admission != Status::Captured) return admission;
            for (const auto& record : records)
                if (!(record.value->definition.key.atlas == atlas.atlas))
                    for (const auto& member : record.value->definition.members)
                        if (member.owner == entry.slot.owner) return Status::ForeignMember;
            if (entry.ownerLabel.IsNull()
                || entry.ownerLabel.Data() != document->GetData())
                return Status::MissingMember;
            retained_finishing::Record receipt;
            if (!retained_finishing::Read(document, entry.ownerLabel, receipt)
                || !receipt.value) return Status::UnsupportedSurface;
            if (!(receipt.value->definition.finishing == entry.slot.finishing))
                return Status::StaleSource;
            if (!(receipt.value->definition.owner == entry.slot.owner))
                return Status::ForeignMember;
            retained_finishing::SourceRevision observed;
            if (!retained_finishing::producer::detail::Source(
                    document, entry.slot.owner, observed, nullptr, &retained[index]))
                return Status::MissingMember;
            if (!(observed == entry.slot.source)) return Status::StaleSource;
        }
        for (const auto& entry : capture.members)
            if (entry.painted && paintedAdmission == PaintedAdmission::Reject) {
                diagnosis = "asset atlas member "
                    + retained_solid::UUIDText(entry.slot.owner.entity)
                    + " carries image-backed painted content; a texture rebake proof is required";
                return Status::PaintedRebakeRequired;
            }

        // Ordinary saved-state admission ends above. From this point both the
        // document route and private export route use the exact same detached
        // chart/pack/assignment implementation.
        std::vector<FinalMemberInput> finalInputs(count);
        for (std::size_t index = 0; index < count; ++index) {
            std::atomic_bool cancelled{false};
            meshcopy::CurrentTessellationCopy copy;
            if (meshcopy::PrepareCurrentTessellationCopy(
                    retained[index].current, copy, cancelled)
                != meshcopy::PreparationResult::Ready)
                return Status::UnsupportedSurface;
            finalInputs[index].savedMember = capture.members[index].slot;
            finalInputs[index].authority = capture.members[index].capture;
            if (!retained_finishing::producer::detail::Triangles(
                    copy, finalInputs[index].triangles,
                    finalInputs[index].faces))
                return Status::Malformed;
        }
        return BuildFinalAtlas(atlas, finalInputs, settings, candidate,
                               assignments, diagnosis, layout);

#if 0
        // Retained below temporarily as a byte-for-byte review aid while the
        // shared detached implementation above is exercised by the bench.
        // It is not compiled and cannot become a second packing route.
        // Merged chart generation; one packer input for the whole asset.
        curveduv::PackerInput packer;
        packer.settings = {settings.resolutionTexels, settings.gutterTexels};
        const shapeyard::uv::Settings kernelSettings{
            settings.resolutionTexels, settings.gutterTexels};
        std::vector<std::vector<std::vector<int>>> memberCharts(count);
        std::vector<std::pair<std::size_t, std::uint64_t>> chartMembers;
        std::vector<std::size_t> memberTriangles(count, 0);
        std::uint64_t corners = 0;
        int nextFaceId = 0;
        for (std::size_t index = 0; index < count; ++index) {
            std::atomic_bool cancelled{false};
            meshcopy::CurrentTessellationCopy copy;
            if (meshcopy::PrepareCurrentTessellationCopy(
                    retained[index].current, copy, cancelled)
                != meshcopy::PreparationResult::Ready) return Status::UnsupportedSurface;
            std::vector<shapeyard::uv::Triangle> triangles;
            std::vector<shapeyard::uv::curved::FaceInput> faces;
            if (!retained_finishing::producer::detail::Triangles(copy, triangles, faces))
                return Status::Malformed;
            for (const auto& face : faces)
                if (face.surfaceType == 3) return Status::UnsupportedSurface;
            corners += 3ULL * triangles.size();
            if (corners > kMaximumFinalCorners) return Status::OverBudget;
            memberTriangles[index] = triangles.size();
            int flatBase = 0;
            for (const auto& chart : packer.charts) flatBase += int(chart.triangles.size());
            std::string reason;
            const auto rejection = detail::AppendMemberCharts(
                triangles, faces, kernelSettings, nextFaceId, flatBase, packer,
                memberCharts[index], reason);
            if (rejection != detail::ChartRejection::Ok) {
                diagnosis = reason;
                return rejection == detail::ChartRejection::Unsupported
                    ? Status::UnsupportedSurface : Status::Refused;
            }
            for (std::size_t ordinal = 0; ordinal < memberCharts[index].size(); ++ordinal)
                chartMembers.push_back({index, std::uint64_t(ordinal)});
            if (packer.charts.size() > 64) return Status::OverBudget;
        }
        const auto packed = curveduv::pack(packer);
        if (!packed.ok) { diagnosis = packed.reason; return Status::Refused; }
        if (packed.summary.subChartCount <= 0
            || packed.summary.subChartCount > int(kMaximumCharts))
            return Status::OverBudget;

        // Identity-deduplicated resources; one material identity per member.
        std::vector<retained_finishing::MaterialResource> resources;
        std::vector<UUID> memberMaterial(count);
        for (std::size_t index = 0; index < count; ++index) {
            const auto& captured = capture.members[index].capture.resources;
            if (captured.empty()) return Status::Malformed;
            memberMaterial[index] = captured.front().identity;
            for (const auto& resource : captured) {
                bool present = false;
                for (const auto& existing : resources)
                    if (existing.identity == resource.identity) { present = true; break; }
                if (!present) resources.push_back(resource);
            }
            if (resources.size() > kMaximumMaterials) return Status::OverBudget;
        }

        candidate.key = atlas;
        candidate.resolutionTexels = settings.resolutionTexels;
        candidate.gutterTexels = settings.gutterTexels;
        candidate.globalTexelsPerMM = packed.summary.globalTexelsPerMM;
        candidate.resources = resources;
        candidate.members.resize(count);
        for (std::size_t index = 0; index < count; ++index) {
            candidate.members[index].member = detail::MemberUUID(
                atlas.atlas, capture.members[index].slot.owner);
            if (!retained_recipe::Nonzero(candidate.members[index].member))
                return Status::Malformed;
            candidate.members[index].owner = capture.members[index].slot.owner;
            candidate.members[index].finishing = capture.members[index].slot.finishing;
            candidate.members[index].source = capture.members[index].slot.source;
        }

        // Map the single pack output back per member: committed UV corners in
        // meshcopy emission order plus one Chart record per (memberChartOrdinal,
        // subChartIndex) present in the output.
        assignments.resize(count);
        for (std::size_t index = 0; index < count; ++index) {
            assignments[index].member = candidate.members[index].member;
            assignments[index].triangleCount = std::uint32_t(memberTriangles[index]);
            assignments[index].corners.resize(3 * memberTriangles[index]);
        }
        struct ChartAccum {
            std::uint32_t triangles = 0;
            double area = 0;
            double loU = INFINITY, loV = INFINITY, hiU = -INFINITY, hiV = -INFINITY;
        };
        std::map<std::pair<int, int>, ChartAccum> groups;
        for (const auto& placed : packed.triangles) {
            if (placed.chartIndex < 0
                || std::size_t(placed.chartIndex) >= chartMembers.size()
                || placed.triangleIndex < 0) return Status::Malformed;
            const auto& [memberIndex, ordinal] = chartMembers[std::size_t(placed.chartIndex)];
            const auto& memberChart = memberCharts[memberIndex][ordinal];
            if (std::size_t(placed.triangleIndex) >= memberChart.size()) return Status::Malformed;
            const int original = memberChart[std::size_t(placed.triangleIndex)];
            if (original < 0 || std::size_t(original) >= memberTriangles[memberIndex])
                return Status::Malformed;
            auto& slot = assignments[memberIndex].corners[3 * std::size_t(original)];
            std::array<double, 2>* target = &slot;
            for (int k = 0; k < 3; ++k) {
                target[k] = {placed.uv[k].x, placed.uv[k].y};
            }
            auto& accum = groups[{placed.chartIndex, placed.subChartIndex}];
            ++accum.triangles;
            if (std::size_t(placed.triangleIndex)
                >= packer.charts[std::size_t(placed.chartIndex)].triangles.size())
                return Status::Malformed;
            const auto& developed = packer.charts[std::size_t(placed.chartIndex)]
                .triangles[std::size_t(placed.triangleIndex)];
            const double cross =
                (developed.corner[1].x - developed.corner[0].x)
                    * (developed.corner[2].y - developed.corner[0].y)
                - (developed.corner[1].y - developed.corner[0].y)
                    * (developed.corner[2].x - developed.corner[0].x);
            accum.area += 0.5 * std::fabs(cross);
            for (int k = 0; k < 3; ++k) {
                accum.loU = std::min(accum.loU, placed.uv[k].x);
                accum.loV = std::min(accum.loV, placed.uv[k].y);
                accum.hiU = std::max(accum.hiU, placed.uv[k].x);
                accum.hiV = std::max(accum.hiV, placed.uv[k].y);
            }
        }
        if (groups.empty() || groups.size() > kMaximumCharts) return Status::OverBudget;
        const auto ranks = detail::SegmentRanks(packer.charts, packed);
        if (ranks.size() != groups.size()) return Status::Malformed;
        for (const auto& [group, accum] : groups) {
            const auto& [memberIndex, ordinal] = chartMembers[std::size_t(group.first)];
            const auto rank = ranks.find(group);
            if (rank == ranks.end()) return Status::Malformed;
            Chart chart;
            chart.member = candidate.members[memberIndex].member;
            chart.chart = detail::ChartUUID(atlas.atlas, chart.member, ordinal, rank->second);
            if (!retained_recipe::Nonzero(chart.chart)) return Status::Malformed;
            chart.material = memberMaterial[memberIndex];
            chart.triangleCount = accum.triangles;
            chart.developedAreaMM2 = accum.area;
            chart.rectUV = {accum.loU, accum.loV, accum.hiU, accum.hiV};
            candidate.charts.push_back(chart);
        }
        if (!BindLayoutProof(candidate, assignments)) return Status::Malformed;
        Refusal refusal = Refusal::None;
        if (!Valid(candidate, refusal)) return Status::Malformed;
        std::vector<std::uint8_t> encoded;
        if (!Encode(candidate, encoded)) return Status::OverBudget;
        if (layout) {
            layout->packed = packed;
            layout->charts = packer.charts;
            layout->chartMembers = chartMembers;
        }
        return Status::Built;
#endif
    } catch (...) { candidate = {}; assignments.clear(); diagnosis.clear(); return Status::Malformed; }
}

} // namespace build
} // namespace core3d::asset_atlas
