#import <Foundation/Foundation.h>

#include "SplineProfileOwner.hxx"
#include "OcctDocument.h"
#include "ReceiptRecord.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BinObjMgt_RRelocationTable.hxx>
#include <BinObjMgt_SRelocationTable.hxx>
#include <TDF_ChildIterator.hxx>
#include <TNaming_NamedShape.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <unordered_map>

namespace core3d::spline_profile {
namespace {
class Reader final {
public:
    explicit Reader(const std::vector<std::uint8_t>& value)
        : bytes(value), end(value.size() >= 32 ? value.size() - 32 : 0) {}
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0;
        if (!ok || width == 0 || width > 8 || cursor > end || width > end - cursor) {
            ok = false; return false;
        }
        for (unsigned index = 0; index < width; ++index)
            value |= std::uint64_t(bytes[cursor++]) << (8 * index);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0; if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
    }
    bool uuid(UUID& value) {
        std::uint64_t raw = 0;
        for (auto& byte : value) { if (!integer(1, raw)) return false; byte = std::uint8_t(raw); }
        return true;
    }
    bool complete() const noexcept { return ok && cursor == end; }
private:
    const std::vector<std::uint8_t>& bytes;
    std::size_t end = 0, cursor = 0;
    bool ok = true;
};

bool ReadLoop(Reader& reader, ProfileCurveLoop& loop) {
    std::uint64_t raw = 0, count = 0;
    if (!reader.integer(4, raw)) return false; loop.identifier = ProfileCurveID(raw);
    if (!reader.integer(2, count) || count < 2 || count > 512) return false;
    loop.vertices.resize(std::size_t(count)); loop.segments.resize(std::size_t(count));
    for (auto& vertex : loop.vertices) {
        double u = 0, v = 0;
        if (!reader.integer(4, raw) || !reader.scalar(u) || !reader.scalar(v)) return false;
        vertex = {ProfileCurveID(raw), gp_Pnt2d(u, v)};
    }
    for (auto& edge : loop.segments) {
        if (!reader.integer(4, raw)) return false; edge.identifier = ProfileCurveID(raw);
        if (!reader.integer(4, raw)) return false; edge.startVertex = ProfileCurveID(raw);
        if (!reader.integer(4, raw)) return false; edge.endVertex = ProfileCurveID(raw);
        if (!reader.integer(1, raw) || raw > 2) return false; edge.kind = ProfileCurveKind(raw);
        double u = 0, v = 0;
        if (!reader.scalar(u) || !reader.scalar(v) || !reader.scalar(edge.radius)
            || !reader.scalar(edge.startDegrees) || !reader.scalar(edge.sweepDegrees)) return false;
        edge.center = gp_Pnt2d(u, v);
    }
    return true;
}
} // namespace

bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 256 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SPRO\1\0\0\0", 8) != 0) return false;
        Digest expected{}, actual{};
        if (!CC_SHA256(bytes.data(), CC_LONG(bytes.size() - 32), expected.data())) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        Reader r(bytes); Definition value; std::uint64_t raw = 0, count = 0;
        for (int i = 0; i < 8; ++i) if (!r.integer(1, raw)) return false;
        if (!r.integer(4, raw)) return false; value.key.kind = std::uint32_t(raw);
        if (!r.integer(4, raw)) return false; value.key.codecVersion = std::uint32_t(raw);
        if (!r.uuid(value.owner.document) || !r.uuid(value.owner.entity)
            || !r.uuid(value.owner.definition) || !r.uuid(value.feature)
            || !r.integer(8, value.revision) || !r.integer(8, value.nextLocalIdentity)
            || !r.scalar(value.metersPerUnit) || !r.uuid(value.frame.identifier)
            || !r.integer(8, value.frame.revision)) return false;
        for (double& scalar : value.frame.origin) if (!r.scalar(scalar)) return false;
        for (double& scalar : value.frame.xAxis) if (!r.scalar(scalar)) return false;
        for (double& scalar : value.frame.yAxis) if (!r.scalar(scalar)) return false;
        for (double& scalar : value.frame.zAxis) if (!r.scalar(scalar)) return false;
        if (!r.integer(1, raw)) return false; value.frame.handedness = bounded_curve::Handedness(raw);
        if (!r.integer(1, raw)) return false; value.plane = int(raw);
        if (!r.integer(1, raw)) return false; value.operation = Operation(raw);
        if (!r.integer(1, raw)) return false; value.spline.innerLoopPolicy = SplineInnerLoopPolicy(raw);
        if (!r.scalar(value.depth) || !r.scalar(value.angleDegrees) || !r.integer(1, raw) || raw > 1) return false;
        if (raw == 1) {
            double ou = 0, ov = 0, du = 0, dv = 0;
            if (!r.scalar(ou) || !r.scalar(ov) || !r.scalar(du) || !r.scalar(dv)) return false;
            value.spline.revolveAxis = SplineRevolveAxis{gp_Pnt2d(ou, ov), gp_Pnt2d(du, dv)};
        }
        if (!ReadLoop(r, value.section.outer) || !r.integer(1, count) || count > 16) return false;
        value.section.inner.resize(std::size_t(count));
        for (auto& inner : value.section.inner) if (!ReadLoop(r, inner)) return false;
        if (!r.integer(1, count) || count == 0 || count > SplineMaximumSegmentsPerSection) return false;
        value.spline.segments.resize(std::size_t(count));
        for (auto& segment : value.spline.segments) {
            std::uint64_t poles = 0, knots = 0, rational = 0;
            if (!r.integer(4, raw)) return false; segment.identifier = ProfileCurveID(raw);
            if (!r.integer(1, raw)) return false; segment.degree = int(raw);
            if (!r.integer(1, poles) || poles < 2 || poles > SplineMaximumPoles
                || !r.integer(1, knots) || knots < 2 || knots > 40
                || !r.integer(1, rational) || rational > 1) return false;
            segment.poles.resize(std::size_t(poles));
            for (auto& pole : segment.poles) {
                double u = 0, v = 0;
                if (!r.integer(4, raw) || !r.scalar(u) || !r.scalar(v)) return false;
                pole = {ProfileCurveID(raw), gp_Pnt2d(u, v)};
            }
            segment.knots.resize(std::size_t(knots)); segment.multiplicities.resize(std::size_t(knots));
            for (std::size_t index = 0; index < segment.knots.size(); ++index) {
                if (!r.scalar(segment.knots[index]) || !r.integer(1, raw)) return false;
                segment.multiplicities[index] = int(raw);
            }
            if (rational) {
                segment.weights.resize(std::size_t(poles));
                for (double& weight : segment.weights) if (!r.scalar(weight)) return false;
            }
        }
        if (!r.integer(2, count) || count > 2048) return false;
        value.identities.resize(std::size_t(count));
        for (auto& identity : value.identities) {
            if (!r.uuid(identity.uuid) || !r.integer(4, raw)) return false;
            identity.local = ProfileCurveID(raw);
        }
        std::vector<std::uint8_t> canonical;
        if (!r.complete() || Validate(value) != Refusal::none
            || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

bool Build(const Definition& definition, DetachedSolid& output) noexcept {
    output = {};
    try {
        if (Validate(definition) != Refusal::none || !Encode(definition, output.canonical)) return false;
        if (!CC_SHA256(output.canonical.data(), CC_LONG(output.canonical.size()), output.digest.data())) return false;
        std::atomic_bool cancelled{false};
        double area = 0;
        const bool revolve = definition.operation == Operation::revolve;
        const double extent = revolve ? definition.angleDegrees : definition.depth;
        if (!SplineProfileExpectedVolume(definition.section, definition.spline,
                definition.plane, extent, revolve, [&] { return cancelled.load(); },
                area, output.expectedVolume)
            || !BuildWithDetachedProfileBranch(definition.section, definition.spline,
                definition.plane, extent, revolve, output.solid, output.seamFaces)) return false;
        return !output.solid.IsNull()
            && BRepCheck_Analyzer(output.solid, Standard_True).IsValid();
    } catch (...) { output = {}; return false; }
}

static bool SamePoint(const gp_Pnt2d& left, const gp_Pnt2d& right) noexcept {
    return left.X() == right.X() && left.Y() == right.Y();
}

static bool SameLoop(const ProfileCurveLoop& left,
                     const ProfileCurveLoop& right) noexcept {
    if (left.identifier != right.identifier
        || left.vertices.size() != right.vertices.size()
        || left.segments.size() != right.segments.size()) return false;
    for (std::size_t index = 0; index < left.vertices.size(); ++index)
        if (left.vertices[index].identifier != right.vertices[index].identifier
            || !SamePoint(left.vertices[index].point, right.vertices[index].point)) return false;
    for (std::size_t index = 0; index < left.segments.size(); ++index) {
        const auto& a = left.segments[index]; const auto& b = right.segments[index];
        if (a.identifier != b.identifier || a.startVertex != b.startVertex
            || a.endVertex != b.endVertex || a.kind != b.kind
            || !SamePoint(a.center, b.center) || a.radius != b.radius
            || a.startDegrees != b.startDegrees || a.sweepDegrees != b.sweepDegrees)
            return false;
    }
    return true;
}

static bool SameFrozenStructure(const Definition& left,
                                const Definition& right) noexcept {
    try {
        if (left.schema != right.schema || left.nextLocalIdentity != right.nextLocalIdentity
            || left.plane != right.plane || left.frame.identifier != right.frame.identifier
            || left.frame.origin != right.frame.origin || left.frame.xAxis != right.frame.xAxis
            || left.frame.yAxis != right.frame.yAxis || left.frame.zAxis != right.frame.zAxis
            || left.frame.handedness != right.frame.handedness
            || !SameLoop(left.section.outer, right.section.outer)
            || left.section.inner.size() != right.section.inner.size()
            || left.spline.segments.size() != right.spline.segments.size()) return false;
        for (std::size_t index = 0; index < left.section.inner.size(); ++index)
            if (!SameLoop(left.section.inner[index], right.section.inner[index])) return false;
        for (std::size_t index = 0; index < left.spline.segments.size(); ++index) {
            const auto& a = left.spline.segments[index];
            const auto& b = right.spline.segments[index];
            if (a.identifier != b.identifier || a.degree != b.degree
                || a.knots != b.knots || a.multiplicities != b.multiplicities
                || a.weights != b.weights || a.poles.size() != b.poles.size()) return false;
            for (std::size_t pole = 0; pole < a.poles.size(); ++pole)
                if (a.poles[pole].identifier != b.poles[pole].identifier) return false;
        }
        return true;
    } catch (...) { return false; }
}

bool ReadAll(const Handle(TDocStd_Document)& document, std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        std::vector<Record> staged; std::size_t aggregate = 0;
        for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) {
            Handle(Attribute) attribute;
            const TDF_Label label = it.Value();
            if (!label.FindAttribute(AttributeID(), attribute)) continue;
            Handle(TNaming_NamedShape) binding;
            std::vector<std::uint8_t> canonical;
            const TDF_Label owner = label.Father();
            if (attribute.IsNull() || label.Tag() != RecordTag
                || owner.IsNull()
                || staged.size() >= MaximumRecordsPerDocument
                || !Encode(attribute->definition(), canonical)
                || canonical != attribute->bytes()
                || aggregate > MaximumPayloadBytes * MaximumRecordsPerDocument - canonical.size()
                || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)
                || binding.IsNull() || binding->Get().IsNull()
                || binding->Get().ShapeType() != TopAbs_SOLID) return false;
            const TopoDS_Shape ownerShape = XCAFDoc_ShapeTool::GetShape(owner);
            if (ownerShape.IsNull() || ownerShape.ShapeType() != TopAbs_SOLID
                || !ownerShape.IsEqual(binding->Get())) return false;
            for (const auto& prior : staged)
                if (prior.owner.IsEqual(owner)
                    || prior.attribute->definition().owner == attribute->definition().owner)
                    return false;
            aggregate += canonical.size(); staged.push_back({label, owner, attribute, binding->Get()});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

Standard_Boolean BinaryDriver::Paste(const BinObjMgt_Persistent& source,
    const Handle(TDF_Attribute)& target, BinObjMgt_RRelocationTable& relocation) const {
    try {
        const auto attribute = Handle(Attribute)::DownCast(target);
        if (attribute.IsNull() || !attribute->bytes_.empty() || !budget_
            || budget_->rejected || relocation.GetHeaderData().IsNull()) return Refuse();
        const auto version = relocation.GetHeaderData()->StorageVersion();
        if (!version.IsIntegerValue()
            || version.IntegerValue() < TDocStd_FormatVersion_VERSION_10
            || version.IntegerValue() > TDocStd_FormatVersion_CURRENT) return Refuse();
        const auto start = source.Position(), length = source.Length();
        if (length < 0 || length > INT_MAX - 8 || start < 8 || start > length + 8)
            return Refuse();
        const auto end = length + 8;
        Standard_Integer schema = 0, count = 0;
        if (!(source >> schema >> count) || schema != 1
            || count <= 0 || count > Standard_Integer(MaximumPayloadBytes)
            || source.Position() > end || count != end - source.Position()
            || budget_->bytes > MaximumDocumentAggregateBytes
            || std::size_t(count) > MaximumDocumentAggregateBytes - budget_->bytes
            || budget_->records >= MaximumRecordsPerDocument) return Refuse();
        std::vector<std::uint8_t> bytes(static_cast<std::size_t>(count));
        Definition definition;
        if (!source.GetByteArray(bytes.data(), count) || source.Position() != end
            || !Decode(bytes, definition)) return Refuse();
        attribute->bytes_ = std::move(bytes); attribute->definition_ = std::move(definition);
        budget_->bytes += std::size_t(count); ++budget_->records;
        return Standard_True;
    } catch (...) { return Refuse(); }
}

void BinaryDriver::Paste(const Handle(TDF_Attribute)& source,
    BinObjMgt_Persistent& target, BinObjMgt_SRelocationTable&) const {
    const auto attribute = Handle(Attribute)::DownCast(source);
    std::vector<std::uint8_t> canonical;
    const auto document = attribute.IsNull() ? Handle(TDocStd_Document)()
        : TDocStd_Document::Get(attribute->Label());
    if (attribute.IsNull() || document.IsNull()
        || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
        || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT
        || !Encode(attribute->definition_, canonical)
        || canonical != attribute->bytes_) Standard_Failure::Raise("C4 writer canonical");
    target << Standard_Integer(1) << Standard_Integer(canonical.size());
    target.PutByteArray(canonical.data(), Standard_Integer(canonical.size()));
}
} // namespace core3d::spline_profile

namespace core3d::spline_profile::owner {
struct OcafOwner::State final {
    OcctDocument* document = nullptr;
    std::shared_ptr<native_opening::Context> context;
    std::uint64_t nextSession = 0, nextPreparation = 0;
    struct Capture { std::shared_ptr<const Opening> value; std::shared_ptr<OcctSplineProfileCapture> exact; bool create = false; };
    struct Preparation { std::shared_ptr<const Prepared> value; Capture capture; };
    std::unordered_map<std::uint64_t, Capture> captures;
    std::unordered_map<std::uint64_t, Preparation> preparations;
    bool bound() const noexcept {
        return document && context && !document->Document().IsNull()
            && context->openingFence().document() == document->Document()
            && context->openingFence().data() == document->Document()->GetData();
    }
};

static Receipt Refuse(Outcome outcome, const char* reason) {
    Receipt receipt; receipt.outcome = outcome; receipt.reason = reason; return receipt;
}

OcafOwner::OcafOwner(OcctDocument& document,
    std::shared_ptr<native_opening::Context> context) noexcept : state_(new State()) {
    state_->document = &document; state_->context = std::move(context);
}
OcafOwner::~OcafOwner() = default;

std::shared_ptr<const Opening> OcafOwner::beginCreate(const Definition& input) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()) return {};
    try {
        Definition value = input;
        if (Validate(value) != Refusal::none || state_->nextSession == UINT64_MAX) return {};
        DetachedSolid detached; if (!Build(value, detached)) return {};
        auto opening = std::make_shared<Opening>(); opening->definition = std::move(value);
        opening->canonical = detached.canonical; opening->digest = detached.digest;
        opening->session = ++state_->nextSession;
        state_->captures.emplace(opening->session, State::Capture{opening, {}, true});
        return opening;
    } catch (...) { return {}; }
}

std::shared_ptr<const Opening> OcafOwner::capture(const std::string& entity) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()) return {};
    try {
        auto exact = std::make_shared<OcctSplineProfileCapture>();
        if (!state_->document->CaptureSplineProfileExact(entity, *state_->context, *exact)) return {};
        auto opening = std::make_shared<Opening>(); opening->definition = exact->definition;
        if (!Encode(opening->definition, opening->canonical)
            || !CC_SHA256(opening->canonical.data(), CC_LONG(opening->canonical.size()),
                          opening->digest.data()) || state_->nextSession == UINT64_MAX) return {};
        opening->session = ++state_->nextSession;
        state_->captures.emplace(opening->session, State::Capture{opening, exact, false});
        return opening;
    } catch (...) { return {}; }
}

std::shared_ptr<const Prepared> OcafOwner::prepare(
    const std::shared_ptr<const Opening>& opening, const Definition& replacement,
    Receipt& receipt) noexcept {
    receipt = Refuse(Outcome::refused, "prepare-refused");
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !opening
        || state_->nextPreparation == UINT64_MAX) return {};
    try {
        const auto found = state_->captures.find(opening->session);
        if (found == state_->captures.end() || found->second.value != opening) return {};
        if (!found->second.create) {
            OcctSplineProfileCapture current;
            if (!state_->document->ReadSplineProfileExact(opening->definition.owner, current)
                || !found->second.exact || !found->second.exact->IsEqual(current)) {
                receipt = Refuse(Outcome::staleDefinition, "capture-no-longer-current"); return {};
            }
        }
        // Identity, registry key, units, frame identity and structure are
        // immutable in row 270. Pole coordinates, axis, extent and policy are
        // complete replacement values, never index-addressed deltas.
        if (!(replacement.owner == opening->definition.owner)
            || replacement.feature != opening->definition.feature
            || replacement.key.kind != opening->definition.key.kind
            || replacement.key.codecVersion != opening->definition.key.codecVersion
            || replacement.metersPerUnit != opening->definition.metersPerUnit
            || !SameFrozenStructure(replacement, opening->definition)
            || replacement.identities != opening->definition.identities
            || replacement.revision != opening->definition.revision + (found->second.create ? 0 : 1))
            return {};
        DetachedSolid detached; if (!Build(replacement, detached)) return {};
        auto prepared = std::make_shared<Prepared>(); prepared->opening = *opening;
        prepared->replacement = replacement; prepared->detached = std::move(detached);
        prepared->preparation = ++state_->nextPreparation;
        state_->preparations.emplace(prepared->preparation,
            State::Preparation{prepared, found->second});
        receipt.outcome = Outcome::prepared; receipt.reason = "prepared";
        receipt.session = opening->session; receipt.preparation = prepared->preparation;
        return prepared;
    } catch (...) { receipt.reason = "prepare-exception"; return {}; }
}

Receipt OcafOwner::apply(const std::shared_ptr<const Prepared>& prepared,
                         const std::string& requestedName) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !prepared)
        return Refuse(Outcome::refused, "apply-refused");
    try {
        const auto found = state_->preparations.find(prepared->preparation);
        if (found == state_->preparations.end() || found->second.value != prepared)
            return Refuse(Outcome::staleDefinition, "preparation-not-current");
        std::vector<OcctIssuedLabelIdentity> issued;
        Definition definition = prepared->replacement;
        if (found->second.capture.create) {
            if (!state_->document->ReserveExactLabelIdentities(1, {}, issued) || issued.size() != 1
                || !receipt::ParseUUID(issued.front().EntityIdentifier(), definition.owner.entity)
                || !receipt::ParseUUID(issued.front().DefinitionIdentifier(), definition.owner.definition))
                return Refuse(Outcome::busy, "identity-issuance-refused");
        }
        DetachedSolid detached; if (!Build(definition, detached)) return Refuse(Outcome::refused, "rebuild-refused");
        const int before = state_->document->Document()->GetAvailableUndos();
        auto lease = state_->context->beginCommandLease(state_->context->openingFence(), 64, 64);
        if (!lease) return Refuse(Outcome::busy, "command-refused");
        OcctSplineProfileCapture staged;
        const bool stagedOK = found->second.capture.create
            ? state_->document->StageSplineProfileCreate(*lease, issued.front(), definition,
                  detached, requestedName, staged)
            : state_->document->StageSplineProfileReplacement(*lease,
                  *found->second.capture.exact, definition, detached, staged);
        if (!stagedOK) {
            const bool restored = lease->abort();
            return Refuse(restored ? Outcome::refused : Outcome::recoveryRequired,
                          restored ? "stage-refused" : "abort-unknown");
        }
        if (!lease->commit()) return Refuse(Outcome::outcomeUnknown, "close-unknown");
        OcctSplineProfileCapture read;
        const int delta = state_->document->Document()->GetAvailableUndos() - before;
        if (delta != 1 || !state_->document->ReadSplineProfileExact(definition.owner, read)
            || !staged.IsEqual(read)) return Refuse(Outcome::outcomeUnknown, "post-close-proof-failed");
        Receipt receipt; receipt.outcome = Outcome::committed; receipt.reason = "committed";
        receipt.session = prepared->opening.session; receipt.preparation = prepared->preparation;
        receipt.historyDelta = delta; state_->captures.erase(receipt.session);
        state_->preparations.erase(found); return receipt;
    } catch (...) { return Refuse(Outcome::recoveryRequired, "apply-exception"); }
}

Receipt OcafOwner::cancel(std::uint64_t session) noexcept {
    if (!state_) return Refuse(Outcome::refused, "owner-retired");
    for (auto it = state_->preparations.begin(); it != state_->preparations.end(); )
        if (it->second.capture.value->session == session) it = state_->preparations.erase(it); else ++it;
    if (state_->captures.erase(session) == 0) return Refuse(Outcome::unchanged, "already-retired");
    Receipt receipt; receipt.outcome = Outcome::cancelled; receipt.reason = "cancelled";
    receipt.session = session; return receipt;
}
} // namespace core3d::spline_profile::owner
