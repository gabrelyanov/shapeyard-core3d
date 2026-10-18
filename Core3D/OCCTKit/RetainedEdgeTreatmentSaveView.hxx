#pragma once

#include "RetainedEdgeTreatmentAttribute.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_LocationDriver.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Data.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_Iterator.hxx>
#include <TNaming_Tool.hxx>
#include <XCAFDoc_Location.hxx>
#include <algorithm>
#include <memory>
#include <vector>

// Writer-only, bounded serialization view for an already-admitted B1 owner.
// It never modifies the live TDF_Data: the private attributes below exist on
// a separate TDF_Data solely for the duration of one synchronous SaveAs.
namespace core3d::retained_edge_treatment::save_view {
inline constexpr std::size_t MaximumViewRecords = 256;
inline constexpr std::size_t MaximumViewLabels = 4096;
inline constexpr std::size_t MaximumNamingPairs = 4096;

struct NamingPair final {
    TopoDS_Shape oldShape;
    TopoDS_Shape newShape;
};

struct Entry final {
    TDF_Label owner;
    TDF_Label source;
    Handle(TNaming_NamedShape) ownerNaming;
    Handle(TNaming_NamedShape) sourceNaming;
    Handle(TNaming_NamedShape) ownerView;
    Handle(TNaming_NamedShape) sourceView;
    std::shared_ptr<const Payload> payload;
    TopoDS_Shape current;
};

inline bool SameShape(const TopoDS_Shape& first,
                      const TopoDS_Shape& second) noexcept {
    return first.IsNull() ? second.IsNull()
        : !second.IsNull() && first.IsEqual(second);
}

inline bool ReadPairs(const Handle(TNaming_NamedShape)& naming,
                      std::vector<NamingPair>& pairs) noexcept {
    pairs.clear();
    try {
        if (naming.IsNull()) return false;
        for (TNaming_Iterator it(naming); it.More(); it.Next()) {
            if (pairs.size() >= MaximumNamingPairs) return false;
            pairs.push_back({it.OldShape(), it.NewShape()});
        }
        return !pairs.empty();
    } catch (...) { pairs.clear(); return false; }
}

inline bool HasShape(const Handle(TNaming_NamedShape)& naming,
                     const TopoDS_Shape& shape) noexcept {
    try {
        std::size_t count = 0;
        for (TNaming_Iterator it(naming); it.More(); it.Next()) {
            if (++count > MaximumNamingPairs) return true;
            if (SameShape(it.OldShape(), shape)
                || SameShape(it.NewShape(), shape)) return true;
        }
    } catch (...) { return true; }
    return false;
}

inline bool Populate(const Handle(TDF_Data)& data,
                     const Handle(TNaming_NamedShape)& original,
                     const TopoDS_Shape& current,
                     const TopoDS_Shape& replay,
                     bool reverse,
                     Handle(TNaming_NamedShape)& output) noexcept {
    output.Nullify();
    try {
        std::vector<NamingPair> pairs;
        if (data.IsNull() || !ReadPairs(original, pairs)) return false;
        std::size_t replaced = 0;
        for (auto& pair : pairs) {
            if (SameShape(pair.oldShape, current)) {
                pair.oldShape = replay; ++replaced;
            }
            if (SameShape(pair.newShape, current)) {
                pair.newShape = replay; ++replaced;
            }
        }
        if (!replaced) return false;
        const TDF_Label label = data->Root().NewChild();
        TNaming_Builder builder(label);
        const auto add = [&](const NamingPair& pair) {
            switch (original->Evolution()) {
                case TNaming_PRIMITIVE: builder.Generated(pair.newShape); break;
                case TNaming_GENERATED:
                    builder.Generated(pair.oldShape, pair.newShape); break;
                case TNaming_MODIFY:
                    builder.Modify(pair.oldShape, pair.newShape); break;
                case TNaming_DELETE: builder.Delete(pair.oldShape); break;
                case TNaming_SELECTED:
                    builder.Select(pair.newShape, pair.oldShape); break;
                default: Standard_Failure::Raise("B5 unsupported naming evolution");
            }
        };
        if (reverse) {
            for (auto it = pairs.rbegin(); it != pairs.rend(); ++it) add(*it);
        } else {
            for (const auto& pair : pairs) add(pair);
        }
        output = builder.NamedShape();
        if (output.IsNull() || output->Evolution() != original->Evolution()) return false;
        output->SetVersion(original->Version());
        std::vector<NamingPair> written;
        if (!ReadPairs(output, written) || written.size() != pairs.size()) return false;
        for (std::size_t index = 0; index < pairs.size(); ++index)
            if (!SameShape(written[index].oldShape, pairs[index].oldShape)
                || !SameShape(written[index].newShape, pairs[index].newShape)) return false;
        return output->Version() == original->Version();
    } catch (...) { output.Nullify(); return false; }
}

class View final {
public:
    static std::shared_ptr<const View> Build(
        const Handle(TDocStd_Document)& document) noexcept {
        try {
            if (document.IsNull() || document->GetData().IsNull()
                || document->HasOpenCommand()
                || !Core3DValidateRetainedEdgeTreatmentDocument(document)) return {};
            std::vector<Record> records;
            std::vector<RecordR2> recordsR2;
            Refusal refusal = Refusal::MalformedCarrier;
            if (!ReadAllMixed(document, 0, records, recordsR2, refusal)
                || records.size() > MaximumViewRecords) return {};
            auto view = std::shared_ptr<View>(new View(document));
            for (const auto& record : records)
                if (!view->Add(record)) return {};
            return view->StillCurrent() ? view : std::shared_ptr<const View>();
        } catch (...) { return {}; }
    }

    bool Matches(const Handle(TDocStd_Document)& document) const noexcept {
        return !document.IsNull() && document == document_
            && document->GetData() == data_;
    }

    bool Empty() const noexcept { return entries_.empty(); }

    bool StillCurrent() const noexcept {
        try {
            if (document_.IsNull() || document_->GetData() != data_
                || document_->HasOpenCommand()) return false;
            for (const auto& entry : entries_) {
                Handle(Attribute) carrier;
                Handle(TNaming_NamedShape) ownerNaming, sourceNaming;
                if (entry.owner.IsNull() || entry.source.IsNull()
                    || entry.owner.Data() != data_ || entry.source.Data() != data_
                    || !entry.source.FindAttribute(AttributeID(), carrier)
                    || carrier.IsNull() || carrier->value() != entry.payload
                    || !entry.owner.FindAttribute(TNaming_NamedShape::GetID(), ownerNaming)
                    || !entry.source.FindAttribute(TNaming_NamedShape::GetID(), sourceNaming)
                    || ownerNaming != entry.ownerNaming || sourceNaming != entry.sourceNaming
                    || !SameShape(XCAFDoc_ShapeTool::GetShape(entry.owner), entry.current)
                    || !SameShape(TNaming_Tool::GetShape(sourceNaming), entry.current)) return false;
                Record live{entry.source, entry.owner, entry.payload,
                    TNaming_Tool::GetShape(sourceNaming)};
                if (!live.IsCurrent(document_, entry.owner)) return false;
            }
            return true;
        } catch (...) { return false; }
    }

    bool Replacement(const Handle(TNaming_NamedShape)& source,
                     Handle(TNaming_NamedShape)& output) const noexcept {
        output.Nullify();
        if (source.IsNull()) return false;
        for (const auto& entry : entries_) {
            if (source == entry.ownerNaming) { output = entry.ownerView; return true; }
            if (source == entry.sourceNaming) { output = entry.sourceView; return true; }
        }
        return false;
    }

private:
    explicit View(const Handle(TDocStd_Document)& document)
        : document_(document), data_(document->GetData()), privateData_(new TDF_Data) {}

    bool PrivateNaming(const Handle(TNaming_NamedShape)& original,
                       const TopoDS_Shape& current,
                       const TopoDS_Shape& replay,
                       Handle(TNaming_NamedShape)& output) noexcept {
        if (Populate(privateData_, original, current, replay, true, output)) return true;
        return Populate(privateData_, original, current, replay, false, output);
    }

    bool Add(const Record& record) noexcept {
        try {
            if (record.owner.IsNull() || record.label.IsNull() || !record.value
                || !record.IsCurrent(document_, record.owner)
                || record.value->base.IsNull() || record.current.IsNull()) return false;
            std::vector<std::uint8_t> encoded;
            Refusal refusal = Refusal::MalformedCarrier;
            if (!Encode(record.value->definition, encoded, refusal)
                || encoded != record.value->bytes) return false;
            ReplayBudget budget;
            TopoDS_Shape replay;
            std::vector<StepProof> proofs;
            if (!Replay(record.value->base, record.value->definition,
                    replay, proofs, budget, refusal)
                || proofs.size() != record.value->definition.steps.size()
                || !EquivalentReplayGeometry(record.current, replay, budget, refusal)
                || replay.IsNull() || replay.ShapeType() != TopAbs_SOLID
                || replay.Orientation() != TopAbs_FORWARD) return false;
            Handle(TNaming_NamedShape) ownerNaming, sourceNaming;
            if (!record.owner.FindAttribute(TNaming_NamedShape::GetID(), ownerNaming)
                || !record.label.FindAttribute(TNaming_NamedShape::GetID(), sourceNaming)
                || ownerNaming.IsNull() || sourceNaming.IsNull()
                || ownerNaming == sourceNaming
                || !SameShape(TNaming_Tool::GetShape(ownerNaming), record.current)
                || !SameShape(TNaming_Tool::GetShape(sourceNaming), record.current)) return false;

            // A full-shape consumer other than the owner and its one source/
            // carrier binding is not silently redirected by a private save.
            std::size_t labels = 0;
            const TDF_Label root = data_->Root();
            Handle(TNaming_NamedShape) rootNaming;
            if (root.FindAttribute(TNaming_NamedShape::GetID(), rootNaming)
                && HasShape(rootNaming, record.current)) return false;
            for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
                if (++labels > MaximumViewLabels) return false;
                Handle(TNaming_NamedShape) naming;
                if (!it.Value().FindAttribute(TNaming_NamedShape::GetID(), naming)
                    || !HasShape(naming, record.current)) continue;
                if (naming != ownerNaming && naming != sourceNaming) return false;
            }
            Entry entry;
            entry.owner = record.owner; entry.source = record.label;
            entry.ownerNaming = ownerNaming; entry.sourceNaming = sourceNaming;
            entry.payload = record.value; entry.current = record.current;
            if (!PrivateNaming(ownerNaming, record.current, replay, entry.ownerView)
                || !PrivateNaming(sourceNaming, record.current, replay,
                    entry.sourceView)) return false;
            entries_.push_back(std::move(entry));
            return true;
        } catch (...) { return false; }
    }

    Handle(TDocStd_Document) document_;
    Handle(TDF_Data) data_;
    Handle(TDF_Data) privateData_;
    std::vector<Entry> entries_;
};

inline thread_local std::shared_ptr<const View> ActiveView;

class Scope final {
public:
    explicit Scope(std::shared_ptr<const View> view)
        : previous_(std::move(ActiveView)) { ActiveView = std::move(view); }
    ~Scope() noexcept { ActiveView = std::move(previous_); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
private:
    std::shared_ptr<const View> previous_;
};

class NamedShapeDriver final : public BinMNaming_NamedShapeDriver {
public:
    NamedShapeDriver(const Handle(Message_Messenger)& messenger,
                     const Handle(BinMNaming_NamedShapeDriver)& original)
        : BinMNaming_NamedShapeDriver(messenger) {
        if (original.IsNull()) Standard_Failure::Raise("B5 missing naming writer");
        SetWithTriangles(original->IsWithTriangles());
        SetWithNormals(original->IsWithNormals());
        EnableQuickPart(original->IsQuickPart());
    }

    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
        const Handle(TDF_Attribute)& target,
        BinObjMgt_RRelocationTable& relocation) const override {
        return BinMNaming_NamedShapeDriver::Paste(source, target, relocation);
    }

    void Paste(const Handle(TDF_Attribute)& source,
        BinObjMgt_Persistent& target,
        BinObjMgt_SRelocationTable& relocation) const override {
        const auto view = ActiveView;
        if (!view || !view->StillCurrent())
            Standard_Failure::Raise("B5 inactive or stale save view");
        const auto naming = Handle(TNaming_NamedShape)::DownCast(source);
        Handle(TNaming_NamedShape) replacement;
        if (!naming.IsNull() && view->Replacement(naming, replacement))
            BinMNaming_NamedShapeDriver::Paste(replacement, target, relocation);
        else BinMNaming_NamedShapeDriver::Paste(source, target, relocation);
    }
};

inline Handle(BinMNaming_NamedShapeDriver) Install(
    const Handle(BinMDF_ADriverTable)& table,
    const Handle(Message_Messenger)& messenger) {
    if (table.IsNull()) Standard_Failure::Raise("B5 missing driver table");
    Handle(BinMDF_ADriver) found;
    table->GetDriver(STANDARD_TYPE(TNaming_NamedShape), found);
    const auto original = Handle(BinMNaming_NamedShapeDriver)::DownCast(found);
    if (original.IsNull()) Standard_Failure::Raise("B5 missing shared naming writer");
    Handle(NamedShapeDriver) adapter = new NamedShapeDriver(messenger, original);
    table->AddDriver(adapter);
    found.Nullify(); table->GetDriver(STANDARD_TYPE(TNaming_NamedShape), found);
    if (found != adapter) Standard_Failure::Raise("B5 naming adapter registration");
    Handle(BinMDF_ADriver) locationBase;
    table->GetDriver(STANDARD_TYPE(XCAFDoc_Location), locationBase);
    const auto location = Handle(BinMXCAFDoc_LocationDriver)::DownCast(locationBase);
    if (!location.IsNull()) location->SetNSDriver(adapter);
    return adapter;
}
} // namespace core3d::retained_edge_treatment::save_view
