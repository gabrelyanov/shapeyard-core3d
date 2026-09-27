#pragma once

// DEBUG-only saved-document fixture for C2 register rows 16/17. It stages a
// real SYCR/2 owner, embedded SYCV/1 path, SCSW/1 feature and canonical BRep;
// the caller owns save and source-document destruction.
#include "SpatialSweepProbe.hxx"
#include "PartBooleanCodecProbe.hxx"
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_ChildIterator.hxx>
#include <TNaming_Builder.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace core3d::composite_recipe {
// CompositeRecipeAttribute deliberately exposes fixture staging only to this
// DEBUG friend. Production mutation remains exclusively in the G0 transaction.
struct SpatialSweepColdOpenFixture {
    enum class Variant { regression, b10PostV1 };

    using DocumentValidator = std::function<bool(const Handle(TDocStd_Document)&)>;
    using AdmissionChecks = std::map<std::string, bool>;

    static bool StageSpatialSweepColdOpenFixture(
        const Handle(TDocStd_Document)& document,
        const double metersPerUnit,
        const Variant variant = Variant::regression) noexcept {
        try {
            using spatial_sweep::debug::Hash;
            using spatial_sweep::debug::ID;
            if (document.IsNull()
                || (metersPerUnit != 0.001 && metersPerUnit != 1.0)) return false;
            XCAFDoc_DocumentTool::SetLengthUnit(document, metersPerUnit);

            const OwnerKey ownerKey{ID(1), ID(2), ID(3)};
            bounded_curve::Value curve;
            curve.feature = ID(21);
            curve.definition = spatial_sweep::debug::SpatialPolynomialCurve();
            if (variant == Variant::b10PostV1) {
                const std::array<std::array<double, 3>, 4> poles{{
                    {{0, 0, 150}}, {{0, 0, 1350}},
                    {{600, 0, 1350}}, {{600, 0, 950}}
                }};
                if (curve.definition.controlPoints.size() != poles.size()) return false;
                for (std::size_t index = 0; index < poles.size(); ++index)
                    curve.definition.controlPoints[index].local = poles[index];
                curve.definition.weights = std::vector<double>(poles.size(), 1.0);
            }
            const double nativePerMM = 0.001 / metersPerUnit;
            for (auto& pole : curve.definition.controlPoints)
                for (double& scalar : pole.local) scalar *= nativePerMM;
            std::vector<std::uint8_t> curveBytes;
            Digest curveDigest{};
            if (!bounded_curve::Encode(curve, curveBytes)
                || !bounded_curve::Hash(curveBytes,
                    bounded_curve::MaximumDefinitionBytes, curveDigest)) return false;

            spatial_sweep::Definition sweep;
            sweep.path.inputNode = ID(20);
            sweep.path.curveFeature = curve.feature;
            sweep.path.sourceRecipeDigest = curveDigest;
            sweep.path.ownerState.definitionRevision = 1;
            sweep.path.ownerState.nextLocalID = 5;
            sweep.path.ownerState.canonicalDefinitionDigest = curveDigest;
            sweep.dimensionMetersPerUnit = metersPerUnit;
            sweep.sectionIdentifier = ID(24);
            sweep.orientation.authoredSeed = variant == Variant::b10PostV1
                ? std::array<double, 3>{{0, 1, 0}}
                : std::array<double, 3>{{0, 0, 1}};
            sweep.orientation.phaseRadians = 0;
            const double radius = (variant == Variant::b10PostV1 ? 12.0 : 2.0)
                * nativePerMM;
            sweep.radius = {spatial_sweep::RadiusLawKind::Constant, radius, radius};
            sweep.twist = {spatial_sweep::TwistLawKind::LinearArcLength, 0, 0};
            sweep.closure = spatial_sweep::ClosureKind::OpenFlatCaps;
            std::vector<std::uint8_t> featureBytes;
            if (!spatial_sweep::Encode(sweep, featureBytes)) return false;

            TopoDS_Shape sourceWire;
            Digest wireDigest{};
            if (!spatial_sweep::editor::MakeNativeWire(
                    curve.definition, sourceWire, wireDigest)) return false;
            std::atomic_bool cancelled{false};
            const auto first = spatial_sweep::BuildGeomFillSweep(
                curve.definition, sweep, cancelled);
            const auto second = spatial_sweep::BuildGeomFillSweep(
                curve.definition, sweep, cancelled);
            if (!first.built() || !second.built()) return false;
            const auto fixed = spatial_g0::PrepareFixedPoint(
                first.solid, second.solid, spatial_g0::PinnedProfile);
            if (!fixed.admitted()) return false;

            SourceNode source;
            source.node = sweep.path.inputNode;
            source.localID = 1;
            source.original = {ownerKey.document, ownerKey.entity,
                ownerKey.definition, curve.feature};
            source.recipe = {RecipeKind::BoundedCurvePath,
                bounded_curve::Schema, curveBytes};
            source.inputToCarrier.sourceMetersPerUnit = metersPerUnit;
            source.inputToCarrier.carrierMetersPerUnit = metersPerUnit;
            source.commitments = {wireDigest, curveDigest, Hash(81), Hash(82), Hash(83)};
            source.shapeSlot = 0;
            FeatureNode feature;
            feature.node = ID(30); feature.feature = ID(31); feature.localID = 2;
            feature.kind = SpatialCircleSweepFeatureKind;
            feature.codecVersion = SpatialCircleSweepFeatureCodec;
            feature.inputs = {source.node}; feature.parameters = featureBytes;

            auto payload = std::make_shared<Payload>();
            payload->definition.schemaVersion = 2;
            payload->definition.owner = ownerKey;
            payload->definition.outputNode = feature.node;
            payload->definition.issuance.nextLocalID = 3;
            payload->definition.nodes = {Node{source}, Node{feature}};
            payload->sourceShapes = {sourceWire};
            if (!EncodeV2(payload->definition, payload->bytes)) return false;

            const Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return false;
            const TDF_Label owner = shapes->AddShape(
                fixed.canonical, Standard_False, Standard_True);
            if (owner.IsNull()) return false;
            TDataStd_Integer::Set(owner,
                Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1);
            TDataStd_Name::Set(owner,
                TCollection_ExtendedString(variant == Variant::b10PostV1
                    ? "Lantern.Post" : "C2 retained spatial sweep"));
            if (variant == Variant::b10PostV1) {
                const Handle(XCAFDoc_VisMaterialTool) materials =
                    XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
                if (materials.IsNull()) return false;
                XCAFDoc_VisMaterialPBR pbr;
                pbr.BaseColor = Quantity_ColorRGBA(
                    Quantity_Color(41.0 / 255.0, 50.0 / 255.0, 58.0 / 255.0,
                        Quantity_TOC_sRGB), 1.0f);
                pbr.Metallic = 0.7f;
                pbr.Roughness = 0.38f;
                pbr.IsDefined = Standard_True;
                Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
                material->SetPbrMaterial(pbr);
                const TDF_Label materialLabel = materials->AddMaterial(
                    material, "B10 Lantern.Post iron");
                if (materialLabel.IsNull()) return false;
                materials->SetShapeMaterial(owner, materialLabel);
            }
            const auto setUUID = [](const TDF_Label& label,
                                    const char* attributeID,
                                    const UUID& value) {
                return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
                    TCollection_AsciiString(retained_solid::UUIDText(value).c_str())).IsNull();
            };
            if (!setUUID(document->Main(),
                    "74386E4E-F620-498F-8092-E6D883AF33A4", ownerKey.document)
                || !setUUID(owner,
                    "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", ownerKey.entity)
                || !setUUID(owner,
                    "3611F2B2-C694-4E12-AED8-A2A97A3D283B", ownerKey.definition)) return false;

            document->SetUndoLimit(16);
            document->NewCommand();
            const TDF_Label record = owner.FindChild(MinimumRecordTag, Standard_True);
            Handle(Attribute) attribute = new Attribute();
            if (record.IsNull() || attribute.IsNull()) {
                document->AbortCommand(); return false;
            }
            record.AddAttribute(attribute);
            attribute->value_ = payload;
            TNaming_Builder(record).Select(fixed.canonical, fixed.canonical);
            if (!document->CommitCommand()) return false;
            Record readback;
            return Read(document, owner, readback) && readback.value
                && readback.value->bytes == payload->bytes
                && readback.current.IsEqual(fixed.canonical);
        } catch (...) { return false; }
    }

    static AdmissionChecks DocumentAdmissionChecks(
        const DocumentValidator& validate) noexcept {
        AdmissionChecks checks;
        try {
            if (!validate) return checks;
            struct StandaloneDocument final {
                Handle(TDF_Data) retainedData;
                Handle(TDocStd_Document) document = new TDocStd_Document(
                    TCollection_ExtendedString("BinXCAF"));
                bool closed = false;
                ~StandaloneDocument() noexcept { (void)close(); }
                bool close() noexcept {
                    if (closed) return true;
                    bool clean = !document.IsNull();
                    try {
                        if (!document.IsNull()) {
                            retainedData = document->GetData();
                            clean = clean && !retainedData.IsNull();
                            if (document->HasOpenCommand()) {
                                document->AbortCommand();
                                clean = false;
                            }
                            document->BeforeClose();
                        }
                    } catch (...) {
                        clean = false;
                        try { if (!document.IsNull()) document->ClearUndos(); } catch (...) {}
                    }
                    document.Nullify();
                    closed = true;
                    return clean;
                }
            };
            struct ValidationResult final {
                bool setup = false;
                bool evaluated = false;
                bool accepted = false;
                bool cleaned = false;
                operator bool() const noexcept {
                    return setup && evaluated && accepted && cleaned;
                }
                bool operator!() const noexcept {
                    return setup && evaluated && !accepted && cleaned;
                }
            };
            const auto fresh = [](double metersPerUnit) {
                auto holder = std::make_unique<StandaloneDocument>();
                const bool staged = !holder->document.IsNull()
                    && StageSpatialSweepColdOpenFixture(holder->document, metersPerUnit);
                return std::make_pair(std::move(holder), staged);
            };
            const auto attribute = [](const Handle(TDocStd_Document)& document) {
                Handle(Attribute) result;
                if (document.IsNull() || document->GetData().IsNull()) return result;
                for (TDF_ChildIterator child(document->GetData()->Root(), Standard_True);
                     child.More(); child.Next()) {
                    if (child.Value().FindAttribute(AttributeID(), result)) return result;
                }
                result.Nullify();
                return result;
            };
            const auto mutate = [&](double metersPerUnit, const auto& change) {
                ValidationResult result;
                auto [holder, staged] = fresh(metersPerUnit);
                Handle(Attribute) stored = attribute(holder->document);
                result.setup = staged && !stored.IsNull() && stored->value_;
                if (result.setup) {
                    try {
                        auto changed = std::make_shared<Payload>(*stored->value_);
                        change(holder->document, *changed);
                        stored->Backup();
                        stored->value_ = changed;
                        result.accepted = validate(holder->document);
                        result.evaluated = true;
                    } catch (...) {}
                }
                result.cleaned = holder->close();
                return result;
            };
            for (const double metersPerUnit : {0.001, 1.0}) {
                const std::string unit = metersPerUnit == 0.001 ? "mm" : "m";
                auto [positive, staged] = fresh(metersPerUnit);
                ValidationResult result;
                result.setup = staged && !attribute(positive->document).IsNull();
                if (result.setup) {
                    try {
                        result.accepted = validate(positive->document);
                        result.evaluated = true;
                    } catch (...) {}
                }
                result.cleaned = positive->close();
                checks[unit + "-staging-succeeds"] = result.setup && result.cleaned;
                checks[unit + "-valid-forward-bound-wire-accepts"] = result;

                checks[unit + "-null-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.sourceShapes.front().Nullify();
                    });
                checks[unit + "-empty-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        BRep_Builder builder;
                        TopoDS_Wire empty;
                        builder.MakeWire(empty);
                        empty.Orientation(TopAbs_FORWARD);
                        payload.sourceShapes.front() = empty;
                    });
                checks[unit + "-reversed-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.sourceShapes.front().Reverse();
                    });
                checks[unit + "-wrong-shape-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.sourceShapes.front() = BRepPrimAPI_MakeBox(1, 1, 1).Shape();
                    });
                checks[unit + "-invalid-geometric-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        BRep_Builder builder;
                        TopoDS_Wire wire;
                        TopoDS_Edge edge;
                        builder.MakeWire(wire);
                        builder.MakeEdge(edge);
                        builder.Add(wire, edge);
                        wire.Orientation(TopAbs_FORWARD);
                        payload.sourceShapes.front() = wire;
                    });
                checks[unit + "-foreign-owner-binding-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        auto& source = std::get<SourceNode>(payload.definition.nodes.front().value);
                        source.original.entity = spatial_sweep::debug::ID(201);
                    });
                checks[unit + "-foreign-path-binding-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        auto& feature = std::get<FeatureNode>(payload.definition.nodes.back().value);
                        spatial_sweep::Definition sweep;
                        if (spatial_sweep::Decode(feature.parameters, sweep)) {
                            sweep.path.inputNode = spatial_sweep::debug::ID(202);
                            (void)spatial_sweep::Encode(sweep, feature.parameters);
                        }
                    });
                checks[unit + "-foreign-digest-binding-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        auto& feature = std::get<FeatureNode>(payload.definition.nodes.back().value);
                        spatial_sweep::Definition sweep;
                        if (spatial_sweep::Decode(feature.parameters, sweep)) {
                            sweep.path.sourceRecipeDigest = spatial_sweep::debug::Hash(203);
                            (void)spatial_sweep::Encode(sweep, feature.parameters);
                        }
                    });
                checks[unit + "-extra-node-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.definition.nodes.push_back(payload.definition.nodes.back());
                    });
                checks[unit + "-uninstalled-feature-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        std::get<FeatureNode>(payload.definition.nodes.back().value).kind = 0xffffffffU;
                    });
                checks[unit + "-sycr1-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.definition.schemaVersion = 1;
                    });
                checks[unit + "-legacy-boolean-wire-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        payload.definition.schemaVersion = 1;
                        auto& feature = std::get<FeatureNode>(payload.definition.nodes.back().value);
                        feature.kind = PartBooleanFeatureKind;
                        feature.codecVersion = PartBooleanFeatureCodec;
                    });
                checks[unit + "-over-budget-wire-topology-refuses"] = !mutate(metersPerUnit,
                    [](const Handle(TDocStd_Document)&, Payload& payload) {
                        BRep_Builder builder;
                        TopoDS_Wire wire;
                        builder.MakeWire(wire);
                        for (unsigned index = 0; index < 4096; ++index) {
                            const double x = double(index);
                            const TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(
                                gp_Pnt(x, 0, 0), gp_Pnt(x + 0.5, 0, 0)).Edge();
                            builder.Add(wire, edge);
                        }
                        wire.Orientation(TopAbs_FORWARD);
                        payload.sourceShapes.front() = wire;
                    });
                checks[unit + "-shared-shape-second-occurrence-under-budget"] = mutate(
                    metersPerUnit, [](const Handle(TDocStd_Document)& document, Payload& payload) {
                        const Handle(XCAFDoc_ShapeTool) shapes =
                            XCAFDoc_DocumentTool::ShapeTool(document->Main());
                        if (!shapes.IsNull())
                            (void)shapes->AddShape(payload.sourceShapes.front(),
                                Standard_False, Standard_True);
                    });
            }

            const auto legacySolid = [&](bool invalid) {
                ValidationResult result;
                StandaloneDocument holder;
                const Handle(TDocStd_Document)& document = holder.document;
                const Definition definition = Probe::LegacyDefinition();
                std::vector<std::uint8_t> bytes;
                if (document.IsNull() || !EncodeV1(definition, bytes)) return result;
                XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
                const Handle(XCAFDoc_ShapeTool) shapes =
                    XCAFDoc_DocumentTool::ShapeTool(document->Main());
                if (shapes.IsNull()) return result;
                const TopoDS_Shape carrier = BRepPrimAPI_MakeBox(12, 10, 8).Shape();
                const TDF_Label owner = shapes->AddShape(
                    carrier, Standard_False, Standard_True);
                if (owner.IsNull()) return result;
                const auto setUUID = [](const TDF_Label& label,
                                        const char* id,
                                        const UUID& value) {
                    return !TDataStd_AsciiString::Set(label, Standard_GUID(id),
                        TCollection_AsciiString(retained_solid::UUIDText(value).c_str())).IsNull();
                };
                if (!setUUID(document->Main(),
                        "74386E4E-F620-498F-8092-E6D883AF33A4", definition.owner.document)
                    || !setUUID(owner,
                        "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", definition.owner.entity)
                    || !setUUID(owner,
                        "3611F2B2-C694-4E12-AED8-A2A97A3D283B", definition.owner.definition))
                    return result;
                TopoDS_Shape first = BRepPrimAPI_MakeBox(4, 4, 4).Shape();
                if (invalid) {
                    BRep_Builder builder;
                    TopoDS_Solid empty;
                    builder.MakeSolid(empty);
                    empty.Orientation(TopAbs_FORWARD);
                    first = empty;
                }
                auto payload = std::make_shared<Payload>();
                payload->definition = definition;
                payload->bytes = bytes;
                payload->sourceShapes = {first, BRepPrimAPI_MakeBox(3, 3, 3).Shape()};
                const TDF_Label record = owner.FindChild(MinimumRecordTag, Standard_True);
                Handle(Attribute) stored = new Attribute();
                if (record.IsNull() || stored.IsNull()) return result;
                record.AddAttribute(stored);
                stored->value_ = payload;
                TNaming_Builder(record).Select(carrier, carrier);
                result.setup = true;
                try {
                    result.accepted = validate(document);
                    result.evaluated = true;
                } catch (...) {}
                result.cleaned = holder.close();
                return result;
            };
            checks["legacy-solid-control-accepts"] = legacySolid(false);
            checks["open-or-invalid-solid-control-refuses"] = !legacySolid(true);
        } catch (...) {
            checks["probe-exception"] = false;
        }
        return checks;
    }
};
} // namespace core3d::composite_recipe
