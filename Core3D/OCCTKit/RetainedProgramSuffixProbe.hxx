#pragma once
#if DEBUG
// A3/P2 DEBUG evidence for the retained program suffix on a composite Boolean
// owner. No edit authority, no live document mutation outside the one staged
// fixture command, no production route. Public admission remains disabled.
#include "RetainedProgramSuffixBuild.hxx"
#include "PartBooleanCodecProbe.hxx"
#include "PartBooleanOwner.hxx"
#include "NativeDocumentSession.hxx"
#include "PartBooleanBuild.hxx"
#include <Prs3d_Drawer.hxx>
#include <TNaming_Builder.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <cstdio>
#ifdef __OBJC__
#import <Foundation/Foundation.h>
#endif

namespace core3d::retained_program_suffix {

struct Probe final {
    using Checks = std::map<std::string, bool>;
    static retained_recipe::UUID Identifier(std::uint8_t seed) noexcept {
        return composite_recipe::Probe::Identifier(seed);
    }
    static retained_recipe::Digest FixedDigest(std::uint8_t seed) noexcept {
        return composite_recipe::Probe::FixedDigest(seed);
    }

    // --- value fixtures -----------------------------------------------------
    static analytic_boolean::Operand Bore(std::uint32_t id, double x, double y, double radius) {
        analytic_boolean::Operand operand;
        operand.identifier = id;
        operand.kind = analytic_boolean::OperandKind::Cylinder;
        operand.extent = analytic_boolean::Extent::ThroughAll;
        operand.axis = analytic_boolean::Axis::Z;
        operand.point = {x, y, 0};
        operand.radius = radius;
        return operand;
    }
    static analytic_boolean::Operand Ring(std::uint32_t id, double cx, double cy,
                                          double bolt, double hole, unsigned count) {
        analytic_boolean::Operand operand;
        operand.identifier = id;
        operand.kind = analytic_boolean::OperandKind::CylinderRing;
        operand.extent = analytic_boolean::Extent::ThroughAll;
        operand.axis = analytic_boolean::Axis::Z;
        operand.point = {cx, cy, 5};
        operand.radius = hole;
        operand.boltCircleRadius = bolt;
        operand.count = count;
        operand.hostRadiusRatio = 8.0 / 15;
        return operand;
    }
    static analytic_boolean::Operand Wedge(std::uint32_t id, double x, double y) {
        analytic_boolean::Operand operand;
        operand.identifier = id;
        operand.kind = analytic_boolean::OperandKind::Wedge;
        operand.extent = analytic_boolean::Extent::ThroughAll;
        operand.axis = analytic_boolean::Axis::Z;
        operand.point = {x, y, 0};
        operand.directionAngle = 0;
        operand.halfWidthApex = 2;
        operand.halfWidthMouth = 4;
        operand.length = 20;
        return operand;
    }
    static Definition Program(const retained_recipe::UUID& base,
                              std::vector<analytic_boolean::Operand> operands,
                              bool fillet = false) {
        Definition value;
        value.baseFeature = base;
        value.derivedFeature = Identifier(0xC0);
        value.carrierMetersPerUnit = 0.001;
        value.nextOperandID = 1;
        for (auto& operand : operands) {
            retained_boolean::Step step;
            step.operation = analytic_boolean::Operation::Difference;
            step.operand = operand;
            value.steps.push_back(step);
            value.nextOperandID = value.nextOperandID <= operand.identifier
                ? std::uint64_t(operand.identifier) + 1 : value.nextOperandID;
        }
        if (fillet) {
            retained_fillet::Step round;
            round.stepIdentifier = 1;
            round.radiusLocal = 0.2;
            retained_fillet::EdgeAnchor anchor;
            anchor.identifier = 1;
            anchor.curveKind = retained_fillet::CurveKind::Circle;
            anchor.anchorPoint = {3.5, 5, 10};
            anchor.axis = {0, 0, 1};
            anchor.circleRadius = 1;
            round.anchors.push_back(anchor);
            value.filletSteps.push_back(std::move(round));
            value.nextFilletStepID = 2;
            value.nextFilletEdgeID = 2;
        }
        return value;
    }

    // Append a suffix feature node to an otherwise valid composite graph.
    static composite_recipe::Definition Suffixed(composite_recipe::Definition graph,
                                                 const Definition& program,
                                                 std::uint8_t seed) {
        composite_recipe::FeatureNode node;
        node.node = Identifier(seed);
        node.feature = Identifier(std::uint8_t(seed + 1));
        node.localID = graph.issuance.nextLocalID;
        node.kind = composite_recipe::RetainedProgramSuffixFeatureKind;
        node.codecVersion = composite_recipe::RetainedProgramSuffixFeatureCodec;
        node.inputs = {graph.outputNode};
        if (!Encode(program, node.parameters))
            throw std::invalid_argument("suffix fixture encode");
        graph.outputNode = node.node;
        ++graph.issuance.nextLocalID;
        graph.nodes.push_back({std::move(node)});
        return graph;
    }

    // One rectangular Profile source whose authored rectangle matches one
    // analytic prism fixture input, in identity placement.
    static composite_recipe::SourceNode RectSource(const std::array<double, 3>& origin,
                                                   const std::array<double, 3>& dimensions,
                                                   std::uint32_t slot, std::uint8_t seed,
                                                   const retained_recipe::UUID& document) {
        profile::Parameters profile;
        profile.metersPerUnit = 0.001;
        profile.definition.plane = 0;
        profile.definition.depth = dimensions[2];
        profile.definition.points = {
            gp_Pnt2d(origin[0], origin[1]),
            gp_Pnt2d(origin[0] + dimensions[0], origin[1]),
            gp_Pnt2d(origin[0] + dimensions[0], origin[1] + dimensions[1]),
            gp_Pnt2d(origin[0], origin[1] + dimensions[1]),
        };
        composite_recipe::SourceNode source;
        source.node = Identifier(seed);
        source.localID = std::uint64_t(slot) + 1;
        source.original.document = document;
        source.original.entity = Identifier(std::uint8_t(seed + 1));
        source.original.definition = Identifier(std::uint8_t(seed + 2));
        source.original.sourceFeature = Identifier(std::uint8_t(seed + 3));
        source.recipe.kind = composite_recipe::RecipeKind::Profile;
        source.recipe.schema = 1;
        std::vector<double> values;
        if (!profile::Encode(profile, values)
            || !composite_recipe::EncodeScalarRecipe(source.recipe.kind, 1, values,
                source.recipe.bytes))
            throw std::invalid_argument("rect source encode");
        source.inputToCarrier.sourceMetersPerUnit = 0.001;
        source.inputToCarrier.carrierMetersPerUnit = 0.001;
        source.commitments.geometry = FixedDigest(std::uint8_t(seed + 4));
        if (!composite_recipe::Hash(source.recipe.bytes, source.commitments.recipe))
            throw std::invalid_argument("rect source digest");
        source.commitments.placement = FixedDigest(std::uint8_t(seed + 5));
        source.commitments.material = FixedDigest(std::uint8_t(seed + 6));
        source.commitments.groups = FixedDigest(std::uint8_t(seed + 7));
        source.shapeSlot = slot;
        return source;
    }

    static composite_recipe::Definition AnalyticGraph(part_boolean::AnalyticDefinition& analytic) {
        analytic = part_boolean::AnalyticDefinition();
        analytic.operation = part_boolean::Operation::Union;
        composite_recipe::Definition graph;
        graph.schemaVersion = 3;
        graph.owner = {Identifier(1), Identifier(2), Identifier(3)};
        composite_recipe::SourceNode left = RectSource({0, 0, 0}, {10, 10, 10}, 0, 10, graph.owner.document);
        composite_recipe::SourceNode right = RectSource({5, 0, 0}, {10, 10, 10}, 1, 30, graph.owner.document);
        for (std::size_t index = 0; index < 2; ++index) {
            auto& input = analytic.inputs[index];
            const auto& source = index == 0 ? left : right;
            input.rootNode = source.node;
            input.originalSourceFeature = source.original.sourceFeature;
            input.dimensions = index == 0
                ? std::array<double, 3>{{10, 10, 10}} : std::array<double, 3>{{10, 10, 10}};
            input.translation = index == 0
                ? std::array<double, 3>{{0, 0, 0}} : std::array<double, 3>{{5, 0, 0}};
            input.metersPerUnit = 0.001;
            input.commitments.geometry = source.commitments.geometry;
            input.commitments.recipe = source.commitments.recipe;
            input.commitments.placement = source.commitments.placement;
            input.commitments.material = source.commitments.material;
            input.commitments.groups = source.commitments.groups;
            input.originalMaterial = composite_recipe::Probe::Material(std::uint8_t(90 + index));
        }
        composite_recipe::FeatureNode feature;
        feature.node = Identifier(50);
        feature.feature = Identifier(51);
        feature.localID = 3;
        feature.kind = composite_recipe::PartBooleanFeatureKind;
        feature.codecVersion = composite_recipe::PartBooleanFeatureCodec;
        feature.inputs = {left.node, right.node};
        if (!part_boolean::EncodeAnalytic(analytic, feature.parameters))
            throw std::invalid_argument("analytic fixture encode");
        graph.outputNode = feature.node;
        graph.issuance.nextLocalID = 4;
        graph.nodes = {{std::move(left)}, {std::move(right)}, {std::move(feature)}};
        return graph;
    }

    static retained_recipe::RevisionFence Fence(const composite_recipe::Definition& graph) {
        retained_recipe::RevisionFence fence;
        fence.documentGeneration = 1;
        fence.modelRevision = 1;
        fence.effectiveMetersPerUnit = 0.001;
        fence.ownerShape = FixedDigest(201);
        fence.ownerRecipe = FixedDigest(202);
        fence.ownerPlacement = FixedDigest(203);
        fence.ownerMaterial = FixedDigest(204);
        for (const composite_recipe::Node& node : graph.nodes) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            retained_recipe::DependencyRead read;
            read.locator = {graph.owner, source->node, source->original.sourceFeature};
            read.geometry = source->commitments.geometry;
            read.recipe = source->commitments.recipe;
            read.placement = source->commitments.placement;
            read.material = source->commitments.material;
            read.groups = source->commitments.groups;
            fence.dependencies.push_back(read);
        }
        return fence;
    }

    static cut_display::Settings Display() {
        cut_display::Settings display;
        Handle(Prs3d_Drawer) drawer = new Prs3d_Drawer();
        if (!cut_display::Capture(drawer, display))
            throw std::invalid_argument("display capture");
        return display;
    }

    // --- scenario 0: SYPS/1 codec and composite carrier ----------------------
    static Checks CodecEvidence() noexcept {
        Checks checks;
        try {
            const retained_recipe::UUID base = Identifier(60);
            const Definition program = Program(base, {Bore(1, 30, 15, 2), Ring(2, 3, 4, 80, 2, 6)}, true);
            std::vector<std::uint8_t> bytes, reencoded;
            Definition decoded;
            checks["syps1-exact-roundtrip"] = Encode(program, bytes)
                && Decode(bytes, decoded) && Encode(decoded, reencoded)
                && reencoded == bytes && decoded.steps.size() == 2
                && decoded.filletSteps.size() == 1;
            auto corrupt = bytes; corrupt[corrupt.size() - 33] ^= 1;
            checks["syps1-digest-corruption-refused"] = !Decode(corrupt, decoded);
            corrupt = bytes; corrupt.push_back(0);
            checks["syps1-trailing-byte-refused"] = !Decode(corrupt, decoded);
            corrupt = bytes; corrupt[4] = 9;
            checks["syps1-unknown-version-refused"] = !Decode(corrupt, decoded);
            corrupt = bytes; corrupt[5] = 1;
            checks["syps1-reserved-byte-refused"] = !Decode(corrupt, decoded);
            Definition invalid = program; invalid.steps.clear();
            checks["syps1-empty-steps-refused"] = !Valid(invalid) && !Encode(invalid, corrupt);
            invalid = program; invalid.steps[1].operand.identifier = 1;
            checks["syps1-duplicate-operand-refused"] = !Valid(invalid);
            invalid = program; invalid.nextOperandID = 2;
            checks["syps1-high-water-refused"] = !Valid(invalid);
            invalid = program;
            invalid.steps.push_back({analytic_boolean::Operation::Difference, Bore(3, 1, 1, 1)});
            invalid.steps.push_back({analytic_boolean::Operation::Difference, Bore(4, 2, 1, 1)});
            invalid.steps.push_back({analytic_boolean::Operation::Difference, Bore(5, 3, 1, 1)});
            invalid.nextOperandID = 6;
            checks["syps1-fifth-step-refused"] = !Valid(invalid);
            invalid = program;
            invalid.steps = {{analytic_boolean::Operation::Difference, Ring(1, 3, 4, 80, 2, 16)},
                             {analytic_boolean::Operation::Difference, Ring(2, 3, 4, 80, 2, 16)},
                             {analytic_boolean::Operation::Difference, Ring(3, 3, 4, 80, 2, 16)}};
            invalid.nextOperandID = 4;
            std::vector<retained_boolean::Disk> sections;
            checks["syps1-33-sections-refused"] = !ExpandedSections(invalid, sections);

            // The composite carrier accepts the suffix node on both base
            // families and preserves every legacy byte scope.
            const composite_recipe::Definition legacy = composite_recipe::Probe::LegacyDefinition();
            const composite_recipe::Definition shell = composite_recipe::Probe::ShellDefinition();
            std::vector<std::uint8_t> legacyBytes, shellBytes;
            if (!composite_recipe::EncodeV1(legacy, legacyBytes)
                || !composite_recipe::EncodeV2(shell, shellBytes))
                throw std::invalid_argument("base graph encode");
            composite_recipe::Definition registeredShell = shell;
            registeredShell.schemaVersion = 3;
            const Definition onShell = Program(registeredShell.outputNode, {Bore(1, 30, 15, 2)});
            const composite_recipe::Definition suffixedShell = Suffixed(registeredShell, onShell, 0x70);
            std::vector<std::uint8_t> suffixedBytes, resaved;
            composite_recipe::Definition reopened;
            checks["composite-suffix-graph-accepted"] = composite_recipe::Valid(suffixedShell)
                && composite_recipe::EncodeV3(suffixedShell, suffixedBytes);
            checks["composite-suffix-roundtrip-byte-exact"] =
                composite_recipe::Decode(suffixedBytes, reopened)
                && composite_recipe::Encode(reopened, resaved) && resaved == suffixedBytes
                && reopened.outputNode == suffixedShell.outputNode
                && reopened.nodes.size() == 4;
            part_boolean::AnalyticDefinition analytic;
            const composite_recipe::Definition registeredAnalytic = AnalyticGraph(analytic);
            const Definition onAnalytic = Program(registeredAnalytic.outputNode, {Bore(1, 30, 15, 2)});
            checks["composite-suffix-sycr3-accepted"] =
                composite_recipe::Valid(Suffixed(registeredAnalytic, onAnalytic, 0x80));
            composite_recipe::Definition tampered = suffixedShell;
            std::get<composite_recipe::FeatureNode>(tampered.nodes.back().value).inputs = {
                composite_recipe::NodeID(suffixedShell.nodes.front())};
            checks["suffix-input-source-refused"] = !composite_recipe::Valid(tampered);
            tampered = Suffixed(suffixedShell, Program(suffixedShell.outputNode,
                {Bore(3, 1, 1, 1)}), 0x90);
            checks["suffix-chaining-refused"] = !composite_recipe::Valid(tampered);
            Definition wrongBinding = onShell;
            wrongBinding.baseFeature = Identifier(0x99);
            tampered = suffixedShell;
            auto& node = std::get<composite_recipe::FeatureNode>(tampered.nodes.back().value);
            checks["suffix-base-binding-refused"] = Encode(wrongBinding, node.parameters)
                && !composite_recipe::Valid(tampered);
            tampered = suffixedShell;
            std::get<composite_recipe::FeatureNode>(tampered.nodes.back().value).kind = 99;
            checks["suffix-unknown-kind-refused"] = !composite_recipe::Valid(tampered);
            tampered = suffixedShell;
            std::get<composite_recipe::FeatureNode>(tampered.nodes.back().value).kind = 3;
            checks["raw-kind-3-refused"] = !composite_recipe::Valid(tampered);
            checks["family-banded-registry-allocation-used"] =
                retained_program_suffix::FeatureKind
                    == retained_feature::RetainedProgramSuffixKind
                && retained_program_suffix::FeatureKind
                    == retained_feature::ReservedD67FeatureKind1001
                && retained_feature::ProductionRegistry().find(
                    {retained_program_suffix::FeatureKind,
                     retained_program_suffix::FeatureCodec}) != nullptr;
            std::vector<std::uint8_t> legacyAgain, shellAgain;
            checks["legacy-sycr-byte-identical"] = composite_recipe::EncodeV1(legacy, legacyAgain)
                && composite_recipe::EncodeV2(shell, shellAgain)
                && legacyAgain == legacyBytes && shellAgain == shellBytes;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    // --- scenario 1: typed admission refusals --------------------------------
    static Checks AdmissionEvidence() noexcept {
        Checks checks;
        try {
            composite_recipe::Definition graph = composite_recipe::Probe::ShellDefinition();
            graph.schemaVersion = 3;
            Request request;
            request.graph = graph;
            request.fence = Fence(graph);
            request.program = Program(graph.outputNode, {Bore(1, 30, 15, 2)});
            const Decision admitted = Evaluate(request);
            checks["route-switch-off"] =
                !part_boolean::family_admission::RetainedProgramSuffixInstalled
                && !part_boolean::family_admission::RetainedProgramSuffixRouteInstalled;
            checks["admission-disabled-typed"] =
                admitted.refusal == Refusal::AdmissionDisabled && !admitted.admitted()
                && admitted.completeReadSet.size() == 2;
            Request stale = request;
            stale.fence.dependencies.front().geometry[0] ^= 0xff;
            checks["stale-fence-typed"] = Evaluate(stale).refusal == Refusal::StaleFence;
            Request foreign = request;
            foreign.fence.dependencies.front().locator.owner.entity = Identifier(0xFE);
            checks["foreign-owner-ambiguous"] = Evaluate(foreign).refusal == Refusal::OwnerAmbiguous;
            Request oversized = request;
            oversized.program = Program(graph.outputNode, {Ring(1, 3, 4, 80, 2, 16),
                Ring(2, 3, 4, 80, 2, 16), Ring(3, 3, 4, 80, 2, 16)});
            checks["oversized-typed"] = Evaluate(oversized).refusal == Refusal::ProgramOversized;
            Request invalid = request;
            invalid.program.nextOperandID = 1;
            checks["invalid-program-typed"] = Evaluate(invalid).refusal == Refusal::ProgramInvalid;
            Request mismatched = request;
            mismatched.program.baseFeature = Identifier(0x99);
            checks["base-mismatch-typed"] = Evaluate(mismatched).refusal == Refusal::BaseFeatureMismatch;
            Request suffixed = request;
            suffixed.graph = Suffixed(graph, request.program, 0x70);
            suffixed.program.baseFeature = suffixed.graph.outputNode;
            checks["suffixed-owner-refused"] =
                Evaluate(suffixed).refusal == Refusal::UnsupportedBaseFamily;
            Request malformed = request;
            malformed.graph.outputNode = Identifier(0x97);
            checks["malformed-owner-refused"] = Evaluate(malformed).refusal == Refusal::OwnerMalformed;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    // --- scenario 2: real document, one staged command, cold reopen ----------
    // Installs the N1 analytic owner fixture, appends one retained bore suffix
    // through the composite envelope, and proves the persisted program bytes
    // survive Undo/Redo and two cold save/open cycles in both unit systems,
    // with a cache-free replay of the reopened graph matching the persisted
    // result. The production append route remains disabled; this stages through
    // the DEBUG attribute seam only.
    static Checks ColdReopenUnit(double unit, bool& ok) noexcept {
        Checks checks;
        ok = false;
        std::vector<Handle(OcctDocument)> opened;
        const auto trace = [&](const char* phase) noexcept {
            std::fprintf(stderr, "[retained-program-suffix] cold-reopen unit=%.17g phase=%s\n",
                unit, phase);
            std::fflush(stderr);
        };
        trace("begin");
        try {
            NativeDocumentSession native;
            const Handle(OcctDocument) wrapper = native.Document();
            if (wrapper.IsNull() || wrapper->Document().IsNull()) {
                trace("null-document"); return checks;
            }
            XCAFDoc_DocumentTool::SetLengthUnit(wrapper->Document(), unit);
            part_boolean::owner::PartBooleanOwner* owner = wrapper->PartBooleanOwnerService();
            TDF_Label carrier;
            if (!owner || !owner->installEvidenceFixture(
                    part_boolean::Operation::Subtract, unit, carrier)) {
                trace("fixture-install"); return checks;
            }
            composite_recipe::Record record;
            if (!composite_recipe::Read(wrapper->Document(), carrier, record)
                || !record.value) {
                trace("record-read"); return checks;
            }
            const composite_recipe::Definition graph = record.value->definition;
            composite_recipe::Definition registryGraph = graph;
            registryGraph.schemaVersion = 3;
            const auto* baseFeature =
                std::get_if<composite_recipe::FeatureNode>(&graph.nodes.back().value);
            part_boolean::AnalyticDefinition analytic;
            if (!baseFeature || !part_boolean::DecodeAnalytic(baseFeature->parameters, analytic)) {
                trace("base-decode"); return checks;
            }
            // Place one bore inside the retained left input, clear of the
            // subtracted right input, computed from the persisted recipes.
            const auto& left = analytic.inputs[0];
            const auto& right = analytic.inputs[1];
            const double scale = left.metersPerUnit / unit;
            const double leftMid = left.translation[0] + left.dimensions[0] / 2;
            const double rightMid = right.translation[0] + right.dimensions[0] / 2;
            const double boreX = (rightMid >= leftMid
                ? left.translation[0] + left.dimensions[0] / 4
                : left.translation[0] + 3 * left.dimensions[0] / 4) * scale;
            const double boreY = (left.translation[1] + left.dimensions[1] / 2) * scale;
            const double radius = std::min({left.dimensions[0], left.dimensions[1],
                left.dimensions[2]}) / 8 * scale;
            Definition program = Program(graph.outputNode, {Bore(1, boreX, boreY, radius)});
            program.carrierMetersPerUnit = unit;
            program.derivedFeature = Identifier(0xB0);
            const std::atomic_bool stop(false);
            build::Budget budget;
            const cut_display::Settings display = Display();
            const auto built = build::Build(record.current, registryGraph, program, display, stop, budget);
            if (built.status != build::Status::Built) {
                trace("initial-build"); return checks;
            }

            composite_recipe::Definition staged = Suffixed(registryGraph, program, 0xA0);
            std::vector<std::uint8_t> stagedBytes;
            if (!composite_recipe::Encode(staged, stagedBytes)) {
                trace("staged-encode"); return checks;
            }
            auto payload = std::make_shared<composite_recipe::Payload>();
            payload->definition = staged;
            payload->bytes = stagedBytes;
            payload->sourceShapes = record.value->sourceShapes;
            wrapper->Document()->NewCommand();
            XCAFDoc_DocumentTool::ShapeTool(wrapper->Document()->Main())
                ->SetShape(carrier, built.solid);
            TNaming_Builder binding(record.label);
            binding.Select(built.solid, built.solid);
            const bool stagedOk = composite_recipe::Probe::ReplaceStored(record.label, payload);
            if (!stagedOk || !wrapper->Document()->CommitCommand()) {
                trace("staged-commit"); return checks;
            }
            checks["suffix-staged-one-command"] = true;

            composite_recipe::Record observed;
            checks["undo-restores-original-envelope"] = wrapper->undo()
                && composite_recipe::Read(wrapper->Document(), carrier, observed)
                && observed.value && observed.value->bytes == record.value->bytes;
            checks["redo-restores-suffix-envelope"] = wrapper->redo()
                && composite_recipe::Read(wrapper->Document(), carrier, observed)
                && observed.value && observed.value->bytes == stagedBytes;
            if (!checks["undo-restores-original-envelope"]
                || !checks["redo-restores-suffix-envelope"]) {
                trace("undo-redo"); return checks;
            }

            const auto uniqueBase = [](const char* suffix) {
                NSString* name = [NSString stringWithFormat:@"a3-p2-suffix-%@-%s.fixture",
                    [[NSUUID UUID] UUIDString], suffix];
                return std::string([[NSTemporaryDirectory()
                    stringByAppendingPathComponent:name] UTF8String]);
            };
            const std::string saved = wrapper->save(uniqueBase("first"));
            if (saved.empty()) { trace("first-save"); return checks; }

            Handle(OcctDocument) fresh = new OcctDocument();
            Core3DDefineSafeBinXCAFFormat(fresh->myApp);
            Handle(TDocStd_Document) candidate;
            Core3DBeginSafeBinaryRead();
            const PCDM_ReaderStatus status = fresh->myApp->Open(
                TCollection_ExtendedString(saved.c_str(), Standard_True), candidate);
            if (Core3DSafeBinaryReadWasRejected() || status != PCDM_RS_OK || candidate.IsNull())
                { trace("first-cold-open"); return checks; }
            fresh->myOcafDoc = candidate;
            fresh->ObserveSuccessfulNativeDocumentAdoption();
            candidate->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
            opened.push_back(fresh);
            std::vector<composite_recipe::Record> records;
            if (!composite_recipe::ReadAll(candidate, records) || records.size() != 1)
                { trace("first-cold-read"); return checks; }
            const composite_recipe::Record& cold = records.front();
            checks["cold-reopen-envelope-byte-identical"] = cold.value
                && cold.value->bytes == stagedBytes;
            const auto* coldSuffix =
                std::get_if<composite_recipe::FeatureNode>(&cold.value->definition.nodes.back().value);
            std::vector<std::uint8_t> payloadAgain;
            checks["cold-reopen-suffix-payload-identical"] = coldSuffix
                && coldSuffix->kind == composite_recipe::RetainedProgramSuffixFeatureKind
                && Encode(program, payloadAgain) && coldSuffix->parameters == payloadAgain;
            bool sourcesIntact = cold.value->definition.nodes.size() == 4;
            for (std::size_t index = 0; index + 1 < graph.nodes.size() && sourcesIntact; ++index) {
                const auto* before = std::get_if<composite_recipe::SourceNode>(
                    &graph.nodes[index].value);
                const auto* after = std::get_if<composite_recipe::SourceNode>(
                    &cold.value->definition.nodes[index].value);
                sourcesIntact = before && after && before->recipe.bytes == after->recipe.bytes
                    && before->original.entity == after->original.entity
                    && before->original.sourceFeature == after->original.sourceFeature;
            }
            checks["cold-reopen-both-source-recipes-intact"] = sourcesIntact;

            // Cache-free replay: rebuild the prefix from the reopened graph's
            // authored values, replay the suffix, compare exact result bytes.
            const auto& coldGraph = cold.value->definition;
            part_boolean::AnalyticDefinition coldAnalytic;
            const auto* coldBase = std::get_if<composite_recipe::FeatureNode>(
                &coldGraph.nodes[2].value);
            retained_part_boolean::OperandReadSet reads;
            bool readsOk = coldBase
                && part_boolean::DecodeAnalytic(coldBase->parameters, coldAnalytic);
            retained_recipe::DependencyRead* slots[] = {&reads.leftSource, &reads.rightSource};
            for (std::size_t index = 0; index < 2 && readsOk; ++index) {
                const auto* source = std::get_if<composite_recipe::SourceNode>(
                    &coldGraph.nodes[index].value);
                if (!source) { readsOk = false; break; }
                slots[index]->locator = {coldGraph.owner, source->node,
                    source->original.sourceFeature};
                slots[index]->geometry = source->commitments.geometry;
                slots[index]->recipe = source->commitments.recipe;
                slots[index]->placement = source->commitments.placement;
                slots[index]->material = source->commitments.material;
                slots[index]->groups = source->commitments.groups;
            }
            if (!readsOk) { trace("replay-read-set"); return checks; }
            const auto prefix = part_boolean::build::BuildAnalytic(coldAnalytic, unit, reads, reads);
            if (!prefix.complete) { trace("prefix-replay"); return checks; }
            composite_recipe::Definition prefixGraph = coldGraph;
            prefixGraph.nodes.pop_back();
            prefixGraph.outputNode = coldBase->node;
            build::Budget coldBudget;
            const auto replayed = build::Build(prefix.candidate.solid, prefixGraph,
                program, display, stop, coldBudget);
            build::Budget readbackBudget;
            TopoDS_Shape readbackShape;
            std::string replayBytes, persistedBytes, persistedReadback;
            checks["cold-replay-matches-persisted-result"] =
                replayed.status == build::Status::Built
                && retained_part_boolean::ExactShapeBytes(replayed.solid, replayBytes)
                && retained_part_boolean::ExactShapeBytes(cold.current, persistedBytes)
                && (replayBytes == persistedBytes
                    || (saved_boolean_build::ReadbackGeometry(cold.current, stop,
                            readbackBudget, readbackShape)
                        && retained_part_boolean::ExactShapeBytes(readbackShape, persistedReadback)
                        && replayBytes == persistedReadback));

            // Both consumed inputs remain native-editable after reopen. Each
            // edit rebuilds the Boolean prefix and replays the complete suffix;
            // the first edit also crosses Undo/Redo before the second input is
            // edited, proving the dependent program is not retired.
            part_boolean::owner::PartBooleanOwner* coldOwner = fresh->PartBooleanOwnerService();
            const auto editInput = [&](std::size_t index, double delta) {
                if (!coldOwner || index > 1) return false;
                const auto captured = coldOwner->capture(cold.owner);
                if (!captured.handle
                    || captured.receipt.outcome != part_boolean::owner::Outcome::Unchanged) {
                    std::fprintf(stderr, "[retained-program-suffix] cold-reopen unit=%.17g "
                        "input=%zu undo-limit=%d capture-refusal=%s\n", unit, index,
                        static_cast<int>(candidate->GetUndoLimit()),
                        captured.receipt.reason.c_str());
                    return false;
                }
                part_boolean::AnalyticDefinition edited = captured.handle->debugAnalytic();
                edited.inputs[index].dimensions[0] += delta;
                const auto prepared = coldOwner->prepare(captured.handle, edited);
                if (!prepared.handle
                    || prepared.receipt.outcome != part_boolean::owner::Outcome::Unchanged) {
                    std::fprintf(stderr, "[retained-program-suffix] cold-reopen unit=%.17g "
                        "input=%zu undo-limit=%d prepare-refusal=%s\n", unit, index,
                        static_cast<int>(candidate->GetUndoLimit()),
                        prepared.receipt.reason.c_str());
                    return false;
                }
                const auto applied = coldOwner->apply(prepared.handle);
                if (applied.outcome != part_boolean::owner::Outcome::Committed) {
                    std::fprintf(stderr, "[retained-program-suffix] cold-reopen unit=%.17g "
                        "input=%zu undo-limit=%d apply-refusal=%s\n", unit, index,
                        static_cast<int>(candidate->GetUndoLimit()),
                        applied.reason.c_str());
                }
                return applied.outcome == part_boolean::owner::Outcome::Committed;
            };
            checks["cold-reopen-left-input-edit-replays-suffix"] = editInput(0, 2.0);
            checks["left-edit-undo-restores-suffix"] = checks["cold-reopen-left-input-edit-replays-suffix"]
                && fresh->undo();
            checks["left-edit-redo-restores-suffix"] = checks["left-edit-undo-restores-suffix"]
                && fresh->redo();
            checks["cold-reopen-right-input-edit-replays-suffix"] =
                checks["left-edit-redo-restores-suffix"] && editInput(1, 1.0);
            composite_recipe::Record editedRecord;
            if (!checks["cold-reopen-right-input-edit-replays-suffix"]
                || !composite_recipe::Read(candidate, cold.owner, editedRecord)
                || !editedRecord.value) {
                trace("owner-edit-replay"); return checks;
            }
            const std::vector<std::uint8_t> editedBytes = editedRecord.value->bytes;
            const auto* editedSuffix = std::get_if<composite_recipe::FeatureNode>(
                &editedRecord.value->definition.nodes.back().value);
            checks["both-input-identities-and-suffix-feature-stable"] = editedSuffix
                && editedSuffix->node == coldSuffix->node
                && editedSuffix->feature == coldSuffix->feature
                && editedSuffix->parameters == coldSuffix->parameters
                && editedRecord.value->definition.nodes[0].value.index()
                    == cold.value->definition.nodes[0].value.index()
                && editedRecord.value->definition.nodes[1].value.index()
                    == cold.value->definition.nodes[1].value.index()
                && std::get<composite_recipe::SourceNode>(
                    editedRecord.value->definition.nodes[0].value).original.entity
                    == std::get<composite_recipe::SourceNode>(
                        cold.value->definition.nodes[0].value).original.entity
                && std::get<composite_recipe::SourceNode>(
                    editedRecord.value->definition.nodes[1].value).original.entity
                    == std::get<composite_recipe::SourceNode>(
                        cold.value->definition.nodes[1].value).original.entity;

            const std::string secondSaved = fresh->save(uniqueBase("second"));
            Handle(OcctDocument) third = new OcctDocument();
            Core3DDefineSafeBinXCAFFormat(third->myApp);
            Handle(TDocStd_Document) thirdDocument;
            Core3DBeginSafeBinaryRead();
            const PCDM_ReaderStatus thirdStatus = third->myApp->Open(
                TCollection_ExtendedString(secondSaved.c_str(), Standard_True), thirdDocument);
            std::vector<composite_recipe::Record> thirdRecords;
            checks["second-cold-open-stable"] = !secondSaved.empty()
                && !Core3DSafeBinaryReadWasRejected() && thirdStatus == PCDM_RS_OK
                && !thirdDocument.IsNull()
                && composite_recipe::ReadAll(thirdDocument, thirdRecords)
                && thirdRecords.size() == 1 && thirdRecords.front().value
                && thirdRecords.front().value->bytes == editedBytes;
            if (!thirdDocument.IsNull()) {
                third->myOcafDoc = thirdDocument;
                third->ObserveSuccessfulNativeDocumentAdoption();
                opened.push_back(third);
            }
            ok = true;
            for (const Handle(OcctDocument)& document : opened)
                if (!document.IsNull()) document->CloseNativeSession();
            trace("complete");
            return checks;
        } catch (...) {
            for (const Handle(OcctDocument)& document : opened)
                if (!document.IsNull()) document->CloseNativeSession();
            trace("exception");
            return checks;
        }
    }

    static Checks ColdReopenEvidence() noexcept {
        Checks checks;
        bool millimetres = false, metres = false;
        const Checks mm = ColdReopenUnit(0.001, millimetres);
        const Checks m = ColdReopenUnit(1.0, metres);
        for (const auto& row : mm) checks["mm-" + row.first] = row.second;
        for (const auto& row : m) checks["m-" + row.first] = row.second;
        checks["both-unit-systems"] = millimetres && metres;
        return checks;
    }

    // --- scenario 3: detached geometry proof on a proven analytic prefix -----
    static Checks GeometryEvidence() noexcept {
        Checks checks;
        try {
            part_boolean::AnalyticDefinition analytic;
            const composite_recipe::Definition graph = AnalyticGraph(analytic);
            if (!composite_recipe::Valid(graph))
                throw std::invalid_argument("analytic graph fixture");
            const auto reads = part_boolean::build::FixtureReads();
            const auto first = part_boolean::build::BuildAnalytic(analytic, 0.001, reads, reads);
            const auto second = part_boolean::build::BuildAnalytic(analytic, 0.001, reads, reads);
            checks["prefix-builds-admitted"] = first.complete && second.complete;
            if (!checks["prefix-builds-admitted"]) return checks;
            const cut_display::Settings display = Display();
            const std::atomic_bool stop(false);

            const Definition bore = Program(graph.outputNode, {Bore(1, 2.5, 5, 1)});
            const auto boreBuilt = build::Build(first.candidate.solid, graph, bore, display, stop);
            checks["bore-suffix-built"] = boreBuilt.status == build::Status::Built;
            checks["bore-exact-removed-volume"] = boreBuilt.status == build::Status::Built
                && std::abs(boreBuilt.removedVolume - 10 * std::acos(-1))
                    <= 10 * std::acos(-1) * 1e-9;
            checks["bore-correspondence-matched"] = boreBuilt.correspondence.classification
                == saved_boolean_result::Classification::MatchedOrientedBoundary;

            const Definition wedge = Program(graph.outputNode, {Wedge(1, 2.5, 5)});
            checks["wedge-suffix-built"] =
                build::Build(first.candidate.solid, graph, wedge, display, stop).status
                    == build::Status::Built;

            const Definition ring = Program(graph.outputNode, {Ring(1, 2.5, 5, 1.5, 0.5, 4)});
            checks["ring-suffix-built"] =
                build::Build(first.candidate.solid, graph, ring, display, stop).status
                    == build::Status::Built;
            checks["ring-ligament-certificate-clear"] = build::LigamentCertificate(
                graph, ring.steps.front().operand, 0.001) == build::Ligament::Clear;
            const Definition tightRing = Program(graph.outputNode, {Ring(1, 2.5, 5, 2.4, 0.5, 4)});
            build::Budget tightBudget;
            checks["ring-ligament-refusal-typed"] = build::LigamentCertificate(
                graph, tightRing.steps.front().operand, 0.001) == build::Ligament::InsufficientLigament
                && build::Build(first.candidate.solid, graph, tightRing, display, stop,
                    tightBudget).status == build::Status::Refused;

            // Shell-host certificate posture: a wall-transverse ring is
            // unproven, and a ring whose annulus meets the subtracted tool is
            // a typed intersection refusal.
            const composite_recipe::Definition shellGraph = composite_recipe::Probe::ShellDefinition();
            const Definition wallRing = Program(shellGraph.outputNode, {Ring(1, 20, 15, 5, 1, 4)});
            auto transverse = wallRing.steps.front().operand;
            transverse.axis = analytic_boolean::Axis::X;
            checks["shell-wall-ring-unproven"] = build::LigamentCertificate(
                shellGraph, transverse, 0.001) == build::Ligament::HostExtentUnproven;
            checks["shell-overlap-ring-tool-intersection"] = build::LigamentCertificate(
                shellGraph, wallRing.steps.front().operand, 0.001)
                == build::Ligament::ToolIntersection;

            const Definition rounded = Program(graph.outputNode, {Bore(1, 2.5, 5, 1)}, true);
            const auto roundedBuilt = build::Build(first.candidate.solid, graph, rounded,
                display, stop);
            checks["fillet-suffix-built"] = roundedBuilt.status == build::Status::Built
                && roundedBuilt.filletOutcome == retained_fillet::Outcome::Built;

            build::Budget uncutBudget;
            checks["uncut-candidate-refused"] = build::InspectSuffix(first.candidate.solid,
                first.candidate.solid, bore, display, stop, uncutBudget).classification
                == saved_boolean_result::Classification::Refused;
            const auto fixed = build::CheckFixedPoint(first.candidate.solid,
                second.candidate.solid, graph, bore, display, stop);
            checks["suffix-fixed-point"] = fixed.fixedPoint();
            checks["payload-byte-exact-both-builds"] = fixed.payloadsExact;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks Run(int scenario) noexcept {
        if (scenario == 0) return CodecEvidence();
        if (scenario == 1) return AdmissionEvidence();
        if (scenario == 2) return ColdReopenEvidence();
        if (scenario == 3) return GeometryEvidence();
        return {{"valid-scenario", false}};
    }
};

} // namespace core3d::retained_program_suffix
#endif
