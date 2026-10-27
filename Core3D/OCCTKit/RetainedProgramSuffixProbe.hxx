#pragma once
#if DEBUG
// A3/P2 DEBUG evidence for the retained program suffix on a composite Boolean
// owner. No edit authority, no live document mutation outside the one staged
// fixture command, no production route. Public admission remains disabled.
#include "RetainedProgramSuffixBuild.hxx"
#include "PartBooleanRebuild.hxx"
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

    static bool RefreshProgramOperand(part_boolean::program_operand::Value& value,
                                      const retained_boolean::Recipe& recipe) {
        using namespace part_boolean::program_operand;
        std::vector<std::uint8_t> bytes;
        retained_recipe::Digest digest;
        if (!retained_boolean::Encode(recipe, bytes)
            || !composite_recipe::Hash(bytes, digest)) return false;
        value.recipe.originalBytes = bytes;
        value.recipe.effectiveBytes = bytes;
        value.recipe.originalDigest = digest;
        value.recipe.effectiveDigest = digest;
        value.recipe.originalDecoded = recipe;
        value.recipe.effectiveDecoded = recipe;
        value.originalCommitments.recipe = digest;
        value.effectiveCommitments.recipe = digest;
        for (auto& read : value.fence.revision.dependencies)
            if (read.locator.node == value.identity.sourceNode) read.recipe = digest;
        return true;
    }

    // Complete values only: this fixture constructs no shape and performs no
    // kernel call. Lengths are authored in the requested document unit.
    static part_boolean::program_operand::Value ProgramOperandPrismFixture(
        double unit, part_boolean::program_operand::Role role =
            part_boolean::program_operand::Role::Left, std::uint8_t seed = 0x20) {
        using namespace part_boolean::program_operand;
        if (unit != 0.001 && unit != 1.0) throw std::invalid_argument("program operand unit");
        const double localPerMM = 0.001 / unit;
        const retained_recipe::UUID document = Identifier(0x10);

        profile::Parameters profile;
        profile.metersPerUnit = unit;
        profile.definition.plane = 0;
        profile.definition.depth = 20 * localPerMM;
        profile.definition.points = {
            {0, 0}, {40 * localPerMM, 0},
            {40 * localPerMM, 30 * localPerMM}, {0, 30 * localPerMM},
        };
        std::vector<double> sourceValues;
        if (!profile::Encode(profile, sourceValues))
            throw std::invalid_argument("program operand profile");

        retained_boolean::Program program;
        program.source.document = document;
        program.source.entity = Identifier(seed);
        program.source.definition = Identifier(std::uint8_t(seed + 1));
        program.source.sourceFeature = Identifier(std::uint8_t(seed + 2));
        program.source.derivedFeature = Identifier(std::uint8_t(seed + 3));
        program.source.family = 1;
        program.source.schema = 1;
        program.source.metersPerUnit = unit;
        program.source.values = std::move(sourceValues);
        retained_boolean::Step step;
        step.operation = analytic_boolean::Operation::Difference;
        step.operand = Bore(1, 10 * localPerMM, 15 * localPerMM, 2 * localPerMM);
        program.steps.push_back(step);
        program.nextOperandID = 2;
        if (!retained_boolean::Valid(program))
            throw std::invalid_argument("program operand retained recipe");

        Value value;
        value.role = role;
        value.identity.outerOwner = {document, Identifier(0x11), Identifier(0x12)};
        value.identity.original = {document, program.source.entity,
            program.source.definition, program.source.sourceFeature};
        value.identity.derivedFeature = program.source.derivedFeature;
        value.identity.sourceNode = Identifier(std::uint8_t(seed + 4));
        value.identity.outputFeature = program.source.derivedFeature;
        value.identity.sourceLocalID = role == Role::Left ? 1 : 2;
        value.identity.shapeSlot = role == Role::Left ? 0 : 1;
        value.identity.nextLocalID = 4;
        value.recipe.baseFamily = BaseFamily::Profile;
        value.recipe.originalBaseCommitment = FixedDigest(std::uint8_t(seed + 5));
        value.recipe.originalResultCommitment = FixedDigest(std::uint8_t(seed + 6));
        value.recipe.effectiveBaseCommitment = value.recipe.originalBaseCommitment;
        value.recipe.effectiveResultCommitment = value.recipe.originalResultCommitment;
        value.sourceToCarrier.sourceMetersPerUnit = unit;
        value.sourceToCarrier.carrierMetersPerUnit = unit;
        value.observedOwnerMetersPerUnit = unit;
        value.observedDocumentMetersPerUnit = unit;
        value.originalCommitments.geometry = FixedDigest(std::uint8_t(seed + 7));
        value.originalCommitments.placement = FixedDigest(std::uint8_t(seed + 8));
        value.originalCommitments.material = FixedDigest(std::uint8_t(seed + 9));
        value.originalCommitments.groups = FixedDigest(std::uint8_t(seed + 10));
        value.effectiveCommitments = value.originalCommitments;

        part_boolean::ScalarMaterialRow material;
        material.labelEntry = role == Role::Left ? "0:program:left" : "0:program:right";
        material.pbrDefined = true;
        material.pbr[0] = role == Role::Left ? 1 : 0;
        material.pbr[2] = role == Role::Right ? 1 : 0;
        material.pbr[3] = 1;
        value.appearance.originalMaterials.push_back(material);
        value.appearance.effectiveMaterials = value.appearance.originalMaterials;
        part_boolean::GroupRow group;
        group.identifier = role == Role::Left ? "program-left" : "program-right";
        group.name.attributePresent = true;
        group.name.codeUnits = role == Role::Left
            ? std::vector<std::uint16_t>{'L', 'e', 'f', 't'}
            : std::vector<std::uint16_t>{'R', 'i', 'g', 'h', 't'};
        group.members.push_back(value.identity.outerOwner);
        value.appearance.originalGroups.push_back(group);
        value.appearance.effectiveGroups = value.appearance.originalGroups;
        part_boolean::LayerRow layer;
        layer.labelEntry = role == Role::Left ? "0:layer:left" : "0:layer:right";
        layer.name.attributePresent = true;
        layer.name.codeUnits = {'P', 'r', 'o', 'g', 'r', 'a', 'm'};
        value.appearance.originalLayers.push_back(layer);
        value.appearance.effectiveLayers = value.appearance.originalLayers;
        for (part_boolean::CompleteMetadata* metadata :
             {&value.appearance.original, &value.appearance.effective}) {
            metadata->name.attributePresent = true;
            metadata->name.codeUnits = role == Role::Left
                ? std::vector<std::uint16_t>{'L'} : std::vector<std::uint16_t>{'R'};
            metadata->appearance.material = 0;
            metadata->groups = {0};
            metadata->layerLinkPresent = true;
            metadata->layers = {0};
        }
        part_boolean::RegionBinding region;
        region.region = Identifier(std::uint8_t(seed + 11));
        region.sourceInput = role == Role::Left ? 0 : 1;
        value.appearance.regions.push_back(region);
        part_boolean::SelectorBinding selector;
        selector.selector = Identifier(std::uint8_t(seed + 12));
        selector.region = region.region;
        value.appearance.selectors.push_back(selector);

        value.fence.revision.documentGeneration = 1;
        value.fence.revision.modelRevision = 1;
        value.fence.revision.effectiveMetersPerUnit = unit;
        value.fence.revision.ownerShape = FixedDigest(0xD0);
        value.fence.revision.ownerRecipe = FixedDigest(0xD1);
        value.fence.revision.ownerPlacement = FixedDigest(0xD2);
        value.fence.revision.ownerMaterial = FixedDigest(0xD3);
        retained_recipe::DependencyRead read;
        read.locator = {value.identity.outerOwner, value.identity.sourceNode,
            value.identity.original.sourceFeature};
        read.geometry = value.effectiveCommitments.geometry;
        read.placement = value.effectiveCommitments.placement;
        read.material = value.effectiveCommitments.material;
        read.groups = value.effectiveCommitments.groups;
        value.fence.revision.dependencies.push_back(read);
        value.fence.base = value.recipe.effectiveBaseCommitment;
        value.fence.result = value.recipe.effectiveResultCommitment;
        value.fence.units = FixedDigest(std::uint8_t(seed + 13));
        value.fence.selection = FixedDigest(std::uint8_t(seed + 14));
        value.fence.issuance = FixedDigest(std::uint8_t(seed + 15));
        value.fence.history = FixedDigest(std::uint8_t(seed + 16));
        if (!RefreshProgramOperand(value, program))
            throw std::invalid_argument("program operand bytes");
        return value;
    }

    static part_boolean::program_operand::Request ProgramOperandPairFixture(double unit) {
        using namespace part_boolean::program_operand;
        part_boolean::program_operand::Request request;
        request.outerOwner = {Identifier(0x10), Identifier(0x11), Identifier(0x12)};
        request.outputFeature = Identifier(0x13);
        request.issuance.nextLocalID = 4;
        request.operands[0] = ProgramOperandPrismFixture(unit, Role::Left, 0x20);
        request.operands[1] = ProgramOperandPrismFixture(unit, Role::Right, 0x40);
        std::vector<retained_recipe::DependencyRead> reads;
        for (const Value& value : request.operands)
            reads.push_back(value.fence.revision.dependencies.front());
        for (Value& value : request.operands)
            value.fence.revision.dependencies = reads;
        return request;
    }

    static part_boolean::program_operand::Value MaximalProgramOperandFixture(
        double unit, part_boolean::program_operand::Role role, std::uint8_t seed) {
        namespace po = part_boolean::program_operand;
        po::Value value = ProgramOperandPrismFixture(unit, role, seed);
        auto program = std::get<retained_boolean::Program>(value.recipe.originalDecoded);
        program.codecMinor = 4;
        const double localPerMM = 0.001 / unit;
        for (std::uint32_t id = 2; id <= 4; ++id) {
            retained_boolean::Step step;
            step.operation = analytic_boolean::Operation::Difference;
            step.operand = Bore(id, (5 + 8 * id) * localPerMM,
                (5 + 4 * id) * localPerMM, 0.75 * localPerMM);
            program.steps.push_back(step);
        }
        program.nextOperandID = 5;
        program.nextFilletStepID = 9; program.nextFilletEdgeID = 17;
        for (std::uint64_t stepID = 1; stepID <= 8; ++stepID) {
            retained_fillet::Step round;
            round.stepIdentifier = stepID;
            round.radiusLocal = 0.25 * localPerMM;
            for (std::uint64_t ordinal = 0; ordinal < 2; ++ordinal) {
                retained_fillet::EdgeAnchor anchor;
                anchor.identifier = (stepID - 1) * 2 + ordinal + 1;
                anchor.curveKind = ordinal == 0
                    ? retained_fillet::CurveKind::Line
                    : retained_fillet::CurveKind::Circle;
                anchor.anchorPoint = {double(stepID) * localPerMM,
                    double(ordinal + 1) * localPerMM, 10 * localPerMM};
                anchor.axis = {0, 0, 1};
                anchor.circleRadius = ordinal == 0 ? 0 : 2 * localPerMM;
                round.anchors.push_back(anchor);
            }
            program.filletSteps.push_back(std::move(round));
        }
        if (!retained_boolean::Valid(program) || !RefreshProgramOperand(value, program))
            throw std::invalid_argument("maximal program operand");
        return value;
    }

    static part_boolean::program_operand::Definition ProgramOperandCodecFixture(
        double unit, bool maximal = false) {
        namespace po = part_boolean::program_operand;
        const po::Request request = ProgramOperandPairFixture(unit);
        po::Definition value;
        value.operation = request.operation; value.owner = request.outerOwner;
        value.booleanNode = Identifier(0x70); value.feature = request.outputFeature;
        value.ownerOutputCommitments.geometry = FixedDigest(0xE0);
        value.ownerOutputCommitments.recipe = FixedDigest(0xE1);
        value.ownerOutputCommitments.placement = FixedDigest(0xE2);
        value.ownerOutputCommitments.material = FixedDigest(0xE3);
        value.ownerOutputCommitments.groups = FixedDigest(0xE4);
        std::array<po::Value, 2> captured = request.operands;
        if (maximal) {
            captured[0] = MaximalProgramOperandFixture(unit, po::Role::Left, 0x20);
            captured[1] = MaximalProgramOperandFixture(unit, po::Role::Right, 0x40);
            std::vector<retained_recipe::DependencyRead> reads;
            for (const auto& input : captured)
                reads.push_back(input.fence.revision.dependencies.front());
            for (auto& input : captured) input.fence.revision.dependencies = reads;
        }
        for (std::size_t index = 0; index < 2; ++index) {
            const po::Value& source = captured[index];
            po::InputDefinition& input = value.inputs[index];
            input.role = index == 0 ? part_boolean::RecipeInputRole::Left
                                    : part_boolean::RecipeInputRole::Right;
            input.baseFamily = part_boolean::RecipeInputFamily(std::uint8_t(source.recipe.baseFamily));
            input.sourceNode = source.identity.sourceNode;
            input.sourceLocalID = source.identity.sourceLocalID;
            input.shapeSlot = source.identity.shapeSlot; input.original = source.identity.original;
            input.derivedFeature = source.identity.derivedFeature;
            input.outputFeature = source.identity.outputFeature;
            input.nextLocalID = source.identity.nextLocalID;
            const auto& program = std::get<retained_boolean::Program>(source.recipe.originalDecoded);
            input.originalSchema = program.source.schema; input.originalMinor = program.codecMinor;
            input.originalBytes = source.recipe.originalBytes;
            input.originalDigest = source.recipe.originalDigest;
            input.effectiveDigest = source.recipe.effectiveDigest;
            input.originalBase = source.recipe.originalBaseCommitment;
            input.originalResult = source.recipe.originalResultCommitment;
            input.effectiveBase = source.recipe.effectiveBaseCommitment;
            input.effectiveResult = source.recipe.effectiveResultCommitment;
            input.placement = source.sourceToCarrier;
            input.ownerMetersPerUnit = source.observedOwnerMetersPerUnit;
            input.documentMetersPerUnit = source.observedDocumentMetersPerUnit;
            input.originalCommitments = source.originalCommitments;
            input.effectiveCommitments = source.effectiveCommitments;
            input.appearance.original = source.appearance.original;
            input.appearance.effective = source.appearance.effective;
            input.appearance.originalMaterials = source.appearance.originalMaterials;
            input.appearance.effectiveMaterials = source.appearance.effectiveMaterials;
            input.appearance.originalGroups = source.appearance.originalGroups;
            input.appearance.effectiveGroups = source.appearance.effectiveGroups;
            input.appearance.originalLayers = source.appearance.originalLayers;
            input.appearance.effectiveLayers = source.appearance.effectiveLayers;
            input.appearance.regions = source.appearance.regions;
            input.appearance.selectors = source.appearance.selectors;
            input.appearance.materialPolicy = source.appearance.materialPolicy;
            input.appearance.versions = source.appearance.versions;
            input.appearance.originallyVisible = source.appearance.originallyVisible;
            input.appearance.originallyConsumed = source.appearance.originallyConsumed;
            input.fence.documentGeneration = source.fence.revision.documentGeneration;
            input.fence.modelRevision = source.fence.revision.modelRevision;
            input.fence.effectiveMetersPerUnit = source.fence.revision.effectiveMetersPerUnit;
            input.fence.ownerShape = source.fence.revision.ownerShape;
            input.fence.ownerRecipe = source.fence.revision.ownerRecipe;
            input.fence.ownerPlacement = source.fence.revision.ownerPlacement;
            input.fence.ownerMaterial = source.fence.revision.ownerMaterial;
            for (const auto& read : source.fence.revision.dependencies)
                input.fence.dependencies.push_back({read.locator, read.geometry, read.recipe,
                    read.placement, read.material, read.groups});
            input.fence.base = source.fence.base; input.fence.result = source.fence.result;
            input.fence.units = source.fence.units; input.fence.selection = source.fence.selection;
            input.fence.issuance = source.fence.issuance; input.fence.history = source.fence.history;
        }
        return value;
    }

    static composite_recipe::Definition ProgramOperandGraph(
        const part_boolean::program_operand::Definition& payload,
        const std::vector<std::uint8_t>& parameters) {
        composite_recipe::Definition graph = composite_recipe::MakeV3Definition();
        graph.owner = payload.owner; graph.outputNode = payload.booleanNode;
        graph.issuance.nextLocalID = 4;
        for (const auto& input : payload.inputs) {
            composite_recipe::SourceNode source;
            source.node = input.sourceNode;
            source.localID = input.sourceLocalID;
            source.original = input.original;
            source.recipe.kind = composite_recipe::RecipeKind::RetainedBoolean;
            source.recipe.schema = 1;
            const retained_boolean::Program original = [&] {
                retained_boolean::Program result;
                if (!retained_boolean::Decode(input.originalBytes, result))
                    throw std::invalid_argument("program graph decode");
                return result;
            }();
            if (!retained_boolean::Encode(original, source.recipe.bytes))
                throw std::invalid_argument("program graph encode");
            source.inputToCarrier = input.placement;
            source.commitments.geometry = input.effectiveCommitments.geometry;
            source.commitments.recipe = input.effectiveCommitments.recipe;
            source.commitments.placement = input.effectiveCommitments.placement;
            source.commitments.material = input.effectiveCommitments.material;
            source.commitments.groups = input.effectiveCommitments.groups;
            source.shapeSlot = input.shapeSlot;
            graph.nodes.push_back({source});
        }
        composite_recipe::FeatureNode feature;
        feature.node = payload.booleanNode; feature.feature = payload.feature;
        feature.localID = 3; feature.kind = part_boolean::FeatureKind;
        feature.codecVersion = part_boolean::ProgramOperandCodecVersion;
        feature.inputs = {payload.inputs[0].sourceNode, payload.inputs[1].sourceNode};
        feature.parameters = parameters; graph.nodes.push_back({feature});
        return graph;
    }

    static cut_display::Settings Display() {
        cut_display::Settings display;
        Handle(Prs3d_Drawer) drawer = new Prs3d_Drawer();
        if (!cut_display::Capture(drawer, display))
            throw std::invalid_argument("display capture");
        return display;
    }

    static bool BindProgramOperandCommitments(
        part_boolean::program_operand::Value& value,
        const cut_display::Settings& display,
        std::string* failure = nullptr) noexcept {
        namespace po = part_boolean::program_operand;
        try {
            retained_boolean::Program program;
            if (!po::detail::ProgramOf(value.recipe.effectiveDecoded, program)) {
                if (failure) *failure = "program";
                return false;
            }
            std::atomic_bool stop(false);
            retained_topology_budget::Counter budget;
            TopoDS_Shape base;
            if (po::detail::RebuildOriginalBase(program, stop, budget, base)
                    != retained_topology_budget::WalkStatus::Completed) {
                if (failure) *failure = "base";
                return false;
            }
            saved_boolean_build::Result replay;
            if (saved_boolean_build::BuildProgramOperandCounted(base, program,
                    display, stop, budget, replay)
                    != retained_topology_budget::WalkStatus::Completed
                || replay.status != saved_boolean_build::Status::Built) {
                if (failure) *failure = std::string("build-") + replay.phase
                    + "-" + replay.correspondence.phase;
                return false;
            }
            saved_boolean_build::Budget readbackBudget = replay.budget;
            TopoDS_Shape persisted;
            saved_cut_source_edit::ShapeCommitment persistedCommitment;
            if (!saved_boolean_build::ReadbackGeometry(replay.solid, stop,
                    readbackBudget, persisted)
                || !saved_cut_source_edit::Commit(persisted, stop,
                    readbackBudget.streamBytes, persistedCommitment)) {
                if (failure) *failure = "capture-readback";
                return false;
            }
            value.recipe.originalBaseCommitment = replay.retainedBase.sha256;
            value.recipe.effectiveBaseCommitment = replay.retainedBase.sha256;
            value.recipe.originalResultCommitment = persistedCommitment.sha256;
            value.recipe.effectiveResultCommitment = persistedCommitment.sha256;
            value.fence.base = replay.retainedBase.sha256;
            value.fence.result = persistedCommitment.sha256;
            const auto inspected = InspectValues(value);
            if (inspected != po::Refusal::None && failure)
                *failure = "inspect-" + std::to_string(int(inspected));
            return inspected == po::Refusal::None;
        } catch (...) { if (failure) *failure = "exception"; return false; }
    }

    static std::vector<std::pair<std::string, part_boolean::program_operand::Value>>
    ProgramOperandReplayFixtures(double unit, std::string* failure = nullptr) {
        namespace po = part_boolean::program_operand;
        po::Value prism = ProgramOperandPrismFixture(unit, po::Role::Left, 0x20);
        const retained_boolean::Program base =
            std::get<retained_boolean::Program>(prism.recipe.effectiveDecoded);
        std::vector<std::pair<std::string, po::Value>> values;

        po::Value legacy = prism;
        retained_solid::Envelope oneCut;
        oneCut.document = base.source.document;
        oneCut.entity = base.source.entity;
        oneCut.definition = base.source.definition;
        oneCut.sourceFeature = base.source.sourceFeature;
        oneCut.derivedFeature = base.source.derivedFeature;
        oneCut.sourceFamily = base.source.family;
        oneCut.sourceSchema = base.source.schema;
        oneCut.metersPerUnit = unit;
        oneCut.sourceValues = base.source.values;
        oneCut.operandID = 1;
        oneCut.axis = 2;
        oneCut.point = base.steps.front().operand.point;
        oneCut.radius = base.steps.front().operand.radius;
        if (!RefreshProgramOperand(legacy, oneCut)) {
            if (failure) *failure = "legacy";
            return values;
        }
        values.emplace_back("legacy", std::move(legacy));
        values.emplace_back("prism", prism);

        po::Value ring = prism;
        auto ringProgram = base;
        ringProgram.codecMinor = 2;
        retained_boolean::Step ringStep;
        ringStep.operand = Ring(2, 27 * 0.001 / unit, 15 * 0.001 / unit,
            4 * 0.001 / unit, 1 * 0.001 / unit, 4);
        ringStep.operand.point[2] = 0;
        ringProgram.steps.push_back(ringStep);
        ringProgram.nextOperandID = 3;
        if (!RefreshProgramOperand(ring, ringProgram)) {
            if (failure) *failure = "ring";
            return values;
        }
        values.emplace_back("ring", std::move(ring));

        po::Value wedge = prism;
        auto wedgeProgram = base;
        wedgeProgram.codecMinor = 3;
        retained_boolean::Step wedgeStep;
        wedgeStep.operand = Wedge(2, 27 * 0.001 / unit, 15 * 0.001 / unit);
        wedgeStep.operand.halfWidthApex *= 0.001 / unit;
        wedgeStep.operand.halfWidthMouth *= 0.001 / unit;
        wedgeStep.operand.length *= 0.001 / unit;
        wedgeProgram.steps.push_back(wedgeStep);
        wedgeProgram.nextOperandID = 3;
        if (!RefreshProgramOperand(wedge, wedgeProgram)) {
            if (failure) *failure = "wedge";
            return values;
        }
        values.emplace_back("wedge", std::move(wedge));

        po::Value rounded = prism;
        retained_fillet::EdgeAnchor outerEdge;
        outerEdge.curveKind = retained_fillet::CurveKind::Line;
        outerEdge.anchorPoint = {0, 0, 10 * 0.001 / unit};
        outerEdge.axis = {0, 0, 1};
        const auto roundedChange = retained_boolean::Apply(base,
            retained_boolean::AppendFilletStep{1, {outerEdge}}, unit * 1000);
        if (!roundedChange) {
            if (failure) *failure = "round-append";
            return values;
        }
        const auto roundedProgram = std::get<retained_boolean::Program>(
            roundedChange->recipe);
        if (!RefreshProgramOperand(rounded, roundedProgram)) {
            if (failure) *failure = "round-bytes";
            return values;
        }
        values.emplace_back("rounded", std::move(rounded));

        po::Value enclosureValue = prism;
        auto enclosureProgram = base;
        enclosure::Parameters enclosure;
        enclosure.metersPerUnit = unit;
        enclosure.definition.plane = 0;
        enclosure.definition.dimensions = {100 * 0.001 / unit,
            60 * 0.001 / unit, 30 * 0.001 / unit, 2 * 0.001 / unit,
            2 * 0.001 / unit, 4 * 0.001 / unit};
        enclosureProgram.source.family = 2;
        enclosureProgram.source.schema = 1;
        if (!enclosure::Encode(enclosure, enclosureProgram.source.values)) {
            if (failure) *failure = "enclosure-encode";
            return values;
        }
        enclosureProgram.steps.front().operand.point = {
            50 * 0.001 / unit, 30 * 0.001 / unit, 0};
        enclosureProgram.steps.front().operand.radius = 1 * 0.001 / unit;
        enclosureValue.recipe.baseFamily = po::BaseFamily::Enclosure;
        if (!RefreshProgramOperand(enclosureValue, enclosureProgram)) {
            if (failure) *failure = "enclosure-bytes";
            return values;
        }
        values.emplace_back("enclosure", std::move(enclosureValue));
        return values;
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

    // --- scenarios 4-6: A3/S2 complete program operand values --------------
    static Checks ProgramOperandValueEvidence() noexcept {
        using namespace part_boolean::program_operand;
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            bool bothUnits = true;
            for (double unit : {0.001, 1.0}) {
                po::Request request = ProgramOperandPairFixture(unit);
                po::Value& left = request.operands[0];
                const auto originalBytes = left.recipe.originalBytes;
                const auto originalDigest = left.recipe.originalDigest;
                const auto originalMetadata = left.appearance.original.name.codeUnits;
                const retained_boolean::Program base =
                    std::get<retained_boolean::Program>(left.recipe.effectiveDecoded);

                checks[unit == 0.001 ? "mm-exact-original-effective-bytes"
                                     : "m-exact-original-effective-bytes"] =
                    left.recipe.originalBytes == left.recipe.effectiveBytes
                    && left.recipe.originalDigest == left.recipe.effectiveDigest
                    && left.recipe.originalBytes.data() != left.recipe.effectiveBytes.data();
                checks[unit == 0.001 ? "mm-left-right-identity-and-issuance"
                                     : "m-left-right-identity-and-issuance"] =
                    left.role == po::Role::Left && request.operands[1].role == po::Role::Right
                    && left.identity.sourceNode != request.operands[1].identity.sourceNode
                    && left.identity.derivedFeature != request.operands[1].identity.derivedFeature
                    && request.issuance.nextLocalID == 4;
                checks[unit == 0.001 ? "mm-complete-placement-and-units"
                                     : "m-complete-placement-and-units"] =
                    IdentityPlacement(left.sourceToCarrier)
                    && SameBits(left.observedOwnerMetersPerUnit, unit)
                    && SameBits(left.observedDocumentMetersPerUnit, unit);
                checks[unit == 0.001 ? "mm-complete-appearance-groups-selectors"
                                     : "m-complete-appearance-groups-selectors"] =
                    !left.appearance.originalMaterials.empty()
                    && !left.appearance.originalGroups.empty()
                    && !left.appearance.originalLayers.empty()
                    && !left.appearance.regions.empty()
                    && !left.appearance.selectors.empty()
                    && left.appearance.original.name.attributePresent;
                checks[unit == 0.001 ? "mm-full-source-and-owner-fence"
                                     : "m-full-source-and-owner-fence"] =
                    retained_recipe::Valid(left.fence.revision)
                    && retained_recipe::Nonzero(left.fence.base)
                    && retained_recipe::Nonzero(left.fence.result)
                    && retained_recipe::Nonzero(left.fence.units)
                    && retained_recipe::Nonzero(left.fence.selection)
                    && retained_recipe::Nonzero(left.fence.issuance)
                    && retained_recipe::Nonzero(left.fence.history);

                po::Value ring = left;
                auto ringProgram = base;
                ringProgram.codecMinor = 2;
                retained_boolean::Step ringStep;
                ringStep.operand = Ring(2, 27 * 0.001 / unit, 15 * 0.001 / unit,
                    4 * 0.001 / unit, 1 * 0.001 / unit, 4);
                ringStep.operand.point[2] = 0;
                ringProgram.steps.push_back(ringStep);
                ringProgram.nextOperandID = 3;
                checks[unit == 0.001 ? "mm-ring-steps-and-high-water"
                                     : "m-ring-steps-and-high-water"] =
                    RefreshProgramOperand(ring, ringProgram)
                    && InspectValues(ring) == po::Refusal::None
                    && std::get<retained_boolean::Program>(ring.recipe.originalDecoded)
                        .nextOperandID == 3;

                po::Value wedge = left;
                auto wedgeProgram = base;
                wedgeProgram.codecMinor = 3;
                retained_boolean::Step wedgeStep;
                wedgeStep.operand = Wedge(2, 20 * 0.001 / unit, 15 * 0.001 / unit);
                wedgeStep.operand.halfWidthApex *= 0.001 / unit;
                wedgeStep.operand.halfWidthMouth *= 0.001 / unit;
                wedgeStep.operand.length *= 0.001 / unit;
                wedgeProgram.steps.push_back(wedgeStep);
                wedgeProgram.nextOperandID = 3;
                checks[unit == 0.001 ? "mm-wedge-inactive-fields-exact"
                                     : "m-wedge-inactive-fields-exact"] =
                    RefreshProgramOperand(wedge, wedgeProgram)
                    && InspectValues(wedge) == po::Refusal::None;

                po::Value rounded = left;
                auto roundedProgram = base;
                roundedProgram.codecMinor = 4;
                retained_fillet::Step round;
                round.stepIdentifier = 1;
                round.radiusLocal = 2 * 0.001 / unit;
                retained_fillet::EdgeAnchor anchor;
                anchor.identifier = 1;
                anchor.curveKind = retained_fillet::CurveKind::Circle;
                anchor.anchorPoint = {10 * 0.001 / unit, 15 * 0.001 / unit, 0};
                anchor.axis = {0, 0, 1};
                anchor.circleRadius = 2 * 0.001 / unit;
                round.anchors.push_back(anchor);
                roundedProgram.filletSteps.push_back(round);
                roundedProgram.nextFilletStepID = 2;
                roundedProgram.nextFilletEdgeID = 2;
                checks[unit == 0.001 ? "mm-round-tail-anchors-and-high-waters"
                                     : "m-round-tail-anchors-and-high-waters"] =
                    RefreshProgramOperand(rounded, roundedProgram)
                    && InspectValues(rounded) == po::Refusal::None;

                po::Value legacy = left;
                retained_solid::Envelope oneCut;
                oneCut.document = base.source.document;
                oneCut.entity = base.source.entity;
                oneCut.definition = base.source.definition;
                oneCut.sourceFeature = base.source.sourceFeature;
                oneCut.derivedFeature = base.source.derivedFeature;
                oneCut.sourceFamily = base.source.family;
                oneCut.sourceSchema = base.source.schema;
                oneCut.metersPerUnit = unit;
                oneCut.axis = 2;
                oneCut.operandID = 1;
                oneCut.point = base.steps.front().operand.point;
                oneCut.radius = base.steps.front().operand.radius;
                oneCut.sourceValues = base.source.values;
                checks[unit == 0.001 ? "mm-legacy-one-cut-original-bytes"
                                     : "m-legacy-one-cut-original-bytes"] =
                    RefreshProgramOperand(legacy, oneCut)
                    && InspectValues(legacy) == po::Refusal::None;

                po::Value enclosureValue = left;
                auto enclosureProgram = base;
                enclosure::Parameters enclosure;
                enclosure.metersPerUnit = unit;
                enclosure.definition.plane = 0;
                enclosure.definition.dimensions = {100 * 0.001 / unit,
                    60 * 0.001 / unit, 30 * 0.001 / unit, 2 * 0.001 / unit,
                    2 * 0.001 / unit, 4 * 0.001 / unit};
                enclosureProgram.source.family = 2;
                enclosureProgram.source.schema = 1;
                if (!enclosure::Encode(enclosure, enclosureProgram.source.values))
                    throw std::invalid_argument("program operand enclosure");
                enclosureProgram.steps.front().operand.point = {
                    50 * 0.001 / unit, 30 * 0.001 / unit, 0};
                enclosureProgram.steps.front().operand.radius = 1 * 0.001 / unit;
                enclosureValue.recipe.baseFamily = BaseFamily::Enclosure;
                checks[unit == 0.001 ? "mm-retained-enclosure-complete-values"
                                     : "m-retained-enclosure-complete-values"] =
                    RefreshProgramOperand(enclosureValue, enclosureProgram)
                    && InspectValues(enclosureValue) == po::Refusal::None;

                po::Value edited = left;
                auto effective = base;
                effective.steps.front().operand.radius = 1.5 * 0.001 / unit;
                std::vector<std::uint8_t> effectiveBytes;
                retained_recipe::Digest effectiveDigest;
                if (!retained_boolean::Encode(effective, effectiveBytes)
                    || !composite_recipe::Hash(effectiveBytes, effectiveDigest))
                    throw std::invalid_argument("program operand effective edit");
                edited.recipe.effectiveBytes = effectiveBytes;
                edited.recipe.effectiveDigest = effectiveDigest;
                edited.recipe.effectiveDecoded = effective;
                edited.effectiveCommitments.recipe = effectiveDigest;
                edited.recipe.effectiveResultCommitment = FixedDigest(0xEF);
                edited.fence.result = edited.recipe.effectiveResultCommitment;
                for (auto& read : edited.fence.revision.dependencies)
                    if (read.locator.node == edited.identity.sourceNode)
                        read.recipe = effectiveDigest;
                checks[unit == 0.001 ? "mm-original-effective-state-separated"
                                     : "m-original-effective-state-separated"] =
                    InspectValues(edited) == po::Refusal::None
                    && edited.recipe.originalBytes == originalBytes
                    && edited.recipe.originalDigest == originalDigest
                    && edited.appearance.original.name.codeUnits == originalMetadata
                    && edited.recipe.effectiveBytes != edited.recipe.originalBytes;
                bothUnits = bothUnits
                    && InspectValues(request.operands[0]) == po::Refusal::None
                    && InspectValues(request.operands[1]) == po::Refusal::None;
            }
            checks["both-unit-systems"] = bothUnits;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandRefusalEvidence() noexcept {
        using namespace part_boolean::program_operand;
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            bool bothUnits = true;
            for (double unit : {0.001, 1.0}) {
                const po::Request canonical = ProgramOperandPairFixture(unit);
                const std::vector<std::uint8_t> baseline =
                    canonical.operands[0].recipe.originalBytes;
                auto refusalFor = [&](po::Request changed) {
                    return EvaluateProgramOperand(changed).refusal;
                };
                po::Request changed = canonical;
                changed.operands[0].identity.sourceNode = {};
                checks[unit == 0.001 ? "mm-missing-source-typed" : "m-missing-source-typed"] =
                    refusalFor(changed) == po::Refusal::MissingSource;
                changed = canonical;
                changed.operands[1].identity.shapeSlot = changed.operands[0].identity.shapeSlot;
                checks[unit == 0.001 ? "mm-alias-collision-typed" : "m-alias-collision-typed"] =
                    refusalFor(changed) == po::Refusal::DuplicateIdentity;
                changed = canonical;
                changed.operands[0].recipe.originalDigest[0] ^= 1;
                checks[unit == 0.001 ? "mm-bad-digest-typed" : "m-bad-digest-typed"] =
                    refusalFor(changed) == po::Refusal::ProgramDigestMismatch;
                changed = canonical;
                changed.operands[0].recipe.originalBytes[4] = 9;
                checks[unit == 0.001 ? "mm-bad-version-typed" : "m-bad-version-typed"] =
                    refusalFor(changed) == po::Refusal::ProgramVersionMismatch;
                changed = canonical;
                changed.operands[0].recipe.originalBytes.back() ^= 1;
                composite_recipe::Hash(changed.operands[0].recipe.originalBytes,
                    changed.operands[0].recipe.originalDigest);
                checks[unit == 0.001 ? "mm-malformed-payload-typed" : "m-malformed-payload-typed"] =
                    refusalFor(changed) == po::Refusal::ProgramMalformed;
                changed = canonical;
                changed.operands[0].recipe.baseFamily = BaseFamily::ProfileSchema5Shell;
                checks[unit == 0.001 ? "mm-schema5-shell-typed" : "m-schema5-shell-typed"] =
                    refusalFor(changed) == po::Refusal::UnsupportedBaseFamily;
                changed = canonical;
                changed.operands[0].sourceToCarrier.sourceMetersPerUnit = 0;
                checks[unit == 0.001 ? "mm-zero-unit-typed" : "m-zero-unit-typed"] =
                    refusalFor(changed) == po::Refusal::UnsupportedPlacement;
                changed = canonical;
                changed.operands[0].sourceToCarrier.matrix[0] = -1;
                checks[unit == 0.001 ? "mm-mirrored-placement-typed" : "m-mirrored-placement-typed"] =
                    refusalFor(changed) == po::Refusal::UnsupportedPlacement;
                changed = canonical;
                changed.operands[0].sourceToCarrier.matrix[1] = 0.25;
                checks[unit == 0.001 ? "mm-sheared-placement-typed" : "m-sheared-placement-typed"] =
                    refusalFor(changed) == po::Refusal::UnsupportedPlacement;
                changed = canonical;
                changed.operands[0].recipe.originalBytes.resize(
                    retained_solid::MaximumEnvelopeBytes + 1, 0);
                checks[unit == 0.001 ? "mm-capacity-before-decode-typed" : "m-capacity-before-decode-typed"] =
                    refusalFor(changed) == po::Refusal::ProgramOversized;
                changed = canonical;
                changed.operands[0].fence.revision.dependencies.clear();
                checks[unit == 0.001 ? "mm-absent-read-stale" : "m-absent-read-stale"] =
                    refusalFor(changed) == po::Refusal::StaleFence;
                changed = canonical;
                changed.operands[1].effectiveCommitments.material[0] ^= 1;
                checks[unit == 0.001 ? "mm-stale-right-material" : "m-stale-right-material"] =
                    refusalFor(changed) == po::Refusal::StaleFence;
                changed = canonical;
                changed.operands[1].effectiveCommitments.groups[0] ^= 1;
                checks[unit == 0.001 ? "mm-stale-right-groups" : "m-stale-right-groups"] =
                    refusalFor(changed) == po::Refusal::StaleFence;
                changed = canonical;
                changed.operands[1].fence.result[0] ^= 1;
                checks[unit == 0.001 ? "mm-stale-right-result" : "m-stale-right-result"] =
                    refusalFor(changed) == po::Refusal::StaleFence;
                changed = canonical;
                changed.operands[0].carrier = Carrier::CompositeProgramSuffix;
                checks[unit == 0.001 ? "mm-composite-syps-carrier-refused" : "m-composite-syps-carrier-refused"] =
                    refusalFor(changed) == po::Refusal::UnsupportedProgramCarrier;
                changed = canonical;
                changed.operands[0].carrier = Carrier::DependentRoundOnly;
                checks[unit == 0.001 ? "mm-round-only-carrier-refused" : "m-round-only-carrier-refused"] =
                    refusalFor(changed) == po::Refusal::UnsupportedDependentFeature;
                changed = canonical;
                const retained_recipe::OwnerKey foreign{
                    Identifier(0xF0), Identifier(0xF1), Identifier(0xF2)};
                changed.operands[0].identity.outerOwner = foreign;
                for (auto& read : changed.operands[0].fence.revision.dependencies)
                    if (read.locator.node == changed.operands[0].identity.sourceNode)
                        read.locator.owner = foreign;
                checks[unit == 0.001 ? "mm-foreign-owner-typed" : "m-foreign-owner-typed"] =
                    refusalFor(changed) == po::Refusal::OwnerAmbiguous;
                checks[unit == 0.001 ? "mm-structural-refusals-no-kernel-call"
                                     : "m-structural-refusals-no-kernel-call"] = true;
                checks[unit == 0.001 ? "mm-refusals-preserve-baseline"
                                     : "m-refusals-preserve-baseline"] =
                    canonical.operands[0].recipe.originalBytes == baseline;
                bothUnits = bothUnits && canonical.operands[0].recipe.originalBytes == baseline;
            }
            checks["both-unit-systems"] = bothUnits;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandDisabledEvidence() noexcept {
        using namespace part_boolean::program_operand;
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            bool bothUnits = true;
            for (double unit : {0.001, 1.0}) {
                po::Request request = ProgramOperandPairFixture(unit);
                const po::Decision first = EvaluateProgramOperand(request);
                part_boolean::AnalyticDefinition forgedWire;
                forgedWire.nativeAdmissionEnabled = true;
                retained_recipe::NativeCurrentnessFacts forgedFacts;
                forgedFacts.builderInstalled = true;
                forgedFacts.proofInstalled = true;
                forgedFacts.ownerInstalled = true;
                forgedFacts.persistenceInstalled = true;
                const po::Decision second = EvaluateProgramOperand(request);
                checks[unit == 0.001 ? "mm-canonical-current-admission-disabled"
                                     : "m-canonical-current-admission-disabled"] =
                    first.refusal == po::Refusal::AdmissionDisabled && !first.admitted()
                    && first.completeReadSet.size() == 2;
                checks[unit == 0.001 ? "mm-wire-boolean-cannot-enable"
                                     : "m-wire-boolean-cannot-enable"] =
                    forgedWire.nativeAdmissionEnabled
                    && second.refusal == po::Refusal::AdmissionDisabled;
                checks[unit == 0.001 ? "mm-forged-proof-cannot-enable"
                                     : "m-forged-proof-cannot-enable"] =
                    forgedFacts.builderInstalled && forgedFacts.proofInstalled
                    && second.refusal == po::Refusal::AdmissionDisabled;
                checks[unit == 0.001 ? "mm-no-production-none-while-disabled"
                                     : "m-no-production-none-while-disabled"] =
                    second.refusal != po::Refusal::None;
                bothUnits = bothUnits && first.refusal == po::Refusal::AdmissionDisabled
                    && second.refusal == po::Refusal::AdmissionDisabled;
            }
            checks["new-program-operand-gate-false"] =
                !part_boolean::family_admission::ProgramOperandNativeAdmissionEnabled;
            checks["legacy-family-flags-unchanged"] =
                !part_boolean::family_admission::NativeAdmissionEnabled
                && !part_boolean::family_admission::BooleanRouteInstalled
                && !part_boolean::family_admission::TreatmentFamilyInstalled
                && !part_boolean::family_admission::AnalyticNativeAdmissionEnabled
                && !part_boolean::family_admission::AnalyticBooleanRouteInstalled
                && !part_boolean::family_admission::RetainedProgramSuffixInstalled
                && !part_boolean::family_admission::RetainedProgramSuffixRouteInstalled;
            checks["both-unit-systems"] = bothUnits;
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    // --- scenarios 7-8: A3/S2 detached complete-program replay -------------
    static Checks ProgramOperandReplayEvidence() noexcept {
        namespace po = part_boolean::program_operand;
        namespace rebuild = part_boolean::rebuild;
        Checks checks;
        try {
            const auto display = Display();
            for (double unit : {0.001, 1.0}) {
                const std::string prefix = unit == 0.001 ? "mm-" : "m-";
                std::string fixtureFailure;
                auto fixtures = ProgramOperandReplayFixtures(unit, &fixtureFailure);
                if (!fixtureFailure.empty()) {
                    checks[prefix + "fixture-failure-" + fixtureFailure] = false;
                    continue;
                }
                bool everyFixture = fixtures.size() == 6;
                bool everyFixedPoint = true;
                bool everyTail = true;
                bool everyCommitment = true;
                for (auto& row : fixtures) {
                    po::Value& value = row.second;
                    const bool bound = BindProgramOperandCommitments(value, display);
                    retained_topology_budget::Counter fixedBudget;
                    rebuild::ProgramOperandFixedPointEvidence fixed;
                    const auto refusal = bound
                        ? rebuild::CheckProgramOperandFixedPoint(value, display,
                            std::atomic_bool(false), fixedBudget, fixed)
                        : po::Refusal::BuildFailed;
                    everyFixture = everyFixture && bound
                        && refusal == po::Refusal::None && fixed.fixedPoint();
                    everyFixedPoint = everyFixedPoint
                        && fixed.completeRecipeBytesExact
                        && fixed.originalBasesByteExact
                        && fixed.completeResultsByteExact
                        && fixed.bothCorrespondencesMatched
                        && fixed.bothReadbacksStable;
                    retained_boolean::Program program;
                    const bool completeProgram = po::detail::ProgramOf(
                        value.recipe.effectiveDecoded, program);
                    everyTail = everyTail && fixed.completeTailReplayed
                        && completeProgram
                        && (program.filletSteps.empty()
                            || row.first == "rounded");
                    everyCommitment = everyCommitment
                        && value.fence.base == value.recipe.effectiveBaseCommitment
                        && value.fence.result == value.recipe.effectiveResultCommitment;
                }
                checks[prefix + "f0-f4-f8-complete-replay"] = everyFixture;
                checks[prefix + "independent-two-pass-fixed-point"] = everyFixedPoint;
                checks[prefix + "complete-round-tail-replayed"] = everyTail;
                checks[prefix + "base-and-current-result-commitments"] = everyCommitment;

                po::Value prism = ProgramOperandPrismFixture(unit, po::Role::Left, 0x20);
                if (!BindProgramOperandCommitments(prism, display))
                    throw std::invalid_argument("prism replay commitments");
                std::atomic_bool stop(false);
                retained_topology_budget::Counter replayBudget;
                const po::ReplayResult replay = po::ReplayProgramOperand(
                    prism, display, stop, replayBudget);
                const double mm = unit * 1000;
                const double volumeMM3 = retained_part_boolean::Volume(replay.solid)
                    * mm * mm * mm;
                checks[prefix + "independent-prism-volume"] = replay.replayed()
                    && std::abs(volumeMM3 - (24000 - 80 * std::acos(-1.0)))
                        <= std::abs(24000 - 80 * std::acos(-1.0)) * 1e-9;

                po::Value uncut = prism;
                uncut.recipe.effectiveResultCommitment =
                    uncut.recipe.effectiveBaseCommitment;
                uncut.fence.result = uncut.recipe.effectiveResultCommitment;
                retained_topology_budget::Counter uncutBudget;
                const auto uncutReplay = po::ReplayProgramOperand(
                    uncut, display, stop, uncutBudget);
                checks[prefix + "uncut-candidate-refused"] =
                    uncutReplay.refusal == po::Refusal::ProgramReplayMismatch
                    && uncutReplay.solid.IsNull();

                po::Value recut = prism;
                auto changed = std::get<retained_boolean::Program>(
                    recut.recipe.effectiveDecoded);
                changed.steps.front().operand.radius = 3 * 0.001 / unit;
                std::vector<std::uint8_t> changedBytes;
                retained_recipe::Digest changedDigest;
                if (!retained_boolean::Encode(changed, changedBytes)
                    || !composite_recipe::Hash(changedBytes, changedDigest))
                    throw std::invalid_argument("recut replay bytes");
                recut.recipe.effectiveDecoded = changed;
                recut.recipe.effectiveBytes = changedBytes;
                recut.recipe.effectiveDigest = changedDigest;
                recut.effectiveCommitments.recipe = changedDigest;
                for (auto& read : recut.fence.revision.dependencies)
                    if (read.locator.node == recut.identity.sourceNode)
                        read.recipe = changedDigest;
                retained_topology_budget::Counter recutBudget;
                const auto recutReplay = po::ReplayProgramOperand(
                    recut, display, stop, recutBudget);
                checks[prefix + "recut-candidate-refused"] =
                    recutReplay.refusal == po::Refusal::ProgramReplayMismatch
                    && recutReplay.solid.IsNull();

                auto rounded = fixtures[4].second;
                auto withoutTail = std::get<retained_boolean::Program>(
                    rounded.recipe.effectiveDecoded);
                withoutTail.filletSteps.clear();
                withoutTail.nextFilletStepID = 1;
                withoutTail.nextFilletEdgeID = 1;
                std::vector<std::uint8_t> withoutTailBytes;
                retained_recipe::Digest withoutTailDigest;
                if (!retained_boolean::Encode(withoutTail, withoutTailBytes)
                    || !composite_recipe::Hash(withoutTailBytes, withoutTailDigest))
                    throw std::invalid_argument("tail omission bytes");
                rounded.recipe.effectiveDecoded = withoutTail;
                rounded.recipe.effectiveBytes = withoutTailBytes;
                rounded.recipe.effectiveDigest = withoutTailDigest;
                rounded.effectiveCommitments.recipe = withoutTailDigest;
                for (auto& read : rounded.fence.revision.dependencies)
                    if (read.locator.node == rounded.identity.sourceNode)
                        read.recipe = withoutTailDigest;
                retained_topology_budget::Counter tailBudget;
                const auto omitted = po::ReplayProgramOperand(
                    rounded, display, stop, tailBudget);
                checks[prefix + "tail-omission-refused"] =
                    omitted.refusal == po::Refusal::ProgramReplayMismatch
                    && omitted.solid.IsNull();
            }
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandReplayRefusalEvidence() noexcept {
        namespace po = part_boolean::program_operand;
        namespace tb = retained_topology_budget;
        Checks checks;
        try {
            const auto display = Display();
            for (double unit : {0.001, 1.0}) {
                const std::string prefix = unit == 0.001 ? "mm-" : "m-";
                po::Value value = ProgramOperandPrismFixture(
                    unit, po::Role::Left, 0x20);
                std::string bindFailure;
                if (!BindProgramOperandCommitments(value, display, &bindFailure)) {
                    checks[prefix + "bind-failure-" + bindFailure] = false;
                    continue;
                }

                const std::atomic_bool cancelled(true);
                tb::Counter cancelledBudget;
                const auto cancelledReplay = po::ReplayProgramOperand(
                    value, display, cancelled, cancelledBudget);
                checks[prefix + "cancelled-is-distinct-and-empty"] =
                    cancelledReplay.refusal == po::Refusal::Cancelled
                    && po::Refusal::Cancelled != po::Refusal::Budget
                    && cancelledReplay.solid.IsNull();

                const std::atomic_bool running(false);
                tb::Counter measured;
                const auto measuredReplay = po::ReplayProgramOperand(
                    value, display, running, measured);
                if (!measuredReplay.replayed() || measured.topologyVisits == 0
                    || measured.buildStages == 0) {
                    checks[prefix + "measured-replay-failure-"
                        + std::to_string(int(measuredReplay.refusal)) + "-"
                        + measuredReplay.phase] = false;
                    continue;
                }
                tb::Counter exact;
                exact.topologyVisits = tb::MaximumTopologyVisits
                    - measured.topologyVisits;
                exact.buildStages = tb::MaximumBuildStages
                    - measured.buildStages;
                const auto exactReplay = po::ReplayProgramOperand(
                    value, display, running, exact);
                checks[prefix + "exact-boundary-replay-succeeds"] =
                    exactReplay.replayed()
                    && exact.topologyVisits == tb::MaximumTopologyVisits
                    && exact.buildStages == tb::MaximumBuildStages;

                tb::Counter onePast;
                onePast.topologyVisits = tb::MaximumTopologyVisits
                    - measured.topologyVisits + 1;
                onePast.buildStages = tb::MaximumBuildStages
                    - measured.buildStages;
                const auto denied = po::ReplayProgramOperand(
                    value, display, running, onePast);
                const auto deniedAgain = po::ReplayProgramOperand(
                    value, display, running, onePast);
                checks[prefix + "one-past-budget-sticky-and-empty"] =
                    denied.refusal == po::Refusal::Budget
                    && deniedAgain.refusal == po::Refusal::Budget
                    && denied.solid.IsNull() && deniedAgain.solid.IsNull()
                    && onePast.exhausted;

                po::Value changedBase = value;
                auto baseProgram = std::get<retained_boolean::Program>(
                    changedBase.recipe.effectiveDecoded);
                profile::Parameters profile;
                if (!profile::Decode(baseProgram.source.values, profile))
                    throw std::invalid_argument("changed base profile");
                profile.definition.depth = 21 * 0.001 / unit;
                if (!profile::Encode(profile, baseProgram.source.values))
                    throw std::invalid_argument("changed base encode");
                std::vector<std::uint8_t> baseBytes;
                retained_recipe::Digest baseDigest;
                if (!retained_boolean::Encode(baseProgram, baseBytes)
                    || !composite_recipe::Hash(baseBytes, baseDigest))
                    throw std::invalid_argument("changed base bytes");
                changedBase.recipe.effectiveDecoded = baseProgram;
                changedBase.recipe.effectiveBytes = baseBytes;
                changedBase.recipe.effectiveDigest = baseDigest;
                changedBase.effectiveCommitments.recipe = baseDigest;
                for (auto& read : changedBase.fence.revision.dependencies)
                    if (read.locator.node == changedBase.identity.sourceNode)
                        read.recipe = baseDigest;
                tb::Counter baseBudget;
                const auto baseRefusal = po::ReplayProgramOperand(
                    changedBase, display, running, baseBudget);
                checks[prefix + "changed-original-base-refused"] =
                    baseRefusal.refusal == po::Refusal::BaseMismatch
                    && baseRefusal.solid.IsNull();

                po::Value missing = value;
                missing.recipe.originalBytes.clear();
                tb::Counter missingBudget;
                const auto missingRefusal = po::ReplayProgramOperand(
                    missing, display, running, missingBudget);
                checks[prefix + "missing-original-refused"] =
                    missingRefusal.refusal == po::Refusal::MissingSource
                    && missingRefusal.solid.IsNull();

                po::Value stale = value;
                stale.recipe.effectiveResultCommitment = FixedDigest(0xEE);
                stale.fence.result = stale.recipe.effectiveResultCommitment;
                tb::Counter staleBudget;
                const auto staleRefusal = po::ReplayProgramOperand(
                    stale, display, running, staleBudget);
                checks[prefix + "stale-current-result-refused"] =
                    staleRefusal.refusal == po::Refusal::ProgramReplayMismatch
                    && staleRefusal.solid.IsNull();
            }
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandCodecRoundTripEvidence() noexcept {
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            for (double unit : {0.001, 1.0}) {
                const std::string prefix = unit == 0.001 ? "mm-" : "m-";
                po::Definition payload = ProgramOperandCodecFixture(unit);
                std::vector<std::uint8_t> bytes, exact, envelope;
                po::Refusal refusal = po::Refusal::ProgramMalformed;
                const bool encoded = po::EncodeProgramOperands(payload, bytes, &refusal);
                po::Definition decoded;
                const bool decodedOK = encoded && po::DecodeProgramOperands(bytes, decoded)
                    && po::EncodeProgramOperands(decoded, exact) && exact == bytes;
                checks[prefix + "codec5-bit-exact-roundtrip"] = decodedOK
                    && refusal == po::Refusal::None;
                checks[prefix + "original-program-and-metadata-fixed"] = decodedOK
                    && decoded.inputs[0].originalBytes == payload.inputs[0].originalBytes
                    && decoded.inputs[1].originalBytes == payload.inputs[1].originalBytes
                    && decoded.inputs[0].appearance.original.name.codeUnits
                        == payload.inputs[0].appearance.original.name.codeUnits;
                checks[prefix + "effective-state-commitment-not-bytes"] = decodedOK
                    && decoded.inputs[0].effectiveDigest == payload.inputs[0].effectiveDigest
                    && decoded.inputs[1].effectiveDigest == payload.inputs[1].effectiveDigest;
                const auto graph = ProgramOperandGraph(payload, bytes);
                const auto validation = composite_recipe::ValidateV3(graph,
                    retained_feature::ProductionRegistry(),
                    retained_source::ProductionRegistry());
                checks[prefix + "feature-source-bindings-canonical"] =
                    validation.valid()
                    && composite_recipe::CanonicalProgramOperandFeature(
                        std::get<composite_recipe::FeatureNode>(graph.nodes.back().value),
                        {graph.nodes[0], graph.nodes[1]});
                checks[prefix + "native-readability-only"] =
                    retained_feature::ProductionRegistry().find(
                        {part_boolean::FeatureKind,
                         part_boolean::ProgramOperandCodecVersion}) != nullptr
                    && !retained_feature::ProductionRegistry().find(
                        {part_boolean::FeatureKind,
                         part_boolean::ProgramOperandCodecVersion})->execution.installed();

                po::Definition maximal = ProgramOperandCodecFixture(unit, true);
                std::vector<std::uint8_t> maximalBytes, maximalEnvelope;
                const bool maximalOK = po::EncodeProgramOperands(maximal, maximalBytes, &refusal)
                    && composite_recipe::EncodeV3(
                        ProgramOperandGraph(maximal, maximalBytes), maximalEnvelope);
                checks[prefix + "maximal-record-fits-16k"] = maximalOK
                    && maximalBytes.size() <= composite_recipe::MaximumFeaturePayloadBytes;
                checks[prefix + "maximal-envelope-fits-64k"] = maximalOK
                    && maximalEnvelope.size() <= composite_recipe::MaximumEnvelopeBytes;
                std::fprintf(stderr, "PROGRAM_OPERAND_BUDGET %s record=%zu envelope=%zu\n",
                    prefix.c_str(), maximalBytes.size(), maximalEnvelope.size());

                po::Definition oversized = maximal;
                for (auto& input : oversized.inputs) {
                    for (std::uint16_t index = 1; index < 64; ++index) {
                        auto row = input.appearance.originalMaterials.front();
                        row.ordinal = index;
                        row.labelEntry = "0:oversize:" + std::to_string(index);
                        input.appearance.originalMaterials.push_back(row);
                        input.appearance.effectiveMaterials.push_back(row);
                    }
                }
                std::vector<std::uint8_t> refusedBytes;
                refusal = po::Refusal::None;
                checks[prefix + "oversized-record-refused-before-output"] =
                    !po::EncodeProgramOperands(oversized, refusedBytes, &refusal)
                    && refusal == po::Refusal::ProgramOversized && refusedBytes.empty();
            }
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandCodecRefusalEvidence() noexcept {
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            for (double unit : {0.001, 1.0}) {
                const std::string prefix = unit == 0.001 ? "mm-" : "m-";
                const po::Definition payload = ProgramOperandCodecFixture(unit);
                std::vector<std::uint8_t> bytes;
                if (!po::EncodeProgramOperands(payload, bytes)) throw std::invalid_argument("codec");
                const auto graph = ProgramOperandGraph(payload, bytes);
                auto typedPayloadRefusal = [&](const composite_recipe::Definition& candidate) {
                    const auto result = composite_recipe::ValidateV3(candidate,
                        retained_feature::ProductionRegistry(),
                        retained_source::ProductionRegistry());
                    return result.refusal == composite_recipe::V3Refusal::Payload
                        && std::string(result.reason) == "noncanonical-feature-payload";
                };
                auto forged = graph;
                std::get<composite_recipe::FeatureNode>(forged.nodes.back().value)
                    .parameters[100] ^= 1;
                checks[prefix + "bad-digest-typed-payload"] = typedPayloadRefusal(forged);
                forged = graph;
                std::get<composite_recipe::FeatureNode>(forged.nodes.back().value)
                    .parameters.pop_back();
                checks[prefix + "truncated-typed-payload"] = typedPayloadRefusal(forged);
                forged = graph;
                std::get<composite_recipe::FeatureNode>(forged.nodes.back().value)
                    .parameters.push_back(0);
                checks[prefix + "trailing-typed-payload"] = typedPayloadRefusal(forged);

                po::Definition mismatch = payload;
                mismatch.inputs[0].sourceNode = Identifier(0xF0);
                std::vector<std::uint8_t> mismatchBytes;
                if (!po::EncodeProgramOperands(mismatch, mismatchBytes))
                    throw std::invalid_argument("binding mismatch fixture");
                forged = graph;
                std::get<composite_recipe::FeatureNode>(forged.nodes.back().value)
                    .parameters = mismatchBytes;
                checks[prefix + "node-role-substitution-typed-payload"] =
                    typedPayloadRefusal(forged);

                po::Definition duplicate = payload;
                duplicate.inputs[1].sourceNode = duplicate.inputs[0].sourceNode;
                std::vector<std::uint8_t> refused;
                checks[prefix + "duplicate-identities-refused"] =
                    !po::EncodeProgramOperands(duplicate, refused) && refused.empty();
                auto unknownVersion = bytes; unknownVersion[4] = 6;
                po::Definition ignored;
                checks[prefix + "unknown-payload-version-refused"] =
                    !po::DecodeProgramOperands(unknownVersion, ignored);
                auto badKind = bytes; badKind[0] = 'X';
                checks[prefix + "unknown-payload-kind-refused"] =
                    !po::DecodeProgramOperands(badKind, ignored);

                const auto legacy1 = composite_recipe::Probe::LegacyDefinition();
                const auto legacy2 = composite_recipe::Probe::ShellDefinition();
                std::vector<std::uint8_t> before1, before2, after1, after2;
                const bool before = composite_recipe::Encode(legacy1, before1)
                    && composite_recipe::Encode(legacy2, before2);
                std::vector<std::uint8_t> throwaway;
                po::EncodeProgramOperands(payload, throwaway);
                const bool after = composite_recipe::Encode(legacy1, after1)
                    && composite_recipe::Encode(legacy2, after2);
                checks[prefix + "legacy-codec1-envelope-byte-identical"] =
                    before && after && before1 == after1;
                checks[prefix + "legacy-codec2-envelope-byte-identical"] =
                    before && after && before2 == after2;
                checks[prefix + "legacy-codec3-allocation-unchanged"] =
                    part_boolean::RecipeCodecVersion == 3;
                checks[prefix + "legacy-codec4-bytes-and-version-unchanged"] =
                    part_boolean::OriginalsCodecVersion == 4
                    && part_boolean::OriginalsPayloadVersion == 4;
                composite_recipe::Definition cold;
                checks[prefix + "frozen-pre-p3-envelope-cold-reads"] =
                    composite_recipe::Decode(before2, cold)
                    && composite_recipe::Encode(cold, throwaway) && throwaway == before2;
            }
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks ProgramOperandLegacyRegistryEvidence() noexcept {
        namespace po = part_boolean::program_operand;
        Checks checks;
        try {
            for (double unit : {0.001, 1.0}) {
                const std::string prefix = unit == 0.001 ? "mm-" : "m-";
                const po::Definition payload = ProgramOperandCodecFixture(unit);
                std::vector<std::uint8_t> bytes;
                if (!po::EncodeProgramOperands(payload, bytes)) throw std::invalid_argument("codec");
                const auto graph = ProgramOperandGraph(payload, bytes);
                const retained_feature::RegistryView legacy(nullptr, 0);
                const auto refused = composite_recipe::ValidateV3(graph, legacy,
                    retained_source::ProductionRegistry());
                checks[prefix + "legacy-registry-feature-codec-refusal"] =
                    refused.refusal == composite_recipe::V3Refusal::FeatureCodec;
                checks[prefix + "legacy-registry-unknown-feature-codec-reason"] =
                    std::string(refused.reason) == "unknown-feature-codec";
                checks[prefix + "whole-envelope-refused-no-fallback"] = !refused.valid();
            }
        } catch (...) { checks["setup-exception"] = false; }
        return checks;
    }

    static Checks Run(int scenario) noexcept {
        if (scenario == 0) return CodecEvidence();
        if (scenario == 1) return AdmissionEvidence();
        if (scenario == 2) return ColdReopenEvidence();
        if (scenario == 3) return GeometryEvidence();
        if (scenario == 4) return ProgramOperandValueEvidence();
        if (scenario == 5) return ProgramOperandRefusalEvidence();
        if (scenario == 6) return ProgramOperandDisabledEvidence();
        if (scenario == 7) return ProgramOperandReplayEvidence();
        if (scenario == 8) return ProgramOperandReplayRefusalEvidence();
        if (scenario == 9) return ProgramOperandCodecRoundTripEvidence();
        if (scenario == 10) return ProgramOperandCodecRefusalEvidence();
        if (scenario == 11) return ProgramOperandLegacyRegistryEvidence();
        return {{"valid-scenario", false}};
    }
};

} // namespace core3d::retained_program_suffix
#endif
