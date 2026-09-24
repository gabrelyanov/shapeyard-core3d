#pragma once
#if DEBUG

// Fixed DEBUG codec evidence only. This probe has no live-document handle,
// persistence registration, builder, proof, editor, or native admission path.
#include "CompositeRecipeAttribute.hxx"
#include "RetainedRecipeAdmission.hxx"
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Application.hxx>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace core3d::composite_recipe {
struct Probe final {
    using Checks = std::map<std::string, bool>;

    static UUID Identifier(std::uint8_t seed) {
        UUID value{};
        for (std::size_t index = 0; index < value.size(); ++index)
            value[index] = std::uint8_t(seed + index);
        return value;
    }

    static Digest FixedDigest(std::uint8_t seed) {
        Digest value{};
        value.fill(seed);
        return value;
    }

    static profile::Parameters Profile(unsigned schema) {
        profile::Parameters value;
        value.metersPerUnit = 0.001;
        value.definition.plane = 0;
        value.definition.depth = 12;
        if (schema == 3 || schema == 4) {
            ProfileCurveSection section;
            section.outer.identifier = 100;
            section.outer.vertices = {
                {101, gp_Pnt2d(0, 0)}, {102, gp_Pnt2d(40, 0)},
                {103, gp_Pnt2d(40, 30)}, {104, gp_Pnt2d(0, 30)},
            };
            section.outer.segments = {
                {201, 101, 102, ProfileCurveKind::Line, {}, 0, 0, 0},
                {202, 102, 103, ProfileCurveKind::Line, {}, 0, 0, 0},
                {203, 103, 104, ProfileCurveKind::Line, {}, 0, 0, 0},
                {204, 104, 101, ProfileCurveKind::Line, {}, 0, 0, 0},
            };
            value.definition.curves = std::move(section);
        } else {
            value.definition.points = {
                gp_Pnt2d(-0.0, 0), gp_Pnt2d(40, 0),
                gp_Pnt2d(40, 30), gp_Pnt2d(0, 30),
            };
        }
        if (schema == 2 || schema == 4) {
            profile::ConstructionFrame frame;
            frame.values[0] = -0.0;
            value.constructionFrame = frame;
        }
        if (schema == 5) {
            profile::ShellStep shell;
            shell.thickness = 2;
            shell.metersPerLocalUnit = 0.001;
            shell.openings = {5};
            value.shells.push_back(std::move(shell));
        }
        if (schema < 1 || schema > 5 || profile::SchemaFor(value) != int(schema))
            throw std::invalid_argument("profile fixture schema");
        return value;
    }

    static SourceNode Source(unsigned schema, std::uint32_t slot,
                             std::uint8_t seed, const UUID& document) {
        SourceNode source;
        source.node = Identifier(seed);
        source.localID = std::uint64_t(slot) + 1;
        source.original.document = document;
        source.original.entity = Identifier(std::uint8_t(seed + 1));
        source.original.definition = Identifier(std::uint8_t(seed + 2));
        source.original.sourceFeature = Identifier(std::uint8_t(seed + 3));
        source.recipe.kind = RecipeKind::Profile;
        source.recipe.schema = schema;
        std::vector<double> values;
        const auto profile = Profile(schema);
        if (!profile::Encode(profile, values)
            || !EncodeScalarRecipe(source.recipe.kind, schema, values,
                                   source.recipe.bytes))
            throw std::invalid_argument("profile fixture encode");
        source.inputToCarrier.sourceMetersPerUnit = 0.001;
        source.inputToCarrier.carrierMetersPerUnit = 0.001;
        source.commitments.geometry = FixedDigest(std::uint8_t(seed + 4));
        if (!Hash(source.recipe.bytes, source.commitments.recipe))
            throw std::invalid_argument("profile fixture digest");
        source.commitments.placement = FixedDigest(std::uint8_t(seed + 5));
        source.commitments.material = FixedDigest(std::uint8_t(seed + 6));
        source.commitments.groups = FixedDigest(std::uint8_t(seed + 7));
        source.shapeSlot = slot;
        return source;
    }

    static part_boolean::MaterialValue Material(std::uint8_t seed) {
        part_boolean::MaterialValue value;
        value.identifier = Identifier(seed);
        value.baseColorSRGB = {{0.25, 0.5, 0.75, 1.0}};
        value.metallic = 0.125;
        value.roughness = 0.625;
        return value;
    }

    static part_boolean::Definition ShellPayload(const SourceNode& shell,
                                                  const SourceNode& tool) {
        part_boolean::Definition value;
        value.operation = part_boolean::Operation::Subtract;
        const SourceNode* sources[] = {&shell, &tool};
        const part_boolean::InputFamily families[] = {
            part_boolean::InputFamily::ShellProfile,
            part_boolean::InputFamily::AnalyticRectangularPrism,
        };
        for (std::size_t index = 0; index < 2; ++index) {
            auto& input = value.inputs[index];
            const auto& source = *sources[index];
            input.rootNode = source.node;
            input.originalSourceFeature = source.original.sourceFeature;
            input.family = families[index];
            input.commitments.geometry = source.commitments.geometry;
            input.commitments.recipe = source.commitments.recipe;
            input.commitments.placement = source.commitments.placement;
            input.commitments.material = source.commitments.material;
            input.commitments.groups = source.commitments.groups;
            input.originalMaterial = Material(std::uint8_t(90 + index));
            input.originalName = index == 0 ? "Shell" : "Tool";
            input.originalGroups = {index == 0 ? "Body" : "Cutters"};
        }
        value.materials = {
            value.inputs[0].originalMaterial,
            value.inputs[1].originalMaterial,
        };
        value.regions.push_back({Identifier(100), 0,
            part_boolean::RegionKind::OuterWall, 0});
        value.regions.push_back({Identifier(101), 1,
            part_boolean::RegionKind::ToolBoundary, 1});
        value.selectors.push_back({Identifier(102), value.regions[0].region,
            part_boolean::SelectorPolicy::WholeRegion, 1});
        value.shellAliases.push_back({Identifier(103),
            shell.original.sourceFeature, 0, 0});
        return value;
    }

    static Definition LegacyDefinition() {
        Definition value;
        value.schemaVersion = 1;
        value.owner = {Identifier(1), Identifier(2), Identifier(3)};
        SourceNode left = Source(1, 0, 10, value.owner.document);
        SourceNode right = Source(4, 1, 30, value.owner.document);
        FeatureNode feature;
        feature.node = Identifier(50);
        feature.feature = Identifier(51);
        feature.localID = 3;
        feature.kind = PartBooleanFeatureKind;
        feature.codecVersion = PartBooleanFeatureCodec;
        feature.inputs = {left.node, right.node};
        feature.parameters = {0x10, 0x20, 0x30};
        value.outputNode = feature.node;
        value.issuance.nextLocalID = 4;
        value.nodes = {{std::move(left)}, {std::move(right)},
                       {std::move(feature)}};
        return value;
    }

    static Definition ShellDefinition(part_boolean::Definition* payload = nullptr) {
        Definition value;
        value.schemaVersion = 2;
        value.owner = {Identifier(61), Identifier(62), Identifier(63)};
        SourceNode shell = Source(5, 0, 70, value.owner.document);
        SourceNode tool = Source(1, 1, 110, value.owner.document);
        part_boolean::Definition parameters = ShellPayload(shell, tool);
        FeatureNode feature;
        feature.node = Identifier(150);
        feature.feature = Identifier(151);
        feature.localID = 3;
        feature.kind = PartBooleanFeatureKind;
        feature.codecVersion = PartBooleanShellFeatureCodec;
        feature.inputs = {shell.node, tool.node};
        if (!part_boolean::Encode(parameters, feature.parameters))
            throw std::invalid_argument("shell payload fixture encode");
        value.outputNode = feature.node;
        value.issuance.nextLocalID = 4;
        value.nodes = {{std::move(shell)}, {std::move(tool)},
                       {std::move(feature)}};
        if (payload) *payload = std::move(parameters);
        return value;
    }

    static std::shared_ptr<const Payload> Stored(const Definition& definition,
                                                 const std::vector<std::uint8_t>& bytes) {
        auto value = std::make_shared<Payload>();
        value->definition = definition;
        value->bytes = bytes;
        return value;
    }

    static Checks Run() noexcept {
        Checks checks;
        try {
            part_boolean::Definition payload;
            const Definition legacy = LegacyDefinition();
            const Definition shell = ShellDefinition(&payload);
            std::vector<std::uint8_t> legacyBytes, shellBytes, payloadBytes;
            if (!EncodeV1(legacy, legacyBytes) || !EncodeV2(shell, shellBytes)
                || !part_boolean::Encode(payload, payloadBytes))
                throw std::invalid_argument("fixture encode");

            part_boolean::Definition decodedPayload;
            std::vector<std::uint8_t> reencodedPayload;
            checks["sypb2-exact-roundtrip"] = part_boolean::Decode(payloadBytes, decodedPayload)
                && part_boolean::Encode(decodedPayload, reencodedPayload)
                && reencodedPayload == payloadBytes;
            checks["native-admission-remains-disabled"] = !decodedPayload.nativeAdmissionEnabled;
            auto badPayload = payloadBytes;
            badPayload[badPayload.size() - 1] ^= 1;
            checks["sypb2-digest-corruption-refused"] =
                !part_boolean::Decode(badPayload, decodedPayload);
            badPayload = payloadBytes;
            badPayload.push_back(0);
            checks["sypb2-trailing-byte-refused"] =
                !part_boolean::Decode(badPayload, decodedPayload);

            Definition decodedShell;
            std::vector<std::uint8_t> reencodedShell;
            checks["sycr2-exact-roundtrip"] = DecodeV2(shellBytes, decodedShell)
                && EncodeV2(decodedShell, reencodedShell)
                && reencodedShell == shellBytes;
            auto badShell = shellBytes;
            badShell[badShell.size() - 1] ^= 1;
            checks["sycr2-digest-corruption-refused"] = !Decode(badShell, decodedShell);
            badShell = shellBytes;
            badShell.push_back(0);
            checks["sycr2-trailing-byte-refused"] = !Decode(badShell, decodedShell);

            Definition wrongBinding = shell;
            auto changedPayload = payload;
            changedPayload.inputs[0].rootNode = Identifier(170);
            auto& wrongFeature = std::get<FeatureNode>(wrongBinding.nodes.back().value);
            checks["root-binding-refused"] = part_boolean::Encode(changedPayload,
                wrongFeature.parameters) && !Valid(wrongBinding)
                && !EncodeV2(wrongBinding, badShell);

            Definition decodedLegacy;
            std::vector<std::uint8_t> reencodedLegacy;
            checks["sycr1-no-migration"] = DecodeV1(legacyBytes, decodedLegacy)
                && decodedLegacy.schemaVersion == 1
                && EncodeV1(decodedLegacy, reencodedLegacy)
                && reencodedLegacy == legacyBytes;
            for (unsigned schema = 1; schema <= 5; ++schema) {
                std::vector<double> values, decodedValues;
                const auto fixture = Profile(schema);
                SourceRecipe recipe;
                recipe.kind = RecipeKind::Profile;
                recipe.schema = schema;
                std::vector<std::uint8_t> exact;
                const bool encoded = profile::Encode(fixture, values)
                    && EncodeScalarRecipe(recipe.kind, schema, values, recipe.bytes);
                const bool decoded = encoded && ValidRecipe(recipe)
                    && DecodeScalarRecipe(recipe, decodedValues)
                    && EncodeScalarRecipe(recipe.kind, schema, decodedValues, exact);
                checks["profile" + std::to_string(schema) + "-no-migration"] =
                    decoded && decodedValues == values && exact == recipe.bytes;
            }

            retained_recipe::OwnerSnapshot snapshot;
            snapshot.status = retained_recipe::OwnerStatus::CurrentEditable;
            for (const Node& node : shell.nodes) {
                const auto* source = std::get_if<SourceNode>(&node.value);
                if (!source) continue;
                snapshot.sources.push_back({{shell.owner, source->node,
                    source->original.sourceFeature}, source->original, source->recipe,
                    source->inputToCarrier, source->commitments});
            }
            retained_recipe::AdmissionRule rule;
            rule.featureKind = PartBooleanFeatureKind;
            rule.featureCodecVersion = PartBooleanShellFeatureCodec;
            rule.selectorVersion = 1;
            rule.proofProfile = 1;
            rule.orderedSourceKinds = {RecipeKind::Profile, RecipeKind::Profile};
            rule.parameterBounds = FixedDigest(200);
            rule.nativeBuilderInstalled = true;
            rule.nativeProofInstalled = true;
            const auto admission = retained_recipe::Evaluate(snapshot, rule);
            checks["codec2-production-route-refused"] =
                admission.refusal == retained_recipe::Refusal::FeatureKind
                && !admission.admitted();

            struct Holder final {
                Handle(TDocStd_Application) app = new TDocStd_Application();
                Handle(TDocStd_Document) document;
                ~Holder() noexcept {
                    try { if (!document.IsNull()) app->Close(document); } catch (...) {}
                }
            } holder;
            holder.app->NewDocument(TCollection_ExtendedString("BinOcaf"),
                                    holder.document);
            if (holder.document.IsNull()) throw std::invalid_argument("probe document");
            holder.document->SetUndoLimit(4);
            const TDF_Label label = holder.document->Main().FindChild(91, Standard_True);
            holder.document->NewCommand();
            Handle(Attribute) attribute = new Attribute();
            label.AddAttribute(attribute);
            attribute->Backup();
            attribute->value_ = Stored(legacy, legacyBytes);
            if (!holder.document->CommitCommand())
                throw std::invalid_argument("legacy probe commit");
            const auto original = attribute->value_;
            checks["query-byte-preserved"] = original && original->bytes == legacyBytes;
            holder.document->NewCommand();
            attribute->Backup();
            attribute->value_ = Stored(shell, shellBytes);
            if (!holder.document->CommitCommand())
                throw std::invalid_argument("shell probe commit");
            checks["adoption-byte-preserved"] = attribute->value_
                && attribute->value_->bytes == shellBytes
                && original->bytes == legacyBytes;
            checks["undo-byte-preserved"] = holder.document->Undo()
                && attribute->value_ && attribute->value_->bytes == legacyBytes
                && attribute->value_->definition.schemaVersion == 1;
        } catch (...) {
            checks["setup-exception"] = false;
        }
        return checks;
    }
};
} // namespace core3d::composite_recipe

#endif
