#pragma once
#if DEBUG

// Detached codec evidence only. Every positive and refusal below calls the
// production SYPB/3 and SYCR/3 codecs. No document, native owner, execution
// callback, builder, proof, editor, or admission route is created here.
#include "PartBooleanCodecProbe.hxx"
#include <cmath>
#include <map>
#include <string>

namespace core3d::part_boolean_recipe_probe {
using Checks = std::map<std::string, bool>;
using composite_recipe::Definition;
using composite_recipe::FeatureNode;
using composite_recipe::Node;
using composite_recipe::RecipeKind;
using composite_recipe::SourceNode;

inline retained_recipe::UUID Identifier(std::uint8_t seed) {
    return composite_recipe::Probe::Identifier(seed);
}
inline retained_recipe::Digest FixedDigest(std::uint8_t seed) {
    return composite_recipe::Probe::FixedDigest(seed);
}
inline part_boolean::MaterialValue Material(std::uint8_t seed) {
    auto value = composite_recipe::Probe::Material(seed);
    value.baseColorSRGB[0] = double(seed % 7) / 8.0;
    return value;
}

inline bool ProfileValues(double unit, unsigned schema, bool edited,
                          std::vector<double>& values) {
    auto parameters = composite_recipe::Probe::Profile(schema);
    const double scale = 0.001 / unit;
    parameters.metersPerUnit = unit;
    parameters.definition.depth *= scale;
    if (parameters.definition.curves) {
        for (auto& vertex : parameters.definition.curves->outer.vertices) {
            vertex.point.SetX(vertex.point.X() * scale);
            vertex.point.SetY(vertex.point.Y() * scale);
        }
        if (edited) parameters.definition.curves->outer.vertices[1].point.SetX(42 * scale);
    } else if (parameters.definition.circle) {
        parameters.definition.circle->center.SetX(parameters.definition.circle->center.X() * scale);
        parameters.definition.circle->center.SetY(parameters.definition.circle->center.Y() * scale);
        parameters.definition.circle->outerRadius *= scale;
        parameters.definition.circle->innerRadius *= scale;
    } else {
        for (auto& point : parameters.definition.points) {
            point.SetX(point.X() * scale); point.SetY(point.Y() * scale);
        }
        if (edited) parameters.definition.points[1].SetX(42 * scale);
    }
    if (parameters.constructionFrame) parameters.constructionFrame->values[0] = -0.0;
    for (auto& shell : parameters.shells) {
        shell.thickness *= scale; shell.metersPerLocalUnit = unit;
    }
    return profile::SchemaFor(parameters) == int(schema)
        && profile::Encode(parameters, values);
}

inline bool CircleValues(double unit, std::vector<double>& values) {
    profile::Parameters parameters;
    const double scale = 0.001 / unit;
    parameters.metersPerUnit = unit;
    parameters.definition.depth = 30 * scale;
    parameters.definition.circle = ProfileCircularSection{
        gp_Pnt2d(30 * scale, 20 * scale), 15 * scale, 5 * scale};
    return profile::Encode(parameters, values);
}

inline bool EnclosureValues(double unit, bool framed, bool edited,
                            std::vector<double>& values) {
    enclosure::Parameters parameters;
    const double scale = 0.001 / unit;
    parameters.metersPerUnit = unit;
    parameters.definition.dimensions = {
        (edited ? 102 : 100) * scale, 60 * scale, 30 * scale,
        2 * scale, 2 * scale, 4 * scale};
    if (framed) {
        profile::ConstructionFrame frame;
        frame.values[0] = -0.0;
        parameters.definition.constructionFrame = frame;
    }
    return enclosure::Encode(parameters, values);
}

inline bool LoftValues(double unit, bool framed, bool edited,
                       std::vector<double>& values) {
    rectangular_loft::Definition definition;
    const double scale = 0.001 / unit;
    definition.loftIdentifier = 1;
    definition.correspondence = {{10, 11, 12, 13}};
    definition.dimensionMetersPerUnit = unit;
    for (std::size_t index = 0; index < 3; ++index) {
        rectangular_loft::Station station;
        station.identifier = std::uint32_t(100 + index);
        for (std::size_t corner = 0; corner < 4; ++corner) {
            station.cornerIdentifiers[corner] = std::uint32_t(200 + 4 * index + corner);
            station.correspondence[corner] = std::uint32_t(10 + corner);
        }
        station.z = double(index) * 60 * scale;
        station.centerX = (index == 1 ? 6 : 0) * scale;
        station.width = ((index == 1 ? 30 : 20) + (edited && index == 1 ? 2 : 0)) * scale;
        station.depth = (index == 1 ? 20 : 12) * scale;
        definition.stations.push_back(station);
    }
    if (framed) {
        profile::ConstructionFrame frame;
        frame.values[0] = -0.0;
        definition.constructionFrame = frame;
    }
    return loft_persistence::Encode(definition, values);
}

inline bool Values(part_boolean::RecipeInputFamily family, double unit, bool edited,
                   std::vector<double>& values, std::uint32_t& schema) {
    if (family == part_boolean::RecipeInputFamily::Profile) {
        schema = 1; return ProfileValues(unit, schema, edited, values);
    }
    if (family == part_boolean::RecipeInputFamily::Enclosure) {
        schema = 2; return EnclosureValues(unit, true, edited, values);
    }
    if (family == part_boolean::RecipeInputFamily::RectangularLoft) {
        schema = loft_persistence::Schema; return LoftValues(unit, true, edited, values);
    }
    return false;
}

inline RecipeKind Kind(part_boolean::RecipeInputFamily family) {
    if (family == part_boolean::RecipeInputFamily::Profile) return RecipeKind::Profile;
    if (family == part_boolean::RecipeInputFamily::Enclosure) return RecipeKind::Enclosure;
    return RecipeKind::RectangularLoft;
}

inline SourceNode Source(part_boolean::RecipeInputFamily family, double unit,
                         bool edited, std::uint32_t slot, std::uint8_t seed,
                         const retained_recipe::UUID& document) {
    SourceNode source;
    source.node = Identifier(seed); source.localID = std::uint64_t(slot) + 1;
    source.original = {document, Identifier(std::uint8_t(seed + 1)),
        Identifier(std::uint8_t(seed + 2)), Identifier(std::uint8_t(seed + 3))};
    source.recipe.kind = Kind(family);
    std::vector<double> values;
    if (!Values(family, unit, edited, values, source.recipe.schema)
        || !composite_recipe::EncodeScalarRecipe(source.recipe.kind, source.recipe.schema,
                                                 values, source.recipe.bytes))
        throw std::invalid_argument("recipe source");
    source.inputToCarrier.sourceMetersPerUnit = unit;
    source.inputToCarrier.carrierMetersPerUnit = unit;
    source.inputToCarrier.matrix[3] = edited ? 0.25 : -0.0;
    source.commitments.geometry = FixedDigest(std::uint8_t(seed + 4));
    if (!composite_recipe::Hash(source.recipe.bytes, source.commitments.recipe))
        throw std::invalid_argument("recipe digest");
    source.shapeSlot = slot;
    return source;
}

inline part_boolean::RecipeInput PayloadInput(const SourceNode& source,
        part_boolean::RecipeInputRole role, part_boolean::RecipeInputFamily family,
        std::uint8_t seed) {
    part_boolean::RecipeInput input;
    input.role = role; input.family = family;
    input.originallyVisible = role == part_boolean::RecipeInputRole::Left;
    input.sourceNode = source.node; input.sourceLocalID = source.localID;
    input.original = source.original; input.shapeSlot = source.shapeSlot;
    input.recipeSchema = source.recipe.schema;
    input.sourceMetersPerUnit = source.inputToCarrier.sourceMetersPerUnit;
    input.carrierMetersPerUnit = source.inputToCarrier.carrierMetersPerUnit;
    input.inputToCarrier = source.inputToCarrier.matrix;
    input.commitments.geometry = source.commitments.geometry;
    input.commitments.recipe = source.commitments.recipe;
    input.originalMaterial = Material(seed);
    input.originalName = role == part_boolean::RecipeInputRole::Left ? "Left source" : "Right source";
    input.originalGroups = {role == part_boolean::RecipeInputRole::Left ? "Body" : "Tools"};
    if (!composite_recipe::DecodeScalarRecipe(source.recipe, input.recipeScalars)
        || !part_boolean::RecipePlacementDigest(input, input.commitments.placement)
        || !part_boolean::RecipeMaterialDigest(input.originalMaterial, input.commitments.material)
        || !part_boolean::RecipeGroupsDigest(input.originalGroups, input.commitments.groups))
        throw std::invalid_argument("payload input");
    return input;
}

inline Definition Graph(part_boolean::RecipeInputFamily leftFamily,
                        part_boolean::RecipeInputFamily rightFamily,
                        double unit, bool editLeft = false, bool editRight = false,
                        bool swappedSlots = false) {
    Definition graph = composite_recipe::MakeV3Definition();
    graph.owner = {Identifier(1), Identifier(21), Identifier(41)};
    const std::uint32_t leftSlot = swappedSlots ? 1 : 0;
    const std::uint32_t rightSlot = swappedSlots ? 0 : 1;
    SourceNode left = Source(leftFamily, unit, editLeft, leftSlot, 70, graph.owner.document);
    SourceNode right = Source(rightFamily, unit, editRight, rightSlot, 120, graph.owner.document);
    // Local IDs are issuance identities, not physical shape-slot ordinals.
    left.localID = 1; right.localID = 2;
    part_boolean::RecipeDefinition payload;
    payload.operation = part_boolean::Operation::Subtract;
    payload.owner = graph.owner;
    payload.booleanNode = Identifier(170); payload.feature = Identifier(190);
    payload.inputs[0] = PayloadInput(left, part_boolean::RecipeInputRole::Left, leftFamily, 220);
    payload.inputs[1] = PayloadInput(right, part_boolean::RecipeInputRole::Right, rightFamily, 230);
    left.commitments = {payload.inputs[0].commitments.geometry,
        payload.inputs[0].commitments.recipe, payload.inputs[0].commitments.placement,
        payload.inputs[0].commitments.material, payload.inputs[0].commitments.groups};
    right.commitments = {payload.inputs[1].commitments.geometry,
        payload.inputs[1].commitments.recipe, payload.inputs[1].commitments.placement,
        payload.inputs[1].commitments.material, payload.inputs[1].commitments.groups};
    FeatureNode feature;
    feature.node = payload.booleanNode; feature.feature = payload.feature; feature.localID = 3;
    feature.kind = composite_recipe::PartBooleanFeatureKind;
    feature.codecVersion = composite_recipe::PartBooleanRecipeFeatureCodec;
    feature.inputs = {left.node, right.node};
    if (!part_boolean::EncodeRecipe(payload, feature.parameters))
        throw std::invalid_argument("recipe payload");
    graph.outputNode = feature.node; graph.issuance.nextLocalID = 4;
    graph.nodes = {{std::move(left)}, {std::move(right)}, {std::move(feature)}};
    return graph;
}

inline bool Reseal(std::vector<std::uint8_t>& bytes) {
    if (bytes.size() < 32) return false;
    std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
    retained_recipe::Digest digest{};
    if (!retained_solid::Hash(body, digest)) return false;
    std::copy(digest.begin(), digest.end(), bytes.end() - 32); return true;
}

inline std::size_t InputSize(const part_boolean::RecipeInput& input) {
    std::size_t size = 520 + 2 + input.originalName.size() + 2 + 4
        + input.recipeScalars.size() * 8;
    for (const auto& group : input.originalGroups) size += 2 + group.size();
    return size;
}

inline bool LargeProfileValues(std::vector<double>& values) {
    profile::Parameters parameters; parameters.metersPerUnit = 0.001;
    parameters.definition.depth = 12;
    ProfileCurveSection section; section.outer.identifier = 1;
    constexpr std::size_t count = 77;
    for (std::size_t index = 0; index < count; ++index) {
        const double angle = 2 * std::acos(-1.0) * double(index) / double(count);
        section.outer.vertices.push_back({std::uint32_t(10 + index),
            gp_Pnt2d(20 + 15 * std::cos(angle), 20 + 15 * std::sin(angle))});
    }
    for (std::size_t index = 0; index < count; ++index)
        section.outer.segments.push_back({std::uint32_t(100 + index),
            std::uint32_t(10 + index), std::uint32_t(10 + (index + 1) % count),
            ProfileCurveKind::Line, {}, 0, 0, 0});
    parameters.definition.curves = std::move(section);
    return profile::Encode(parameters, values) && values.size() == 934;
}

inline Checks Run() noexcept {
    Checks checks;
    try {
        bool pairs = true, metadata = true, registry = true;
        for (double unit : {0.001, 1.0}) {
            for (unsigned left = 1; left <= 3; ++left) for (unsigned right = 1; right <= 3; ++right) {
                const auto leftFamily = part_boolean::RecipeInputFamily(left);
                const auto rightFamily = part_boolean::RecipeInputFamily(right);
                Definition graph = Graph(leftFamily, rightFamily, unit, false, false,
                    ((left + right) & 1) != 0);
                std::vector<std::uint8_t> bytes, exact;
                Definition decoded;
                pairs = pairs && composite_recipe::EncodeV3(graph, bytes)
                    && composite_recipe::DecodeV3(bytes, decoded)
                    && composite_recipe::EncodeV3(decoded, exact) && exact == bytes;
                const auto& feature = std::get<FeatureNode>(decoded.nodes.back().value);
                part_boolean::RecipeDefinition payload;
                metadata = metadata && part_boolean::DecodeRecipe(feature.parameters, payload)
                    && unsigned(payload.inputs[0].family) == left
                    && unsigned(payload.inputs[1].family) == right
                    && payload.inputs[0].originallyVisible
                    && !payload.inputs[1].originallyVisible
                    && payload.inputs[0].originalName == "Left source"
                    && payload.inputs[1].originalGroups == std::vector<std::string>{"Tools"}
                    && payload.inputs[0].shapeSlot != payload.inputs[1].shapeSlot;
            }
        }
        checks["all-nine-pairs-both-units-canonical"] = pairs;
        checks["metadata-slots-and-five-commitments-retained"] = metadata;

        bool nativeSequences = true;
        for (double unit : {0.001, 1.0}) {
            for (unsigned schema = 1; schema <= 5; ++schema) {
                std::vector<double> values; part_boolean::RecipeInput input;
                input.family = part_boolean::RecipeInputFamily::Profile;
                input.recipeSchema = schema; input.sourceMetersPerUnit = unit;
                nativeSequences = nativeSequences && ProfileValues(unit, schema, false, values);
                input.recipeScalars = values;
                nativeSequences = nativeSequences && part_boolean::ValidNativeRecipe(input);
            }
            std::vector<double> values; part_boolean::RecipeInput input;
            input.family = part_boolean::RecipeInputFamily::Profile; input.recipeSchema = 1;
            input.sourceMetersPerUnit = unit;
            nativeSequences = nativeSequences && CircleValues(unit, values);
            input.recipeScalars = values;
            nativeSequences = nativeSequences && part_boolean::ValidNativeRecipe(input);
            for (bool framed : {false, true}) {
                input.family = part_boolean::RecipeInputFamily::Enclosure;
                input.recipeSchema = framed ? 2 : 1; input.recipeScalars.clear();
                nativeSequences = nativeSequences && EnclosureValues(unit, framed, false, input.recipeScalars)
                    && part_boolean::ValidNativeRecipe(input);
            }
            input.family = part_boolean::RecipeInputFamily::RectangularLoft;
            input.recipeSchema = loft_persistence::Schema; input.recipeScalars.clear();
            nativeSequences = nativeSequences && LoftValues(unit, true, false, input.recipeScalars)
                && part_boolean::ValidNativeRecipe(input);
        }
        checks["complete-native-sequences-and-signed-zero"] = nativeSequences;

        Definition base = Graph(part_boolean::RecipeInputFamily::Profile,
            part_boolean::RecipeInputFamily::Enclosure, 0.001);
        Definition leftEdit = Graph(part_boolean::RecipeInputFamily::Profile,
            part_boolean::RecipeInputFamily::Enclosure, 0.001, true, false);
        Definition rightEdit = Graph(part_boolean::RecipeInputFamily::Profile,
            part_boolean::RecipeInputFamily::Enclosure, 0.001, false, true);
        part_boolean::RecipeDefinition basePayload, leftPayload, rightPayload;
        auto payloadOf = [](const Definition& value, part_boolean::RecipeDefinition& output) {
            return part_boolean::DecodeRecipe(
                std::get<FeatureNode>(value.nodes.back().value).parameters, output);
        };
        checks["independent-left-right-scalar-edits"] = payloadOf(base, basePayload)
            && payloadOf(leftEdit, leftPayload) && payloadOf(rightEdit, rightPayload)
            && !part_boolean::SameScalarBits(basePayload.inputs[0].recipeScalars,
                                             leftPayload.inputs[0].recipeScalars)
            && part_boolean::SameScalarBits(basePayload.inputs[1].recipeScalars,
                                            leftPayload.inputs[1].recipeScalars)
            && part_boolean::SameScalarBits(basePayload.inputs[0].recipeScalars,
                                            rightPayload.inputs[0].recipeScalars)
            && !part_boolean::SameScalarBits(basePayload.inputs[1].recipeScalars,
                                             rightPayload.inputs[1].recipeScalars);

        const auto& production = retained_feature::ProductionRegistry();
        const auto* descriptor = production.find({part_boolean::FeatureKind,
            part_boolean::RecipeCodecVersion});
        registry = production.size() == 4 && descriptor
            && descriptor->codec.graphMajor == 3
            && descriptor->codec.maximumPayloadBytes == part_boolean::MaximumPayloadBytes
            && descriptor->codec.inputCount == 2 && !descriptor->execution.installed()
            && !descriptor->execution.buildDetached && !descriptor->execution.proveFamily
            && !descriptor->execution.verifyFixedPoint
            && !descriptor->execution.editorRouteInstalled
            && !descriptor->execution.dependencyRouteInstalled;
        checks["persistence-descriptor-has-empty-execution"] = registry;

        std::vector<std::uint8_t> graphBytes;
        Definition ownerCross = base;
        auto& ownerFeature = std::get<FeatureNode>(ownerCross.nodes.back().value);
        part_boolean::RecipeDefinition ownerPayload;
        bool ownerJoin = part_boolean::DecodeRecipe(ownerFeature.parameters, ownerPayload);
        ownerPayload.owner.entity = Identifier(250);
        ownerJoin = ownerJoin && part_boolean::EncodeRecipe(ownerPayload, ownerFeature.parameters)
            && !composite_recipe::ValidateV3(ownerCross, retained_feature::ProductionRegistry(),
                                             retained_source::ProductionRegistry()).valid()
            && !composite_recipe::EncodeV3(ownerCross, graphBytes);
        checks["new-key-owner-join-refuses-cross-owner"] = ownerJoin;

        Definition cross = base;
        auto& crossFeature = std::get<FeatureNode>(cross.nodes.back().value);
        part_boolean::RecipeDefinition crossPayload;
        bool crossJoins = part_boolean::DecodeRecipe(crossFeature.parameters, crossPayload);
        crossPayload.inputs[0].sourceLocalID += 10;
        crossJoins = crossJoins && part_boolean::EncodeRecipe(crossPayload, crossFeature.parameters)
            && !composite_recipe::ValidateV3(cross, retained_feature::ProductionRegistry(),
                                             retained_source::ProductionRegistry()).valid();
        cross = base; auto& slotFeature = std::get<FeatureNode>(cross.nodes.back().value);
        crossJoins = crossJoins && part_boolean::DecodeRecipe(slotFeature.parameters, crossPayload);
        std::swap(crossPayload.inputs[0].shapeSlot, crossPayload.inputs[1].shapeSlot);
        crossJoins = crossJoins && part_boolean::EncodeRecipe(crossPayload, slotFeature.parameters)
            && !composite_recipe::ValidateV3(cross, retained_feature::ProductionRegistry(),
                                             retained_source::ProductionRegistry()).valid();
        checks["registry-cross-joins-refused"] = crossJoins;

        std::vector<std::uint8_t> payloadBytes;
        bool corrupt = part_boolean::EncodeRecipe(basePayload, payloadBytes);
        auto bad = payloadBytes; bad.back() ^= 1;
        part_boolean::RecipeDefinition decodedPayload;
        corrupt = corrupt && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; bad[8] = 9; corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; bad[115] = 1; corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; bad[112] = 2; corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; bad[208] = 99; corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; std::fill_n(bad.begin() + 212, 8, 0); corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        bad = payloadBytes; bad[204] = 1; corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        const std::size_t scalarCount = 112 + 520 + 2 + basePayload.inputs[0].originalName.size()
            + 2 + 2 + basePayload.inputs[0].originalGroups.front().size();
        bad = payloadBytes; std::fill_n(bad.begin() + scalarCount, 4, 0); corrupt = corrupt && Reseal(bad)
            && !part_boolean::DecodeRecipe(bad, decodedPayload);
        checks["field-count-reserved-seal-role-recipe-unit-slot-refused"] = corrupt;

        part_boolean::RecipeDefinition limit = basePayload;
        std::vector<double> large;
        bool bounds = LargeProfileValues(large);
        for (std::size_t index = 0; index < 2; ++index) {
            limit.inputs[index].family = part_boolean::RecipeInputFamily::Profile;
            limit.inputs[index].recipeSchema = 3; limit.inputs[index].recipeScalars = large;
            std::vector<std::uint8_t> recipe;
            bounds = bounds && composite_recipe::EncodeScalarRecipe(RecipeKind::Profile, 3, large, recipe)
                && composite_recipe::Hash(recipe, limit.inputs[index].commitments.recipe);
        }
        limit.inputs[0].originalName.clear(); limit.inputs[1].originalName.clear();
        limit.inputs[0].originalGroups.clear(); limit.inputs[1].originalGroups.clear();
        limit.inputs[0].commitments.groups = {}; limit.inputs[1].commitments.groups = {};
        bounds = bounds && part_boolean::RecipeGroupsDigest({}, limit.inputs[0].commitments.groups)
            && part_boolean::RecipeGroupsDigest({}, limit.inputs[1].commitments.groups);
        std::vector<std::uint8_t> small;
        bounds = bounds && part_boolean::EncodeRecipe(limit, small) && small.size() <= 16384;
        std::size_t padding = 16384 - small.size();
        limit.inputs[0].originalName.assign(std::min<std::size_t>(128, padding), 'L');
        padding -= limit.inputs[0].originalName.size();
        limit.inputs[1].originalName.assign(padding, 'R');
        std::vector<std::uint8_t> exactLimit;
        bounds = bounds && limit.inputs[1].originalName.size() <= 128
            && part_boolean::EncodeRecipe(limit, exactLimit) && exactLimit.size() == 16384;
        auto overflow = limit;
        if (overflow.inputs[1].originalName.size() < 128)
            overflow.inputs[1].originalName.push_back('X');
        else overflow.inputs[0].originalName.push_back('X');
        std::vector<std::uint8_t> refused;
        bounds = bounds && !part_boolean::EncodeRecipe(overflow, refused);
        auto tooLargePayload = exactLimit; tooLargePayload.push_back(0);
        std::vector<std::uint8_t> tooLargeGraph(composite_recipe::MaximumEnvelopeBytes + 1, 0);
        Definition ignored;
        bounds = bounds && !part_boolean::DecodeRecipe(tooLargePayload, decodedPayload)
            && !composite_recipe::Decode(tooLargeGraph, ignored)
            && part_boolean::MaximumPayloadBytes == 16 * 1024
            && composite_recipe::MaximumEnvelopeBytes == 64 * 1024
            && composite_recipe::MaximumDocumentAggregateBytes == 8 * 1024 * 1024;
        checks["unchanged-payload-envelope-document-bounds"] = bounds;

        bool oldBytes = true;
        for (double unit : {0.001, 1.0}) {
            part_boolean::AnalyticDefinition analytic;
            analytic.operation = part_boolean::Operation::Union;
            for (std::size_t index = 0; index < 2; ++index) {
                auto& input = analytic.inputs[index];
                input.rootNode = Identifier(std::uint8_t(20 + index * 20));
                input.originalSourceFeature = Identifier(std::uint8_t(30 + index * 20));
                input.commitments = {FixedDigest(std::uint8_t(1 + index)),
                    FixedDigest(std::uint8_t(3 + index)), FixedDigest(std::uint8_t(5 + index)),
                    FixedDigest(std::uint8_t(7 + index)), FixedDigest(std::uint8_t(9 + index))};
                input.dimensions = {{40, 30, 20}}; input.metersPerUnit = unit;
                input.originalMaterial = Material(std::uint8_t(100 + index));
                input.originalName = "Legacy"; input.originalGroups = {"Old"};
            }
            std::vector<std::uint8_t> bytes, exact;
            part_boolean::AnalyticDefinition decoded;
            oldBytes = oldBytes && part_boolean::EncodeAnalytic(analytic, bytes)
                && part_boolean::DecodeAnalytic(bytes, decoded)
                && part_boolean::EncodeAnalytic(decoded, exact) && exact == bytes;
        }
        part_boolean::Definition shellPayload;
        const Definition shell = composite_recipe::Probe::ShellDefinition(&shellPayload);
        std::vector<std::uint8_t> shellBytes, shellExact;
        part_boolean::Definition decodedShell;
        oldBytes = oldBytes && part_boolean::Encode(shellPayload, shellBytes)
            && part_boolean::Decode(shellBytes, decodedShell)
            && part_boolean::Encode(decodedShell, shellExact) && shellExact == shellBytes;
        checks["legacy-bytes-reencode-identically-both-units"] = oldBytes;
    } catch (...) {
        checks["setup-exception"] = false;
    }
    return checks;
}
} // namespace core3d::part_boolean_recipe_probe

#endif
