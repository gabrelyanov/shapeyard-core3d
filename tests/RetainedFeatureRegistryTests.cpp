#include "RetainedFeatureOwner.hxx"
#include "RetainedFeatureRegistryProbe.hxx"
#include "PartBooleanCodecProbe.hxx"
#include "RetainedRecipeProbe.hxx"
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace core3d;

namespace {
retained_recipe::UUID id(std::uint8_t seed) { return retained_recipe::Probe::ID(seed); }

composite_recipe::SourceRecipe profileRecipe(double unit) {
    profile::Parameters value; value.metersPerUnit = unit;
    value.definition.plane = 0; value.definition.depth = 10;
    value.definition.points = {gp_Pnt2d(0,0), gp_Pnt2d(10,0), gp_Pnt2d(10,10), gp_Pnt2d(0,10)};
    std::vector<double> scalars; assert(profile::Encode(value, scalars));
    composite_recipe::SourceRecipe recipe;
    recipe.kind = composite_recipe::RecipeKind::Profile;
    recipe.schema = std::uint32_t(profile::SchemaFor(value));
    assert(composite_recipe::EncodeScalarRecipe(recipe.kind, recipe.schema, scalars, recipe.bytes));
    return recipe;
}

composite_recipe::Definition syntheticGraph(double unit, bool suffix = false) {
    auto definition = composite_recipe::MakeV3Definition();
    definition.owner = {id(1), id(2), id(3)};
    composite_recipe::SourceNode source; source.node = id(4); source.localID = 1;
    source.original = {id(1), id(2), id(3), id(5)}; source.recipe = profileRecipe(unit);
    source.inputToCarrier = retained_recipe::Probe::Placement(unit); source.shapeSlot = 0;
    source.commitments = retained_recipe::Probe::Commit(20, source.recipe.bytes);
    composite_recipe::FeatureNode first; first.node = id(6); first.feature = id(7); first.localID = 2;
    first.kind = retained_feature::RegistryProbe::SyntheticKind;
    first.codecVersion = retained_feature::RegistryProbe::SyntheticCodec;
    first.inputs = {source.node}; first.parameters = {9};
    definition.nodes = {{source}, {first}}; definition.outputNode = first.node;
    definition.issuance.nextLocalID = 3;
    if (suffix) {
        composite_recipe::FeatureNode second; second.node = id(8); second.feature = id(9); second.localID = 3;
        second.kind = first.kind; second.codecVersion = first.codecVersion;
        second.inputs = {first.node}; second.parameters = {10};
        definition.nodes.push_back({second}); definition.outputNode = second.node;
        definition.issuance.nextLocalID = 4;
    }
    return definition;
}

retained_feature::SourceValue sourceValue(const composite_recipe::Definition& definition) {
    retained_feature::SourceValue result;
    result.node = std::get<composite_recipe::SourceNode>(definition.nodes.front().value).node;
    result.value.shape = retained_feature::ShapeKind::Solid;
    result.value.detachedShape = {1,2,3,4};
    assert(composite_recipe::Hash(result.value.detachedShape, result.value.geometry));
    const std::vector<std::uint8_t> proof{5,6,7};
    assert(composite_recipe::Hash(proof, result.value.familyProof));
    return result;
}

class CapturingPort final : public retained_feature::DocumentPort {
public:
    CapturingPort() : graph_(syntheticGraph(.001)), source_(sourceValue(graph_)) {
        retained_feature::ReplayBudget budget;
        const auto replay = retained_feature::ReplayDetached(graph_,
            retained_feature::RegistryProbe::registry(), retained_source::ProductionRegistry(),
            {source_}, budget);
        assert(replay.replayed());
        output_ = replay.output.geometry;
    }

    bool capture(retained_feature::CompleteFence& fence,
                 composite_recipe::Definition& graph,
                 std::vector<retained_feature::SourceValue>& sources) noexcept override {
        fence = {}; graph = graph_; sources = {source_};
        fence.graph.fill(1); fence.output = output_; fence.identities.fill(2);
        fence.metadata.fill(3); fence.history.fill(4);
        retained_recipe::RevisionFence revision;
        revision.documentGeneration = 1; revision.modelRevision = 1;
        revision.effectiveMetersPerUnit = .001;
        revision.ownerShape = fence.output; revision.ownerRecipe = fence.graph;
        revision.ownerMaterial = fence.metadata;
        retained_recipe::DependencyRead dependency;
        const auto& source = std::get<composite_recipe::SourceNode>(graph_.nodes.front().value);
        dependency.locator = {graph_.owner, source.node, source.original.sourceFeature};
        dependency.geometry = source.commitments.geometry;
        dependency.recipe = source.commitments.recipe;
        dependency.placement = source.commitments.placement;
        dependency.material = source.commitments.material;
        dependency.groups = source.commitments.groups;
        revision.dependencies.push_back(dependency);
        return retained_feature::InstallCapturedRevision(
            revision.effectiveMetersPerUnit, std::move(revision), fence) && fence.valid();
    }

    bool stageAndReadBack(const retained_feature::PreparedChange&) noexcept override { return false; }
    bool commitOneCommand(int&) noexcept override { return false; }
    bool abortAndProve(const retained_feature::CompleteFence&) noexcept override { return false; }
    bool reconcile(const retained_feature::CompleteFence&, const retained_feature::CompleteFence&,
                   retained_feature::OwnerOutcome&) noexcept override { return false; }

private:
    composite_recipe::Definition graph_;
    retained_feature::SourceValue source_;
    retained_recipe::Digest output_{};
};

void testOrdinaryPrepareCapturesUnitDigestInInstalledRevision() {
    CapturingPort port;
    retained_feature::Owner owner(port, 1, retained_feature::RegistryProbe::registry(),
                                  retained_source::ProductionRegistry());
    auto graph = syntheticGraph(.001);
    const auto& feature = std::get<composite_recipe::FeatureNode>(graph.nodes.back().value);
    retained_feature::MutationRequest request;
    request.kind = retained_feature::MutationKind::EditFeature;
    request.target = {graph.owner, feature.node, feature.feature};
    request.codec = {feature.kind, feature.codecVersion};
    request.typedParameters = feature.parameters;
    retained_feature::OwnerReceipt receipt;
    const auto prepared = owner.prepare(request, retained_feature::ReplayBudget(), receipt);
    assert(!prepared);
    assert(receipt.reason == "no-op");
}

std::vector<std::uint8_t> readFrozen(const std::string& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("required frozen legacy corpus blob is absent: " + path);
    return {std::istreambuf_iterator<char>(stream), {}};
}

std::string digestHex(const std::vector<std::uint8_t>& bytes) {
    composite_recipe::Digest digest{};
    if (!composite_recipe::Hash(bytes, digest)) throw std::runtime_error("frozen blob hash failed");
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (const auto byte : digest) text << std::setw(2) << unsigned(byte);
    return text.str();
}

bool manifestPins(const std::string& manifest, const std::string& name,
                  const std::vector<std::uint8_t>& bytes) {
    const auto id = manifest.find("\"id\":\"" + name + "\"");
    if (id == std::string::npos) return false;
    const auto begin = manifest.rfind('{', id);
    const auto end = manifest.find('}', id);
    if (begin == std::string::npos) return false;
    if (end == std::string::npos) return false;
    const auto row = manifest.substr(begin, end - begin);
    return row.find("\"path\":\"" + name + "\"") != std::string::npos
        && row.find("\"byte_length\":" + std::to_string(bytes.size())) != std::string::npos
        && row.find("\"sha256\":\"" + digestHex(bytes) + "\"") != std::string::npos;
}

void testFrozenLegacyCompositeV1V2AndD66Boundary() {
    const char* configured = std::getenv("SHAPEYARD_FROZEN_LEGACY_CORPUS");
    if (configured == nullptr || *configured == '\0')
        throw std::runtime_error("SHAPEYARD_FROZEN_LEGACY_CORPUS is required; golden tests refuse to run without the frozen manifest");
    const std::string fixtures = std::string(configured) + "/";
    const auto manifestBytes = readFrozen(fixtures + "legacy-corpus-manifest.json");
    const std::string manifest(manifestBytes.begin(), manifestBytes.end());
    assert(manifest.find("\"schema\":\"shapeyard.pre-g0-legacy-corpus.v1\"") != std::string::npos);
    assert(manifest.find("\"capture_complete\":true") != std::string::npos);
    assert(manifest.find("\"full_design_corpus_complete\":false") != std::string::npos);
    const std::array<const char*, 13> deferred{{
        "syet1-profile", "syet2-profile-selector-absent", "syet2-profile-selector-present",
        "syet1-enclosure", "syet2-enclosure-selector-absent", "syet2-enclosure-selector-present",
        "r2-base-syrs-1-1", "r2-base-syrs-2-1", "r2-base-syrs-2-2",
        "r2-base-syrs-2-3", "r2-base-syrs-2-4", "r2-base-a1-sycr1", "r2-m3-provenance"
    }};
    for (const auto* id : deferred) {
        assert(manifest.find("\"id\":\"" + std::string(id) + "\"") != std::string::npos);
        assert(manifest.find("\"path\":\"" + std::string(id)) == std::string::npos);
    }
    const auto frozenV1 = readFrozen(fixtures + "composite-v1-opaque.bin");
    const auto frozenV2 = readFrozen(fixtures + "composite-v2-shell.bin");
    assert(manifestPins(manifest, "composite-v1-opaque.bin", frozenV1));
    assert(manifestPins(manifest, "composite-v2-shell.bin", frozenV2));
    std::vector<std::uint8_t> v1, v2;
    assert(composite_recipe::EncodeV1(composite_recipe::Probe::LegacyDefinition(), v1));
    assert(composite_recipe::EncodeV2(composite_recipe::Probe::ShellDefinition(), v2));
    assert(v1 == frozenV1);
    assert(v2 == frozenV2);
}

void testV3UnknownKindCodecAndSourceRefuseBeforeAdoption() {
    const auto& registry = retained_feature::RegistryProbe::registry();
    const auto& sources = retained_source::ProductionRegistry();
    auto graph = syntheticGraph(.001);
    assert(composite_recipe::ValidateV3(graph, registry, sources).valid());
    std::vector<std::uint8_t> bytes;
    assert(composite_recipe::EncodeV3(graph, registry, sources, bytes));
    composite_recipe::Definition refused;
    assert(!composite_recipe::Decode(bytes, refused));
    assert(refused.nodes.empty());
    std::get<composite_recipe::FeatureNode>(graph.nodes.back().value).codecVersion = 2;
    assert(composite_recipe::ValidateV3(graph, registry, sources).refusal
        == composite_recipe::V3Refusal::FeatureCodec);
    graph = syntheticGraph(.001);
    auto& source = std::get<composite_recipe::SourceNode>(graph.nodes.front().value);
    source.recipe.kind = composite_recipe::RecipeKind::ReservedPlanarSplineProfile;
    assert(composite_recipe::ValidateV3(graph, registry, sources).refusal
        == composite_recipe::V3Refusal::SourceCodec);
}

void testV3CanonicalPayloadOrderIdentityAndBounds() {
    const auto& registry = retained_feature::RegistryProbe::registry();
    const auto& sources = retained_source::ProductionRegistry();
    auto graph = syntheticGraph(1.0, true); std::vector<std::uint8_t> bytes, exact;
    assert(composite_recipe::EncodeV3(graph, registry, sources, bytes));
    composite_recipe::Definition decoded;
    assert(composite_recipe::DecodeV3(bytes, registry, sources, decoded));
    assert(composite_recipe::EncodeV3(decoded, registry, sources, exact)); assert(bytes == exact);
    auto discarded = graph; discarded.outputNode = std::get<composite_recipe::FeatureNode>(discarded.nodes[1].value).node;
    assert(composite_recipe::ValidateV3(discarded, registry, sources).refusal
        == composite_recipe::V3Refusal::Issuance);
    auto oversized = graph;
    std::get<composite_recipe::FeatureNode>(oversized.nodes.back().value).parameters.assign(2, 1);
    assert(composite_recipe::ValidateV3(oversized, registry, sources).refusal
        == composite_recipe::V3Refusal::Payload);
    retained_feature::Entry duplicateEntries[2]{};
    duplicateEntries[0] = duplicateEntries[1] = {{
        {retained_feature::RegistryProbe::SyntheticKind, 1}, 3, 1,
        {retained_feature::ShapeKind::Solid}, 1, retained_feature::ShapeKind::Solid,
        retained_feature::RegistryProbe::canonical}, {}};
    assert(!retained_feature::RegistryView(duplicateEntries, 2).valid());
    const retained_feature::Key tombstone{retained_feature::RegistryProbe::SyntheticKind, 1};
    assert(!retained_feature::RegistryView(duplicateEntries, 1, &tombstone, 1).valid());
}

void testV3RegistryDoesNotInstallProductFeatures() {
    const auto& registry = retained_feature::ProductionRegistry();
    assert(registry.valid()); assert(registry.size() == 2);
    for (const auto key : {retained_feature::ReservedD67FeatureKind1001,
            retained_feature::ReservedD67FeatureKind3001,
            retained_feature::ExistingChamferAdapterKind,
            retained_feature::ExistingConstantFilletAdapterKind,
            retained_feature::VariableRadiusFilletKind,
            retained_feature::DraftFacesKind,
            retained_feature::SplineProfileRevolveKind})
        assert(registry.find({key, 1}) == nullptr);
    assert(!registry.find({composite_recipe::PartBooleanFeatureKind,
        composite_recipe::PartBooleanFeatureCodec})->execution.installed());
    assert(retained_source::ProductionRegistry().find({
        retained_source::ReservedPlanarSplineProfileKind, 1}) == nullptr);
}

void testV3ReplaySuffixFailureRestoresWholeDocument() {
    const auto graph = syntheticGraph(.001, true); const auto source = sourceValue(graph);
    retained_feature::ReplayBudget complete;
    const auto replay = retained_feature::ReplayDetached(graph,
        retained_feature::RegistryProbe::registry(), retained_source::ProductionRegistry(), {source}, complete);
    assert(replay.replayed()); assert(replay.observations.size() == 2);
    std::vector<std::uint8_t> before, after;
    assert(composite_recipe::EncodeV3(graph, retained_feature::RegistryProbe::registry(),
        retained_source::ProductionRegistry(), before));
    retained_feature::ReplayBudget exhausted; exhausted.workRemaining = 2;
    const auto failed = retained_feature::ReplayDetached(graph,
        retained_feature::RegistryProbe::registry(), retained_source::ProductionRegistry(), {source}, exhausted);
    assert(!failed.replayed()); assert(failed.refusal == retained_feature::ReplayRefusal::BudgetExceeded);
    assert(composite_recipe::EncodeV3(graph, retained_feature::RegistryProbe::registry(),
        retained_source::ProductionRegistry(), after)); assert(before == after);
}
}

int main() {
    testOrdinaryPrepareCapturesUnitDigestInInstalledRevision();
    testFrozenLegacyCompositeV1V2AndD66Boundary();
    testV3UnknownKindCodecAndSourceRefuseBeforeAdoption();
    testV3CanonicalPayloadOrderIdentityAndBounds();
    testV3RegistryDoesNotInstallProductFeatures();
    testV3ReplaySuffixFailureRestoresWholeDocument();
    std::cout << "RetainedFeatureRegistryTests: PASS\n";
    return 0;
}
