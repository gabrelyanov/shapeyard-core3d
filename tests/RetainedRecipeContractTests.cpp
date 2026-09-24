#include "CompositeRecipeCodec.hxx"
#include "RetainedRecipeAdmission.hxx"
#include "RetainedPartBoolean.hxx"
#include "RetainedRecipeProbe.hxx"
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Trsf.hxx>
#include <cassert>
#include <iostream>

using namespace core3d;

namespace {
retained_recipe::UUID id(std::uint8_t seed) { return retained_recipe::Probe::ID(seed); }

profile::Parameters profileRecipe(double metersPerUnit, bool shell) {
    profile::Parameters value; value.metersPerUnit = metersPerUnit;
    value.definition.plane = 0; value.definition.depth = 10;
    value.definition.points = {gp_Pnt2d(0,0),gp_Pnt2d(20,0),gp_Pnt2d(20,20),gp_Pnt2d(0,20)};
    if (shell) {
        profile::ShellStep step; step.thickness = 1; step.metersPerLocalUnit = metersPerUnit;
        step.openings = {5}; value.shells.push_back(step);
    }
    return value;
}

composite_recipe::SourceRecipe scalarRecipe(double metersPerUnit, bool shell) {
    const auto parameters = profileRecipe(metersPerUnit, shell); std::vector<double> values;
    assert(profile::Encode(parameters, values));
    composite_recipe::SourceRecipe recipe;
    recipe.kind = composite_recipe::RecipeKind::Profile;
    recipe.schema = std::uint32_t(profile::SchemaFor(parameters));
    assert(composite_recipe::EncodeScalarRecipe(recipe.kind, recipe.schema, values, recipe.bytes));
    assert(composite_recipe::ValidRecipe(recipe)); return recipe;
}

composite_recipe::SourceRecipe retainedRecipe(double metersPerUnit, bool program) {
    const auto parameters = profileRecipe(metersPerUnit, false); std::vector<double> values;
    assert(profile::Encode(parameters, values));
    retained_solid::Envelope legacy; legacy.document=id(1);legacy.entity=id(2);legacy.definition=id(3);
    legacy.sourceFeature=id(4);legacy.derivedFeature=id(5);legacy.sourceFamily=1;
    legacy.sourceSchema=std::uint32_t(profile::SchemaFor(parameters));legacy.metersPerUnit=metersPerUnit;
    legacy.axis=2;legacy.operandID=1;legacy.radius=2;legacy.sourceValues=values;
    std::vector<std::uint8_t> bytes;
    if (program) {
        retained_boolean::Program promoted; assert(retained_boolean::Promote(legacy,promoted));
        assert(retained_boolean::Encode(promoted,bytes));
    } else assert(retained_solid::Encode(legacy,bytes));
    retained_boolean::Recipe decoded;std::vector<std::uint8_t> exact;
    assert(retained_boolean::Decode(bytes,decoded));assert(retained_boolean::Encode(decoded,exact));assert(exact==bytes);
    composite_recipe::SourceRecipe result{composite_recipe::RecipeKind::RetainedBoolean,1,bytes};
    assert(composite_recipe::ValidRecipe(result));return result;
}

composite_recipe::Definition twoSourceDefinition(double metersPerUnit, bool program) {
    composite_recipe::Definition definition;definition.owner={id(1),id(10),id(11)};
    definition.issuance.nextLocalID=4;
    composite_recipe::SourceNode a;a.node=id(20);a.localID=1;
    a.original={id(1),id(10),id(11),id(12)};a.recipe=retainedRecipe(metersPerUnit,program);
    a.inputToCarrier=retained_recipe::Probe::Placement(metersPerUnit);a.shapeSlot=0;
    a.commitments=retained_recipe::Probe::Commit(40,a.recipe.bytes);
    composite_recipe::SourceNode b;b.node=id(21);b.localID=2;
    b.original={id(1),id(13),id(14),id(15)};b.recipe=scalarRecipe(metersPerUnit,true);
    b.inputToCarrier=retained_recipe::Probe::Placement(metersPerUnit);b.inputToCarrier.matrix[3]=25;
    b.shapeSlot=1;b.commitments=retained_recipe::Probe::Commit(50,b.recipe.bytes);
    composite_recipe::FeatureNode feature;feature.node=id(22);feature.feature=id(23);feature.localID=3;
    feature.kind=composite_recipe::PartBooleanFeatureKind;
    feature.codecVersion=composite_recipe::PartBooleanFeatureCodec;
    feature.inputs={a.node,b.node};feature.parameters={1,2,3,4};
    definition.nodes={composite_recipe::Node{a},composite_recipe::Node{b},composite_recipe::Node{feature}};
    definition.outputNode=feature.node;assert(composite_recipe::Valid(definition));return definition;
}

retained_recipe::RevisionFence fence(const composite_recipe::Definition& definition,double metersPerUnit) {
    retained_recipe::RevisionFence fence;fence.documentGeneration=7;fence.modelRevision=9;
    fence.effectiveMetersPerUnit=metersPerUnit;fence.ownerShape=retained_recipe::Probe::HashValue(70);
    fence.ownerRecipe=retained_recipe::Probe::HashValue(71);fence.ownerPlacement=retained_recipe::Probe::HashValue(72);
    fence.ownerMaterial=retained_recipe::Probe::HashValue(73);
    for(const auto& node:definition.nodes)if(const auto* source=std::get_if<composite_recipe::SourceNode>(&node.value))
        fence.dependencies.push_back({{definition.owner,source->node,source->original.sourceFeature},
            source->commitments.geometry,source->commitments.recipe,source->commitments.placement,
            source->commitments.material,source->commitments.groups});
    return fence;
}

retained_part_boolean::OperandReadSet a1CurrentReads() {
    retained_part_boolean::OperandReadSet result;
    const retained_recipe::OwnerKey owner{id(60),id(61),id(62)};
    auto read=[&](std::uint8_t seed, const retained_recipe::UUID& node,
                  const retained_recipe::UUID& sourceFeature){
        retained_recipe::DependencyRead value;
        value.locator={owner,node,sourceFeature};
        value.geometry=retained_recipe::Probe::HashValue(seed);
        value.recipe=retained_recipe::Probe::HashValue(seed+1);
        value.placement=retained_recipe::Probe::HashValue(seed+2);
        value.material=retained_recipe::Probe::HashValue(seed+3);
        value.groups=retained_recipe::Probe::HashValue(seed+4);
        return value;
    };
    result.leftSource=read(70,id(63),id(64));
    result.rightSource=read(80,id(65),id(66));
    return result;
}

TopoDS_Shape a1TranslatedBox(double x) {
    gp_Trsf move;move.SetTranslation(gp_Vec(x,0,0));
    return BRepBuilderAPI_Transform(
        BRepPrimAPI_MakeBox(10,10,10).Shape(),move).Shape();
}

void testTwoCompleteInputPayloadsRoundTripInMillimetresAndMetres() {
    for(double unit:{.001,1.0})for(bool program:{false,true}) {
        auto definition=twoSourceDefinition(unit,program);std::vector<std::uint8_t> encoded,again;
        composite_recipe::Definition decoded;assert(composite_recipe::Encode(definition,encoded));
        assert(composite_recipe::Decode(encoded,decoded));assert(composite_recipe::Encode(decoded,again));
        assert(encoded==again);assert(std::get<composite_recipe::SourceNode>(decoded.nodes[0].value).recipe.bytes
            ==std::get<composite_recipe::SourceNode>(definition.nodes[0].value).recipe.bytes);
        assert(std::get<composite_recipe::SourceNode>(decoded.nodes[1].value).recipe.schema==5);
    }
}

void testDistinctOwnerNodeFeatureAndConsumedInputIdentityNamespaces() {
    const auto definition=twoSourceDefinition(.001,true);
    const auto& a=std::get<composite_recipe::SourceNode>(definition.nodes[0].value);
    const auto& b=std::get<composite_recipe::SourceNode>(definition.nodes[1].value);
    const auto& feature=std::get<composite_recipe::FeatureNode>(definition.nodes[2].value);
    assert(definition.owner.entity==a.original.entity);assert(b.original.entity!=definition.owner.entity);
    assert(a.node!=b.node&&a.node!=feature.node&&b.node!=feature.node);
    assert(feature.feature!=feature.node&&a.original.sourceFeature!=b.original.sourceFeature);
}

void testMalformedDuplicateCycleForeignUnknownAndTransformInputsRefuse() {
    auto duplicate=twoSourceDefinition(.001,false);
    std::get<composite_recipe::SourceNode>(duplicate.nodes[1].value).node=
        std::get<composite_recipe::SourceNode>(duplicate.nodes[0].value).node;
    assert(!composite_recipe::Valid(duplicate));
    auto forward=twoSourceDefinition(.001,false);
    std::get<composite_recipe::FeatureNode>(forward.nodes[2].value).inputs[0]=id(99);
    assert(!composite_recipe::Valid(forward));
    auto foreign=twoSourceDefinition(.001,false);
    std::get<composite_recipe::SourceNode>(foreign.nodes[1].value).original.document=id(88);
    assert(!composite_recipe::Valid(foreign));
    auto unknown=twoSourceDefinition(.001,false);
    std::get<composite_recipe::FeatureNode>(unknown.nodes[2].value).kind=999;
    assert(!composite_recipe::Valid(unknown));
    auto scaled=twoSourceDefinition(.001,false);
    std::get<composite_recipe::SourceNode>(scaled.nodes[1].value).inputToCarrier.matrix[0]=2;
    assert(!composite_recipe::Valid(scaled));
    auto reflected=twoSourceDefinition(.001,false);
    std::get<composite_recipe::SourceNode>(reflected.nodes[1].value).inputToCarrier.matrix[0]=-1;
    assert(!composite_recipe::Valid(reflected));
}

void testDigestAndCanonicalLengthCorruptionRefuseWithoutPartialOutput() {
    auto definition=twoSourceDefinition(.001,true);std::vector<std::uint8_t> bytes;
    assert(composite_recipe::Encode(definition,bytes));
    bytes[bytes.size()/2]^=1;composite_recipe::Definition output;assert(!composite_recipe::Decode(bytes,output));
    assert(output.nodes.empty());
    assert(composite_recipe::Encode(definition,bytes));bytes.pop_back();assert(!composite_recipe::Decode(bytes,output));
}

void testP1SnapshotRetainsBothReadSetsButNeverAdmitsBooleanGeometry() {
    const auto definition=twoSourceDefinition(1.0,true);const auto captured=fence(definition,1.0);
    const auto snapshot=retained_recipe::Snapshot(definition,captured);
    assert(snapshot.status==retained_recipe::OwnerStatus::UnsupportedVersion);
    assert(snapshot.sources.size()==2);
    const auto decision=retained_recipe::P1NoFeatureAdmission(snapshot);
    assert(!decision.admitted());assert(decision.completeReadSet.size()==2);
}

void testLegacySchemaFiveSnapshotPreservesEveryShellAndParameter() {
    const auto definition=twoSourceDefinition(.001,false);
    const auto& source=std::get<composite_recipe::SourceNode>(definition.nodes[1].value);
    const auto captured=fence(definition,.001);
    const auto snapshot=retained_recipe::LegacySnapshot(definition.owner,source.node,source.original,
        source.recipe,source.inputToCarrier,source.commitments,captured,true);
    assert(snapshot.status==retained_recipe::OwnerStatus::CurrentEditable);
    std::vector<double> values;assert(composite_recipe::DecodeScalarRecipe(snapshot.sources[0].recipe,values));
    profile::Parameters decoded;assert(profile::Decode(values,decoded));assert(decoded.shells.size()==1);
    assert(decoded.shells[0].openings==std::vector<int>{5});
}

void testA1ExactlyTwoAnalyticSourceRecipesAreTheOnlyFirstSliceInput() {
    retained_recipe::OwnerSnapshot snapshot;snapshot.sources.resize(2);
    snapshot.sources[0].recipe.kind=composite_recipe::RecipeKind::Profile;
    snapshot.sources[1].recipe.kind=composite_recipe::RecipeKind::Enclosure;
    assert(retained_part_boolean::HasExactlyTwoAnalyticSourceRecipes(snapshot));
    snapshot.sources[1].recipe.kind=composite_recipe::RecipeKind::RetainedBoolean;
    assert(!retained_part_boolean::HasExactlyTwoAnalyticSourceRecipes(snapshot));
    snapshot.sources.resize(1);
    assert(!retained_part_boolean::HasExactlyTwoAnalyticSourceRecipes(snapshot));
}

void testA1TwoAnalyticPartsProduceOneConnectedSolidForEachOperation() {
    const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
    const TopoDS_Shape rightSource=a1TranslatedBox(5);
    const auto reads=a1CurrentReads();
    for(const auto operation:{retained_part_boolean::Operation::Union,
            retained_part_boolean::Operation::Subtract,
            retained_part_boolean::Operation::Intersect}) {
        const auto candidate=retained_part_boolean::BuildDetachedCandidate(
            operation,leftSource,rightSource,1e-7,reads,reads);
        assert(candidate.admitted());
        assert(retained_part_boolean::SolidCount(candidate.solid)==1);
    }
}

void testA1EmptyResultRefusesWithNullCandidate() {
    const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
    const auto reads=a1CurrentReads();
    const auto candidate=retained_part_boolean::BuildDetachedCandidate(
        retained_part_boolean::Operation::Intersect,leftSource,a1TranslatedBox(20),
        1e-7,reads,reads);
    assert(candidate.refusal==retained_part_boolean::Refusal::EmptyResult);
    assert(candidate.solid.IsNull());
}

void testA1TangentResultRefusesWithNullCandidate() {
    const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
    const auto reads=a1CurrentReads();
    const auto candidate=retained_part_boolean::BuildDetachedCandidate(
        retained_part_boolean::Operation::Union,leftSource,a1TranslatedBox(10),
        1e-7,reads,reads);
    assert(candidate.refusal==retained_part_boolean::Refusal::TangentOnly);
    assert(candidate.solid.IsNull());
}

void testA1DisconnectedResultRefusesWithNullCandidate() {
    const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
    const auto reads=a1CurrentReads();
    const auto candidate=retained_part_boolean::BuildDetachedCandidate(
        retained_part_boolean::Operation::Union,leftSource,a1TranslatedBox(20),
        1e-7,reads,reads);
    assert(candidate.refusal==retained_part_boolean::Refusal::Disconnected);
    assert(candidate.solid.IsNull());
}

void testA1StaleRightSourceRefusesWithNullCandidate() {
    const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
    const TopoDS_Shape rightSource=a1TranslatedBox(5);
    const auto capturedReads=a1CurrentReads();
    auto currentReads=capturedReads;
    currentReads.rightSource.geometry[0]^=0xff;
    const auto candidate=retained_part_boolean::BuildDetachedCandidate(
        retained_part_boolean::Operation::Union,leftSource,rightSource,
        1e-7,capturedReads,currentReads);
    assert(candidate.refusal==retained_part_boolean::Refusal::StaleRightSource);
    assert(candidate.solid.IsNull());
}
}

int main() {
    testTwoCompleteInputPayloadsRoundTripInMillimetresAndMetres();
    testDistinctOwnerNodeFeatureAndConsumedInputIdentityNamespaces();
    testMalformedDuplicateCycleForeignUnknownAndTransformInputsRefuse();
    testDigestAndCanonicalLengthCorruptionRefuseWithoutPartialOutput();
    testP1SnapshotRetainsBothReadSetsButNeverAdmitsBooleanGeometry();
    testLegacySchemaFiveSnapshotPreservesEveryShellAndParameter();
    testA1ExactlyTwoAnalyticSourceRecipesAreTheOnlyFirstSliceInput();
    testA1TwoAnalyticPartsProduceOneConnectedSolidForEachOperation();
    testA1EmptyResultRefusesWithNullCandidate();
    testA1TangentResultRefusesWithNullCandidate();
    testA1DisconnectedResultRefusesWithNullCandidate();
    testA1StaleRightSourceRefusesWithNullCandidate();
    std::cout<<"RetainedRecipeContractTests: PASS\n";
}
