#import "Core3DPatternAIReadback.h"

#if DEBUG
#import "GLViewController+Trick.h"
#import "Core3DViewer.h"
#import "../OCCTKit/GLViewController.h"

#include "../OCCTKit/PatternAllLabelAuthority.hxx"
#include <TCollection_AsciiString.hxx>
#include <TDF_Tool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <cmath>
#include <limits>

namespace {
namespace po = core3d::pattern_owner;
namespace pp = core3d::pattern;
namespace pr = core3d::pattern_recipe_clone;

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data() length:value.size()
                                  encoding:NSUTF8StringEncoding];
}
NSString *UUIDText(const pp::UUID& value) {
    try { return Text(core3d::retained_solid::UUIDText(value)); }
    catch (...) { return nil; }
}
NSString *Decimal(std::uint64_t value) {
    return [NSString stringWithFormat:@"%llu", value];
}
NSString *Size(std::size_t value) {
    return Decimal(static_cast<std::uint64_t>(value));
}
NSString *Base64(const void *bytes, std::size_t size) {
    if (size == 0) return @"";
    if (!bytes || size > NSUIntegerMax) return nil;
    return [[NSData dataWithBytes:bytes length:static_cast<NSUInteger>(size)]
        base64EncodedStringWithOptions:0];
}
template <class T> NSString *Base64(const std::vector<T>& value) {
    if (value.size() > std::numeric_limits<std::size_t>::max() / sizeof(T)) return nil;
    return Base64(value.data(), value.size() * sizeof(T));
}
NSString *Base64(const std::string& value) {
    return Base64(value.data(), value.size());
}
NSString *Entry(const TDF_Label& label) {
    if (label.IsNull()) return nil;
    TCollection_AsciiString value; TDF_Tool::Entry(label, value);
    const char *raw = value.ToCString();
    return raw && raw[0] ? [NSString stringWithUTF8String:raw] : nil;
}
NSString *ExtendedText(const TCollection_ExtendedString& value) {
    if (value.Length() < 0
        || static_cast<unsigned long long>(value.Length()) > NSUIntegerMax) return nil;
    if (value.Length() == 0) return @"";
    return [[NSString alloc]
        initWithCharacters:reinterpret_cast<const unichar *>(value.ToExtString())
                    length:static_cast<NSUInteger>(value.Length())];
}
NSArray<NSNumber *> *Numbers(const double *values, std::size_t count) {
    if (!values && count) return nil;
    NSMutableArray *result = [NSMutableArray arrayWithCapacity:count];
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(values[index])) return nil;
        [result addObject:@(values[index])];
    }
    return result;
}
NSArray<NSNumber *> *Numbers(const std::vector<double>& values) {
    return Numbers(values.data(), values.size());
}
NSString *Kind(pp::Kind value) {
    switch (value) {
        case pp::Kind::Linear: return @"linear";
        case pp::Kind::Radial: return @"radial";
        case pp::Kind::Grid: return @"grid";
    }
    return nil;
}
NSString *State(pp::MemberState value) {
    switch (value) {
        case pp::MemberState::Active: return @"active";
        case pp::MemberState::Suppressed: return @"suppressed";
        case pp::MemberState::Removed: return @"removed";
    }
    return nil;
}
NSString *Family(pr::Family value) {
    switch (value) {
        case pr::Family::None: return @"none";
        case pr::Family::Sweep: return @"sweep";
        case pr::Family::Loft: return @"loft";
        case pr::Family::AnalyticBoolean: return @"analyticBoolean";
    }
    return nil;
}
NSString *Representation(OcctGeometryRepresentation value) {
    switch (value) {
        case OcctGeometryRepresentation::LegacyUnknown: return @"legacyUnknown";
        case OcctGeometryRepresentation::BRep: return @"brep";
        case OcctGeometryRepresentation::TriangleMesh: return @"triangleMesh";
        case OcctGeometryRepresentation::Invalid: return nil;
    }
    return nil;
}

NSArray<NSNumber *> *Frame(
    const std::optional<core3d::profile::ConstructionFrame>& value) {
    return value ? Numbers(value->values.data(), value->values.size()) : @[];
}

NSDictionary *Cost(const po::AllLabelMeasuredCost& value) {
    return @{ @"recordBytes": Size(value.recordBytes),
              @"recipeBytes": Size(value.recipeBytes),
              @"shapeBytes": Size(value.shapeBytes),
              @"topologyNodes": Decimal(static_cast<std::uint64_t>(value.topologyNodes)),
              @"retainedMemoryBytes": Size(value.retainedMemoryBytes) };
}

NSDictionary *Recipe(const pr::Source& value) {
    NSString *family = Family(value.family);
    NSString *feature = Text(po::RecipeFeatureIdentifier(value));
    if (!family || !feature || (value.carrierLengthUnitPresent
        && (!std::isfinite(value.carrierMetersPerUnit)
            || value.carrierMetersPerUnit <= 0))) return nil;
    NSArray *persisted = @[]; NSArray *frame = @[]; NSString *payload = @"";
    if (value.family == pr::Family::Sweep) {
        persisted = Numbers(value.sweep.values); payload = Base64(value.sweep.values);
        frame = Frame(value.sweep.definition.constructionFrame);
    } else if (value.family == pr::Family::Loft) {
        persisted = Numbers(value.loft.values); payload = Base64(value.loft.values);
        frame = Frame(value.loft.definition.constructionFrame);
    } else if (value.family == pr::Family::AnalyticBoolean) {
        if (!value.analyticBoolean.value) return nil;
        payload = Base64(value.analyticBoolean.value->bytes);
    }
    if (!persisted || !frame || !payload) return nil;
    return @{ @"family": family,
              @"carrierLengthUnitPresent": @(value.carrierLengthUnitPresent),
              @"carrierMetersPerUnit": @(value.carrierMetersPerUnit),
              @"persistedValues": persisted, @"constructionFrame": frame,
              @"payloadBytesBase64": payload,
              @"nativeFeatureIdentifier": feature };
}

NSDictionary *Receipt(const OcctExactLabelReceipt& value) {
    const auto& visibility = value.visibility;
    const auto& named = visibility.object;
    const auto& object = named.object;
    NSString *document = Text(value.documentIdentifier);
    NSString *label = Entry(object.label);
    NSString *entity = Text(object.entityIdentifier);
    NSString *definition = Text(object.definitionIdentifier);
    NSString *name = ExtendedText(named.name);
    NSString *stored = Representation(object.storedRepresentation);
    NSString *resolved = Representation(object.resolvedRepresentation);
    if (!document || !label || !entity || !definition || !name || !stored
        || !resolved || object.shape.IsNull()) return nil;
    NSMutableArray *present = [NSMutableArray arrayWithCapacity:object.present.size()];
    for (Standard_Boolean item : object.present) [present addObject:@(bool(item))];
    NSArray *scalars = Numbers(object.scalars.data(), object.scalars.size());
    NSString *scalarBytes = Base64(object.scalars.data(), sizeof(object.scalars));
    NSMutableArray *transform = [NSMutableArray arrayWithCapacity:12];
    for (int row = 1; row <= 3; ++row) for (int column = 1; column <= 4; ++column) {
        const double item = object.transform.Value(row, column);
        if (!std::isfinite(item)) return nil;
        [transform addObject:@(item)];
    }
    if (visibility.layers.size() != visibility.layerInvisibleAttributePresent.size())
        return nil;
    NSMutableArray *layers = [NSMutableArray arrayWithCapacity:visibility.layers.size()];
    for (std::size_t index = 0; index < visibility.layers.size(); ++index) {
        NSString *entry = Entry(visibility.layers[index]);
        if (!entry) return nil;
        [layers addObject:@{ @"labelEntry": entry,
            @"invisibleAttributePresent":
                @(bool(visibility.layerInvisibleAttributePresent[index])) }];
    }
    const auto& appearance = value.appearance;
    NSString *material = appearance.materialLabel.IsNull()
        ? @"" : Entry(appearance.materialLabel);
    NSArray *visual = Numbers(appearance.visualValues);
    if (!scalars || !scalarBytes || !material || !visual) return nil;
    return @{ @"documentIdentifier": document, @"labelEntry": label,
        @"entityIdentifier": entity, @"definitionIdentifier": definition,
        @"namePresent": @(bool(named.namePresent)), @"name": name,
        @"transformScalarPresent": present, @"transformScalars": scalars,
        @"transformScalarsBase64": scalarBytes, @"effectiveTransform": transform,
        @"shapeOrientation": @(static_cast<int>(object.shape.Orientation())),
        @"storedRepresentation": stored, @"resolvedRepresentation": resolved,
        @"ownInvisibleAttributePresent": @(bool(visibility.invisibleAttributePresent)),
        @"layerLinkPresent": @(bool(visibility.layerLinkPresent)), @"layers": layers,
        @"materialLabelPresent": @(!appearance.materialLabel.IsNull()),
        @"materialLabelEntry": material,
        @"legacyPresent": @[@(appearance.legacyPresent[0]),
                              @(appearance.legacyPresent[1])],
        @"legacyValues": @[@(appearance.legacyValues[0]),
                             @(appearance.legacyValues[1])],
        @"localPBR": @(appearance.localPBR), @"visualValues": visual };
}

NSDictionary *MemberRecord(const pp::Member& value) {
    NSString *identity = UUIDText(value.identity), *state = State(value.state);
    if (!identity || !state) return nil;
    return @{ @"row": @(value.coordinate.row), @"column": @(value.coordinate.column),
              @"identity": identity, @"localID": Decimal(value.localID),
              @"state": state };
}

NSDictionary *Definition(const pp::Definition& value) {
    NSString *kind = Kind(value.kind);
    NSArray *frame = Numbers(value.sourceFrame.data(), value.sourceFrame.size());
    NSArray *pivot = Numbers(value.radialPivotLocal.data(), value.radialPivotLocal.size());
    if (!kind || !frame || !pivot || !std::isfinite(value.rowSpacing)
        || !std::isfinite(value.columnSpacing) || !std::isfinite(value.sweepRadians))
        return nil;
    NSString *ownerDocument = UUIDText(value.owner.document);
    NSString *ownerEntity = UUIDText(value.owner.entity);
    NSString *ownerDefinition = UUIDText(value.owner.definition);
    NSString *feature = UUIDText(value.feature);
    NSString *sourceDocument = UUIDText(value.source.document);
    NSString *sourceEntity = UUIDText(value.source.entity);
    NSString *sourceDefinition = UUIDText(value.source.definition);
    NSString *sourceFeature = UUIDText(value.source.sourceFeature);
    if (!ownerDocument || !ownerEntity || !ownerDefinition || !feature
        || !sourceDocument || !sourceEntity || !sourceDefinition || !sourceFeature)
        return nil;
    NSMutableArray *members = [NSMutableArray arrayWithCapacity:value.members.size()];
    for (const auto& item : value.members) {
        NSDictionary *dump = MemberRecord(item); if (!dump) return nil;
        [members addObject:dump];
    }
    NSMutableArray *removals = [NSMutableArray arrayWithCapacity:value.removals.size()];
    for (const auto& item : value.removals) {
        NSDictionary *dump = MemberRecord(item); if (!dump) return nil;
        [removals addObject:dump];
    }
    NSMutableArray *retired =
        [NSMutableArray arrayWithCapacity:value.issuance.retiredLocalIDs.size()];
    for (std::uint64_t item : value.issuance.retiredLocalIDs)
        [retired addObject:Decimal(item)];
    NSString *frameBytes = Base64(value.sourceFrame.data(), sizeof(value.sourceFrame));
    if (!frameBytes) return nil;
    return @{ @"ownerDocumentIdentifier": ownerDocument,
        @"ownerEntityIdentifier": ownerEntity,
        @"ownerDefinitionIdentifier": ownerDefinition, @"featureIdentifier": feature,
        @"sourceDocumentIdentifier": sourceDocument,
        @"sourceEntityIdentifier": sourceEntity,
        @"sourceDefinitionIdentifier": sourceDefinition,
        @"sourceFeatureIdentifier": sourceFeature, @"sourceFrame": frame,
        @"sourceFrameBytesBase64": frameBytes, @"kind": kind,
        @"rowAxis": @(static_cast<unsigned>(value.rowAxis)),
        @"columnAxis": @(static_cast<unsigned>(value.columnAxis)),
        @"rowCount": @(value.rowCount), @"columnCount": @(value.columnCount),
        @"rowSpacing": @(value.rowSpacing), @"columnSpacing": @(value.columnSpacing),
        @"radialPivotLocal": pivot, @"sweepRadians": @(value.sweepRadians),
        @"members": members, @"nextLocalID": Decimal(value.issuance.nextLocalID),
        @"retiredLocalIDs": retired, @"removals": removals };
}

NSDictionary *Label(const OcctExactLabelReceipt& receipt, const pr::Source& recipe,
                    const std::string& shape, const po::AllLabelMeasuredCost& cost,
                    NSString *feature, std::uint32_t row, std::uint32_t column,
                    std::uint64_t localID, bool suppressed, bool source) {
    NSDictionary *r = Receipt(receipt), *p = Recipe(recipe), *c = Cost(cost);
    NSString *bytes = Base64(shape);
    if (!r || !p || !c || !bytes || !feature) return nil;
    return @{ @"source": @(source), @"row": @(row), @"column": @(column),
        @"localID": Decimal(localID), @"suppressed": @(suppressed),
        @"featureIdentifier": feature, @"receipt": r, @"recipe": p,
        @"orientedShapeBytesBase64": bytes,
        @"orientedShapeByteCount": Size(shape.size()), @"measured": c };
}

NSDictionary *Source(const po::AllLabelSnapshot& snapshot, NSString *feature) {
    NSDictionary *receipt = Receipt(snapshot.source);
    NSDictionary *recipe = Recipe(snapshot.sourceRecipe);
    NSString *shape = Base64(snapshot.sourceShapeBytes);
    if (!receipt || !recipe || !shape || !feature) return nil;
    return @{ @"featureIdentifier": feature, @"receipt": receipt, @"recipe": recipe,
        @"orientedShapeBytesBase64": shape,
        @"orientedShapeByteCount": Size(snapshot.sourceShapeBytes.size()) };
}

NSDictionary *Envelope(const po::AllLabelSnapshot& snapshot, double unit) {
    if (!std::isfinite(unit) || unit <= 0
        || snapshot.family != po::AllLabelFamily::PatternD2
        || snapshot.recordLabel.IsNull() || snapshot.canonicalRecordBytes.empty()
        || snapshot.members.empty()) return nil;
    pp::Definition decoded;
    if (!pp::Decode(snapshot.canonicalRecordBytes, decoded)) return nil;
    NSDictionary *definition = Definition(decoded), *cost = Cost(snapshot.measured);
    NSString *document = Text(snapshot.documentIdentifier);
    NSString *recordLabel = Entry(snapshot.recordLabel);
    NSString *recordBytes = Base64(snapshot.canonicalRecordBytes);
    NSString *snapshotFeature = Text(snapshot.featureIdentifier);
    NSString *decodedFeature = UUIDText(decoded.feature);
    NSString *sourceFeature = Text(po::RecipeFeatureIdentifier(snapshot.sourceRecipe));
    if (!definition || !cost || !document || !recordLabel || !recordBytes
        || !snapshotFeature || !decodedFeature || !sourceFeature
        || ![snapshotFeature isEqualToString:decodedFeature]) return nil;
    NSDictionary *source = Source(snapshot, sourceFeature);
    if (!source) return nil;
    NSMutableArray *members = [NSMutableArray arrayWithCapacity:snapshot.members.size()];
    for (const auto& member : snapshot.members) {
        const auto *key = std::get_if<po::D2Coordinate>(&member.key);
        NSString *feature = Text(member.featureIdentifier);
        if (!key || !feature) return nil;
        NSDictionary *dump = Label(member.receipt, member.recipe, member.shapeBytes,
            member.measured, feature, key->row, key->column, member.localIdentifier,
            member.suppressed, key->row == 0 && key->column == 0);
        if (!dump) return nil;
        [members addObject:dump];
    }
    return @{ @"schema": @"shapeyard.d2-pattern-ai-readback.v1",
        @"document": @{ @"identifier": document, @"recordLabelEntry": recordLabel,
            @"documentTime": @(snapshot.documentTime), @"metersPerUnit": @(unit),
            @"canonicalRecordBytesBase64": recordBytes,
            @"canonicalRecordByteCount": Size(snapshot.canonicalRecordBytes.size()) },
        @"definition": definition, @"source": source,
        @"members": members, @"measured": cost };
}
} // namespace

@implementation Core3DViewController (PatternAIReadback)
- (NSDictionary<NSString *, id> *)debugPatternAIReadback:
    (NSString *)entityIdentifier {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || !GLController || !GLController.viewer) return nil;
    @autoreleasepool {
        try {
            const char *raw = entityIdentifier.UTF8String;
            if (!raw) return nil;
            const Handle(OcctDocument) nativeOwner = GLController.viewer->getDocument();
            const Handle(TDocStd_Document) document = nativeOwner.IsNull()
                ? Handle(TDocStd_Document)() : nativeOwner->Document();
            Standard_Real unit = 0;
            if (document.IsNull() || document->HasOpenCommand()
                || !XCAFDoc_DocumentTool::GetLengthUnit(document, unit)
                || !std::isfinite(double(unit)) || unit <= 0) return nil;
            std::shared_ptr<const po::AllLabelSnapshot> snapshot;
            if (!nativeOwner->CapturePatternAllLabelSnapshot(std::string(raw), snapshot)
                || !snapshot) return nil;
            NSDictionary *result = Envelope(*snapshot, double(unit));
            if (!result) return nil;
            std::shared_ptr<const po::AllLabelSnapshot> recaptured;
            if (!nativeOwner->ReadPatternAllLabelSnapshot(*snapshot, recaptured)
                || !recaptured) return nil;
            return result;
        } catch (...) { return nil; }
    }
}
@end
#endif
