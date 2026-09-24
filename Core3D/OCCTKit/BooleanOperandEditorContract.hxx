#pragma once
// A2 native-facing contract.  This is deliberately value-only: a plan never
// receives an OCAF label, TNaming ordinal, BRep, UUID authority, or a lease.
// The serial G0/A1 owner implements these declarations behind the existing
// bridge; this header fixes the A2 capture/prepare/apply/cancel vocabulary.
#include <cstdint>
#include <string>
#include <vector>
#include <variant>

namespace core3d::boolean_editor {

enum class ResultKind : std::uint8_t {
    MutationCommitted, Unchanged, ValidationComplete, ArtifactPrepared,
    Rejected, Cancelled, RecoveryRequired,
};

// Closed outcome; `kind` is the only outcome discriminator.  In particular
// there is no optional-success/optional-cancelled triple.  A Stop maps to
// Cancelled only after native cancellation confirms no mutation; an uncertain
// native return maps to RecoveryRequired and is never success.
struct Result final {
    ResultKind kind = ResultKind::Rejected;
    std::string reason;                 // fixed native vocabulary only
    std::uint32_t historyDelta = 0;     // non-zero only for MutationCommitted
    std::vector<std::string> identityRemap;
    static Result Rejected(std::string reason) { return {ResultKind::Rejected, std::move(reason)}; }
    static Result Cancelled() { return {ResultKind::Cancelled, "cancelled"}; }
    static Result RecoveryRequired() { return {ResultKind::RecoveryRequired, "unknown-native-outcome"}; }
};

enum class PartReferenceKind : std::uint8_t { EarlierPlanLabel, ContextAlias };
struct PartReference final {
    PartReferenceKind kind;
    std::string value; // validated label or host-issued alias, never a native ID
};

// `retainedInputPath` is a bounded semantic role/path from the advertised
// owner input table (for the first Boolean slice exactly `left` or `right`).
// It is deliberately not a topology ordinal or persisted node UUID.
struct InputReference final {
    PartReference owner;
    std::vector<std::string> retainedInputPath;
};

enum class ConsumptionState : std::uint8_t { LiveOutput, RetainedInput, Removed, Unavailable };
struct AdvertisedPart final {
    PartReference reference;
    ConsumptionState state;
    std::vector<std::string> allowedOperations;
    // A retained input remains addressable only through a registered
    // editBooleanInput route; it is not a selectable free result.
    std::vector<std::string> advertisedInputRoles;
};

enum class BooleanOperation : std::uint8_t { Union, Subtract, Intersect };
struct BooleanParts final {
    PartReference target; // earlier label/context alias that remains visible output
    PartReference tool;   // distinct earlier label/context alias, consumed on commit
    BooleanOperation operation;
};

enum class InputEditKind : std::uint8_t { ReplaceRecipeParameters, SetPlacement, SetMaterial };
enum class SourceRecipeFamily : std::uint8_t { Profile, Enclosure, RectangularLoft };
struct RecipeParameterEdit final { SourceRecipeFamily family; std::uint16_t codecVersion; std::vector<std::uint8_t> canonicalValue; };
struct InputPlacementEdit final { double translationMM[3]; double rotationXYZW[4]; };
struct InputMaterialEdit final { double baseColorSRGB[3]; double metallic; double roughness; };
struct EditBooleanInput final {
    InputReference input;
    InputEditKind kind;
    // Closed payload; the serial bridge rejects a kind/payload mismatch before Capture.
    std::variant<RecipeParameterEdit, InputPlacementEdit, InputMaterialEdit> value;
};

struct Capture final {
    Result result;
    std::vector<AdvertisedPart> parts;
    // Opaque native snapshot/fence remains private to the bridge.
};
struct Prepared final { Result result; };

// Required ordering, implemented by the G0/A1 serial owner:
// Capture checks both complete recipes, identities, units, placement, aliases,
// dependency/read sets and currentness before geometry. Prepare rechecks EVERY
// operand/dependent fence and builds a detached A1 candidate. Apply stages
// every graph/identity/material change in one transaction, seals only after
// proof, and reconciles exceptions. Any guard failure occurs before a command
// starts and returns Rejected with document/history unchanged. A partial or
// undecodable multi-step plan must never call Apply for any of its steps.
class NativeBooleanOperandEditor {
public:
    virtual ~NativeBooleanOperandEditor() = default;
    virtual Capture captureBooleanParts(const BooleanParts&) = 0;
    virtual Capture captureEditBooleanInput(const EditBooleanInput&) = 0;
    virtual Prepared prepareBooleanParts(const BooleanParts&, const Capture&) = 0;
    virtual Prepared prepareEditBooleanInput(const EditBooleanInput&, const Capture&) = 0;
    virtual Result apply(const Prepared&) = 0;
    virtual Result cancel(const Prepared&) = 0;
};

inline bool IsTerminalFailure(const Result& value) noexcept {
    return value.kind == ResultKind::Rejected || value.kind == ResultKind::Cancelled
        || value.kind == ResultKind::RecoveryRequired;
}
} // namespace core3d::boolean_editor
