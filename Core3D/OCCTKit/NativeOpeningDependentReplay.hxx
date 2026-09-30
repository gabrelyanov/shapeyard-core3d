#pragma once

// Shared fail-closed boundary for topology-changing edits whose source or host
// is retained by D2, D3, or D4.  Discovery is owned by OcctDocument so callers
// cannot omit a record.  Family-specific collaborators supply detached work,
// but cannot add, remove, or reorder the discovered closure.

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include "RetainedBooleanProgram.hxx"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class OcctDocument;
namespace core3d::native_opening { class CommandLease; class Context; }

namespace core3d::dependent_replay {

using UUID = std::array<std::uint8_t, 16>;

enum class Family : std::uint8_t {
    PatternD2 = 2,
    PathArrayD3 = 3,
    FeaturePatternD4 = 4
};

enum class Mutation : std::uint8_t { Replace = 1, Remove = 2 };

enum class Refusal : std::uint8_t {
    None = 0,
    ClosedDocument,
    OpenCommand,
    StaleTarget,
    CorruptTable,
    MissingRecipe,
    UnsupportedDescendant,
    Cycle,
    RecordBudget,
    DocumentBudget,
    MemoryBudget,
    TopologyBudget,
    StaleClosure,
    ForeignLease,
    SourceStageFailed,
    DependentStageFailed,
    ReadbackFailed
};

struct Limits final {
    std::size_t records = 128;
    std::size_t documentBytes = 8 * 1024 * 1024;
    std::size_t memoryBytes = 256 * 1024 * 1024;
    std::size_t topologyNodes = 2'000'000;
};

//! Exact retained-record edge in the transitive closure. canonicalRecordBytes
//! are the actual SYPT/SYPA/SYFP bytes read from OCAF, never a reconstructed
//! descriptor. memberIdentities preserve the retained member/child identities.
struct Dependency final {
    Family family = Family::PatternD2;
    UUID feature{};
    UUID inputEntity{};
    UUID resultEntity{};
    TDF_Label recordLabel;
    std::vector<std::uint8_t> canonicalRecordBytes;
    std::vector<UUID> memberIdentities;

    bool exactlyEquals(const Dependency& other) const noexcept {
        return family == other.family && feature == other.feature
            && inputEntity == other.inputEntity && resultEntity == other.resultEntity
            && !recordLabel.IsNull() && recordLabel.IsEqual(other.recordLabel)
            && canonicalRecordBytes == other.canonicalRecordBytes
            && memberIdentities == other.memberIdentities;
    }
};

//! Family-specific detached work. prepare() is called before any command.
//! stage() may only use the supplied caller lease; it must neither commit nor
//! begin nested work. read() proves the staged/committed identity and recipe.
class PreparedReplay {
public:
    virtual ~PreparedReplay() = default;
    virtual Family family() const noexcept = 0;
    virtual UUID feature() const noexcept = 0;
    virtual UUID resultEntity() const noexcept = 0;
    virtual std::size_t documentBytes() const noexcept = 0;
    virtual std::size_t memoryBytes() const noexcept = 0;
    virtual std::size_t topologyNodes() const noexcept = 0;
    virtual bool openingCurrent(OcctDocument&) const noexcept = 0;
    virtual bool stage(OcctDocument&, native_opening::CommandLease&) const noexcept = 0;
    virtual bool read(OcctDocument&) const noexcept = 0;
};

class Preparer {
public:
    virtual ~Preparer() = default;
    virtual Refusal prepare(OcctDocument&, const Dependency&, Mutation,
        std::shared_ptr<const PreparedReplay>&) noexcept = 0;
};

//! Prospective source/host state produced by the accepted source editor. It is
//! detached and immutable; family collaborators still capture all old OCAF
//! authority themselves before preparing a replay.
struct Candidate final {
    std::string targetEntityIdentifier;
    TopoDS_Shape resultShape;
    TopoDS_Shape retainedBase;
    retained_boolean::Recipe retainedRecipe;
    std::vector<std::uint8_t> retainedRecipeBytes;
    bool hasRetainedRecipe = false;
};

Refusal PreparePatternD2Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, std::shared_ptr<const PreparedReplay>&) noexcept;
Refusal PreparePathArrayD3Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, const std::shared_ptr<native_opening::Context>&,
    std::shared_ptr<const PreparedReplay>&) noexcept;
Refusal PrepareFeaturePatternD4Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, const std::shared_ptr<native_opening::Context>&,
    std::shared_ptr<const PreparedReplay>&) noexcept;

class ProductionPreparer final : public Preparer {
public:
    ProductionPreparer(Candidate candidate,
        std::shared_ptr<native_opening::Context> context) noexcept
        : candidate_(std::move(candidate)), context_(std::move(context)) {}
    Refusal prepare(OcctDocument& owner, const Dependency& dependency,
        Mutation mutation,
        std::shared_ptr<const PreparedReplay>& output) noexcept override {
        output.reset();
        if (!context_ || candidate_.targetEntityIdentifier.empty()
            || candidate_.resultShape.IsNull()) return Refusal::MissingRecipe;
        switch (dependency.family) {
            case Family::PatternD2:
                return PreparePatternD2Replay(owner, dependency, mutation,
                                              candidate_, output);
            case Family::PathArrayD3:
                return PreparePathArrayD3Replay(owner, dependency, mutation,
                    candidate_, context_, output);
            case Family::FeaturePatternD4:
                return PrepareFeaturePatternD4Replay(owner, dependency, mutation,
                    candidate_, context_, output);
        }
        return Refusal::UnsupportedDescendant;
    }
private:
    Candidate candidate_;
    std::shared_ptr<native_opening::Context> context_;
};

//! The source/host mutation is injected so OcctDocument can keep its narrow
//! authorization active for exactly source stage + complete dependent stage.
class SourceMutation {
public:
    virtual ~SourceMutation() = default;
    virtual bool stage(OcctDocument&, native_opening::CommandLease&) noexcept = 0;
};

class Plan final {
public:
    Plan() = delete;
    Plan(const Plan&) = delete;
    Plan& operator=(const Plan&) = delete;

    const std::vector<Dependency>& dependencies() const noexcept {
        return dependencies_;
    }
    Mutation mutation() const noexcept { return mutation_; }
    const std::string& targetEntityIdentifier() const noexcept {
        return targetEntityIdentifier_;
    }

private:
    friend class ::OcctDocument;
    Plan(Mutation mutation, std::string entity, std::string definition)
        : mutation_(mutation), targetEntityIdentifier_(std::move(entity)),
          targetDefinitionIdentifier_(std::move(definition)) {}

    Mutation mutation_ = Mutation::Replace;
    std::string documentIdentifier_;
    std::string targetEntityIdentifier_;
    std::string targetDefinitionIdentifier_;
    const void* documentData_ = nullptr;
    Standard_Integer documentTime_ = 0;
    Limits limits_;
    std::vector<Dependency> dependencies_;
    std::vector<std::shared_ptr<const PreparedReplay>> prepared_;
    std::vector<std::string> authorizedEntityIdentifiers_;
};

} // namespace core3d::dependent_replay
