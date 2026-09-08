#pragma once
// Isolated prototype; not registered in any production document format.
#include "../Scene/AuthoredTangentArchive.hpp"
#include <BinMDataStd_ByteArrayDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <BinObjMgt_RRelocationTable.hxx>
#include <BinObjMgt_SRelocationTable.hxx>
#include <Standard_Failure.hxx>
#include <Standard_GUID.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <memory>
#include <utility>

namespace core3d::persistence {

// Reserved in persistent-driver-design.md; not yet used by any production record.
inline const Standard_GUID& AuthoredFrameAttributeID() {
    static const Standard_GUID id("7AE0E058-6ABE-4561-AD01-8A1ECE250D36"); return id;
}
struct AuthoredFrameReadBudget {
    std::size_t bytes = 0;
    std::size_t limit = 64 * 1024 * 1024;
    bool rejected = false;
};

class BoundedAuthoredFrameDriver final : public BinMDataStd_ByteArrayDriver {
public:
    BoundedAuthoredFrameDriver(const Handle(Message_Messenger)& messenger,
                              std::shared_ptr<AuthoredFrameReadBudget> budget)
        : BinMDataStd_ByteArrayDriver(messenger), myBudget(std::move(budget)) {}

    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
        const Handle(TDF_Attribute)& target, BinObjMgt_RRelocationTable& relocation) const override {
        using namespace scene::authored;
        try {
            if (!myBudget || myBudget->rejected || relocation.GetHeaderData().IsNull()
                || relocation.GetHeaderData()->StorageVersion().IntegerValue() < TDocStd_FormatVersion_VERSION_10)
                return Reject();
            if (relocation.GetHeaderData()->StorageVersion().IntegerValue() > TDocStd_FormatVersion_CURRENT) return Reject();
            const Standard_Integer start = source.Position();
            const Standard_Integer length = source.Length();
            constexpr Standard_Integer recordHeader = 3 * sizeof(Standard_Integer);
            if (length < 0 || length > std::numeric_limits<Standard_Integer>::max() - recordHeader)
                return Reject();
            const Standard_Integer end = length + recordHeader;
            if (start < recordHeader || start > end || end - start < 8) return Reject();
            Standard_Integer first = -1, last = -1;
            if (!(source >> first >> last) || first != 0 || last < 127
                || last >= Standard_Integer(kMaximumArchiveBytes)) return Reject();
            const std::size_t count = std::size_t(last) + 1;
            if (source.Position() < start || source.Position() > end
                || count > std::size_t(end - source.Position())
                || myBudget->bytes > myBudget->limit || count > myBudget->limit - myBudget->bytes)
                return Reject();
            std::vector<std::uint8_t> bytes(count);
            if (!source.GetByteArray(bytes.data(), Standard_Integer(count))) return Reject();
            Standard_Byte delta = 1; Standard_GUID identifier;
            if (!(source >> delta >> identifier) || delta != 0 || identifier != AuthoredFrameAttributeID()
                || source.Position() != end || !ValidArchive(bytes)) return Reject();
            if (!source.SetPosition(start) || !BinMDataStd_ByteArrayDriver::Paste(source, target, relocation))
                return Reject();
            const auto attribute = Handle(TDataStd_ByteArray)::DownCast(target);
            if (attribute.IsNull() || attribute->ID() != AuthoredFrameAttributeID()
                || attribute->Lower() != 0 || attribute->Upper() != last || attribute->GetDelta()
                || source.Position() != end) return Reject();
            for (Standard_Integer i = 0; i <= last; ++i)
                if (attribute->Value(i) != bytes[std::size_t(i)]) return Reject();
            myBudget->bytes += count;
            return Standard_True;
        } catch (...) { return Reject(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable& relocation) const override {
        using namespace scene::authored;
        const auto attribute = Handle(TDataStd_ByteArray)::DownCast(source);
        if (attribute.IsNull() || attribute->ID() != AuthoredFrameAttributeID() || attribute->GetDelta()
            || attribute->Lower() != 0 || attribute->Upper() < 127
            || attribute->Upper() >= Standard_Integer(kMaximumArchiveBytes))
            Standard_Failure::Raise("Unsupported authored frame attribute.");
        std::vector<std::uint8_t> bytes(std::size_t(attribute->Upper()) + 1);
        for (std::size_t i = 0; i < bytes.size(); ++i) bytes[i] = attribute->Value(Standard_Integer(i));
        if (!ValidArchive(bytes)) Standard_Failure::Raise("Invalid authored frame archive.");
        // The enclosing writer still owns aggregate document admission.
        BinMDataStd_ByteArrayDriver::Paste(source, target, relocation);
    }

private:
    static bool ValidArchive(const std::vector<std::uint8_t>& bytes) {
        using namespace scene::authored;
        if (bytes.size() < kHeaderBytes + 48 + kDigestBytes || bytes.size() > kMaximumArchiveBytes) return false;
        GeometryIdentity identity; std::memcpy(identity.data(), bytes.data() + 16, identity.size());
        std::vector<scene::Float4> frames;
        return Decode(bytes.data(), bytes.size(), identity, frames) == ArchiveStatus::Valid;
    }
    Standard_Boolean Reject() const noexcept {
        if (myBudget) myBudget->rejected = true;
        return Standard_False;
    }
    std::shared_ptr<AuthoredFrameReadBudget> myBudget;
};
} // namespace core3d::persistence
