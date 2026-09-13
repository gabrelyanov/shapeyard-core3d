#pragma once
#include "ReceiptDirectFrame.hxx"
#include <BinMDF_ADriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <BinObjMgt_RRelocationTable.hxx>
#include <TDF_Attribute.hxx>
#include <sstream>
#include <string>

namespace core3d::persistence::receipt_framing {
// Isolated adapter, not a ReadSubTree override. Caller has already matched the
// exact registered V3 driver/target, refused aliases, checked document format12+
// and derived enclosingRemaining from validated current file/label boundaries.
inline bool PasteReceiptFrame(std::istream& original,
                              const ValidatedHeader& header, bool inverse,
                              std::uint64_t enclosingRemaining, LoadBudget& budget,
                              const Handle(BinMDF_ADriver)& driver,
                              const Handle(TDF_Attribute)& target,
                              BinObjMgt_RRelocationTable& relocation) {
    static_assert(sizeof(Standard_Integer) == 4, "OCCT record header changed");
    if (driver.IsNull() || target.IsNull()) return budget.refuse();
    return ReadDirectFrame(original, header, inverse, enclosingRemaining, budget,
        [&](const ValidatedHeader& captured, std::istream& bounded, std::uint64_t) {
            // Exactly12 cached bytes and a required zero DataLength. The original
            // stream header is never re-read by OCCT or sought backwards here.
            const auto& bytes = captured.bytes();
            const std::string encoded(reinterpret_cast<const char*>(bytes.data()), bytes.size());
            std::istringstream input(encoded, std::ios::in | std::ios::binary);
            BinObjMgt_Persistent persistent;
            persistent.Read(input);
            if (!input || persistent.IsError() || persistent.Length() != 0 ||
                !persistent.IsDirect() || persistent.TypeId() != captured.typeID() || persistent.Id() != captured.objectID())
                return false;
            persistent.SetIStream(bounded);
            return bool(driver->Paste(persistent, target, relocation));
        });
}
} // namespace core3d::persistence::receipt_framing
