#pragma once
#include <Standard_GUID.hxx>

namespace core3d::retained_edge_treatment {
inline const Standard_GUID& AttributeID() noexcept {
    static const Standard_GUID id("D7C65BD1-B6A0-4F7A-9E76-E6E9B7D308B1");
    return id;
}
} // namespace core3d::retained_edge_treatment
