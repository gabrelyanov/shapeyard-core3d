#pragma once
#include <Standard_GUID.hxx>

namespace core3d::persistence {
// Permanent native geometry attribute ID; independent of material assignment.
inline const Standard_GUID& AuthoredFrameAttributeID() {
    static const Standard_GUID id("7AE0E058-6ABE-4561-AD01-8A1ECE250D36"); return id;
}
} // namespace core3d::persistence
