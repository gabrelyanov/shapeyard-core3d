// Registration is intentionally left to the serial Core3D integration owner.
// Exact tests are named in the F2 source-bound design draft.
#include "ExportMeshDecimationDefinition.hxx"

static_assert(core3d::export_decimation::kMinimumClosedMeshTriangles == 4,
              "A closed triangular two-manifold cannot target fewer than four triangles.");
static_assert(core3d::export_decimation::kMaximumTriangleTarget >
                  core3d::export_decimation::kMinimumClosedMeshTriangles,
              "The decimation target bound must leave an admitted range.");
