#!/bin/bash
set -euo pipefail
# Same standalone C++17 invocation documented by the original prototype tests.
# The caller supplies a retained output directory; no temporary-file deletion.
: "${1:?usage: run_prototype_tests.sh OUTPUT_DIRECTORY}"
mkdir -p "$1"
output_dir=$(cd "$1" && pwd)
tests_dir=$(cd "$(dirname "$0")" && pwd)
for name in CurvedMeshUVPrototypeTests ToroidalMeshUVPrototypeTests CurvedUVPackerTests CurvedFaceUVUnwrapTests; do
    /usr/bin/clang++ -std=c++17 -Wall -Wextra -Werror -O2 "$tests_dir/$name.cpp" -o "$output_dir/$name"
    "$output_dir/$name"
done
# gp_Pnt2d is header-only for this detached math helper; no OCCT library or
# simulator is linked. Exact solid/history coverage lives in XCTest.
/usr/bin/clang++ -std=c++17 -Wall -Wextra -Werror -O2 \
    -isystem "$tests_dir/../Core3D/occt/inc" \
    "$tests_dir/PlanarSweepTangentArcTests.cpp" -o "$output_dir/PlanarSweepTangentArcTests"
"$output_dir/PlanarSweepTangentArcTests"
# Optional retained host OCCT libraries enable the native planar-loft regression.
# Explicit input avoids selecting iOS libraries or invoking Xcode/tool discovery.
if [[ -n "${2:-}" ]]; then
    /usr/bin/clang++ -std=c++17 -DDEBUG=1 -Wall -Wextra -Werror -Wno-deprecated-declarations -O2 \
        -I "$tests_dir/../Core3D/OCCTKit" -isystem "$tests_dir/../Core3D/occt/inc" \
        "$tests_dir/RectangularLoftPlanarBuilderTests.cpp" -L "$2" \
        -lTKOffset -lTKBool -lTKBO -lTKPrim -lTKShHealing -lTKTopAlgo -lTKGeomAlgo \
        -lTKBRep -lTKGeomBase -lTKG3d -lTKG2d -lTKMath -lTKernel \
        -lTKXCAF -lTKCAF -lTKLCAF -lTKCDF -o "$output_dir/RectangularLoftPlanarBuilderTests"
    "$output_dir/RectangularLoftPlanarBuilderTests"
else
    echo "SKIP RectangularLoftPlanarBuilderTests: pass retained host OCCT library directory as argument 2"
fi
