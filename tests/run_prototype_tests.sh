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
