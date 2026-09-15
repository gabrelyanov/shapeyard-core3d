#!/usr/bin/env bash
# Guarded worker: run_v1_topology_checkpoint.sh --macos-core --package INSTALL.
set -euo pipefail
die() { printf 'error: %s\n' "$*" >&2; exit 78; }
(( $# == 1 )) || die "one owned build root is required"
root=$1
[[ "$root" =~ ^/private/tmp/shapeyard-macos-core-[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] || die "unsafe build root"
[[ -d "$root" && ! -L "$root" ]] || die "missing owned build root"
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
core_root=$(cd "$script_dir/.." && pwd -P)
source "$script_dir/verify_disk_guard_ancestry.sh"
verify_disk_guard_ancestry
expected_marker=$(printf 'schema=shapeyard-disk-budget-owned-v2\nuid=%s\nroot=%s\ntoken=shapeyard-macos-core-v1' "$(id -u)" "$root")
[[ "$(cat "$root/.shapeyard-disk-budget-owned")" == "$expected_marker" ]] || die "guard ownership mismatch"
[[ -z "$(find "$root" -mindepth 1 ! -name .shapeyard-disk-budget-owned -print -quit)" ]] || die "build root is not empty"
package=${SHAPEYARD_OCCT_MACOS_ROOT:-}
[[ "$package" == /* ]] || die "absolute qualified package path required"
[[ "$(uname -m)" == arm64 ]] || die "arm64 host required"
command -v xcodegen >/dev/null || die "xcodegen unavailable"
mkdir -m 700 "$root/project" "$root/validation"
/usr/bin/python3 -I -B "$script_dir/verify_occt_macos_package.py" "$package" | tee "$root/validation/package-before.txt"

# Stage project metadata only. Every implementation still references canonical
# source, which the surrounding checkpoint seals through terminal cleanup.
/usr/bin/python3 -I -B - "$core_root" "$root/project" <<'PY'
from pathlib import Path
import shutil
import sys
core, output = map(Path, sys.argv[1:])
source = str(core / "Core3D")
if any(character in source for character in '\n\r"$'):
    raise SystemExit("unsupported source path characters")
spec = (core / "Mac/project.yml").read_text()
(output / "project.yml").write_text(spec.replace("@CORE3D_SOURCE_ROOT@", source))
shutil.copyfile(core / "Mac/Core3D.h", output / "Core3D.h")
PY
xcodegen generate --spec "$root/project/project.yml" --project "$root/project"
xcodebuild -project "$root/project/Core3D-macOS.xcodeproj" -scheme Core3D \
  -configuration Release -destination 'generic/platform=macOS' \
  -derivedDataPath "$root/DerivedData" -resultBundlePath "$root/core-build.xcresult" \
  -jobs 2 "SHAPEYARD_OCCT_MACOS_ROOT=$package" CODE_SIGNING_ALLOWED=NO build
binary="$root/DerivedData/Build/Products/Release/Core3D.framework/Versions/A/Core3D"
[[ -f "$binary" ]] || die "shared framework binary missing"
[[ "$(xcrun lipo -archs "$binary")" == arm64 ]] || die "unexpected framework architecture"
xcrun vtool -show-build "$binary" > "$root/validation/framework-platform.txt"
grep -Eq 'platform MACOS$' "$root/validation/framework-platform.txt" || die "framework is not macOS"
grep -Eq 'minos 14\.2$' "$root/validation/framework-platform.txt" || die "unexpected deployment target"
shasum -a 256 "$binary" > "$root/validation/framework.sha256"
framework_dir="$root/DerivedData/Build/Products/Release"
"$framework_dir/Core3DViewerProbe" | tee "$root/validation/shared-viewer-probe.txt"
"$framework_dir/Core3DMetalRendererProbe" | tee "$root/validation/shared-metal-renderer-probe.txt"
"$framework_dir/ShapeyardMac.app/Contents/MacOS/ShapeyardMac" --checkpoint "$root" | tee "$root/validation/mac-document-window-probe.txt"
xcrun --sdk macosx clang++ -std=c++20 -fobjc-arc -arch arm64 \
  -mmacosx-version-min=14.2 -I "$package/include" -I "$core_root/Core3D/OCCTKit" \
  "$script_dir/macos_core3d_image_export_probe.mm" \
  -F "$framework_dir" -framework Core3D -framework Foundation \
  -framework CoreGraphics -framework CoreImage -framework ImageIO \
  "-Wl,-rpath,$framework_dir" -o "$root/validation/image-export-probe"
"$root/validation/image-export-probe" | tee "$root/validation/image-export-probe.txt"
/usr/bin/python3 -I -B "$script_dir/verify_occt_macos_package.py" "$package" | tee "$root/validation/package-after.txt"
printf 'Shared Core3D framework, native viewer and PNG probes passed for macOS arm64; editor UI qualification remains pending.\n'
