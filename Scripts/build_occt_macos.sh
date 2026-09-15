#!/usr/bin/env bash
# Guarded worker. Invoke only through build_locked.sh macos-dependencies.
set -euo pipefail

readonly OCCT_REPOSITORY="https://github.com/Open-Cascade-SAS/OCCT.git"
readonly OCCT_COMMIT="656b0d217fcc3f6611dfabc0206bd2d967ed5265"
readonly RAPIDJSON_REPOSITORY="https://github.com/Tencent/rapidjson.git"
readonly RAPIDJSON_COMMIT="24b5e7a8b27f42fa16b96fc70aade9106cf7102f"
readonly DEPLOYMENT_TARGET=14.2
readonly ARCHITECTURE=arm64
readonly CANONICAL_PREFIX=/shapeyard/occt-macos-build
readonly OWNER_TOKEN=shapeyard-occt-macos-v1

die() { printf 'error: %s\n' "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"; }

(( $# == 1 )) || die "usage: $0 /private/tmp/shapeyard-occt-macos-UUID"
root=$1
[[ "$root" =~ ^/private/tmp/shapeyard-occt-macos-[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] \
  || die "unsafe artifact root"
[[ -d "$root" && ! -L "$root" ]] || die "artifact root is missing or symlinked"
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
source "$script_dir/verify_disk_guard_ancestry.sh"
verify_disk_guard_ancestry
expected_marker=$(printf 'schema=shapeyard-disk-budget-owned-v2\nuid=%s\nroot=%s\ntoken=%s' \
  "$(id -u)" "$root" "$OWNER_TOKEN")
actual_marker=$(/bin/cat "$root/.shapeyard-disk-budget-owned" 2>/dev/null) \
  || die "guard marker is unavailable"
[[ "$actual_marker" == "$expected_marker" ]] || die "guard marker does not match"
if [[ -n "${SHAPEYARD_OCCT_MACOS_RESUME_ID:-}" ]]; then
  [[ "$root" == "/private/tmp/shapeyard-occt-macos-$SHAPEYARD_OCCT_MACOS_RESUME_ID" \
      && -f "$root/build/CMakeCache.txt" ]] || die "resume root/cache mismatch"
  for entry in occt rapidjson build install validation; do
    [[ -d "$root/$entry" && ! -L "$root/$entry" ]] || die "unsafe resume entry: $entry"
  done
  printf 'Resuming guard-owned macOS dependency cache: %s\n' "$root"
else
  [[ -z "$(find "$root" -mindepth 1 ! -name .shapeyard-disk-budget-owned -print -quit)" ]] \
    || die "artifact root is not empty"
fi

for tool in git cmake xcrun xcodebuild shasum awk sed find sort strings file; do need "$tool"; done
occt_source="$root/occt"
rapidjson_source="$root/rapidjson"
build_root="$root/build"
output_root="$root/install"
validation_root="$root/validation"
mkdir -p -m 700 "$output_root" "$validation_root"

checkout_pin() {
  local label=$1 repository=$2 commit=$3 destination=$4 actual
  if [[ -z "${SHAPEYARD_OCCT_MACOS_RESUME_ID:-}" ]]; then
    git init -q "$destination"
    git -C "$destination" remote add origin "$repository"
    git -C "$destination" fetch -q --depth=1 --no-tags origin "$commit"
    git -C "$destination" -c advice.detachedHead=false checkout -q --detach FETCH_HEAD
  fi
  actual=$(git -C "$destination" rev-parse HEAD)
  [[ "$actual" == "$commit" ]] || die "$label resolved to $actual"
  [[ "$(git -C "$destination" remote get-url origin)" == "$repository" ]] \
    || die "$label origin changed"
  [[ -z "$(git -C "$destination" status --porcelain --untracked-files=all)" ]] \
    || die "$label checkout is dirty"
}

checkout_pin OCCT "$OCCT_REPOSITORY" "$OCCT_COMMIT" "$occt_source"
checkout_pin RapidJSON "$RAPIDJSON_REPOSITORY" "$RAPIDJSON_COMMIT" "$rapidjson_source"
grep -Eq '^#define OCC_VERSION_COMPLETE +"7\.8\.0"$' \
  "$occt_source/src/Standard/Standard_Version.hxx" || die "OCCT is not 7.8.0"
[[ -f "$rapidjson_source/include/rapidjson/document.h" ]] || die "RapidJSON headers missing"

# This is the non-Draw static toolkit parity set currently vendored by Core3D,
# excluding the iOS-only TKOpenGles renderer. The desktop uses its existing Metal
# renderer; OCCT visualization/data classes remain available through TKV3d/TKService.
toolkits=(
  TKBO TKBRep TKBin TKBinL TKBinTObj TKBinXCAF TKBool TKCAF TKCDF TKDE
  TKDECascade TKDEGLTF TKDEIGES TKDEOBJ TKDEPLY TKDESTEP TKDESTL TKDEVRML
  TKExpress TKFeat TKFillet TKG2d TKG3d TKGeomAlgo TKGeomBase TKHLR TKLCAF
  TKMath TKMesh TKMeshVS TKOffset TKPrim TKRWMesh TKService TKShHealing TKStd
  TKStdL TKTObj TKTopAlgo TKV3d TKVCAF TKXCAF TKXMesh TKXSBase TKXml TKXmlL
  TKXmlTObj TKXmlXCAF TKernel
)
sdk_path=$(xcrun --sdk macosx --show-sdk-path)
flags="-ffile-prefix-map=$root=$CANONICAL_PREFIX"
flags+=" -fmacro-prefix-map=$root=$CANONICAL_PREFIX"
flags+=" -fdebug-prefix-map=$root=$CANONICAL_PREFIX"

cmake -S "$occt_source" -B "$build_root" -G 'Unix Makefiles' \
  -DCMAKE_POLICY_VERSION_MINIMUM:STRING=3.5 \
  -DCMAKE_OSX_SYSROOT:PATH="$sdk_path" \
  -DCMAKE_OSX_ARCHITECTURES:STRING="$ARCHITECTURE" \
  -DCMAKE_OSX_DEPLOYMENT_TARGET:STRING="$DEPLOYMENT_TARGET" \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DINSTALL_DIR:PATH="$output_root" \
  -DINSTALL_DIR_INCLUDE:PATH=include \
  -DINSTALL_DIR_LIB:PATH=lib \
  -DUSE_OPENGL:BOOL=OFF -DUSE_GLES2:BOOL=OFF \
  -DCMAKE_C_FLAGS:STRING="$flags" \
  -DCMAKE_CXX_FLAGS:STRING="$flags" \
  -DBUILD_LIBRARY_TYPE:STRING=Static -DBUILD_SHARED_LIBS:BOOL=OFF \
  -DBUILD_CPP_STANDARD:STRING=C++17 \
  -DBUILD_RELEASE_DISABLE_EXCEPTIONS:BOOL=OFF \
  -DBUILD_MODULE_FoundationClasses:BOOL=ON \
  -DBUILD_MODULE_ModelingData:BOOL=ON \
  -DBUILD_MODULE_ModelingAlgorithms:BOOL=ON \
  -DBUILD_MODULE_Visualization:BOOL=ON \
  -DBUILD_MODULE_ApplicationFramework:BOOL=ON \
  -DBUILD_MODULE_DataExchange:BOOL=ON \
  -DBUILD_MODULE_DETools:BOOL=ON \
  -DBUILD_MODULE_Draw:BOOL=OFF -DBUILD_Inspector:BOOL=OFF \
  -DBUILD_DOC_Overview:BOOL=OFF -DBUILD_SAMPLES_QT:BOOL=OFF \
  -DBUILD_YACCLEX:BOOL=OFF -DUSE_TK:BOOL=OFF \
  -DUSE_FREETYPE:BOOL=OFF -DUSE_FREEIMAGE:BOOL=OFF -DUSE_FFMPEG:BOOL=OFF \
  -DUSE_OPENVR:BOOL=OFF -DUSE_RAPIDJSON:BOOL=ON -DUSE_DRACO:BOOL=OFF \
  -DUSE_TBB:BOOL=OFF \
  -D3RDPARTY_RAPIDJSON_DIR:PATH="$rapidjson_source" \
  -D3RDPARTY_RAPIDJSON_INCLUDE_DIR:PATH="$rapidjson_source/include" \
  -DINSTALL_RAPIDJSON:BOOL=OFF

cache="$build_root/CMakeCache.txt"
grep -Fxq "CMAKE_OSX_ARCHITECTURES:STRING=$ARCHITECTURE" "$cache" || die "arch drift"
grep -Fxq "CMAKE_OSX_SYSROOT:PATH=$sdk_path" "$cache" || die "SDK drift"
grep -Fxq "CMAKE_OSX_DEPLOYMENT_TARGET:STRING=$DEPLOYMENT_TARGET" "$cache" \
  || die "deployment-target drift"
grep -Fxq 'CMAKE_BUILD_TYPE:STRING=Release' "$cache" || die "configuration drift"
grep -Fxq 'USE_RAPIDJSON:BOOL=ON' "$cache" || die "RapidJSON disabled"
grep -Fxq 'USE_FREETYPE:BOOL=OFF' "$cache" || die "FreeType unexpectedly enabled"
grep -Fxq 'BUILD_RELEASE_DISABLE_EXCEPTIONS:BOOL=OFF' "$cache" \
  || die "release exceptions unexpectedly disabled"
for setting in 'USE_OPENGL:BOOL=OFF' 'USE_GLES2:BOOL=OFF' \
  'BUILD_LIBRARY_TYPE:STRING=Static' 'BUILD_CPP_STANDARD:STRING=C++17'; do
  grep -Fxq "$setting" "$cache" || die "configuration drift: $setting"
done
cmake --build "$build_root" --parallel 4
cmake --install "$build_root" > "$root/install.log"

/usr/bin/python3 -I -B - "$output_root" "${toolkits[@]}" <<'PY'
import os, pathlib, sys
root = pathlib.Path(sys.argv[1])
expected = {"lib" + name + ".a" for name in sys.argv[2:]}
actual = {p.name for p in (root / "lib").glob("*.a")}
if actual != expected:
    raise SystemExit("Installed toolkit set differs: " + repr(actual.symmetric_difference(expected)))
for p in root.rglob("*"):
    if p.is_symlink() and not (p.relative_to(root).as_posix() == "bin/ExpToCasExe"
                              and os.readlink(p) == "ExpToCasExe-7.8.0"
                              and not (p.parent / os.readlink(p)).is_symlink()
                              and (p.parent / os.readlink(p)).is_file()):
        raise SystemExit("Unexpected installed symlink: " + str(p))
PY

mkdir -p -m 700 "$output_root/licenses"
cp "$occt_source/LICENSE_LGPL_21.txt" "$output_root/licenses/"
cp "$occt_source/OCCT_LGPL_EXCEPTION.txt" "$output_root/licenses/"
cp "$rapidjson_source/license.txt" "$output_root/licenses/RapidJSON-license.txt"

for toolkit in "${toolkits[@]}"; do
  # CMake's install-time ranlib may rewrite archive metadata. Qualify the
  # actual installed object payloads and platform, not incidental timestamps.
  archive="$output_root/lib/lib$toolkit.a"
  [[ "$(xcrun lipo -archs "$archive")" == arm64 ]] || die "$toolkit is not arm64-only"
  # Inspect the archive directly so duplicate member names cannot hide objects.
  xcrun ar -t "$archive" > "$validation_root/$toolkit.members"
  xcrun otool -l "$archive" > "$validation_root/$toolkit.load-commands"
  /usr/bin/python3 -I -B - "$archive" "$validation_root/$toolkit.members" \
    "$validation_root/$toolkit.load-commands" "$DEPLOYMENT_TARGET" <<'PY'
import pathlib, re, sys
archive, members_path, commands_path, minimum = sys.argv[1:]
members = [n for n in pathlib.Path(members_path).read_text().splitlines()
           if not n.startswith("__.SYMDEF")]
text = pathlib.Path(commands_path).read_text()
headers = list(re.finditer(r"^" + re.escape(archive) + r"\(([^\n]+)\):\s*$", text, re.M))
if not members or [h.group(1) for h in headers] != members:
    raise SystemExit("Archive member inventory differs from Mach-O inspection")
for index, header in enumerate(headers):
    block = text[header.end():headers[index + 1].start() if index + 1 < len(headers) else len(text)]
    if len(re.findall(r"^\s*cmd LC_BUILD_VERSION\s*$", block, re.M)) != 1:
        raise SystemExit("Missing or ambiguous build platform: " + header.group(1))
    if re.findall(r"^\s*platform (\S+)\s*$", block, re.M) not in (["1"], ["MACOS"]):
        raise SystemExit("Non-macOS archive member: " + header.group(1))
    if re.findall(r"^\s*minos (\S+)\s*$", block, re.M) != [minimum]:
        raise SystemExit("Unexpected deployment target: " + header.group(1))
print(archive + ": verified " + str(len(members)) + " macOS members")
PY
done

for header in Standard_Version.hxx TDocStd_Document.hxx XCAFDoc_ShapeTool.hxx \
  Poly_Triangulation.hxx RWGltf_CafReader.hxx; do
  cmp "$script_dir/../Core3D/occt/inc/$header" "$output_root/include/$header"
done

[[ -z "$(git -C "$occt_source" status --porcelain --untracked-files=all)" ]] \
  || die "build modified OCCT source"
[[ -z "$(git -C "$rapidjson_source" status --porcelain --untracked-files=all)" ]] \
  || die "build modified RapidJSON source"
# Installed public headers must be standalone, not generated build-tree forwarding headers.
/usr/bin/python3 -I -B - "$output_root/include" "$root" <<'PY'
import pathlib, sys
headers = list(pathlib.Path(sys.argv[1]).rglob("*.hxx"))
if not headers or any(sys.argv[2].encode() in p.read_bytes() for p in headers):
    raise SystemExit("Installed headers are missing or reference the ephemeral source root")
PY

xcrun --sdk macosx clang++ -std=c++17 -arch "$ARCHITECTURE" \
  -mmacosx-version-min="$DEPLOYMENT_TARGET" -isysroot "$sdk_path" \
  -I "$output_root/include" "$script_dir/probes/macos_occt_document_probe.cxx" \
  "$output_root"/lib/*.a -framework Cocoa -framework IOKit \
  -framework CoreGraphics -framework ImageIO -o "$validation_root/document-probe"
"$validation_root/document-probe" "$validation_root"
/usr/bin/python3 -I -B - "$validation_root/box.glb" <<'PY'
import json, pathlib, struct, sys
data = pathlib.Path(sys.argv[1]).read_bytes()
if struct.unpack_from("<4sII", data) != (b"glTF", 2, len(data)):
    raise SystemExit("Invalid exported GLB header")
size, kind = struct.unpack_from("<I4s", data, 12)
if kind != b"JSON":
    raise SystemExit("GLB JSON chunk missing")
scene = json.loads(data[20:20 + size])
primitives = [p for mesh in scene["meshes"] for p in mesh["primitives"]]
if not primitives or any(p.get("mode", 4) != 4 or "POSITION" not in p["attributes"]
                         or "NORMAL" not in p["attributes"] for p in primitives):
    raise SystemExit("Box GLB lacks triangle positions/normals")
if sum(scene["accessors"][p["indices"]]["count"] for p in primitives) != 36:
    raise SystemExit("Box GLB does not contain twelve triangles")
print("PASS: independent GLB structure and twelve-triangle box check")
PY

manifest="$output_root/OCCT_MACOS_ARM64_BUILD_MANIFEST.txt"
xcode_version=$(xcodebuild -version | tr '\n' ' ' | sed 's/ $//')
cmake_version=$(cmake --version | awk 'NR==1 {print $3}')
sdk_version=$(xcrun --sdk macosx --show-sdk-version)
{
  printf 'schema_version=1\ncomponent=OCCT_MACOS_STATIC\n'
  printf 'occt_commit=%s\nrapidjson_commit=%s\n' "$OCCT_COMMIT" "$RAPIDJSON_COMMIT"
  printf 'platform=MACOS\narchitectures=arm64\ndeployment_target=%s\n' "$DEPLOYMENT_TARGET"
  printf 'cpp_standard=C++17\nrelease_exceptions=enabled\nparallel_jobs=4\n'
  printf 'freetype=disabled\nocct_graphics_driver=none\n'
  printf 'xcode=%s\ncmake_version=%s\nmacosx_sdk_version=%s\n' \
    "$xcode_version" "$cmake_version" "$sdk_version"
  printf 'canonical_source_prefix=%s\n' "$CANONICAL_PREFIX"
  printf 'qualification=desktop-dependency-probe-only\n'
  printf 'permitted_symlink=bin/ExpToCasExe->ExpToCasExe-7.8.0\n'
  printf 'source_manifest_sha256=%s\n' "$SHAPEYARD_SOURCE_MANIFEST_SHA256"
  printf 'toolkits=%s\n' "$(IFS=,; printf '%s' "${toolkits[*]}")"
  (cd "$output_root" && find . -type f ! -name OCCT_MACOS_ARM64_BUILD_MANIFEST.txt -print | LC_ALL=C sort | \
    while IFS= read -r path; do shasum -a 256 "$path"; done)
} > "$manifest"
chmod 600 "$manifest"
grep -E '^[0-9a-f]{64}  ' "$manifest" > "$validation_root/package-inventory.sha256"
(cd "$output_root" && shasum -a 256 -c "$validation_root/package-inventory.sha256") \
  > "$validation_root/package-verification.log"
printf 'package_manifest_sha256=%s\n' "$(shasum -a 256 "$manifest" | awk '{print $1}')"
printf 'Validated external macOS arm64 OCCT package: %s\n' "$output_root"
