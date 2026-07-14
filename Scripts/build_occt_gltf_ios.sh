#!/bin/bash

# Rebuild only OCCT's TKDEGLTF static archive with RapidJSON enabled for the
# architectures shipped by Core3D. Companion OCCT archives and headers are
# deliberately left untouched.

set -euo pipefail

readonly OCCT_REPOSITORY="https://github.com/Open-Cascade-SAS/OCCT.git"
readonly OCCT_COMMIT="656b0d217fcc3f6611dfabc0206bd2d967ed5265"
readonly RAPIDJSON_REPOSITORY="https://github.com/Tencent/rapidjson.git"
readonly RAPIDJSON_COMMIT="24b5e7a8b27f42fa16b96fc70aade9106cf7102f"
readonly DEPLOYMENT_TARGET="16.0"
readonly RAPIDJSON_FALLBACK_TEXT="OCCT has been built without RapidJSON support"
readonly RAPIDJSON_ENABLED_TEXT="Invalid glTF syntax"

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd -P)"
PROJECT_ROOT="$(cd "${REPOSITORY_ROOT}/.." && pwd -P)"
DISK_GUARD="${PROJECT_ROOT}/Scripts/with_disk_budget.sh"
VENDORED_INCLUDE_DIRECTORY="${REPOSITORY_ROOT}/Core3D/occt/inc"
DEVICE_DESTINATION="${REPOSITORY_ROOT}/Core3D/occt/lib/libTKDEGLTF.a"
SIMULATOR_DESTINATION="${REPOSITORY_ROOT}/Core3D/occt/lib_sim/libTKDEGLTF.a"
NOTICES_DIRECTORY="${REPOSITORY_ROOT}/ThirdPartyNotices"
MANIFEST_DESTINATION="${REPOSITORY_ROOT}/Core3D/occt/TKDEGLTF_BUILD_MANIFEST.txt"
LOCK_DIRECTORY="${REPOSITORY_ROOT}/.occt-gltf-build.lock"
WORK_DIRECTORY_IS_AUTOMATIC=0

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

note() {
  printf '==> %s\n' "$*"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

require_file() {
  [[ -f "$1" ]] || die "required file not found: $1"
}

acquire_build_lock() {
  local lock_host lock_started
  if ! mkdir "${LOCK_DIRECTORY}" 2>/dev/null; then
    die "another TKDEGLTF rebuild may be active; inspect the fail-closed lock at ${LOCK_DIRECTORY}"
  fi
  LOCK_HELD=1
  lock_host="$(hostname)"
  lock_started="$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
  if ! printf 'pid=%s\nhost=%s\nstarted_utc=%s\ncommand=%s\nwork_directory=%s\n' \
    "$$" "${lock_host}" "${lock_started}" \
    'Scripts/build_occt_gltf_ios.sh' "${WORK_DIRECTORY}" \
    > "${LOCK_DIRECTORY}/owner"; then
    rm -f "${LOCK_DIRECTORY}/owner"
    rmdir "${LOCK_DIRECTORY}" 2>/dev/null || true
    LOCK_HELD=0
    die "could not record TKDEGLTF build-lock ownership"
  fi
  note "Acquired exclusive repository build lock"
}

check_cmake_version() {
  local version major minor
  version="$(cmake --version | awk 'NR == 1 { print $3 }')"
  major="${version%%.*}"
  version="${version#*.}"
  minor="${version%%.*}"

  [[ "${major}" =~ ^[0-9]+$ && "${minor}" =~ ^[0-9]+$ ]] \
    || die "could not parse the CMake version"
  if (( major < 3 || (major == 3 && minor < 21) )); then
    die "CMake 3.21 or newer is required (found ${major}.${minor})"
  fi
}

prepare_work_directory() {
  local work_token=$1
  local requested_directory root_name
  local parent_device root_device root_owner root_mode
  if [[ -n "${OCCT_GLTF_WORK_DIR:-}" ]]; then
    requested_directory="${OCCT_GLTF_WORK_DIR}"
    [[ "${requested_directory}" =~ ^(/private)?/tmp/shapeyard-[A-Za-z0-9._-]+$ ]] \
      || die "OCCT_GLTF_WORK_DIR must be a single /tmp/shapeyard-* directory"
    root_name="${requested_directory##*/}"
    WORK_DIRECTORY="/private/tmp/${root_name}"
    if [[ ! -e "${WORK_DIRECTORY}" ]]; then
      mkdir -m 700 "${WORK_DIRECTORY}"
    fi
    [[ -z "$(find "${WORK_DIRECTORY}" -mindepth 1 -print -quit)" ]] \
      || die "OCCT_GLTF_WORK_DIR must be empty: ${WORK_DIRECTORY}"
    [[ -d "${WORK_DIRECTORY}" && ! -L "${WORK_DIRECTORY}" ]] \
      || die "work directory must be a real directory, not a symlink"
    WORK_DIRECTORY="$(cd "${WORK_DIRECTORY}" && pwd -P)"
    [[ "${WORK_DIRECTORY}" =~ ^/private/tmp/shapeyard-[A-Za-z0-9._-]+$ ]] \
      || die "work directory must be a single /tmp/shapeyard-* directory: ${WORK_DIRECTORY}"

    parent_device="$(stat -f %d /private/tmp)"
    root_device="$(stat -f %d "${WORK_DIRECTORY}")"
    root_owner="$(stat -f %u "${WORK_DIRECTORY}")"
    root_mode="$(stat -f %Lp "${WORK_DIRECTORY}")"
    [[ "${root_device}" == "${parent_device}" \
        && "${root_owner}" == "$(id -u)" \
        && "${root_mode}" == "700" ]] \
      || die "work directory must be current-user-owned mode 0700 on /private/tmp"
  else
    WORK_DIRECTORY="/private/tmp/shapeyard-occt-gltf-ios.${work_token}"
    [[ ! -e "${WORK_DIRECTORY}" && ! -L "${WORK_DIRECTORY}" ]] \
      || die "automatic OCCT work directory already exists: ${WORK_DIRECTORY}"
    WORK_DIRECTORY_IS_AUTOMATIC=1
  fi
  note "Work directory: ${WORK_DIRECTORY}"
}

checkout_pinned_source() {
  local name repository commit destination actual_commit actual_remote
  name="$1"
  repository="$2"
  commit="$3"
  destination="$4"

  note "Fetching ${name} at ${commit}"
  git init -q "${destination}"
  git -C "${destination}" remote add origin "${repository}"
  git -C "${destination}" fetch -q --depth=1 --no-tags origin "${commit}"
  git -C "${destination}" -c advice.detachedHead=false checkout -q --detach FETCH_HEAD

  actual_commit="$(git -C "${destination}" rev-parse HEAD)"
  [[ "${actual_commit}" == "${commit}" ]] \
    || die "${name} resolved to ${actual_commit}, expected ${commit}"
  actual_remote="$(git -C "${destination}" remote get-url origin)"
  [[ "${actual_remote}" == "${repository}" ]] \
    || die "${name} origin is ${actual_remote}, expected ${repository}"
  [[ -z "$(git -C "${destination}" status --porcelain --untracked-files=all)" ]] \
    || die "${name} checkout is not clean"
}

validate_pinned_sources() {
  local package_manifest
  package_manifest="${OCCT_SOURCE_DIRECTORY}/src/TKDEGLTF/PACKAGES"

  require_file "${package_manifest}"
  require_file "${OCCT_SOURCE_DIRECTORY}/src/RWGltf/RWGltf_CafReader.cxx"
  require_file "${OCCT_SOURCE_DIRECTORY}/src/RWGltf/RWGltf_GltfJsonParser.cxx"
  require_file "${RAPIDJSON_SOURCE_DIRECTORY}/include/rapidjson/document.h"
  [[ "$(tr -d '[:space:]' < "${package_manifest}")" == "RWGltf" ]] \
    || die "unexpected TKDEGLTF package manifest"
  grep -Fq '#ifdef HAVE_RAPIDJSON' \
    "${OCCT_SOURCE_DIRECTORY}/src/RWGltf/RWGltf_GltfJsonParser.cxx" \
    || die "pinned glTF parser does not contain the RapidJSON feature gate"
  grep -Fq 'int maxExp = (expFrac + 2147483639) / 10;' \
    "${RAPIDJSON_SOURCE_DIRECTORY}/include/rapidjson/reader.h" \
    || die "RapidJSON is missing the exponent-underflow fix"

  # TKDEGLTF is intentionally compiled without rebuilding the rest of OCCT.
  # Its transitive public includes therefore come from Core3D's complete,
  # already-vendored OCCT include tree. Check the critical direct and
  # transitive headers against this exact upstream checkout before using it.
  cmp -s \
    "${VENDORED_INCLUDE_DIRECTORY}/RWGltf_CafReader.hxx" \
    "${OCCT_SOURCE_DIRECTORY}/src/RWGltf/RWGltf_CafReader.hxx" \
    || die "vendored RWGltf_CafReader.hxx does not match pinned OCCT"
  cmp -s \
    "${VENDORED_INCLUDE_DIRECTORY}/BinXCAFDrivers.hxx" \
    "${OCCT_SOURCE_DIRECTORY}/src/BinXCAFDrivers/BinXCAFDrivers.hxx" \
    || die "vendored BinXCAFDrivers.hxx does not match pinned OCCT"
  grep -Eq '^#define OCC_VERSION_COMPLETE +"7\.8\.0"$' \
    "${VENDORED_INCLUDE_DIRECTORY}/Standard_Version.hxx" \
    || die "vendored OCCT headers are not version 7.8.0"

  cmp -s \
    "${NOTICES_DIRECTORY}/OpenCASCADE/LICENSE_LGPL_21.txt" \
    "${OCCT_SOURCE_DIRECTORY}/LICENSE_LGPL_21.txt" \
    || die "vendored OCCT LGPL text differs from pinned upstream"
  cmp -s \
    "${NOTICES_DIRECTORY}/OpenCASCADE/OCCT_LGPL_EXCEPTION.txt" \
    "${OCCT_SOURCE_DIRECTORY}/OCCT_LGPL_EXCEPTION.txt" \
    || die "vendored OCCT exception text differs from pinned upstream"
  cmp -s \
    "${NOTICES_DIRECTORY}/RapidJSON/license.txt" \
    "${RAPIDJSON_SOURCE_DIRECTORY}/license.txt" \
    || die "vendored RapidJSON license text differs from pinned upstream"
}

configure_and_build() {
  local sdk architectures build_directory sdk_path archive canonical_flags
  sdk="$1"
  architectures="$2"
  build_directory="$3"

  sdk_path="$(xcrun --sdk "${sdk}" --show-sdk-path)"
  [[ -d "${sdk_path}" ]] || die "SDK path does not exist: ${sdk_path}"
  canonical_flags="-I${VENDORED_INCLUDE_DIRECTORY}"
  canonical_flags+=" -ffile-prefix-map=${WORK_DIRECTORY}=/shapeyard/occt-gltf-build"
  canonical_flags+=" -fmacro-prefix-map=${WORK_DIRECTORY}=/shapeyard/occt-gltf-build"
  canonical_flags+=" -fdebug-prefix-map=${WORK_DIRECTORY}=/shapeyard/occt-gltf-build"
  canonical_flags+=" -ffile-prefix-map=${REPOSITORY_ROOT}=/shapeyard/core3d"
  canonical_flags+=" -fmacro-prefix-map=${REPOSITORY_ROOT}=/shapeyard/core3d"
  canonical_flags+=" -fdebug-prefix-map=${REPOSITORY_ROOT}=/shapeyard/core3d"

  note "Configuring TKDEGLTF for ${sdk} (${architectures})"
  cmake \
    -S "${OCCT_SOURCE_DIRECTORY}" \
    -B "${build_directory}" \
    -G Xcode \
    -DCMAKE_POLICY_VERSION_MINIMUM:STRING=3.5 \
    -DCMAKE_SYSTEM_NAME=iOS \
    -DCMAKE_OSX_SYSROOT:PATH="${sdk_path}" \
    -DCMAKE_OSX_ARCHITECTURES:STRING="${architectures}" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET:STRING="${DEPLOYMENT_TARGET}" \
    -DCMAKE_TRY_COMPILE_TARGET_TYPE:STRING=STATIC_LIBRARY \
    -DCMAKE_XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH:STRING=NO \
    -DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_ALLOWED:STRING=NO \
    -DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED:STRING=NO \
    -DCMAKE_XCODE_ATTRIBUTE_ENABLE_BITCODE:STRING=NO \
    -DCMAKE_CXX_FLAGS:STRING="${canonical_flags}" \
    -DBUILD_LIBRARY_TYPE:STRING=Static \
    -DBUILD_SHARED_LIBS:BOOL=OFF \
    -DBUILD_CPP_STANDARD:STRING=C++11 \
    -DBUILD_ADDITIONAL_TOOLKITS:STRING=TKDEGLTF \
    -DBUILD_MODULE_FoundationClasses:BOOL=OFF \
    -DBUILD_MODULE_ModelingData:BOOL=OFF \
    -DBUILD_MODULE_ModelingAlgorithms:BOOL=OFF \
    -DBUILD_MODULE_Visualization:BOOL=OFF \
    -DBUILD_MODULE_ApplicationFramework:BOOL=OFF \
    -DBUILD_MODULE_DataExchange:BOOL=OFF \
    -DBUILD_MODULE_DETools:BOOL=OFF \
    -DBUILD_MODULE_Draw:BOOL=OFF \
    -DBUILD_DOC_Overview:BOOL=OFF \
    -DBUILD_Inspector:BOOL=OFF \
    -DBUILD_SAMPLES_QT:BOOL=OFF \
    -DBUILD_YACCLEX:BOOL=OFF \
    -DUSE_TK:BOOL=OFF \
    -DUSE_FREETYPE:BOOL=OFF \
    -DUSE_FREEIMAGE:BOOL=OFF \
    -DUSE_FFMPEG:BOOL=OFF \
    -DUSE_OPENVR:BOOL=OFF \
    -DUSE_RAPIDJSON:BOOL=ON \
    -DUSE_DRACO:BOOL=OFF \
    -DUSE_TBB:BOOL=OFF \
    -D3RDPARTY_RAPIDJSON_DIR:PATH="${RAPIDJSON_SOURCE_DIRECTORY}" \
    -D3RDPARTY_RAPIDJSON_INCLUDE_DIR:PATH="${RAPIDJSON_SOURCE_DIRECTORY}/include" \
    -DINSTALL_RAPIDJSON:BOOL=OFF

  grep -Fq 'USE_RAPIDJSON:BOOL=ON' "${build_directory}/CMakeCache.txt" \
    || die "CMake did not enable RapidJSON for ${sdk}"
  grep -Fq "CMAKE_OSX_ARCHITECTURES:STRING=${architectures}" \
    "${build_directory}/CMakeCache.txt" \
    || die "CMake did not retain the requested architectures for ${sdk}"
  grep -Fq "CMAKE_OSX_DEPLOYMENT_TARGET:STRING=${DEPLOYMENT_TARGET}" \
    "${build_directory}/CMakeCache.txt" \
    || die "CMake did not retain the iOS deployment target for ${sdk}"
  grep -Fq "CMAKE_OSX_SYSROOT:PATH=${sdk_path}" \
    "${build_directory}/CMakeCache.txt" \
    || die "CMake did not retain the selected SDK for ${sdk}"
  grep -Fq "3RDPARTY_RAPIDJSON_INCLUDE_DIR:PATH=${RAPIDJSON_SOURCE_DIRECTORY}/include" \
    "${build_directory}/CMakeCache.txt" \
    || die "CMake did not use the pinned RapidJSON headers for ${sdk}"
  grep -Fq "CMAKE_CXX_FLAGS:STRING=${canonical_flags}" \
    "${build_directory}/CMakeCache.txt" \
    || die "CMake did not retain the canonicalized OCCT compiler flags for ${sdk}"

  # Deliberately compile this one target with one build job. OCCT's generated
  # Xcode target does not resolve or build its companion toolkit targets in
  # this all-modules-off configuration; their pinned public headers above are
  # sufficient to compile TKDEGLTF, and their existing archives stay intact.
  note "Building only TKDEGLTF for ${sdk}, sequentially"
  cmake --build "${build_directory}" \
    --config Release \
    --target TKDEGLTF \
    --parallel 1

  [[ -z "$(find "${build_directory}" -type f -name 'libTK*.a' ! -name libTKDEGLTF.a -print -quit)" ]] \
    || die "the ${sdk} build produced an unexpected companion OCCT archive"
  [[ -z "$(git -C "${OCCT_SOURCE_DIRECTORY}" status --porcelain --untracked-files=all)" ]] \
    || die "the ${sdk} build modified the pinned OCCT source checkout"
  [[ -z "$(git -C "${RAPIDJSON_SOURCE_DIRECTORY}" status --porcelain --untracked-files=all)" ]] \
    || die "the ${sdk} build modified the pinned RapidJSON source checkout"

  # Multi-architecture Xcode builds retain one thin intermediate archive per
  # slice in addition to this lipo-combined product. Select the stable CMake
  # output path instead of treating those expected intermediates as outputs.
  archive="${build_directory}/mac64/clang/lib/libTKDEGLTF.a"
  require_file "${archive}"
  BUILT_ARCHIVE="${archive}"
}

validate_archive() {
  local archive label report_prefix actual_architecture architecture expected_count actual_count
  local expected_platform
  archive="$1"
  label="$2"
  shift 2
  expected_count="$#"
  report_prefix="${VALIDATION_DIRECTORY}/${label}"
  case "${label}" in
    iphoneos) expected_platform="IOS" ;;
    iphonesimulator) expected_platform="IOSSIMULATOR" ;;
    *) die "unknown archive platform label: ${label}" ;;
  esac

  require_file "${archive}"
  [[ -s "${archive}" ]] || die "${label} archive is empty"
  actual_count="$(xcrun lipo -archs "${archive}" | wc -w | tr -d '[:space:]')"
  [[ "${actual_count}" == "${expected_count}" ]] \
    || die "${label} archive has unexpected slices: $(xcrun lipo -archs "${archive}")"

  for architecture in "$@"; do
    xcrun lipo "${archive}" -verify_arch "${architecture}" >/dev/null \
      || die "${label} archive is missing ${architecture}"
  done
  for actual_architecture in $(xcrun lipo -archs "${archive}"); do
    case " $* " in
      *" ${actual_architecture} "*) ;;
      *) die "${label} archive contains unexpected ${actual_architecture} slice" ;;
    esac
  done

  for architecture in "$@"; do
    local thin_archive members_file strings_file undefined_file defined_file
    local defined_symbols expected_objects actual_objects symbol objects_directory member build_report
    thin_archive="${report_prefix}-${architecture}.a"
    members_file="${report_prefix}-${architecture}.members"
    strings_file="${report_prefix}-${architecture}.strings"
    undefined_file="${report_prefix}-${architecture}.undefined"
    defined_file="${report_prefix}-${architecture}.defined"
    defined_symbols="${report_prefix}-${architecture}.defined-symbols"
    expected_objects="${report_prefix}-${architecture}.expected-objects"
    actual_objects="${report_prefix}-${architecture}.actual-objects"
    objects_directory="${report_prefix}-${architecture}.objects"

    xcrun lipo "${archive}" -thin "${architecture}" -output "${thin_archive}"
    xcrun ar -t "${thin_archive}" > "${members_file}"
    sed -n 's/\.cxx$/.o/p' "${OCCT_SOURCE_DIRECTORY}/src/RWGltf/FILES" \
      | LC_ALL=C sort > "${expected_objects}"
    grep -E '\.o$' "${members_file}" \
      | LC_ALL=C sort > "${actual_objects}"
    cmp -s "${expected_objects}" "${actual_objects}" \
      || die "${label}/${architecture} does not contain exactly the TKDEGLTF source objects"

    mkdir -p "${objects_directory}"
    (cd "${objects_directory}" && xcrun ar -x "${thin_archive}")
    while IFS= read -r member; do
      build_report="${objects_directory}/${member}.build-version"
      xcrun vtool -show-build "${objects_directory}/${member}" > "${build_report}"
      grep -Eq "^[[:space:]]*platform ${expected_platform}$" "${build_report}" \
        || die "${label}/${architecture}/${member} has the wrong Apple platform"
      grep -Eq "^[[:space:]]*minos ${DEPLOYMENT_TARGET}$" "${build_report}" \
        || die "${label}/${architecture}/${member} has the wrong deployment target"
    done < "${actual_objects}"

    xcrun strings "${thin_archive}" > "${strings_file}"
    if grep -Fq "${RAPIDJSON_FALLBACK_TEXT}" "${strings_file}"; then
      die "${label}/${architecture} contains OCCT's RapidJSON-disabled fallback"
    fi
    grep -Fq "${RAPIDJSON_ENABLED_TEXT}" "${strings_file}" \
      || die "${label}/${architecture} is missing RapidJSON-enabled parser code"
    if grep -Fq "${WORK_DIRECTORY}" "${strings_file}"; then
      die "${label}/${architecture} embeds the random build directory"
    fi
    if grep -Fq "${REPOSITORY_ROOT}" "${strings_file}"; then
      die "${label}/${architecture} embeds the local repository path"
    fi
    grep -Fq '/shapeyard/occt-gltf-build/occt/src/RWGltf/' "${strings_file}" \
      || die "${label}/${architecture} is missing canonical source provenance"

    xcrun nm -u "${thin_archive}" > "${undefined_file}"
    xcrun nm -gU "${thin_archive}" > "${defined_file}"
    awk 'NF >= 3 { print $NF }' "${defined_file}" \
      | LC_ALL=C sort -u > "${defined_symbols}"
    # Static archives report cross-object calls as undefined at the member
    # level. Accept a RapidJSON-named reference only when the same mangled
    # symbol is defined by another TKDEGLTF member; reject true external
    # RapidJSON link dependencies.
    while IFS= read -r symbol; do
      grep -Fxq "${symbol}" "${defined_symbols}" \
        || die "${label}/${architecture} contains unresolved RapidJSON symbol ${symbol}"
    done < <(awk 'NF == 1 && tolower($1) ~ /rapidjson/ { print $1 }' \
      "${undefined_file}")
  done

  note "Validated ${label}: $(xcrun lipo -archs "${archive}")"
}

write_pair_manifest() {
  local destination device_archive simulator_archive
  local xcode_version xcode_build cmake_version
  local iphoneos_sdk iphoneos_sdk_build simulator_sdk simulator_sdk_build
  local device_sha256 simulator_sha256
  destination="$1"
  device_archive="$2"
  simulator_archive="$3"

  xcode_version="$(xcodebuild -version | awk 'NR == 1 { print $2 }')"
  xcode_build="$(xcodebuild -version | awk 'NR == 2 { print $3 }')"
  cmake_version="$(cmake --version | awk 'NR == 1 { print $3 }')"
  iphoneos_sdk="$(xcrun --sdk iphoneos --show-sdk-version)"
  iphoneos_sdk_build="$(xcrun --sdk iphoneos --show-sdk-build-version)"
  simulator_sdk="$(xcrun --sdk iphonesimulator --show-sdk-version)"
  simulator_sdk_build="$(xcrun --sdk iphonesimulator --show-sdk-build-version)"
  device_sha256="$(shasum -a 256 "${device_archive}" | awk '{ print $1 }')"
  simulator_sha256="$(shasum -a 256 "${simulator_archive}" | awk '{ print $1 }')"

  {
    printf 'schema_version=1\n'
    printf 'component=OCCT_TKDEGLTF\n'
    printf 'occt_commit=%s\n' "${OCCT_COMMIT}"
    printf 'rapidjson_commit=%s\n' "${RAPIDJSON_COMMIT}"
    printf 'deployment_target=%s\n' "${DEPLOYMENT_TARGET}"
    printf 'xcode_version=%s\n' "${xcode_version}"
    printf 'xcode_build=%s\n' "${xcode_build}"
    printf 'cmake_version=%s\n' "${cmake_version}"
    printf 'iphoneos_sdk_version=%s\n' "${iphoneos_sdk}"
    printf 'iphoneos_sdk_build=%s\n' "${iphoneos_sdk_build}"
    printf 'iphonesimulator_sdk_version=%s\n' "${simulator_sdk}"
    printf 'iphonesimulator_sdk_build=%s\n' "${simulator_sdk_build}"
    printf 'device_platform=IOS\n'
    printf 'device_architectures=arm64,arm64e\n'
    printf 'device_archive=Core3D/occt/lib/libTKDEGLTF.a\n'
    printf 'device_sha256=%s\n' "${device_sha256}"
    printf 'simulator_platform=IOSSIMULATOR\n'
    printf 'simulator_architectures=arm64,x86_64\n'
    printf 'simulator_archive=Core3D/occt/lib_sim/libTKDEGLTF.a\n'
    printf 'simulator_sha256=%s\n' "${simulator_sha256}"
    printf 'canonical_source_prefix=/shapeyard/occt-gltf-build\n'
  } > "${destination}"
  chmod 0644 "${destination}"
}

validate_pair_manifest() {
  local manifest device_archive simulator_archive device_sha256 simulator_sha256 key count
  manifest="$1"
  device_archive="$2"
  simulator_archive="$3"
  device_sha256="$(shasum -a 256 "${device_archive}" | awk '{ print $1 }')"
  simulator_sha256="$(shasum -a 256 "${simulator_archive}" | awk '{ print $1 }')"

  require_file "${manifest}"
  for key in schema_version component occt_commit rapidjson_commit deployment_target \
    xcode_version xcode_build cmake_version iphoneos_sdk_version iphoneos_sdk_build \
    iphonesimulator_sdk_version iphonesimulator_sdk_build device_platform \
    device_architectures device_archive device_sha256 simulator_platform \
    simulator_architectures simulator_archive simulator_sha256 canonical_source_prefix; do
    count="$(awk -F= -v key="${key}" '$1 == key { count += 1 } END { print count + 0 }' "${manifest}")"
    [[ "${count}" == "1" ]] || die "pair manifest must contain exactly one ${key} entry"
  done
  grep -Fxq 'schema_version=1' "${manifest}" \
    || die "pair manifest has an unsupported schema"
  grep -Fxq 'component=OCCT_TKDEGLTF' "${manifest}" \
    || die "pair manifest has an unexpected component"
  grep -Fxq "occt_commit=${OCCT_COMMIT}" "${manifest}" \
    || die "pair manifest has an unexpected OCCT commit"
  grep -Fxq "rapidjson_commit=${RAPIDJSON_COMMIT}" "${manifest}" \
    || die "pair manifest has an unexpected RapidJSON commit"
  grep -Fxq "deployment_target=${DEPLOYMENT_TARGET}" "${manifest}" \
    || die "pair manifest has an unexpected deployment target"
  grep -Fxq 'device_platform=IOS' "${manifest}" \
    || die "pair manifest has an unexpected device platform"
  grep -Fxq 'device_architectures=arm64,arm64e' "${manifest}" \
    || die "pair manifest has unexpected device architectures"
  grep -Fxq 'device_archive=Core3D/occt/lib/libTKDEGLTF.a' "${manifest}" \
    || die "pair manifest has an unexpected device archive path"
  grep -Fxq "device_sha256=${device_sha256}" "${manifest}" \
    || die "pair manifest does not match the device archive"
  grep -Fxq 'simulator_platform=IOSSIMULATOR' "${manifest}" \
    || die "pair manifest has an unexpected simulator platform"
  grep -Fxq 'simulator_architectures=arm64,x86_64' "${manifest}" \
    || die "pair manifest has unexpected simulator architectures"
  grep -Fxq 'simulator_archive=Core3D/occt/lib_sim/libTKDEGLTF.a' "${manifest}" \
    || die "pair manifest has an unexpected simulator archive path"
  grep -Fxq "simulator_sha256=${simulator_sha256}" "${manifest}" \
    || die "pair manifest does not match the simulator archive"
  grep -Fxq 'canonical_source_prefix=/shapeyard/occt-gltf-build' "${manifest}" \
    || die "pair manifest has an unexpected canonical source prefix"
  if grep -Fq "${WORK_DIRECTORY}" "${manifest}"; then
    die "pair manifest embeds the random build directory"
  fi
  if grep -Fq "${REPOSITORY_ROOT}" "${manifest}"; then
    die "pair manifest embeds the local repository path"
  fi
}

restore_file_atomically() {
  local backup destination restore
  backup="$1"
  destination="$2"
  restore=""
  if ! restore="$(mktemp "${TRANSACTION_DIRECTORY}/rollback.XXXXXX")"; then
    printf 'error: could not create rollback temp for %s\n' "${destination}" >&2
    return 1
  fi
  if ! cp "${backup}" "${restore}" \
    || ! chmod 0644 "${restore}" \
    || ! cmp -s "${backup}" "${restore}"; then
    printf 'error: could not stage a verified rollback copy for %s\n' "${destination}" >&2
    rm -f "${restore}"
    return 1
  fi
  if ! mv -f "${restore}" "${destination}"; then
    printf 'error: could not install rollback copy for %s\n' "${destination}" >&2
    rm -f "${restore}"
    return 1
  fi
}

rollback_installation() {
  local rollback_status
  note "Rolling back libTKDEGLTF archive replacement"
  rollback_status=0
  restore_file_atomically "${DEVICE_BACKUP}" "${DEVICE_DESTINATION}" || rollback_status=1
  restore_file_atomically "${SIMULATOR_BACKUP}" "${SIMULATOR_DESTINATION}" || rollback_status=1
  if [[ "${MANIFEST_PREEXISTED:-0}" == "1" ]]; then
    restore_file_atomically "${MANIFEST_BACKUP}" "${MANIFEST_DESTINATION}" || rollback_status=1
  else
    rm -f "${MANIFEST_DESTINATION}" || rollback_status=1
  fi
  [[ "${rollback_status}" == "0" ]]
}

interrupt_installation() {
  local exit_code rollback_failed
  exit_code="$1"
  trap - HUP INT TERM
  rollback_failed=0
  if [[ "${INSTALLATION_IN_PROGRESS:-0}" == "1" ]]; then
    rollback_installation || rollback_failed=1
  fi
  if [[ "${rollback_failed}" == "1" ]]; then
    printf 'error: TKDEGLTF installation was interrupted and rollback was incomplete; the pair manifest must be treated as authoritative\n' >&2
  fi
  exit "${exit_code}"
}

install_archive_pair() {
  local device_staged simulator_staged manifest_staged
  device_staged="$1"
  simulator_staged="$2"
  manifest_staged="$3"

  require_file "${DEVICE_DESTINATION}"
  require_file "${SIMULATOR_DESTINATION}"
  DEVICE_BACKUP="${STAGE_DIRECTORY}/previous-device-libTKDEGLTF.a"
  SIMULATOR_BACKUP="${STAGE_DIRECTORY}/previous-simulator-libTKDEGLTF.a"
  cp "${DEVICE_DESTINATION}" "${DEVICE_BACKUP}"
  cp "${SIMULATOR_DESTINATION}" "${SIMULATOR_BACKUP}"
  MANIFEST_PREEXISTED=0
  MANIFEST_BACKUP="${STAGE_DIRECTORY}/previous-TKDEGLTF_BUILD_MANIFEST.txt"
  if [[ -f "${MANIFEST_DESTINATION}" ]]; then
    cp "${MANIFEST_DESTINATION}" "${MANIFEST_BACKUP}"
    MANIFEST_PREEXISTED=1
  fi

  DEVICE_PENDING="$(mktemp "${TRANSACTION_DIRECTORY}/device.pending.XXXXXX")"
  SIMULATOR_PENDING="$(mktemp "${TRANSACTION_DIRECTORY}/simulator.pending.XXXXXX")"
  MANIFEST_PENDING="$(mktemp "${TRANSACTION_DIRECTORY}/manifest.pending.XXXXXX")"
  cp "${device_staged}" "${DEVICE_PENDING}"
  cp "${simulator_staged}" "${SIMULATOR_PENDING}"
  cp "${manifest_staged}" "${MANIFEST_PENDING}"
  chmod 0644 "${DEVICE_PENDING}" "${SIMULATOR_PENDING}" "${MANIFEST_PENDING}"
  cmp -s "${device_staged}" "${DEVICE_PENDING}" \
    || die "device staging copy verification failed"
  cmp -s "${simulator_staged}" "${SIMULATOR_PENDING}" \
    || die "simulator staging copy verification failed"
  cmp -s "${manifest_staged}" "${MANIFEST_PENDING}" \
    || die "manifest staging copy verification failed"

  note "Atomically replacing the two vendored libTKDEGLTF archives"
  INSTALLATION_IN_PROGRESS=1
  trap 'interrupt_installation 129' HUP
  trap 'interrupt_installation 130' INT
  trap 'interrupt_installation 143' TERM
  if ! mv -f "${DEVICE_PENDING}" "${DEVICE_DESTINATION}"; then
    INSTALLATION_IN_PROGRESS=0
    trap - HUP INT TERM
    die "could not replace device libTKDEGLTF.a"
  fi
  DEVICE_PENDING=""
  if ! mv -f "${SIMULATOR_PENDING}" "${SIMULATOR_DESTINATION}"; then
    if ! rollback_installation; then
      INSTALLATION_IN_PROGRESS=0
      trap - HUP INT TERM
      die "could not replace simulator archive and rollback was incomplete; verify the pair manifest before building"
    fi
    INSTALLATION_IN_PROGRESS=0
    trap - HUP INT TERM
    die "could not replace simulator libTKDEGLTF.a"
  fi
  SIMULATOR_PENDING=""

  # The manifest is the transaction commit record. It is deliberately moved
  # only after both archives, so an uncatchable interruption leaves old hashes
  # that reveal a mixed or incomplete pair instead of falsely blessing it.
  if ! mv -f "${MANIFEST_PENDING}" "${MANIFEST_DESTINATION}"; then
    if ! rollback_installation; then
      INSTALLATION_IN_PROGRESS=0
      trap - HUP INT TERM
      die "could not replace the pair manifest and rollback was incomplete; verify the pair manifest before building"
    fi
    INSTALLATION_IN_PROGRESS=0
    trap - HUP INT TERM
    die "could not replace the TKDEGLTF pair manifest"
  fi
  MANIFEST_PENDING=""

  if ! cmp -s "${device_staged}" "${DEVICE_DESTINATION}" \
    || ! cmp -s "${simulator_staged}" "${SIMULATOR_DESTINATION}" \
    || ! cmp -s "${manifest_staged}" "${MANIFEST_DESTINATION}"; then
    if ! rollback_installation; then
      INSTALLATION_IN_PROGRESS=0
      trap - HUP INT TERM
      die "installed archive verification failed and rollback was incomplete; verify the pair manifest before building"
    fi
    INSTALLATION_IN_PROGRESS=0
    trap - HUP INT TERM
    die "installed archive verification failed"
  fi
  INSTALLATION_IN_PROGRESS=0
  trap - HUP INT TERM
}

cleanup_pending_files() {
  if [[ -n "${DEVICE_PENDING:-}" && -f "${DEVICE_PENDING}" ]]; then
    rm -f "${DEVICE_PENDING}"
  fi
  if [[ -n "${SIMULATOR_PENDING:-}" && -f "${SIMULATOR_PENDING}" ]]; then
    rm -f "${SIMULATOR_PENDING}"
  fi
  if [[ -n "${MANIFEST_PENDING:-}" && -f "${MANIFEST_PENDING}" ]]; then
    rm -f "${MANIFEST_PENDING}"
  fi
}

cleanup_legacy_transaction_files() {
  local candidate destination_directory destination_device
  local candidate_device candidate_owner
  for candidate in \
      "${DEVICE_DESTINATION}.pending."* \
      "${SIMULATOR_DESTINATION}.pending."* \
      "${MANIFEST_DESTINATION}.pending."* \
      "${DEVICE_DESTINATION}.rollback."* \
      "${SIMULATOR_DESTINATION}.rollback."* \
      "${MANIFEST_DESTINATION}.rollback."*; do
    [[ -e "${candidate}" || -L "${candidate}" ]] || continue
    [[ -f "${candidate}" && ! -L "${candidate}" ]] \
      || die "refusing unsafe legacy OCCT transaction path: ${candidate}"
    case "${candidate}" in
      "${DEVICE_DESTINATION}.pending."*|"${DEVICE_DESTINATION}.rollback."*)
        destination_directory="${DEVICE_DESTINATION%/*}"
        ;;
      "${SIMULATOR_DESTINATION}.pending."*|"${SIMULATOR_DESTINATION}.rollback."*)
        destination_directory="${SIMULATOR_DESTINATION%/*}"
        ;;
      "${MANIFEST_DESTINATION}.pending."*|"${MANIFEST_DESTINATION}.rollback."*)
        destination_directory="${MANIFEST_DESTINATION%/*}"
        ;;
      *)
        die "unexpected legacy OCCT transaction path: ${candidate}"
        ;;
    esac
    [[ -d "${destination_directory}" && ! -L "${destination_directory}" ]] \
      || die "legacy OCCT transaction destination is missing or unsafe: ${destination_directory}"
    destination_device="$(stat -f %d "${destination_directory}")"
    candidate_device="$(stat -f %d "${candidate}")"
    candidate_owner="$(stat -f %u "${candidate}")"
    [[ "${candidate_device}" == "${destination_device}" \
        && "${candidate_owner}" == "$(id -u)" ]] \
      || die "refusing unowned legacy OCCT transaction file: ${candidate}"
    rm -f "${candidate}"
    [[ ! -e "${candidate}" && ! -L "${candidate}" ]] \
      || die "could not remove legacy OCCT transaction file: ${candidate}"
  done
}

cleanup_all() {
  local lock_owner
  cleanup_pending_files
  if [[ "${LOCK_HELD:-0}" == "1" ]]; then
    lock_owner=""
    if [[ -f "${LOCK_DIRECTORY}/owner" ]]; then
      lock_owner="$(awk -F= '$1 == "pid" { print $2 }' "${LOCK_DIRECTORY}/owner")"
    fi
    if [[ "${lock_owner}" == "$$" ]]; then
      rm -f "${LOCK_DIRECTORY}/owner"
      rmdir "${LOCK_DIRECTORY}" 2>/dev/null || true
    fi
  fi
}

verify_disk_guard_ancestry() {
  local protocol guard_pid supervisor_pid advertised_pgid
  local worker_pgid supervisor_parent lock_owner guard_command

  protocol="${SHAPEYARD_DISK_GUARD_PROTOCOL:-}"
  guard_pid="${SHAPEYARD_DISK_GUARD_PID:-}"
  supervisor_pid="${SHAPEYARD_DISK_GUARD_SUPERVISOR_PID:-}"
  advertised_pgid="${SHAPEYARD_DISK_GUARD_PGID:-}"
  [[ "${protocol}" == "v1" \
      && "${guard_pid}" =~ ^[0-9]+$ \
      && "${supervisor_pid}" =~ ^[0-9]+$ \
      && "${advertised_pgid}" =~ ^[0-9]+$ ]] \
    || die "the OCCT worker is missing live disk-guard provenance"
  [[ "${supervisor_pid}" == "${advertised_pgid}" \
      && "${PPID}" == "${supervisor_pid}" ]] \
    || die "the OCCT worker is outside the expected guard supervisor"

  worker_pgid="$(/bin/ps -o pgid= -p "$$")"
  worker_pgid="${worker_pgid//[[:space:]]/}"
  supervisor_parent="$(/bin/ps -o ppid= -p "${supervisor_pid}")"
  supervisor_parent="${supervisor_parent//[[:space:]]/}"
  [[ "${worker_pgid}" == "${advertised_pgid}" \
      && "${supervisor_parent}" == "${guard_pid}" ]] \
    || die "the OCCT worker process group or ancestry is not guard-owned"
  kill -0 "${guard_pid}" 2>/dev/null \
    && kill -0 "${supervisor_pid}" 2>/dev/null \
    || die "the OCCT disk guard or supervisor is no longer alive"

  lock_owner="$(sed -n '1p' /private/tmp/.shapeyard-disk-budget-global.lock 2>/dev/null || true)"
  [[ "${lock_owner}" == "${guard_pid}" ]] \
    || die "the live disk-guard lock does not match the OCCT worker ancestry"
  guard_command="$(/bin/ps -o command= -p "${guard_pid}")"
  [[ "${guard_command}" == *"with_disk_budget.sh"* ]] \
    || die "the advertised OCCT guard process is not the canonical disk guard"

  unset SHAPEYARD_DISK_GUARD_PROTOCOL
  unset SHAPEYARD_DISK_GUARD_PID
  unset SHAPEYARD_DISK_GUARD_SUPERVISOR_PID
  unset SHAPEYARD_DISK_GUARD_PGID
}

initialize_guarded_worker() {
  local requested_work_directory=$1
  local automatic_flag=$2
  local worker_token=$3
  local marker marker_contents marker_owner marker_mode marker_device
  local expected_marker_contents
  local parent_device root_device root_owner root_mode
  local directory directory_device directory_owner directory_mode
  local destination_directory

  [[ "${worker_token}" =~ ^[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ \
      && "${OCCT_GLTF_INTERNAL_WORKER_TOKEN:-}" == "${worker_token}" ]] \
    || die "the guarded OCCT worker may only be launched by this script"
  unset OCCT_GLTF_INTERNAL_WORKER_TOKEN
  [[ "${automatic_flag}" == "0" || "${automatic_flag}" == "1" ]] \
    || die "invalid guarded-worker ownership mode"
  [[ "${requested_work_directory}" =~ ^/private/tmp/shapeyard-[A-Za-z0-9._-]+$ \
      && -d "${requested_work_directory}" \
      && ! -L "${requested_work_directory}" ]] \
    || die "guarded worker received an unsafe work directory"

  WORK_DIRECTORY="$(cd "${requested_work_directory}" && pwd -P)"
  [[ "${WORK_DIRECTORY}" == "${requested_work_directory}" ]] \
    || die "guarded worker work directory changed during launch"
  WORK_DIRECTORY_IS_AUTOMATIC="${automatic_flag}"

  parent_device="$(stat -f %d /private/tmp)"
  root_device="$(stat -f %d "${WORK_DIRECTORY}")"
  root_owner="$(stat -f %u "${WORK_DIRECTORY}")"
  root_mode="$(stat -f %Lp "${WORK_DIRECTORY}")"
  [[ "${root_device}" == "${parent_device}" \
      && "${root_owner}" == "$(id -u)" \
      && "${root_mode}" == "700" ]] \
    || die "guarded worker work directory failed owner, mode, or device checks"

  marker="${WORK_DIRECTORY}/.shapeyard-disk-budget-owned"
  [[ -f "${marker}" && ! -L "${marker}" ]] \
    || die "guarded worker ownership marker is missing or unsafe"
  marker_contents="$(cat "${marker}")"
  marker_owner="$(stat -f %u "${marker}")"
  marker_mode="$(stat -f %Lp "${marker}")"
  marker_device="$(stat -f %d "${marker}")"
  expected_marker_contents="$(printf 'schema=shapeyard-disk-budget-owned-v2\nuid=%s\nroot=%s\ntoken=%s' \
    "$(id -u)" "${WORK_DIRECTORY}" "${worker_token}")"
  [[ "${marker_contents}" == "${expected_marker_contents}" \
      && "${marker_owner}" == "$(id -u)" \
      && "${marker_mode}" == "600" \
      && "${marker_device}" == "${parent_device}" ]] \
    || die "guarded worker ownership marker is invalid"

  OCCT_SOURCE_DIRECTORY="${WORK_DIRECTORY}/occt"
  RAPIDJSON_SOURCE_DIRECTORY="${WORK_DIRECTORY}/rapidjson"
  STAGE_DIRECTORY="${WORK_DIRECTORY}/stage"
  VALIDATION_DIRECTORY="${WORK_DIRECTORY}/validation"
  TRANSACTION_DIRECTORY="${WORK_DIRECTORY}/transaction"
  TMP_DIRECTORY="${WORK_DIRECTORY}/tmp"
  for directory in "${STAGE_DIRECTORY}" "${VALIDATION_DIRECTORY}" \
      "${TRANSACTION_DIRECTORY}" "${TMP_DIRECTORY}"; do
    if [[ ! -e "${directory}" ]]; then
      /bin/mkdir -m 700 "${directory}"
    fi
    [[ -d "${directory}" && ! -L "${directory}" ]] \
      || die "guarded worker directory is missing or unsafe: ${directory}"
    directory_device="$(stat -f %d "${directory}")"
    directory_owner="$(stat -f %u "${directory}")"
    directory_mode="$(stat -f %Lp "${directory}")"
    [[ "${directory_device}" == "${root_device}" \
        && "${directory_owner}" == "$(id -u)" \
        && "${directory_mode}" == "700" ]] \
      || die "guarded worker directory failed owner, mode, or device checks: ${directory}"
  done
  for destination_directory in \
      "${DEVICE_DESTINATION%/*}" \
      "${SIMULATOR_DESTINATION%/*}" \
      "${MANIFEST_DESTINATION%/*}"; do
    [[ -d "${destination_directory}" && ! -L "${destination_directory}" \
        && "$(stat -f %d "${destination_directory}")" == "${root_device}" ]] \
      || die "OCCT transaction destination is not on the guarded work-root device"
  done
  TMPDIR="${TMP_DIRECTORY}/"
  export TMPDIR
}

guarded_worker_main() {
  local device_build simulator_build device_archive simulator_archive
  local staged_device staged_simulator staged_manifest
  local requested_work_directory=$1
  local automatic_flag=$2
  local worker_token=$3

  verify_disk_guard_ancestry
  [[ "${OCCT_GLTF_KEEP_WORK_DIR:-0}" == "0" \
      || "${OCCT_GLTF_KEEP_WORK_DIR:-0}" == "1" ]] \
    || die "OCCT_GLTF_KEEP_WORK_DIR must be 0 or 1"

  initialize_guarded_worker \
    "${requested_work_directory}" "${automatic_flag}" "${worker_token}"
  trap cleanup_all EXIT

  require_command awk
  require_command cmake
  require_command cmp
  require_command date
  require_command find
  require_command git
  require_command grep
  require_command hostname
  require_command id
  require_command mktemp
  require_command sed
  require_command shasum
  require_command stat
  require_command xcodebuild
  require_command xcrun
  [[ -x "${DISK_GUARD}" ]] || die "disk guard is not executable: ${DISK_GUARD}"
  check_cmake_version
  xcrun --sdk iphoneos --show-sdk-path >/dev/null
  xcrun --sdk iphonesimulator --show-sdk-path >/dev/null

  require_file "${DEVICE_DESTINATION}"
  require_file "${SIMULATOR_DESTINATION}"
  require_file "${VENDORED_INCLUDE_DIRECTORY}/Standard_Version.hxx"
  require_file "${NOTICES_DIRECTORY}/OpenCASCADE/LICENSE_LGPL_21.txt"
  require_file "${NOTICES_DIRECTORY}/OpenCASCADE/OCCT_LGPL_EXCEPTION.txt"
  require_file "${NOTICES_DIRECTORY}/RapidJSON/license.txt"

  acquire_build_lock
  cleanup_legacy_transaction_files
  if [[ -f "${MANIFEST_DESTINATION}" ]]; then
    "${SCRIPT_DIRECTORY}/verify_occt_gltf_binary_pair.sh"
  fi
  checkout_pinned_source "OCCT" "${OCCT_REPOSITORY}" "${OCCT_COMMIT}" "${OCCT_SOURCE_DIRECTORY}"
  checkout_pinned_source "RapidJSON" "${RAPIDJSON_REPOSITORY}" "${RAPIDJSON_COMMIT}" "${RAPIDJSON_SOURCE_DIRECTORY}"
  validate_pinned_sources

  device_build="${WORK_DIRECTORY}/build-iphoneos"
  simulator_build="${WORK_DIRECTORY}/build-iphonesimulator"
  configure_and_build iphoneos 'arm64;arm64e' "${device_build}"
  device_archive="${BUILT_ARCHIVE}"
  configure_and_build iphonesimulator 'arm64;x86_64' "${simulator_build}"
  simulator_archive="${BUILT_ARCHIVE}"

  staged_device="${STAGE_DIRECTORY}/iphoneos-libTKDEGLTF.a"
  staged_simulator="${STAGE_DIRECTORY}/iphonesimulator-libTKDEGLTF.a"
  cp "${device_archive}" "${staged_device}"
  cp "${simulator_archive}" "${staged_simulator}"
  chmod 0644 "${staged_device}" "${staged_simulator}"

  validate_archive "${staged_device}" iphoneos arm64 arm64e
  validate_archive "${staged_simulator}" iphonesimulator arm64 x86_64
  staged_manifest="${STAGE_DIRECTORY}/TKDEGLTF_BUILD_MANIFEST.txt"
  write_pair_manifest "${staged_manifest}" "${staged_device}" "${staged_simulator}"
  validate_pair_manifest "${staged_manifest}" "${staged_device}" "${staged_simulator}"
  install_archive_pair "${staged_device}" "${staged_simulator}" "${staged_manifest}"
  validate_pair_manifest "${MANIFEST_DESTINATION}" "${DEVICE_DESTINATION}" "${SIMULATOR_DESTINATION}"

  note "Installed validated RapidJSON-enabled archive pair and manifest"
  shasum -a 256 "${DEVICE_DESTINATION}" "${SIMULATOR_DESTINATION}"
  if [[ "${WORK_DIRECTORY_IS_AUTOMATIC}" == "1" \
        && "${OCCT_GLTF_KEEP_WORK_DIR:-0}" != "1" ]]; then
    note "Automatic work directory will be removed on exit: ${WORK_DIRECTORY}"
  else
    note "Build evidence and previous archives remain in ${WORK_DIRECTORY}"
  fi
}

launch_guarded_worker() {
  local worker_token=$1
  local guard_arguments=()

  [[ "${worker_token}" =~ ^[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] \
    || die "could not create a guarded-worker token"
  if [[ "${WORK_DIRECTORY_IS_AUTOMATIC}" == "1" \
        && "${OCCT_GLTF_KEEP_WORK_DIR:-0}" != "1" ]]; then
    guard_arguments+=(--remove-artifact-roots-on-exit)
  fi
  guard_arguments+=(--artifact-root "${WORK_DIRECTORY}" "${worker_token}")

  note "Launching the complete OCCT rebuild and installation under the disk guard"
  export OCCT_GLTF_INTERNAL_WORKER_TOKEN="${worker_token}"
  exec "${DISK_GUARD}" \
    "${guard_arguments[@]}" \
    -- \
    "${SCRIPT_DIRECTORY}/build_occt_gltf_ios.sh" \
    --guarded-worker \
    "${WORK_DIRECTORY}" \
    "${WORK_DIRECTORY_IS_AUTOMATIC}" \
    "${worker_token}"
}

launcher_main() {
  local worker_token
  [[ "$#" == "0" ]] \
    || die "this script accepts no arguments; use OCCT_GLTF_WORK_DIR to choose a work directory"
  [[ "${OCCT_GLTF_KEEP_WORK_DIR:-0}" == "0" \
      || "${OCCT_GLTF_KEEP_WORK_DIR:-0}" == "1" ]] \
    || die "OCCT_GLTF_KEEP_WORK_DIR must be 0 or 1"
  require_command find
  require_command id
  require_command stat
  require_command uuidgen
  [[ -x "${DISK_GUARD}" ]] \
    || die "disk guard is not executable: ${DISK_GUARD}"

  # The disk guard creates automatic roots itself. Nothing is left behind if
  # this launcher fails before exec, and removal is bound to this UUID.
  worker_token="$(uuidgen)"
  [[ "${worker_token}" =~ ^[[:xdigit:]]{8}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{4}-[[:xdigit:]]{12}$ ]] \
    || die "could not create a guarded-worker token"
  prepare_work_directory "${worker_token}"
  launch_guarded_worker "${worker_token}"
}

if [[ "${1:-}" == "--guarded-worker" ]]; then
  shift
  [[ "$#" == "3" ]] || die "invalid guarded-worker invocation"
  guarded_worker_main "$@"
else
  launcher_main "$@"
fi
