#!/bin/bash

# Fail every Core3D build when the two vendored TKDEGLTF archives do not match
# the pair manifest written last by build_occt_gltf_ios.sh.

set -euo pipefail

readonly EXPECTED_OCCT_COMMIT="656b0d217fcc3f6611dfabc0206bd2d967ed5265"
readonly EXPECTED_RAPIDJSON_COMMIT="24b5e7a8b27f42fa16b96fc70aade9106cf7102f"
readonly EXPECTED_DEPLOYMENT_TARGET="16.0"
readonly RAPIDJSON_FALLBACK_TEXT="OCCT has been built without RapidJSON support"
readonly RAPIDJSON_ENABLED_TEXT="Invalid glTF syntax"

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIRECTORY}/.." && pwd -P)"
MANIFEST="${REPOSITORY_ROOT}/Core3D/occt/TKDEGLTF_BUILD_MANIFEST.txt"
DEVICE_ARCHIVE="${REPOSITORY_ROOT}/Core3D/occt/lib/libTKDEGLTF.a"
SIMULATOR_ARCHIVE="${REPOSITORY_ROOT}/Core3D/occt/lib_sim/libTKDEGLTF.a"

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  if [[ -n "${TEMP_DIRECTORY:-}" && -d "${TEMP_DIRECTORY}" ]]; then
    rm -rf -- "${TEMP_DIRECTORY}"
  fi
}

require_file() {
  [[ -f "$1" ]] || die "required TKDEGLTF integrity file not found: $1"
}

manifest_value() {
  local key count
  key="$1"
  count="$(awk -F= -v key="${key}" '$1 == key { count += 1 } END { print count + 0 }' "${MANIFEST}")"
  [[ "${count}" == "1" ]] || die "manifest must contain exactly one ${key} entry"
  awk -F= -v key="${key}" '$1 == key { print substr($0, length($1) + 2) }' "${MANIFEST}"
}

verify_architectures() {
  local archive label expected_count architecture actual_count
  archive="$1"
  label="$2"
  shift 2
  expected_count="$#"
  actual_count="$(xcrun lipo -archs "${archive}" | wc -w | tr -d '[:space:]')"
  [[ "${actual_count}" == "${expected_count}" ]] \
    || die "${label} archive has an unexpected architecture count"
  for architecture in "$@"; do
    xcrun lipo "${archive}" -verify_arch "${architecture}" >/dev/null \
      || die "${label} archive is missing ${architecture}"
  done
}

verify_slice_objects() {
  local archive label architecture expected_platform thin_archive objects_directory
  local expected_objects actual_objects member build_report strings_report
  archive="$1"
  label="$2"
  architecture="$3"
  expected_platform="$4"
  thin_archive="${TEMP_DIRECTORY}/${label}-${architecture}.a"
  objects_directory="${TEMP_DIRECTORY}/${label}-${architecture}-objects"
  expected_objects="${TEMP_DIRECTORY}/${label}-${architecture}.expected"
  actual_objects="${TEMP_DIRECTORY}/${label}-${architecture}.actual"
  strings_report="${TEMP_DIRECTORY}/${label}-${architecture}.strings"

  xcrun lipo "${archive}" -thin "${architecture}" -output "${thin_archive}"
  mkdir -p "${objects_directory}"
  (cd "${objects_directory}" && xcrun ar -x "${thin_archive}")
  printf '%s\n' \
    RWGltf_CafReader.o \
    RWGltf_CafWriter.o \
    RWGltf_ConfigurationNode.o \
    RWGltf_GltfJsonParser.o \
    RWGltf_GltfLatePrimitiveArray.o \
    RWGltf_GltfMaterialMap.o \
    RWGltf_Provider.o \
    RWGltf_TriangulationReader.o \
    | LC_ALL=C sort > "${expected_objects}"
  find "${objects_directory}" -maxdepth 1 -type f -name '*.o' -exec basename {} \; \
    | LC_ALL=C sort > "${actual_objects}"
  cmp -s "${expected_objects}" "${actual_objects}" \
    || die "${label}/${architecture} has an unexpected TKDEGLTF object set"

  while IFS= read -r member; do
    build_report="${objects_directory}/${member}.build-version"
    xcrun vtool -show-build "${objects_directory}/${member}" > "${build_report}"
    grep -Eq "^[[:space:]]*platform ${expected_platform}$" "${build_report}" \
      || die "${label}/${architecture}/${member} has the wrong Apple platform"
    grep -Eq "^[[:space:]]*minos ${EXPECTED_DEPLOYMENT_TARGET}$" "${build_report}" \
      || die "${label}/${architecture}/${member} has the wrong deployment target"
  done < "${actual_objects}"

  xcrun strings "${thin_archive}" > "${strings_report}"
  grep -Fq "${RAPIDJSON_ENABLED_TEXT}" "${strings_report}" \
    || die "${label}/${architecture} is missing RapidJSON-enabled parser code"
  if grep -Fq "${RAPIDJSON_FALLBACK_TEXT}" "${strings_report}"; then
    die "${label}/${architecture} contains OCCT's RapidJSON-disabled fallback"
  fi
  grep -Fq '/shapeyard/occt-gltf-build/occt/src/RWGltf/' "${strings_report}" \
    || die "${label}/${architecture} is missing canonical source provenance"
}

main() {
  local expected_device_hash expected_simulator_hash actual_device_hash actual_simulator_hash key value
  [[ "$#" == "0" ]] || die "this verifier accepts no arguments"
  command -v awk >/dev/null 2>&1 || die "awk is required"
  command -v cmp >/dev/null 2>&1 || die "cmp is required"
  command -v find >/dev/null 2>&1 || die "find is required"
  command -v shasum >/dev/null 2>&1 || die "shasum is required"
  command -v xcrun >/dev/null 2>&1 || die "xcrun is required"
  require_file "${MANIFEST}"
  require_file "${DEVICE_ARCHIVE}"
  require_file "${SIMULATOR_ARCHIVE}"
  TEMP_DIRECTORY="$(mktemp -d "${TMPDIR:-/tmp}/shapeyard-tkdegltf-verify.XXXXXX")"
  trap cleanup EXIT

  [[ "$(manifest_value schema_version)" == "1" ]] || die "unsupported pair manifest schema"
  [[ "$(manifest_value component)" == "OCCT_TKDEGLTF" ]] || die "unexpected pair manifest component"
  [[ "$(manifest_value occt_commit)" == "${EXPECTED_OCCT_COMMIT}" ]] \
    || die "unexpected OCCT commit in pair manifest"
  [[ "$(manifest_value rapidjson_commit)" == "${EXPECTED_RAPIDJSON_COMMIT}" ]] \
    || die "unexpected RapidJSON commit in pair manifest"
  for key in xcode_version xcode_build cmake_version iphoneos_sdk_version \
    iphoneos_sdk_build iphonesimulator_sdk_version \
    iphonesimulator_sdk_build; do
    value="$(manifest_value "${key}")"
    [[ -n "${value}" ]] || die "manifest entry ${key} must not be empty"
  done
  [[ "$(manifest_value deployment_target)" == "${EXPECTED_DEPLOYMENT_TARGET}" ]] \
    || die "unexpected deployment target in pair manifest"
  [[ "$(manifest_value device_platform)" == "IOS" ]] \
    || die "unexpected device platform in pair manifest"
  [[ "$(manifest_value device_architectures)" == "arm64,arm64e" ]] \
    || die "unexpected device architectures in pair manifest"
  [[ "$(manifest_value device_archive)" == "Core3D/occt/lib/libTKDEGLTF.a" ]] \
    || die "unexpected device archive path in pair manifest"
  [[ "$(manifest_value simulator_platform)" == "IOSSIMULATOR" ]] \
    || die "unexpected simulator platform in pair manifest"
  [[ "$(manifest_value simulator_architectures)" == "arm64,x86_64" ]] \
    || die "unexpected simulator architectures in pair manifest"
  [[ "$(manifest_value simulator_archive)" == "Core3D/occt/lib_sim/libTKDEGLTF.a" ]] \
    || die "unexpected simulator archive path in pair manifest"
  [[ "$(manifest_value canonical_source_prefix)" == "/shapeyard/occt-gltf-build" ]] \
    || die "unexpected canonical source prefix in pair manifest"

  expected_device_hash="$(manifest_value device_sha256)"
  expected_simulator_hash="$(manifest_value simulator_sha256)"
  actual_device_hash="$(shasum -a 256 "${DEVICE_ARCHIVE}" | awk '{ print $1 }')"
  actual_simulator_hash="$(shasum -a 256 "${SIMULATOR_ARCHIVE}" | awk '{ print $1 }')"
  [[ "${actual_device_hash}" == "${expected_device_hash}" ]] \
    || die "device TKDEGLTF archive does not match the pair manifest"
  [[ "${actual_simulator_hash}" == "${expected_simulator_hash}" ]] \
    || die "simulator TKDEGLTF archive does not match the pair manifest"

  verify_architectures "${DEVICE_ARCHIVE}" device arm64 arm64e
  verify_architectures "${SIMULATOR_ARCHIVE}" simulator arm64 x86_64
  verify_slice_objects "${DEVICE_ARCHIVE}" device arm64 IOS
  verify_slice_objects "${DEVICE_ARCHIVE}" device arm64e IOS
  verify_slice_objects "${SIMULATOR_ARCHIVE}" simulator arm64 IOSSIMULATOR
  verify_slice_objects "${SIMULATOR_ARCHIVE}" simulator x86_64 IOSSIMULATOR
  printf 'Verified TKDEGLTF archive pair: %s / %s\n' "${actual_device_hash}" "${actual_simulator_hash}"
}

main "$@"
