# Third-party provenance

Core3D vendors Open CASCADE Technology (OCCT) headers and static libraries.
The RapidJSON headers are used only to rebuild OCCT's `TKDEGLTF` archive with
glTF parsing enabled.

| Component | Version | Pinned upstream commit | Upstream source | Included notices |
| --- | --- | --- | --- | --- |
| Open CASCADE Technology | 7.8.0 (`V7_8_0`) | `656b0d217fcc3f6611dfabc0206bd2d967ed5265` | <https://github.com/Open-Cascade-SAS/OCCT> | `OpenCASCADE/LICENSE_LGPL_21.txt`, `OpenCASCADE/OCCT_LGPL_EXCEPTION.txt` |
| RapidJSON | Post-1.1.0 upstream snapshot | `24b5e7a8b27f42fa16b96fc70aade9106cf7102f` | <https://github.com/Tencent/rapidjson> | `RapidJSON/license.txt` |

The notice files are byte-for-byte copies from those commits. Keep them with
distributed binaries and review the upstream terms when packaging releases.
RapidJSON is header-only; the build uses its `include/rapidjson` tree and does
not use `bin/jsonchecker`.

## Distribution status

Including these notices in `Core3D.framework` is an engineering prerequisite,
not distribution clearance. A Shapeyard production archive must remain blocked
until either a commercial OCCT license is confirmed or a reviewed LGPL 2.1 plus
OCCT-exception path supplies the corresponding source and modifiable/relinkable
materials required for the chosen distribution channel. An in-app notices view
is also required before release.

The RapidJSON pin is an exact upstream-master snapshot. It includes the 2018
exponent-underflow correction referenced by CVE-2024-38517 and later parser
hardening; the build script verifies that correction before compiling. The
unmerged patch proposed in pull request 2357 for CVE-2024-39684 is deliberately
not treated as an iOS fix: its `unsigned` to `uint32_t` change is a semantic
no-op on this platform. Shapeyard enables its deliberately limited GLB import
subset only after a descriptor-pinned, bounded preflight validates the complete
container, JSON structure, binary geometry, embedded images, scene graph, and
transform ranges. Hostile-input regression tests cover malformed and
unsupported content; extensions, external resources, sparse accessors, skins,
animations, cameras, morph targets, normal/occlusion/metallic-roughness maps,
legacy material techniques/values, and non-similarity transforms fail closed.
This engineering gate does not change the separate distribution-license block
above.

## Rebuilding `TKDEGLTF` for iOS

Run `Scripts/build_occt_gltf_ios.sh` from any directory. Prerequisites are:

- macOS with full Xcode selected, its license accepted, and the iPhoneOS and
  iPhoneSimulator SDKs installed;
- CMake 3.21 or newer, Git, HTTPS access to the two official repositories, and
  enough free space for two clean Xcode builds;
- the repository's complete `Core3D/occt/inc` tree from OCCT 7.8.0 and the two
  existing destination archives.

The script checks out the exact commits above into a fresh work directory,
configures all OCCT modules off with only `BUILD_ADDITIONAL_TOOLKITS=TKDEGLTF`,
enables RapidJSON, and asks Xcode to compile only the `TKDEGLTF` target with one
build job. In this narrow configuration OCCT does not populate all transitive
headers or build companion toolkit targets, so the script explicitly adds the
repository's complete pinned OCCT include tree. It verifies critical headers
against the checkout before compiling; it does not rebuild or replace companion
OCCT libraries.

The outputs target iOS 16.0 and contain exactly these slices:

- `Core3D/occt/lib/libTKDEGLTF.a`: iPhoneOS `arm64`, `arm64e`
- `Core3D/occt/lib_sim/libTKDEGLTF.a`: iPhoneSimulator `arm64`, `x86_64`

Before replacing either archive, the script validates both source-object sets,
architecture sets, RapidJSON-enabled parser strings, absence of OCCT's
RapidJSON-disabled fallback, and absence of unresolved RapidJSON symbols. It
also canonicalizes `__FILE__` paths and rejects archives that expose the random
work directory or local repository path, making outputs reproducible across
checkout locations.

Only one rebuild may run in a checkout at a time. The script owns the atomic
directory lock `.occt-gltf-build.lock` for the complete operation and fails
closed if a previous uncatchable interruption left that lock behind. Inspect
the recorded PID and confirm that no rebuild is active before manually removing
a stale lock.

The script uses same-directory atomic renames and rolls back the pair if any
handled replacement or validation step fails. It writes
`Core3D/occt/TKDEGLTF_BUILD_MANIFEST.txt` last, after both archives. That
manifest records the exact source commits, build tools, SDKs, platforms,
architectures, deployment target, and SHA-256 hash of each installed archive.
An uncatchable interruption between archive renames therefore leaves a stale
manifest whose hashes reveal the incomplete pair rather than blessing it. No
other vendored archive is staged or replaced.

Set `OCCT_GLTF_WORK_DIR` to an empty directory to choose where source, build,
validation, staged output, and rollback evidence are retained. Otherwise a new
directory is created below `$TMPDIR`; it is intentionally retained and printed
at the end of the run.
