# OpenCASCADE simulator libraries

These universal archives contain `arm64` and `x86_64` iOS Simulator slices for
the vendored OpenCASCADE 7.8.0 headers and device libraries in the adjacent
`inc` and `lib` directories.

They are versioned with Core3D so a clean Apple-silicon checkout can build and
run the app's simulator and UI-test targets without relying on an untracked
developer-machine directory. Keep this directory, `inc`, and `lib` on the same
OpenCASCADE version and build configuration.

Verified with Xcode 26.2 using the `ShapeyardDev` scheme and a generic iOS
Simulator destination.
