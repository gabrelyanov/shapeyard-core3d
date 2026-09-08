MikkTSpace by Morten S. Mikkelsen

Upstream: https://github.com/mmikk/MikkTSpace
Pinned commit: 3e895b49d05ea07e4c2133156cfa94369e19e409 (25 March 2020).
Vendored mikktspace.c and mikktspace.h are unmodified. License retained in both source files and LICENSE.txt.

SHA-256:
- mikktspace.c: de87e74107df766ce68108801262bd8d53899414236b59810509a8fc2a51e288
- mikktspace.h: 17fc433894f24c73753d548086cc4d8c5c0379f4a6edfb98b5da243e4f0bc3d0

Shapeyard adapter: Scene/MikkTangentSpace.cpp. Triangle-only, bounded, returns original index-corner order. Degenerate geometry/UVs are explicitly rejected. Native renderer, Metal and export integration are separately qualified.
