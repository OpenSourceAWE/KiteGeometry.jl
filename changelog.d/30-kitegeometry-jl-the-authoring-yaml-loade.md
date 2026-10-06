<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: CC-BY-4.0
-->

### Added

- `load_authoring(path; set)` reads SymbolicAWEModels' authoring YAML, with `set`'s KiteUtils settings for what a row leaves out, and returns the placed `SystemDefinition`: tethers at their stretched lengths and `transforms` applied, so `pos_ENU` and `Q_KA_to_ENU` hold the initial pose.
- `SegmentSpring`, the segment model `spring`: a segment's `unit_damping`, `compression_frac` and `compression_damping_frac`.
