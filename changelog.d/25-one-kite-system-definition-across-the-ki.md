<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: CC-BY-4.0
-->

### Added

- `SystemDefinition` and its components `Point`, `Segment`, `Station`, `Pulley`,
  `Tether`, `Winch`, `Wing`, `CanopyFace`, `Body` and `Tube`, generated from the awesIO
  structure schema 1.0.0; `load_structure` reads a structure document and
  `structure_document` writes one back, units and extra blocks included.
- Every component `T{M}` carries a `model::M` whose fields it forwards;
  `register_model!` maps a row's `model` column to `M`, built from its extra columns in
  the units it was registered with; reading refuses any other filled extra column, or
  with `strict=false` drops it with a warning.
- Julia 1.13 is supported.
