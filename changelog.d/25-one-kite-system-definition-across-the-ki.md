<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: CC-BY-4.0
-->

### Added

- `SystemDefinition` and its components `Point`, `Segment`, `Station`, `Pulley`,
  `Tether`, `Winch`, `Body` and `Tube{M}`, generated from the awesIO structure schema
  1.0.0; `load_structure` reads a structure document and `structure_document` writes one
  back, extra columns and blocks included. `register_tube_model!` maps a tube's `model`
  column to `M`.
- Julia 1.13 is supported.
