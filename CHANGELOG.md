<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# Changelog

## KiteGeometry v0.1.0 2026-10-09

### Added

- `SystemDefinition` and its components `Point`, `Segment`, `Station`, `Pulley`,
  `Tether`, `Winch`, `Wing`, `CanopyFace`, `Body` and `Tube`, generated from the
  awesIO structure schema 1.0.0; `load_structure` reads a structure document and
  `structure_document` writes one back, units and extra blocks included.
- `to_yaml`/`from_yaml` and `to_json`/`from_json` write and read a structure
  document as text, and `KiteGeometry.definition` reads the `topology` string a
  log carries. Reading refuses a document whose `connectivity_sha` does not
  describe its own tables or whose `awesIO_version` is another major version,
  and warns on another minor version.
- `connectivity_sha` hashes the schema's connectivity preimage;
  `structure_document` writes the system's own.
- Every component `T{M}` carries a `model::M` whose fields it forwards;
  `register_model!(block, name, M, units; default)` maps a row's `model` column
  to `M`, built from its extra columns in the units it was registered with, and
  with `default=true` makes `M` the model of rows without a `model` column.
  Reading refuses any other filled extra column, or with `strict=false` drops it
  with a warning.
- `SegmentSpring`, the segment model `spring`: a segment's `unit_damping`,
  `compression_frac` and `compression_damping_frac`.
- `load_authoring(path; set)` reads the authoring YAML, `canopy_faces` included,
  with `set`'s KiteUtils settings for what a row leaves out, and returns the
  placed `SystemDefinition`: tethers at their stretched lengths and `transforms`
  applied. It refuses a filled cell or block nothing reads.
- Julia 1.12 and 1.13 are supported.
