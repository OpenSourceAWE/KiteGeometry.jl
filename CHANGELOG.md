<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# Changelog

## Unreleased

### Added

- `model_columns(M)` names the fields of a model type that are document columns, all of them unless a method says fewer; a model may keep fields of its own that are neither read nor written.

### Changed

- BREAKING: Rows are mutable: `point.extra_mass = 2.5` and `point.pos_ENU .= [1, 2, 3]` change a row in place, and a field of its model is set through the row (`tube.EA = 3e5`). Vector and matrix fields are `MVector`/`MMatrix`. Rows and models compare and hash by their fields.

## KiteGeometry v0.2.0 2026-10-16

### Added

- The `materials` block and its `Material` rows; segments, tubes and a wing's `canopy_material` name one.
- A wing's `pos_ENU` and `Q_KA2ENU`, the origin and rotation of its KA frame, and `pos_KA(wing, pos_ENU)`, which puts a world position into that frame.
- Optional awesIO columns are fields that are `nothing` where a document leaves them out, and are written only where a row fills them.

### Changed

- BREAKING: KiteGeometry follows awesIO 0.1.0 (1-Bart-1/awesIO@99a36e2) and refuses awesIO 1.0.0 documents. A point is `name`, `fixed`, `body`, `pos_ENU`; a segment `name`, `points`; `l0` is the optional `rest_length`, `Station` is `SpanwiseStation` in the `spanwise_stations` block, and a body's `Q_KA_to_ENU` is `Q_KA2ENU`. `DynamicsType`, point drag, a segment's density and `unit_stiffness`, a body's and a station's type, and a pulley's type and efficiency are no longer fields; the last four are columns of the models a tool registers.
- BREAKING: `load_authoring` writes the kite at zenith: transforms only shift their members so the base point is at its base position, each wing's KA frame is the authoring frame so shifted, and the `transforms` block is kept as an extra block. Segment stiffness and density become `linear` materials, and a tube's `breukels2011` law a material with SymbolicAWEModels' constants.
- The writer leaves out the `model` column of a table whose rows all read their block's default model, and a model column no row fills. A model column a document leaves out reads as unset where its field can be `nothing`.

### Fixed

- A model field typed `Union{Nothing, Float64, Vector{Float64}}` reads a cell holding one number or a vector; both were refused with a `MethodError`.

## KiteGeometry v0.1.1 2026-10-15

### Added

- `save_log(logger, system, name)` saves a KiteUtils log with the `SystemDefinition` `system` in its metadata under `topology`, and `KiteGeometry.definition(log)` reads it back from a log `load_log` loaded.

## KiteGeometry v0.1.0 09-10-2026

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
