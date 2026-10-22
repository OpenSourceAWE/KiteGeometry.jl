<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# Changelog

## Unreleased

### Changed

- BREAKING: KiteGeometry no longer has a `calc_heading(R_b_to_w, pos)` of its own; use KiteUtils' `calc_heading(attitude, elevation, azimuth)`. `using KiteUtils, KiteGeometry` no longer leaves `calc_heading` ambiguous.

## KiteGeometry v0.3.2 2026-10-21

### Added

- A block's rows are `Rows`, found by their name as well as their position: `system.points[:kite]` and `system.points["kite"]`. `load_authoring` returns its transforms as `Rows` too.
- `load_authoring(authored::AbstractDict; set, name)` loads a parsed authoring document, so a tool can build one in code rather than write a file.
- `load_authoring(...; place=false)` leaves the definition in its design pose, for a tool that places it itself.

### Changed

- A row's `name`, a `Transform`'s `name` and a reference by name (`NameRef`) are `Symbol`s, not `String`s: `system.points[1].name == :kite`. Documents and keyword constructors still take names as strings, and writing a document writes them as strings.

## KiteGeometry v0.3.1 2026-10-19

### Added

- `Metadata(; name, n_points)` builds metadata by keyword, with the other fields defaulting to an empty description and note, the awesIO version this package reads, the structure schema, and an empty `connectivity_sha`.

## KiteGeometry v0.3.0 2026-10-18

### Added

- `Transform`, kept beside a `SystemDefinition`: the points and bodies it moves, its base, and its elevation, azimuth and heading with their rates. A transform can be chained on another. `place!(system, transforms)` applies changed transforms to a definition, and `reposition!` and `transform_velocities` act on a `Pose`, the positions and body frames of a run.
- Rows are mutable: `point.extra_mass = 2.5` and `point.pos_ENU .= [1, 2, 3]` change a row in place, and a field of its model is set through the row (`tube.EA = 3e5`). Vector and matrix fields are `MVector`/`MMatrix` rather than `SVector`/`SMatrix`, so reading one gives the row's own array and not a copy. Rows and models compare and hash by their fields.
- `model_columns(M)` names the fields of a model type that are document columns, all of them unless a method says fewer; a model may keep fields of its own that are neither read nor written.
- `load_authoring` keeps every block it does not read, such as a plate wing's `surfaces`, as an extra block instead of refusing it. A wing's `dynamics_type`, `origin_idx`, `z_ref_points` and `y_ref_points` reach a registered wing model that has fields of those names.

### Changed

- BREAKING: `load_authoring` refuses with `error(...)` (an `ErrorException`) naming the offending row instead of an `ArgumentError`, refuses a station's point that names no wing or another wing, and names the point whose wing does not exist.
- BREAKING: `load_authoring` returns the definition and its transforms, and places the system by its transforms' elevation, azimuth and heading, as SymbolicAWEModels does, rather than at zenith. It reads chained transforms (`base_transform_idx`), and the transforms are no longer kept as an extra block of the definition.

### Fixed

- `load_authoring` reads `0` in an optional reference column (`wing_idx`, `transform_idx`, `body_idx`) as none, as SymbolicAWEModels writes it.
- A model field typed `Any` no longer overflows the stack when it is filled.
- `load_authoring` reads an authoring YAML in file order, so a cyclic variable reference is reported from its first variable in the file on every Julia version.
- The points `load_authoring` adds along a tether and the body it adds for each wing carry their block's default model, where that model needs no column filled, so the definition reads back from its own YAML with the same models.

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
