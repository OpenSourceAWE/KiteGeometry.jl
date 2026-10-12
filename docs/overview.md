<!--
SPDX-FileCopyrightText: 2026 Uwe Fechner
SPDX-License-Identifier: MIT
-->

# Code structure of KiteGeometry.jl

KiteGeometry.jl is a small package of about 2,700 lines. Its job is to hold an
airborne-wind-energy system (the kite, tethers, winches and bridle) as one data type,
`SystemDefinition`, and to read and write it in the **awesIO** structure format, as YAML or
JSON.

## How the parts fit

```text
awesIO schema (YAML) ──build.jl──▶ _structure.jl   (generated types)
                                        │
           models.jl  ─────────────────┤ (extra per-row parameters)
           components.jl ──────────────┤ (constructors, defaults)
                                        ▼
   YAML/JSON ◀──document.jl──▶  SystemDefinition  ◀──authoring.jl + placement.jl── SymbolicAWEModels authoring YAML
```

## Source files

The include order in [src/KiteGeometry.jl](../src/KiteGeometry.jl) matters, because each
file builds on the ones before it.

| File | Role |
|---|---|
| [KiteGeometry.jl](../src/KiteGeometry.jl) | The module itself. It defines `NameRef` (a reference to another row, by name or by index) and the abstract type `Component`. Its `__init__` registers the `"spring"` model for segments. |
| [models.jl](../src/models.jl) | A plug-in registry that lets tools attach extra columns to a block's rows. Each component is `T{M}`, where `M <: AbstractModel` comes from the row's `model` column (`NoModel` if there is none). `register_model!` adds models; `SegmentSpring` is the built-in one. `getproperty` is overloaded so model fields read like ordinary fields. |
| [_structure.jl](../src/_structure.jl) | **Generated, do not edit.** It holds one struct per schema block (`Point`, `Segment`, `Station`, `Pulley`, `Tether`, `Winch`, `Wing`, `CanopyFace`, `Body`, `Tube`), plus `Metadata`, `DynamicsType`, `SystemDefinition` and the `BLOCKS`/`UNITS` tables. Units are written into the docstrings. |
| [build.jl](../src/build.jl) | The generator for `_structure.jl`. It reads the vendored schema [src/awesio/structure_schema.yml](../src/awesio/structure_schema.yml) and the pinned awesIO commit recorded in [gen/awesio.toml](../gen/awesio.toml). Run it with `julia --project src/build.jl`. |
| [components.jl](../src/components.jl) | Keyword constructors `T(; kwargs...)` for all components, with per-type `defaults` (for example `type=DYNAMIC` and `efficiency=1.0`). `to_field` converts document values (strings, nested vectors) into field types (enums, `SVector`, `SMatrix`). |
| [document.jl](../src/document.jl) | Converts between a `SystemDefinition` and a document. `SystemDefinition(dict; strict)` resolves name references to indices and checks the awesIO version. `structure_document` goes the other way. On top of these sit `from_yaml`/`to_yaml`, `from_json`/`to_json` and `load_structure`, plus `connectivity_sha`, a hash of the connectivity that catches state replayed against a structure with different indexing. Blocks the schema doesn't name are kept in `extras`. |
| [authoring.jl](../src/authoring.jl) (the largest file) | `load_authoring` reads SymbolicAWEModels' **authoring** YAML dialect: points given as `pos_cad`, tethers given by segment count and length, `variables` and multi-variables, and `transforms`. It fills in missing material, gear ratio and drum values from KiteUtils `Settings`, and computes wing mass and inertia. It refuses cells and blocks that nothing reads. |
| [placement.jl](../src/placement.jl) | The geometry `load_authoring` uses to place the design-frame system in the world. `place!` applies the elevation/azimuth/heading transforms, sets segment rest lengths and the initial stretch of tethers and pulleys, moves rigid bodies with their attached points, and fits wing frames. It also contains the quaternion and rotation helpers. |

## Tests and data

- [test/runtests.jl](../test/runtests.jl) automatically includes every `test-*.jl` file as
  its own testset:
  - [test-structure.jl](../test/test-structure.jl) covers reading and writing awesIO
    documents.
  - [test-authoring.jl](../test/test-authoring.jl) covers `load_authoring`.
  - [test/models.jl](../test/models.jl) is a helper, not a testset.
- `test/data/` holds the fixtures:
  - full-form awesIO documents: `v3_beam_structure.yml` and `v3_psm_structure.yml`
    (vendored from awesIO), plus `2plate_kite_structure.yml`;
  - authoring-format inputs: the `2plate_kite/` and `v3_psm/` folders.

## Everything else

- `docs/` is a Documenter site; [docs/src/structure.md](src/structure.md) is the API page.
- `bin/` holds helper scripts for installing, releasing, REUSE licence linting and updating
  the default manifests.
- The rest comes from a copier template: the CI workflows in `.github/`, the REUSE licence
  headers, and the `Manifest-v1.1x.toml.default` files.

## Design principle

The awesIO schema is the single source of truth: the component types are generated from it,
so a schema change means updating `gen/awesio.toml` and the vendored schema, then re-running
`build.jl`.
