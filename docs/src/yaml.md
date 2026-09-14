<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# The YAML file

[`load_definition`](@ref) reads a file of tables, one per component kind. Every
block is optional and every row is the arguments of that component's
constructor, so the reference for a column is the constructor's docstring.

A table is written either as `headers` plus rows of values:

```yaml
segments:
  headers: [name, point_i, point_j, l0, diameter_mm, youngs_modulus,
            damping_per_stiffness, density]
  data:
    - [tether_1, ground, p1, 5.0, 4.0, 55.0e9, 0.00077, 724.0]
```

or as a list of mappings, which is the readable form for a block with many
optional columns:

```yaml
transforms:
  data:
    - name: main_transform
      elevation: 50
      azimuth: 0.0
      heading: 0.0
      wing_idx: main_wing
      base_pos: [0.0, 0.0, 0.0]
      base_point_idx: ground
```

A row may stop short of the last column, and a cell written `nothing` is one
row's way of leaving a column the other rows fill empty.

## The blocks

| Block | Builds | Required columns |
| --- | --- | --- |
| `points` | [`Point`](@ref) | `name`, `pos_cad`, `type` |
| `segments` | [`Segment`](@ref) | `name`, `point_i`, `point_j` |
| `pulleys` | [`Pulley`](@ref) | `name`, `segment_i`, `segment_j`, `type` |
| `twist_surfaces` | [`TwistSurface`](@ref) | `name`, `points`, `type` |
| `tethers` | [`Tether`](@ref) | `name`, and either `segment_idxs` or `start_point`/`end_point`/`n_segments` |
| `winches` | [`Winch`](@ref) | `name`, `tether_idxs`, `winch_point` |
| `wings` | [`Wing`](@ref) | `name`, `dynamics_type` |
| `bodies` | [`Body`](@ref) | `name`, `mass`, `pos`, and `inertia` or `inertia_principal` |
| `elastic_joints` | [`Joint`](@ref) with an [`ElasticJoint`](@ref) | `name`, `body_a`, `body_b`, the four stiffnesses |
| `timoshenko_joints` | [`Joint`](@ref) with a [`TimoshenkoBeam`](@ref) | `name`, `body_a`, `body_b`, `EA`, `GA`, `GJ`, `EIy`, `EIz` |
| `transforms` | [`Transform`](@ref) | `name`, `elevation`, `azimuth`, `heading` |

A component is referenced by its `name`, or by its 1-based position in its own
block. A row without a `name` can only be referenced by position.

## Material

A segment's elasticity is written either per element — `unit_stiffness` [N] and
`unit_damping` [N·s], which scale with the cross section — or as a material,
`youngs_modulus` [Pa] and `damping_per_stiffness` [s], which does not, so the
same material can be given to elements of different diameter. Writing both
forms of one quantity is an error. Anything left out comes from the settings.

## Variables

A top-level `variables` block names values used more than once. A variable
holding a number, string or list replaces any cell written as its name; one
holding a mapping fills the columns it names at once, wherever it appears.

```yaml
variables:
  bridle_comp: 0.01
  dyneema: {youngs_modulus: 55.0e9, damping_per_stiffness: 0.00077,
            density: 724.0}

segments:
  headers: [name, point_i, point_j, diameter_mm, youngs_modulus,
            damping_per_stiffness, density, compression_frac]
  data:
    - [bridle_1, kcu, le_left, 1.0, dyneema, bridle_comp]
```

A variable may name another variable. A variable sharing a name with a
component is an error, since references to that component would resolve to the
variable instead.
