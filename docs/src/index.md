<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# KiteGeometry.jl

```@docs
KiteGeometry
```

One system definition for airborne wind energy systems — topology, CAD geometry
and material — shared by the OpenSourceAWE packages.

[`SystemDefinition`](@ref) holds what a kite system *is*: its points, the
spring-damper segments between them, the sheaves, tethers and winches, the
rigid bodies and the elastic joints between those, the chordwise surfaces a
wing twists about, and the transforms that place the whole thing in the world.
Nothing in it is written per simulation step, so the same definition can be
handed to the package that integrates the system and to the one that draws it.

The field sets are SymbolicAWEModels' authoring columns, written by hand. The
awesIO structure schema is the format they are to follow; where the two differ
today, the schema is right.

## Reading a system

```julia
using KiteGeometry, KiteUtils

set_data_path("data/2plate_kite")
definition = load_definition("data/2plate_kite/particle_structural_geometry.yaml";
                             set=Settings("system.yaml"))

definition.points[:kcu].pos_cad          # by name
definition.segments[1].l0                # or by position
definition.wings[:main_wing].twist_surface_idxs
```

[`load_definition`](@ref) reads the authoring YAML dialect described under
[The YAML file](@ref); every reference in the file is resolved to an index
while it is read, and a tether written as a start point, an end point and a
segment count has its intermediate points and segments generated.

## Frames

Three frames appear, and a name says which one it means.

- **CAD** is the design frame the file is written in, suffixed `_cad`.
  `pos_cad` is a design position, never a place in the world — do not draw it
  as one.
- **KA** is a rigid body's own frame, suffixed `_KA`. A rotation reads
  `R_<from>_to_<to>`, as in a body's `R_KA_to_CAD`.
- **ENU** is the world: east, north, up, suffixed `_ENU`. A
  [`Transform`](@ref) maps CAD to ENU at a chosen elevation, azimuth and
  heading.

Angles in a file are in degrees and diameters in millimetres; angles in the
structs are in radians and every length is in metres.

## Where the physics goes

A definition carries the parameters of a model, never the model. A segment
holds a [`LinearSpring`](@ref) or a [`NonlinearSpring`](@ref); a joint holds
an [`ElasticJoint`](@ref) or a [`TimoshenkoBeam`](@ref); a winch holds a
[`TorqueWinch`](@ref). What each of them means as an equation belongs to the
package that assembles and solves it, which is free to add its own model types
under the same supertypes.

Each of those models is a type parameter of the component that holds it, so a
system whose segments all share one force law is concretely typed all the way
down, and reading a segment's stiffness costs no dynamic dispatch.

## License

MIT.
