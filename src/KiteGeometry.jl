# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    KiteGeometry

One system definition for airborne wind energy systems: the topology, the geometry and the
material of a kite, a tether and a winch, in one type, `SystemDefinition`.

Its component types are generated from the awesIO structure schema by `src/build.jl`.

See the [documentation](https://OpenSourceAWE.github.io/KiteGeometry.jl/stable/)
for more information.
"""
module KiteGeometry

using OrderedCollections: OrderedDict
using StaticArrays: SMatrix, SVector
using YAML: YAML

export SystemDefinition, Metadata, Point, Segment, Station, Pulley, Tether, Winch, Body,
       Tube
export DynamicsType, DYNAMIC, STATIC, BODY_STATIC, KINEMATIC
export NameRef, AbstractTubeModel, PlainTube, register_tube_model!, tube_model
export load_structure, structure_document

"""
    NameRef

A reference to another component: its name as a document writes it, or its index into
its block once a `SystemDefinition` has resolved it.
"""
const NameRef = Union{Int, String}

"""
    Component

Supertype of the row types of the table blocks.
"""
abstract type Component end

include("tube_models.jl")
include("_structure.jl")
include("components.jl")
include("document.jl")

end
