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

using JSON: JSON
using OrderedCollections: OrderedDict, OrderedSet
using SHA: sha256
using StaticArrays: SMatrix, SVector
using YAML: YAML

export SystemDefinition, Metadata, Point, Segment, Station, Pulley, Tether, Winch, Wing,
       CanopyFace, Body, Tube
export DynamicsType, DYNAMIC, STATIC, BODY_STATIC, KINEMATIC
export NameRef, AbstractModel, NoModel, register_model!, model_type
export load_structure, structure_document, connectivity_sha
export from_yaml, to_yaml, from_json, to_json
public definition

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

include("models.jl")
include("_structure.jl")
include("components.jl")
include("document.jl")

end
