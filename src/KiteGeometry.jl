# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    KiteGeometry

One system definition for airborne wind energy systems: the topology, the CAD
geometry and the material of a kite, a tether and a winch, in one type.

[`SystemDefinition`](@ref) is that type and [`load_definition`](@ref) reads one
from an authoring YAML file. Nothing here carries simulation state, so a
definition can be shared between the package that integrates it and the ones
that draw it.

See the [documentation](https://OpenSourceAWE.github.io/KiteGeometry.jl/stable/)
for more information.
"""
module KiteGeometry

using LinearAlgebra
using StaticArrays
using DocStringExtensions
using YAML
using KiteUtils

export SystemDefinition, NamedCollection, load_definition
export Point, TwistSurface, Segment, Pulley, Tether, Winch, Body, Wing
export Joint, Transform, WeightedRefPoints
export AbstractSegmentModel, LinearSpring, NonlinearSpring
export AbstractJointModel, ElasticJoint, TimoshenkoBeam
export AbstractWinchModel, TorqueWinch
export DynamicsType, DYNAMIC, STATIC, BODY_STATIC, KINEMATIC
export WingType, RIGID_DYNAMICS, PARTICLE_DYNAMICS
export PrincipalFrameMethod, EIGEN_DECOMP, Y_ROTATION
export NameRef, SimFloat, KVec3
export is_wing, principal_frame

"""
    SimFloat = Float64

The floating-point type every quantity in a definition is stored as.
"""
const SimFloat = Float64

"""
    KVec3 = MVector{3, SimFloat}

A mutable 3-vector: a position, a direction or a set of per-axis coefficients.
"""
const KVec3 = MVector{3, SimFloat}

include("named_collection.jl")
include("segment_models.jl")
include("winch_models.jl")
include("types.jl")
include("rigid_body.jl")
include("joints.jl")
include("system_definition.jl")
include("yaml_loader.jl")

end
