# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    abstract type AbstractJointModel

Supertype of a two-body joint's stiffness model. The concrete subtype is the
type parameter of [`Joint`](@ref), so a collection of joints sharing one model
is concretely typed. Built-in subtypes: [`ElasticJoint`](@ref),
[`TimoshenkoBeam`](@ref).

Every stiffness is either a `Real` (a linear law) or a callable of that degree
of freedom's deformation returning the effective stiffness there — a
curvature-softening `EIy(κ)` for an inflating tube, say. Callables must all be
the same type, which is what keeps the joint concrete.
"""
abstract type AbstractJointModel end

"""
    ElasticJoint(; stiffness_axial, stiffness_shear, stiffness_torsion,
                 stiffness_bending)

Lumped 6-DOF elasticity: a restoring wrench from the relative pose of the two
anchors, decomposed in body A's frame.

$(TYPEDFIELDS)
"""
struct ElasticJoint{S} <: AbstractJointModel
    "axial stiffness: `EA` [N/m], or a callable `force(Δx)` (body A x axis)"
    stiffness_axial::S
    "shear stiffness: `GA` [N/m], or a callable `force(Δ)` (both transverse)"
    stiffness_shear::S
    "torsional stiffness: `GJ` [N·m/rad], or a callable `moment(Δθ)` (x axis)"
    stiffness_torsion::S
    "bending stiffness: `EI` [N·m/rad], or a callable `moment(Δθ)` (transverse)"
    stiffness_bending::S
end

function ElasticJoint(; stiffness_axial, stiffness_shear, stiffness_torsion,
                      stiffness_bending)
    stiffnesses = joint_stiffnesses("ElasticJoint",
        (stiffness_axial, stiffness_shear, stiffness_torsion, stiffness_bending))
    return ElasticJoint{eltype(stiffnesses)}(stiffnesses...)
end

"""
    TimoshenkoBeam(; EA, GA, GJ, EIy, EIz, shear_coeff=5/6, rest_length=0.0)

A 2-node Timoshenko beam element: the distributed-compliance counterpart of
[`ElasticJoint`](@ref), coupling each node's transverse displacement to its
rotation so that transverse shear is represented. A chain of them is a beam.

$(TYPEDFIELDS)
"""
struct TimoshenkoBeam{S} <: AbstractJointModel
    "axial rigidity: `EA` [N], or a callable `EA(ε)` of the axial strain"
    EA::S
    "shear rigidity before `shear_coeff`: `GA` [N], or a callable `GA(γ)`"
    GA::S
    "torsional rigidity: `GJ` [N·m²], or a callable `GJ(κ)` of the twist rate"
    GJ::S
    "bending rigidity about y: `EIy` [N·m²], or a callable `EIy(κ)`"
    EIy::S
    "bending rigidity about z: `EIz` [N·m²], or a callable `EIz(κ)`"
    EIz::S
    "shear correction factor [-]: 5/6 solid, 8/9 an inflated tube"
    shear_coeff::SimFloat
    "rest (unstrained) chord length [m]; 0 = taken from the placed geometry"
    rest_length::SimFloat
end

function TimoshenkoBeam(; EA, GA, GJ, EIy, EIz, shear_coeff=5 / 6,
                        rest_length=0.0)
    rigidities = joint_stiffnesses("TimoshenkoBeam", (EA, GA, GJ, EIy, EIz))
    return TimoshenkoBeam{eltype(rigidities)}(rigidities...,
        SimFloat(shear_coeff), SimFloat(rest_length))
end

"""
    joint_stiffnesses(label, stiffnesses) -> Vector

`stiffnesses` with every `Real` narrowed to `SimFloat` and callables left as
they are, typed by their common supertype. More than one callable type is an
error: it is what would make the joint's fields abstract.
"""
function joint_stiffnesses(label, stiffnesses)
    converted = [stiffness isa Real ? SimFloat(stiffness) : stiffness
                 for stiffness in stiffnesses]
    callable_types = unique(typeof(s) for s in converted if !(s isa Real))
    length(callable_types) > 1 && error("$label: all callable stiffnesses " *
        "must be the same type, got $callable_types. Mix only `Real`s and a " *
        "single callable type.")
    return Vector{Union{map(typeof, converted)...}}(converted)
end

"""
    mutable struct Joint{M<:AbstractJointModel}

An elastic connection between two [`Body`](@ref)s, anchored at a body-frame
offset on each, carrying its stiffness model in `model`.

$(TYPEDFIELDS)
"""
mutable struct Joint{M<:AbstractJointModel}
    "position in the joints collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved index of body A"
    body_a_idx::Int64
    "resolved index of body B"
    body_b_idx::Int64
    "raw reference of body A"
    const body_a_ref::NameRef
    "raw reference of body B"
    const body_b_ref::NameRef
    "anchor offset from body A's origin, body A frame [m]"
    const anchor_a_b::KVec3
    "anchor offset from body B's origin, body B frame [m]"
    const anchor_b_b::KVec3
    "stiffness model of this joint"
    model::M
    "Rayleigh damping β [s]: every degree of freedom is damped in proportion
    to its own stiffness, so rigid motion stays undamped"
    damping::SimFloat
    "cylinder radius for drawing the element [m]; `nothing` = not drawn"
    radius::Union{Nothing, SimFloat}
end

"""
    Joint(name, body_a, body_b, model; anchor_a, anchor_b, damping=0.0,
          radius=nothing)

Connect `body_a` to `body_b` (names or indices) with the stiffness `model`.
`anchor_a` and `anchor_b` are the connection points in each body's own frame
[m]. `radius` is for drawing only and has no effect on the dynamics.
"""
Joint(name, body_a, body_b, model::AbstractJointModel;
      anchor_a=zeros(KVec3), anchor_b=zeros(KVec3), damping=0.0,
      radius=nothing) =
    Joint(0, name, 0, 0, name_ref(body_a), name_ref(body_b),
          KVec3(anchor_a), KVec3(anchor_b), model, SimFloat(damping),
          opt_simfloat(radius))
