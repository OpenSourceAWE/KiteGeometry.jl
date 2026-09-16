# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    mutable struct WeightedRefPoints

A weighted combination of points, used to define a body's frame from the
structure around it.

$(TYPEDFIELDS)
"""
mutable struct WeightedRefPoints
    "raw point references"
    const refs::Vector{NameRef}
    "resolved point indices"
    ids::Vector{Int64}
    "weights, normalised to sum to 1"
    const weights::Vector{Float64}
end

"""
    WeightedRefPoints(reference)
    WeightedRefPoints(references::AbstractVector)

One point, an equal-weight average of several (`[:le, :te]`), or a weighted
combination (`[(:le, 0.7), (:te, 0.3)]`).
"""
WeightedRefPoints(reference::Integer) =
    WeightedRefPoints(NameRef[], Int64[Int64(reference)], [1.0])
WeightedRefPoints(reference::Symbol) =
    WeightedRefPoints(NameRef[reference], Int64[], [1.0])

function WeightedRefPoints(references::AbstractVector)
    isempty(references) && error("WeightedRefPoints needs at least one " *
        "reference point, got an empty vector.")
    if references[1] isa Tuple
        weights = Float64[Float64(entry[2]) for entry in references]
        normalize_weights!(weights)
        return WeightedRefPoints(
            name_refs(entry[1] for entry in references), Int64[], weights)
    end
    refs = name_refs(references)
    return WeightedRefPoints(refs, Int64[], fill(1 / length(refs), length(refs)))
end

"""
    normalize_weights!(weights)

Scale `weights` to sum to 1, warning when they did not already.
"""
function normalize_weights!(weights::Vector{Float64})
    total = sum(weights)
    total > 0 || error("Reference point weights sum to $total; " *
        "every weight must be positive.")
    isapprox(total, 1.0; atol=1e-6) && return
    @warn "Reference point weights sum to $total, normalizing to 1.0"
    weights ./= total
end

# ==================== BODY ==================== #

"""
    mutable struct Body

A rigid body, optionally carrying aerodynamics — a body that does is a wing,
built by [`Wing`](@ref).

The rigid-body core (`mass`, `inertia_principal`, the frames) is either given
here or derived by the simulator from the body's points; a wing whose frame is
defined by `z_ref_points`, `y_ref_points` and `origin` leaves them at their
defaults.

$(TYPEDFIELDS)
"""
mutable struct Body
    "position in the bodies collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved transform index; 0 = no transform"
    transform_idx::Int64
    "raw transform reference; 0 = no transform"
    const transform_ref::NameRef
    "resolved parent-wing index; 0 = no parent wing"
    wing_idx::Int64
    "raw parent-wing reference; 0 = no parent wing"
    const wing_ref::NameRef
    "total mass [kg]; 0 = derived from the body's points"
    mass::SimFloat
    "principal moments of inertia `[Ixx, Iyy, Izz]` [kg·m²]"
    const inertia_principal::KVec3
    "body frame → principal frame"
    const R_KA_to_principal::Matrix{SimFloat}
    "offset from the body origin to the centre of mass, body frame [m]"
    const com_offset_KA::KVec3
    "how the principal frame is computed from an inertia tensor"
    principal_frame_method::PrincipalFrameMethod
    "per-axis spin damping about the body axes [1/s]"
    angular_damping::KVec3
    "per-mass translational damping on the world axes [1/s]"
    world_frame_damping::KVec3
    "per-mass translational damping on the body axes [1/s]"
    body_frame_damping::KVec3
    "if true, the centre of mass is confined to a sphere about the origin"
    fix_sphere::Bool
    "if true, the body is frozen where it is"
    fix_static::Bool
    "dynamics type"
    type::DynamicsType
    "structural representation of a wing"
    dynamics_type::WingType
    "position of the body origin in the CAD frame [m]"
    const pos_cad::KVec3
    "body frame → CAD frame; the placed orientation before any transform"
    const R_KA_to_CAD::Matrix{SimFloat}
    "name of the aerodynamic model this body carries; `:none` = a plain body"
    aero_model::Symbol
    "resolved twist-surface indices"
    twist_surface_idxs::Vector{Int64}
    "raw twist-surface references"
    const twist_surface_refs::Vector{NameRef}
    "fraction of the body's drag applied at the body rather than its points"
    drag_frac::SimFloat
    "whether twist-surface points add their moment to the body"
    group_points_moment::Bool
    "point pair defining the body-frame z axis"
    z_ref_points::Union{Nothing, Tuple{WeightedRefPoints, WeightedRefPoints}}
    "point pair defining the body-frame y axis"
    y_ref_points::Union{Nothing, Tuple{WeightedRefPoints, WeightedRefPoints}}
    "points whose weighted centroid is the body origin"
    origin::Union{Nothing, WeightedRefPoints}
end

"""
    is_wing(body::Body) -> Bool

Whether `body` carries aerodynamics, which is what makes a body a wing.
"""
is_wing(body::Body) = body.aero_model !== :none

"""
    Body(name; mass, inertia_principal | inertia, pos_cad, Q_KA_to_CAD,
         com_offset_KA, R_KA_to_principal, angular_damping,
         world_frame_damping, body_frame_damping, fix_sphere, fix_static,
         type, transform, wing, principal_frame_method)

A plain rigid body of mass `mass` [kg] whose origin sits at `pos_cad` [m] in
the CAD frame, oriented by the quaternion `Q_KA_to_CAD` (scalar first).

Give the inertia one of two ways: `inertia_principal`, the three principal
moments, with `R_KA_to_principal` naming the body → principal rotation; or
`inertia`, the full 3×3 body-frame tensor, from which both are derived by
`principal_frame_method`. One, not both.

`type` is `DYNAMIC` (free 6-DOF) or `STATIC` (clamped to its placed pose).
`transform` places the body like a wing; `wing` names the parent wing the
body-frame damping resolves against.
"""
function Body(name;
    mass::Real, inertia_principal=nothing, inertia=nothing, pos_cad,
    Q_KA_to_CAD=SimFloat[1, 0, 0, 0], com_offset_KA=zeros(KVec3),
    R_KA_to_principal=Matrix{SimFloat}(I, 3, 3), angular_damping=0.0,
    world_frame_damping=0.0, body_frame_damping=0.0, fix_sphere=false,
    fix_static=false, type::DynamicsType=DYNAMIC, transform=nothing,
    wing=nothing, principal_frame_method::PrincipalFrameMethod=EIGEN_DECOMP
)
    type in (DYNAMIC, STATIC) ||
        error("Body $name: type must be DYNAMIC or STATIC, got $type.")
    if !isnothing(inertia)
        isnothing(inertia_principal) ||
            error("Body $name: give `inertia` or `inertia_principal`, not both.")
        inertia_principal, R_KA_to_principal =
            principal_frame(inertia, principal_frame_method)
    elseif isnothing(inertia_principal)
        error("Body $name: provide `inertia_principal` or `inertia`.")
    end
    return Body(0, name, 0, isnothing(transform) ? 0 : transform,
        0, isnothing(wing) ? 0 : wing,
        SimFloat(mass), KVec3(inertia_principal),
        Matrix{SimFloat}(R_KA_to_principal), KVec3(com_offset_KA),
        principal_frame_method, damping_vector(angular_damping),
        damping_vector(world_frame_damping), damping_vector(body_frame_damping),
        fix_sphere, fix_static, type, RIGID_DYNAMICS, KVec3(pos_cad),
        quaternion_to_rotation_matrix(Q_KA_to_CAD),
        :none, Int64[], NameRef[], one(SimFloat), true, nothing, nothing, nothing)
end

"""
    Wing(name, twist_surfaces; dynamics_type=RIGID_DYNAMICS, aero_model,
         transform, pos_cad, R_KA_to_CAD, inertia_principal, mass,
         com_offset_KA, angular_damping, world_frame_damping,
         body_frame_damping, group_points_moment, z_ref_points, y_ref_points,
         origin, principal_frame_method)

Build a [`Body`](@ref) carrying aerodynamics, spanning the given twist surfaces.

`aero_model` names the aerodynamic model the simulator is to build; this
package only records the name, and defaults it to `:linearized` for a rigid
wing and `:direct` for a particle one. `mass` and `inertia_principal` default to zero,
which asks the simulator to derive them from the wing's points, as do the frame
keywords when `z_ref_points`, `y_ref_points` and `origin` are given instead.
"""
function Wing(name, twist_surfaces;
    dynamics_type::WingType=RIGID_DYNAMICS, aero_model=nothing,
    transform=nothing, pos_cad=zeros(KVec3),
    R_KA_to_CAD=Matrix{SimFloat}(I, 3, 3), inertia_principal=zeros(KVec3),
    mass=0.0, com_offset_KA=zeros(KVec3), angular_damping=[0.0, 150.0, 0.0],
    world_frame_damping=0.0, body_frame_damping=0.0, drag_frac=1.0,
    group_points_moment::Bool=true, z_ref_points=nothing, y_ref_points=nothing,
    origin=nothing, principal_frame_method::PrincipalFrameMethod=EIGEN_DECOMP
)
    isnothing(aero_model) && (aero_model =
        dynamics_type == RIGID_DYNAMICS ? :linearized : :direct)
    return Body(0, name, 0, isnothing(transform) ? 1 : transform, 0, 0,
        SimFloat(mass), KVec3(inertia_principal),
        Matrix{SimFloat}(I, 3, 3), KVec3(com_offset_KA), principal_frame_method,
        damping_vector(angular_damping), damping_vector(world_frame_damping),
        damping_vector(body_frame_damping), false, false,
        dynamics_type == RIGID_DYNAMICS ? DYNAMIC : KINEMATIC, dynamics_type,
        KVec3(pos_cad), Matrix{SimFloat}(R_KA_to_CAD),
        Symbol(aero_model), Int64[], name_refs(twist_surfaces),
        SimFloat(drag_frac),
        group_points_moment, weighted_ref_pair(z_ref_points),
        weighted_ref_pair(y_ref_points),
        isnothing(origin) ? nothing : WeightedRefPoints(origin))
end

"""
    weighted_ref_pair(points) -> Union{Tuple, Nothing}

The two [`WeightedRefPoints`](@ref) of a frame-fitting axis, or `nothing`.
"""
weighted_ref_pair(points) = isnothing(points) ? nothing :
    (WeightedRefPoints(points[1]), WeightedRefPoints(points[2]))

"""
    principal_frame(inertia, method=EIGEN_DECOMP)
        -> (inertia_principal, R_KA_to_principal)

Diagonalise a 3×3 symmetric inertia tensor into its principal moments and the
rotation `R` taking the input frame to the principal frame, so that
`R · inertia · R' = Diagonal(inertia_principal)`.

Under `EIGEN_DECOMP` the principal axes are permuted and signed to align as
closely as possible with the input axes — a near-diagonal tensor gives `R ≈ I`
— and `R` is always a proper rotation. Under `Y_ROTATION` the rotation is the
closed-form one about y that zeroes the `I[1, 3]` cross terms, which is unique
where the permutation search is ambiguous.
"""
function principal_frame(inertia::AbstractMatrix,
                         method::PrincipalFrameMethod=EIGEN_DECOMP)
    method == Y_ROTATION && return inertia_y_rotation(inertia)
    decomposition = eigen(Symmetric(Matrix{SimFloat}(inertia)))
    moments = decomposition.values
    axes = Matrix(decomposition.vectors)   # columns: principal axes, input frame
    order = collect(best_axis_permutation(axes))
    axes, moments = axes[:, order], moments[order]
    for i in 1:3
        axes[i, i] < 0 && (axes[:, i] .*= -1)
    end
    if det(axes) < 0
        flip = argmin([abs(axes[i, i]) for i in 1:3])
        axes[:, flip] .*= -1
    end
    return Vector{SimFloat}(moments), Matrix{SimFloat}(axes')
end

"""
    best_axis_permutation(axes) -> NTuple{3, Int}

The column order of `axes` that aligns principal axis `i` most closely with
input axis `i`.
"""
function best_axis_permutation(axes)
    best_perm, best_score = (1, 2, 3), -Inf
    for perm in ((1,2,3), (1,3,2), (2,1,3), (2,3,1), (3,1,2), (3,2,1))
        score = abs(axes[1, perm[1]]) + abs(axes[2, perm[2]]) +
                abs(axes[3, perm[3]])
        score > best_score && ((best_perm, best_score) = (perm, score))
    end
    return best_perm
end

"""
    inertia_y_rotation(inertia) -> (inertia_principal, R_KA_to_principal)

Diagonalise `inertia` by the closed-form rotation about y through
`θ = atan(2·I₁₃, I₁₁ − I₃₃) / 2`, which zeroes the `I[1, 3]` cross terms and
leaves the y axis alone.
"""
function inertia_y_rotation(inertia)
    θ = atan(2 * inertia[1, 3], inertia[1, 1] - inertia[3, 3]) / 2
    rotation = SimFloat[cos(θ) 0 sin(θ); 0 1 0; -sin(θ) 0 cos(θ)]
    return Vector{SimFloat}(diag(rotation * inertia * rotation')), rotation
end

"""
    quaternion_to_rotation_matrix(quaternion) -> Matrix{SimFloat}

Rotation matrix of a scalar-first quaternion `[w, x, y, z]`. The quaternion need
not be normalised.
"""
function quaternion_to_rotation_matrix(quaternion::AbstractVector)
    w, x, y, z = quaternion[1], quaternion[2], quaternion[3], quaternion[4]
    s = 2 / (w * w + x * x + y * y + z * z)
    return SimFloat[
        1-s*(y*y+z*z) s*(x*y-z*w)   s*(x*z+y*w)
        s*(x*y+z*w)   1-s*(x*x+z*z) s*(y*z-x*w)
        s*(x*z-y*w)   s*(y*z+x*w)   1-s*(x*x+y*y)
    ]
end
