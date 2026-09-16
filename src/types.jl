# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    DynamicsType `DYNAMIC` `STATIC` `BODY_STATIC` `KINEMATIC`

The dynamic model governing a point's motion, a body's motion, or a twist
surface's twist. `DYNAMIC` quantities carry differential state and are solved
by the dynamics; the others are prescribed.

# Elements
- `DYNAMIC`: solved by the dynamics — a point moves by Newton's second law, a
  body integrates its 6-DOF state, a twist surface solves its equilibrium.
- `STATIC`: prescribed and stateless — a point is welded in the world frame, a
  body is clamped to the world, a twist angle is a control input.
- `BODY_STATIC`: the point rides a [`Body`](@ref), static in that body's frame,
  and feeds its net force and moment into it.
- `KINEMATIC`: a twist source whose deflection is prescribed by geometry, such
  as a flap hinge between two bodies.
"""
@enum DynamicsType begin
    DYNAMIC
    STATIC
    BODY_STATIC
    KINEMATIC
end

"""
    WingType `RIGID_DYNAMICS` `PARTICLE_DYNAMICS`

The structural representation of a wing.

# Elements
- `RIGID_DYNAMICS`: quaternion-based rigid-body dynamics, deformation confined
  to the twist degrees of freedom of its [`TwistSurface`](@ref)s. Its
  structural points are `BODY_STATIC` and ride the body.
- `PARTICLE_DYNAMICS`: no rigid-body constraint. Its structural points are
  ordinary `DYNAMIC` particles, so the wing deforms with the structure.
"""
@enum WingType begin
    RIGID_DYNAMICS
    PARTICLE_DYNAMICS
end

"""
    PrincipalFrameMethod `EIGEN_DECOMP` `Y_ROTATION`

How the principal frame is computed from an inertia tensor.

# Elements
- `EIGEN_DECOMP`: full 3-axis eigendecomposition plus permutation search.
  Correct for any body.
- `Y_ROTATION`: closed-form rotation about y only, for wings symmetric about
  the xz-plane, where the permutation search is ambiguous when two principal
  moments are close.
"""
@enum PrincipalFrameMethod begin
    EIGEN_DECOMP
    Y_ROTATION
end

"""
    NameRef = Union{Int, Symbol}

A reference to another component, by symbolic name (`:ground`) or by position
(`1`).

Components reference each other by name or position at construction, stored in
`_ref` fields (`point_refs`, `wing_ref`, …). Building a
[`SystemDefinition`](@ref) resolves every one of them to a position and stores
it in the matching `_idx` field (`point_idxs`, `wing_idx`, …).
"""
const NameRef = Union{Int, Symbol}

# ==================== POINT ==================== #

"""
    mutable struct Point

A point mass: a node of the mass-spring system.

$(TYPEDFIELDS)
"""
mutable struct Point
    "position in the points collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved transform index; 0 = no transform"
    transform_idx::Int64
    "resolved wing index; 0 = no wing"
    wing_idx::Int64
    "resolved body index of a body-anchored point; 0 = not anchored"
    body_idx::Int64
    "resolved joint index of a beam-anchored point; 0 = not beam-anchored"
    joint_idx::Int64
    "raw transform reference; 0 = no transform"
    const transform_ref::NameRef
    "raw wing reference; 0 = no wing"
    const wing_ref::NameRef
    "raw body reference for anchoring; 0 = not anchored to a body"
    const body_ref::NameRef
    "raw beam-anchoring joint reference; 0 = not beam-anchored"
    const joint_ref::NameRef
    "position in the CAD frame [m]"
    const pos_CAD::KVec3
    "undeformed position relative to the wing COM, principal frame [m]"
    const pos_undeformed_KA::KVec3
    "anchor offset in the anchoring body's frame [m]; derived from `pos_CAD`
    when left at zero"
    anchor_KA::KVec3
    "parameter `s ∈ [0, 1]` along the beam element; derived from `pos_CAD`"
    beam_frac::SimFloat
    "perpendicular offset off the beam centerline, rest element frame [m]"
    beam_offset_b::KVec3
    "dynamics type"
    const type::DynamicsType
    "point mass [kg], excluding the share of the connected segments"
    extra_mass::SimFloat
    "per-axis damping in the body frame [N·s/m]"
    body_frame_damping::KVec3
    "per-axis damping in the world frame [N·s/m]"
    world_frame_damping::KVec3
    "cross-sectional area for drag [m²]"
    area::SimFloat
    "drag coefficient [-]"
    drag_coeff::SimFloat
    "if true, the point is constrained to a sphere"
    fix_sphere::Bool
    "if true, the point is frozen where it is"
    fix_static::Bool
    "true when the point is a structural node of one of a wing's twist
    surfaces; derived from twist-surface membership"
    is_wing_node::Bool
end

"""
    Point(name, pos_CAD, type; wing, transform, body, joint, ...)

A point mass at CAD position `pos_CAD` [m] with the given
[`DynamicsType`](@ref).

A `BODY_STATIC` point rides a [`Body`](@ref) and needs one of `body`, `joint`
or `wing` to say which — a wing is a body, so `wing` rides that wing's body.

# Keyword Arguments
- `wing`: wing reference (name or index).
- `transform`: [`Transform`](@ref) reference placing the point.
- `body`: [`Body`](@ref) the point is anchored to; requires `BODY_STATIC`.
- `joint`: beam [`Joint`](@ref) whose deformed centerline the point rides;
  requires `BODY_STATIC`.
- `anchor_KA::KVec3`: anchor offset in the body frame [m], used with `body`.
- `extra_mass`: point mass [kg].
- `body_frame_damping`, `world_frame_damping`: scalar or per-axis damping
  [N·s/m].
- `area`: cross-sectional area for drag [m²].
- `drag_coeff`: drag coefficient [-].
- `fix_sphere`, `fix_static`: constrain the point to a sphere, or freeze it.
"""
function Point(name, pos_CAD, type;
    wing=nothing, transform=nothing, body=nothing, anchor_KA=nothing,
    joint=nothing, extra_mass=0.0, body_frame_damping=nothing,
    world_frame_damping=nothing, area=0.0, drag_coeff=0.0,
    fix_sphere=false, fix_static=false
)
    if type == BODY_STATIC
        (isnothing(body) && isnothing(joint) && isnothing(wing)) && error(
            "Point $name: BODY_STATIC requires a `body`, a `joint`, or a " *
            "`wing` reference (a wing is a body, so `wing` rides that " *
            "wing's body).")
    elseif !isnothing(body)
        error("Point $name: `body` is only valid with type BODY_STATIC.")
    end
    (!isnothing(body) && !isnothing(joint)) && error(
        "Point $name: set either `body` (rigid rider) or `joint` (beam " *
        "rider), not both.")
    (!isnothing(joint) && type != BODY_STATIC) && error(
        "Point $name: `joint` (beam anchoring) requires type BODY_STATIC.")
    # A body-anchored point takes its wing from the body it rides.
    wing_ref = isnothing(wing) ? (type == BODY_STATIC ? 0 : 1) : wing
    Point(0, name, 0, 0, 0, 0,
        isnothing(transform) ? 0 : transform, wing_ref,
        isnothing(body) ? 0 : body, isnothing(joint) ? 0 : joint,
        KVec3(pos_CAD...), zeros(KVec3),
        isnothing(anchor_KA) ? zeros(KVec3) : KVec3(anchor_KA...),
        zero(SimFloat), zeros(KVec3),
        type, extra_mass,
        damping_vector(body_frame_damping), damping_vector(world_frame_damping),
        area, drag_coeff, fix_sphere, fix_static, false)
end

"""
    damping_vector(damping) -> KVec3

Per-axis damping from `nothing` (no damping), a scalar (the same on all three
axes) or a 3-vector.
"""
damping_vector(::Nothing) = zeros(KVec3)
damping_vector(damping::Real) = KVec3(damping, damping, damping)
damping_vector(damping) = KVec3(damping...)

# ==================== TWIST SURFACE ==================== #

"""
    mutable struct TwistSurface

A chordwise piece of a wing whose points share one twist angle.

$(TYPEDFIELDS)
"""
mutable struct TwistSurface
    "position in the twist_surfaces collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved point indices"
    point_idxs::Vector{Int64}
    "raw point references"
    const point_refs::Vector{NameRef}
    "resolved owning-wing index; 0 = inferred from body membership"
    wing_idx::Int64
    "raw owning-wing reference; 0 = inferred from body membership"
    const wing_ref::NameRef
    "resolved member-body indices — every body in this chordwise piece"
    body_idxs::Vector{Int64}
    "raw member-body references"
    const body_refs::Vector{NameRef}
    "resolved flap-hinge body indices, ordered `[main, flap]`; empty = no flap"
    flap_body_idxs::Vector{Int64}
    "raw flap-hinge body references, ordered `[main, flap]`; empty = no flap"
    const flap_body_refs::Vector{NameRef}
    "dynamics type of the twist degree of freedom"
    const type::DynamicsType
    "chordwise rotation point fraction (0 = leading edge, 1 = trailing edge)"
    moment_frac::SimFloat
    "damping of the twist dynamics [N·m·s/rad]"
    damping::SimFloat
    "torsional restoring stiffness of the twist dynamics [N·m/rad]"
    stiffness::SimFloat
    "chord direction in the body frame; twist is measured against it. Zero =
    derived from the aerodynamic geometry"
    chord::KVec3
    "spanwise direction in the body frame. Zero = derived from the
    aerodynamic geometry"
    y_airf::KVec3
    "flap-hinge axis (unit) in the main body's frame"
    flap_axis::KVec3
    "surface area [m²] of a flat-plate section; `NaN` when unused"
    area::SimFloat
end

"""
    TwistSurface(name, points, type, moment_frac; damping=50.0, ...)

The points of one chordwise piece of a wing, sharing a twist angle.

# Keyword Arguments
- `damping`: damping of the twist dynamics [N·m·s/rad].
- `stiffness`: torsional restoring stiffness [N·m/rad].
- `x_airf`, `y_airf`: chord and spanwise reference directions in the body
  frame. Derived from the aerodynamic geometry when omitted.
- `area`: surface area [m²] of a flat-plate section.
- `wing`: owning-wing reference. Other surfaces infer their wing from body
  membership.
- `bodies`: member-body references — every body in this chordwise piece.
- `flap_bodies`: ordered `[main, flap]` body references of a flap hinge. With
  `type = KINEMATIC` the surface then carries a live deflection.
- `flap_axis`: flap-hinge axis (unit) in the main body's frame.
"""
function TwistSurface(name, points, type, moment_frac;
    damping=50.0, stiffness=0.0, x_airf=nothing, y_airf=nothing, area=NaN,
    wing=0, bodies=NameRef[], flap_bodies=NameRef[], flap_axis=[0.0, 1.0, 0.0]
)
    TwistSurface(0, name, Int64[], name_refs(points),
        0, name_ref(wing), Int64[], name_refs(bodies),
        Int64[], name_refs(flap_bodies),
        type, moment_frac, damping, stiffness,
        isnothing(x_airf) ? zeros(KVec3) : KVec3(x_airf),
        isnothing(y_airf) ? zeros(KVec3) : KVec3(y_airf),
        KVec3(flap_axis), SimFloat(area))
end

# ==================== SEGMENT ==================== #

"""
    mutable struct Segment{M<:AbstractSegmentModel}

A spring-damper connecting two points, carrying its force law in `model`.

$(TYPEDFIELDS)
"""
mutable struct Segment{M<:AbstractSegmentModel}
    "position in the segments collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved endpoint indices"
    point_idxs::Tuple{Int64, Int64}
    "raw endpoint references"
    const point_refs::Tuple{NameRef, NameRef}
    "force law of this segment"
    model::M
    "rest (unstretched) length [m]; 0 = taken from the CAD geometry"
    l0::SimFloat
    "compressive/tensile stiffness ratio (0-1); 0 = a slack segment carries no
    spring force"
    compression_frac::SimFloat
    "fraction of the damping that still acts under compression (0-1)"
    compression_damping_frac::SimFloat
    "segment diameter [m]"
    diameter::SimFloat
    "material density [kg/m³]"
    density::SimFloat
end

"""
    Segment(name, point_i, point_j, model; l0=0, diameter=NaN, density=NaN,
            compression_frac=0.1, compression_damping_frac=1.0)

A spring-damper from `point_i` to `point_j` (names or indices) with the force
law `model`.

# Keyword Arguments
- `l0`: rest length [m]; 0 takes it from the CAD geometry.
- `diameter`: segment diameter [m].
- `density`: material density [kg/m³].
- `compression_frac`: compressive/tensile stiffness ratio (0-1).
- `compression_damping_frac`: fraction of the damping still acting under
  compression (0-1).
"""
function Segment(name, point_i, point_j, model::AbstractSegmentModel;
    l0=zero(SimFloat), diameter=NaN, density=NaN,
    compression_frac=0.1, compression_damping_frac=1.0
)
    Segment(0, name, (0, 0), (name_ref(point_i), name_ref(point_j)), model,
        SimFloat(l0), SimFloat(compression_frac),
        SimFloat(compression_damping_frac), SimFloat(diameter),
        SimFloat(density))
end

"""
    Segment(name, set, point_i, point_j; unit_stiffness, unit_damping,
            youngs_modulus, damping_per_stiffness, diameter, density, l0, ...)

A spring-damper whose force law is completed from `set` by
[`segment_model`](@ref): whatever material keyword is left out comes from the
settings.
"""
function Segment(name, set::Settings, point_i, point_j;
    l0=zero(SimFloat), compression_frac=0.1, compression_damping_frac=1.0,
    diameter=NaN, unit_stiffness=NaN, unit_damping=NaN, density=NaN,
    youngs_modulus=NaN, damping_per_stiffness=NaN
)
    model, diameter, density = segment_model("Segment $name", set;
        unit_stiffness, unit_damping, diameter, density, youngs_modulus,
        damping_per_stiffness)
    return Segment(name, point_i, point_j, model;
        l0, diameter, density, compression_frac, compression_damping_frac)
end

# ==================== PULLEY ==================== #

"""
    mutable struct Pulley

A sheave over which two segments share one length at equal tension.

$(TYPEDFIELDS)
"""
mutable struct Pulley
    "position in the pulleys collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved indices of the two segments sharing length"
    segment_idxs::Tuple{Int64, Int64}
    "raw references of the two segments sharing length"
    const segment_refs::Tuple{NameRef, NameRef}
    "dynamics type"
    const type::DynamicsType
    "fraction of the line tension the sheave passes on (0-1); the rest opposes
    travel"
    efficiency::SimFloat
    "artificial damping on rope travel [N·s/m]; a debugging aid, not a sheave
    property"
    damping::SimFloat
    "friction smoothing width [m/s]: the rope speed below which the friction's
    sign ramps in"
    friction_epsilon::SimFloat
end

"""
    Pulley(name, segment_i, segment_j, type; efficiency=0.95, damping=0.0,
           friction_epsilon=0.1)

A sheave redistributing length between `segment_i` and `segment_j`.

`efficiency` is the whole friction model: the friction is
`(1 − efficiency) · line_tension`, so it scales with load rather than with rope
speed. 0.95 is a sealed ball-bearing sheave, 0.88–0.92 a bronze bushing, 1.0 an
ideal pulley. A narrow `friction_epsilon` makes a stiff system out of a small
force and wants raising rather than lowering.
"""
Pulley(name, segment_i, segment_j, type;
       efficiency=0.95, damping=0.0, friction_epsilon=0.1) =
    Pulley(0, name, (0, 0), (name_ref(segment_i), name_ref(segment_j)), type,
           SimFloat(efficiency), SimFloat(damping), SimFloat(friction_epsilon))

# ==================== TETHER ==================== #

"""
    mutable struct Tether{M<:AbstractSegmentModel}

A line built from segments, reeled by a [`Winch`](@ref).

A tether is written either as explicit segment references, or as a start point,
an end point and a segment count — in which case building the
[`SystemDefinition`](@ref) generates the intermediate points and segments, each
carrying this tether's `model`, `diameter` and `density`.

# Initial length
Two lengths, set independently:
- `init_stretched_len` is the placed standoff the geometry spans.
- the unstretched rest length is derived from it by `init_stretch_frac`
  (`len = frac · stretched`) or by `init_tether_force`
  (`len = stretched · (1 − force/stiffness)`).

$(TYPEDFIELDS)
"""
mutable struct Tether{M<:AbstractSegmentModel}
    "position in the tethers collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved segment indices"
    segment_idxs::Vector{Int64}
    "raw segment references"
    const segment_refs::Vector{NameRef}
    "resolved start point index"
    start_point_idx::Int64
    "raw start point reference; `nothing` when written as explicit segments"
    const start_point_ref::Union{NameRef, Nothing}
    "resolved end point index"
    end_point_idx::Int64
    "raw end point reference; `nothing` when written as explicit segments"
    const end_point_ref::Union{NameRef, Nothing}
    "number of segments"
    const n_segments::Int64
    "force law given to every generated segment; `NaN` for a tether written
    as explicit segments, which carry their own"
    const model::M
    "diameter of every generated segment [m]; `NaN` as above"
    const diameter::SimFloat
    "material density of every generated segment [kg/m³]; `NaN` as above"
    const density::SimFloat
    "compressive/tensile stiffness ratio (0-1) of every generated segment"
    const compression_frac::SimFloat
    "fraction of the damping still acting under compression (0-1), per segment"
    const compression_damping_frac::SimFloat
    "placed stretched standoff [m]; `nothing` = use the CAD length"
    init_stretched_len::Union{SimFloat, Nothing}
    "target initial spring force [N]; mutually exclusive with
    `init_stretch_frac`"
    init_tether_force::Union{SimFloat, Nothing}
    "initial unstretched/stretched length fraction: 0.9 gives 10% pre-stretch,
    1.0 no tension, above 1.0 slack. Mutually exclusive with
    `init_tether_force`"
    init_stretch_frac::Union{SimFloat, Nothing}
end

"""
    Tether(name, segments, stretched_length=nothing; start_point, end_point,
           tether_force, stretch_frac)

A tether from explicit segment references. The segments carry their own
material, so this route takes none.
"""
function Tether(name, segments::AbstractVector, stretched_length=nothing;
    start_point=nothing, end_point=nothing, tether_force=nothing,
    stretch_frac=nothing
)
    init_force, init_frac = tether_init(name, tether_force, stretch_frac)
    return Tether(0, name, Int64[], name_refs(segments),
        0, opt_name_ref(start_point), 0, opt_name_ref(end_point),
        length(segments),
        LinearSpring(NaN, NaN), SimFloat(NaN), SimFloat(NaN), 0.1, 1.0,
        opt_simfloat(stretched_length), init_force, init_frac)
end

"""
    Tether(name, set, stretched_length=nothing; start_point, end_point,
           n_segments, unit_stiffness, unit_damping, youngs_modulus,
           damping_per_stiffness, diameter, density, compression_frac,
           compression_damping_frac, tether_force, stretch_frac)

A tether whose intermediate points and segments are generated when the
[`SystemDefinition`](@ref) is built. Its material is completed from `set` by
[`segment_model`](@ref) and given to every generated segment.
"""
function Tether(name, set::Settings, stretched_length=nothing;
    start_point, end_point, n_segments, unit_stiffness=NaN, unit_damping=NaN,
    diameter=NaN, density=NaN, youngs_modulus=NaN, damping_per_stiffness=NaN,
    compression_frac=0.1, compression_damping_frac=1.0, tether_force=nothing,
    stretch_frac=nothing
)
    init_force, init_frac = tether_init(name, tether_force, stretch_frac)
    model, diameter, density = segment_model("Tether $name", set;
        unit_stiffness, unit_damping, diameter, density, youngs_modulus,
        damping_per_stiffness)
    segment_refs = Vector{NameRef}(
        [Symbol("$(name)_seg_$i") for i in 1:n_segments])
    return Tether(0, name, Int64[], segment_refs,
        0, name_ref(start_point), 0, name_ref(end_point), Int64(n_segments),
        model, diameter, density, SimFloat(compression_frac),
        SimFloat(compression_damping_frac),
        opt_simfloat(stretched_length), init_force, init_frac)
end

"""
    tether_init(name, tether_force, stretch_frac)
        -> (init_tether_force, init_stretch_frac)

The two mutually exclusive ways of deriving a tether's unstretched length from
its placed one. Neither given means a zero initial spring force.
"""
function tether_init(name, tether_force, stretch_frac)
    (!isnothing(tether_force) && !isnothing(stretch_frac)) && error(
        "Tether $name: set only one of `tether_force` and `stretch_frac`.")
    !isnothing(stretch_frac) && return nothing, SimFloat(stretch_frac)
    !isnothing(tether_force) && return SimFloat(tether_force), nothing
    return zero(SimFloat), nothing
end

# ==================== WINCH ==================== #

"""
    mutable struct Winch

A drum reeling one or more tethers.

The winch has no length of its own: it is the mean of its tethers' unstretched
lengths. To set the initial reeled length, set the tethers' initial length.

$(TYPEDFIELDS)
"""
mutable struct Winch{M<:AbstractWinchModel}
    "position in the winches collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved indices of the reeled tethers"
    tether_idxs::Vector{Int64}
    "raw references of the reeled tethers"
    const tether_refs::Vector{NameRef}
    "resolved index of the point the tethers leave the drum at"
    winch_point_idx::Int64
    "raw reference of the point the tethers leave the drum at"
    const winch_point_ref::NameRef
    "initial reel-out velocity [m/s]"
    init_vel::SimFloat
    "if true, the reel-out velocity is prescribed rather than integrated from
    the motor dynamics, and `model` is ignored"
    speed_controlled::Bool
    "gear ratio [-]"
    gear_ratio::SimFloat
    "drum radius [m]"
    drum_radius::SimFloat
    "Coulomb friction force [N]"
    coulomb_friction::SimFloat
    "viscous friction coefficient [N·s/m]"
    viscous_coefficient::SimFloat
    "total rotational inertia seen from the motor [kg·m²]"
    inertia_total::SimFloat
    "motor model carrying its own parameters"
    model::M
end

"""
    Winch(name, set, tethers; winch_point, init_vel=0.0,
          speed_controlled=false, friction_epsilon=6.0, model=TorqueWinch(...))

A winch reeling `tethers` at `winch_point`, with the drum parameters read from
`set` (`gear_ratio`, `drum_radius`, `f_coulomb`, `c_vf`, `inertia_total`).

`friction_epsilon` is forwarded into the default [`TorqueWinch`](@ref) and
ignored when an explicit `model` is passed.
"""
Winch(name, set::Settings, tethers; winch_point, init_vel=0.0,
      speed_controlled=false, friction_epsilon=6.0,
      model::AbstractWinchModel=TorqueWinch(; friction_epsilon)) =
    Winch(name, tethers, set.gear_ratio, set.drum_radius, set.f_coulomb,
          set.c_vf, set.inertia_total;
          winch_point, init_vel, speed_controlled, model)

"""
    Winch(name, tethers, gear_ratio, drum_radius, coulomb_friction,
          viscous_coefficient, inertia_total; winch_point, init_vel=0.0,
          speed_controlled=false, model=TorqueWinch())

A winch whose drum parameters are given directly.
"""
Winch(name, tethers, gear_ratio, drum_radius, coulomb_friction,
      viscous_coefficient, inertia_total; winch_point, init_vel=0.0,
      speed_controlled=false, model::AbstractWinchModel=TorqueWinch()) =
    Winch(0, name, Int64[], name_refs(tethers), 0, name_ref(winch_point),
          SimFloat(init_vel), speed_controlled, SimFloat(gear_ratio),
          SimFloat(drum_radius), SimFloat(coulomb_friction),
          SimFloat(viscous_coefficient), SimFloat(inertia_total), model)

# ==================== TRANSFORM ==================== #

"""
    mutable struct Transform

The map from CAD geometry to the ENU world frame: where a group of components
is placed, and how it is oriented.

$(TYPEDFIELDS)
"""
mutable struct Transform
    "position in the transforms collection (assigned by `SystemDefinition`)"
    idx::Int64
    "name other components' `_ref` fields look up"
    const name::Union{Int, Symbol, Nothing}
    "resolved index of the wing placed at (elevation, azimuth)"
    wing_idx::Union{Int64, Nothing}
    "raw reference of the wing placed at (elevation, azimuth)"
    const wing_ref::Union{NameRef, Nothing}
    "resolved index of the point placed at (elevation, azimuth)"
    rot_point_idx::Union{Int64, Nothing}
    "raw reference of the point placed at (elevation, azimuth)"
    const rot_point_ref::Union{NameRef, Nothing}
    "resolved index of the point placed at `base_pos_ENU`"
    base_point_idx::Union{Int64, Nothing}
    "raw reference of the point placed at `base_pos_ENU`"
    const base_point_ref::Union{NameRef, Nothing}
    "resolved index of the transform this one chains onto"
    base_transform_idx::Union{Int64, Nothing}
    "raw reference of the transform this one chains onto"
    const base_transform_ref::Union{NameRef, Nothing}
    "elevation angle [rad]"
    elevation::SimFloat
    "azimuth angle [rad]"
    azimuth::SimFloat
    "heading angle [rad]"
    heading::SimFloat
    "angular velocity in the elevation direction [rad/s]"
    elevation_vel::SimFloat
    "angular velocity in the azimuth direction [rad/s]"
    azimuth_vel::SimFloat
    "angular velocity about the radial axis [rad/s]"
    turn_rate::SimFloat
    "where the base point lands, ENU [m]; `nothing` = taken from
    `base_transform`"
    base_pos_ENU::Union{KVec3, Nothing}
end

"""
    Transform(name, elevation, azimuth, heading; base_point, base_pos,
              base_transform, wing, rot_point, elevation_vel, azimuth_vel,
              turn_rate)

Place a group of components at spherical coordinates `elevation`, `azimuth` and
`heading` [rad].

The base is either `base_pos` with `base_point` — the ENU position [m] a named
point lands at — or `base_transform`, chaining onto another transform's
position. The object rotated to (elevation, azimuth) is either a `wing` or a
`rot_point`, one of the two.
"""
function Transform(name, elevation, azimuth, heading;
    base_point=nothing, base_pos=nothing, base_transform=nothing,
    wing=nothing, rot_point=nothing,
    elevation_vel=0.0, azimuth_vel=0.0, turn_rate=0.0
)
    (isnothing(wing) == isnothing(rot_point)) &&
        error("Transform $name: provide a `wing` or a `rot_point`, not both " *
              "or neither.")
    (isnothing(base_pos) == isnothing(base_transform)) &&
        error("Transform $name: provide a `base_pos` or a `base_transform`, " *
              "not both or neither.")
    (!isnothing(base_pos) && isnothing(base_point)) &&
        error("Transform $name: a `base_pos` also needs a `base_point`.")
    Transform(0, name, nothing, opt_name_ref(wing),
        nothing, opt_name_ref(rot_point), nothing, opt_name_ref(base_point),
        nothing, opt_name_ref(base_transform),
        SimFloat(elevation), SimFloat(azimuth), SimFloat(heading),
        SimFloat(elevation_vel), SimFloat(azimuth_vel), SimFloat(turn_rate),
        isnothing(base_pos) ? nothing : KVec3(base_pos...))
end

# ==================== REFERENCE HELPERS ==================== #

"""
    name_ref(reference) -> NameRef

A reference as stored: an integer stays a position, anything else becomes a
`Symbol`.
"""
name_ref(reference::Integer) = Int(reference)
name_ref(reference) = Symbol(reference)

"""
    name_refs(references) -> Vector{NameRef}

[`name_ref`](@ref) over a collection.
"""
name_refs(references) = Vector{NameRef}([name_ref(r) for r in references])

"""
    opt_name_ref(reference) -> Union{NameRef, Nothing}

[`name_ref`](@ref) that passes `nothing` through, for an optional reference.
"""
opt_name_ref(::Nothing) = nothing
opt_name_ref(reference) = name_ref(reference)

"""
    opt_simfloat(value) -> Union{SimFloat, Nothing}

`value` as a `SimFloat`, passing `nothing` through.
"""
opt_simfloat(::Nothing) = nothing
opt_simfloat(value) = SimFloat(value)
