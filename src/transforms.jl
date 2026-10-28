# SPDX-FileCopyrightText: 2025 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    Transform

The rigid motion that takes some of a system's points and bodies from the frame they are
authored in to their pose in the world. It turns its object, the weighted `rot_point`
else the position of `body`, to `elevation` and `azimuth` seen from its base, and about
that radial until the frame of `body` has `heading`. The base is `base_pos`, where
`base_point` is moved to, or the object `base_transform` turns. References are indices
into the system's blocks and into the transforms.
"""
mutable struct Transform
    "unique name of the transform"
    name::Symbol
    "elevation of the turned object above the base [rad]"
    elevation::Float64
    "azimuth of the turned object around the base, from east towards south [rad]"
    azimuth::Float64
    "heading of the x axis of `body` in its tangential frame [rad]"
    heading::Float64
    "rate of `elevation` [rad/s]"
    elevation_vel::Float64
    "rate of `azimuth` [rad/s]"
    azimuth_vel::Float64
    "rate of turn about the radial through the turned object [rad/s]"
    turn_rate::Float64
    "the position `base_point` is moved to [m], `nothing` where `base_transform` is set"
    base_pos::Union{Nothing, MVector{3, Float64}}
    "the point moved to `base_pos`"
    base_point::Union{Nothing, Int}
    "the transform whose turned object is the base"
    base_transform::Union{Nothing, Int}
    "the points, weighted, whose position is the turned object, `nothing` for `body`"
    rot_point::Union{Nothing, RefPoints}
    "the body whose frame has `heading`, `nothing` for none"
    body::Union{Nothing, Int}
    "the points the transform moves"
    points::Vector{Int}
    "the bodies the transform moves"
    bodies::Vector{Int}
end

block_name(::Type{Transform}) = :transforms

"""
    Transform(; name, elevation, azimuth, heading, base_pos=nothing, base_point=nothing,
              base_transform=nothing, rot_point=nothing, body=nothing, elevation_vel=0.0,
              azimuth_vel=0.0, turn_rate=0.0, points=Int[], bodies=Int[])

A transform from its fields, angles in radians. Refuses one turning neither a `rot_point`
nor a `body`, one with both or neither of a `base_pos` and a `base_transform`, and a
`base_pos` without its `base_point`.
"""
function Transform(; name, elevation, azimuth, heading, base_pos=nothing,
                   base_point=nothing, base_transform=nothing, rot_point=nothing,
                   body=nothing, elevation_vel=0.0, azimuth_vel=0.0, turn_rate=0.0,
                   points=Int[], bodies=Int[])
    isnothing(rot_point) && isnothing(body) &&
        error("transform $name turns a rot_point or a body: give one")
    isnothing(base_pos) == isnothing(base_transform) &&
        error("transform $name has a base_pos or a base_transform: give one")
    !isnothing(base_pos) && isnothing(base_point) &&
        error("transform $name: a base_pos needs its base_point")
    return Transform(name, elevation, azimuth, heading, elevation_vel, azimuth_vel,
                     turn_rate, base_pos, base_point, base_transform, rot_point, body,
                     points, bodies)
end

"""
    Pose

Where the points and bodies of a system are: the position of each point [m], and a
position [m] fixed in each body and the rotation of its frame into the world, indexed as
the system's blocks. The transform functions move it in place.
"""
struct Pose{P <: AbstractVector, B <: AbstractVector, R <: AbstractVector}
    pos::P
    body_pos::B
    body_R::R
end

Base.copy(pose::Pose) = Pose(copy(pose.pos), copy(pose.body_pos), copy(pose.body_R))

"""
    Pose(system::SystemDefinition)

The pose `system` stands in, each body at the centre of its own mass.
"""
function Pose(system::SystemDefinition)
    return Pose([Vec3(point.pos_ENU) for point in system.points],
                [Vec3(body.pos_ENU) for body in system.bodies],
                [quaternion_to_rotation_matrix(body.Q_KA2ENU) for body in system.bodies])
end

"""The `Pose` of `placement`, sharing its vectors."""
Pose(placement::Placement) = Pose(placement.pos, placement.body_pos, placement.body_R)

# ==================== TANGENTIAL FRAME ==================== #

"""
    radial_direction(transform)

Unit vector in the world from a transform's base towards where its `elevation` and
`azimuth` put the turned object: `(cos el cos az, -cos el sin az, sin el)`.
"""
function radial_direction(transform)
    (; elevation, azimuth) = transform
    return Vec3(cos(elevation) * cos(azimuth), -cos(elevation) * sin(azimuth),
                sin(elevation))
end

"""
    spherical_spin(transform)

Angular velocity [rad/s] in the world of the rigid rotation about a transform's base its
`elevation_vel`, `azimuth_vel` and `turn_rate` describe, about the tangential `-y`, the
world's `-z` and [`radial_direction`](@ref).
"""
function spherical_spin(transform)
    (; azimuth, elevation_vel, azimuth_vel, turn_rate) = transform
    return elevation_vel * Vec3(-sin(azimuth), -cos(azimuth), 0) -
           azimuth_vel * Vec3(0, 0, 1) + turn_rate * radial_direction(transform)
end

# ==================== POSES ==================== #

"""Position [m] in `pose` of what `transform` turns."""
function turned_position(pose, transform)
    isnothing(transform.rot_point) && return pose.body_pos[transform.body]
    return ref_position(pose.pos, transform.rot_point)
end

"""Position [m] in `pose` of the base of `transform`, one of `transforms`: its base
point, or what its base transform turns."""
function base_position(pose, transforms, transform)
    isnothing(transform.base_transform) && return pose.pos[transform.base_point]
    return turned_position(pose, transforms[transform.base_transform])
end

"""Move the members of `transform` in `pose` by `shift` [m]."""
function shift_members!(pose, transform, shift)
    foreach(index -> pose.pos[index] += shift, transform.points)
    foreach(index -> pose.body_pos[index] += shift, transform.bodies)
    return nothing
end

"""Rotate the members of `transform` in `pose`, bodies' frames included, about the axis
`k` through `base_pos` [m] by `θ` [rad]."""
function rotate_members!(pose, transform, base_pos, k, θ)
    for index in transform.points
        pose.pos[index] = base_pos + rotate_v_around_k(pose.pos[index] - base_pos, k, θ)
    end
    for index in transform.bodies
        pose.body_pos[index] = base_pos +
                               rotate_v_around_k(pose.body_pos[index] - base_pos, k, θ)
        pose.body_R[index] = rotate_frame(pose.body_R[index], k, θ)
    end
    return nothing
end

"""
    turn!(pose, transform, base_pos)

Rotate the members of `transform` about `base_pos` [m]: by the smallest rotation that puts
what it turns on its [`radial_direction`](@ref), then about that radial until its `body`
has its `heading`. Refuses a turned object at the base, and warns at the zenith or nadir,
where azimuth and heading are undefined.
"""
function turn!(pose, transform, base_pos)
    rel_pos = turned_position(pose, transform) - base_pos
    norm(rel_pos) < 1e-6 && error(
        "transform $(transform.name): what it turns is at its base $base_pos, so it has " *
        "no elevation or azimuth")
    abs(abs(transform.elevation) - π / 2) < 1e-6 &&
        @warn "Transform $(transform.name): elevation " *
              "$(round(rad2deg(transform.elevation); digits=2))° is at the zenith or " *
              "nadir, where azimuth and heading are undefined."
    radial = radial_direction(transform)
    rotate_members!(pose, transform, base_pos, min_rotation(normalize(rel_pos), radial)...)
    isnothing(transform.body) && return nothing
    # KiteUtils' heading turns the other way to a transform's
    heading = -calc_heading(pose.body_R[transform.body], transform.elevation,
                            -transform.azimuth)
    rotate_members!(pose, transform, base_pos, radial,
                    wrap2pi(transform.heading - heading))
    return nothing
end

"""
    place_transforms!(pose::Pose, design::Pose, transforms)

Apply each of `transforms`, in order, to `pose`: shift its members so its base point is
at its `base_pos`, or by how far what its base transform turns has moved from `design`;
then [`turn!`](@ref) them about that base.
"""
function place_transforms!(pose::Pose, design::Pose, transforms)
    for transform in transforms
        base_pos, curr_base_pos = if isnothing(transform.base_transform)
            transform.base_pos, pose.pos[transform.base_point]
        else
            base_position(pose, transforms, transform),
            base_position(design, transforms, transform)
        end
        shift_members!(pose, transform, base_pos - curr_base_pos)
        turn!(pose, transform, base_pos)
    end
    return pose
end

"""
    place!(system::SystemDefinition, transforms)

Apply `transforms` ([`place_transforms!`](@ref)) to `system` where it stands, writing
its points, its bodies and the wings sharing their names in place.
"""
function place!(system::SystemDefinition, transforms)
    design = Pose(system)
    pose = place_transforms!(copy(design), design, transforms)
    carry_wings!(system, design, pose)
    for (point, pos) in zip(system.points, pose.pos)
        point.pos_ENU .= pos
    end
    for (body, pos, R) in zip(system.bodies, pose.body_pos, pose.body_R)
        body.pos_ENU .= pos
        body.Q_KA2ENU .= rotation_matrix_to_quaternion(R)
    end
    return system
end

"""Move the KA frame of each wing of `system` as the body sharing its name moved from
`before` to `after`."""
function carry_wings!(system, before::Pose, after::Pose)
    bodies = name_indices(system.bodies, :bodies)
    for wing in system.wings
        body = get(bodies, wing.name, nothing)
        isnothing(body) && continue
        R = after.body_R[body] * before.body_R[body]'
        wing.pos_ENU .= after.body_pos[body] + R * (wing.pos_ENU - before.body_pos[body])
        wing.Q_KA2ENU .= rotation_matrix_to_quaternion(
            R * quaternion_to_rotation_matrix(wing.Q_KA2ENU))
    end
    return system
end

"""
    reposition!(pose::Pose, transforms)

[`turn!`](@ref) the members of each of `transforms`, in order, to its current
`elevation`, `azimuth` and `heading` about its base where it stands in `pose`, without
moving that base. A chained transform is not carried with its base, as
[`place_transforms!`](@ref) carries it.
"""
function reposition!(pose::Pose, transforms)
    for transform in transforms
        turn!(pose, transform, base_position(pose, transforms, transform))
    end
    return pose
end

"""
    transform_velocities(pose::Pose, transforms) -> (point_vel, body_vel, body_spin)

The velocity [m/s] of each point and body position, and the angular velocity [rad/s] of
each body in the world, of the rigid rotation about its base ([`spherical_spin`](@ref))
each of `transforms` gives its members in `pose`; zero for what none moves. The members of
a chained transform get its own rotation alone, not its base's.
"""
function transform_velocities(pose::Pose, transforms)
    point_vel = fill(zero(Vec3), length(pose.pos))
    body_vel = fill(zero(Vec3), length(pose.body_pos))
    body_spin = fill(zero(Vec3), length(pose.body_pos))
    for transform in transforms
        base_pos = base_position(pose, transforms, transform)
        spin = spherical_spin(transform)
        for index in transform.points
            point_vel[index] = cross(spin, pose.pos[index] - base_pos)
        end
        for index in transform.bodies
            body_vel[index] = cross(spin, pose.body_pos[index] - base_pos)
            body_spin[index] = spin
        end
    end
    return point_vel, body_vel, body_spin
end
