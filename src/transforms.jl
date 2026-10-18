# SPDX-FileCopyrightText: 2025 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    Transform

The rigid motion that takes the points and bodies of a system from the frame they are
authored in to their pose in the world. It turns its object — the body of its `wing`, else
its `rot_point` — to `elevation` and `azimuth` seen from its base, and about that radial to
`heading`. The base is `base_pos`, where `base_point` is moved to, or the object that
`base_transform` turns. References are names, or indices once a `SystemDefinition` holds
the transform.
"""
mutable struct Transform
    "unique name of the transform"
    name::String
    "elevation of the turned object above the base [rad]"
    elevation::Float64
    "azimuth of the turned object around the base, from east towards south [rad]"
    azimuth::Float64
    "heading of the turned object's x axis in its tangential frame [rad]"
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
    base_point::Union{Nothing, NameRef}
    "the transform whose turned object is the base"
    base_transform::Union{Nothing, NameRef}
    "the wing whose body, the one sharing its name, the transform turns"
    wing::Union{Nothing, NameRef}
    "the point the transform turns where it names no wing"
    rot_point::Union{Nothing, NameRef}
    "the points the transform moves"
    points::Vector{NameRef}
    "the bodies the transform moves"
    bodies::Vector{NameRef}
end

"""
    Transform(; name, elevation, azimuth, heading, base_pos=nothing, base_point=nothing,
              base_transform=nothing, wing=nothing, rot_point=nothing, elevation_vel=0.0,
              azimuth_vel=0.0, turn_rate=0.0, points=[], bodies=[])

A transform from its fields, angles in radians. Refuses one turning both or neither of a
`wing` and a `rot_point`, one with both or neither of a `base_pos` and a `base_transform`,
and a `base_pos` without its `base_point`.
"""
function Transform(; name, elevation, azimuth, heading, base_pos=nothing,
                   base_point=nothing, base_transform=nothing, wing=nothing,
                   rot_point=nothing, elevation_vel=0.0, azimuth_vel=0.0, turn_rate=0.0,
                   points=NameRef[], bodies=NameRef[])
    isnothing(wing) == isnothing(rot_point) && throw(ArgumentError(
        "transform $name turns a wing or a rot_point: give one"))
    isnothing(base_pos) == isnothing(base_transform) && throw(ArgumentError(
        "transform $name has a base_pos or a base_transform: give one"))
    !isnothing(base_pos) && isnothing(base_point) && throw(ArgumentError(
        "transform $name: a base_pos needs its base_point"))
    return Transform(name, elevation, azimuth, heading, elevation_vel, azimuth_vel,
                     turn_rate, base_pos, base_point, base_transform, wing, rot_point,
                     points, bodies)
end

"""
    Pose

Where the points and bodies of a system are: the position of each point [m], and the
origin [m] and the rotation into the world of each body's frame, indexed as the system's
blocks. The transform functions move it in place.
"""
struct Pose{P <: AbstractVector, B <: AbstractVector, R <: AbstractVector}
    pos::P
    body_pos::B
    body_R::R
end

Base.copy(pose::Pose) = Pose(copy(pose.pos), copy(pose.body_pos), copy(pose.body_R))

# ==================== ROTATIONS ==================== #

"""Wrap `angle` [rad] into [-π, π]."""
wrap_to_pi(angle) = mod(angle + π, 2π) - π

"""`v` rotated about the axis `k` by `θ` [rad], by Rodrigues' formula."""
function rotate_v_around_k(v, k, θ)
    k = normalize(k)
    return v * cos(θ) + cross(k, v) * sin(θ) + k * dot(k, v) * (1 - cos(θ))
end

"""`R` with each column rotated about the axis `k` by `θ` [rad]."""
rotate_frame(R, k, θ) = Mat3(hcat((rotate_v_around_k(R[:, i], k, θ) for i in 1:3)...))

"""
    calc_R_t_to_w(pos)

Rotation from the tangential frame at `pos` [m] into the world: z radial along `pos`, y
azimuthal and level, x towards elevation, `y × z`.
"""
function calc_R_t_to_w(pos)
    z = normalize(pos)
    y = abs(pos[1]) < 1e-6 && abs(pos[2]) < 1e-6 ? Vec3(0, 1, 0) :
        normalize(Vec3(-pos[2], pos[1], 0))
    return hcat(cross(y, z), y, z)
end

"""
    calc_heading(R_b_to_w, pos)

Heading [rad] of the frame `R_b_to_w` at `pos` [m] from the base: the angle of its x axis
in the tangential plane, from the elevation direction towards the azimuthal one; zero
with the nose towards the base.
"""
function calc_heading(R_b_to_w, pos)
    e_x_t = calc_R_t_to_w(pos)' * R_b_to_w[:, 1]
    return atan(e_x_t[2], e_x_t[1])
end

"""
    min_rotation(curr_dir, target_dir) -> (axis, angle)

Axis and angle [rad] of the smallest rotation taking the unit vector `curr_dir` onto
`target_dir`, which adds no roll about either; about some perpendicular axis where the two
are opposite.
"""
function min_rotation(curr_dir, target_dir)
    cosang = clamp(dot(curr_dir, target_dir), -1.0, 1.0)
    cosang > 1 - 1e-12 && return Vec3(0, 0, 1), 0.0
    if cosang < -1 + 1e-12
        ref = abs(curr_dir[1]) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0)
        return normalize(cross(curr_dir, ref)), Float64(π)
    end
    return normalize(cross(curr_dir, target_dir)), acos(cosang)
end

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

"""The index of the body `transform` turns, the one named as its wing, or `nothing` where
it turns a point."""
function turned_body(system, transform)
    isnothing(transform.wing) && return nothing
    name = system.wings[transform.wing].name
    index = findfirst(body -> body.name == name, system.bodies)
    isnothing(index) && throw(ArgumentError(
        "transform $(transform.name): wing $name has no body of its name"))
    return index
end

"""Position [m] in `pose` of what `transform` turns."""
function turned_position(pose, system, transform)
    body = turned_body(system, transform)
    return isnothing(body) ? pose.pos[transform.rot_point] : pose.body_pos[body]
end

"""Position [m] in `pose` of the base of `transform`: its base point, or what its base
transform turns."""
function base_position(pose, system, transform)
    isnothing(transform.base_transform) && return pose.pos[transform.base_point]
    return turned_position(pose, system, system.transforms[transform.base_transform])
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
    turn_to_radial!(pose, system, transform, base_pos)

Rotate the members of `transform` about `base_pos` [m] by the smallest rotation that puts
what it turns on its [`radial_direction`](@ref). Refuses a turned object at the base, and
warns at the zenith or nadir, where azimuth and heading are undefined.
"""
function turn_to_radial!(pose, system, transform, base_pos)
    rel_pos = turned_position(pose, system, transform) - base_pos
    norm(rel_pos) < 1e-6 && throw(ArgumentError(
        "transform $(transform.name): what it turns is at its base $base_pos, so it has " *
        "no elevation or azimuth"))
    abs(abs(transform.elevation) - π / 2) < 1e-6 &&
        @warn "Transform $(transform.name): elevation " *
              "$(round(rad2deg(transform.elevation); digits=2))° is at the zenith or " *
              "nadir, where azimuth and heading are undefined."
    axis, angle = min_rotation(normalize(rel_pos), radial_direction(transform))
    rotate_members!(pose, transform, base_pos, axis, angle)
    return nothing
end

"""The body whose frame gives the heading of `transform`: the one it turns, else its
first body, else `nothing`."""
function heading_body(system, transform)
    body = turned_body(system, transform)
    isnothing(body) || return body
    return isempty(transform.bodies) ? nothing : first(transform.bodies)
end

"""
    turn_to_heading!(pose, system, transform, base_pos)

Rotate the members of `transform` about the radial through `base_pos` [m] and its
[`heading_body`](@ref) until that body's frame has the transform's `heading`; nothing
where the transform moves no body.
"""
function turn_to_heading!(pose, system, transform, base_pos)
    body = heading_body(system, transform)
    isnothing(body) && return nothing
    rel_pos = pose.body_pos[body] - base_pos
    delta = wrap_to_pi(transform.heading - calc_heading(pose.body_R[body], rel_pos))
    rotate_members!(pose, transform, base_pos, normalize(rel_pos), delta)
    return nothing
end

"""
    place_transforms!(pose::Pose, design::Pose, system)

Apply each of `system.transforms`, in order, to `pose`: shift its members so its base
point is at its `base_pos`, or by how far what its base transform turns has moved from
`design`; then [`turn_to_radial!`](@ref) and [`turn_to_heading!`](@ref) about that base.
"""
function place_transforms!(pose::Pose, design::Pose, system)
    for transform in system.transforms
        base_pos, curr_base_pos = if isnothing(transform.base_transform)
            transform.base_pos, pose.pos[transform.base_point]
        else
            base_position(pose, system, transform), base_position(design, system, transform)
        end
        shift_members!(pose, transform, base_pos - curr_base_pos)
        turn_to_radial!(pose, system, transform, base_pos)
        turn_to_heading!(pose, system, transform, base_pos)
    end
    return pose
end

"""
    reposition!(pose::Pose, system)

Turn the members of each of `system.transforms`, in order, to its current `elevation`,
`azimuth` and `heading` about its base where it stands in `pose`, without moving that
base.
"""
function reposition!(pose::Pose, system)
    for transform in system.transforms
        base_pos = base_position(pose, system, transform)
        turn_to_radial!(pose, system, transform, base_pos)
        turn_to_heading!(pose, system, transform, base_pos)
    end
    return pose
end

"""
    transform_velocities(pose::Pose, system) -> (point_vel, body_vel, body_spin)

The velocity [m/s] of each point and body origin, and the angular velocity [rad/s] of
each body in the world, of the rigid rotation about its base ([`spherical_spin`](@ref))
each transform gives its members in `pose`; zero for what no transform moves.
"""
function transform_velocities(pose::Pose, system)
    point_vel = fill(zero(Vec3), length(pose.pos))
    body_vel = fill(zero(Vec3), length(pose.body_pos))
    body_spin = fill(zero(Vec3), length(pose.body_pos))
    for transform in system.transforms
        base_pos = base_position(pose, system, transform)
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
