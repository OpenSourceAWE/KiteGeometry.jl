# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""The dynamics types a wing's model names in its `dynamics_type` column."""
const WING_DYNAMICS = ("PARTICLE_DYNAMICS", "RIGID_DYNAMICS")

"""
    WingSpec

What the model of a wing row says frames and weighs its body: whether it is a particle
wing, its origin and its frame as point indices, and the centre [m] and per-unit-mass
inertia [m²] of its mesh in the world, each `nothing` where the model leaves it out.
"""
struct WingSpec
    particle::Bool
    origin::Union{Nothing, RefPoints}
    frame::Union{Nothing, WingFrame}
    com::Union{Nothing, Vec3}
    unit_inertia::Union{Nothing, Mat3}
end

"""The cell `column` of the model of `component`, `nothing` where the model has no such
column or leaves it unset."""
function model_cell(component, column)
    model = component.model
    hasproperty(model, column) || return nothing
    value = getproperty(model, column)
    return yaml_unset(value) ? nothing : value
end

"""
    wing_spec(system, wing) -> WingSpec

The [`WingSpec`](@ref) the model of wing `wing` of `system` gives in its columns
`dynamics_type`, rigid where unset, `origin_idx`, `z_ref_points`, `y_ref_points`, `com` and
`unit_inertia`, written as `load_authoring` reads them, with points as names or indices. Refuses an unknown dynamics type and
a particle wing without an origin.
"""
function wing_spec(system, wing)
    row = system.wings[wing]
    dynamics = something(model_cell(row, :dynamics_type), "RIGID_DYNAMICS")
    dynamics in WING_DYNAMICS || error(
        "wing $(row.name): dynamics_type $dynamics is not $(join(WING_DYNAMICS, " or "))")
    particle = dynamics == "PARTICLE_DYNAMICS"
    origin_cell = model_cell(row, :origin_idx)
    particle && isnothing(origin_cell) &&
        error("particle wing $(row.name) needs an `origin_idx`")
    point_names = indices_of(system, :points)
    origin = isnothing(origin_cell) ? nothing :
             resolve_ref_points(yaml_ref_points(origin_cell), point_names)
    z_ref_points, y_ref_points = (ref_point_pair(row, field, point_names)
                                  for field in (:z_ref_points, :y_ref_points))
    frame = any(isnothing, (origin, z_ref_points, y_ref_points)) ? nothing :
            WingFrame(origin, z_ref_points, y_ref_points)
    com = model_cell(row, :com)
    unit_inertia = model_cell(row, :unit_inertia)
    return WingSpec(particle, origin, frame, isnothing(com) ? nothing : Vec3(com...),
                    isnothing(unit_inertia) ? nothing : unit_inertia_matrix(unit_inertia))
end

"""The pair of reference points the model of the wing `row` gives in `field`, as point
indices among `point_names`, or `nothing`."""
function ref_point_pair(row, field, point_names)
    value = model_cell(row, field)
    isnothing(value) && return nothing
    length(value) == 2 || error("wing $(row.name): $field must hold two reference points")
    return Tuple(resolve_ref_points(yaml_ref_points(refs), point_names) for refs in value)
end

"""The 3×3 per-unit-mass inertia [m²] given as a matrix, its rows, or
`[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`."""
function unit_inertia_matrix(value)
    value isa AbstractMatrix && return Mat3(value)
    first(value) isa AbstractVector && return to_field(Mat3, value)
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = value
    return Mat3(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz)
end

"""The reference points of the origin of the body of wing `wing` of `system`, `nothing`
where that origin is the centre of its mass."""
function wing_origin(system, wing)
    spec = wing_spec(system, wing)
    return spec.particle || !isnothing(spec.frame) ? spec.origin : nothing
end

"""The frame points of wing `wing` of `system`: the points of its stations and the points
fixed in its body."""
function frame_points(system, wing)
    body = system.wings[wing].body
    on_station = falses(length(system.points))
    for station in system.spanwise_stations
        station.wing == wing && (on_station[Int.(station.points)] .= true)
    end
    return [index for (index, point) in enumerate(system.points)
            if on_station[index] || !isnothing(body) && point.body == body]
end

"""The mass [kg] a point or body carries of its own."""
own_mass(component) = something(component.extra_mass, 0.0)

"""
    complete_wings!(system::SystemDefinition, set::Settings) -> system

Give each wing of `system` the kite's mass `set.mass` where it carries none
([`KiteGeometry.wing_mass!`](@ref)), and put its body in its design pose from where its
points stand ([`KiteGeometry.wing_body_pose`](@ref)). A wing's frame points are its
stations' points and the points fixed in its body; what frames and weighs it is read from
its model ([`KiteGeometry.wing_spec`](@ref)). Call it before `place!`.
"""
function complete_wings!(system::SystemDefinition, set::Settings)
    for wing in eachindex(system.wings)
        spec = wing_spec(system, wing)
        members = frame_points(system, wing)
        wing_mass!(system, wing, spec, members, set)
        body_index = system.wings[wing].body
        (isempty(members) || isnothing(body_index)) && continue
        body = system.bodies[body_index]
        origin, R, com_offset, inertia = wing_body_pose(system.points, spec, members,
                                                        own_mass(body))
        body.pos_ENU .= origin + R * com_offset
        body.Q_KA2ENU .= rotation_matrix_to_quaternion(R)
        isnothing(inertia) || (body.extra_inertia_KA = inertia)
    end
    return system
end

"""
    wing_mass!(system, wing, spec, members, set)

Spread `set.mass` equally over the frame points `members` of wing `wing` where the wing
carries no mass of its own: for a rigid wing, neither its body nor its members; for a
particle wing, neither its members nor the bodies they and its stations ride
([`KiteGeometry.particle_wing_parts`](@ref)).
"""
function wing_mass!(system, wing, spec, members, set)
    if spec.particle
        point_idxs, body_idxs = particle_wing_parts(system, wing, members)
        mass = sum(own_mass(system.points[index]) for index in point_idxs; init=0.0) +
               sum(own_mass(system.bodies[index]) for index in body_idxs; init=0.0)
        mass > 0 && return nothing
        if set.mass <= 0
            @warn "Wing $(system.wings[wing].name) (PARTICLE_DYNAMICS) has zero mass."
            return nothing
        end
    else
        body = system.wings[wing].body
        body_mass = isnothing(body) ? 0.0 : own_mass(system.bodies[body])
        iszero(body_mass) && set.mass > 0 &&
            all(index -> iszero(own_mass(system.points[index])), members) || return nothing
    end
    foreach(index -> system.points[index].extra_mass = set.mass / length(members), members)
    return nothing
end

"""
    particle_wing_parts(system, wing, members) -> (point_idxs, body_idxs)

The points of particle wing `wing` — its stations' points, else its frame points `members`
— and the other bodies sharing a tube-connected group with the bodies those points ride.
"""
function particle_wing_parts(system, wing, members)
    point_idxs = Set{Int}()
    for station in system.spanwise_stations
        station.wing == wing && union!(point_idxs, station.points)
    end
    isempty(point_idxs) && union!(point_idxs, members)
    root = connected_body_groups(length(system.bodies), system.tubes)
    wing_bodies = Set(row.body for row in system.wings)
    seeds = Set(root[body] for body in (system.points[index].body for index in point_idxs)
                if !isnothing(body) && !(body in wing_bodies))
    body_idxs = [index for index in eachindex(system.bodies)
                 if !(index in wing_bodies) && root[index] in seeds]
    return point_idxs, body_idxs
end

"""
    wing_body_pose(points, spec, members, mass) -> (origin, R, com_offset, inertia)

The design pose of the body of a wing `spec` describes, of own mass `mass` [kg]: its origin
[m], its rotation into the world, the centre of its own mass in its frame [m], and its own
inertia about that centre in its axes [kg*m²], `nothing` for none. A particle wing's body
sits at its origin, framed by its reference points where it has them; a rigid wing's is
framed by its reference points, else at the centre of its own mass
([`KiteGeometry.wing_inertia`](@ref)) in the world's axes.
"""
function wing_body_pose(points, spec, members, mass)
    pos = [point.pos_ENU for point in points]
    if spec.particle
        R = isnothing(spec.frame) ? Mat3(I) : first(wing_frame(pos, spec.frame))
        return ref_position(pos, spec.origin), R, zero(Vec3), nothing
    end
    com, inertia = wing_inertia(points, members, spec)
    R, origin = isnothing(spec.frame) ? (Mat3(I), com) : wing_frame(pos, spec.frame)
    own_inertia = isnothing(inertia) ? nothing : R' * (mass * inertia) * R
    return origin, R, R' * (com - origin), own_inertia
end

"""
    wing_inertia(points, members, spec) -> (com, inertia)

Centre [m] and per-unit-mass inertia [m²] about it, in world axes, of a rigid wing's own
mass: the `com` and `unit_inertia` of its `spec`, else those of its frame points `members`
as point masses, `inertia` `nothing` where they carry no mass and `com` their centroid.
"""
function wing_inertia(points, members, spec)
    isnothing(spec.com) || isnothing(spec.unit_inertia) ||
        return spec.com, spec.unit_inertia
    masses = [own_mass(points[index]) for index in members]
    positions = [points[index].pos_ENU for index in members]
    total = sum(masses)
    total > 0 || return sum(positions) / length(positions), nothing
    com = sum(masses .* positions) / total
    inertia = sum(mass * (dot(r, r) * I - r * r')
                  for (mass, r) in zip(masses, (position - com for position in positions)))
    return com, Mat3(inertia / total)
end
