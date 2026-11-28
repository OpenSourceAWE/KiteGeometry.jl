# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    WingSpec(; particle=false, origin=nothing, z_ref_points=nothing, y_ref_points=nothing,
             com=nothing, unit_inertia=nothing)

What frames and weighs a wing's body: whether it is a particle wing, the reference points
of its origin and the two pairs of its z and y axes, and the centre [m] and per-unit-mass
inertia [m²] of its mesh in the world, each `nothing` where left out. Reference points are
point indices: one, a list averaged equally, or `[index, weight]` pairs or
`index => weight`. Refuses a particle wing without an origin.
"""
struct WingSpec
    particle::Bool
    origin::Union{Nothing, RefPoints}
    frame::Union{Nothing, WingFrame}
    com::Union{Nothing, Vec3}
    unit_inertia::Union{Nothing, Mat3}
end

function WingSpec(; particle=false, origin=nothing, z_ref_points=nothing,
                  y_ref_points=nothing, com=nothing, unit_inertia=nothing)
    particle && isnothing(origin) && throw(ArgumentError("a particle wing needs an origin"))
    origin, z_ref_points, y_ref_points = (point_weights(origin),
                                          ref_point_pair(z_ref_points),
                                          ref_point_pair(y_ref_points))
    frame = any(isnothing, (origin, z_ref_points, y_ref_points)) ? nothing :
            WingFrame(origin, z_ref_points, y_ref_points)
    return WingSpec(particle, origin, frame, isnothing(com) ? nothing : Vec3(com),
                    isnothing(unit_inertia) ? nothing : Mat3(unit_inertia))
end

"""
    wing_spec(model) -> Union{WingSpec, Nothing}

The [`WingSpec`](@ref) of a wing whose row carries `model`, with its reference columns as
point indices ([`model_references`](@ref)). A tool's wing model adds a method; without
one a wing has nothing to complete.
"""
wing_spec(::AbstractModel) = nothing

"""The reference points `value` as point indices and weights, `nothing` for `nothing`."""
function point_weights(value)
    isnothing(value) && return nothing
    weights = yaml_ref_points(value)
    all(((ref, _),) -> ref isa Int, weights) ||
        throw(ArgumentError("reference points $value are not point indices"))
    return RefPoints(weights)
end

"""The two reference points of the list `value`, `nothing` for `nothing`."""
function ref_point_pair(value)
    isnothing(value) && return nothing
    length(value) == 2 ||
        throw(ArgumentError("$value must hold two reference points, not $(length(value))"))
    return Tuple(point_weights(refs) for refs in value)
end

"""The reference points of the origin of the body of wing `wing` of `system`, `nothing`
where that origin is the centre of its mass or the wing has nothing to complete."""
function wing_origin(system, wing)
    spec = wing_spec(system.wings[wing].model)
    isnothing(spec) && return nothing
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

Put the body of each wing of `system` in its design pose from where its points stand
([`KiteGeometry.wing_body_pose`](@ref)), weighing its points as if they carried their shares
of the kite's mass `set.mass` ([`wing_mass_shares`](@ref)) without writing those into
them. A wing's frame points are its stations' points and the points fixed in its body;
what frames and weighs it is what its model gives ([`wing_spec`](@ref)), and a wing whose
model gives nothing is left as it is. A wing with no frame points keeps its body's pose
and gets the inertia its unit inertia gives its body's mass.
A particle wing's body carries no mass of its own, warning where it had some. Call it
before `place!`.
"""
function complete_wings!(system::SystemDefinition, set::Settings)
    for (wing, row) in enumerate(system.wings)
        spec = wing_spec(row.model)
        isnothing(spec) && continue
        members = frame_points(system, wing)
        shares = wing_mass_shares(system, wing, set.mass)
        isnothing(row.body) && continue
        body = system.bodies[row.body]
        if spec.particle && own_mass(body) > 0
            @warn "Particle wing $(row.name): extra_mass $(body.extra_mass) is ignored; " *
                  "a particle wing carries its mass on its points and section bodies."
            body.extra_mass = 0.0
        end
        if isempty(members)
            R = quaternion_to_rotation_matrix(body.Q_KA2ENU)
            isnothing(spec.unit_inertia) ||
                (body.extra_inertia_KA = body_inertia(R, own_mass(body), spec.unit_inertia))
            continue
        end
        masses = [get(shares, index, own_mass(system.points[index])) for index in members]
        origin, R, com_offset, inertia = wing_body_pose(system.points, spec, members,
                                                        masses, own_mass(body))
        body.pos_ENU .= origin + R * com_offset
        body.Q_KA2ENU .= rotation_matrix_to_quaternion(R)
        isnothing(inertia) || (body.extra_inertia_KA = inertia)
    end
    return system
end

"""
    wing_mass_shares(system::SystemDefinition, wing, mass) -> Dict{Int, Float64}

The mass [kg] each point of wing `wing` (an index) of `system` takes of a wing mass `mass`
[kg]: an equal share for each of its frame points where the wing carries no mass of its
own, none where it does or where its model gives no [`wing_spec`](@ref). A rigid wing
carries its own where its body or a frame point does, a particle wing where any of its
parts does ([`KiteGeometry.particle_wing_parts`](@ref)).
"""
function wing_mass_shares(system::SystemDefinition, wing, mass)
    no_shares = Dict{Int, Float64}()
    spec = wing_spec(system.wings[wing].model)
    isnothing(spec) && return no_shares
    members = frame_points(system, wing)
    if spec.particle
        point_idxs, body_idxs = particle_wing_parts(system, wing)
        own = sum(own_mass(system.points[index]) for index in point_idxs; init=0.0) +
              sum(own_mass(system.bodies[index]) for index in body_idxs; init=0.0)
        own > 0 && return no_shares
        if mass <= 0
            @warn "Particle wing $(system.wings[wing].name) has zero mass."
            return no_shares
        end
    else
        body = system.wings[wing].body
        body_mass = isnothing(body) ? 0.0 : own_mass(system.bodies[body])
        iszero(body_mass) && mass > 0 &&
            all(index -> iszero(own_mass(system.points[index])), members) ||
            return no_shares
    end
    return Dict(index => mass / length(members) for index in members)
end

"""
    particle_wing_parts(system, wing) -> (point_idxs, body_idxs)

The parts that carry the mass of particle wing `wing` of `system`: its stations' points,
else its frame points, and the bodies of no wing that tubes connect to a body, not a wing's,
that those points ride or its stations' models name ([`model_references`](@ref)).
"""
function particle_wing_parts(system, wing)
    point_idxs = Set{Int}()
    seed_bodies = Set{Int}()
    for station in system.spanwise_stations
        station.wing == wing || continue
        union!(point_idxs, station.points)
        model = station.model
        for (column, reference) in pairs(model_references(typeof(model)))
            cell = getfield(model, column)
            union!(seed_bodies, referenced_rows(cell, reference, :bodies))
        end
    end
    isempty(point_idxs) && union!(point_idxs, frame_points(system, wing))
    for index in point_idxs
        body = system.points[index].body
        isnothing(body) || push!(seed_bodies, body)
    end
    root = connected_body_groups(length(system.bodies), system.tubes)
    wing_bodies = Set(row.body for row in system.wings)
    seeds = Set(root[body] for body in setdiff(seed_bodies, wing_bodies))
    body_idxs = [index for index in eachindex(system.bodies)
                 if !(index in wing_bodies) && root[index] in seeds]
    return point_idxs, body_idxs
end

"""
    wing_body_pose(points, spec, members, masses, mass) -> (origin, R, com_offset, inertia)

The design pose of the body of a wing `spec` describes, of own mass `mass` [kg] and with
its frame points `members` weighing `masses` [kg]: its origin
[m], its rotation into the world, the centre of its own mass in its frame [m], and its own
inertia about that centre in its axes [kg*m²], `nothing` for none. A particle wing's body
sits at its origin, framed by its reference points where it has them; a rigid wing's is
framed by its reference points, else at the centre of its own mass
([`KiteGeometry.wing_inertia`](@ref)) in the world's axes.
"""
function wing_body_pose(points, spec, members, masses, mass)
    pos = [point.pos_ENU for point in points]
    if spec.particle
        R = isnothing(spec.frame) ? Mat3(I) : first(wing_frame(pos, spec.frame))
        return ref_position(pos, spec.origin), R, zero(Vec3), nothing
    end
    com, inertia = wing_inertia(points, members, masses, spec)
    R, origin = isnothing(spec.frame) ? (Mat3(I), com) : wing_frame(pos, spec.frame)
    own_inertia = isnothing(inertia) ? nothing : body_inertia(R, mass, inertia)
    return origin, R, R' * (com - origin), own_inertia
end

"""The inertia [kg*m²] in the axes of a body turned by `R` of mass `mass` [kg] with
per-unit-mass inertia `unit_inertia` [m²] in world axes."""
body_inertia(R, mass, unit_inertia) = R' * (mass * unit_inertia) * R

"""
    wing_inertia(points, members, masses, spec) -> (com, inertia)

Centre [m] and per-unit-mass inertia [m²] about it, in world axes, of a rigid wing's own
mass: each the one its `spec` gives, else that of its frame points `members` as point
masses `masses` [kg], the centre their centroid and `inertia` `nothing` where they weigh
nothing.
"""
function wing_inertia(points, members, masses, spec)
    positions = [points[index].pos_ENU for index in members]
    total = sum(masses)
    com = something(spec.com, total > 0 ? sum(masses .* positions) / total :
                                          sum(positions) / length(positions))
    isnothing(spec.unit_inertia) && total > 0 || return com, spec.unit_inertia
    inertia = sum(mass * (dot(r, r) * I - r * r')
                  for (mass, r) in zip(masses, (position - com for position in positions)))
    return com, Mat3(inertia / total)
end
