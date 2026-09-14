# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    struct SystemDefinition

One airborne wind energy system: its topology, its CAD geometry and its
material, and nothing that a simulation of it produces.

Each collection mirrors one table of the awesIO structure document, so a system
that can be written can be read by anything that reads the schema. Every
collection is a type parameter, so a system whose segments all share one force
law — or whose joints all share one stiffness model — is concretely typed.

# Components
- [`Point`](@ref): point masses.
- [`TwistSurface`](@ref): the chordwise pieces of a wing that share a twist.
- [`Segment`](@ref): spring-damper elements.
- [`Pulley`](@ref): sheaves that redistribute line length.
- [`Tether`](@ref): lines of segments, reeled by a winch.
- [`Winch`](@ref): the drums that reel them.
- [`Body`](@ref): rigid bodies; one carrying aerodynamics is a wing, and
  `wings` holds the subset of `bodies` that does.
- [`Joint`](@ref): two-body elastic links; a chain of beam joints is a beam.
- [`Transform`](@ref): the maps from CAD geometry to the ENU world frame.

$(TYPEDFIELDS)
"""
struct SystemDefinition{S, T, W, J}
    "name of the system"
    name::String
    "the point masses"
    points::NamedCollection{Point}
    "the chordwise pieces of a wing that share a twist"
    twist_surfaces::NamedCollection{TwistSurface}
    "the spring-damper elements"
    segments::NamedCollection{S}
    "the sheaves"
    pulleys::NamedCollection{Pulley}
    "the lines of segments"
    tethers::NamedCollection{T}
    "the drums"
    winches::NamedCollection{W}
    "every rigid body, the wings first"
    bodies::NamedCollection{Body}
    "the bodies carrying aerodynamics, the same objects as in `bodies`"
    wings::NamedCollection{Body}
    "the two-body elastic links"
    joints::NamedCollection{J}
    "the maps from CAD geometry to ENU"
    transforms::NamedCollection{Transform}
    "the settings the material defaults were taken from"
    set::Settings
end

"""
    SystemDefinition(name, set; points, twist_surfaces, segments, pulleys,
                     tethers, winches, wings, bodies, joints, transforms)

Assemble a system from its components: generate the points and segments of any
tether written as a start point, an end point and a segment count, assign every
component its position, and resolve every name reference to an index.

Segment rest lengths left at zero are taken from the CAD geometry, densities
left at `NaN` from `set`, and a body-anchored point's `anchor_b` from its
`pos_cad` where it is zero.
"""
function SystemDefinition(name, set::Settings;
    points=Point[], twist_surfaces=TwistSurface[], segments=Segment[],
    pulleys=Pulley[], tethers=Tether[], winches=Winch[], wings=Body[],
    bodies=Body[], joints=Joint[], transforms=Transform[]
)
    expand_auto_tethers!(points, segments, tethers)
    segments, tethers, winches, joints =
        narrow(segments), narrow(tethers), narrow(winches), narrow(joints)
    # Wings come first so that a wing's position in `bodies` is its own index.
    bodies = vcat(wings, bodies)

    assign_indices!(points, twist_surfaces, segments, pulleys, tethers,
                    winches, bodies, joints, transforms)
    point_names = build_name_dict(points)
    segment_names = build_name_dict(segments)
    tether_names = build_name_dict(tethers)
    twist_surface_names = build_name_dict(twist_surfaces)
    body_names = build_name_dict(bodies)
    wing_names = build_name_dict(wings)
    transform_names = build_name_dict(transforms)
    joint_names = build_name_dict(joints)

    for point in points
        point.wing_idx = resolve_ref(point.wing_ref, wing_names, "wing")
        point.transform_idx =
            resolve_ref(point.transform_ref, transform_names, "transform")
        point.body_idx = resolve_ref(point.body_ref, body_names, "body")
        point.joint_idx = resolve_ref(point.joint_ref, joint_names, "joint")
    end
    for twist_surface in twist_surfaces
        twist_surface.point_idxs =
            resolve_refs(twist_surface.point_refs, point_names, "point")
        twist_surface.wing_idx =
            resolve_ref(twist_surface.wing_ref, body_names, "body")
        twist_surface.body_idxs =
            resolve_refs(twist_surface.body_refs, body_names, "body")
        twist_surface.flap_body_idxs =
            resolve_refs(twist_surface.flap_body_refs, body_names, "body")
    end
    mark_wing_nodes!(points, twist_surfaces)
    for segment in segments
        segment.point_idxs = (
            resolve_ref(segment.point_refs[1], point_names, "point"),
            resolve_ref(segment.point_refs[2], point_names, "point"))
        iszero(segment.l0) && (segment.l0 = segment_cad_length(segment, points))
        isnan(segment.density) && (segment.density = set.rho_tether)
    end
    for pulley in pulleys
        pulley.segment_idxs = (
            resolve_ref(pulley.segment_refs[1], segment_names, "segment"),
            resolve_ref(pulley.segment_refs[2], segment_names, "segment"))
    end
    for tether in tethers
        tether.segment_idxs =
            resolve_refs(tether.segment_refs, segment_names, "segment")
        resolve_tether_endpoints!(tether, segments, point_names)
    end
    for winch in winches
        winch.tether_idxs =
            resolve_refs(winch.tether_refs, tether_names, "tether")
        winch.winch_point_idx =
            resolve_ref(winch.winch_point_ref, point_names, "point")
    end
    for transform in transforms
        transform.wing_idx =
            opt_resolve_ref(transform.wing_ref, wing_names, "wing")
        transform.rot_point_idx =
            opt_resolve_ref(transform.rot_point_ref, point_names, "point")
        transform.base_point_idx =
            opt_resolve_ref(transform.base_point_ref, point_names, "point")
        transform.base_transform_idx = opt_resolve_ref(
            transform.base_transform_ref, transform_names, "transform")
    end
    for body in bodies
        body.transform_idx =
            resolve_ref(body.transform_ref, transform_names, "transform")
        body.wing_idx = resolve_ref(body.wing_ref, wing_names, "wing")
        body.twist_surface_idxs = resolve_refs(
            body.twist_surface_refs, twist_surface_names, "twist_surface")
        resolve_body_frame_refs!(body, point_names)
    end
    for joint in joints
        joint.body_a_idx = resolve_ref(joint.body_a_ref, body_names, "body")
        joint.body_b_idx = resolve_ref(joint.body_b_ref, body_names, "body")
    end
    for point in points
        anchor_point!(point, bodies, joints)
    end

    return SystemDefinition(String(name),
        NamedCollection(points, point_names),
        NamedCollection(twist_surfaces, twist_surface_names),
        NamedCollection(segments, segment_names),
        NamedCollection(pulleys, build_name_dict(pulleys)),
        NamedCollection(tethers, tether_names),
        NamedCollection(winches, build_name_dict(winches)),
        NamedCollection(bodies, body_names),
        NamedCollection(wings, wing_names),
        NamedCollection(joints, joint_names),
        NamedCollection(transforms, transform_names),
        set)
end

"""
    narrow(components) -> Vector

`components` re-typed to its elements' common concrete type where they have
one. It is what makes a collection of segments or joints sharing one model
concrete, and so free of the type instability an abstract model field would
cost every step that reads it.
"""
function narrow(components::Vector)
    isempty(components) && return components
    element_type = typeof(first(components))
    all(component -> typeof(component) === element_type, components) || return components
    return convert(Vector{element_type}, components)
end

"""
    assign_indices!(collections...)

Give every component the position it holds in its collection.
"""
function assign_indices!(collections...)
    for collection in collections, (i, component) in enumerate(collection)
        component.idx = i
    end
end

"""
    resolve_ref(ref, name_dict, component_type) -> Int64

The index a reference names: an integer is already one, a symbol is looked up.
Unknown names are an error; `nothing` and `0` mean "no reference" and stay 0.
"""
resolve_ref(ref::Int, ::Dict{Symbol, Int64}, ::AbstractString) = Int64(ref)
resolve_ref(::Nothing, ::Dict{Symbol, Int64}, ::AbstractString) = Int64(0)

function resolve_ref(ref::Symbol, name_dict::Dict{Symbol, Int64},
                     component_type::AbstractString)
    haskey(name_dict, ref) || error("Unknown $component_type name: $ref")
    return name_dict[ref]
end

"""
    resolve_refs(refs, name_dict, component_type) -> Vector{Int64}

[`resolve_ref`](@ref) over a collection of references.
"""
resolve_refs(refs, name_dict, component_type) =
    Int64[resolve_ref(ref, name_dict, component_type) for ref in refs]

"""
    opt_resolve_ref(ref, name_dict, component_type) -> Union{Int64, Nothing}

[`resolve_ref`](@ref) for a field whose "unset" is `nothing` rather than 0.
"""
opt_resolve_ref(::Nothing, ::Dict{Symbol, Int64}, ::AbstractString) = nothing
opt_resolve_ref(ref, name_dict, component_type) =
    resolve_ref(ref, name_dict, component_type)

"""
    resolve!(ref_points::WeightedRefPoints, name_dict, component_type)

Fill `ref_points.ids` from its raw references. A no-op once resolved.
"""
function resolve!(ref_points::WeightedRefPoints,
                  name_dict::Dict{Symbol, Int64},
                  component_type::AbstractString)
    isempty(ref_points.refs) && return
    ref_points.ids = resolve_refs(ref_points.refs, name_dict, component_type)
end

"""
    resolve_body_frame_refs!(body, point_names)

Resolve the points that define a body's frame: its origin and the two pairs
spanning its z and y axes.
"""
function resolve_body_frame_refs!(body::Body, point_names)
    isnothing(body.origin) || resolve!(body.origin, point_names, "point")
    for pair in (body.z_ref_points, body.y_ref_points)
        isnothing(pair) && continue
        resolve!(pair[1], point_names, "point")
        resolve!(pair[2], point_names, "point")
    end
end

"""
    mark_wing_nodes!(points, twist_surfaces)

Set `point.is_wing_node` for every point belonging to a twist surface. A wing's
structural points carry their wing membership through twist-surface membership,
and this flag is what the per-point aerodynamics and the wing-frame fit read.
Needs `twist_surface.point_idxs` resolved first.
"""
function mark_wing_nodes!(points, twist_surfaces)
    members = Set{Int64}()
    for twist_surface in twist_surfaces
        union!(members, twist_surface.point_idxs)
    end
    for point in points
        point.is_wing_node = point.idx in members
    end
end

"""
    segment_cad_length(segment, points) -> SimFloat

Distance between a segment's endpoints as the CAD geometry places them [m].
"""
segment_cad_length(segment, points) =
    norm(points[segment.point_idxs[1]].pos_cad -
         points[segment.point_idxs[2]].pos_cad)

"""
    resolve_tether_endpoints!(tether, segments, point_names)

Resolve a tether's start and end points, reading them off the first and last
segment of the chain when the tether names neither.
"""
function resolve_tether_endpoints!(tether, segments, point_names)
    if isnothing(tether.start_point_ref)
        isempty(tether.segment_idxs) && return
        tether.start_point_idx =
            segments[tether.segment_idxs[1]].point_idxs[1]
        tether.end_point_idx = segments[tether.segment_idxs[end]].point_idxs[2]
        return
    end
    tether.start_point_idx =
        resolve_ref(tether.start_point_ref, point_names, "point")
    tether.end_point_idx =
        resolve_ref(tether.end_point_ref, point_names, "point")
end

"""
    anchor_point!(point, bodies, joints)

Complete a point that rides a body or a beam. A `BODY_STATIC` point given only
a wing rides that wing's own body; an `anchor_b` left at zero is derived from
the point's CAD position, and a beam rider's position along the element from
the same. A point riding a body whose own frame is fitted from reference
points keeps its zero anchor — there is no frame yet to express it in.
"""
function anchor_point!(point::Point, bodies, joints)
    if point.type == BODY_STATIC && iszero(point.body_idx) &&
            point.wing_idx > 0
        point.body_idx = point.wing_idx
    elseif point.body_idx > 0 && point.wing_idx > 0 &&
            is_wing(bodies[point.body_idx]) && point.body_idx != point.wing_idx
        error("Point $(point.name): `body` and `wing` name different wings " *
            "(body $(point.body_idx) vs wing $(point.wing_idx)); a wing is a " *
            "body, so they must reference the same one.")
    end
    if point.body_idx > 0 && iszero(point.anchor_b)
        body = bodies[point.body_idx]
        has_fitted_frame(body) || (point.anchor_b =
            KVec3(body.R_body_to_cad' * (point.pos_cad - body.pos_cad)))
    end
    point.joint_idx == 0 && return
    derive_beam_anchor!(point, joints[point.joint_idx], bodies)
end

"""
    has_fitted_frame(body) -> Bool

Whether a body's own frame is fitted from reference points rather than placed
by `pos_cad` and `R_body_to_cad`. What fits it is the package that simulates
or draws the definition, so nothing derived from that frame is filled here.
"""
has_fitted_frame(body::Body) =
    !isnothing(body.origin) || !isnothing(body.z_ref_points) ||
    !isnothing(body.y_ref_points)

"""
    derive_beam_anchor!(point, joint, bodies)

Derive a beam-anchored point's `beam_frac` and `beam_offset_b` from its
`pos_cad`, by projecting onto the rest beam line between the joint's two node
anchors: the axial fraction `s ∈ [0, 1]` and the perpendicular remainder in the
rest element frame, so the point keeps that offset as the beam bends.
"""
function derive_beam_anchor!(point::Point, joint, bodies)
    body_a, body_b = bodies[joint.body_a_idx], bodies[joint.body_b_idx]
    node_a = body_a.pos_cad .+ body_a.R_body_to_cad * joint.anchor_a_b
    node_b = body_b.pos_cad .+ body_b.R_body_to_cad * joint.anchor_b_b
    e1, e2, e3, len = beam_element_frame(node_a, node_b, body_a.R_body_to_cad)
    relative = point.pos_cad .- node_a
    s = clamp(dot(relative, e1) / len, 0.0, 1.0)
    point.beam_frac = s
    perpendicular = relative .- (s * len) .* e1
    point.beam_offset_b = KVec3(dot(perpendicular, e1), dot(perpendicular, e2),
                                dot(perpendicular, e3))
end

"""
    beam_element_frame(node_a, node_b, R_a) -> (e1, e2, e3, len)

Orthonormal element frame of a beam: `e1` along the chord from node A to node
B, `e2` and `e3` from node A's y axis projected transverse to it, and the chord
length `len` [m].
"""
function beam_element_frame(node_a, node_b, R_a)
    chord = node_b .- node_a
    len = norm(chord)
    e1 = chord ./ len
    y_a = R_a[:, 2]
    e2 = y_a .- dot(y_a, e1) .* e1
    e2 = e2 ./ norm(e2)
    return e1, e2, e1 × e2, len
end

"""
    expand_auto_tethers!(points, segments, tethers)

Generate the intermediate points and segments of every tether written as a
start point, an end point and a segment count. Each generated segment carries
the tether's force law, diameter, density and compression behaviour, and each
generated point inherits the endpoints' transform. Tethers written as explicit
segments, and those whose segments a caller has already created, are left
alone.
"""
function expand_auto_tethers!(points, segments, tethers)
    point_names = build_name_dict(points)
    segment_names = build_name_dict(segments)
    for tether in tethers
        isnothing(tether.start_point_ref) && continue
        haskey(segment_names, Symbol(tether.segment_refs[1])) && continue

        start_idx = resolve_ref(tether.start_point_ref, point_names, "point")
        end_idx = resolve_ref(tether.end_point_ref, point_names, "point")
        start_pos = points[start_idx].pos_cad
        direction = points[end_idx].pos_cad - start_pos
        transform = shared_transform(tether, points[start_idx], points[end_idx])
        n_segments = tether.n_segments
        segment_l0 = something(tether.init_stretched_len, norm(direction)) /
            n_segments

        for i in 1:(n_segments - 1)
            name = Symbol("$(tether.name)_point_$i")
            transform_kw = transform == 0 ? () : (transform=transform,)
            push!(points, Point(name, start_pos + (i / n_segments) * direction,
                                DYNAMIC; transform_kw...))
            point_names[name] = length(points)
        end
        for i in 1:n_segments
            name = Symbol(tether.segment_refs[i])
            start_ref = i == 1 ? tether.start_point_ref :
                Symbol("$(tether.name)_point_$(i - 1)")
            end_ref = i == n_segments ? tether.end_point_ref :
                Symbol("$(tether.name)_point_$i")
            push!(segments, Segment(name, start_ref, end_ref, tether.model;
                l0=segment_l0, diameter=tether.diameter,
                density=tether.density,
                compression_frac=tether.compression_frac,
                compression_damping_frac=tether.compression_damping_frac))
            segment_names[name] = length(segments)
        end
    end
end

"""
    shared_transform(tether, start_point, end_point) -> NameRef

The transform a generated tether point inherits. The two endpoints must not
name different ones; 0 when neither does.
"""
function shared_transform(tether, start_point, end_point)
    start_transform, end_transform =
        start_point.transform_ref, end_point.transform_ref
    (start_transform != 0 && end_transform != 0 &&
        start_transform != end_transform) && error(
        "Tether $(tether.name): start point $(start_point.name) " *
        "(transform $start_transform) and end point $(end_point.name) " *
        "(transform $end_transform) have different transforms.")
    return start_transform != 0 ? start_transform : end_transform
end
