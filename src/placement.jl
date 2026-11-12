# SPDX-FileCopyrightText: 2025 Bart van de Lint, Jelle Poland
# SPDX-License-Identifier: MIT

const Vec3 = SVector{3, Float64}
const Mat3 = SMatrix{3, 3, Float64, 9}

"""Reference points as a weight per point index, the weights summing to one."""
const RefPoints = Vector{Pair{Int, Float64}}

"""
    WingFrame

The points that fix a wing body's frame: its origin, and the two pairs giving its z and its
y axis.
"""
struct WingFrame
    origin::RefPoints
    z_ref_points::NTuple{2, RefPoints}
    y_ref_points::NTuple{2, RefPoints}
end

"""
    TetherInit(; stretched_length=nothing, force=nothing, stretch_frac=nothing)

How a tether starts when it is placed, each optional. All `nothing` leaves the tether as it
stands.
"""
@kwdef struct TetherInit
    "length the tether is stretched to [m]"
    stretched_length::Union{Nothing, Float64} = nothing
    "tension the placed tether carries, which sets its rest length [N]"
    force::Union{Nothing, Float64} = nothing
    "rest length as a fraction of the placed length"
    stretch_frac::Union{Nothing, Float64} = nothing
end

"""
    tether_init(model) -> TetherInit

How a tether whose row carries `model` starts when placed: nothing unless a tool's model
adds a method.
"""
tether_init(::AbstractModel) = TetherInit()

"""Whether `init` gives a stretched length, force or stretch fraction."""
gives_anything(init::TetherInit) =
    !all(isnothing, (init.stretched_length, init.force, init.stretch_frac))

"""
    Placement

A `SystemDefinition` while it is placed: where each point and body is, each segment's rest
length, and what placing reads of it, derived from its rows; the vectors are indexed as its
blocks.
"""
struct Placement
    system::SystemDefinition
    "position of each point [m]"
    pos::Vector{Vec3}
    "rest length of each segment [m]"
    l0::Vector{Float64}
    "position of each body [m]"
    body_pos::Vector{Vec3}
    "rotation of each body's frame into the world"
    body_R::Vector{Mat3}
    "position of each body-carried point in its body's frame [m]"
    anchor::Vector{Vec3}
    "index of the body named after the wing whose station holds each point, 0 for none"
    station_body::Vector{Int}
    "whether each body is held in the world"
    static_body::Vector{Bool}
    tether_inits::Vector{TetherInit}
end

"""
    Placement(system::SystemDefinition, tether_inits)

`system` where it stands. Refuses `tether_inits` that are not one per tether.
"""
function Placement(system::SystemDefinition, tether_inits)
    length(tether_inits) == length(system.tethers) || throw(ArgumentError(
        "$(length(tether_inits)) tether_inits for $(length(system.tethers)) tethers"))
    (; pos, body_pos, body_R) = Pose(system)
    anchor = [isnothing(point.body) ? zero(Vec3) :
              body_R[point.body]' * (pos[index] - body_pos[point.body])
              for (index, point) in enumerate(system.points)]
    bodies = name_indices(system.bodies, :bodies)
    station_body = zeros(Int, length(system.points))
    for station in system.spanwise_stations
        wing_name = system.wings[station.wing].name
        station_body[Int.(station.points)] .= get(bodies, wing_name, 0)
    end
    return Placement(system, pos, [segment.rest_length for segment in system.segments],
                     body_pos, body_R, anchor, station_body,
                     [body.fixed === true for body in system.bodies],
                     collect(TetherInit, tether_inits))
end

# ==================== ROTATIONS ==================== #

"""Rotation matrix of the scalar-first quaternion `q`."""
function quaternion_to_rotation_matrix(q)
    w, x, y, z = q
    s = 2 / (w * w + x * x + y * y + z * z)
    return Mat3(1 - s * (y * y + z * z), s * (x * y + z * w), s * (x * z - y * w),
                s * (x * y - z * w), 1 - s * (x * x + z * z), s * (y * z + x * w),
                s * (x * z + y * w), s * (y * z - x * w), 1 - s * (x * x + y * y))
end

"""Scalar-first quaternion of the rotation matrix `R`, from its numerically stable branch:
the trace where it is positive, else the largest diagonal element."""
function rotation_matrix_to_quaternion(R)
    trace = R[1, 1] + R[2, 2] + R[3, 3]
    if trace > 0
        S = 2 * sqrt(trace + 1)
        return SVector(S / 4, (R[3, 2] - R[2, 3]) / S, (R[1, 3] - R[3, 1]) / S,
                       (R[2, 1] - R[1, 2]) / S)
    elseif R[1, 1] >= R[2, 2] && R[1, 1] >= R[3, 3]
        S = 2 * sqrt(1 + R[1, 1] - R[2, 2] - R[3, 3])
        return SVector((R[3, 2] - R[2, 3]) / S, S / 4, (R[1, 2] + R[2, 1]) / S,
                       (R[1, 3] + R[3, 1]) / S)
    elseif R[2, 2] >= R[3, 3]
        S = 2 * sqrt(1 + R[2, 2] - R[1, 1] - R[3, 3])
        return SVector((R[1, 3] - R[3, 1]) / S, (R[1, 2] + R[2, 1]) / S, S / 4,
                       (R[2, 3] + R[3, 2]) / S)
    end
    S = 2 * sqrt(1 + R[3, 3] - R[1, 1] - R[2, 2])
    return SVector((R[2, 1] - R[1, 2]) / S, (R[1, 3] + R[3, 1]) / S,
                   (R[2, 3] + R[3, 2]) / S, S / 4)
end

"""`v` rotated about the axis `k` by `θ` [rad], by Rodrigues' formula."""
function rotate_v_around_k(v, k, θ)
    k = normalize(k)
    return v * cos(θ) + cross(k, v) * sin(θ) + k * dot(k, v) * (1 - cos(θ))
end

"""`R` with each column rotated about the axis `k` by `θ` [rad]."""
rotate_frame(R, k, θ) = Mat3(hcat((rotate_v_around_k(R[:, i], k, θ) for i in 1:3)...))

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

# ==================== WING FRAMES ==================== #

"""Weighted position of the reference points `refs` among the positions `pos`."""
ref_position(pos, refs) = sum(weight * pos[index] for (index, weight) in refs)

"""Rotation into the world and origin of the wing frame `frame` on the positions `pos`: z
from the first z reference to the second, x across the y references and z, y completing
the right-handed frame."""
function wing_frame(pos, frame::WingFrame)
    z_axis = normalize(ref_position(pos, frame.z_ref_points[2]) -
                       ref_position(pos, frame.z_ref_points[1]))
    y_temp = normalize(ref_position(pos, frame.y_ref_points[2]) -
                       ref_position(pos, frame.y_ref_points[1]))
    x_axis = normalize(cross(y_temp, z_axis))
    return hcat(x_axis, cross(z_axis, x_axis), z_axis), ref_position(pos, frame.origin)
end

# ==================== PLACE ==================== #

"""
    place!(system::SystemDefinition, transforms=Transform[];
           tether_inits=[tether_init(tether.model) for tether in system.tethers],
           ignore_l0=false) -> system

Place `system` from where it stands, in place: each unset rest length to the length it has
there, tethers to their `tether_inits` ([`TetherInit`](@ref)) and the rest lengths those
give, pulley rest lengths shared out, `transforms` applied ([`place_transforms!`](@ref)),
each body's points carried with it, and each wing's KA frame with its body. `ignore_l0`
makes every rest length the placed length.
"""
function place!(system::SystemDefinition, transforms=Transform[];
                tether_inits=[tether_init(tether.model) for tether in system.tethers],
                ignore_l0=false)
    design = Pose(system)
    placement = Placement(system, tether_inits)
    update_segment_lengths!(placement)
    apply_tether_init_stretched_lens!(placement)
    apply_tether_init_forces!(placement)
    init_pulley_lengths!(placement)
    place_transforms!(Pose(placement), design, transforms)
    carry_body_points!(placement)
    ignore_l0 && relax_segments!(placement)
    store_placement!(placement, design)
    return system
end

"""Write where `placement` stands into its system's rows, its wings carried from
`design`."""
function store_placement!(placement::Placement, design)
    (; system) = placement
    carry_wings!(system, design, Pose(placement))
    foreach((point, pos) -> point.pos_ENU .= pos, system.points, placement.pos)
    foreach((segment, l0) -> segment.rest_length = l0, system.segments, placement.l0)
    for (body, pos, R) in zip(system.bodies, placement.body_pos, placement.body_R)
        body.pos_ENU .= pos
        body.Q_KA2ENU .= rotation_matrix_to_quaternion(R)
    end
    return nothing
end

"""Length of `segment` between the positions `pos` [m]."""
segment_length(segment, pos) = norm(pos[segment.points[1]] - pos[segment.points[2]])

"""Set every unset (zero) rest length to the segment's length."""
function update_segment_lengths!(placement::Placement)
    for (i, segment) in enumerate(placement.system.segments)
        placement.l0[i] ≈ 0 && (placement.l0[i] = segment_length(segment, placement.pos))
    end
    return nothing
end

"""Make every rest length the segment's length, so no segment is stretched."""
function relax_segments!(placement::Placement)
    for (i, segment) in enumerate(placement.system.segments)
        placement.l0[i] = segment_length(segment, placement.pos)
    end
    return nothing
end

"""Share each pulley's summed rest length between its two segments in proportion to their
lengths."""
function init_pulley_lengths!(placement::Placement)
    (; system, pos, l0) = placement
    for pulley in system.pulleys
        segment_a, segment_b = pulley.segments
        sum_len = l0[segment_a] + l0[segment_b]
        len_a = segment_length(system.segments[segment_a], pos)
        len_b = segment_length(system.segments[segment_b], pos)
        l0[segment_a] = len_a / (len_a + len_b) * sum_len
        l0[segment_b] = sum_len - l0[segment_a]
    end
    return nothing
end

"""Put every body-carried point at its anchor on its body, where the body now is."""
function carry_body_points!(placement::Placement)
    for (i, point) in enumerate(placement.system.points)
        isnothing(point.body) && continue
        placement.pos[i] = placement.body_pos[point.body] +
                           placement.body_R[point.body] * placement.anchor[i]
    end
    return nothing
end

# ==================== TETHER LENGTHS ==================== #

"""Point indices along `tether`, from its start through each segment's far end."""
function tether_ordered_point_idxs(tether, segments)
    return [tether.start_point; [segments[index].points[2] for index in tether.segments]]
end

"""`(anchor, free)` endpoints of `tether`: the one in `boundary` and the other, or
`(nothing, nothing)` where neither or both are."""
function tether_anchor_free(tether, boundary)
    on_start, on_end = tether.start_point in boundary, tether.end_point in boundary
    on_start && !on_end && return tether.start_point, tether.end_point
    on_end && !on_start && return tether.end_point, tether.start_point
    return nothing, nothing
end

"""Body indices whose placement carries point `index`: the body it rides, both bodies of the
tube it rides, or the wing body of the station it is a member of. Empty for a point that
stands free."""
function anchor_body_idxs(placement::Placement, index)
    point = placement.system.points[index]
    isnothing(point.body) || return (point.body,)
    isnothing(point.tube) || return placement.system.tubes[point.tube].bodies
    station_body = placement.station_body[index]
    return station_body == 0 ? () : (station_body,)
end

"""Each body index mapped to the bodies it shares a tube with."""
function beam_body_neighbors(tubes)
    neighbors = Dict{Int, Vector{Int}}()
    for tube in tubes
        body_a, body_b = tube.bodies
        push!(get!(neighbors, body_a, Int[]), body_b)
        push!(get!(neighbors, body_b, Int[]), body_a)
    end
    return neighbors
end

"""Root of each body's group of bodies joined by tubes, by union-find."""
function connected_body_groups(n_bodies, tubes)
    root = collect(1:n_bodies)
    find(x) = root[x] == x ? x : (root[x] = find(root[x]))
    for tube in tubes
        root[find(tube.bodies[1])] = find(tube.bodies[2])
    end
    return [find(body) for body in 1:n_bodies]
end

"""The bodies that translate with `seeds`: the seeds and every body reached from them over
tubes, stopping at the bodies `static_body` clamps."""
function translated_body_idxs(seeds, static_body, body_neighbors)
    moved = Set{Int}()
    queue = Int[]
    for seed in seeds
        (seed in moved || static_body[seed]) && continue
        push!(moved, seed)
        push!(queue, seed)
    end
    while !isempty(queue)
        for neighbor in get(body_neighbors, pop!(queue), Int[])
            (neighbor in moved || static_body[neighbor]) && continue
            push!(moved, neighbor)
            push!(queue, neighbor)
        end
    end
    return moved
end

"""Each point riding a body, a tube or a wing mapped to all the points riding any body of
its tube-connected group (`root`)."""
function rigid_point_siblings(placement::Placement, root)
    group_members = Dict{Int, Set{Int}}()
    for index in eachindex(placement.pos), body in anchor_body_idxs(placement, index)
        push!(get!(group_members, root[body], Set{Int}()), index)
    end
    siblings = Dict{Int, Set{Int}}()
    for members in values(group_members)
        foreach(member -> siblings[member] = members, members)
    end
    return siblings
end

"""The points reached from `from` through segments outside `tether` and through
`rigid_siblings`, stopping at `boundary`: what moves with the tether's free end. Refuses a
structure that reaches back to `anchor`."""
function tether_downstream_idxs(tether, segments, boundary, from, anchor, rigid_siblings)
    own_segments = Set(tether.segments)
    visited = Set(tether_ordered_point_idxs(tether, segments))
    downstream = Set{Int}()
    queue = [from]
    while !isempty(queue)
        current = pop!(queue)
        neighbors = Int[]
        for (index, segment) in enumerate(segments)
            index in own_segments && continue
            point_a, point_b = segment.points
            point_a == current && push!(neighbors, point_b)
            point_b == current && push!(neighbors, point_a)
        end
        for sibling in get(rigid_siblings, current, ())
            sibling == current || push!(neighbors, sibling)
        end
        for neighbor in neighbors
            neighbor == anchor && error(
                "tether $(tether.name): the structure downstream of it connects back to " *
                "its anchor, so it cannot be placed at its length")
            neighbor in visited && continue
            push!(visited, neighbor)
            neighbor in boundary && continue
            push!(downstream, neighbor)
            push!(queue, neighbor)
        end
    end
    return downstream
end

"""The tether indices `placed` grouped into clusters whose `reach` (points each touches)
overlap, by union-find."""
function tether_clusters(placed, reach)
    parent = collect(eachindex(placed))
    find(i) = parent[i] == i ? i : (parent[i] = find(parent[i]))
    for i in eachindex(placed), j in (i + 1):length(placed)
        isempty(intersect(reach[placed[i]], reach[placed[j]])) && continue
        parent[find(i)] = find(j)
    end
    clusters = Dict{Int, Vector{Int}}()
    for i in eachindex(placed)
        push!(get!(clusters, find(i), Int[]), placed[i])
    end
    return collect(values(clusters))
end

"""Translation along the direction of the mean of the anchor-to-free vectors `spans` that
brings their mean length to `mean_len`, by Newton's method."""
function cluster_standoff_shift(spans, mean_len)
    direction = normalize(sum(spans))
    t = 0.0
    for _ in 1:50
        lens = [norm(span + t * direction) for span in spans]
        residual = sum(lens) / length(lens) - mean_len
        abs(residual) < 1e-12 * mean_len && break
        slope = sum(dot(span + t * direction, direction) / len
                    for (span, len) in zip(spans, lens)) / length(lens)
        t -= residual / slope
    end
    return t * direction
end

"""Move one cluster of tethers to their mean stretched length: the free ends, what is
downstream of them and the bodies those ride translate along the mean tether, and each
tether's inner points are spread along it in proportion to their segment lengths."""
function apply_cluster_init_stretched_len!(placement::Placement, cluster, downstream,
                                           boundary, body_neighbors)
    (; system, pos) = placement
    snaps = map(cluster) do index
        tether = system.tethers[index]
        anchor, free = tether_anchor_free(tether, boundary)
        ordered = tether_ordered_point_idxs(tether, system.segments)
        seg_lens = [segment_length(system.segments[segment], pos)
                    for segment in tether.segments]
        if ordered[1] != anchor
            reverse!(ordered)
            reverse!(seg_lens)
        end
        path_len = sum(seg_lens)
        path_len > 0 || error(
            "tether $(tether.name) has no length to scale to its stretched length")
        (; index, free, anchor_pos=pos[anchor], free_pos=pos[free], ordered, seg_lens,
         path_len)
    end
    stretched = [placement.tether_inits[snap.index].stretched_length for snap in snaps]
    delta = cluster_standoff_shift([snap.free_pos - snap.anchor_pos for snap in snaps],
                                   sum(stretched) / length(stretched))
    norm(delta) ≈ 0 && return nothing

    moved = Set{Int}()
    for snap in snaps, index in [collect(downstream[snap.index]); snap.free]
        index in moved && continue
        push!(moved, index)
        pos[index] += delta
    end
    seeds = Set{Int}()
    for index in moved
        union!(seeds, anchor_body_idxs(placement, index))
    end
    for body in translated_body_idxs(seeds, placement.static_body, body_neighbors)
        placement.body_pos[body] += delta
    end
    for snap in snaps
        line = snap.free_pos + delta - snap.anchor_pos
        along = 0.0
        for k in 2:(length(snap.ordered) - 1)
            along += snap.seg_lens[k - 1]
            pos[snap.ordered[k]] = snap.anchor_pos + (along / snap.path_len) * line
        end
    end
    return nothing
end

"""
    apply_tether_init_stretched_lens!(placement::Placement)

Move every tether given a stretched length to it, from the endpoint on the boundary —
a fixed or winch point, or one carried only by bodies held in the world. Tethers whose reach
overlaps are placed together at their mean length. Refuses a tether with neither endpoint
on the boundary, and skips one with both, warning.
"""
function apply_tether_init_stretched_lens!(placement::Placement)
    (; system) = placement
    specified = [index for (index, init) in enumerate(placement.tether_inits)
                 if !isnothing(init.stretched_length)]
    isempty(specified) && return nothing
    boundary = Set{Int}(winch.winch_point for winch in system.winches)
    for (index, point) in enumerate(system.points)
        point.fixed && push!(boundary, index)
        isnothing(point.body) && isnothing(point.tube) && continue
        carriers = anchor_body_idxs(placement, index)
        !isempty(carriers) && all(body -> placement.static_body[body], carriers) &&
            push!(boundary, index)
    end
    anchored = filter(specified) do index
        tether = system.tethers[index]
        both = tether.start_point in boundary && tether.end_point in boundary
        both && @warn "Tether $(tether.name): both endpoints are fixed; skipping its " *
                      "length placement."
        return !both
    end
    for index in anchored
        tether = system.tethers[index]
        isnothing(first(tether_anchor_free(tether, boundary))) && error(
            "tether $(tether.name) has neither endpoint on a fixed or winch point, so " *
            "it cannot be placed at its stretched length")
    end
    rigid_siblings = rigid_point_siblings(
        placement, connected_body_groups(length(system.bodies), system.tubes))
    downstream = Dict(map(anchored) do index
        tether = system.tethers[index]
        anchor, free = tether_anchor_free(tether, boundary)
        index => tether_downstream_idxs(tether, system.segments, boundary, free, anchor,
                                        rigid_siblings)
    end)
    reach = Dict(index => union(
        setdiff(Set(tether_ordered_point_idxs(system.tethers[index], system.segments)),
                boundary), downstream[index]) for index in anchored)
    body_neighbors = beam_body_neighbors(system.tubes)
    for cluster in tether_clusters(anchored, reach)
        apply_cluster_init_stretched_len!(placement, cluster, downstream, boundary,
                                          body_neighbors)
    end
    return nothing
end

"""
    init_unstretched_len(placement, index)

Rest length [m] of tether `index` from its placed length: that length times its
`stretch_frac`, or the length its `force` [N] stretches to at the segments' common
`unit_stiffness`; the placed length where neither is given.
"""
function init_unstretched_len(placement::Placement, index)
    tether = placement.system.tethers[index]
    segments = placement.system.segments
    stretched = sum(segment_length(segments[segment], placement.pos)
                    for segment in tether.segments)
    (; force, stretch_frac) = placement.tether_inits[index]
    !isnothing(force) && !isnothing(stretch_frac) && error(
        "tether $(tether.name): give init_stretch_frac or init_tether_force, not both")
    if !isnothing(stretch_frac)
        stretch_frac > 0 || error(
            "tether $(tether.name): init_stretch_frac $stretch_frac must be positive")
        return stretched * stretch_frac
    end
    force = something(force, 0.0)
    force >= 0 || error("tether $(tether.name): init_tether_force $force N is negative")
    force == 0 && return stretched
    stiffness = tether_unit_stiffness(tether, segments, placement.system.materials)
    force < stiffness || error(
        "tether $(tether.name): init_tether_force $force N is not below its " *
        "unit_stiffness $stiffness N")
    return stretched * (1 - force / stiffness)
end

"""The axial stiffness times rest length [N] all segments of `tether` share, from their
`linear` `materials` and diameters, refusing another law or segments that differ."""
function tether_unit_stiffness(tether, segments, materials)
    stiffnesses = [unit_stiffness(segments[index], materials) for index in tether.segments]
    any(isnothing, stiffnesses) && error(
        "tether $(tether.name): init_tether_force needs a linear material; use " *
        "init_stretch_frac")
    all(≈(first(stiffnesses)), stiffnesses) || error(
        "tether $(tether.name): its segments differ in unit_stiffness ($stiffnesses)")
    return first(stiffnesses)
end

"""
    unit_stiffness(material::Material, diameter)
    unit_stiffness(segment, materials)

Axial stiffness times rest length [N] of a line of `material` and `diameter` [m],
`youngs_modulus * π (diameter/2)^2`, or of `segment` of its material among `materials`;
`nothing` where the material's law is not `linear`.
"""
function unit_stiffness(material::Material, diameter)
    material.law == "linear" || return nothing
    area = π * (diameter / 2)^2
    parameter = material.parameters["youngs_modulus"]
    # the Float64 branch types the multiply, so the untyped parameter boxes no result
    youngs_modulus = parameter isa Float64 ? parameter : Float64(parameter)::Float64
    return youngs_modulus * area
end

unit_stiffness(segment::Segment, materials) =
    unit_stiffness(materials[segment.material], segment.diameter)

"""Share the rest length of each tether with a [`TetherInit`](@ref) that gives anything
([`init_unstretched_len`](@ref)) equally over its segments."""
function apply_tether_init_forces!(placement::Placement)
    for (index, tether) in enumerate(placement.system.tethers)
        gives_anything(placement.tether_inits[index]) || continue
        isempty(tether.segments) && continue
        len = init_unstretched_len(placement, index)
        len > 0 || error("tether $(tether.name): rest length $len m must be positive")
        placement.l0[Int.(tether.segments)] .= len / length(tether.segments)
    end
    return nothing
end
