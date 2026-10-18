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

"""Stretched length [m], force [N] and stretch fraction a tether starts at, each
optional."""
const TetherInit = @NamedTuple{stretched_length::Union{Nothing, Float64},
                               force::Union{Nothing, Float64},
                               stretch_frac::Union{Nothing, Float64}}

"""
    Placement

The pose of a system while it is placed: where each point and body origin is, how each body
is turned, and what of the authoring YAML moves them. `system` holds the components, every
reference an index; the vectors are indexed as its blocks.
"""
struct Placement
    system::SystemDefinition
    "position of each point [m]"
    pos::Vector{Vec3}
    "rest length of each segment [m]"
    l0::Vector{Float64}
    "position of each body's origin [m]"
    body_pos::Vector{Vec3}
    "rotation of each body's frame into the world"
    body_R::Vector{Mat3}
    "centre of each body's own mass in its frame [m]"
    com_offset::Vector{Vec3}
    "position of each body-carried point in its body's frame [m]"
    anchor::Vector{Vec3}
    "index of the wing body each point belongs to, 0 for none"
    point_wing::Vector{Int}
    "whether each point is a member of a station"
    wing_node::Vector{Bool}
    "whether each body is clamped to the world"
    static_body::Vector{Bool}
    "whether each wing is a particle wing"
    particle_wing::Vector{Bool}
    tether_init::Vector{TetherInit}
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
    place_authored!(placement::Placement, design::Pose, transforms; ignore_l0=false)

Place the system by its tethers' lengths and `transforms`, from the pose `design`
`placement` stands in: tethers to their stretched lengths, every unset rest length to the
length it then has, tether and pulley rest lengths shared out, the transforms applied
([`place_transforms!`](@ref)), and each body's points carried with it. `ignore_l0` makes
every rest length the placed length.
"""
function place_authored!(placement::Placement, design, transforms; ignore_l0=false)
    apply_tether_init_stretched_lens!(placement)
    update_segment_lengths!(placement)
    apply_tether_init_forces!(placement)
    init_pulley_lengths!(placement)
    place_transforms!(Pose(placement), design, transforms)
    carry_body_points!(placement)
    ignore_l0 && relax_segments!(placement)
    return placement
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

"""Body indices whose placement carries point `index`: the body it rides, or the wing body
of the station it is a member of. Empty for a point that stands free."""
function anchor_body_idxs(placement::Placement, index)
    body = placement.system.points[index].body
    isnothing(body) || return (body,)
    wing = placement.point_wing[index]
    placement.wing_node[index] && wing != 0 && return (wing,)
    return ()
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

"""Each point riding a rigid structure mapped to all the points sharing it: the members of
a rigid wing, and the points riding any body of one tube-connected group (`root`)."""
function rigid_point_siblings(placement::Placement, root)
    (; system, point_wing, wing_node) = placement
    siblings = Dict{Int, Set{Int}}()
    for wing in eachindex(system.wings)
        placement.particle_wing[wing] && continue
        members = Set(index for index in eachindex(system.points)
                      if wing_node[index] && point_wing[index] == wing)
        foreach(member -> siblings[member] = members, members)
    end
    body_members = Dict{Int, Set{Int}}()
    for (index, point) in enumerate(system.points)
        (!isnothing(point.body) || wing_node[index]) || continue
        for body in anchor_body_idxs(placement, index)
            push!(get!(body_members, root[body], Set{Int}()), index)
        end
    end
    for members in values(body_members)
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
            neighbor == anchor && throw(ArgumentError(
                "tether $(tether.name): the structure downstream of it connects back to " *
                "its anchor, so it cannot be placed at its length"))
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
        path_len > 0 || throw(ArgumentError(
            "tether $(tether.name) has no length to scale to its stretched length"))
        (; index, free, anchor_pos=pos[anchor], free_pos=pos[free], ordered, seg_lens,
         path_len)
    end
    stretched = [placement.tether_init[snap.index].stretched_length for snap in snaps]
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
a fixed or winch point, or one carried only by `STATIC` bodies. Tethers whose reach
overlaps are placed together at their mean length. Refuses a tether with neither endpoint
on the boundary, and skips one with both, warning.
"""
function apply_tether_init_stretched_lens!(placement::Placement)
    (; system) = placement
    specified = [index for (index, init) in enumerate(placement.tether_init)
                 if !isnothing(init.stretched_length)]
    isempty(specified) && return nothing
    boundary = Set{Int}(winch.winch_point for winch in system.winches)
    for (index, point) in enumerate(system.points)
        point.fixed && push!(boundary, index)
        isnothing(point.body) && continue
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
        isnothing(first(tether_anchor_free(tether, boundary))) && throw(ArgumentError(
            "tether $(tether.name) has neither endpoint on a fixed or winch point, so " *
            "it cannot be placed at its stretched length"))
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
    (; force, stretch_frac) = placement.tether_init[index]
    !isnothing(force) && !isnothing(stretch_frac) && throw(ArgumentError(
        "tether $(tether.name): give init_stretch_frac or init_tether_force, not both"))
    if !isnothing(stretch_frac)
        stretch_frac > 0 || throw(ArgumentError(
            "tether $(tether.name): init_stretch_frac $stretch_frac must be positive"))
        return stretched * stretch_frac
    end
    force = something(force, 0.0)
    force >= 0 || throw(ArgumentError(
        "tether $(tether.name): init_tether_force $force N is negative"))
    force == 0 && return stretched
    stiffness = tether_unit_stiffness(tether, segments, placement.system.materials)
    force < stiffness || throw(ArgumentError(
        "tether $(tether.name): init_tether_force $force N is not below its " *
        "unit_stiffness $stiffness N"))
    return stretched * (1 - force / stiffness)
end

"""The axial stiffness times rest length [N] all segments of `tether` share, from their
`linear` `materials` and diameters, refusing another law or segments that differ."""
function tether_unit_stiffness(tether, segments, materials)
    stiffnesses = [unit_stiffness(segments[index], materials, tether)
                   for index in tether.segments]
    all(≈(first(stiffnesses)), stiffnesses) || throw(ArgumentError(
        "tether $(tether.name): its segments differ in unit_stiffness ($stiffnesses)"))
    return first(stiffnesses)
end

"""Axial stiffness times rest length [N] of `segment` of `tether`, from its `linear`
material among `materials` and its diameter."""
function unit_stiffness(segment, materials, tether)
    material = materials[segment.material]
    material.law == "linear" || throw(ArgumentError(
        "tether $(tether.name): init_tether_force needs a linear material; use " *
        "init_stretch_frac"))
    return material.parameters["youngs_modulus"] * π * (segment.diameter / 2)^2
end

"""Share each tether's rest length ([`init_unstretched_len`](@ref)) equally over its
segments."""
function apply_tether_init_forces!(placement::Placement)
    for (index, tether) in enumerate(placement.system.tethers)
        isempty(tether.segments) && continue
        len = init_unstretched_len(placement, index)
        len > 0 || throw(ArgumentError(
            "tether $(tether.name): rest length $len m must be positive"))
        placement.l0[Int.(tether.segments)] .= len / length(tether.segments)
    end
    return nothing
end
