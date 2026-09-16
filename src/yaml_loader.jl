# SPDX-FileCopyrightText: 2026 Bart van de Lint, Jelle Poland
# SPDX-License-Identifier: MIT

"""
    load_definition(yaml_path; name="from_yaml", set=nothing, dynamics_type,
                    aero_model)

Build a [`SystemDefinition`](@ref) from an authoring YAML file.

Each top-level block — `points`, `segments`, `pulleys`, `twist_surfaces`,
`tethers`, `winches`, `wings`, `bodies`, `elastic_joints`, `timoshenko_joints`,
`transforms` — is a table
written either as `headers` plus `data` rows, or as a list of mappings. Every
block is optional, and a row's columns are the keyword arguments of the
component's constructor. An optional `variables` block names values reused
across the file; see [`resolve_yaml_variables`](@ref).

Components reference each other by name. Angles are in degrees and diameters
in millimetres, the two places the file's units differ from the struct's.

# Keyword Arguments
- `name`: the definition's name.
- `set::Settings`: the settings the material defaults come from. Defaults to
  the `base` settings of the current data path.
- `dynamics_type::WingType`: override every wing's `dynamics_type` column.
- `aero_model::Symbol`: override every wing's `aero_model` column.
"""
function load_definition(yaml_path::AbstractString; name="from_yaml",
    set::Union{Nothing, Settings}=nothing,
    dynamics_type::Union{Nothing, WingType}=nothing,
    aero_model::Union{Nothing, Symbol}=nothing
)
    data = resolve_yaml_variables(YAML.load_file(yaml_path))
    set = isnothing(set) ? load_settings("base") : set

    points = load_yaml_points(data)
    (isempty(points) && !haskey(data, "bodies")) &&
        error("No points or bodies found in YAML file $yaml_path.")

    return SystemDefinition(name, set;
        points,
        segments=load_yaml_segments(data, set),
        pulleys=load_yaml_pulleys(data),
        twist_surfaces=load_yaml_twist_surfaces(data),
        tethers=load_yaml_tethers(data, set),
        winches=load_yaml_winches(data, set),
        wings=load_yaml_wings(data, dynamics_type, aero_model),
        bodies=load_yaml_bodies(data),
        joints=load_yaml_joints(data),
        transforms=load_yaml_transforms(data))
end

# ==================== BLOCK LOADERS ==================== #

"""
    load_yaml_points(data) -> Vector{Point}

The `points` block. Required columns `name`, `pos_cad` and `type`; optional
`wing_idx`, `transform_idx`, `body_idx`, `joint`, `anchor_b`, `extra_mass`,
`body_frame_damping`, `world_frame_damping`, `area`, `drag_coeff`,
`fix_sphere`, `fix_static`.
"""
function load_yaml_points(data)
    points = Point[]
    for (i, row) in enumerate(yaml_rows(data, "points"))
        push!(points, call_yaml_constructor(Point, row,
            [:name, :pos_cad, :type],
            [:wing, :transform, :body, :joint, :anchor_b, :extra_mass,
             :body_frame_damping, :world_frame_damping, :area, :drag_coeff,
             :fix_sphere, :fix_static];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :pos_cad => row -> KVec3(row.pos_cad...),
                :type => row -> parse_dynamics_type(String(row.type)),
                :wing => row -> yaml_ref(row, :wing_idx),
                :transform => row -> yaml_ref(row, :transform_idx),
                :body => row -> yaml_ref(row, :body_idx),
                :joint => row -> yaml_ref(row, :joint),
                :anchor_b => row -> yaml_vec3(row, :anchor_b))))
    end
    return points
end

"""
    load_yaml_segments(data, set) -> Vector{Segment}

The `segments` block. Required columns `name`, `point_i` and `point_j`;
optional `l0`, `diameter_mm`, `unit_stiffness`, `unit_damping`,
`youngs_modulus`, `damping_per_stiffness`, `density`, `compression_frac`,
`compression_damping_frac`. What is left out comes from `set`.
"""
function load_yaml_segments(data, set)
    segments = Segment[]
    for (i, row) in enumerate(yaml_rows(data, "segments"))
        push!(segments, call_yaml_constructor(Segment, row,
            [:name, :set, :point_i, :point_j],
            [:l0, :diameter, :unit_stiffness, :unit_damping, :density,
             :youngs_modulus, :damping_per_stiffness, :compression_frac,
             :compression_damping_frac];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :set => row -> set,
                :point_i => row -> yaml_ref(row, :point_i),
                :point_j => row -> yaml_ref(row, :point_j),
                :diameter => row -> yaml_diameter(row))))
    end
    return segments
end

"""
    load_yaml_pulleys(data) -> Vector{Pulley}

The `pulleys` block. Required columns `name`, `segment_i`, `segment_j` and
`type`; optional `efficiency`, `damping`, `friction_epsilon`.
"""
function load_yaml_pulleys(data)
    pulleys = Pulley[]
    for (i, row) in enumerate(yaml_rows(data, "pulleys"))
        push!(pulleys, call_yaml_constructor(Pulley, row,
            [:name, :segment_i, :segment_j, :type],
            [:efficiency, :damping, :friction_epsilon];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :segment_i => row -> yaml_ref(row, :segment_i),
                :segment_j => row -> yaml_ref(row, :segment_j),
                :type => row -> parse_dynamics_type(String(row.type)))))
    end
    return pulleys
end

"""
    load_yaml_twist_surfaces(data) -> Vector{TwistSurface}

The `twist_surfaces` block. Required columns `name`, `points` (or the older
`point_idxs`) and `type`; optional `moment_frac`, `damping`, `stiffness`,
`area`, `wing`, `bodies`, `flap_bodies`, `flap_axis`.
"""
function load_yaml_twist_surfaces(data)
    twist_surfaces = TwistSurface[]
    for (i, row) in enumerate(yaml_rows(data, "twist_surfaces"))
        push!(twist_surfaces, call_yaml_constructor(TwistSurface, row,
            [:name, :points, :type, :moment_frac],
            [:damping, :stiffness, :area, :wing, :bodies, :flap_bodies,
             :flap_axis];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :points => row -> yaml_refs(row, :points, :point_idxs),
                :type => row -> parse_dynamics_type(String(row.type)),
                :moment_frac => row -> something(
                    yaml_float(row, :moment_frac), 0.0),
                :wing => row -> something(yaml_ref(row, :wing), 0),
                :bodies => row -> yaml_refs(row, :bodies),
                :flap_bodies => row -> yaml_refs(row, :flap_bodies))))
    end
    return twist_surfaces
end

"""
    load_yaml_tethers(data, set) -> Vector{Tether}

The `tethers` block. A row naming `segment_idxs` is a tether of explicit
segments; one naming `start_point`, `end_point` and `n_segments` generates its
own, taking the material columns of [`load_yaml_segments`](@ref). Optional on
both: `init_stretched_length`, `init_tether_force`, `init_stretch_frac`.
"""
function load_yaml_tethers(data, set)
    tethers = Tether[]
    for (i, row) in enumerate(yaml_rows(data, "tethers"))
        name = yaml_row_name(row, i)
        stretched_length = yaml_float(row, :init_stretched_length)
        tether_force = yaml_float(row, :init_tether_force)
        stretch_frac = yaml_float(row, :init_stretch_frac)
        segment_refs = yaml_field(row, :segment_idxs)
        tether = if isnothing(segment_refs)
            Tether(name, set, stretched_length;
                start_point=yaml_ref(row, :start_point),
                end_point=yaml_ref(row, :end_point),
                n_segments=Int(row.n_segments),
                unit_stiffness=yaml_float_or_nan(row, :unit_stiffness),
                unit_damping=yaml_float_or_nan(row, :unit_damping),
                diameter=yaml_diameter(row),
                density=yaml_float_or_nan(row, :density),
                youngs_modulus=yaml_float_or_nan(row, :youngs_modulus),
                damping_per_stiffness=yaml_float_or_nan(
                    row, :damping_per_stiffness),
                compression_frac=something(
                    yaml_float(row, :compression_frac), 0.1),
                compression_damping_frac=something(
                    yaml_float(row, :compression_damping_frac), 1.0),
                tether_force, stretch_frac)
        else
            Tether(name, name_refs(segment_refs), stretched_length;
                start_point=yaml_ref(row, :start_point),
                end_point=yaml_ref(row, :end_point),
                tether_force, stretch_frac)
        end
        push!(tethers, tether)
    end
    return tethers
end

"""
    load_yaml_winches(data, set) -> Vector{Winch}

The `winches` block. Required columns `name`, `tether_idxs` and `winch_point`;
optional `init_vel`, `speed_controlled`, `friction_epsilon`. The drum
parameters come from `set`.
"""
function load_yaml_winches(data, set)
    winches = Winch[]
    for (i, row) in enumerate(yaml_rows(data, "winches"))
        push!(winches, call_yaml_constructor(Winch, row,
            [:name, :set, :tethers],
            [:winch_point, :init_vel, :speed_controlled, :friction_epsilon];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :set => row -> set,
                :tethers => row -> yaml_refs(row, :tether_idxs),
                :winch_point => row -> yaml_ref(row, :winch_point))))
    end
    return winches
end

"""
    load_yaml_wings(data, dynamics_type, aero_model) -> Vector{Body}

The `wings` block. Required columns `name` and `dynamics_type` (unless the
caller overrides it); optional `aero_model`, `twist_surfaces`,
`transform_idx`, `pos_cad`, `mass`, `com`, `inertia_principal`,
`angular_damping`, `principal_frame_method`, `origin_idx`, `z_ref_points`,
`y_ref_points`. An unnamed `aero_model` is the [`Wing`](@ref) default for the
wing's `dynamics_type`.
"""
function load_yaml_wings(data, dynamics_type, aero_model)
    wings = Body[]
    for (i, row) in enumerate(yaml_rows(data, "wings"))
        push!(wings, call_yaml_constructor(Wing, row,
            [:name, :twist_surfaces],
            [:dynamics_type, :aero_model, :transform, :pos_cad, :mass,
             :com_offset_b, :inertia_principal, :angular_damping,
             :world_frame_damping, :body_frame_damping, :drag_frac,
             :group_points_moment, :principal_frame_method,
             :origin, :z_ref_points, :y_ref_points];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :twist_surfaces => row -> yaml_refs(row, :twist_surfaces),
                :dynamics_type => row -> isnothing(dynamics_type) ?
                    parse_wing_type(String(yaml_required(row, :dynamics_type,
                        "Wing $(yaml_row_name(row, i))"))) : dynamics_type,
                :aero_model => row -> isnothing(aero_model) ?
                    yaml_symbol(row, :aero_model) : aero_model,
                :transform => row -> yaml_ref(row, :transform_idx),
                :pos_cad => row -> yaml_vec3(row, :pos_cad),
                :com_offset_b => row -> yaml_vec3(row, :com),
                :inertia_principal => row -> yaml_vec3(row,
                    :inertia_principal),
                :principal_frame_method => row -> parse_principal_frame_method(
                    yaml_symbol(row, :principal_frame_method)),
                :origin => row -> yaml_weighted(row, :origin_idx),
                :z_ref_points => row -> yaml_ref_pair(row, :z_ref_points),
                :y_ref_points => row -> yaml_ref_pair(row, :y_ref_points))))
    end
    return wings
end

"""
    load_yaml_bodies(data) -> Vector{Body}

The `bodies` block of plain rigid bodies. Required columns `name`, `mass`,
`pos` and one of `inertia_principal` (3-vector) or `inertia` (3×3); optional
`type`, `transform_idx`, `wing`, `Q_b_to_w` (4-vector), `com_offset_b`,
`angular_damping`, `world_frame_damping`, `body_frame_damping`, `fix_sphere`,
`fix_static`, `principal_frame_method`.
"""
function load_yaml_bodies(data)
    bodies = Body[]
    for (i, row) in enumerate(yaml_rows(data, "bodies"))
        push!(bodies, call_yaml_constructor(Body, row,
            [:name],
            [:mass, :pos_cad, :inertia_principal, :inertia, :Q_body_to_cad,
             :com_offset_b, :type, :transform, :wing, :angular_damping,
             :world_frame_damping, :body_frame_damping, :fix_sphere,
             :fix_static, :principal_frame_method];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :mass => row -> yaml_required(row, :mass,
                    "Body $(yaml_row_name(row, i))"),
                :pos_cad => row -> KVec3(yaml_required(row, :pos,
                    "Body $(yaml_row_name(row, i))")...),
                :inertia_principal => row ->
                    yaml_vec3(row, :inertia_principal),
                :inertia => row -> yaml_matrix3(row, :inertia),
                :Q_body_to_cad => row -> yaml_quaternion(row, :Q_b_to_w),
                :com_offset_b => row -> yaml_vec3(row, :com_offset_b),
                :type => row -> parse_optional_dynamics_type(
                    yaml_field(row, :type)),
                :transform => row -> yaml_ref(row, :transform_idx),
                :wing => row -> yaml_ref(row, :wing),
                :principal_frame_method => row -> parse_principal_frame_method(
                    yaml_symbol(row, :principal_frame_method)))))
    end
    return bodies
end

"""
    load_yaml_joints(data) -> Vector{Joint}

The `elastic_joints` and `timoshenko_joints` blocks. Every row needs `body_a`,
`body_b` and its model's stiffnesses — `stiffness_axial`, `stiffness_shear`,
`stiffness_torsion` and `stiffness_bending` for an [`ElasticJoint`](@ref),
`EA`, `GA`, `GJ`, `EIy` and `EIz` for a [`TimoshenkoBeam`](@ref); `anchor_a`,
`anchor_b`, `damping` and `radius` are optional on both, as are the beam's
`shear_coeff` and `rest_length`. Stiffnesses written in a file are linear;
a nonlinear law is supplied programmatically.
"""
function load_yaml_joints(data)
    joints = Joint[]
    for (block, model) in (("elastic_joints", ElasticJoint),
                           ("timoshenko_joints", TimoshenkoBeam))
        for (i, row) in enumerate(yaml_rows(data, block))
            name = yaml_row_name(row, i)
            body_a = yaml_ref(row, :body_a)
            body_b = yaml_ref(row, :body_b)
            (isnothing(body_a) || isnothing(body_b)) &&
                error("$(nameof(model)) $name: requires `body_a` and `body_b`.")
            push!(joints, Joint(name, body_a, body_b,
                joint_model(model, row, name);
                anchor_a=something(yaml_vec3(row, :anchor_a), zeros(KVec3)),
                anchor_b=something(yaml_vec3(row, :anchor_b), zeros(KVec3)),
                damping=something(yaml_float(row, :damping), 0.0),
                radius=yaml_float(row, :radius)))
        end
    end
    return joints
end

"""
    joint_model(::Type{ElasticJoint}, row, name) -> ElasticJoint
    joint_model(::Type{TimoshenkoBeam}, row, name) -> TimoshenkoBeam

The stiffness model of one joint row, with each required stiffness read and
each optional parameter defaulted.
"""
joint_model(::Type{ElasticJoint}, row, name) = ElasticJoint(;
    stiffnesses(row, "ElasticJoint $name",
        (:stiffness_axial, :stiffness_shear, :stiffness_torsion,
         :stiffness_bending))...)

joint_model(::Type{TimoshenkoBeam}, row, name) = TimoshenkoBeam(;
    stiffnesses(row, "TimoshenkoBeam $name", (:EA, :GA, :GJ, :EIy, :EIz))...,
    shear_coeff=something(yaml_float(row, :shear_coeff), 5 / 6),
    rest_length=something(yaml_float(row, :rest_length), 0.0))

"""
    stiffnesses(row, label, fields) -> NamedTuple

Every one of `fields` read off `row` as a stiffness the joint cannot do
without.
"""
stiffnesses(row, label, fields) =
    NamedTuple(field => yaml_required(row, field, label) for field in fields)

"""
    load_yaml_transforms(data) -> Vector{Transform}

The `transforms` block. Required columns `name`, `elevation`, `azimuth` and
`heading`, all in degrees; one of `base_pos` with `base_point_idx` or
`base_transform_idx`; one of `wing_idx` or `rot_point_idx`. Optional
`elevation_vel`, `azimuth_vel` and `turn_rate`, in degrees per second.
"""
function load_yaml_transforms(data)
    transforms = Transform[]
    for (i, row) in enumerate(yaml_rows(data, "transforms"))
        push!(transforms, call_yaml_constructor(Transform, row,
            [:name, :elevation, :azimuth, :heading],
            [:base_point, :base_pos, :base_transform, :wing, :rot_point,
             :elevation_vel, :azimuth_vel, :turn_rate];
            mappings=Dict(
                :name => row -> yaml_row_name(row, i),
                :elevation => row -> deg2rad(row.elevation),
                :azimuth => row -> deg2rad(row.azimuth),
                :heading => row -> deg2rad(row.heading),
                :elevation_vel => row -> yaml_rad(row, :elevation_vel),
                :azimuth_vel => row -> yaml_rad(row, :azimuth_vel),
                :turn_rate => row -> yaml_rad(row, :turn_rate),
                :base_pos => row -> yaml_vec3(row, :base_pos),
                :base_point => row -> yaml_ref(row, :base_point_idx),
                :base_transform => row -> yaml_ref(row, :base_transform_idx),
                :rot_point => row -> yaml_ref(row, :rot_point_idx),
                :wing => row -> yaml_ref(row, :wing_idx))))
    end
    return transforms
end

# ==================== TABLES ==================== #

"""
    yaml_rows(data, block) -> Vector{NamedTuple}

The rows of one top-level block, empty when the block is absent or holds no
`data`.
"""
function yaml_rows(data, block)
    haskey(data, block) || return NamedTuple[]
    return parse_table(data[block])
end

"""
    parse_table(table) -> Vector{NamedTuple}

The rows of a `headers`/`data` table, or of a table whose rows are mappings.
In the `headers` form a row may stop short of the last column, and a row whose
first cell starts with `#` is a comment.
"""
function parse_table(table)::Vector{NamedTuple}
    haskey(table, "data") || throw(ArgumentError("table is missing `data`"))
    rows = table["data"]
    (isnothing(rows) || isempty(rows)) && return NamedTuple[]
    if first(rows) isa AbstractDict
        return [NamedTuple{Tuple(Symbol.(keys(row)))}(Tuple(values(row)))
                for row in rows]
    end

    haskey(table, "headers") ||
        throw(ArgumentError("table with array rows requires `headers`"))
    headers = Tuple(Symbol.(table["headers"]))
    parsed = NamedTuple[]
    for (k, row) in enumerate(rows)
        (isempty(row) || (row[1] isa String && startswith(row[1], "#"))) &&
            continue
        if length(row) > length(headers)
            @warn "Skipping row $k: has $(length(row)) values, expected " *
                "$(length(headers))."
            continue
        end
        length(row) < length(headers) &&
            (row = vcat(row, fill(nothing, length(headers) - length(row))))
        push!(parsed, NamedTuple{headers}(Tuple(row)))
    end
    return parsed
end

"""
    call_yaml_constructor(Constructor, row, args_spec, kwargs_spec; mappings)

Call `Constructor` with the positional arguments named by `args_spec` and the
keyword arguments named by `kwargs_spec`, each read from `row` — or, where the
name has an entry in `mappings`, from that function of the row. A keyword whose
column is absent or [`yaml_unset`](@ref), or whose mapping returns `nothing`,
is left to the constructor's own default.
"""
function call_yaml_constructor(Constructor, row::NamedTuple,
    args_spec::Vector{Symbol}, kwargs_spec::Vector{Symbol};
    mappings::Dict{Symbol, <:Function}=Dict{Symbol, Function}()
)
    args = [yaml_argument(row, name, mappings) for name in args_spec]
    kwargs = Dict{Symbol, Any}()
    for name in kwargs_spec
        value = yaml_argument(row, name, mappings; required=false)
        isnothing(value) || (kwargs[name] = value)
    end
    return Constructor(args...; kwargs...)
end

"""
    yaml_argument(row, name, mappings; required=true) -> value

One constructor argument: the `mappings` entry for `name` applied to the row,
else the row's own `name` column. A required argument that is missing is an
error; an optional one comes back as `nothing`.
"""
function yaml_argument(row, name, mappings; required=true)
    haskey(mappings, name) && return mappings[name](row)
    value = yaml_field(row, name)
    (required && isnothing(value)) && error("Missing required arg $name")
    return value
end

# ==================== CELLS ==================== #

"""
    yaml_unset(value) -> Bool

Whether a cell is unset: `nothing`, or the `nothing` placeholder a written
table uses to leave one row's cell empty in a column the other rows fill.
"""
yaml_unset(value) = isnothing(value) || value == "nothing" || value === :nothing

"""
    yaml_field(row, field)

An optional cell, `nothing` when the column is absent or [`yaml_unset`](@ref).
"""
function yaml_field(row, field)
    hasfield(typeof(row), field) || return nothing
    value = getfield(row, field)
    return yaml_unset(value) ? nothing : value
end

"""
    yaml_required(row, field, label)

A cell that must be there, named by `label` in the error when it is not.
"""
function yaml_required(row, field, label)
    value = yaml_field(row, field)
    isnothing(value) && error("$label: missing required `$field`.")
    return value
end

"""
    yaml_float(row, field) -> Union{Float64, Nothing}

An optional numeric cell.
"""
yaml_float(row, field) =
    (value = yaml_field(row, field); isnothing(value) ? nothing : Float64(value))

"""
    yaml_float_or_nan(row, field) -> Float64

An optional numeric cell, `NaN` when it is absent — the sentinel that asks for
the settings' value.
"""
yaml_float_or_nan(row, field) = something(yaml_float(row, field), NaN)

"""
    yaml_rad(row, field) -> Float64

An optional angle cell, written in degrees and returned in radians. Absent
means zero.
"""
yaml_rad(row, field) = deg2rad(something(yaml_float(row, field), 0.0))

"""
    yaml_diameter(row) -> Float64

The `diameter_mm` cell in metres, `NaN` when absent.
"""
function yaml_diameter(row)
    diameter_mm = yaml_float(row, :diameter_mm)
    return isnothing(diameter_mm) ? NaN : 0.001 * diameter_mm
end

"""
    yaml_vec3(row, field) -> Union{KVec3, Nothing}

An optional 3-vector cell.
"""
yaml_vec3(row, field) =
    (value = yaml_field(row, field); isnothing(value) ? nothing : KVec3(value...))

"""
    yaml_quaternion(row, field) -> Union{Vector{SimFloat}, Nothing}

An optional 4-element quaternion cell, scalar first.
"""
yaml_quaternion(row, field) =
    (value = yaml_field(row, field);
     isnothing(value) ? nothing : Vector{SimFloat}(value))

"""
    yaml_matrix3(row, field) -> Union{Matrix{SimFloat}, Nothing}

An optional 3×3 matrix cell, written as three rows.
"""
function yaml_matrix3(row, field)
    value = yaml_field(row, field)
    isnothing(value) && return nothing
    return Matrix{SimFloat}(permutedims(reduce(hcat,
        (Vector{SimFloat}(entry) for entry in value))))
end

"""
    yaml_symbol(row, field) -> Union{Symbol, Nothing}

An optional cell naming something, as a `Symbol`.
"""
yaml_symbol(row, field) =
    (value = yaml_field(row, field); isnothing(value) ? nothing : Symbol(value))

"""
    yaml_row_name(row, i)

A row's name: its `name` cell as a `Symbol`, or the 1-based row index when the
row has none, so a component can be referenced either way.
"""
yaml_row_name(row, i) = something(yaml_symbol(row, :name), i)

# ==================== REFERENCES ==================== #

"""
    yaml_ref(row, field) -> Union{NameRef, Nothing}

An optional reference cell. An integer stays a position and anything else
becomes a name.
"""
yaml_ref(row, field) =
    (value = yaml_field(row, field); isnothing(value) ? nothing : name_ref(value))

"""
    yaml_refs(row, fields...) -> Vector{NameRef}

An optional list-of-references cell, taken from the first of `fields` the row
fills, and empty when it fills none.
"""
function yaml_refs(row, fields...)
    for field in fields
        value = yaml_field(row, field)
        isnothing(value) || return name_refs(value)
    end
    return NameRef[]
end

"""
    yaml_weighted(row, field)

An optional reference-point cell in the form [`WeightedRefPoints`](@ref) takes:
one point (`kcu`), several averaged equally (`[le, te]`), or several weighted
(`[[le, 0.7], [te, 0.3]]`).
"""
yaml_weighted(row, field) =
    (value = yaml_field(row, field);
     isnothing(value) ? nothing : weighted_refs(value))

"""
    weighted_refs(value)

A reference-point cell in the form [`WeightedRefPoints`](@ref) takes: a scalar
stays one, a list of `[name, weight]` pairs becomes a list of tuples.
"""
function weighted_refs(value)
    value isa AbstractVector || return name_ref(value)
    (isempty(value) || !(value[1] isa AbstractVector)) &&
        return [name_ref(entry) for entry in value]
    return map(value) do entry
        (entry isa AbstractVector && length(entry) == 2 &&
            entry[2] isa Number) || throw(ArgumentError(
            "Invalid weighted reference point $(repr(entry)); expected " *
            "[[name, weight], ...] with a numeric weight."))
        (name_ref(entry[1]), Float64(entry[2]))
    end
end

"""
    yaml_ref_pair(row, field) -> Union{Tuple, Nothing}

An optional pair of reference-point cells, the two ends of a body-frame axis.
"""
function yaml_ref_pair(row, field)
    value = yaml_field(row, field)
    isnothing(value) && return nothing
    length(value) == 2 ||
        throw(ArgumentError("$field must have 2 elements, got $(length(value))"))
    return (weighted_refs(value[1]), weighted_refs(value[2]))
end

# ==================== ENUMS ==================== #

"""
    parse_dynamics_type(text) -> DynamicsType

A [`DynamicsType`](@ref) named in a file, case-insensitively.
"""
function parse_dynamics_type(text::AbstractString)
    for value in instances(DynamicsType)
        uppercase(text) == String(Symbol(value)) && return value
    end
    error("Unknown DynamicsType: $text. Known: " *
        join(instances(DynamicsType), ", "))
end

"""
    parse_optional_dynamics_type(text) -> Union{DynamicsType, Nothing}

[`parse_dynamics_type`](@ref) that passes `nothing` through, so an absent
column keeps the constructor's default.
"""
parse_optional_dynamics_type(::Nothing) = nothing
parse_optional_dynamics_type(text) = parse_dynamics_type(String(text))

"""
    parse_wing_type(text) -> WingType

A [`WingType`](@ref) named in a file, case-insensitively.
"""
function parse_wing_type(text::AbstractString)
    for value in instances(WingType)
        uppercase(text) == String(Symbol(value)) && return value
    end
    error("Unknown WingType: $text. Known: " * join(instances(WingType), ", "))
end

"""
    parse_principal_frame_method(name) -> Union{PrincipalFrameMethod, Nothing}

A [`PrincipalFrameMethod`](@ref) named in a file, case-insensitively, passing
`nothing` through so an absent column keeps the default.
"""
parse_principal_frame_method(::Nothing) = nothing

function parse_principal_frame_method(name::Symbol)
    for value in instances(PrincipalFrameMethod)
        uppercase(String(name)) == String(Symbol(value)) && return value
    end
    error("Unknown PrincipalFrameMethod: $name. Known: " *
        join(instances(PrincipalFrameMethod), ", "))
end

# ==================== VARIABLES ==================== #

"""
    resolve_yaml_variables(data) -> data

Apply an optional top-level `variables` block to the rest of the tree and drop
the block. A variable holding a number, string or list replaces any cell
written as its name; a variable holding a mapping fills the columns it names
at once. Variables may name other variables.

```yaml
variables:
  bridle_comp: 0.01
  dyneema: {youngs_modulus: 55.0e9, damping_per_stiffness: 0.00077,
            density: 724.0}
```
"""
function resolve_yaml_variables(data)
    haskey(data, "variables") || return data
    raw = data["variables"]
    raw isa AbstractDict || throw(ArgumentError(
        "`variables` must be a mapping of name => value, got $(typeof(raw))."))

    variables = Dict{String, Any}()
    for name in keys(raw)
        resolve_variable!(variables, raw, String(name), String[])
    end
    check_variable_names(data, variables)

    multi_variables = Dict{String, Any}()
    scalars = Dict{String, Any}()
    for (name, value) in variables
        (value isa AbstractDict ? multi_variables : scalars)[name] = value
    end

    lookup = name -> get(scalars, name, name)
    resolved = Dict{Any, Any}()
    for (key, value) in data
        key == "variables" && continue
        resolved[key] = expand_multi_variables(
            substitute_variables(value, lookup), multi_variables)
    end
    return resolved
end

"""
    substitute_variables(value, lookup)

Replace every string inside `value` — scalars, list entries and mapping
values, recursively — by `lookup(string)`. A table's `headers` entry is left
alone, so a column may share its name with a variable.
"""
substitute_variables(value::AbstractString, lookup) = lookup(value)
substitute_variables(value::AbstractVector, lookup) =
    [substitute_variables(entry, lookup) for entry in value]
substitute_variables(value::AbstractDict, lookup) =
    Dict{Any, Any}(key => (key == "headers" ? entry :
        substitute_variables(entry, lookup)) for (key, entry) in value)
substitute_variables(value, lookup) = value

"""
    resolve_variable!(resolved, raw, name, pending) -> value

The value of variable `name`, substituting any variable it names first.
Strings that are not variable names come back unchanged; `pending` is the
chain being resolved, which is what reports a cycle.
"""
function resolve_variable!(resolved::Dict{String, Any}, raw, name::String,
                           pending::Vector{String})
    haskey(resolved, name) && return resolved[name]
    haskey(raw, name) || return name
    name in pending &&
        error("Cyclic variable reference: " * join([pending; name], " -> "))
    push!(pending, name)
    value = substitute_variables(raw[name],
        other -> resolve_variable!(resolved, raw, other, pending))
    pop!(pending)
    resolved[name] = value
    return value
end

"""
    check_variable_names(data, variables)

Error when a variable's name is also a component's name, since references to
that component would resolve to the variable instead.
"""
function check_variable_names(data, variables::Dict{String, Any})
    for (key, table) in data
        key == "variables" && continue
        (table isa AbstractDict && haskey(table, "data")) || continue
        for row in parse_table(table)
            name = yaml_field(row, :name)
            (name isa AbstractString && haskey(variables, name)) && error(
                "Variable `$name` has the same name as a component in block " *
                "`$key`; rename one of them.")
        end
    end
end

"""
    expand_multi_variables(table, multi_variables) -> table

Expand the multi-variables used in the rows of one block. A block holding no
`data` rows comes back unchanged.
"""
function expand_multi_variables(table, multi_variables::Dict{String, Any})
    (table isa AbstractDict && haskey(table, "data")) || return table
    rows = table["data"]
    (isnothing(rows) || isempty(rows)) && return table
    headers = haskey(table, "headers") ? String.(table["headers"]) : String[]
    expanded = Dict{Any, Any}(table)
    expanded["data"] = [expand_multi_variable_row(row, headers, multi_variables)
                        for row in rows]
    return expanded
end

"""
    expand_multi_variable_row(row, headers, multi_variables) -> row

Replace every cell of `row` naming a multi-variable by that variable's fields.
In a `headers`/`data` row they fill the columns from that cell on, written in
header order, so the row carries one entry for the whole group. In a mapping
row they are merged in, without overwriting what the row states itself.
"""
function expand_multi_variable_row(row::AbstractVector, headers,
                                   multi_variables::Dict{String, Any})
    any(entry -> entry isa AbstractString && haskey(multi_variables, entry),
        row) || return row
    expanded = Any[]
    for entry in row
        if !(entry isa AbstractString) || !haskey(multi_variables, entry)
            push!(expanded, entry)
            continue
        end
        fields = multi_variables[entry]
        first_column = length(expanded) + 1
        last_column = first_column + length(fields) - 1
        last_column <= length(headers) || error("Variable `$entry` fills " *
            "$(length(fields)) columns, but only " *
            "$(max(0, length(headers) - first_column + 1)) headers are left " *
            "at that position.")
        columns = headers[first_column:last_column]
        issetequal(columns, keys(fields)) || error("Variable `$entry` defines " *
            "$(join(sort!(collect(keys(fields))), ", ")), but the columns at " *
            "that position are $(join(columns, ", ")).")
        append!(expanded, (fields[column] for column in columns))
    end
    return expanded
end

function expand_multi_variable_row(row::AbstractDict, headers,
                                   multi_variables::Dict{String, Any})
    references = [(key, value) for (key, value) in row
        if value isa AbstractString && haskey(multi_variables, value)]
    isempty(references) && return row
    expanded = Dict{Any, Any}(row)
    for (key, value) in references
        delete!(expanded, key)
        for (field, item) in multi_variables[value]
            haskey(expanded, field) || (expanded[field] = item)
        end
    end
    return expanded
end
