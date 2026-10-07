# SPDX-FileCopyrightText: 2025 Bart van de Lint, Jelle Poland
# SPDX-License-Identifier: MIT

# ==================== VARIABLES ==================== #

"""
    substitute_variables(value, lookup)

Replace every string inside `value` (scalars, list entries and mapping values,
recursively) by `lookup(string)`. The `headers` entry of a table is left alone,
so a column may share its name with a variable.
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

Value of variable `name`, substituting any variables it refers to first. Strings
that are not variable names are returned unchanged; `pending` tracks the chain
being resolved to report cycles.
"""
function resolve_variable!(resolved::Dict{String, Any}, raw, name::String,
                           pending::Vector{String})
    haskey(resolved, name) && return resolved[name]
    haskey(raw, name) || return name
    name in pending && error("Cyclic variable reference: " *
        join([pending; name], " -> "))
    push!(pending, name)
    value = substitute_variables(raw[name],
        other -> resolve_variable!(resolved, raw, other, pending))
    pop!(pending)
    resolved[name] = value
    return value
end

"""
    check_variable_names(data, variables)

Error when a variable name is also used as a component `name`, since references
to that component would resolve to the variable instead.
"""
function check_variable_names(data, variables::Dict{String, Any})
    for (key, table) in data
        key == "variables" && continue
        (table isa AbstractDict && haskey(table, "data")) || continue
        for row in parse_table(table)
            name = yaml_field(row, :name)
            (name isa AbstractString && haskey(variables, name)) &&
                error("Variable `$name` has the same name as a component in " *
                      "block `$key`; rename one of them.")
        end
    end
end

"""
    expand_multi_variable_row(row, headers, multi_variables) -> row

Replace every cell of `row` naming a multi-variable by that variable's fields. In
a `headers`/`data` row the fields fill the columns starting at the cell, written
in header order, so the row carries one entry for the whole group. In a dict row
they are merged in without overwriting fields the row states itself.
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

"""
    expand_multi_variables(table, multi_variables) -> table

Expand the multi-variables used in the rows of one YAML block. Blocks that hold
no `data` rows are returned unchanged.
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
    resolve_yaml_variables(data) -> data

Apply an optional top-level `variables` block to the rest of the YAML tree and
drop the block. A variable holding a number, string or list replaces any cell
written as its name; a variable holding a mapping is a multi-variable and fills
the columns it names at once. Variables may refer to other variables.
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
        target = value isa AbstractDict ? multi_variables : scalars
        target[name] = value
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

# ==================== TABLES ==================== #

"""
    parse_table(table) -> Vector{NamedTuple}

The rows of a `headers`/`data` table, or of a `data` list of mappings, as named tuples.
Rows starting with a `#` string are skipped, short rows padded with `nothing`, and long
rows skipped with a warning.
"""
function parse_table(table)::Vector{NamedTuple}
    haskey(table, "data") || throw(ArgumentError("table is missing `data`"))

    rows = table["data"]
    (isnothing(rows) || isempty(rows)) && return NamedTuple[]

    # Dict first row => dict format; Array first row => header format.
    first_row = first(rows)

    if first_row isa AbstractDict
        out = NamedTuple[]
        for row in rows
            named_row = NamedTuple{Tuple(Symbol.(keys(row)))}(
                Tuple(values(row)))
            push!(out, named_row)
        end
        return out
    else
        # Array format: requires headers
        haskey(table, "headers") ||
            throw(ArgumentError(
                "table with array rows requires `headers`"))
        headers = String.(table["headers"])

        out = NamedTuple[]
        for (k, row) in enumerate(rows)
            # skip empty or comment rows
            if isempty(row) ||
               (isa(row[1], String) && startswith(row[1], "#"))
                continue
            end
            # allow missing trailing columns (fill with nothing)
            if length(row) < length(headers)
                row = vcat(row, fill(nothing,
                    length(headers) - length(row)))
            end
            if length(row) > length(headers)
                @warn "Skipping row $k: has $(length(row)) " *
                      "values, expected $(length(headers))."
                continue
            end
            named_row = NamedTuple{Tuple(Symbol.(headers))}(Tuple(row))
            push!(out, named_row)
        end
        return out
    end
end

"""
    yaml_unset(value) -> Bool

Whether a cell is unset: `nothing`, or the `nothing` placeholder a written table
uses to leave one row's cell empty in a column the other rows fill.
"""
yaml_unset(value) = isnothing(value) || value == "nothing"

"""Optional row field, `nothing` when the column is absent or [`yaml_unset`](@ref)."""
function yaml_field(row, field)
    hasfield(typeof(row), field) || return nothing
    value = getfield(row, field)
    return yaml_unset(value) ? nothing : value
end

"""Optional numeric field as a `Float64`."""
function yaml_float(row, field)
    value = yaml_field(row, field)
    return isnothing(value) ? nothing : Float64(value)
end

"""Optional 3-vector field."""
function yaml_vec3(row, field)
    value = yaml_field(row, field)
    return isnothing(value) ? nothing : Vec3(value...)
end

"""Optional reference field: an index or a name."""
yaml_ref(row, field) = yaml_to_ref(yaml_field(row, field))

"""A reference cell as a `NameRef`, `nothing` where it is unset."""
yaml_to_ref(value) = yaml_unset(value) ? nothing :
                     value isa Integer ? Int(value) : String(value)

"""Name of a row: its `name` field, else its one-based index `i`, which an `idx` field must
repeat."""
function yaml_row_name(row, i)
    idx = yaml_field(row, :idx)
    isnothing(idx) || idx == i || throw(ArgumentError("row $i is numbered `idx` $idx"))
    return string(something(yaml_field(row, :name), i))
end

"""The dynamics type a cell names, refusing the removed `WING`."""
function parse_dynamics_type(text)
    name = uppercase(String(text))
    name == "WING" && throw(ArgumentError(
        "DynamicsType `WING` was removed: use `BODY_STATIC` (rigid wing, with " *
        "`body_idx` = the wing) or `DYNAMIC` (particle wing), and make the point " *
        "a member of one of the wing's stations."))
    return to_field(DynamicsType, name)
end

"""Reference points as a row writes them — one reference, a list averaged equally, or
`[reference, weight]` pairs — as weights per reference."""
function yaml_ref_points(value)
    value isa AbstractVector || return [yaml_to_ref(value) => 1.0]
    first(value) isa AbstractVector || return [yaml_to_ref(entry) => 1 / length(value)
                                               for entry in value]
    weights = [Float64(weight) for (_, weight) in value]
    total = sum(weights)
    total > 0 || throw(ArgumentError("reference point weights sum to $total"))
    isapprox(total, 1.0; atol=1e-6) ||
        @warn "Reference point weights sum to $total, normalizing to 1.0"
    return [yaml_to_ref(ref) => weight / total
            for ((ref, _), weight) in zip(value, weights)]
end

"""The pair of reference points in `field` of a wing row, or `nothing`."""
function yaml_ref_point_pair(row, field)
    value = yaml_field(row, field)
    isnothing(value) && return nothing
    length(value) == 2 || throw(ArgumentError("$field must hold two reference points"))
    return (yaml_ref_points(value[1]), yaml_ref_points(value[2]))
end

"""The 3×3 per-unit-mass inertia [m²] a wing row gives as a matrix or as
`[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`, or `nothing`."""
function yaml_unit_inertia(row)
    value = yaml_field(row, :unit_inertia)
    isnothing(value) && return nothing
    first(value) isa AbstractVector && return to_field(Mat3, value)
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = value
    return Mat3(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz)
end

"""`component` with the fields `changes` names replaced."""
function with_fields(component::T; changes...) where {T <: Component}
    return T((get(changes, field, getfield(component, field))
              for field in fieldnames(T))...)
end

"""
    read_rows(data, block, columns; modelled=true) -> [(row, model)]

The rows of `block` of `data`, none where it is absent or empty, each with the model it
reads ([`authoring_model`](@ref)) where `modelled`, else `NoModel`. Refuses a filled cell in
a column that neither the loader, which reads `columns` and `idx`, nor that model reads.
"""
function read_rows(data, block, columns; modelled=true)
    table = get(data, String(block), nothing)
    rows = isnothing(table) || isnothing(get(table, "data", nothing)) ? NamedTuple[] :
           parse_table(table)
    models = [modelled ? authoring_model(block, row) : NoModel() for row in rows]
    unread = OrderedSet{String}()
    read = (:idx, columns...)
    for (row, model) in zip(rows, models)
        headers = [String(column) for column in keys(row) if !(column in read)]
        union!(unread, unread_columns(headers, [yaml_field(row, Symbol(header))
                                                for header in headers], model))
    end
    isempty(unread) || throw(ArgumentError(
        "$block columns $(join(unread, ", ")) are read by neither the loader nor a " *
        "registered model"))
    return collect(zip(rows, models))
end

"""
    authoring_model(block, row)

The model of `row` of `block`: the one its `model` cell names, else, where the row has no
`model` column, the block's default, built from the columns named after its fields;
`NoModel` where that names no registered model. An absent column is an unset cell, refused
for a field that cannot hold `nothing`.
"""
function authoring_model(block, row)
    name = hasfield(typeof(row), :model) ? yaml_field(row, :model) : default_model(block)
    M = model_type(block, name)
    return M((model_value(M, field, yaml_field(row, field)) for field in fieldnames(M))...)
end

"""`value` as `field` of model `M`, refusing `nothing` where the field cannot hold it."""
function model_value(M, field, value)
    T = fieldtype(M, field)
    isnothing(value) && !(Nothing <: T) &&
        throw(ArgumentError("model $M needs its $field column filled"))
    return to_field(T, value)
end

# ==================== MATERIAL ==================== #

"""
    resolve_material(label, set; diameter, unit_stiffness, unit_damping, density,
                     youngs_modulus, damping_per_stiffness)

`(diameter, unit_stiffness, unit_damping, density)` of a segment from what its row gives,
`NaN` for what it leaves out: `youngs_modulus` [Pa] and `damping_per_stiffness` [s] in
place of `unit_stiffness` [N] and `unit_damping` [N*s], and `set`'s `d_tether` [mm],
`rho_tether`, `e_tether` and `rel_damping` for the rest. A `unit_stiffness` naming a
nonlinear law needs `unit_damping` given.
"""
function resolve_material(label, set::Settings; diameter=NaN, unit_stiffness=NaN,
                          unit_damping=NaN, density=NaN, youngs_modulus=NaN,
                          damping_per_stiffness=NaN)
    isnan(diameter) && (diameter = 0.001 * set.d_tether)
    isnan(density) && (density = set.rho_tether)
    area = π * (diameter / 2)^2
    linear = unit_stiffness isa Real
    if !isnan(youngs_modulus)
        (!linear || !isnan(unit_stiffness)) && throw(ArgumentError(
            "$label: give either `unit_stiffness` or `youngs_modulus`, not both"))
        unit_stiffness = youngs_modulus * area
    elseif linear && isnan(unit_stiffness)
        unit_stiffness = set.e_tether * area
    end
    if !isnan(damping_per_stiffness)
        isnan(unit_damping) || throw(ArgumentError(
            "$label: give either `unit_damping` or `damping_per_stiffness`, not both"))
        linear || throw(ArgumentError(
            "$label: `damping_per_stiffness` needs a linear `unit_stiffness`"))
        unit_damping = damping_per_stiffness * unit_stiffness
    elseif isnan(unit_damping)
        linear || throw(ArgumentError(
            "$label: give `unit_damping` for a nonlinear `unit_stiffness`"))
        set.rel_damping == 0 &&
            @warn "$label: unit_damping is zero (no rel_damping in the settings)."
        unit_damping = set.rel_damping * unit_stiffness
    end
    return Float64(diameter), unit_stiffness, Float64(unit_damping), Float64(density)
end

"""The columns [`material`](@ref) reads."""
const MATERIAL_COLUMNS = (:diameter_mm, :unit_stiffness, :unit_damping, :density,
                          :youngs_modulus, :damping_per_stiffness)

"""The material columns of a segment or tether row, `NaN` where a column is unset."""
function material(row)
    diameter_mm = yaml_float(row, :diameter_mm)
    stiffness = yaml_field(row, :unit_stiffness)
    return (; diameter=isnothing(diameter_mm) ? NaN : 0.001 * diameter_mm,
            unit_stiffness=stiffness isa AbstractString ? String(stiffness) :
                           something(yaml_float(row, :unit_stiffness), NaN),
            (field => something(yaml_float(row, field), NaN)
             for field in MATERIAL_COLUMNS[3:end])...)
end

"""A segment with its spring resolved from `materials` ([`resolve_material`](@ref)) and
`set`; a zero `l0` is set from the design pose before placement."""
function spring_segment(name, points, set; l0=0.0, compression_frac=0.1,
                        compression_damping_frac=1.0, materials...)
    diameter, unit_stiffness, unit_damping, density =
        resolve_material("segment $name", set; materials...)
    return Segment(; name, points, l0, diameter, density, unit_stiffness,
                   model=SegmentSpring(unit_damping, compression_frac,
                                       compression_damping_frac))
end

# ==================== BLOCKS ==================== #

"""The component `ref` names in `rows`: by name, or by its one-based index."""
function find_row(rows, ref, block)
    ref isa Int && return rows[ref]
    index = findfirst(row -> row.name == ref, rows)
    isnothing(index) && throw(ArgumentError("no $block named $ref"))
    return rows[index]
end

"""The points of the `points` block, each with the wing and transform it names."""
function read_points(data)
    points = Point[]
    point_wings = Union{Nothing, NameRef}[]
    point_transforms = Union{Nothing, NameRef}[]
    columns = (:name, :type, :wing_idx, :body_idx, :body, :pos_cad, :extra_mass, :area,
               :drag_coeff, :transform_idx)
    for (i, (row, model)) in enumerate(read_rows(data, :points, columns))
        name = yaml_row_name(row, i)
        type = parse_dynamics_type(row.type)
        wing = yaml_ref(row, :wing_idx)
        body = something(yaml_ref(row, :body_idx), yaml_ref(row, :body), Some(nothing))
        if type == BODY_STATIC
            isnothing(body) && isnothing(wing) && throw(ArgumentError(
                "point $name: BODY_STATIC requires a `body` or a `wing` to ride"))
            body = something(body, wing)
        elseif !isnothing(body)
            throw(ArgumentError("point $name: `body` is only valid with type BODY_STATIC"))
        end
        push!(points, Point(; name, type, body, pos_ENU=yaml_vec3(row, :pos_cad),
                            extra_mass=something(yaml_float(row, :extra_mass), 0.0),
                            drag_area=something(yaml_float(row, :area), 0.0),
                            drag_coefficient=something(yaml_float(row, :drag_coeff), 0.0),
                            model))
        push!(point_wings, wing)
        push!(point_transforms, yaml_ref(row, :transform_idx))
    end
    return points, point_wings, point_transforms
end

"""The segments of the `segments` block, their springs resolved against `set`."""
function read_segments(data, set)
    columns = (:name, :type, :point_i, :point_j, :l0, :compression_frac,
               :compression_damping_frac, MATERIAL_COLUMNS...)
    rows = read_rows(data, :segments, columns; modelled=false)
    return map(enumerate(rows)) do (i, (row, _))
        isnothing(yaml_field(row, :type)) || throw(ArgumentError(
            "the segment `type` column was removed; delete it"))
        spring_segment(yaml_row_name(row, i),
                       (yaml_to_ref(row.point_i), yaml_to_ref(row.point_j)), set;
                       l0=something(yaml_float(row, :l0), 0.0),
                       compression_frac=something(yaml_float(row, :compression_frac), 0.1),
                       compression_damping_frac=something(
                           yaml_float(row, :compression_damping_frac), 1.0),
                       material(row)...)
    end
end

"""The pulleys of the `pulleys` block."""
function read_pulleys(data)
    rows = read_rows(data, :pulleys, (:name, :segment_i, :segment_j, :type, :efficiency))
    return Pulley[Pulley(; name=yaml_row_name(row, i),
                         segments=(yaml_to_ref(row.segment_i), yaml_to_ref(row.segment_j)),
                         type=parse_dynamics_type(row.type),
                         efficiency=something(yaml_float(row, :efficiency), 0.95), model)
                  for (i, (row, model)) in enumerate(rows)]
end

"""Reference points as a row writes them: a weight per point reference."""
const AuthoredRefPoints = Vector{Pair{NameRef, Float64}}

"""
    AuthoringWing

What a row of the `wings` block says of its wing besides its name: whether it is a
particle wing, its stations and transform, and what places and weighs its body.
"""
struct AuthoringWing
    particle::Bool
    stations::Vector{NameRef}
    transform::Union{Nothing, NameRef}
    origin::Union{Nothing, AuthoredRefPoints}
    z_ref_points::Union{Nothing, NTuple{2, AuthoredRefPoints}}
    y_ref_points::Union{Nothing, NTuple{2, AuthoredRefPoints}}
    pos_cad::Union{Nothing, Vec3}
    extra_mass::Float64
    com::Union{Nothing, Vec3}
    unit_inertia::Union{Nothing, Mat3}
end

"""The wings of the `wings` block, each with what it says of its body."""
function read_wings(data)
    wings = Wing[]
    authored = AuthoringWing[]
    columns = (:name, :dynamics_type, :type, :mass, :stations, :transform_idx, :origin_idx,
               :z_ref_points, :y_ref_points, :pos_cad, :extra_mass, :com, :unit_inertia)
    for (i, (row, model)) in enumerate(read_rows(data, :wings, columns))
        name = yaml_row_name(row, i)
        dynamics = yaml_field(row, :dynamics_type)
        if isnothing(dynamics)
            dynamics = yaml_field(row, :type)
            isnothing(dynamics) && throw(ArgumentError(
                "wing $name is missing its `dynamics_type`"))
            @warn "Wing YAML field `type` is deprecated; rename to `dynamics_type`."
        end
        dynamics in ("PARTICLE_DYNAMICS", "RIGID_DYNAMICS") || throw(ArgumentError(
            "wing $name: dynamics_type $dynamics is not PARTICLE_DYNAMICS or " *
            "RIGID_DYNAMICS"))
        isnothing(yaml_field(row, :mass)) || throw(ArgumentError(
            "wing $name: the `mass` column was renamed to `extra_mass`"))
        origin = yaml_field(row, :origin_idx)
        push!(wings, Wing(; name, canopy_material=nothing, model))
        push!(authored, AuthoringWing(
            dynamics == "PARTICLE_DYNAMICS",
            yaml_to_ref.(something(yaml_field(row, :stations), [])),
            yaml_ref(row, :transform_idx),
            isnothing(origin) ? nothing : yaml_ref_points(origin),
            yaml_ref_point_pair(row, :z_ref_points),
            yaml_ref_point_pair(row, :y_ref_points),
            yaml_vec3(row, :pos_cad), something(yaml_float(row, :extra_mass), 0.0),
            yaml_vec3(row, :com), yaml_unit_inertia(row)))
    end
    return wings, authored
end

"""The stations of the `stations` block, each on the wing it names or the wing listing
it among its `stations`."""
function read_stations(data, wings, authored)
    rows = read_rows(data, :stations, (:name, :wing, :type, :points, :point_idxs))
    return map(enumerate(rows)) do (i, (row, model))
        name = yaml_row_name(row, i)
        wing = yaml_ref(row, :wing)
        if isnothing(wing)
            listing = findfirst(wing -> name in wing.stations || i in wing.stations,
                                authored)
            isnothing(listing) && throw(ArgumentError(
                "station $name names no wing and no wing lists it"))
            wing = wings[listing].name
        end
        points = something(yaml_field(row, :points), yaml_field(row, :point_idxs), [])
        Station(; name, wing, type=parse_dynamics_type(row.type),
                points=yaml_to_ref.(points), model)
    end
end

"""The canopy faces of the `canopy_faces` block, their wing and corners by name or
index."""
function read_canopy_faces(data)
    rows = read_rows(data, :canopy_faces, (:name, :wing, :points))
    return CanopyFace[CanopyFace(; name=yaml_row_name(row, i), wing=yaml_to_ref(row.wing),
                                 points=yaml_to_ref.(row.points), model)
                      for (i, (row, model)) in enumerate(rows)]
end

"""What a tether row gives of the length it starts at, refusing the removed
`init_unstretched_length`."""
function tether_init(row, name)
    isnothing(yaml_field(row, :init_unstretched_length)) || throw(ArgumentError(
        "tether $name: init_unstretched_length was removed; the rest length is derived " *
        "from init_stretched_length with init_tether_force or init_stretch_frac"))
    return TetherInit((yaml_float(row, :init_stretched_length),
                       yaml_float(row, :init_tether_force),
                       yaml_float(row, :init_stretch_frac)))
end

"""
    read_tethers!(points, segments, point_transforms, data, set)

The tethers of the `tethers` block and the length each starts at. A tether row naming its
`segment_idxs` runs over those segments, between the points they chain unless it names
them; one naming `n_segments` adds that many segments and the points between them, on the
straight line between its ends, to `points` and `segments`, its material resolved against
`set`.
"""
function read_tethers!(points, segments, point_transforms, data, set)
    tethers = Tether[]
    inits = TetherInit[]
    columns = (:name, :segment_idxs, :start_point, :end_point, :n_segments,
               :init_unstretched_length, :init_stretched_length, :init_tether_force,
               :init_stretch_frac, :compression_frac, :compression_damping_frac,
               MATERIAL_COLUMNS...)
    for (i, (row, model)) in enumerate(read_rows(data, :tethers, columns))
        name = yaml_row_name(row, i)
        push!(inits, tether_init(row, name))
        segment_refs = yaml_field(row, :segment_idxs)
        if !isnothing(segment_refs)
            refs = yaml_to_ref.(segment_refs)
            start_point = something(yaml_ref(row, :start_point),
                                    find_row(segments, first(refs), "segments").points[1])
            end_point = something(yaml_ref(row, :end_point),
                                  find_row(segments, last(refs), "segments").points[2])
            push!(tethers, Tether(; name, start_point, end_point, segments=refs, model))
            continue
        end
        start_point, end_point = yaml_to_ref(row.start_point), yaml_to_ref(row.end_point)
        push!(tethers, expand_tether!(points, segments, point_transforms, name, start_point,
                                      end_point, Int(row.n_segments), last(inits), model,
                                      set;
                                      compression_frac=something(
                                          yaml_float(row, :compression_frac), 0.1),
                                      compression_damping_frac=something(
                                          yaml_float(row, :compression_damping_frac), 1.0),
                                      material(row)...))
    end
    return tethers, inits
end

"""The tether `name` of `n` segments from `start_point` to `end_point`, its `n - 1` inner
points `<name>_point_<i>` added to `points` on the straight line between them, in the
transform of whichever end has one, and its segments `<name>_seg_<i>` to `segments`; the
tether carries `model`."""
function expand_tether!(points, segments, point_transforms, name, start_point, end_point,
                        n, init, model, set; spring...)
    ends = [start_point, end_point]
    indices = [ref isa Int ? ref : findfirst(point -> point.name == ref, points)
               for ref in ends]
    for (ref, index) in zip(ends, indices)
        isnothing(index) && throw(ArgumentError("tether $name: no point named $ref"))
    end
    transforms = unique(filter(!isnothing, point_transforms[indices]))
    length(transforms) <= 1 || throw(ArgumentError(
        "tether $name: its ends are in different transforms"))
    transform = isempty(transforms) ? nothing : only(transforms)
    start_pos, end_pos = points[indices[1]].pos_ENU, points[indices[2]].pos_ENU
    l0 = something(init.stretched_length, norm(end_pos - start_pos)) / n
    point_names = [start_point; ["$(name)_point_$i" for i in 1:(n - 1)]; end_point]
    for i in 1:(n - 1)
        push!(points, Point(; name=point_names[i + 1], type=DYNAMIC,
                            pos_ENU=start_pos + i / n * (end_pos - start_pos)))
        push!(point_transforms, transform)
    end
    segment_names = ["$(name)_seg_$i" for i in 1:n]
    for i in 1:n
        push!(segments, spring_segment(segment_names[i],
                                       (point_names[i], point_names[i + 1]), set;
                                       l0, spring...))
    end
    return Tether(; name, start_point, end_point, segments=segment_names, model)
end

"""The winches of the `winches` block, with `set`'s gear ratio and drum radius."""
function read_winches(data, set)
    rows = read_rows(data, :winches, (:name, :tether_idxs, :winch_point))
    return Winch[Winch(; name=yaml_row_name(row, i), tethers=yaml_to_ref.(row.tether_idxs),
                       winch_point=yaml_to_ref(row.winch_point), gear_ratio=set.gear_ratio,
                       drum_radius=set.drum_radius, model)
                 for (i, (row, model)) in enumerate(rows)]
end

"""The transforms of the `transforms` block, their angles in radians and references
unresolved, refusing one chained to another."""
function read_transforms(data)
    columns = (:name, :elevation, :azimuth, :heading, :base_pos, :base_point_idx,
               :base_transform_idx, :wing_idx, :rot_point_idx)
    rows = read_rows(data, :transforms, columns; modelled=false)
    return map(enumerate(rows)) do (i, (row, _))
        name = yaml_row_name(row, i)
        isnothing(yaml_field(row, :base_transform_idx)) || throw(ArgumentError(
            "transform $name is chained to another, which this loader does not place"))
        (; name, elevation=deg2rad(row.elevation), azimuth=deg2rad(row.azimuth),
         heading=deg2rad(row.heading), base_pos=Vec3(row.base_pos...),
         base_point=yaml_to_ref(row.base_point_idx), wing=yaml_ref(row, :wing_idx),
         rot_point=yaml_ref(row, :rot_point_idx))
    end
end

"""The plain bodies of the `bodies` block at their origins, each with its transform and
the centre of its own mass in its frame [m]."""
function read_bodies(data)
    bodies = Body[]
    authored = @NamedTuple{transform::Union{Nothing, NameRef}, com_offset::Vec3}[]
    columns = (:name, :mass, :extra_mass, :pos, :inertia, :inertia_principal, :type,
               :Q_b_to_w, :transform_idx, :com_offset_b)
    for (i, (row, model)) in enumerate(read_rows(data, :bodies, columns))
        name = yaml_row_name(row, i)
        isnothing(yaml_field(row, :mass)) || throw(ArgumentError(
            "body $name: the `mass` column was renamed to `extra_mass`"))
        extra_mass = yaml_float(row, :extra_mass)
        pos = yaml_vec3(row, :pos)
        (isnothing(extra_mass) || isnothing(pos)) && throw(ArgumentError(
            "body $name needs `extra_mass` and `pos`"))
        inertia = yaml_field(row, :inertia)
        principal = yaml_vec3(row, :inertia_principal)
        isnothing(inertia) == isnothing(principal) && throw(ArgumentError(
            "body $name: give one of `inertia` or `inertia_principal`"))
        type = parse_dynamics_type(something(yaml_field(row, :type), "DYNAMIC"))
        type in (DYNAMIC, STATIC) || throw(ArgumentError(
            "body $name: type must be DYNAMIC or STATIC, got $type"))
        push!(bodies, Body(; name, type, pos_ENU=pos, extra_mass,
                           Q_KA_to_ENU=something(yaml_field(row, :Q_b_to_w),
                                                 IDENTITY_QUATERNION),
                           extra_inertia_KA=isnothing(inertia) ? Mat3(Diagonal(principal)) :
                                            to_field(Mat3, inertia), model))
        push!(authored, (; transform=yaml_ref(row, :transform_idx),
                         com_offset=something(yaml_vec3(row, :com_offset_b), zero(Vec3))))
    end
    return bodies, authored
end

"""The tubes of the `tubes` block."""
function read_tubes(data)
    rows = read_rows(data, :tubes, (:name, :bodies, :diameter, :pressure, :law))
    return map(enumerate(rows)) do (i, (row, model))
        name = yaml_row_name(row, i)
        bodies = yaml_field(row, :bodies)
        (isnothing(bodies) || length(bodies) != 2) && throw(ArgumentError(
            "tube $name: `bodies` must name the two bodies it joins"))
        Tube(; name, bodies=yaml_to_ref.(bodies), diameter=yaml_float(row, :diameter),
             pressure=yaml_float(row, :pressure), law=String(row.law), model)
    end
end

# ==================== LOAD ==================== #

"""The blocks `load_authoring` reads, beside `variables`."""
const AUTHORING_BLOCKS = ("points", "segments", "pulleys", "tethers", "winches", "stations",
                          "wings", "canopy_faces", "transforms", "bodies", "tubes")

"""Whether the block `table` holds anything: a row, or a value that is no table."""
holds_rows(table) = !isnothing(table) &&
    !(table isa AbstractDict && isempty(something(get(table, "data", nothing), ())))

"""
    load_authoring(path; set::Settings, name, ignore_l0=false)

The `SystemDefinition` SymbolicAWEModels' authoring YAML at `path` describes, placed:
design positions moved by its tethers' stretched lengths and its `transforms`, so every
`pos_ENU` and `Q_KA_to_ENU` is the initial pose. `set` gives what a row leaves out:
segment material, and every winch's gear ratio and drum radius. `name` is the metadata
name, the file's by default; `ignore_l0` makes every rest length the placed length.
Refuses a block it does not read that holds rows.
"""
function load_authoring(path; set::Settings, name=first(splitext(basename(path))),
                        ignore_l0=false)
    data = resolve_yaml_variables(YAML.load_file(path))
    for key in ("materials", "elements", "segment_properties")
        haskey(data, key) && throw(ArgumentError(
            "the `$key` block was removed; define shared properties as a mapping under " *
            "`variables` and name its fields as columns"))
    end
    unread = sort!([key for (key, table) in data
                    if !(key in AUTHORING_BLOCKS) && holds_rows(table)])
    isempty(unread) || throw(ArgumentError(
        "blocks $(join(unread, ", ")) are read by nothing"))
    points, point_wings, point_transforms = read_points(data)
    segments = read_segments(data, set)
    tethers, inits = read_tethers!(points, segments, point_transforms, data, set)
    wings, authored_wings = read_wings(data)
    bodies, authored_bodies = read_bodies(data)
    wing_bodies = [Body(; name=wing.name, type=authored.particle ? KINEMATIC : DYNAMIC,
                        pos_ENU=something(authored.pos_cad, zero(Vec3)),
                        extra_mass=authored.particle ? 0.0 : authored.extra_mass,
                        extra_inertia_KA=zero(Mat3))
                   for (wing, authored) in zip(wings, authored_wings)]
    metadata = Metadata(name, "", "", AWESIO_VERSION, "structure_schema.yml",
                        length(points), "")
    draft = SystemDefinition(; metadata, points, segments,
                             stations=read_stations(data, wings, authored_wings),
                             pulleys=read_pulleys(data), tethers,
                             winches=read_winches(data, set), wings,
                             canopy_faces=read_canopy_faces(data),
                             bodies=[wing_bodies; bodies], tubes=read_tubes(data))
    placement = design_pose(draft, set, point_wings, point_transforms, authored_wings,
                            authored_bodies, inits, read_transforms(data))
    return placed_definition(place!(placement; ignore_l0))
end

"""Index of each component of `block` of `system` by name."""
indices_of(system, block) = name_indices(getfield(system, block), block)

"""`ref` resolved into the indices `names`, 0 for `nothing`."""
resolve_index(::Nothing, names, block) = 0
resolve_index(ref, names, block) = resolve(ref, block, NamedTuple{(block,)}((names,)))

"""`refs` resolved into point indices."""
resolve_ref_points(refs, names) = RefPoints([resolve_index(ref, names, :points) => weight
                                             for (ref, weight) in refs])

"""The frame `authored` gives its wing body in point indices, or `nothing` where its row
leaves out its origin or either pair of reference points."""
function authored_frame(authored, point_names)
    any(isnothing, (authored.origin, authored.z_ref_points, authored.y_ref_points)) &&
        return nothing
    return WingFrame(resolve_ref_points(authored.origin, point_names),
                     resolve_ref_points.(authored.z_ref_points, (point_names,)),
                     resolve_ref_points.(authored.y_ref_points, (point_names,)))
end

"""
    wing_body_pose(draft, authored, members, point_names)
        -> (origin, R, com_offset, inertia)

The design pose of a wing's body: its origin [m], its rotation into the world, the centre of
its own mass in its frame [m], and its own inertia about that centre in its axes [kg*m²],
`nothing` for none. A particle wing's body sits at its origin, framed by its reference
points where it has them; a rigid wing's is framed by its reference points, else at the
centre of its own mass ([`wing_inertia`](@ref)) in the world's axes.
"""
function wing_body_pose(draft, authored, members, point_names)
    pos = [point.pos_ENU for point in draft.points]
    frame = authored_frame(authored, point_names)
    if authored.particle
        isnothing(authored.origin) && throw(ArgumentError(
            "a particle wing needs an `origin_idx`"))
        R = isnothing(frame) ? Mat3(I) : first(wing_frame(pos, frame))
        origin = ref_position(pos, resolve_ref_points(authored.origin, point_names))
        return origin, R, zero(Vec3), nothing
    end
    com, inertia = wing_inertia(draft.points, members, authored)
    R, origin = isnothing(frame) ? (Mat3(I), com) : wing_frame(pos, frame)
    own_inertia = isnothing(inertia) ? nothing : R' * (authored.extra_mass * inertia) * R
    return origin, R, R' * (com - origin), own_inertia
end

"""
    design_pose(draft, set, point_wings, point_transforms, authored_wings, authored_bodies,
                inits, transforms)

The `Placement` of `draft` in its design pose: each wing body given its mass
([`wing_mass!`](@ref)) and posed by [`wing_body_pose`](@ref), each carried point anchored
in its body's frame, and every unset rest length the design length.
"""
function design_pose(draft, set, point_wings, point_transforms, authored_wings,
                     authored_bodies, inits, transforms)
    point_names, wing_names = indices_of(draft, :points), indices_of(draft, :wings)
    transform_names = name_indices(transforms, :transforms)
    point_wing = [resolve_index(ref, wing_names, :wings) for ref in point_wings]
    wing_node = falses(length(draft.points))
    foreach(station -> wing_node[Int.(station.points)] .= true, draft.stations)
    body_pos = [body.pos_ENU for body in draft.bodies]
    body_R = [quaternion_to_rotation_matrix(body.Q_KA_to_ENU) for body in draft.bodies]
    com_offset = [fill(zero(Vec3), length(draft.wings));
                  [body.com_offset for body in authored_bodies]]
    for (wing, authored) in enumerate(authored_wings)
        members = [index for (index, point) in enumerate(draft.points)
                   if (wing_node[index] || point.type == BODY_STATIC) &&
                      point_wing[index] == wing]
        wing_mass!(draft, wing, authored, members, set)
        isempty(members) && continue
        body_pos[wing], body_R[wing], com_offset[wing], inertia =
            wing_body_pose(draft, authored, members, point_names)
        isnothing(inertia) ||
            (draft.bodies[wing] = with_fields(draft.bodies[wing]; extra_inertia_KA=inertia))
    end
    pos = [point.pos_ENU for point in draft.points]
    anchor = [isnothing(point.body) ? zero(Vec3) :
              body_R[point.body]' * (pos[index] - body_pos[point.body])
              for (index, point) in enumerate(draft.points)]
    first_transform = isempty(transforms) ? nothing : 1
    body_transforms = [[something(wing.transform, Some(first_transform))
                        for wing in authored_wings];
                       [body.transform for body in authored_bodies]]
    frames = Union{Nothing, WingFrame}[
        [wing.particle ? authored_frame(wing, point_names) : nothing
         for wing in authored_wings]; fill(nothing, length(authored_bodies))]
    placed_transforms = [PlacedTransform(
        transform.elevation, transform.azimuth, transform.heading, transform.base_pos,
        resolve_index(transform.base_point, point_names, :points),
        resolve_index(transform.wing, wing_names, :wings),
        resolve_index(transform.rot_point, point_names, :points))
        for transform in transforms]
    placement = Placement(draft, pos, [segment.l0 for segment in draft.segments], body_pos,
                          body_R, com_offset, anchor,
                          resolve_index.(point_transforms, (transform_names,), :transforms),
                          resolve_index.(body_transforms, (transform_names,), :transforms),
                          point_wing, wing_node, frames, inits,
                          placed_transforms)
    update_segment_lengths!(placement)
    return placement
end

"""
    wing_mass!(draft, wing, authored, members, set)

Spread `set.mass` equally over the frame points `members` of `wing` where the wing carries
no mass of its own: for a rigid wing, neither its `extra_mass` nor its members'; for a
particle wing, neither its members nor the bodies they and its stations ride
([`particle_wing_parts`](@ref)). A particle wing's own `extra_mass` is ignored, warning.
"""
function wing_mass!(draft, wing, authored, members, set)
    name = draft.wings[wing].name
    masses = [draft.points[index].extra_mass for index in members]
    if authored.particle
        authored.extra_mass > 0 && @warn "Wing $name (PARTICLE_DYNAMICS): extra_mass " *
            "$(authored.extra_mass) is ignored; a particle wing carries its mass on its " *
            "points and section bodies."
        point_idxs, body_idxs = particle_wing_parts(draft, wing, members)
        own_mass = sum(draft.points[index].extra_mass for index in point_idxs; init=0.0) +
                   sum(draft.bodies[index].extra_mass for index in body_idxs; init=0.0)
        own_mass > 0 && return nothing
        if set.mass <= 0
            @warn "Wing $name (PARTICLE_DYNAMICS) has zero mass."
            return nothing
        end
    else
        iszero(authored.extra_mass) && set.mass > 0 && all(iszero, masses) || return nothing
    end
    for index in members
        draft.points[index] = with_fields(draft.points[index];
                                          extra_mass=set.mass / length(members))
    end
    return nothing
end

"""
    particle_wing_parts(draft, wing, members) -> (point_idxs, body_idxs)

The points of particle `wing` — its stations' points, else its frame points `members` —
and the other bodies sharing a tube-connected group with the bodies those points ride.
"""
function particle_wing_parts(draft, wing, members)
    point_idxs = Set{Int}()
    for station in draft.stations
        station.wing == wing && union!(point_idxs, station.points)
    end
    isempty(point_idxs) && union!(point_idxs, members)
    root = connected_body_groups(length(draft.bodies), draft.tubes)
    seeds = Set(root[draft.points[index].body] for index in point_idxs
                if !isnothing(draft.points[index].body) && draft.points[index].body != wing)
    body_idxs = [index for index in eachindex(draft.bodies)
                 if index > length(draft.wings) && root[index] in seeds]
    return point_idxs, body_idxs
end

"""
    wing_inertia(points, members, authored) -> (com, inertia)

Centre [m] and per-unit-mass inertia [m²] about it, in world axes, of a rigid wing's own
mass: the `com` and `unit_inertia` its row gives, else those of its frame points `members`
as point masses, `inertia` `nothing` where they carry no mass and `com` their centroid.
"""
function wing_inertia(points, members, authored)
    isnothing(authored.com) || isnothing(authored.unit_inertia) ||
        return authored.com, authored.unit_inertia
    masses = [points[index].extra_mass for index in members]
    positions = [points[index].pos_ENU for index in members]
    total = sum(masses)
    total > 0 || return sum(positions) / length(positions), nothing
    com = sum(masses .* positions) / total
    inertia = sum(mass * (dot(r, r) * I - r * r')
                  for (mass, r) in zip(masses, (position - com for position in positions)))
    return com, Mat3(inertia / total)
end

"""
    placed_definition(placement::Placement)

The `SystemDefinition` of `placement` where it stands: each point and body at its placed
position, a body at the centre of its own mass, and the metadata's `connectivity_sha` that
of its tables.
"""
function placed_definition(placement::Placement)
    (; system) = placement
    metadata = Metadata((field == :connectivity_sha ? connectivity_sha(system) :
                         getfield(system.metadata, field)
                         for field in fieldnames(Metadata))...)
    points = [with_fields(point; pos_ENU=placement.pos[index])
              for (index, point) in enumerate(system.points)]
    segments = [with_fields(segment; l0=placement.l0[index])
                for (index, segment) in enumerate(system.segments)]
    bodies = map(enumerate(system.bodies)) do (index, body)
        R = placement.body_R[index]
        mass_centre = placement.body_pos[index] + R * placement.com_offset[index]
        with_fields(body; pos_ENU=mass_centre, Q_KA_to_ENU=rotation_matrix_to_quaternion(R))
    end
    return SystemDefinition(metadata, points, segments, system.stations, system.pulleys,
                            system.tethers, system.winches, system.wings,
                            system.canopy_faces, bodies, system.tubes, system.extras)
end
