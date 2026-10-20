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
    yaml_variables(data) -> Dict

The variables of the `variables` block of `data`, each with the variables it refers to
substituted; none where it has no such block.
"""
function yaml_variables(data)
    raw = get(data, "variables", Dict())
    raw isa AbstractDict || error(
        "`variables` must be a mapping of name => value, got $(typeof(raw)).")
    variables = Dict{String, Any}()
    for name in keys(raw)
        resolve_variable!(variables, raw, String(name), String[])
    end
    return variables
end

"""
    resolve_yaml_variables(data, variables=yaml_variables(data)) -> data

Apply an optional top-level `variables` block to the rest of the YAML tree and
drop the block. A variable holding a number, string or list replaces any cell
written as its name; a variable holding a mapping is a multi-variable and fills
the columns it names at once. Variables may refer to other variables.
"""
function resolve_yaml_variables(data, variables=yaml_variables(data))
    haskey(data, "variables") || return data
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
    haskey(table, "data") || error("table is missing `data`")

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
        haskey(table, "headers") || error("table with array rows requires `headers`")
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

"""Optional reference field: an index or a name, `nothing` where unset or 0."""
function yaml_ref(row, field)
    ref = yaml_to_ref(yaml_field(row, field))
    return ref == 0 ? nothing : ref
end

"""A reference cell as a `NameRef`, `nothing` where it is unset."""
yaml_to_ref(value) = yaml_unset(value) ? nothing :
                     value isa Integer ? Int(value) : Symbol(value)

"""Name of a row: its `name` field, else its one-based index `i`, which an `idx` field must
repeat."""
function yaml_row_name(row, i)
    idx = yaml_field(row, :idx)
    isnothing(idx) || idx == i || error("row $i is numbered `idx` $idx")
    return Symbol(something(yaml_field(row, :name), i))
end

"""The dynamics types of SymbolicAWEModels."""
const DYNAMICS_TYPES = ("DYNAMIC", "STATIC", "BODY_STATIC", "KINEMATIC")

"""The dynamics type a cell names, upper case, refusing the removed `WING`."""
function parse_dynamics_type(text)
    name = uppercase(String(text))
    name == "WING" && error(
        "DynamicsType `WING` was removed: use `BODY_STATIC` (rigid wing, with " *
        "`body_idx` = the wing) or `DYNAMIC` (particle wing), and make the point " *
        "a member of one of the wing's stations.")
    name in DYNAMICS_TYPES || error("no dynamics type $name")
    return name
end

"""Reference points as a row writes them — one reference, a list averaged equally, or
`[reference, weight]` pairs — as weights per reference."""
function yaml_ref_points(value)
    value isa AbstractVector || return [yaml_to_ref(value) => 1.0]
    first(value) isa AbstractVector || return [yaml_to_ref(entry) => 1 / length(value)
                                               for entry in value]
    weights = [Float64(weight) for (_, weight) in value]
    total = sum(weights)
    total > 0 || error("reference point weights sum to $total")
    isapprox(total, 1.0; atol=1e-6) ||
        @warn "Reference point weights sum to $total, normalizing to 1.0"
    return [yaml_to_ref(ref) => weight / total
            for ((ref, _), weight) in zip(value, weights)]
end

"""The pair of reference points in `field` of the row of wing `name`, or `nothing`."""
function yaml_ref_point_pair(row, field, name)
    value = yaml_field(row, field)
    isnothing(value) && return nothing
    length(value) == 2 || error("wing $name: $field must hold two reference points")
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
    read_rows(data, block, columns; modelled=true, key=String(block)) -> [(row, model)]

The rows of the table `key` of `data`, none where it is absent or empty, each with the model
of `block` it reads ([`authoring_model`](@ref)) where `modelled`, else `NoModel`. Refuses a
filled cell in a column that neither the loader, which reads `columns` and `idx`, nor that
model reads.
"""
function read_rows(data, block, columns; modelled=true, key=String(block))
    table = get(data, key, nothing)
    rows = isnothing(table) || isnothing(get(table, "data", nothing)) ? NamedTuple[] :
           parse_table(table)
    models = [modelled ? authoring_model(block, row) : NoModel() for row in rows]
    unread = OrderedSet{String}()
    read_columns = (:idx, columns...)
    for (row, model) in zip(rows, models)
        headers = [String(column) for column in keys(row) if !(column in read_columns)]
        union!(unread, unread_columns(headers, [yaml_field(row, Symbol(header))
                                                for header in headers], model))
    end
    isempty(unread) || error(
        "$block columns $(join(unread, ", ")) are read by neither the loader nor a " *
        "registered model")
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
    name = hasfield(typeof(row), :model) ? yaml_field(row, :model) : missing
    return row_model(block, name) do M, field, _
        model_value(M, field, yaml_field(row, field))
    end
end

"""The model of a row the loader adds to `block`: the block's default with every column
unset, `NoModel` where that default needs a column filled."""
function added_row_model(block)
    M = model_type(block, default_model(block))
    all(field -> Nothing <: fieldtype(M, field), model_columns(M)) || return NoModel()
    return authoring_model(block, (;))
end

# ==================== MATERIAL ==================== #

"""
    resolve_material(label, set; diameter, unit_stiffness, unit_damping, density,
                     youngs_modulus, damping_per_stiffness)

`(diameter, stiffness, unit_damping, density)` of a segment from what its row gives, `NaN`
for what it leaves out: `stiffness` is its Young's modulus [Pa], from `youngs_modulus` or
from `unit_stiffness` [N] over its cross-section, or the name of the nonlinear law
`unit_stiffness` names. `damping_per_stiffness` [s] stands in for `unit_damping` [N*s], and
`set`'s `d_tether` [mm], `rho_tether`, `e_tether` and `rel_damping` for the rest. A
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
        (!linear || !isnan(unit_stiffness)) && error(
            "$label: give either `unit_stiffness` or `youngs_modulus`, not both")
        stiffness = youngs_modulus
    elseif linear
        stiffness = isnan(unit_stiffness) ? set.e_tether : unit_stiffness / area
    else
        stiffness = unit_stiffness
    end
    if !isnan(damping_per_stiffness)
        isnan(unit_damping) || error(
            "$label: give either `unit_damping` or `damping_per_stiffness`, not both")
        linear || error("$label: `damping_per_stiffness` needs a linear `unit_stiffness`")
        unit_damping = damping_per_stiffness * stiffness * area
    elseif isnan(unit_damping)
        linear || error("$label: give `unit_damping` for a nonlinear `unit_stiffness`")
        set.rel_damping == 0 &&
            @warn "$label: unit_damping is zero (no rel_damping in the settings)."
        unit_damping = set.rel_damping * stiffness * area
    end
    return Float64(diameter), stiffness, Float64(unit_damping), Float64(density)
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

"""The constants of SymbolicAWEModels' Breukels (2011) inflated tube."""
const BREUKELS2011 = OrderedDict{String, Any}(
    "C1" => 6582.82, "C2" => -272.43, "C3" => 40852.38, "C4" => 14.31,
    "C5" => 271865251.42, "C6" => 215.93, "C7" => 14021.79, "C8" => -589.05,
    "C9" => 322.55, "C10" => 0.0239, "C11" => 5.3833, "C12" => 0.0461, "C13" => 1467.0,
    "C14" => 40.908, "C15" => -191.8, "C16" => 47.406, "C17" => -17703.0,
    "C18" => 358.05, "C19" => 0.0918)

"""
    MaterialTable

The materials a load has named so far, and the variables of its YAML they may be named
after.
"""
struct MaterialTable
    rows::Vector{Material}
    variables::Dict{String, Any}
end

"""
    material!(materials::MaterialTable, name, law, parameters)

The name of the material of `law` and `parameters` among `materials`, added where none has
them as `name`, or as `name_2`, `name_3`, … where another material holds that name.
"""
function material!(materials::MaterialTable, name, law, parameters)
    for row in materials.rows
        row.law == law && row.parameters == parameters && return row.name
    end
    taken = Set(row.name for row in materials.rows)
    unique_name = Symbol(name)
    n = 1
    while unique_name in taken
        n += 1
        unique_name = Symbol(name, "_", n)
    end
    push!(materials.rows, Material(; name=unique_name, law, parameters))
    return unique_name
end

"""The name of the first multi-variable among `variables` whose fields hold `parameters`,
else `label`."""
function variable_name(variables, parameters, label)
    for name in sort!(collect(keys(variables)))
        fields = variables[name]
        fields isa AbstractDict || continue
        all(get(fields, key, nothing) == value for (key, value) in parameters) &&
            return name
    end
    return label
end

"""The name among `materials` ([`material!`](@ref)) of a line of `density` [kg/m^3] and
`stiffness`: a `linear` material of that Young's modulus [Pa], named after the
multi-variable giving both else `label`, or the nonlinear law of that name."""
function line_material!(materials, label, density, stiffness::Real)
    parameters = OrderedDict{String, Any}("density" => density,
                                          "youngs_modulus" => stiffness)
    return material!(materials, variable_name(materials.variables, parameters, label),
                     "linear", parameters)
end
function line_material!(materials, label, density, law::String)
    return material!(materials, law, law, OrderedDict{String, Any}("density" => density))
end

"""A segment with its spring resolved from `spring` ([`resolve_material`](@ref)) and `set`,
its material among `materials`; a zero `rest_length` is set from the design pose before
placement."""
function spring_segment(name, points, set, materials; rest_length=0.0,
                        compression_frac=0.1, compression_damping_frac=1.0, spring...)
    diameter, stiffness, unit_damping, density =
        resolve_material("segment $name", set; spring...)
    return Segment(; name, points, rest_length, diameter,
                   material=line_material!(materials, name, density, stiffness),
                   model=SegmentSpring(unit_damping, compression_frac,
                                       compression_damping_frac))
end

# ==================== BLOCKS ==================== #

"""The component `ref` names among the `rows` of `block` for `owner`: by name, or by its
one-based index."""
function find_row(rows, ref, block, owner)
    index = ref isa Int ? (ref in eachindex(rows) ? ref : nothing) :
            findfirst(row -> row.name == ref, rows)
    isnothing(index) && error("$owner: no $block $ref")
    return rows[index]
end

"""The points of the `points` block, each with the wing and transform it names."""
function read_points(data)
    points = Point[]
    point_wings = Union{Nothing, NameRef}[]
    point_transforms = Union{Nothing, NameRef}[]
    columns = (:name, :type, :wing_idx, :body_idx, :body, :pos_cad, :extra_mass,
               :transform_idx)
    for (i, (row, model)) in enumerate(read_rows(data, :points, columns))
        name = yaml_row_name(row, i)
        type = parse_dynamics_type(row.type)
        wing = yaml_ref(row, :wing_idx)
        body = something(yaml_ref(row, :body_idx), yaml_ref(row, :body), Some(nothing))
        if type == "BODY_STATIC"
            isnothing(body) && isnothing(wing) && error(
                "point $name: BODY_STATIC requires a `body` or a `wing` to ride")
            body = something(body, wing)
        elseif !isnothing(body)
            error("point $name: `body` is only valid with type BODY_STATIC")
        end
        push!(points, Point(; name, fixed=type == "STATIC", body,
                            pos_ENU=yaml_vec3(row, :pos_cad),
                            extra_mass=something(yaml_float(row, :extra_mass), 0.0), model))
        push!(point_wings, wing)
        push!(point_transforms, yaml_ref(row, :transform_idx))
    end
    return points, point_wings, point_transforms
end

"""The segments of the `segments` block, their springs resolved against `set` and their
materials among `materials`."""
function read_segments(data, set, materials)
    columns = (:name, :type, :point_i, :point_j, :l0, :compression_frac,
               :compression_damping_frac, MATERIAL_COLUMNS...)
    rows = read_rows(data, :segments, columns; modelled=false)
    return map(enumerate(rows)) do (i, (row, _))
        isnothing(yaml_field(row, :type)) || error(
            "the segment `type` column was removed; delete it")
        spring_segment(yaml_row_name(row, i),
                       (yaml_to_ref(row.point_i), yaml_to_ref(row.point_j)), set, materials;
                       rest_length=something(yaml_float(row, :l0), 0.0),
                       compression_frac=something(yaml_float(row, :compression_frac), 0.1),
                       compression_damping_frac=something(
                           yaml_float(row, :compression_damping_frac), 1.0),
                       material(row)...)
    end
end

"""The pulleys of the `pulleys` block."""
function read_pulleys(data)
    rows = read_rows(data, :pulleys, (:name, :segment_i, :segment_j))
    return Pulley[Pulley(; name=yaml_row_name(row, i),
                         segments=(yaml_to_ref(row.segment_i), yaml_to_ref(row.segment_j)),
                         model)
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
            isnothing(dynamics) && error("wing $name is missing its `dynamics_type`")
            @warn "Wing YAML field `type` is deprecated; rename to `dynamics_type`."
        end
        dynamics in ("PARTICLE_DYNAMICS", "RIGID_DYNAMICS") || error(
            "wing $name: dynamics_type $dynamics is not PARTICLE_DYNAMICS or " *
            "RIGID_DYNAMICS")
        isnothing(yaml_field(row, :mass)) || error(
            "wing $name: the `mass` column was renamed to `extra_mass`")
        origin = yaml_field(row, :origin_idx)
        dynamics == "PARTICLE_DYNAMICS" && isnothing(origin) &&
            error("particle wing $name needs an `origin_idx`")
        push!(wings, Wing(; name, pos_ENU=zero(Vec3), model))
        push!(authored, AuthoringWing(
            dynamics == "PARTICLE_DYNAMICS",
            yaml_to_ref.(something(yaml_field(row, :stations), [])),
            yaml_ref(row, :transform_idx),
            isnothing(origin) ? nothing : yaml_ref_points(origin),
            yaml_ref_point_pair(row, :z_ref_points, name),
            yaml_ref_point_pair(row, :y_ref_points, name),
            yaml_vec3(row, :pos_cad), something(yaml_float(row, :extra_mass), 0.0),
            yaml_vec3(row, :com), yaml_unit_inertia(row)))
    end
    return wings, authored
end

"""The spanwise stations of the `stations` block, each on the wing it names or the wing
listing it among its `stations`."""
function read_stations(data, wings, authored)
    rows = read_rows(data, :spanwise_stations, (:name, :wing, :points, :point_idxs);
                     key="stations")
    return map(enumerate(rows)) do (i, (row, model))
        name = yaml_row_name(row, i)
        wing = yaml_ref(row, :wing)
        if isnothing(wing)
            listing = findfirst(wing -> name in wing.stations || i in wing.stations,
                                authored)
            isnothing(listing) && error("station $name names no wing and no wing lists it")
            wing = wings[listing].name
        end
        points = something(yaml_field(row, :points), yaml_field(row, :point_idxs), [])
        SpanwiseStation(; name, wing, points=yaml_to_ref.(points), model)
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
    isnothing(yaml_field(row, :init_unstretched_length)) || error(
        "tether $name: init_unstretched_length was removed; the rest length is derived " *
        "from init_stretched_length with init_tether_force or init_stretch_frac")
    return TetherInit((yaml_float(row, :init_stretched_length),
                       yaml_float(row, :init_tether_force),
                       yaml_float(row, :init_stretch_frac)))
end

"""
    read_tethers!(points, segments, point_transforms, data, set, materials)

The tethers of the `tethers` block and the length each starts at. A tether row naming its
`segment_idxs` runs over those segments, between the points they chain unless it names
them; one naming `n_segments` adds that many segments and the points between them, on the
straight line between its ends, to `points` and `segments`, its material resolved against
`set` and named among `materials`.
"""
function read_tethers!(points, segments, point_transforms, data, set, materials)
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
            first_segment = find_row(segments, first(refs), "segments", "tether $name")
            last_segment = find_row(segments, last(refs), "segments", "tether $name")
            start_point = something(yaml_ref(row, :start_point), first_segment.points[1])
            end_point = something(yaml_ref(row, :end_point), last_segment.points[2])
            push!(tethers, Tether(; name, start_point, end_point, segments=refs, model))
            continue
        end
        start_point, end_point = yaml_to_ref(row.start_point), yaml_to_ref(row.end_point)
        push!(tethers, expand_tether!(points, segments, point_transforms, name, start_point,
                                      end_point, Int(row.n_segments), last(inits), set,
                                      materials;
                                      model,
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
                        n, init, set, materials; model, spring...)
    ends = [start_point, end_point]
    indices = [ref isa Int ? ref : findfirst(point -> point.name == ref, points)
               for ref in ends]
    for (ref, index) in zip(ends, indices)
        isnothing(index) && error("tether $name: no point named $ref")
    end
    transforms = unique(filter(!isnothing, point_transforms[indices]))
    length(transforms) <= 1 || error("tether $name: its ends are in different transforms")
    transform = isempty(transforms) ? nothing : only(transforms)
    start_pos, end_pos = points[indices[1]].pos_ENU, points[indices[2]].pos_ENU
    rest_length = something(init.stretched_length, norm(end_pos - start_pos)) / n
    point_names = [start_point; ["$(name)_point_$i" for i in 1:(n - 1)]; end_point]
    for i in 1:(n - 1)
        push!(points, Point(; name=point_names[i + 1], extra_mass=0.0,
                            pos_ENU=start_pos + i / n * (end_pos - start_pos),
                            model=added_row_model(:points)))
        push!(point_transforms, transform)
    end
    segment_names = ["$(name)_seg_$i" for i in 1:n]
    for i in 1:n
        push!(segments, spring_segment(segment_names[i],
                                       (point_names[i], point_names[i + 1]), set, materials;
                                       rest_length, spring...))
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

"""The angle a transform row `name` gives in degrees in `field`, in radians: `default`
where it is unset, refusing an unset one without."""
function yaml_radians(row, field, name; default=nothing)
    degrees = something(yaml_float(row, field), default, Some(nothing))
    isnothing(degrees) && error("transform $name needs its $field")
    return deg2rad(degrees)
end

"""The rows of the `transforms` block, their angles read in degrees, references by name."""
function read_transforms(data)
    columns = (:name, :elevation, :azimuth, :heading, :elevation_vel, :azimuth_vel,
               :turn_rate, :base_pos, :base_point_idx, :base_transform_idx, :wing_idx,
               :rot_point_idx)
    rows = read_rows(data, :transforms, columns; modelled=false)
    return map(enumerate(rows)) do (i, (row, _))
        name = yaml_row_name(row, i)
        (; name, elevation=yaml_radians(row, :elevation, name),
         azimuth=yaml_radians(row, :azimuth, name),
         heading=yaml_radians(row, :heading, name),
         elevation_vel=yaml_radians(row, :elevation_vel, name; default=0.0),
         azimuth_vel=yaml_radians(row, :azimuth_vel, name; default=0.0),
         turn_rate=yaml_radians(row, :turn_rate, name; default=0.0),
         base_pos=yaml_vec3(row, :base_pos), base_point=yaml_ref(row, :base_point_idx),
         base_transform=yaml_ref(row, :base_transform_idx), wing=yaml_ref(row, :wing_idx),
         rot_point=yaml_ref(row, :rot_point_idx))
    end
end

"""The plain bodies of the `bodies` block at their origins, each with its transform, the
centre of its own mass in its frame [m], and whether it is `STATIC`."""
function read_bodies(data)
    bodies = Body[]
    authored = @NamedTuple{transform::Union{Nothing, NameRef}, com_offset::Vec3,
                           static::Bool}[]
    columns = (:name, :mass, :extra_mass, :pos, :inertia, :inertia_principal, :type,
               :Q_b_to_w, :transform_idx, :com_offset_b)
    for (i, (row, model)) in enumerate(read_rows(data, :bodies, columns))
        name = yaml_row_name(row, i)
        isnothing(yaml_field(row, :mass)) || error(
            "body $name: the `mass` column was renamed to `extra_mass`")
        extra_mass = yaml_float(row, :extra_mass)
        pos = yaml_vec3(row, :pos)
        (isnothing(extra_mass) || isnothing(pos)) && error(
            "body $name needs `extra_mass` and `pos`")
        inertia = yaml_field(row, :inertia)
        principal = yaml_vec3(row, :inertia_principal)
        isnothing(inertia) == isnothing(principal) && error(
            "body $name: give one of `inertia` or `inertia_principal`")
        type = parse_dynamics_type(something(yaml_field(row, :type), "DYNAMIC"))
        type in ("DYNAMIC", "STATIC") || error(
            "body $name: type must be DYNAMIC or STATIC, got $type")
        rotation = something(yaml_field(row, :Q_b_to_w), IDENTITY_QUATERNION)
        push!(bodies, Body(; name, pos_ENU=pos, extra_mass, Q_KA2ENU=rotation,
                           extra_inertia_KA=isnothing(inertia) ? Mat3(Diagonal(principal)) :
                                            to_field(Mat3, inertia), model))
        push!(authored, (; transform=yaml_ref(row, :transform_idx),
                         com_offset=something(yaml_vec3(row, :com_offset_b), zero(Vec3)),
                         static=type == "STATIC"))
    end
    return bodies, authored
end

"""The tubes of the `tubes` block, each made of the material of its `law` among
`materials`: SymbolicAWEModels' constants for `breukels2011`, no parameters for another."""
function read_tubes(data, materials)
    rows = read_rows(data, :tubes, (:name, :bodies, :diameter, :pressure, :law))
    return map(enumerate(rows)) do (i, (row, model))
        name = yaml_row_name(row, i)
        bodies = yaml_field(row, :bodies)
        (isnothing(bodies) || length(bodies) != 2) && error(
            "tube $name: `bodies` must name the two bodies it joins")
        law = String(row.law)
        parameters = law == "breukels2011" ? copy(BREUKELS2011) : OrderedDict{String, Any}()
        Tube(; name, bodies=yaml_to_ref.(bodies), diameter=yaml_float(row, :diameter),
             pressure=yaml_float(row, :pressure),
             material=material!(materials, law, law, parameters), model)
    end
end

# ==================== LOAD ==================== #

"""The blocks `load_authoring` reads."""
const AUTHORING_BLOCKS = ("variables", "points", "segments", "pulleys", "tethers",
                          "winches", "stations", "wings", "canopy_faces", "transforms",
                          "bodies", "tubes")

"""Whether the block `table` holds anything: any value but a table without rows."""
function holds_rows(table)
    isnothing(table) && return false
    table isa AbstractDict && haskey(table, "headers") || return true
    return !isempty(something(get(table, "data", nothing), ()))
end

"""
    load_authoring(path; set::Settings, name, ignore_l0=false) -> (system, transforms)
    load_authoring(authored::AbstractDict; set::Settings, name, ignore_l0=false)

The `SystemDefinition` SymbolicAWEModels' authoring YAML at `path` describes, or the parsed
authoring document `authored`, and its `Transform`s, as `Rows`. The definition is placed:
design positions moved by its tethers' stretched lengths and then by the transforms, each
wing's KA frame the authored frame moved with its body. Each transform holds the points and
bodies it moves: those naming it, a tether's inner points that of its ends, and a wing's
body its wing's, else the first transform.
`set` gives what a row leaves out: segment material, and every winch's gear ratio and drum
radius. Each distinct line material is named after the multi-variable giving it, else
after its first segment.
`name` is the metadata name, the file's by default for a path; `ignore_l0` makes every rest
length the placed length. Each non-empty block it does not read is kept as an extra block;
one named after a block of the structure document, or holding a single value, is refused.
"""
function load_authoring(path::AbstractString; set::Settings,
                        name=first(splitext(basename(path))), ignore_l0=false)
    authored = YAML.load_file(path; dicttype=OrderedDict{String, Any})
    return load_authoring(authored; set, name, ignore_l0)
end

function load_authoring(authored::AbstractDict; set::Settings, name::AbstractString,
                        ignore_l0=false)
    variables = yaml_variables(authored)
    data = resolve_yaml_variables(authored, variables)
    for key in ("materials", "elements", "segment_properties")
        haskey(data, key) && error(
            "the `$key` block was removed; define shared properties as a mapping under " *
            "`variables` and name its fields as columns")
    end
    extra_blocks = filter(key -> !(key in AUTHORING_BLOCKS) && holds_rows(data[key]),
                          sort!(collect(keys(data))))
    for key in extra_blocks
        data[key] isa Union{AbstractDict, AbstractVector} ||
            error("$key: $(repr(data[key])) is not a block")
        (key == "metadata" || haskey(BLOCKS, Symbol(key))) &&
            error("block $key is a block of the structure document, not of an authoring " *
                  "YAML")
    end
    materials = MaterialTable(Material[], variables)
    points, point_wings, point_transforms = read_points(data)
    segments = read_segments(data, set, materials)
    tethers, inits = read_tethers!(points, segments, point_transforms, data, set, materials)
    wings, authored_wings = read_wings(data)
    bodies, authored_bodies = read_bodies(data)
    tubes = read_tubes(data, materials)
    wing_bodies = [Body(; name=wing.name, pos_ENU=something(authored.pos_cad, zero(Vec3)),
                        extra_mass=authored.particle ? 0.0 : authored.extra_mass,
                        extra_inertia_KA=zero(Mat3), model=added_row_model(:bodies))
                   for (wing, authored) in zip(wings, authored_wings)]
    metadata = Metadata(; name, n_points=length(points))
    authored_transforms = read_transforms(data)
    first_transform = isempty(authored_transforms) ? nothing : 1
    body_transforms = [[something(wing.transform, Some(first_transform))
                        for wing in authored_wings];
                       [body.transform for body in authored_bodies]]
    extras = OrderedDict{String, Any}(key => data[key] for key in extra_blocks)
    draft = SystemDefinition(; metadata, materials=materials.rows, points, segments,
                             spanwise_stations=read_stations(data, wings, authored_wings),
                             pulleys=read_pulleys(data), tethers,
                             winches=read_winches(data, set), wings,
                             canopy_faces=read_canopy_faces(data),
                             bodies=[wing_bodies; bodies], tubes, extras)
    transforms = resolve_transforms(authored_transforms, draft, authored_wings,
                                    point_transforms, body_transforms)
    placement = design_pose(draft, set, point_wings, authored_wings, authored_bodies, inits)
    design = copy(Pose(placement))
    place_authored!(placement, design, transforms; ignore_l0)
    return placed_definition(placement, design), Rows{Transform}(transforms)
end

"""`ref` resolved into the indices `names`, `nothing` for `nothing`."""
function optional_index(ref, names, block)
    return isnothing(ref) ? nothing : resolve_index(ref, names, block)
end

"""
    resolve_transforms(authored, draft, authored_wings, point_transforms, body_transforms)

The `Transform` of each authored transform row on `draft`, its references indices. Its
members are the points and bodies whose transform in `point_transforms` and
`body_transforms` names it. A transform naming a wing turns that wing's origin and heads
its body; one naming a `rot_point` turns that point and heads its first member body.
"""
function resolve_transforms(authored, draft, authored_wings, point_transforms,
                            body_transforms)
    names = name_indices(authored, :transforms)
    point_names, wing_names = indices_of(draft, :points), indices_of(draft, :wings)
    return map(enumerate(authored)) do (i, row)
        points = transform_members(point_transforms, names, i)
        bodies = transform_members(body_transforms, names, i)
        wing = optional_index(row.wing, wing_names, :wings)
        rot_point, body = if isnothing(wing)
            optional_rot_point(row.rot_point, point_names),
            isempty(bodies) ? nothing : first(bodies)
        else
            wing_origin(authored_wings[wing], point_names), wing
        end
        Transform(; row.name, row.elevation, row.azimuth, row.heading, row.elevation_vel,
                  row.azimuth_vel, row.turn_rate, row.base_pos,
                  base_point=optional_index(row.base_point, point_names, :points),
                  base_transform=optional_index(row.base_transform, names, :transforms),
                  rot_point, body, points, bodies)
    end
end

"""The point `ref` as reference points of weight one, `nothing` for `nothing`."""
function optional_rot_point(ref, point_names)
    return isnothing(ref) ? nothing : [resolve_index(ref, point_names, :points) => 1.0]
end

"""The indices whose transform in `refs`, resolved into the indices `names`, is `i`."""
function transform_members(refs, names, i)
    return [index for (index, ref) in enumerate(refs)
            if !isnothing(ref) && resolve_index(ref, names, :transforms) == i]
end

"""The reference points of the origin of the body of the wing `authored`, `nothing` where
that origin is the centre of its mass."""
function wing_origin(authored, point_names)
    authored.particle || !isnothing(authored_frame(authored, point_names)) ||
        return nothing
    return resolve_ref_points(authored.origin, point_names)
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
points where it has them; a rigid wing's is framed by its reference points,
else at the centre of its own mass ([`wing_inertia`](@ref)) in the world's axes.
"""
function wing_body_pose(draft, authored, members, point_names)
    pos = [point.pos_ENU for point in draft.points]
    frame = authored_frame(authored, point_names)
    if authored.particle
        R = isnothing(frame) ? Mat3(I) : first(wing_frame(pos, frame))
        origin = ref_position(pos, resolve_ref_points(authored.origin, point_names))
        return origin, R, zero(Vec3), nothing
    end
    com, inertia = wing_inertia(draft.points, members, authored)
    R, origin = isnothing(frame) ? (Mat3(I), com) : wing_frame(pos, frame)
    own_inertia = isnothing(inertia) ? nothing : R' * (authored.extra_mass * inertia) * R
    return origin, R, R' * (com - origin), own_inertia
end

"""The index of the wing `ref` names for `point`, 0 for none, refusing one that does not
exist."""
function point_wing_index(point, ref, wing_names)
    isnothing(ref) && return 0
    index = ref isa Int ? (ref in values(wing_names) ? ref : nothing) :
            get(wing_names, ref, nothing)
    isnothing(index) && error("point $(point.name): wing $ref does not exist")
    return index
end

"""
    point_wing_indices(draft, point_wings, wing_names) -> Vector{Int}

The index of the wing each point of `draft` names in `point_wings`, none for a point a
tether adds ([`point_wing_index`](@ref)). Refuses a station's point that names another wing
than the station's, or none.
"""
function point_wing_indices(draft, point_wings, wing_names)
    point_wing = [point_wing_index(point, get(point_wings, index, nothing), wing_names)
                  for (index, point) in enumerate(draft.points)]
    for station in draft.spanwise_stations, index in station.points
        point_wing[index] == station.wing && continue
        named = point_wing[index] == 0 ? "no wing" : draft.wings[point_wing[index]].name
        error("point $(draft.points[index].name) is a point of station $(station.name) " *
              "of wing $(draft.wings[station.wing].name), but names $named")
    end
    return point_wing
end

"""
    design_pose(draft, set, point_wings, authored_wings, authored_bodies, inits)

The `Placement` of `draft` in its design pose: each wing body given its mass
([`wing_mass!`](@ref)) and posed by [`wing_body_pose`](@ref), each carried point anchored
in its body's frame, and every unset rest length the design length.
"""
function design_pose(draft, set, point_wings, authored_wings, authored_bodies, inits)
    point_names, wing_names = indices_of(draft, :points), indices_of(draft, :wings)
    point_wing = point_wing_indices(draft, point_wings, wing_names)
    wing_node = falses(length(draft.points))
    foreach(station -> wing_node[Int.(station.points)] .= true, draft.spanwise_stations)
    body_pos = [body.pos_ENU for body in draft.bodies]
    body_R = [quaternion_to_rotation_matrix(body.Q_KA2ENU) for body in draft.bodies]
    com_offset = [fill(zero(Vec3), length(draft.wings));
                  [body.com_offset for body in authored_bodies]]
    for (wing, authored) in enumerate(authored_wings)
        members = [index for (index, point) in enumerate(draft.points)
                   if (wing_node[index] || !isnothing(point.body)) &&
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
    static_body = [fill(false, length(authored_wings));
                   [body.static for body in authored_bodies]]
    placement = Placement(draft, pos, [segment.rest_length for segment in draft.segments],
                          body_pos, body_R, com_offset, anchor, point_wing, wing_node,
                          static_body, [wing.particle for wing in authored_wings], inits)
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
    for station in draft.spanwise_stations
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
    placed_definition(placement::Placement, design::Pose)

The `SystemDefinition` of `placement` where it stands: each point and body at its placed
position, a body at the centre of its own mass, each wing's KA frame the authored frame
moved as its body moved from `design`, and the metadata's `connectivity_sha` that of its
tables.
"""
function placed_definition(placement::Placement, design::Pose)
    (; system) = placement
    metadata = Metadata((field == :connectivity_sha ? connectivity_sha(system) :
                         getfield(system.metadata, field)
                         for field in fieldnames(Metadata))...)
    points = [with_fields(point; pos_ENU=placement.pos[index])
              for (index, point) in enumerate(system.points)]
    segments = [with_fields(segment; rest_length=placement.l0[index])
                for (index, segment) in enumerate(system.segments)]
    bodies = map(enumerate(system.bodies)) do (index, body)
        R = placement.body_R[index]
        mass_centre = placement.body_pos[index] + R * placement.com_offset[index]
        with_fields(body; pos_ENU=mass_centre, Q_KA2ENU=rotation_matrix_to_quaternion(R))
    end
    placed = SystemDefinition(metadata, system.materials, points, segments,
                              system.spanwise_stations, system.pulleys, system.tethers,
                              system.winches, system.wings,
                              system.canopy_faces, bodies, system.tubes, system.extras)
    return carry_wings!(placed, design, Pose(placement))
end
