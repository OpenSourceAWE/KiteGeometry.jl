# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""The block each reference column of a table block refers into."""
const REFERENCES = Dict(
    (:points, :body) => :bodies,
    (:segments, :points) => :points,
    (:stations, :wing) => :wings,
    (:stations, :points) => :points,
    (:pulleys, :segments) => :segments,
    (:tethers, :start_point) => :points,
    (:tethers, :end_point) => :points,
    (:tethers, :segments) => :segments,
    (:winches, :tethers) => :tethers,
    (:winches, :winch_point) => :points,
    (:canopy_faces, :wing) => :wings,
    (:canopy_faces, :points) => :points,
    (:tubes, :bodies) => :bodies,
)

"""
    SystemDefinition(; metadata, points, segments, stations, pulleys, tethers, winches,
                     wings, canopy_faces, bodies, tubes, extras)

A system definition from its components, every reference resolved to the index of the
component it names. Absent blocks are empty; `extras` holds the blocks the schema does not
name. Refuses a `metadata.n_points` that is not the number of points.
"""
function SystemDefinition(; metadata::Metadata, extras=OrderedDict{String, Any}(),
                          components...)
    unknown = setdiff(keys(components), keys(BLOCKS))
    isempty(unknown) || throw(ArgumentError("no block named $(join(unknown, ", "))"))
    n_points = length(get(components, :points, ()))
    metadata.n_points == n_points ||
        throw(ArgumentError("metadata.n_points is $(metadata.n_points), not $n_points"))
    blocks = NamedTuple{keys(BLOCKS)}(
        convert(Vector{BLOCKS[block]}, get(components, block, BLOCKS[block][]))
        for block in keys(BLOCKS))
    indices = NamedTuple{keys(BLOCKS)}(name_indices(blocks[block], block)
                                       for block in keys(BLOCKS))
    resolved = ([resolve(component, block, indices) for component in blocks[block]]
                for block in keys(BLOCKS))
    return SystemDefinition(metadata, resolved..., extras)
end

"""The index of each component of `rows` by its name, refusing a name used twice."""
function name_indices(rows, block)
    indices = Dict{String, Int}()
    for (index, component) in enumerate(rows)
        haskey(indices, component.name) &&
            throw(ArgumentError("two $block are named $(component.name)"))
        indices[component.name] = index
    end
    return indices
end

"""`component` of `block` with every reference in it an index into `indices`' blocks."""
function resolve(component::Component, block, indices)
    T = typeof(component)
    return T((resolve(getfield(component, field), get(REFERENCES, (block, field), nothing),
                      indices) for field in fieldnames(T))...)
end
resolve(value, ::Nothing, indices) = value
resolve(::Nothing, target::Symbol, indices) = nothing
function resolve(refs::Union{Tuple, Vector}, target::Symbol, indices)
    return resolve.(refs, target, (indices,))
end
function resolve(name::String, target::Symbol, indices)
    haskey(indices[target], name) || throw(ArgumentError("no $target named $name"))
    return indices[target][name]
end
function resolve(index::Int, target::Symbol, indices)
    index in 1:length(indices[target]) ||
        throw(ArgumentError("no $target at index $index"))
    return index
end

"""
    SystemDefinition(document::AbstractDict)

The system definition a parsed awesIO structure document describes. A row reads the columns
of the model its `model` column registers and drops the other columns the schema does not
name; the blocks the schema does not name are kept as `extras`, in the order read.
"""
function SystemDefinition(document::AbstractDict)
    for block in REQUIRED_BLOCKS
        haskey(document, String(block)) || throw(ArgumentError("no $block block"))
    end
    fields = document["metadata"]
    metadata = Metadata((to_field(fieldtype(Metadata, field), fields[String(field)])
                         for field in fieldnames(Metadata))...)
    tables = (block => read_table(block, document[String(block)])
              for block in keys(BLOCKS) if haskey(document, String(block)))
    extras = OrderedDict{String, Any}(
        name => block for (name, block) in document
        if name != "metadata" && !haskey(BLOCKS, Symbol(name)))
    return SystemDefinition(; metadata, extras, tables...)
end

"""
    load_structure(path)

The `SystemDefinition` in the awesIO structure document at `path`.
"""
function load_structure(path)
    return SystemDefinition(YAML.load_file(path; dicttype=OrderedDict{String, Any}))
end

"""The names of the columns of `T` that the schema requires, in order."""
required_columns(T) = filter(!=(:model), fieldnames(T))

"""The components in the `headers`/`units`/`data` table of `block`."""
function read_table(block, table)
    T = BLOCKS[block]
    headers = table["headers"]
    units = get(table, "units", nothing)
    columns = required_columns(T)
    n = length(columns)
    length(headers) >= n && Symbol.(headers[1:n]) == collect(columns) ||
        throw(ArgumentError("$block headers must open with $(join(columns, ", "))"))
    !isnothing(units) && length(units) == length(headers) ||
        throw(ArgumentError("$block needs one unit per header"))
    Tuple(units[1:n]) == UNITS[block] ||
        throw(ArgumentError("$block units must open with $(join(UNITS[block], ", "))"))
    for row in table["data"]
        length(row) == length(headers) || throw(ArgumentError(
            "a $block row has $(length(row)) of $(length(headers)) cells"))
    end
    model_column = findfirst(==("model"), headers)
    return [T(; zip(columns, row[1:n])...,
              model=read_model(block, headers, units, row, model_column))
            for row in table["data"]]
end

"""The model of `row` of `block`, read from the columns named after its fields: `NoModel`
where `model_column` is `nothing` or its cell names no registered model."""
function read_model(block, headers, units, row, model_column)
    name = isnothing(model_column) ? nothing : row[model_column]
    model = get(MODELS, (block, name), nothing)
    isnothing(model) && return NoModel()
    return model.type((model_field(model.type, field, unit, headers, units, row)
                       for (field, unit) in zip(fieldnames(model.type), model.units))...)
end

"""The `field` of the model `M` in `row`, from the column of that name. Refuses a column
that is absent or not in the registered `unit`."""
function model_field(M, field, unit, headers, units, row)
    column = findfirst(==(String(field)), headers)
    isnothing(column) && throw(ArgumentError("model $M needs a $field column"))
    units[column] == unit ||
        throw(ArgumentError("the $field column of model $M must be in $unit"))
    return to_field(fieldtype(M, field), row[column])
end

"""
    structure_document(system::SystemDefinition)

The awesIO structure document of `system`, as `load_structure` reads it: references by
name, model columns after the schema's, extra blocks after the schema's, empty optional
blocks left out.
"""
function structure_document(system::SystemDefinition)
    metadata = system.metadata
    document = OrderedDict{String, Any}(
        "metadata" => OrderedDict{String, Any}(String(field) => getfield(metadata, field)
                                               for field in fieldnames(Metadata)))
    for block in keys(BLOCKS)
        rows = getfield(system, block)
        block in REQUIRED_BLOCKS || !isempty(rows) || continue
        document[String(block)] = write_table(system, block, rows)
    end
    return merge!(document, system.extras)
end

"""The `headers`/`units`/`data` table of the components `rows` of `block`: the schema's
columns, then the `model` column and the columns of each row's model where a row has one."""
function write_table(system, block, rows)
    columns = required_columns(BLOCKS[block])
    registrations = [model_registration(block, typeof(row.model)) for row in rows]
    model_units = OrderedDict{String, String}()
    for model in unique(filter(!isnothing, registrations))
        model_units["model"] = "-"
        for (field, unit) in zip(fieldnames(model.type), model.units)
            get!(model_units, String(field), unit) == unit || throw(ArgumentError(
                "two $block models give the column $field different units"))
        end
    end
    data = [Any[(document_value(getfield(row, column),
                                get(REFERENCES, (block, column), nothing), system)
                 for column in columns)...,
                (model_cell(row.model, registration, header)
                 for header in keys(model_units))...]
            for (row, registration) in zip(rows, registrations)]
    return OrderedDict{String, Any}(
        "headers" => [String.(columns)..., keys(model_units)...],
        "units" => [UNITS[block]..., values(model_units)...],
        "data" => data)
end

"""The cell of `model`, registered as `registration`, in the model column `header`: the
registered name for `model`, else the field of that name, `nothing` where it has none."""
function model_cell(model, registration, header)
    header == "model" && return isnothing(registration) ? nothing : registration.name
    hasfield(typeof(model), Symbol(header)) || return nothing
    return document_value(getfield(model, Symbol(header)), nothing, nothing)
end

"""`value` as a structure document writes it, a reference into `target` by name."""
document_value(value, ::Nothing, system) = value
document_value(value::DynamicsType, ::Nothing, system) = string(value)
document_value(value::SVector, ::Nothing, system) = collect(value)
document_value(value::SMatrix, ::Nothing, system) = [collect(row) for row in eachrow(value)]
document_value(::Nothing, target::Symbol, system) = nothing
document_value(index::Int, target::Symbol, system) = getfield(system, target)[index].name
document_value(name::String, target::Symbol, system) = name
function document_value(refs::Union{Tuple, Vector}, target::Symbol, system)
    return [document_value(ref, target, system) for ref in refs]
end
