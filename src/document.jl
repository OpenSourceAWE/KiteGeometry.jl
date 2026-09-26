# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""The block each reference column of a table block refers into."""
const REFERENCES = Dict(
    (:points, :body) => :bodies,
    (:segments, :points) => :points,
    (:stations, :points) => :points,
    (:pulleys, :segments) => :segments,
    (:tethers, :start_point) => :points,
    (:tethers, :end_point) => :points,
    (:tethers, :segments) => :segments,
    (:winches, :tethers) => :tethers,
    (:winches, :winch_point) => :points,
    (:tubes, :bodies) => :bodies,
)

"""
    SystemDefinition(; metadata, points, segments, stations, pulleys, tethers, winches,
                     bodies, tubes, extras)

A system definition from its components, every reference resolved to the index of the
component it names. Absent blocks are empty; `extras` holds blocks the schema does not name.
"""
function SystemDefinition(; metadata::Metadata, extras=OrderedDict{String, Any}(),
                          components...)
    unknown = setdiff(keys(components), keys(BLOCKS))
    isempty(unknown) || throw(ArgumentError("no block named $(join(unknown, ", "))"))
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

The system definition a parsed awesIO structure document describes, with the columns and
blocks the schema does not name kept as `extras`, in the order read.
"""
function SystemDefinition(document::AbstractDict)
    for block in REQUIRED_BLOCKS
        haskey(document, String(block)) || throw(ArgumentError("no $block block"))
    end
    fields = document["metadata"]
    metadata = Metadata((to_field(fieldtype(Metadata, field), fields[String(field)])
                         for field in fieldnames(Metadata))...)
    components = (block => read_table(BLOCKS[block], document[String(block)])
                  for block in keys(BLOCKS) if haskey(document, String(block)))
    extras = OrderedDict{String, Any}(
        name => block for (name, block) in document
        if name != "metadata" && !haskey(BLOCKS, Symbol(name)))
    return SystemDefinition(; metadata, extras, components...)
end

"""
    load_structure(path)

The `SystemDefinition` in the awesIO structure document at `path`.
"""
function load_structure(path)
    return SystemDefinition(YAML.load_file(path; dicttype=OrderedDict{String, Any}))
end

"""The names of the columns of `T` that the schema requires, in order."""
required_columns(T) = filter(!=(:extras), fieldnames(T))

"""The components of type `T` in a `headers`/`data` table."""
function read_table(T, table)
    headers = table["headers"]
    columns = required_columns(T)
    n = length(columns)
    length(headers) >= n && Symbol.(headers[1:n]) == collect(columns) ||
        throw(ArgumentError("$T headers must open with $(join(columns, ", "))"))
    rows = T[]
    for row in table["data"]
        length(row) == length(headers) ||
            throw(ArgumentError("a $T row has $(length(row)) of $(length(headers)) cells"))
        extras = OrderedDict{String, Any}(zip(headers[(n + 1):end], row[(n + 1):end]))
        push!(rows, T(; zip(columns, row[1:n])..., extras))
    end
    return rows
end

"""
    structure_document(system::SystemDefinition)

The awesIO structure document of `system`, as `load_structure` reads it: references by
name, extra columns after the schema's, extra blocks after the schema's, empty optional
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

"""The `headers`/`data` table of the components `rows` of `block`."""
function write_table(system, block, rows)
    columns = required_columns(BLOCKS[block])
    extra_headers = isempty(rows) ? String[] : collect(keys(first(rows).extras))
    data = [Any[(document_value(getfield(row, column),
                                get(REFERENCES, (block, column), nothing), system)
                 for column in columns)..., values(row.extras)...] for row in rows]
    return OrderedDict{String, Any}("headers" => [String.(columns)..., extra_headers...],
                                    "data" => data)
end

"""`value` as a structure document writes it, a reference into `target` by name."""
document_value(value, ::Nothing, system) = value
document_value(value::DynamicsType, ::Nothing, system) = string(value)
document_value(value::SVector, ::Nothing, system) = collect(value)
document_value(value::SMatrix, ::Nothing, system) = [collect(row) for row in eachrow(value)]
document_value(::Nothing, target::Symbol, system) = nothing
document_value(index::Int, target::Symbol, system) = getfield(system, target)[index].name
function document_value(refs::Union{Tuple, Vector}, target::Symbol, system)
    return [document_value(ref, target, system) for ref in refs]
end
