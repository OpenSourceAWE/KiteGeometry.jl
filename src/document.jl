# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""The block each reference column of a block refers into."""
const REFERENCES = Dict(
    (:system_tethers, :material) => :materials,
    (:points, :body) => :bodies,
    (:segments, :points) => :points,
    (:segments, :material) => :materials,
    (:spanwise_stations, :wing) => :wings,
    (:spanwise_stations, :points) => :points,
    (:pulleys, :segments) => :segments,
    (:tethers, :start_point) => :points,
    (:tethers, :end_point) => :points,
    (:tethers, :segments) => :segments,
    (:winches, :tethers) => :tethers,
    (:winches, :winch_point) => :points,
    (:wings, :canopy_material) => :materials,
    (:canopy_faces, :wing) => :wings,
    (:canopy_faces, :points) => :points,
    (:tubes, :bodies) => :bodies,
    (:tubes, :material) => :materials,
)

"""
    Metadata(; name, n_points, description="", note="", awesIO_version=AWESIO_VERSION,
             schema="structure_schema.yml", connectivity_sha="", system="", system_sha="")

Metadata from one keyword argument per field. An empty `connectivity_sha`, `system` or
`system_sha` stands for one not yet computed or written.
"""
function Metadata(; name, n_points, description="", note="", awesIO_version=AWESIO_VERSION,
                  schema="structure_schema.yml", connectivity_sha="", system="",
                  system_sha="")
    return Metadata(name, description, note, awesIO_version, schema, n_points,
                    connectivity_sha, system, system_sha)
end

"""
    SystemMetadata(; name, description="", note="", awesIO_version=AWESIO_VERSION,
                   schema="system_schema.yml")

The metadata of a system document from one keyword argument per field.
"""
function SystemMetadata(; name, description="", note="", awesIO_version=AWESIO_VERSION,
                        schema="system_schema.yml")
    return SystemMetadata(name, description, note, awesIO_version, schema)
end

"""
    SystemDefinition(; metadata, system_metadata, materials, system_tethers, points,
                     segments, spanwise_stations, pulleys, tethers, winches, wings,
                     canopy_faces, bodies, tubes, extras, system_extras)

A system definition from its components, every reference resolved to the index of the
component it names. Absent blocks are empty; `system_metadata` defaults to the name of
`metadata`; `extras` and `system_extras` hold what the schemas do not name. Refuses a
`metadata.n_points` that is not the number of points.
"""
function SystemDefinition(; metadata::Metadata,
                          system_metadata=SystemMetadata(; name=metadata.name),
                          extras=OrderedDict{String, Any}(),
                          system_extras=OrderedDict{String, Any}(), components...)
    unknown = setdiff(keys(components), keys(ALL_BLOCKS))
    isempty(unknown) || throw(ArgumentError("no block named $(join(unknown, ", "))"))
    n_points = length(get(components, :points, ()))
    metadata.n_points == n_points ||
        throw(ArgumentError("metadata.n_points is $(metadata.n_points), not $n_points"))
    blocks = NamedTuple{keys(ALL_BLOCKS)}(
        convert(Vector{ALL_BLOCKS[block]}, get(components, block, ALL_BLOCKS[block][]))
        for block in keys(ALL_BLOCKS))
    indices = NamedTuple{keys(ALL_BLOCKS)}(name_indices(blocks[block], block)
                                           for block in keys(ALL_BLOCKS))
    resolved = ([resolve(component, block, indices) for component in blocks[block]]
                for block in keys(ALL_BLOCKS))
    return SystemDefinition(metadata, system_metadata, resolved..., extras, system_extras)
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
function resolve(name::Symbol, target::Symbol, indices)
    haskey(indices[target], name) || throw(ArgumentError("no $target named $name"))
    return indices[target][name]
end
function resolve(index::Int, target::Symbol, indices)
    index in 1:length(indices[target]) ||
        throw(ArgumentError("no $target at index $index"))
    return index
end

"""
    SystemDefinition(document::AbstractDict, system_document::AbstractDict; strict=true)

The system definition a parsed awesIO structure document and the parsed system document it
names describe; only [`load_structure`](@ref) checks `system_sha`. A row reads the columns
of the model its `model` column registers; a filled cell in any other column the schema does
not name is refused, or with `strict=false` dropped with a warning. What the schemas do not
name is kept as `extras` and `system_extras`, in the order read. Refuses another major
`awesIO_version` than `AWESIO_VERSION`, warning on another minor one, a `connectivity_sha`
that does not describe the document's own tables, and a tether the system does not list or
whose segments disagree with it.
"""
function SystemDefinition(document::AbstractDict, system_document::AbstractDict;
                          strict=true)
    for block in REQUIRED_BLOCKS
        haskey(document, String(block)) || throw(ArgumentError("no $block block"))
    end
    fields = document["metadata"]
    check_version(get(fields, "awesIO_version", nothing))
    metadata = Metadata((to_field(fieldtype(Metadata, field), fields[String(field)])
                         for field in fieldnames(Metadata))...)
    tables = (block => read_table(block, document[String(block)], strict)
              for block in keys(BLOCKS) if haskey(document, String(block)))
    extras = OrderedDict{String, Any}(
        name => block for (name, block) in document
        if name != "metadata" && !haskey(BLOCKS, Symbol(name)))
    system = SystemDefinition(; metadata, extras, read_system(system_document)...,
                              tables...)
    sha = connectivity_sha(system)
    metadata.connectivity_sha == sha || throw(ArgumentError(
        "connectivity_sha $(metadata.connectivity_sha) does not describe the document's " *
        "points, segments, bodies, tubes and canopy faces, whose is $sha"))
    foreach(tether -> check_tether(tether, system), system.tethers)
    return system
end

"""The relative difference allowed between floats a writer summed or printed."""
const TETHER_TOLERANCE = 1e-9

"""Refuses `tether` of `system` where the system does not list it, or where its segments
disagree with the material, diameter and length the system gives it. A field left out states
nothing."""
function check_tether(tether, system)
    name = tether.name
    haskey(system.system_tethers, name) ||
        throw(ArgumentError("tether $name is not in the system"))
    design = system.system_tethers[name]
    segments = [system.segments[segment] for segment in tether.segments]
    for segment in segments
        isnothing(design.material) || isnothing(segment.material) ||
            design.material == segment.material || throw(ArgumentError(
            "tether $name segment $(segment.name) is not of the system's material"))
        isnothing(design.diameter) || isnothing(segment.diameter) ||
            isapprox(segment.diameter, design.diameter; rtol=TETHER_TOLERANCE) ||
            throw(ArgumentError("tether $name segment $(segment.name) has diameter " *
                                "$(segment.diameter), the system says $(design.diameter)"))
    end
    any(segment -> isnothing(segment.rest_length), segments) && return nothing
    total = sum(segment -> segment.rest_length, segments; init=0.0)
    isapprox(total, design.length; rtol=TETHER_TOLERANCE) || throw(ArgumentError(
        "tether $name segments add up to $total m, the system says $(design.length) m"))
    return nothing
end

"""The keyword arguments of `SystemDefinition` the parsed system `document` gives."""
function read_system(document)
    for block in ("metadata", "materials", "components")
        haskey(document, block) || throw(ArgumentError("no $block block in the system"))
    end
    components = document["components"]
    haskey(components, "tethers") ||
        throw(ArgumentError("no components.tethers in the system"))
    fields = document["metadata"]
    check_version(get(fields, "awesIO_version", nothing))
    system_metadata = SystemMetadata((to_field(fieldtype(SystemMetadata, field),
                                               fields[String(field)])
                                      for field in fieldnames(SystemMetadata))...)
    materials = [Material(; name=row["name"], law=row["law"], parameters=row["parameters"])
                 for row in document["materials"]]
    system_extras = OrderedDict{String, Any}(
        name => block for (name, block) in document
        if !(name in ("metadata", "materials", "components")))
    others = filter(component -> first(component) != "tethers", components)
    isempty(others) || (system_extras["components"] = others)
    system_tethers = read_system_tether.(components["tethers"])
    return (; system_metadata, materials, system_tethers, system_extras)
end

"""The fields of a `SystemTether` a document nests under its `structure`."""
const TETHER_STRUCTURE = Tuple(setdiff(fieldnames(SystemTether),
                                       (:name, :type, :version, :joints, :model)))

"""The `SystemTether` of the parsed `row` of a system document, refusing a `structure` field
the schema does not name."""
function read_system_tether(row)
    structure = (Symbol(field) => value for (field, value) in row["structure"])
    unknown = setdiff(first.(structure), TETHER_STRUCTURE)
    isempty(unknown) || throw(ArgumentError(
        "tether $(row["name"]) of the system holds $(join(unknown, ", ")), which is no " *
        "field of a system tether"))
    return SystemTether(; name=row["name"], type=row["type"], version=row["version"],
                        joints=row["joints"], structure...)
end

"""Applies the `awesIO_version` rule of `SystemDefinition(document)` to `version`."""
function check_version(version)
    version isa AbstractString || throw(ArgumentError(
        "awesIO_version must be a quoted version string, not $(repr(version))"))
    written, ours = VersionNumber(version), VersionNumber(AWESIO_VERSION)
    written.major == ours.major || throw(ArgumentError(
        "awesIO $version is a major version a reader of awesIO $AWESIO_VERSION refuses"))
    written.minor == ours.minor ||
        @warn "Reading an awesIO $version document as awesIO $AWESIO_VERSION"
    return nothing
end

"""
    connectivity_sha(sections...)

Lowercase hex SHA-256 of the structure schema's connectivity preimage of `sections`, each a
count and its elements, each element the one-based row numbers it joins.
"""
function connectivity_sha(sections...)
    preimage = IOBuffer()
    for (count, elements) in sections
        print(preimage, count, ';')
        for element in elements
            join(preimage, element, ',')
            print(preimage, ';')
        end
    end
    return bytes2hex(sha256(take!(preimage)))
end

"""
    connectivity_sha(system::SystemDefinition)

The `connectivity_sha` of the points and segments, bodies and tubes, and canopy faces of
`system`.
"""
function connectivity_sha(system::SystemDefinition)
    return connectivity_sha(
        (length(system.points), row_numbers(system, :segments, :points)),
        (length(system.bodies), row_numbers(system, :tubes, :bodies)),
        (length(system.canopy_faces), row_numbers(system, :canopy_faces, :points)))
end

"""The one-based row numbers each component of `block` refers to in `column`."""
function row_numbers(system, block, column)
    target = REFERENCES[(block, column)]
    rows = getfield(system, target)
    return [row_number.(getfield(component, column), (rows,), target)
            for component in getfield(system, block)]
end
row_number(index::Int, rows, target) = index
function row_number(name::Symbol, rows, target)
    index = get(name_indices(rows), name, nothing)
    isnothing(index) && throw(ArgumentError("no $target named $name"))
    return index
end

"""
    system_sha(path)

Lowercase hex SHA-256 of the bytes of the file at `path`, which a structure naming that file
as its system holds as `system_sha`.
"""
system_sha(path) = bytes2hex(open(sha256, path))

"""`text` parsed as YAML, its mappings keeping their order."""
parse_yaml(text) = YAML.load(text; dicttype=OrderedDict{String, Any})

"""
    load_structure(path; strict=true)

The `SystemDefinition` in the YAML structure document at `path` and the system file its
`metadata.system` names relative to it, read as `from_yaml` reads them. Refuses a
`metadata.system_sha` that is not the hash of that file.
"""
function load_structure(path; strict=true)
    document = parse_yaml(read(path, String))
    metadata = get(document, "metadata", Dict{String, Any}())
    named = get(metadata, "system", "")
    isempty(named) && throw(ArgumentError("structure $path names no system file"))
    system_path = joinpath(dirname(path), named)
    sha = system_sha(system_path)
    get(metadata, "system_sha", "") == sha || throw(ArgumentError(
        "structure $path holds system_sha $(get(metadata, "system_sha", "")), but the " *
        "hash of $system_path is $sha"))
    return SystemDefinition(document, parse_yaml(read(system_path, String)); strict)
end

"""
    from_yaml(text, system_text; strict=true)

The `SystemDefinition` in the YAML structure document `text` and system document
`system_text`, read as `SystemDefinition(document, system_document; strict)` reads them.
"""
function from_yaml(text::AbstractString, system_text::AbstractString; strict=true)
    return SystemDefinition(parse_yaml(text), parse_yaml(system_text); strict)
end

"""
    from_json(text, system_text; strict=true)

The `SystemDefinition` in the JSON structure document `text` and system document
`system_text`, read as `SystemDefinition(document, system_document; strict)` reads them.
"""
function from_json(text::AbstractString, system_text::AbstractString; strict=true)
    return SystemDefinition(JSON.parse(text; dicttype=OrderedDict{String, Any}),
                            JSON.parse(system_text; dicttype=OrderedDict{String, Any});
                            strict)
end

"""
    definition(topology::AbstractString, system::AbstractString; strict=true)

The `SystemDefinition` in the JSON structure and system documents a log carries under the
keys `topology` and `system`, read as `from_json` reads them.
"""
function definition(topology::AbstractString, system::AbstractString; strict=true)
    return from_json(topology, system; strict)
end

"""
    definition(log::SysLog; strict=true)

The `SystemDefinition` a log loaded by `load_log` carries under its metadata keys `topology`
and `system`, refusing a log without them.
"""
function definition(log::SysLog; strict=true)
    for key in ("topology", "system")
        haskey(log.metadata, key) || throw(ArgumentError(
            "the log carries no $key: it was saved without a definition, or before a log " *
            "carried its system"))
    end
    return definition(log.metadata["topology"], log.metadata["system"]; strict)
end

"""
    save_log(logger::Logger, system::SystemDefinition, name="sim_log", compress=true;
             metadata=Dict{String, String}(), kwargs...)

`save_log(logger, name, compress; metadata, kwargs...)` with `system` added to `metadata` as
JSON, its structure under the key `topology` and its system under the key `system`, which
`KiteGeometry.definition` reads back.
"""
function KiteUtils.save_log(logger::Logger, system::SystemDefinition, name="sim_log",
                            compress=true; metadata=Dict{String, String}(), kwargs...)
    metadata = merge(metadata, Dict("topology" => to_json(system),
                                    "system" => to_json(system_document(system))))
    return save_log(logger, name, compress; metadata, kwargs...)
end

"""
    to_yaml(system::SystemDefinition)

`structure_document(system)` as YAML text, refusing a `system` whose metadata names no
system file: give `structure_document` the file's path and hash.
"""
function to_yaml(system::SystemDefinition)
    (isempty(system.metadata.system) || isempty(system.metadata.system_sha)) &&
        throw(ArgumentError("structure $(system.metadata.name) names no system file and " *
                            "hash: write it with structure_document(system; system, " *
                            "system_sha)"))
    return to_yaml(structure_document(system))
end

"""
    to_json(system::SystemDefinition)

`structure_document(system)` as JSON text.
"""
to_json(system::SystemDefinition) = to_json(structure_document(system))

"""
    to_json(document::AbstractDict)

`document` as JSON text.
"""
to_json(document::AbstractDict) = JSON.json(document; pretty=true)

"""The names of the columns of `block` that the schema requires, in order."""
required_columns(block) = fieldnames(BLOCKS[block])[eachindex(UNITS[block])]

"""The components in the `headers`/`units`/`data` table of `block`: the required columns
leading, then optional and model columns by header. Refuses a filled column they do not read
unless `strict` is false, which warns."""
function read_table(block, table, strict)
    T = BLOCKS[block]
    headers = table["headers"]
    units = get(table, "units", nothing)
    columns = required_columns(block)
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
    optional = optional_column_indices(block, headers, units)
    open_columns = [index for index in (n + 1):length(headers)
                    if !(index in values(optional))]
    model_column = findfirst(==("model"), headers)
    components = T[]
    unread = OrderedSet{String}()
    for row in table["data"]
        name = isnothing(model_column) ? missing : row[model_column]
        model = row_model(block, name) do M, field, unit
            model_field(M, field, unit, headers, units, row)
        end
        push!(components, T(; zip(columns, row[1:n])...,
                            (field => row[index] for (field, index) in optional)..., model))
        union!(unread, unread_columns(headers[open_columns], row[open_columns], model))
    end
    isempty(unread) && return components
    message = "$block columns $(join(unread, ", ")) name no field of the schema or of a " *
              "registered model"
    strict && throw(ArgumentError(message))
    @warn "$message; dropping them"
    return components
end

"""The index of each optional column of `block` among `headers`, refusing one whose unit
is not the schema's."""
function optional_column_indices(block, headers, units)
    indices = Dict{Symbol, Int}()
    for (field, unit) in pairs(OPTIONAL_UNITS[block])
        index = findfirst(==(String(field)), headers)
        isnothing(index) && continue
        units[index] == unit ||
            throw(ArgumentError("the $field column of $block must be in $unit"))
        indices[field] = index
    end
    return indices
end

"""The `headers` of the filled `cells` that `model` does not read."""
function unread_columns(headers, cells, model)
    read = model isa NoModel ? () : ("model", String.(model_columns(typeof(model)))...)
    return (header for (header, cell) in zip(headers, cells)
            if !isnothing(cell) && !(header in read))
end

"""The `field` of the model `M` in `row`, from the column of that name, unset where the
column is absent. Refuses a column not in the registered `unit`."""
function model_field(M, field, unit, headers, units, row)
    column = findfirst(==(String(field)), headers)
    isnothing(column) && return model_value(M, field, nothing)
    units[column] == unit ||
        throw(ArgumentError("the $field column of model $M must be in $unit"))
    return model_value(M, field, row[column])
end

"""
    structure_document(structure::SystemDefinition; description, note, system, system_sha)

The awesIO structure document of `structure`, as `SystemDefinition` reads it: references by
name, model columns after the schema's, extra blocks after the schema's, empty optional
blocks and unset optional columns left out, and of the tethers those its system lists. Its
`awesIO_version` is `AWESIO_VERSION` and its `connectivity_sha` that of `structure`'s own
tables, whatever `structure.metadata` holds.
`description`, `note`, `system`, the path of the system file relative to this one, and
`system_sha`, that file's [`system_sha`](@ref), default to those of `structure.metadata`.
"""
function structure_document(structure::SystemDefinition;
                            description=structure.metadata.description,
                            note=structure.metadata.note,
                            system=structure.metadata.system,
                            system_sha=structure.metadata.system_sha)
    metadata = OrderedDict{String, Any}(String(field) => getfield(structure.metadata, field)
                                        for field in fieldnames(Metadata))
    merge!(metadata, Dict("awesIO_version" => AWESIO_VERSION,
                          "connectivity_sha" => connectivity_sha(structure),
                          "description" => description, "note" => note,
                          "system" => system, "system_sha" => system_sha))
    document = OrderedDict{String, Any}("metadata" => metadata)
    for block in keys(BLOCKS)
        rows = block == :tethers ? listed_tethers(structure) : getfield(structure, block)
        block in REQUIRED_BLOCKS || !isempty(rows) || continue
        document[String(block)] = write_table(structure, block, rows)
    end
    return merge!(document, structure.extras)
end

"""The tethers of `structure` its system lists."""
function listed_tethers(structure)
    listed = structure.system_tethers
    return filter(tether -> haskey(listed, tether.name), structure.tethers)
end

"""
    system_document(definitions::AbstractVector; name, description="", note="")
    system_document(definition::SystemDefinition; name, description, note)

The awesIO system document that several structures can name: the union by name of the
materials and of the tethers of `definitions`, and of what else their systems hold, refusing
one name with two different values. For one `definition`, `name`, `description` and `note`
default to those of its `system_metadata`.
"""
function system_document(definitions::AbstractVector{<:SystemDefinition}; name,
                         description="", note="")
    document = OrderedDict{String, Any}(
        "metadata" => OrderedDict{String, Any}(
            "name" => name, "description" => description, "note" => note,
            "awesIO_version" => AWESIO_VERSION, "schema" => "system_schema.yml"),
        "materials" => union_by_name(
            "materials", material_row(material) for definition in definitions
                         for material in definition.materials))
    extras = OrderedDict{String, Any}()
    for definition in definitions, (block, value) in definition.system_extras
        get!(extras, block, value) == value ||
            throw(ArgumentError("the systems hold two different $block blocks"))
    end
    tethers = union_by_name("tethers", system_tether_row(definition, tether)
                                       for definition in definitions
                                       for tether in definition.system_tethers)
    document["components"] = merge(get(extras, "components", OrderedDict{String, Any}()),
                                   OrderedDict{String, Any}("tethers" => tethers))
    delete!(extras, "components")
    return merge!(document, extras)
end
function system_document(definition::SystemDefinition;
                         name=definition.system_metadata.name,
                         description=definition.system_metadata.description,
                         note=definition.system_metadata.note)
    return system_document([definition]; name, description, note)
end

"""Each of the document `rows` once, refusing one name with two different values."""
function union_by_name(block, rows)
    union = OrderedDict{String, Any}()
    for row in rows
        get!(union, row["name"], row) == row || throw(ArgumentError(
            "two definitions give the $block row $(row["name"]) different values: " *
            "$(union[row["name"]]) and $row"))
    end
    return collect(values(union))
end

"""The row of `material` in a system document."""
function material_row(material)
    return OrderedDict{String, Any}("name" => String(material.name), "law" => material.law,
                                    "parameters" => material.parameters)
end

"""The row of `tether` of `definition` in a system document."""
function system_tether_row(definition, tether)
    structure = OrderedDict{String, Any}()
    for field in TETHER_STRUCTURE
        target = get(REFERENCES, (:system_tethers, field), nothing)
        value = document_value(getfield(tether, field), target, definition)
        isnothing(value) || (structure[String(field)] = value)
    end
    return OrderedDict{String, Any}("name" => String(tether.name), "type" => tether.type,
                                    "version" => tether.version, "joints" => tether.joints,
                                    "structure" => structure)
end

"""The `headers`/`units`/`data` table of the components `rows` of `block`: the required
columns and the optional columns a row fills, then each column of a row's model that a row
fills, after the `model` column unless every row reads the block's default model."""
function write_table(system, block, rows)
    filled = [field for field in keys(OPTIONAL_UNITS[block])
              if any(row -> !isnothing(getfield(row, field)), rows)]
    columns = (required_columns(block)..., filled...)
    registrations = [model_registration(block, typeof(row.model)) for row in rows]
    model_units = OrderedDict{String, String}()
    any(!isnothing, registrations) &&
        !all(reads_default.(registrations, block)) && (model_units["model"] = "-")
    for model in unique(filter(!isnothing, registrations))
        for (field, unit) in zip(model_columns(model.type), model.units)
            any(row -> row.model isa model.type && !isnothing(getfield(row.model, field)),
                rows) || continue
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
        "units" => [UNITS[block]..., (OPTIONAL_UNITS[block][field] for field in filled)...,
                    values(model_units)...],
        "data" => data)
end

"""Whether a row registered as `registration` reads the default model of `block`."""
function reads_default(registration, block)
    return !isnothing(registration) && registration.name == default_model(block)
end

"""The cell of `model`, registered as `registration`, in the model column `header`: the
registered name for `model`, else the field of that name, `nothing` where it has none."""
function model_cell(model, registration, header)
    header == "model" && return isnothing(registration) ? nothing : registration.name
    Symbol(header) in model_columns(typeof(model)) || return nothing
    return document_value(getfield(model, Symbol(header)), nothing, nothing)
end

"""`value` as a structure document writes it, a reference into `target` by name."""
document_value(value, ::Nothing, system) = value
document_value(value::StaticVector, ::Nothing, system) = collect(value)
function document_value(value::StaticMatrix, ::Nothing, system)
    return [collect(row) for row in eachrow(value)]
end
document_value(::Nothing, target::Symbol, system) = nothing
document_value(name::Symbol, ::Nothing, system) = String(name)
function document_value(names::AbstractVector{NameRef}, ::Nothing, system)
    return document_value.(names, nothing, (system,))
end
function document_value(index::Int, target::Symbol, system)
    return String(getfield(system, target)[index].name)
end
document_value(name::Symbol, target::Symbol, system) = String(name)
function document_value(refs::Union{Tuple, Vector}, target::Symbol, system)
    return [document_value(ref, target, system) for ref in refs]
end
