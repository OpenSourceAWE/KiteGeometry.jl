# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    to_yaml(document::AbstractDict)

`document` as YAML 1.2 text whose every value reads back as what it is in YAML 1.1 readers
as well: a string that could read as anything else is quoted, and a float has its point and
its exponent's sign. A mapping is written in block style, a list of scalars on one line, and
a list of lists one inner list per line.
"""
function to_yaml(document::AbstractDict)
    io = IOBuffer()
    write_mapping(io, document, 0, 0)
    return String(take!(io))
end

"""Writes `mapping` one key per line at `indent`, the first key at `first_indent`."""
function write_mapping(io, mapping, indent, first_indent)
    for (index, (key, value)) in enumerate(mapping)
        print(io, " "^(index == 1 ? first_indent : indent), yaml_flow(key), ":")
        write_value(io, value, indent)
    end
    return nothing
end

"""Writes the `value` of a key at `indent`, on its line where it fits there."""
function write_value(io, value, indent)
    if value isa AbstractDict && !isempty(value)
        println(io)
        write_mapping(io, value, indent + 2, indent + 2)
    elseif value isa Union{AbstractVector, Tuple} && any(is_collection, value)
        println(io)
        foreach(item -> write_item(io, item, indent + 2), value)
    else
        println(io, " ", yaml_flow(value))
    end
    return nothing
end

"""Writes `item` of a block list at `indent`."""
function write_item(io, item, indent)
    print(io, " "^indent, "- ")
    if item isa AbstractDict && !isempty(item)
        write_mapping(io, item, indent + 2, 0)
    else
        println(io, yaml_flow(item))
    end
    return nothing
end

is_collection(value) = value isa Union{AbstractDict, AbstractVector, Tuple}

"""`value` as one line of YAML."""
function yaml_flow(mapping::AbstractDict)
    entries = (yaml_flow(key) * ": " * yaml_flow(value) for (key, value) in mapping)
    return "{" * join(entries, ", ") * "}"
end
yaml_flow(list::Union{AbstractVector, Tuple}) = "[" * join(yaml_flow.(list), ", ") * "]"
yaml_flow(::Nothing) = "null"
yaml_flow(value::Union{Bool, Integer}) = string(value)
yaml_flow(name::Symbol) = yaml_flow(String(name))
function yaml_flow(value::AbstractFloat)
    isnan(value) && return ".nan"
    isinf(value) && return value > 0 ? ".inf" : "-.inf"
    return replace(string(Float64(value)), r"e(\d)" => s"e+\1")
end
function yaml_flow(text::AbstractString)
    return is_plain_scalar(text) ? String(text) : JSON.json(String(text))
end

function yaml_flow(value)
    throw(ArgumentError("a document cannot hold $(repr(value)), a $(typeof(value))"))
end

"""The words a YAML 1.1 reader takes for a boolean or null."""
const RESERVED_WORDS = ("null", "true", "false", "yes", "no", "on", "off", "y", "n")

"""Whether `text`, unquoted, reads as that string in every YAML reader."""
function is_plain_scalar(text)
    return occursin(r"^[A-Za-z_][A-Za-z0-9_.]*$", text) &&
           !(lowercase(text) in RESERVED_WORDS)
end
