# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    NamedCollection{T} <: AbstractVector{T}

A vector of components that is also indexable by the component's symbolic name.

Subtyping `AbstractVector` keeps it usable wherever a vector is expected. Names
come from each item's `name` field; items with `name === nothing` are reachable
by numeric index only.

```julia
points = NamedCollection([Point(:kcu, [0, 0, 0], DYNAMIC)])
points[1]              # by position
points[:kcu]           # by name
haskey(points, :kcu)   # true
```

$(TYPEDFIELDS)
"""
struct NamedCollection{T} <: AbstractVector{T}
    "the components, in definition order"
    items::Vector{T}
    "name → position in `items`"
    name_to_idx::Dict{Symbol, Int64}
end

"""
    NamedCollection(items::Vector{T}) where T

Wrap `items`, building the name → index map from their `name` fields. A name
used twice is an error.
"""
NamedCollection(items::Vector{T}) where T =
    NamedCollection{T}(items, build_name_dict(items))

"""
    build_name_dict(items) -> Dict{Symbol, Int64}

Name → position map of `items`. Items with `name === nothing` are skipped and
an integer name becomes the matching `Symbol`.
"""
function build_name_dict(items)
    name_to_idx = Dict{Symbol, Int64}()
    for (i, item) in enumerate(items)
        item_name = item.name
        isnothing(item_name) && continue
        name = item_name isa Symbol ? item_name : Symbol(item_name)
        haskey(name_to_idx, name) && error("Duplicate name '$name' at " *
            "indices $(name_to_idx[name]) and $i")
        name_to_idx[name] = i
    end
    return name_to_idx
end

Base.getindex(collection::NamedCollection, i::Integer) = collection.items[i]

function Base.getindex(collection::NamedCollection, name::Symbol)
    haskey(collection.name_to_idx, name) || error("Name '$name' not found. " *
        "Available names: $(collect(keys(collection.name_to_idx)))")
    return collection.items[collection.name_to_idx[name]]
end

Base.setindex!(collection::NamedCollection, value, i::Integer) =
    (collection.items[i] = value)

function Base.setindex!(collection::NamedCollection, value, name::Symbol)
    haskey(collection.name_to_idx, name) ||
        error("Name '$name' not found in collection")
    collection.items[collection.name_to_idx[name]] = value
end

Base.size(collection::NamedCollection) = size(collection.items)
Base.haskey(collection::NamedCollection, name::Symbol) =
    haskey(collection.name_to_idx, name)
Base.keys(collection::NamedCollection) = keys(collection.name_to_idx)
Base.push!(collection::NamedCollection, item) = push!(collection.items, item)

function Base.show(io::IO, ::MIME"text/plain", collection::NamedCollection{T}) where T
    names = sort!(String.(collect(keys(collection.name_to_idx))))
    println(io, "NamedCollection{$T} with $(length(collection.items)) items " *
        "($(length(names)) named):")
    isempty(names) || println(io, "  Names: ", join(names, ", "))
end
