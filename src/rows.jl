# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    Rows{T} <: AbstractVector{T}

Elements named by a `Symbol` that cannot change in place, indexed by position or by name:
`system.points[3]`, `system.points[:kite]` and `system.points["kite"]` are rows of
`points`. `Rows(elements)` holds a copy of `elements`; it and every write refuse two rows
of one name.
"""
struct Rows{T} <: AbstractVector{T}
    rows::Vector{T}
    indices::Dict{Symbol, Int}
    function Rows{T}(elements) where {T}
        rows = collect(T, elements)
        return new{T}(rows, name_indices(rows, block_name(T)))
    end
end
Rows(rows::AbstractVector{T}) where {T} = Rows{T}(rows)
Rows{T}() where {T} = Rows{T}(T[])

Base.convert(::Type{Rows{T}}, rows::Rows{T}) where {T} = rows
Base.convert(::Type{Rows{T}}, rows::AbstractVector) where {T} = Rows{T}(rows)
Base.size(rows::Rows) = size(rows.rows)
Base.getindex(rows::Rows, index::Int) = rows.rows[index]
Base.getindex(rows::Rows, name::Union{Symbol, AbstractString}) =
    rows.rows[row_index(rows, name)]
Base.setindex!(rows::Rows, row, name::Union{Symbol, AbstractString}) =
    setindex!(rows, row, row_index(rows, name))
Base.haskey(rows::Rows, name::Union{Symbol, AbstractString}) =
    haskey(rows.indices, Symbol(name))

function Base.setindex!(rows::Rows{T}, row, index::Int) where {T}
    row = convert(T, row)
    previous = rows.rows[index].name
    if row_name(row) !== previous
        rows.indices[new_name(rows, row)] = index
        delete!(rows.indices, previous)
    end
    rows.rows[index] = row
    return rows
end
function Base.push!(rows::Rows{T}, row) where {T}
    row = convert(T, row)
    name = new_name(rows, row)
    push!(rows.rows, row)
    rows.indices[name] = length(rows)
    return rows
end
function Base.insert!(rows::Rows{T}, index::Integer, row) where {T}
    row = convert(T, row)
    new_name(rows, row)
    insert!(rows.rows, index, row)
    return reindex!(rows, index:length(rows))
end
function Base.deleteat!(rows::Rows, index)
    deleteat!(rows.rows, index)
    empty!(rows.indices)
    return reindex!(rows, eachindex(rows.rows))
end
Base.empty!(rows::Rows) = (empty!(rows.rows); empty!(rows.indices); rows)

"""
    row_index(rows::Rows, name)

The position of the row of `rows` named `name`, a `Symbol` or a string, refusing a name no
row has.
"""
row_index(rows::Rows, name) = rows.indices[Symbol(name)]

"""The position of each row of `rows` by name."""
name_indices(rows::Rows) = rows.indices

"""The position of each element of `rows` by its name, refusing a name of `block` used
twice."""
function name_indices(rows, block)
    indices = Dict{Symbol, Int}()
    for (index, row) in enumerate(rows)
        name = row_name(row)
        haskey(indices, name) && throw(duplicate_name(block, name))
        indices[name] = index
    end
    return indices
end

"""The name of `row`, refusing it where another row of `rows` holds it."""
function new_name(rows::Rows{T}, row) where {T}
    name = row_name(row)
    haskey(rows.indices, name) && throw(duplicate_name(block_name(T), name))
    return name
end

"""`rows` with the rows at `positions` indexed by their names."""
function reindex!(rows::Rows, positions)
    for index in positions
        rows.indices[rows.rows[index].name] = index
    end
    return rows
end

"""The name of `row`, refusing one that is no `Symbol` or can change in place."""
function row_name(row)
    name = row.name
    name isa Symbol || throw(ArgumentError("a row is named by a Symbol, not $(repr(name))"))
    ismutable(row) && !isconst(typeof(row), :name) &&
        throw(ArgumentError("a row's name is a const field, not one of $(typeof(row))"))
    return name
end

"""The block a row of type `T` belongs to, else `T` itself."""
block_name(::Type{T}) where {T} = something(findfirst(==(T), BLOCKS), T)

"""The error for two rows of `block` named `name`."""
duplicate_name(block, name) = ArgumentError("two $block are named $name")
