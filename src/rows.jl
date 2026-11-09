# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    Rows{T} <: AbstractVector{T}

Elements with a `name`, indexed by position or by name: `system.points[3]`,
`system.points[:kite]` and `system.points["kite"]` are rows of `points`. A row is renamed by
replacing it. Writing to `rows` drops its name index, which the next name lookup rebuilds.
"""
mutable struct Rows{T} <: AbstractVector{T}
    const rows::Vector{T}
    @atomic indices::Union{Nothing, Dict{Symbol, Int}}
    Rows{T}(rows) where {T} = new{T}(rows, nothing)
end
Rows(rows::Vector{T}) where {T} = Rows{T}(rows)
Rows{T}() where {T} = Rows{T}(T[])

"""`rows`, its name index dropped."""
function changed!(rows::Rows)
    @atomic rows.indices = nothing
    return rows
end

Base.convert(::Type{Rows{T}}, rows::Rows{T}) where {T} = rows
Base.convert(::Type{Rows{T}}, rows::AbstractVector) where {T} =
    Rows{T}(convert(Vector{T}, rows))
Base.size(rows::Rows) = size(rows.rows)
Base.getindex(rows::Rows, index::Int) = rows.rows[index]
Base.setindex!(rows::Rows, row, index::Int) = (rows.rows[index] = row; changed!(rows))
Base.getindex(rows::Rows, name::Union{Symbol, AbstractString}) =
    rows.rows[row_index(rows, name)]
Base.setindex!(rows::Rows, row, name::Union{Symbol, AbstractString}) =
    setindex!(rows, row, row_index(rows, name))
Base.haskey(rows::Rows, name::Union{Symbol, AbstractString}) =
    haskey(name_indices(rows), Symbol(name))
Base.resize!(rows::Rows, n::Integer) = (resize!(rows.rows, n); changed!(rows))
Base.push!(rows::Rows, row) = (push!(rows.rows, row); changed!(rows))
Base.insert!(rows::Rows, index::Integer, row) =
    (insert!(rows.rows, index, row); changed!(rows))
Base.deleteat!(rows::Rows, index) = (deleteat!(rows.rows, index); changed!(rows))
Base.empty!(rows::Rows) = (empty!(rows.rows); changed!(rows))

"""
    row_index(rows::Rows, name)

The position of the row of `rows` named `name`, a `Symbol` or a string, refusing a name no
row has and any two rows of one name.
"""
row_index(rows::Rows, name) = name_indices(rows)[Symbol(name)]

"""The position of each row of `rows` by name, rebuilt where a write dropped it."""
function name_indices(rows::Rows{T}) where {T}
    indices = @atomic rows.indices
    isnothing(indices) || return indices
    indices = name_indices(rows.rows, block_name(T))
    @atomic rows.indices = indices
    return indices
end

"""The position of each element of `rows` by its name, refusing a name of `block` used
twice."""
function name_indices(rows, block)
    indices = Dict{Symbol, Int}()
    for (index, row) in enumerate(rows)
        name = Symbol(row.name)
        haskey(indices, name) && throw(duplicate_name(block, name))
        indices[name] = index
    end
    return indices
end

"""The block a row of type `T` belongs to, else `T` itself."""
block_name(::Type{T}) where {T} = something(findfirst(==(T), BLOCKS), T)

"""The error for two rows of `block` named `name`."""
duplicate_name(block, name) = ArgumentError("two $block are named $name")
