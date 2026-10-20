# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    Rows{T} <: AbstractVector{T}

The rows of a block, indexed by position or by name: `system.points[3]`,
`system.points[:kite]` and `system.points["kite"]` are rows of `points`.
"""
struct Rows{T} <: AbstractVector{T}
    rows::Vector{T}
end
Rows{T}() where {T} = Rows{T}(T[])

Base.convert(::Type{Rows{T}}, rows::Rows{T}) where {T} = rows
Base.convert(::Type{Rows{T}}, rows::AbstractVector) where {T} =
    Rows{T}(convert(Vector{T}, rows))
Base.size(rows::Rows) = size(rows.rows)
Base.getindex(rows::Rows, index::Int) = rows.rows[index]
Base.setindex!(rows::Rows, row, index::Int) = (rows.rows[index] = row)
Base.getindex(rows::Rows, name::Union{Symbol, AbstractString}) =
    rows.rows[row_index(rows, name)]
Base.setindex!(rows::Rows, row, name::Union{Symbol, AbstractString}) =
    (rows.rows[row_index(rows, name)] = row)
Base.haskey(rows::Rows, name::Union{Symbol, AbstractString}) =
    !isnothing(findfirst(row -> row.name == Symbol(name), rows.rows))
Base.resize!(rows::Rows, n::Integer) = (resize!(rows.rows, n); rows)
Base.push!(rows::Rows, row) = (push!(rows.rows, row); rows)
Base.insert!(rows::Rows, index::Integer, row) = (insert!(rows.rows, index, row); rows)
Base.deleteat!(rows::Rows, index) = (deleteat!(rows.rows, index); rows)
Base.empty!(rows::Rows) = (empty!(rows.rows); rows)

"""The index of the row of `rows` named `name`, refusing a name no row has."""
function row_index(rows::Rows, name)
    index = findfirst(row -> row.name == Symbol(name), rows.rows)
    isnothing(index) && throw(KeyError(name))
    return index
end
