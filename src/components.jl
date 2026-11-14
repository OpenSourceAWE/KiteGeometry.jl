# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

const IDENTITY_QUATERNION = SVector(1.0, 0.0, 0.0, 0.0)

"""Rows, models and metadata of one type are equal where every field is."""
const ByValue = Union{Component, AbstractModel, Metadata}
function Base.:(==)(a::T, b::T) where {T <: ByValue}
    return all(getfield(a, field) == getfield(b, field) for field in fieldnames(T))
end
function Base.hash(value::ByValue, seed::UInt)
    code = hash(typeof(value), seed)
    for field in fieldnames(typeof(value))
        code = hash(getfield(value, field), code)
    end
    return code
end

"""
    defaults(T)

The keyword arguments a `T(; kwargs...)` constructor fills in when they are not given: every
optional column its schema default or unset, `default_values(T)`, and no model.
"""
function defaults(T::Type{<:Component})
    block = findfirst(==(Base.typename(T).wrapper), BLOCKS)
    optional = NamedTuple(field => optional_default(block, field)
                          for field in keys(OPTIONAL_UNITS[block]))
    return (; optional..., default_values(T)..., model=NoModel())
end

"""The value the optional column `field` of `block` has where it is left out: the schema's
default, else `nothing`."""
optional_default(block, field) = get(OPTIONAL_DEFAULTS[block], field, nothing)

"""The required columns of `T` its constructor fills in: a free point of its own, and the
identity rotation of a wing's or a body's frame."""
default_values(::Type{<:Component}) = (;)
default_values(::Type{<:Point}) = (; fixed=false, body=nothing)
default_values(::Type{<:Union{Wing, Body}}) = (; Q_KA2ENU=IDENTITY_QUATERNION)

"""
    T(; kwargs...) where {T <: Component}

A component of type `T` from one keyword argument per field, with `defaults(T)` filling
those not given; `T{M}` takes `M` from `model`. References may be names or indices.
"""
function (T::Type{<:Component})(; kwargs...)
    values = merge(defaults(T), kwargs)
    missing_fields = setdiff(fieldnames(T), keys(values))
    isempty(missing_fields) ||
        throw(ArgumentError("$T needs $(join(missing_fields, ", "))"))
    S = T isa UnionAll ? T{typeof(values.model)} : T
    return S((to_field(fieldtype(S, field), values[field]) for field in fieldnames(S))...)
end

"""
    to_field(T, value)

`value`, as a keyword argument or a structure document gives it, as a field of type `T`.
"""
to_field(T, value) = convert(T, value)
to_field(::Type{Union{Nothing, T}}, value) where {T} = isnothing(value) ? nothing :
                                                    to_field(T, value)
to_field(::Type{Any}, value) = value
to_field(::Type{Symbol}, name::AbstractString) = Symbol(name)
to_field(::Type{NameRef}, name::AbstractString) = Symbol(name)
to_field(T::Type{<:Tuple}, value::Union{Tuple, AbstractVector}) =
    T(to_field.(fieldtypes(T), Tuple(value)))
to_field(::Type{Vector{T}}, value::AbstractVector) where {T} =
    T[to_field(T, x) for x in value]
to_field(T::Type{<:StaticMatrix}, rows::AbstractVector) =
    T(permutedims(reduce(hcat, rows)))
to_field(T::Type{<:OrderedDict}, value::AbstractDict) = T(value)
to_field(::Type{Union{Nothing, Float64, Vector{Float64}}}, value::Real) = Float64(value)
function to_field(::Type{Union{Nothing, Float64, Vector{Float64}}}, value::AbstractVector)
    return Vector{Float64}(value)
end

"""
    pos_KA(wing::Wing, pos_ENU)

The position `pos_ENU` [m] in the world, in `wing`'s KA frame: `Rᵀ (pos_ENU − o)`, with `o`
the wing's `pos_ENU` and `R` the rotation of its `Q_KA2ENU`.
"""
function pos_KA(wing::Wing, pos_ENU)
    return quaternion_to_rotation_matrix(wing.Q_KA2ENU)' * (pos_ENU - wing.pos_ENU)
end
