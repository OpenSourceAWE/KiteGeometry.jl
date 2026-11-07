# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    AbstractModel

Supertype of the models `M` of a component `T{M}`: the parameters a tool attaches to a row
through the model its `model` column names, one field per extra column.
"""
abstract type AbstractModel end

"""
    NoModel <: AbstractModel

The model of a component whose `model` column is absent or names no registered model.
"""
struct NoModel <: AbstractModel end

"""A registered model: its type and the unit of each of its columns, in field order."""
const MODELS = Dict{Tuple{Symbol, String},
                    @NamedTuple{type::Type{<:AbstractModel}, units::Vector{String}}}()

"""The name of the model each block's rows read where its table has no `model` column."""
const DEFAULT_MODELS = Dict{Symbol, String}()

"""
    model_columns(M)

The fields of the model type `M` that are columns of a document, in order: all of them
unless `M` adds a method naming fewer. An `M` with fields that are no column is built from
its columns alone, `M(columns...)`.
"""
model_columns(M::Type{<:AbstractModel}) = fieldnames(M)

"""
    register_model!(block, name, M, units; default=false)

Make a row of `block` whose `model` column reads `name` carry an `M`, built from the columns
named after the [`model_columns`](@ref) of `M`, whose `units` are given in that order. With
`default`, the rows of a `block` table without a `model` column carry an `M` too. Refuses an
`M` with a field the component already has, and a second default for `block`. A package
registering its models calls this from its `__init__`.
"""
function register_model!(block::Symbol, name::AbstractString, M::Type{<:AbstractModel},
                         units; default=false)
    clashes = intersect(fieldnames(M), fieldnames(BLOCKS[block]))
    isempty(clashes) ||
        throw(ArgumentError("$M shadows the $block fields $(join(clashes, ", "))"))
    length(units) == length(model_columns(M)) ||
        throw(ArgumentError("$M needs one unit per column"))
    default && get(DEFAULT_MODELS, block, name) != name && throw(ArgumentError(
        "$block already reads the default model $(DEFAULT_MODELS[block])"))
    MODELS[(block, name)] = (; type=M, units=collect(String, units))
    default && (DEFAULT_MODELS[block] = name)
    return M
end

"""The name of the model the rows of a `block` table without a `model` column read, or
`nothing`."""
default_model(block) = get(DEFAULT_MODELS, block, nothing)

"""
    row_model(field_value, block, name)

The model of a row of `block`: the one registered under `name`, under the block's default
where `name` is `missing` for a table without a `model` column, and `NoModel` where that
names no registered model. Each field of its type `M` is `field_value(M, field, unit)`.
"""
function row_model(field_value, block, name)
    model = get(MODELS, (block, coalesce(name, default_model(block))), nothing)
    isnothing(model) && return NoModel()
    M = model.type
    return M((field_value(M, field, unit)
              for (field, unit) in zip(model_columns(M), model.units))...)
end

"""`value` as `field` of model `M`, refusing `nothing` where the field cannot hold it."""
function model_value(M, field, value)
    T = fieldtype(M, field)
    isnothing(value) && !(Nothing <: T) &&
        throw(ArgumentError("model $M needs its $field column filled"))
    return to_field(T, value)
end

"""
    model_type(block, name)

The model registered for `block` under `name`, or `NoModel` for `nothing` or an unregistered
name.
"""
function model_type(block, name)
    model = get(MODELS, (block, name), nothing)
    return isnothing(model) ? NoModel : model.type
end

"""
    model_registration(block, M)

The registration for `block` whose type `M` is a subtype of, such as a parametric model's
`UnionAll`: its name, that type and its column units. `nothing` for `NoModel`.
"""
function model_registration(block, M)
    M === NoModel && return nothing
    for ((registered_block, name), model) in MODELS
        registered_block == block && M <: model.type &&
            return (; name, model.type, model.units)
    end
    throw(ArgumentError("$M is no model registered for $block"))
end

function Base.getproperty(component::Component, name::Symbol)
    hasfield(typeof(component), name) && return getfield(component, name)
    return getproperty(getfield(component, :model), name)
end
function Base.setproperty!(component::Component, name::Symbol, value)
    hasfield(typeof(component), name) || return setproperty!(component.model, name, value)
    return setfield!(component, name, to_field(fieldtype(typeof(component), name), value))
end
function Base.propertynames(component::Component)
    return (fieldnames(typeof(component))..., propertynames(component.model)...)
end

"""Rows, and models, of one type are equal where every field is."""
const ByValue = Union{Component, AbstractModel}
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
    SegmentSpring <: AbstractModel

The spring of a segment besides its `unit_stiffness`: its damping and how it carries
compression. Registered as the segment model `spring`.
"""
struct SegmentSpring <: AbstractModel
    "damping per unit length [N*s]"
    unit_damping::Float64
    "compressive over tensile stiffness [-]; 0 for a segment that goes slack"
    compression_frac::Float64
    "fraction of `unit_damping` still acting under compression [-]"
    compression_damping_frac::Float64
end
