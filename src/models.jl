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
    register_model!(block, name, M, units; default=false)

Make a row of `block` whose `model` column reads `name` carry an `M`, built from the columns
named after the fields of `M`, whose `units` are given in field order. With `default`, the
rows of a `block` table without a `model` column carry an `M` too. Refuses an `M` with a
field the component already has, and a second default for `block`. A package registering its
models calls this from its `__init__`.
"""
function register_model!(block::Symbol, name::AbstractString, M::Type{<:AbstractModel},
                         units; default=false)
    clashes = intersect(fieldnames(M), fieldnames(BLOCKS[block]))
    isempty(clashes) ||
        throw(ArgumentError("$M shadows the $block fields $(join(clashes, ", "))"))
    length(units) == fieldcount(M) ||
        throw(ArgumentError("$M needs one unit per field"))
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

The name `M` is registered under for `block`, with `M` and its column units, or `nothing`
for `NoModel`.
"""
function model_registration(block, M)
    M === NoModel && return nothing
    for ((registered_block, name), model) in MODELS
        registered_block == block && model.type === M &&
            return (; name, model.type, model.units)
    end
    throw(ArgumentError("$M is no model registered for $block"))
end

function Base.getproperty(component::Component, name::Symbol)
    hasfield(typeof(component), name) && return getfield(component, name)
    return getfield(getfield(component, :model), name)
end
function Base.propertynames(component::Component)
    return (fieldnames(typeof(component))..., fieldnames(typeof(component.model))...)
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
