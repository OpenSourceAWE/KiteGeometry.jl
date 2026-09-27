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

const MODELS = Dict{Tuple{Symbol, String}, Type{<:AbstractModel}}()

"""
    register_model!(block, name, M)

Make a row of `block` whose `model` column reads `name` carry an `M`, built from the extra
columns named after the fields of `M`. Refuses an `M` with a field the component already
has. A package registering its models calls this from its `__init__`.
"""
function register_model!(block::Symbol, name::AbstractString, M::Type{<:AbstractModel})
    clashes = intersect(fieldnames(M), fieldnames(BLOCKS[block]))
    isempty(clashes) ||
        throw(ArgumentError("$M shadows the $block fields $(join(clashes, ", "))"))
    MODELS[(block, name)] = M
    return M
end

"""
    model_type(block, name)

The model registered for `block` under `name`, or `NoModel` for `nothing` or an unregistered
name.
"""
model_type(block, name) = get(MODELS, (block, name), NoModel)

"""
    model_name(block, M)

The name `M` is registered under for `block`, or `nothing` for `NoModel`.
"""
function model_name(block, M)
    M === NoModel && return nothing
    for ((registered_block, name), registered) in MODELS
        registered_block == block && registered === M && return name
    end
    throw(ArgumentError("$M is no model registered for $block"))
end

"""The names of the columns the model `M` reads, in field order."""
model_columns(M) = String.(fieldnames(M))

function Base.getproperty(component::Component, name::Symbol)
    hasfield(typeof(component), name) && return getfield(component, name)
    return getfield(getfield(component, :model), name)
end
function Base.propertynames(component::Component)
    return (fieldnames(typeof(component))..., fieldnames(typeof(component.model))...)
end
