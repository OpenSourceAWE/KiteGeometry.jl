# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    AbstractTubeModel

Supertype of the models `M` of a `Tube{M}`, which a tool names in a tube's `model` column.
"""
abstract type AbstractTubeModel end

"""
    PlainTube <: AbstractTubeModel

The model of a tube whose `model` column is absent or names no registered model.
"""
struct PlainTube <: AbstractTubeModel end

const TUBE_MODELS = Dict{String, Type{<:AbstractTubeModel}}()

"""
    register_tube_model!(name, M)

Make a tube whose `model` column reads `name` a `Tube{M}`.
"""
function register_tube_model!(name::AbstractString, M::Type{<:AbstractTubeModel})
    TUBE_MODELS[name] = M
    return M
end

"""
    tube_model(name)

The model registered under `name`, or `PlainTube` for `nothing` or an unregistered name.
"""
tube_model(name) = get(TUBE_MODELS, name, PlainTube)
