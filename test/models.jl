# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""Take back the model registered for `block` under `name`, and its default."""
function unregister_model!(block, name)
    delete!(KiteGeometry.MODELS, (block, name))
    get(KiteGeometry.DEFAULT_MODELS, block, nothing) == name &&
        delete!(KiteGeometry.DEFAULT_MODELS, block)
end

"""SAM's pulley columns, each optional: a stand-in for the default pulley model SAM
registers."""
struct SamPulley <: AbstractModel
    type::Union{Nothing, String}
    efficiency::Union{Nothing, Float64}
end

"""`f()` with `M` registered as the default model `name` of `block`, its columns in
`units`."""
function with_default_model(f, block, name, M, units)
    register_model!(block, name, M, units; default=true)
    try
        return f()
    finally
        unregister_model!(block, name)
    end
end

"""SAM's segment spring, each column optional and as authored: a stand-in for the default
segment model SAM registers."""
struct SamSpring <: AbstractModel
    unit_damping::Union{Nothing, Float64}
    damping_per_stiffness::Union{Nothing, Float64}
    compression_frac::Union{Nothing, Float64}
    compression_damping_frac::Union{Nothing, Float64}
end

"""The units of the columns of `SamSpring`."""
SPRING_UNITS = ("N*s", "s", "-", "-")
