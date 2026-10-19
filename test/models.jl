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

"""`f()` with `SamPulley` registered as the default pulley model `sam_pulley`."""
function with_sam_pulley(f)
    register_model!(:pulleys, "sam_pulley", SamPulley, ("-", "-"); default=true)
    try
        return f()
    finally
        unregister_model!(:pulleys, "sam_pulley")
    end
end
