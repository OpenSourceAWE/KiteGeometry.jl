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

"""The columns SAM's default models add to awesIO's V3 files, block by block, with units."""
const V3_COLUMNS = (
    points=(type="-", body_frame_damping="N*s/m", world_frame_damping="N*s/m", area="m^2",
            drag_coeff="-", tube="-", wing_idx="-"),
    spanwise_stations=(type="-", moment_frac="-", damping="N*m*s/rad", flap_points="-",
                       flap_axis="-"),
    pulleys=(type="-", efficiency="-"),
    wings=(aero_scale_chord="-", aero_z_offset="m", origin_idx="-", z_ref_points="-",
           y_ref_points="-", stations="-"),
    bodies=(type="-", wing="-"),
    tethers=(init_stretched_length="m",),
    tubes=(shear_coeff="-", damping="s"),
)

"""A struct standing in for SAM's default model of `block`, one field per column, each
holding its cell as it was read."""
function v3_model(block, columns)
    name = Symbol(:V3, join(uppercasefirst.(split(String(block), '_'))))
    fields = (:($column::Any) for column in columns)
    return Core.eval(@__MODULE__, :(struct $name <: AbstractModel
                                        $(fields...)
                                    end; $name))
end

"""The stand-ins for SAM's default models, block by block."""
const V3_MODELS = NamedTuple{keys(V3_COLUMNS)}(Tuple(
    v3_model(block, keys(units)) for (block, units) in pairs(V3_COLUMNS)))

"""`f()` with `V3_MODELS` registered as the default models of their blocks, and `SamSpring`
as the segment model `spring` the V3 files name on every segment."""
function with_v3_columns(f)
    for (block, units) in pairs(V3_COLUMNS)
        register_model!(block, "v3_columns", V3_MODELS[block], values(units); default=true)
    end
    register_model!(:segments, "spring", SamSpring, SPRING_UNITS)
    try
        return f()
    finally
        foreach(block -> unregister_model!(block, "v3_columns"), keys(V3_COLUMNS))
        unregister_model!(:segments, "spring")
    end
end
