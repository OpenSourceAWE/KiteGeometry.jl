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

"""A stand-in for SAM's default model of `block`: its V3 columns as they were read."""
struct V3Columns{block} <: AbstractModel
    cells::NamedTuple
    function V3Columns{block}(cells...) where {block}
        return new{block}(NamedTuple{keys(V3_COLUMNS[block])}(cells))
    end
end
KiteGeometry.model_columns(::Type{V3Columns{block}}) where {block} = keys(V3_COLUMNS[block])
KiteGeometry.column_type(::Type{<:V3Columns}, ::Symbol) = Any
function Base.getproperty(model::V3Columns, column::Symbol)
    return column === :cells ? getfield(model, :cells) : getfield(model, :cells)[column]
end

"""`f()` with `V3Columns` registered as the default model of each block of `V3_COLUMNS`."""
function with_v3_columns(f)
    for (block, units) in pairs(V3_COLUMNS)
        register_model!(block, "v3_columns", V3Columns{block}, values(units); default=true)
    end
    try
        return f()
    finally
        foreach(block -> unregister_model!(block, "v3_columns"), keys(V3_COLUMNS))
    end
end
