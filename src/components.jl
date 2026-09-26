# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

const IDENTITY_QUATERNION = SVector(1.0, 0.0, 0.0, 0.0)

"""
    defaults(T)

The keyword arguments a `T(; kwargs...)` constructor fills in when they are not given.
"""
defaults(::Type{<:Component}) = (; extras=OrderedDict{String, Any}())
function defaults(::Type{Point})
    return (; type=DYNAMIC, body=nothing, extra_mass=0.0, drag_area=0.0,
            drag_coefficient=0.0, defaults(Component)...)
end
defaults(::Type{Station}) = (; type=DYNAMIC, defaults(Component)...)
defaults(::Type{Pulley}) = (; type=DYNAMIC, efficiency=1.0, defaults(Component)...)
defaults(::Type{Winch}) = (; gear_ratio=1.0, defaults(Component)...)
function defaults(::Type{Body})
    return (; type=DYNAMIC, Q_KA_to_ENU=IDENTITY_QUATERNION, extra_mass=0.0,
            extra_inertia_KA=zeros(SMatrix{3, 3, Float64}), defaults(Component)...)
end

"""
    T(; kwargs...) where {T <: Component}

A component of type `T` from one keyword argument per field, with `defaults(T)` filling
those not given. References may be names or indices.
"""
function (T::Type{<:Component})(; kwargs...)
    values = merge(defaults(T), kwargs)
    missing_fields = setdiff(fieldnames(T), keys(values))
    isempty(missing_fields) ||
        throw(ArgumentError("$T needs $(join(missing_fields, ", "))"))
    return T((to_field(fieldtype(T, field), values[field]) for field in fieldnames(T))...)
end

"""
    to_field(T, value)

`value`, as a keyword argument or a structure document gives it, as a field of type `T`.
"""
to_field(T, value) = convert(T, value)
function to_field(::Type{DynamicsType}, name::AbstractString)
    for type in instances(DynamicsType)
        string(type) == name && return type
    end
    throw(ArgumentError("no dynamics type $name"))
end
to_field(T::Type{<:Tuple}, value::AbstractVector) = convert(T, Tuple(value))
to_field(T::Type{<:SMatrix}, rows::AbstractVector) = T(permutedims(reduce(hcat, rows)))
to_field(::Type{Union{Float64, String}}, value::Real) = Float64(value)

"""
    Tube(; kwargs...)

A `Tube` of the model its `model` extra column names, `PlainTube` where it names none.
"""
function Tube(; extras=OrderedDict{String, Any}(), kwargs...)
    return Tube{tube_model(get(extras, "model", nothing))}(; extras, kwargs...)
end
