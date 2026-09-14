# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    abstract type AbstractSegmentModel

Supertype of a spring-damper element's force law. The concrete subtype is the
type parameter of [`Segment`](@ref) and [`Tether`](@ref), so a collection of
segments sharing one law is concretely typed. Built-in subtypes:
[`LinearSpring`](@ref), [`NonlinearSpring`](@ref).
"""
abstract type AbstractSegmentModel end

"""
    LinearSpring(unit_stiffness, unit_damping)

Spring-damper whose stiffness and damping scale inversely with length:
`k = unit_stiffness / length` [N/m] and `c = unit_damping / length` [N·s/m].

$(TYPEDFIELDS)
"""
struct LinearSpring <: AbstractSegmentModel
    "stiffness per unit length [N]"
    unit_stiffness::SimFloat
    "damping per unit length [N·s]"
    unit_damping::SimFloat
end

"""
    NonlinearSpring(force_law, unit_damping)

Spring-damper whose spring force is `force_law(ε)` [N] of the axial strain
`ε = (len − l0) / l0`. The law owns the whole force curve, slack and
compression included, so `compression_frac` does not apply to it. Damping stays
linear: `c = unit_damping / length` [N·s/m].

$(TYPEDFIELDS)
"""
struct NonlinearSpring{F} <: AbstractSegmentModel
    "callable `force_law(ε)` returning the spring force [N]"
    force_law::F
    "damping per unit length [N·s]"
    unit_damping::SimFloat
end

"""
    segment_model(label, set; unit_stiffness, unit_damping, diameter,
                  density, youngs_modulus, damping_per_stiffness)
        -> (model, diameter, density)

Complete the elastic properties of a spring element.

`unit_stiffness` [N] and `unit_damping` [N·s] scale with the cross section, so
they describe one element rather than a material. A material shared by elements
of different diameter is given as `youngs_modulus` [Pa] and
`damping_per_stiffness` [s] instead: `unit_stiffness = youngs_modulus·π(d/2)²`
and `unit_damping = damping_per_stiffness·unit_stiffness`. Giving both forms of
the same quantity is an error. What is left `NaN` comes from `set` (`d_tether`,
`rho_tether`, `e_tether`, `rel_damping`). A callable `unit_stiffness` builds a
[`NonlinearSpring`](@ref) and needs an explicit `unit_damping`.
"""
function segment_model(label, set; unit_stiffness=NaN, unit_damping=NaN,
    diameter=NaN, density=NaN, youngs_modulus=NaN, damping_per_stiffness=NaN
)
    isnan(diameter) && (diameter = 0.001 * set.d_tether)
    isnan(density) && (density = set.rho_tether)
    area = π * (diameter / 2)^2

    if !isnan(youngs_modulus)
        (!(unit_stiffness isa Real) || !isnan(unit_stiffness)) &&
            error("$label: give either `unit_stiffness` or `youngs_modulus`, " *
                  "not both.")
        unit_stiffness = youngs_modulus * area
    elseif unit_stiffness isa Real && isnan(unit_stiffness)
        unit_stiffness = set.e_tether * area
    end

    unit_damping = resolve_unit_damping(label, set, unit_stiffness,
        unit_damping, damping_per_stiffness)
    model = unit_stiffness isa Real ?
        LinearSpring(unit_stiffness, unit_damping) :
        NonlinearSpring(unit_stiffness, unit_damping)
    return model, SimFloat(diameter), SimFloat(density)
end

"""
    resolve_unit_damping(label, set, unit_stiffness, unit_damping,
                         damping_per_stiffness) -> SimFloat

Damping per unit length [N·s], from `unit_damping`, from
`damping_per_stiffness · unit_stiffness`, or from the settings' damping ratio.
"""
function resolve_unit_damping(label, set, unit_stiffness, unit_damping,
                              damping_per_stiffness)
    if !isnan(damping_per_stiffness)
        isnan(unit_damping) || error("$label: give either `unit_damping` or " *
            "`damping_per_stiffness`, not both.")
        unit_stiffness isa Real || error("$label: `damping_per_stiffness` " *
            "needs a linear `unit_stiffness`.")
        return SimFloat(damping_per_stiffness * unit_stiffness)
    end
    isnan(unit_damping) || return SimFloat(unit_damping)
    unit_stiffness isa Real || error("$label: `unit_damping` must be given " *
        "explicitly when `unit_stiffness` is a nonlinear force law.")
    if hasproperty(set, :rel_damping) && set.rel_damping != 0.0
        return SimFloat(set.rel_damping * unit_stiffness)
    elseif hasproperty(set, :unit_damping) && hasproperty(set, :unit_stiffness) &&
            set.unit_damping != 0.0
        return SimFloat(set.unit_damping / set.unit_stiffness * unit_stiffness)
    end
    @warn "$label: unit_damping is zero " *
        "(no rel_damping or unit_damping in settings)."
    return zero(SimFloat)
end
