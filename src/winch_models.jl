# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    abstract type AbstractWinchModel

Supertype of a winch's motor model. It carries that model's own parameters; the
drum parameters common to every model (`gear_ratio`, `drum_radius`,
`coulomb_friction`, `viscous_coefficient`, `inertia_total`) stay on
[`Winch`](@ref). How the model turns those parameters into dynamics belongs to
the package that simulates it. Built-in subtype: [`TorqueWinch`](@ref).
"""
abstract type AbstractWinchModel end

"""
    TorqueWinch(; friction_epsilon=6.0)

Torque-controlled winch motor: the winch setpoint is the motor torque [N·m].

$(TYPEDFIELDS)
"""
mutable struct TorqueWinch <: AbstractWinchModel
    "smoothing width of the Coulomb-friction sign function [rad/s]"
    friction_epsilon::SimFloat
end
TorqueWinch(; friction_epsilon=6.0) = TorqueWinch(friction_epsilon)
