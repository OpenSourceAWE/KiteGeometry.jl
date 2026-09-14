# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry
using KiteUtils
using Test
using LinearAlgebra

#=
Don't add your tests to runtests.jl. Instead, create files named

    test-title-for-my-test.jl

The file will be automatically included inside a `@testset` with title "Title For My Test".
=#

"""
    kite_data_path() -> String

A throwaway copy of the `2plate_kite` fixture, so that a test never writes into
the tracked one.
"""
function kite_data_path()
    data_path = joinpath(mktempdir(), "2plate_kite")
    cp(joinpath(dirname(@__DIR__), "data", "2plate_kite"), data_path)
    return data_path
end

"""
    load_2plate(geometry_file; kwargs...) -> SystemDefinition

The 2-plate kite fixture, loaded from one of its structural geometry files.
"""
function load_2plate(geometry_file; kwargs...)
    data_path = kite_data_path()
    set_data_path(data_path)
    return load_definition(joinpath(data_path, geometry_file);
        name="2plate_kite", set=Settings("system.yaml"), kwargs...)
end

"""
    load_yaml(yaml; kwargs...) -> SystemDefinition

A definition from a YAML string, against the fixture's settings.
"""
function load_yaml(yaml; kwargs...)
    data_path = kite_data_path()
    set_data_path(data_path)
    path = joinpath(data_path, "inline.yaml")
    write(path, yaml)
    return load_definition(path; set=Settings("system.yaml"), kwargs...)
end

for (_, _, files) in walkdir(@__DIR__)
    for file in files
        isnothing(match(r"^test-.*\.jl$", file)) && continue
        title = titlecase(replace(splitext(file[6:end])[1], "-" => " "))
        @testset "$title" begin
            include(file)
        end
    end
end
nothing
