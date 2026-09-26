# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: BLOCKS, REFERENCES, NameRef
using OrderedCollections: OrderedDict
using StaticArrays: SVector
using YAML

struct TestBeam <: AbstractTubeModel end

const FIXTURE = joinpath(@__DIR__, "data", "v3_beam_structure.yml")

fixture_document() = YAML.load_file(FIXTURE; dicttype=OrderedDict{String, Any})

function small_system(; kwargs...)
    metadata = Metadata("small", "", "", "1.0.0", "structure_schema.yml", 2, "0"^64)
    points = [Point(; name="anchor", type=STATIC, pos_ENU=[0, 0, 0]),
              Point(; name="kite", pos_ENU=[0, 0, 100])]
    segments = [Segment(; name="line", points=("anchor", "kite"), l0=100, diameter=0.004,
                        density=970, unit_stiffness=6e5)]
    return SystemDefinition(; metadata, points, segments, kwargs...)
end

@testset "every component builds from the vendored example" begin
    document = fixture_document()
    system = load_structure(FIXTURE)
    for block in keys(BLOCKS)
        @test length(getfield(system, block)) == length(document[String(block)]["data"])
    end
    @test system.metadata.n_points == length(system.points)
    point = system.points[1]
    @test point.type == BODY_STATIC
    @test system.bodies[point.body].name == document["points"]["data"][1][3]
    @test point.pos_ENU isa SVector{3, Float64}
    @test all(tube -> tube isa Tube{PlainTube}, system.tubes)
    @test isempty(system.extras)
end

@testset "every reference column names the block it refers into" begin
    reference_types = (NameRef, Union{Nothing, NameRef}, NTuple{2, NameRef},
                       Vector{NameRef})
    for (block, T) in pairs(BLOCKS), field in fieldnames(T)
        refers = fieldtype(Base.unwrap_unionall(T), field) in reference_types
        @test refers == haskey(REFERENCES, (block, field))
    end
end

@testset "extra columns and blocks come back as they were written" begin
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "model", "colour")
    for (i, row) in enumerate(tubes["data"])
        push!(row, isodd(i) ? "test_beam" : "unknown_beam", "red")
    end
    document["aero_mesh"] = OrderedDict{String, Any}("panels" => 40)
    register_tube_model!("test_beam", TestBeam)
    system = SystemDefinition(document)
    @test system.tubes[1] isa Tube{TestBeam}
    @test system.tubes[2] isa Tube{PlainTube}
    @test collect(keys(system.tubes[1].extras)) == ["model", "colour"]
    @test collect(keys(system.extras)) == ["aero_mesh"]
    written = structure_document(system)
    @test written == document
    @test collect(keys(written)) == collect(keys(document))
    @test written["tubes"]["headers"] == tubes["headers"]
end

@testset "names in, indices held" begin
    system = small_system(;
        tethers=[Tether(; name="main", start_point="anchor", end_point=2,
                        segments=["line"])])
    @test system.segments[1].points == (1, 2)
    @test system.tethers[1].start_point == 1
    @test system.tethers[1].segments == [1]
    @test_throws ArgumentError small_system(;
        tethers=[Tether(; name="main", start_point="ground", end_point=2,
                        segments=["line"])])
    @test_throws ArgumentError small_system(;
        tethers=[Tether(; name="main", start_point=3, end_point=2, segments=[1])])
    @test_throws ArgumentError small_system(; wings=[])
end

@testset "constructors fill in the defaults" begin
    point = Point(; name="kite", pos_ENU=[0, 0, 100])
    @test point.type == DYNAMIC
    @test isnothing(point.body)
    @test point.extra_mass == 0
    body = Body(; name="wing", pos_ENU=[0, 0, 100])
    @test body.Q_KA_to_ENU == [1, 0, 0, 0]
    @test iszero(body.extra_inertia_KA)
    @test Tube(; name="le", bodies=("a", "b"), diameter=0.1, pressure=3e4,
               law="breukels2011") isa Tube{PlainTube}
    @test_throws ArgumentError Segment(; name="line", points=("a", "b"))
end

@testset "a table whose headers leave the schema's order is refused" begin
    document = fixture_document()
    headers = document["segments"]["headers"]
    headers[3], headers[4] = headers[4], headers[3]
    @test_throws ArgumentError SystemDefinition(document)
end
