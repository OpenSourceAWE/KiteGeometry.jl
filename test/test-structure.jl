# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: BLOCKS, REFERENCES, NameRef
using OrderedCollections: OrderedDict
using StaticArrays: SVector
using YAML

struct TestBeam <: AbstractModel
    EA::Float64
    EI::Float64
end

struct StiffBeam <: AbstractModel
    EA::Float64
end

struct ShadowingBeam <: AbstractModel
    diameter::Float64
end

const FIXTURE = joinpath(@__DIR__, "data", "v3_beam_structure.yml")

fixture_document() = YAML.load_file(FIXTURE; dicttype=OrderedDict{String, Any})

"""`f()` with `TestBeam` registered as the tube model `test_beam`."""
function with_test_beam(f)
    register_model!(:tubes, "test_beam", TestBeam, ("N", "N*m^2"))
    try
        return f()
    finally
        delete!(KiteGeometry.MODELS, (:tubes, "test_beam"))
    end
end

"""The fixture with tube columns `model`, `EA` and `EI`: odd tubes the registered
`test_beam`, even ones no model."""
function modelled_document()
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "model", "EA", "EI")
    push!(tubes["units"], "-", "N", "N*m^2")
    for (i, row) in enumerate(tubes["data"])
        push!(row, (isodd(i) ? ("test_beam", 2e5, 40.0) : (nothing, nothing, nothing))...)
    end
    return document
end

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
    @test all(tube -> tube isa Tube{NoModel}, system.tubes)
    face = system.canopy_faces[1]
    @test system.wings[face.wing].name == document["canopy_faces"]["data"][1][2]
    @test system.stations[1].wing == face.wing
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

@testset "model columns and extra blocks come back as they were written" begin
    document = modelled_document()
    document["aero_mesh"] = OrderedDict{String, Any}("panels" => 40)
    system = with_test_beam(() -> SystemDefinition(document))
    @test system.tubes[1] isa Tube{TestBeam}
    @test system.tubes[2] isa Tube{NoModel}
    @test collect(keys(system.extras)) == ["aero_mesh"]
    written = with_test_beam(() -> structure_document(system))
    @test written == document
    @test collect(keys(written)) == collect(keys(document))
end

@testset "an unregistered model is no model, and columns no model names are dropped" begin
    document = modelled_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "colour")
    push!(tubes["units"], "-")
    foreach(row -> push!(row, "red"), tubes["data"])
    tubes["data"][2][end - 3] = "unknown_beam"
    system = with_test_beam(() -> SystemDefinition(document))
    @test system.tubes[1] isa Tube{TestBeam}
    @test system.tubes[2] isa Tube{NoModel}
    @test with_test_beam(() -> structure_document(system)) == modelled_document()
end

@testset "a model column absent or in another unit is refused" begin
    document = modelled_document()
    document["tubes"]["units"][end - 1] = "kN"
    @test_throws ArgumentError with_test_beam(() -> SystemDefinition(document))
    document = modelled_document()
    pop!(document["tubes"]["headers"])
    pop!(document["tubes"]["units"])
    foreach(pop!, document["tubes"]["data"])
    @test_throws ArgumentError with_test_beam(() -> SystemDefinition(document))
end

@testset "a model's columns are its fields, reached through the component" begin
    system = with_test_beam(() -> SystemDefinition(modelled_document()))
    tube = system.tubes[1]
    @test tube.model == TestBeam(2e5, 40.0)
    @test tube.EA == 2e5
    @test :EI in propertynames(tube)
    built = Tube(; name="le", bodies=("a", "b"), diameter=0.1, pressure=3e4,
                 law="breukels2011", model=TestBeam(1, 2))
    @test built isa Tube{TestBeam}
    @test built.EI == 2
end

@testset "a model whose fields shadow the component's is refused" begin
    @test_throws ArgumentError register_model!(:tubes, "shadowing", ShadowingBeam, ("m",))
end

@testset "a model registered without one unit per field is refused" begin
    @test_throws ArgumentError register_model!(:tubes, "test_beam", TestBeam, ("N",))
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
    @test_throws ArgumentError small_system(; rudders=[])
end

@testset "a point count the points disagree with is refused" begin
    points = small_system().points
    metadata = Metadata("small", "", "", "1.0.0", "structure_schema.yml", 3, "0"^64)
    @test_throws ArgumentError SystemDefinition(; metadata, points)
end

@testset "a system holding names still writes them" begin
    system = small_system()
    segments = [Segment(; name="line", points=("anchor", "kite"), l0=100, diameter=0.004,
                        density=970, unit_stiffness=6e5)]
    held_names = SystemDefinition(system.metadata, system.points, segments,
                                  system.stations, system.pulleys, system.tethers,
                                  system.winches, system.wings, system.canopy_faces,
                                  system.bodies, system.tubes, system.extras)
    @test structure_document(held_names) == structure_document(system)
end

@testset "models giving one column two units are refused on writing" begin
    tubes = [Tube(; name="le", bodies=("a", "b"), diameter=0.1, pressure=3e4,
                  law="breukels2011", model=TestBeam(1, 2)),
             Tube(; name="te", bodies=("a", "b"), diameter=0.1, pressure=3e4,
                  law="breukels2011", model=StiffBeam(3))]
    system = small_system(; bodies=[Body(; name="a", pos_ENU=[0, 0, 0]),
                                    Body(; name="b", pos_ENU=[0, 0, 1])], tubes)
    register_model!(:tubes, "stiff_beam", StiffBeam, ("kN",))
    try
        @test_throws ArgumentError with_test_beam(() -> structure_document(system))
    finally
        delete!(KiteGeometry.MODELS, (:tubes, "stiff_beam"))
    end
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
               law="breukels2011") isa Tube{NoModel}
    @test_throws ArgumentError Segment(; name="line", points=("a", "b"))
end

@testset "a table whose headers leave the schema's order is refused" begin
    document = fixture_document()
    headers = document["segments"]["headers"]
    headers[3], headers[4] = headers[4], headers[3]
    @test_throws ArgumentError SystemDefinition(document)
end

@testset "a table whose units are not the schema's is refused" begin
    document = fixture_document()
    document["segments"]["units"][3] = "mm"
    @test_throws ArgumentError SystemDefinition(document)
    delete!(document["segments"], "units")
    @test_throws ArgumentError SystemDefinition(document)
end
