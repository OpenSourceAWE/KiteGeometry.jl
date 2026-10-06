# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: BLOCKS, REFERENCES, NameRef, connectivity_sha
using JSON
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
const FIXTURES = (FIXTURE, joinpath(@__DIR__, "data", "v3_psm_structure.yml"))

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

@testset "both awesIO V3 documents write back equal, through YAML and JSON as well" begin
    for path in FIXTURES
        document = YAML.load_file(path; dicttype=OrderedDict{String, Any})
        system = load_structure(path)
        @test structure_document(system) == document
        @test structure_document(from_yaml(to_yaml(system))) == document
        @test structure_document(from_json(to_json(system))) == document
        @test structure_document(KiteGeometry.definition(to_json(system))) == document
    end
end

@testset "connectivity_sha hashes the schema's worked examples" begin
    @test connectivity_sha((8, [(1, 2), (2, 3), (3, 4), (4, 5), (4, 7)])) ==
          "d98529e23af6047ce9f49f172743d5dcef053f6a768a3b53de6d47260afd60b7"
    @test connectivity_sha((3, [(1, 2), (2, 3)]), (2, [(1, 2)]), (1, [[1, 2, 3]])) ==
          "1f9a31aca7b6d655aaae4f5de9872d3fd489f907f90316e8f33e3125b1fd18ea"
end

@testset "the writer gives the connectivity of the system's own tables" begin
    document = structure_document(small_system())
    @test document["metadata"]["connectivity_sha"] ==
          "d515a8af37debb0d04e7718442eac3160b21b2aac4546bba68e5aefcab5c68da"
    @test structure_document(SystemDefinition(document)) == document
end

@testset "a connectivity_sha that does not describe the document is refused" begin
    document = fixture_document()
    document["metadata"]["connectivity_sha"] = "0"^64
    @test_throws "connectivity_sha" SystemDefinition(document)
    document = fixture_document()
    reverse!(document["points"]["data"])
    @test_throws "connectivity_sha" SystemDefinition(document)
end

"""The fixture as written against awesIO `version`."""
function versioned_document(version)
    document = fixture_document()
    document["metadata"]["awesIO_version"] = version
    return document
end

@testset "another major awesIO version is refused, another minor warns" begin
    @test_throws "0.1.0" SystemDefinition(versioned_document("0.1.0"))
    @test_throws "2.0.0" KiteGeometry.definition(JSON.json(versioned_document("2.0.0")))
    system = @test_logs (:warn, r"1.1.0") SystemDefinition(versioned_document("1.1.0"))
    @test system.metadata.awesIO_version == "1.1.0"
    @test_logs SystemDefinition(versioned_document("1.0.3"))
    @test_throws ArgumentError SystemDefinition(versioned_document(1.0))
    document = fixture_document()
    delete!(document["metadata"], "awesIO_version")
    @test_throws ArgumentError SystemDefinition(document)
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

"""The modelled fixture with a `colour` column no model names, and an unregistered model
named in the second tube."""
function unread_document()
    document = modelled_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "colour")
    push!(tubes["units"], "-")
    foreach(row -> push!(row, "red"), tubes["data"])
    tubes["data"][2][end - 3] = "unknown_beam"
    return document
end

@testset "a filled column no schema field or registered model names is refused" begin
    @test_throws "colour, model" with_test_beam(() -> SystemDefinition(unread_document()))
    document = unread_document()
    foreach(row -> row[end] = nothing, document["tubes"]["data"])
    @test_throws "tubes columns model" with_test_beam(() -> SystemDefinition(document))
end

@testset "strict=false drops those columns with a warning, and the model is no model" begin
    system = @test_logs (:warn, r"colour, model") with_test_beam(
        () -> SystemDefinition(unread_document(); strict=false))
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

@testset "a table without a model column reads the block's default model" begin
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "EA", "EI")
    push!(tubes["units"], "N", "N*m^2")
    foreach(row -> push!(row, 2e5, 40.0), tubes["data"])
    register_model!(:tubes, "test_beam", TestBeam, ("N", "N*m^2"); default=true)
    try
        system = SystemDefinition(document)
        @test all(tube -> tube.model == TestBeam(2e5, 40.0), system.tubes)
        written = structure_document(system)["tubes"]
        @test unique(row[end - 2] for row in written["data"]) == ["test_beam"]
    finally
        delete!(KiteGeometry.MODELS, (:tubes, "test_beam"))
        delete!(KiteGeometry.DEFAULT_MODELS, :tubes)
    end
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
    held_names.segments[1] = Segment(; name="line", points=("anchor", "nowhere"), l0=100,
                                     diameter=0.004, density=970, unit_stiffness=6e5)
    @test_throws "no points named nowhere" structure_document(held_names)
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
