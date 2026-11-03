# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: AWESIO_VERSION, BLOCKS, REFERENCES, NameRef, connectivity_sha
using JSON
using OrderedCollections: OrderedDict
using StaticArrays: MVector
using YAML

include("models.jl")
register_model!(:pulleys, "sam_pulley", SamPulley, ("-", "-"); default=true)

struct TestBeam <: AbstractModel
    EA::Float64
    EI::Float64
end

"""A tube model with a column of names."""
struct NamedBeam <: AbstractModel
    neighbours::Vector{NameRef}
end

"""A tube model whose type parameter `A` none of its columns name."""
struct TaggedBeam{A} <: AbstractModel
    EA::Float64
end
TaggedBeam(EA) = TaggedBeam{:stiff}(EA)

struct StiffBeam <: AbstractModel
    EA::Float64
end

struct ShadowingBeam <: AbstractModel
    diameter::Float64
end

"""A tube model with a field of its own beside its columns `EA` and `EI`."""
mutable struct LinkedBeam <: AbstractModel
    EA::Float64
    EI::Float64
    scratch::Vector{Float64}
end
LinkedBeam(EA, EI) = LinkedBeam(EA, EI, Float64[])
KiteGeometry.model_columns(::Type{LinkedBeam}) = (:EA, :EI)

const FIXTURE = joinpath(@__DIR__, "data", "structure", "v3_beam_structure.yml")
const FIXTURES = (FIXTURE, joinpath(@__DIR__, "data", "structure", "v3_psm_structure.yml"))
const SYSTEM_FIXTURE = joinpath(@__DIR__, "data", "system_config", "v3_system.yml")

load_document(path) = YAML.load_file(path; dicttype=OrderedDict{String, Any})
fixture_document() = load_document(FIXTURE)
system_fixture() = load_document(SYSTEM_FIXTURE)

"""The `SystemDefinition` of the structure `document` and the V3 system."""
with_system(document; kwargs...) = SystemDefinition(document, system_fixture(); kwargs...)

"""The system document of `system` as YAML text."""
system_text(system) = to_yaml(system_document(system))

"""`f()` with `TestBeam` registered as the tube model `test_beam`."""
function with_test_beam(f)
    register_model!(:tubes, "test_beam", TestBeam, ("N", "N*m^2"))
    try
        return f()
    finally
        unregister_model!(:tubes, "test_beam")
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

"""The segment `line` from the anchor to `kite`."""
function line_segment(kite="kite")
    return Segment(; name="line", points=("anchor", kite), rest_length=100, diameter=0.004)
end

function small_system(; kwargs...)
    metadata = Metadata(; name="small", n_points=2, connectivity_sha="0"^64)
    points = [Point(; name="anchor", fixed=true, pos_ENU=[0, 0, 0]),
              Point(; name="kite", pos_ENU=[0, 0, 100])]
    return SystemDefinition(; metadata, points, segments=[line_segment()], kwargs...)
end

@testset "every component builds from the vendored example" begin
    document = fixture_document()
    system = load_structure(FIXTURE)
    for block in keys(BLOCKS)
        @test length(getfield(system, block)) == length(document[String(block)]["data"])
    end
    @test system.metadata.n_points == length(system.points)
    point = system.points[1]
    @test !point.fixed
    @test system.bodies[point.body].name == Symbol(document["points"]["data"][1][3])
    @test system.materials[system.segments[1].material].law == "linear"
    @test system.materials[system.tubes[1].material].parameters["C19"] == 0.0918
    @test point.pos_ENU isa MVector{3, Float64}
    @test all(tube -> tube isa Tube{NoModel}, system.tubes)
    face = system.canopy_faces[1]
    @test system.wings[face.wing].name == Symbol(document["canopy_faces"]["data"][1][2])
    @test system.spanwise_stations[1].wing == face.wing
    @test system.wings[face.wing].Q_KA2ENU == [1, 0, 0, 0]
    @test isempty(system.extras)
end

@testset "both awesIO V3 documents write back equal, through YAML and JSON as well" begin
    for path in FIXTURES
        document = load_document(path)
        system = load_structure(path)
        @test structure_document(system) == document
        @test system_document(system) == system_fixture()
        for reloaded in (from_yaml(to_yaml(system), system_text(system)),
                         from_json(to_json(system), to_json(system_document(system))),
                         KiteGeometry.definition(to_json(system),
                                                 to_json(system_document(system))))
            @test structure_document(reloaded) == document
            @test system_document(reloaded) == system_fixture()
        end
    end
end

@testset "an optional column a document leaves out is written back left out" begin
    document = fixture_document()
    points = document["points"]
    pop!(points["headers"])
    pop!(points["units"])
    foreach(pop!, points["data"])
    system = with_system(document)
    @test all(point -> isnothing(point.extra_mass), system.points)
    @test structure_document(system) == document
end

@testset "a position in the world is put into a wing's KA frame" begin
    wing = Wing(; name="wing", pos_ENU=[3, 4, 50],
                Q_KA2ENU=[0.7071067811865476, 0, 0, 0.7071067811865476])
    @test isapprox(pos_KA(wing, [3, 6, 51]), [2, 0, 1]; atol=1e-12)
    @test pos_KA(Wing(; name="wing", pos_ENU=[0, 0, 100]), [1, 2, 103]) == [1, 2, 3]
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
    @test structure_document(with_system(document)) == document
end

@testset "a connectivity_sha that does not describe the document is refused" begin
    document = fixture_document()
    document["metadata"]["connectivity_sha"] = "0"^64
    @test_throws "connectivity_sha" with_system(document)
    document = fixture_document()
    reverse!(document["points"]["data"])
    @test_throws "connectivity_sha" with_system(document)
end

"""The fixture as written against awesIO `version`."""
function versioned_document(version)
    document = fixture_document()
    document["metadata"]["awesIO_version"] = version
    return document
end

@testset "another major awesIO version is refused, another minor warns" begin
    @test_throws "1.0.0" with_system(versioned_document("1.0.0"))
    @test_throws "2.0.0" KiteGeometry.definition(JSON.json(versioned_document("2.0.0")),
                                                      JSON.json(system_fixture()))
    system = @test_logs (:warn, r"0.2.0") with_system(versioned_document("0.2.0"))
    @test system.metadata.awesIO_version == "0.2.0"
    @test_logs with_system(versioned_document("0.1.3"))
    @test_throws ArgumentError with_system(versioned_document(1.0))
    document = fixture_document()
    delete!(document["metadata"], "awesIO_version")
    @test_throws ArgumentError with_system(document)
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
    system = with_test_beam(() -> with_system(document))
    @test system.tubes[1] isa Tube{TestBeam}
    @test system.tubes[2] isa Tube{NoModel}
    @test collect(keys(system.extras)) == ["aero_mesh"]
    written = with_test_beam(() -> structure_document(system))
    @test written == document
    @test collect(keys(written)) == collect(keys(document))
end

@testset "a model column of names reads Symbols and writes strings" begin
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "model", "neighbours")
    push!(tubes["units"], "-", "-")
    foreach(row -> push!(row, "named_beam", ["le", "te"]), tubes["data"])
    register_model!(:tubes, "named_beam", NamedBeam, ("-",))
    try
        system = with_system(document)
        @test system.tubes[1].neighbours == [:le, :te]
        @test structure_document(system) == document
    finally
        unregister_model!(:tubes, "named_beam")
    end
end

@testset "a row whose model is an instance of a registered parametric model writes" begin
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "model", "EA")
    push!(tubes["units"], "-", "N")
    foreach(row -> push!(row, "tagged_beam", 1.0e5), tubes["data"])
    register_model!(:tubes, "tagged_beam", TaggedBeam, ("N",))
    try
        system = with_system(document)
        @test system.tubes[1].model isa TaggedBeam{:stiff}
        @test structure_document(system) == document
    finally
        unregister_model!(:tubes, "tagged_beam")
    end
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
    @test_throws "colour, model" with_test_beam(() -> with_system(unread_document()))
    document = unread_document()
    foreach(row -> row[end] = nothing, document["tubes"]["data"])
    @test_throws "tubes columns model" with_test_beam(() -> with_system(document))
end

@testset "strict=false drops those columns with a warning, and the model is no model" begin
    system = @test_logs (:warn, r"colour, model") with_test_beam(
        () -> with_system(unread_document(); strict=false))
    @test system.tubes[1] isa Tube{TestBeam}
    @test system.tubes[2] isa Tube{NoModel}
    @test with_test_beam(() -> structure_document(system)) == modelled_document()
end

@testset "a model column absent or in another unit is refused" begin
    document = modelled_document()
    document["tubes"]["units"][end - 1] = "kN"
    @test_throws ArgumentError with_test_beam(() -> with_system(document))
    document = modelled_document()
    pop!(document["tubes"]["headers"])
    pop!(document["tubes"]["units"])
    foreach(pop!, document["tubes"]["data"])
    @test_throws ArgumentError with_test_beam(() -> with_system(document))
end

@testset "a model's columns are its fields, reached through the component" begin
    system = with_test_beam(() -> with_system(modelled_document()))
    tube = system.tubes[1]
    @test tube.model == TestBeam(2e5, 40.0)
    @test tube.EA == 2e5
    @test :EI in propertynames(tube)
    built = Tube(; name="le", bodies=("a", "b"), diameter=0.1, pressure=3e4,
                 model=TestBeam(1, 2))
    @test built isa Tube{TestBeam}
    @test built.EI == 2
end

"""`f()` with `LinkedBeam` registered as the tube model `test_beam`."""
function with_linked_beam(f)
    register_model!(:tubes, "test_beam", LinkedBeam, ("N", "N*m^2"))
    try
        return f()
    finally
        unregister_model!(:tubes, "test_beam")
    end
end

@testset "a model's fields beyond its columns are neither read nor written" begin
    document = modelled_document()
    system = with_linked_beam(() -> with_system(document))
    tube = system.tubes[1]
    @test tube.model isa LinkedBeam
    @test isempty(tube.scratch)
    push!(tube.scratch, 1.0)
    @test with_linked_beam(() -> structure_document(system)) == document
end

@testset "a row's fields and its model's are set in place" begin
    system = with_linked_beam(() -> with_system(modelled_document()))
    point = system.points[1]
    point.extra_mass = 2.5
    point.pos_ENU .= [1, 2, 3]
    @test point.extra_mass == 2.5
    @test point.pos_ENU == [1, 2, 3]
    tube = system.tubes[1]
    tube.EA = 3e5
    @test tube.model.EA == 3e5
    written = with_linked_beam(() -> structure_document(system))
    @test written["points"]["data"][1][4] == [1, 2, 3]
    @test written["tubes"]["data"][1][end - 1] == 3e5
end

@testset "rows and models are equal, and hash alike, where every field is" begin
    one, other = load_structure(FIXTURE), load_structure(FIXTURE)
    @test one.points == other.points
    @test hash(one.points[1]) == hash(other.points[1])
    other.points[1].extra_mass = something(one.points[1].extra_mass, 0.0) + 1
    @test one.points[1] != other.points[1]
    @test LinkedBeam(1, 2) == LinkedBeam(1, 2)
end

@testset "a table without a model column reads the block's default model" begin
    document = fixture_document()
    tubes = document["tubes"]
    push!(tubes["headers"], "EA", "EI")
    push!(tubes["units"], "N", "N*m^2")
    foreach(row -> push!(row, 2e5, 40.0), tubes["data"])
    register_model!(:tubes, "test_beam", TestBeam, ("N", "N*m^2"); default=true)
    try
        system = with_system(document)
        @test all(tube -> tube.model == TestBeam(2e5, 40.0), system.tubes)
        @test structure_document(system) == document
    finally
        unregister_model!(:tubes, "test_beam")
    end
end

@testset "a model whose fields shadow the component's is refused" begin
    @test_throws ArgumentError register_model!(:tubes, "shadowing", ShadowingBeam, ("m",))
end

@testset "a model registered without one unit per field is refused" begin
    @test_throws ArgumentError register_model!(:tubes, "test_beam", TestBeam, ("N",))
end

@testset "a field of one number or one per axis takes either" begin
    field = Union{Nothing, Float64, Vector{Float64}}
    @test KiteGeometry.to_field(field, 2) === 2.0
    @test KiteGeometry.to_field(field, [1, 2, 3]) == [1.0, 2.0, 3.0]
    @test isnothing(KiteGeometry.to_field(field, nothing))
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

@testset "a block's rows are found by their name as well as their position" begin
    system = small_system()
    @test system.points[:kite] === system.points["kite"] === system.points[2]
    @test haskey(system.points, :anchor) && !haskey(system.points, :ground)
    @test_throws KeyError system.points[:ground]
    push!(system.segments, Segment(; name="spare", points=(1, 2)))
    @test system.segments[:spare].points == (1, 2)
end

@testset "a name two rows share is refused on lookup and on writing" begin
    system = small_system()
    system.points[2].name = :anchor
    @test_throws "two points are named anchor" system.points[:anchor]
    @test_throws "two points are named anchor" structure_document(system)
    push!(system.segments, Segment(; name="line", points=(1, 2)))
    @test_throws "two segments are named line" system.segments[:line]
end

@testset "metadata by keyword defaults to this awesIO version and an empty checksum" begin
    @test Metadata(; name="small", n_points=2) ==
          Metadata("small", "", "", AWESIO_VERSION, "structure_schema.yml", 2, "", "", "")
end

@testset "a point count the points disagree with is refused" begin
    points = small_system().points
    metadata = Metadata(; name="small", n_points=3, connectivity_sha="0"^64)
    @test_throws ArgumentError SystemDefinition(; metadata, points)
end

@testset "a system holding names still writes them" begin
    system = small_system()
    held_names = SystemDefinition(system.metadata, system.system_metadata, system.materials,
                                  system.system_tethers, system.points, [line_segment()],
                                  system.spanwise_stations, system.pulleys, system.tethers,
                                  system.winches, system.wings, system.canopy_faces,
                                  system.bodies, system.tubes, system.extras,
                                  system.system_extras)
    @test structure_document(held_names) == structure_document(system)
    held_names.segments[1] = line_segment("nowhere")
    @test_throws "no points named nowhere" structure_document(held_names)
end

@testset "models giving one column two units are refused on writing" begin
    tubes = [Tube(; name="le", bodies=("a", "b"), model=TestBeam(1, 2)),
             Tube(; name="te", bodies=("a", "b"), model=StiffBeam(3))]
    system = small_system(; bodies=[Body(; name="a", pos_ENU=[0, 0, 0]),
                                    Body(; name="b", pos_ENU=[0, 0, 1])], tubes)
    register_model!(:tubes, "stiff_beam", StiffBeam, ("kN",))
    try
        @test_throws ArgumentError with_test_beam(() -> structure_document(system))
    finally
        unregister_model!(:tubes, "stiff_beam")
    end
end

@testset "constructors fill in the defaults" begin
    point = Point(; name="kite", pos_ENU=[0, 0, 100])
    @test !point.fixed
    @test isnothing(point.body)
    @test isnothing(point.extra_mass)
    body = Body(; name="wing", pos_ENU=[0, 0, 100])
    @test body.Q_KA2ENU == [1, 0, 0, 0]
    @test isnothing(body.extra_inertia_KA)
    @test Wing(; name="wing", pos_ENU=[0, 0, 100]).Q_KA2ENU == [1, 0, 0, 0]
    @test Tube(; name="le", bodies=("a", "b")) isa Tube{NoModel}
    @test_throws ArgumentError Point(; name="kite")
end

@testset "a table whose headers leave the schema's order is refused" begin
    document = fixture_document()
    headers = document["points"]["headers"]
    headers[2], headers[3] = headers[3], headers[2]
    @test_throws ArgumentError with_system(document)
end

@testset "a table whose units are not the schema's is refused" begin
    document = fixture_document()
    document["segments"]["units"][3] = "mm"
    @test_throws ArgumentError with_system(document)
    delete!(document["segments"], "units")
    @test_throws ArgumentError with_system(document)
end


"""The V3 pair copied to a directory laid out as awesIO's examples, given to `f`, its system
file's bytes first changed by `change`."""
function with_v3_copy(f; change=identity)
    mktempdir() do directory
        structure = joinpath(directory, "structure", "v3_beam_structure.yml")
        system = joinpath(directory, "system_config", "v3_system.yml")
        mkpath.(dirname.((structure, system)))
        cp(FIXTURE, structure)
        write(system, change(read(SYSTEM_FIXTURE, String)))
        return f(structure, system)
    end
end

@testset "a structure is read with the system it names, whose hash it holds" begin
    system = load_structure(FIXTURE)
    @test system.metadata.system_sha == system_sha(SYSTEM_FIXTURE)
    @test [tether.name for tether in system.system_tethers] == [:tether, Symbol("1")]
    tether = system.system_tethers[:tether]
    @test (tether.length, tether.diameter) == (250.0, 0.004)
    @test system.materials[tether.material].name == :v3_beam_line
    @test tether.joints["upper_tether_joint"]["id"] == "kcu"
    @test isempty(system.system_extras)
    @test system.system_metadata.name == "v3_kite"
end

@testset "a structure whose system_sha is not its system file's hash is refused" begin
    with_v3_copy(; change=text -> text * "\n") do structure, system
        @test_throws "system_sha" load_structure(structure)
    end
    with_v3_copy() do structure, system
        @test load_structure(structure).metadata.system_sha == system_sha(system)
        document = load_document(structure)
        document["metadata"]["system"] = ""
        write(structure, to_yaml(document))
        @test_throws "names no system file" load_structure(structure)
    end
end

@testset "a structure naming a material its system lacks is refused" begin
    system = system_fixture()
    filter!(material -> material["name"] != "v3_beam_line", system["materials"])
    system["components"]["tethers"] = []
    @test_throws "no materials named v3_beam_line" (
        SystemDefinition(fixture_document(), system))
end

@testset "a system without its required blocks or with a tether's density is refused" begin
    for block in ("materials", "components")
        system = system_fixture()
        delete!(system, block)
        @test_throws "no $block block" SystemDefinition(fixture_document(), system)
    end
    system = system_fixture()
    delete!(system["components"], "tethers")
    @test_throws "components.tethers" SystemDefinition(fixture_document(), system)
    system = system_fixture()
    system["components"]["tethers"][1]["structure"]["density"] = 970.0
    @test_throws "density" SystemDefinition(fixture_document(), system)
end

@testset "what a system holds beside materials and tethers is written back" begin
    system = system_fixture()
    assembly = OrderedDict{String, Any}("connectivity_matrix" => [["kite", "tether"]])
    system["assembly"] = assembly
    system["components"]["ground_station"] = OrderedDict{String, Any}("name" => "winch")
    definition = SystemDefinition(fixture_document(), system)
    written = system_document(definition)
    @test written["assembly"] == assembly
    @test written["components"]["ground_station"]["name"] == "winch"
    @test length(written["components"]["tethers"]) == 2
end

"""The V3 beam structure with a system of one tether, `tether_name`, of `length`."""
function narrowed_system(tether_name, length)
    system = load_structure(FIXTURE)
    tether = KiteGeometry.with_fields(first(system.system_tethers); name=Symbol(tether_name),
                                      length)
    empty!(system.system_tethers)
    push!(system.system_tethers, tether)
    return system
end

@testset "a structure disagreeing with its system's tether is refused" begin
    refused(change!) = SystemDefinition(fixture_document(),
                                        change!(system_fixture()))
    function tether_field!(system, field, value)
        first(system["components"]["tethers"])["structure"][field] = value
        return system
    end
    @test_throws "tether tether is not in the system" refused() do system
        system["components"]["tethers"] = system["components"]["tethers"][2:end]
        return system
    end
    @test_throws "add up to" refused(system -> tether_field!(system, "length", 251.0))
    @test_throws "has diameter" refused(system -> tether_field!(system, "diameter", 0.005))
    @test_throws "not of the system's material" refused(
        system -> tether_field!(system, "material", "control_line"))
    @test SystemDefinition(fixture_document(),
                           tether_field!(system_fixture(), "diameter", 0.004)) isa
          SystemDefinition
end

@testset "a value no document can hold is refused by name when writing" begin
    force_law(strain) = strain
    @test_throws "cannot hold force_law" to_yaml(OrderedDict{String, Any}("law" => force_law))
end

@testset "a definition naming no system file does not write as a structure file" begin
    system = small_system()
    @test_throws "names no system file" to_yaml(system)
    @test structure_document(system)["metadata"]["system"] == ""
end

@testset "a system written from several definitions is the union of their rows" begin
    one, other = narrowed_system("one", 100.0), narrowed_system("other", 200.0)
    document = system_document([one, other]; name="pair", description="both", note="noted")
    @test document["metadata"]["name"] == "pair"
    @test document["metadata"]["description"] == "both"
    @test document["metadata"]["note"] == "noted"
    @test document["metadata"]["schema"] == "system_schema.yml"
    @test [tether["name"] for tether in document["components"]["tethers"]] ==
          ["one", "other"]
    @test document["materials"] == system_fixture()["materials"]
    @test system_document([one, one]; name="same")["components"]["tethers"] ==
          system_document(one)["components"]["tethers"]
end

@testset "one name with two values is refused when writing a system" begin
    one, other = narrowed_system("one", 100.0), narrowed_system("one", 200.0)
    @test_throws "tethers row one" system_document([one, other]; name="pair")
    changed = load_structure(FIXTURE)
    changed.materials[:v3_beam_line].parameters = OrderedDict{String, Any}("density" => 1.0)
    @test_throws "materials row v3_beam_line" (
        system_document([load_structure(FIXTURE), changed]; name="pair"))
end

@testset "a structure is written with the system's path and hash and a note" begin
    document = structure_document(load_structure(FIXTURE); system="../system.yml",
                                  system_sha="a"^64, note="written",
                                  description="described")
    @test document["metadata"]["system"] == "../system.yml"
    @test document["metadata"]["system_sha"] == "a"^64
    @test document["metadata"]["note"] == "written"
    @test document["metadata"]["description"] == "described"
end

@testset "a pair written to disk loads back through its hash" begin
    mktempdir() do directory
        loaded = load_structure(FIXTURE)
        system_path, structure_path = joinpath.(directory, ("system.yml", "structure.yml"))
        write(system_path, system_text(loaded))
        write(structure_path, to_yaml(structure_document(
            loaded; system="system.yml", system_sha=system_sha(system_path))))
        reloaded = load_structure(structure_path)
        @test system_document(reloaded) == system_fixture()
        @test reloaded.metadata.system_sha == system_sha(system_path)
    end
end

@testset "the writer's YAML reads the same everywhere" begin
    text = to_yaml(OrderedDict{String, Any}(
        "units" => ["-", "-", "m", "N*s", "N"],
        "names" => ["1", "true", "null", "no", "a b", "1e3", "~", "plain_name.yml"],
        "numbers" => [1e-12, 5.5e10, 1.0, -2.5e-7, 1.5e300], "count" => 3,
        "empty" => OrderedDict{String, Any}(), "none" => [], "unset" => nothing,
        "rows" => [["a", [1.0, 2.0], nothing, true], ["b", [3.0, 4.0], false, false]],
        "list" => [OrderedDict{String, Any}(
            "name" => "x", "nested" => OrderedDict{String, Any}("depth" => 1.0))],
        "note" => "says \"quoted\": things\nand a second line 0.5°"))
    @test occursin("""units: ["-", "-", m, "N*s", "N"]\n""", text)
    @test occursin("numbers: [1.0e-12, 5.5e+10, 1.0, -2.5e-7, 1.5e+300]\n", text)
    @test occursin(
        """names: ["1", "true", "null", "no", "a b", "1e3", "~", plain_name.yml]""", text)
    @test occursin("  - [a, [1.0, 2.0], null, true]\n", text)
    @test occursin("  - name: x\n    nested:\n      depth: 1.0\n", text)
    parsed = YAML.load(text; dicttype=OrderedDict{String, Any})
    @test parsed["units"] == ["-", "-", "m", "N*s", "N"]
    @test parsed["names"] ==
          ["1", "true", "null", "no", "a b", "1e3", "~", "plain_name.yml"]
    @test parsed["numbers"] == [1e-12, 5.5e10, 1.0, -2.5e-7, 1.5e300]
    @test parsed["count"] === 3
    @test parsed["empty"] == Dict() && parsed["none"] == [] && isnothing(parsed["unset"])
    @test parsed["rows"][2] == ["b", [3.0, 4.0], false, false]
    @test parsed["list"][1]["nested"]["depth"] == 1.0
    @test parsed["note"] == "says \"quoted\": things\nand a second line 0.5°"
end

"""Every float among the values of a parsed document."""
floats(value::AbstractFloat) = [value]
floats(value::AbstractDict) = reduce(vcat, floats.(values(value)); init=Float64[])
floats(value::AbstractVector) = reduce(vcat, floats.(value); init=Float64[])
floats(value) = Float64[]

@testset "every float the writer gives both V3 files matches YAML 1.1's float pattern" begin
    pyyaml_float = r"^[-+]?[0-9]+\.[0-9]*([eE][-+][0-9]+)?$"
    for path in FIXTURES
        loaded = load_structure(path)
        for document in (structure_document(loaded), system_document(loaded))
            values = floats(document)
            @test !isempty(values)
            @test all(occursin.(pyyaml_float, KiteGeometry.yaml_flow.(values)))
            @test YAML.load(to_yaml(document);
                            dicttype=OrderedDict{String, Any}) == document
        end
    end
end

unregister_model!(:pulleys, "sam_pulley")
