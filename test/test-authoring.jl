# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using JSON
using JSONSchema: Schema, validate
using KiteUtils: Settings, set_data_path
using OrderedCollections: OrderedDict
using YAML

include("models.jl")

KITE = joinpath(@__DIR__, "data", "2plate_kite")
AUTHORING_SYSTEMS = filter(endswith("_structural_geometry.yaml"), readdir(KITE))
SCHEMA = Schema(YAML.load_file(joinpath(pkgdir(KiteGeometry), "src", "awesio",
                                         "structure_schema.yml")))

"""SAM's point columns, as a stand-in for the default point model SAM registers."""
struct SamPoint <: AbstractModel
    body_frame_damping::Float64
    world_frame_damping::Float64
end

"""`SamPoint` with the tube a point rides."""
struct RidingPoint <: AbstractModel
    body_frame_damping::Float64
    world_frame_damping::Float64
    tube::Union{Nothing, String}
end

"""SAM's station columns."""
struct SamStation <: AbstractModel
    moment_frac::Float64
    damping::Float64
end

"""SAM's wing columns, each optional."""
struct SamWing <: AbstractModel
    aero_z_offset::Union{Nothing, Float64}
    aero_scale_chord::Union{Nothing, Float64}
end

"""SAM's Timoshenko beam, a tube model chosen by name."""
struct Timoshenko <: AbstractModel
    EI::Float64
    GJ::Float64
    shear_coeff::Float64
end

SAM_MODELS = [(:points, "sam_point", SamPoint, ("N*s/m", "N*s/m"), true),
              (:stations, "sam_station", SamStation, ("-", "N*m*s"), true),
              (:wings, "sam_wing", SamWing, ("m", "-"), true),
              (:tubes, "timoshenko", Timoshenko, ("N*m^2", "N*m^2", "-"), false)]

"""Register the stand-ins `models` for SAM's models."""
function register_sam_models!(models=SAM_MODELS)
    for (block, name, M, units, default) in models
        register_model!(block, name, M, units; default)
    end
end

"""`f()` with the stand-ins for SAM's models on `blocks` taken back."""
function without_sam_models(f, blocks...)
    models = [model for model in SAM_MODELS if first(model) in blocks]
    foreach(model -> unregister_model!(model[1], model[2]), models)
    try
        return f()
    finally
        register_sam_models!(models)
    end
end

register_sam_models!()

"""The settings the kite in `directory` is authored against."""
function kite_settings(directory=KITE)
    set_data_path(directory)
    return Settings("system.yaml")
end

"""The 2-plate kite of the authoring `file`, loaded and placed."""
load_kite(file) = load_authoring(joinpath(KITE, file); set=kite_settings())

"""Whether `value` and `expected`, cells of a document, agree to rounding."""
cells_agree(value::Real, expected::Real) = isapprox(value, expected; rtol=1e-12, atol=1e-12)
function cells_agree(value::AbstractVector, expected::AbstractVector)
    return length(value) == length(expected) && all(cells_agree.(value, expected))
end
cells_agree(value, expected) = value == expected

"""The cell of `column` in each row of `table`, or `nothing` where it has no such column."""
function column(table, header)
    index = findfirst(==(header), table["headers"])
    isnothing(index) && return nothing
    return [row[index] for row in table["data"]]
end

"""`system` with the canopy faces of `other`."""
function with_canopy_of(system, other)
    blocks = NamedTuple(block => getfield(system, block)
                        for block in keys(KiteGeometry.BLOCKS) if block != :canopy_faces)
    return SystemDefinition(; system.metadata, system.extras, blocks...,
                            canopy_faces=other.canopy_faces)
end

@testset "the rigid 2-plate kite loads as SAM's golden document describes it" begin
    golden_path = joinpath(@__DIR__, "data", "2plate_kite_structure.yml")
    golden = YAML.load_file(golden_path; dicttype=OrderedDict{String, Any})
    system = load_kite("rigid_structural_geometry.yaml")
    document = structure_document(system)
    @test document["metadata"]["n_points"] == golden["metadata"]["n_points"]
    @test column(document["canopy_faces"], "points") ==
          [["le_left", "te_left", "te_center", "le_center"],
           ["le_center", "te_center", "te_right", "le_right"]]
    golden_system = without_sam_models(() -> load_structure(golden_path; strict=false),
                                       :points, :stations, :wings)
    @test document["metadata"]["connectivity_sha"] ==
          connectivity_sha(with_canopy_of(golden_system, system))
    unread = String[]
    for (block, expected) in golden
        block == "metadata" && continue
        isempty(expected["data"]) && continue
        table = document[block]
        @test column(table, "name") == column(expected, "name")
        for header in expected["headers"]
            values = column(table, header)
            isnothing(values) && (push!(unread, "$block.$header"); continue)
            for (name, value, cell) in zip(column(expected, "name"), values,
                                           column(expected, header))
                cells_agree(value, cell) || @error "$block $name $header" value cell
                @test cells_agree(value, cell)
            end
        end
    end
    @test sort(unread) == ["bodies.apparent_mass", "bodies.wing", "points.wing",
                           "stations.stiffness", "winches.model", "wings.aero"]
end

@testset "every authoring system writes a document the vendored schema accepts" begin
    @test length(AUTHORING_SYSTEMS) == 2
    for file in AUTHORING_SYSTEMS
        system = load_kite(file)
        document = JSON.parse(to_json(system))
        @test isnothing(validate(SCHEMA, document))
        @test length(system.canopy_faces) == 2
        @test structure_document(from_yaml(to_yaml(system))) == structure_document(system)
    end
end

@testset "a segment carries its spring as the segment model spring" begin
    system = load_kite("rigid_structural_geometry.yaml")
    segment = system.segments[1]
    @test segment isa Segment{SegmentSpring}
    @test segment.unit_stiffness ≈ 5000.0
    @test segment.unit_damping ≈ 10.0
    @test model_type(:segments, "spring") === SegmentSpring
end

@testset "the tether is placed at its stretched length, its points evenly along it" begin
    system = load_kite("particle_structural_geometry.yaml")
    tether = only(system.tethers)
    start_pos = system.points[tether.start_point].pos_ENU
    end_pos = system.points[tether.end_point].pos_ENU
    @test hypot((start_pos - end_pos)...) ≈ 20.0
    lengths = [system.segments[index].l0 for index in tether.segments]
    @test all(≈(20.0 / 6), lengths)
end

"""The rigid 2-plate kite with `edit` applied to its parsed authoring YAML, loaded."""
function load_edited(edit)
    data = YAML.load_file(joinpath(KITE, "rigid_structural_geometry.yaml"))
    edit(data)
    path = joinpath(mktempdir(), "edited.yaml")
    YAML.write_file(path, data)
    return load_authoring(path; set=kite_settings())
end

"""`data` with two bodies and the `timoshenko` tube `strut` between them."""
function add_timoshenko_tube!(data)
    data["bodies"] = Dict("headers" => ["name", "extra_mass", "pos", "inertia_principal"],
                          "data" => [["strut_a", 1.0, [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]],
                                     ["strut_b", 1.0, [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]]])
    data["tubes"] = Dict("headers" => ["name", "bodies", "diameter", "pressure", "law",
                                       "model", "EI", "GJ", "shear_coeff"],
                         "data" => [["strut", ["strut_a", "strut_b"], 0.1, 0.3,
                                     "breukels2011", "timoshenko", 120.0, 80.0, 0.85]])
    return data
end

"""`data` with a `tube` column, every point riding `strut`."""
function add_riders!(data)
    push!(data["points"]["headers"], "tube")
    foreach(row -> push!(row, "strut"), data["points"]["data"])
    return data
end

@testset "SAM's columns read into the models it registers and are written back" begin
    authored = YAML.load_file(joinpath(KITE, "rigid_structural_geometry.yaml"))["points"]
    system = load_edited(add_timoshenko_tube!)
    @test system.points[1] isa Point{SamPoint}
    @test only(system.tubes).model == Timoshenko(120.0, 80.0, 0.85)
    document = JSON.parse(to_json(system))
    @test isnothing(validate(SCHEMA, document))
    damping = column(authored, "body_frame_damping")
    @test column(document["points"], "body_frame_damping")[eachindex(damping)] == damping
    @test column(document["points"], "model")[eachindex(damping)] == fill("sam_point",
                                                                            length(damping))
    tubes = document["tubes"]
    @test [column(tubes, header) for header in ("model", "EI", "GJ", "shear_coeff")] ==
          [["timoshenko"], [120.0], [80.0], [0.85]]
end

@testset "a column neither the loader nor a registered model reads is refused" begin
    @test_throws "points columns body_frame_damping, world_frame_damping are read by " *
                 "neither the loader nor a registered model" without_sam_models(:points) do
        load_kite("rigid_structural_geometry.yaml")
    end
    @test_throws "tubes columns model, EI, GJ, shear_coeff" without_sam_models(:tubes) do
        load_edited(add_timoshenko_tube!)
    end
    @test_throws "points columns tube" load_edited(add_riders!)
    @test_throws "points columns colour" load_edited() do data
        push!(data["points"]["headers"], "colour")
        foreach(row -> push!(row, "red"), data["points"]["data"])
    end
end

@testset "a point rides a tube where its default model reads the tube column" begin
    system = without_sam_models(:points) do
        register_model!(:points, "riding_point", RidingPoint, ("N*s/m", "N*s/m", "-");
                        default=true)
        try
            return load_edited(data -> add_riders!(add_timoshenko_tube!(data)))
        finally
            unregister_model!(:points, "riding_point")
        end
    end
    @test system.points[1].tube == "strut"
end

@testset "a block takes one default model" begin
    @test_throws "points already reads the default model sam_point" register_model!(
        :points, "riding_point", RidingPoint, ("N*s/m", "N*s/m", "-"); default=true)
    @test !haskey(KiteGeometry.MODELS, (:points, "riding_point"))
end

@testset "a chained transform is refused whatever is registered" begin
    chained = @test_throws ArgumentError load_edited() do data
        only(data["transforms"]["data"])["base_transform_idx"] = "main_transform"
    end
    @test occursin("chained to another", chained.value.msg)
end

@testset "a block nothing reads is refused, an empty one is not" begin
    @test_throws "blocks groups, twist are read by nothing" load_edited() do data
        data["twist"] = Dict("headers" => ["gamma"], "data" => [[0.1]])
        data["groups"] = Dict("headers" => ["idx", "point_idxs"], "data" => [[1, [1, 2]]])
    end
    @test_throws "blocks metadata are read by nothing" load_edited() do data
        data["metadata"] = Dict("author" => "x")
    end
    system = load_edited() do data
        data["groups"] = Dict("headers" => ["idx", "point_idxs"], "data" => nothing)
        data["twist"] = nothing
    end
    @test !isempty(system.points)
end

@testset "an idx column repeats the row's position" begin
    @test_throws "row 2 is numbered `idx` 7" load_edited() do data
        pulleys = data["pulleys"]
        push!(pulleys["headers"], "idx")
        pulleys["data"] = [[row; idx] for (row, idx) in zip(pulleys["data"], (1, 7))]
    end
end

@testset "a wing has the canopy faces its authoring YAML states and no others" begin
    system = load_edited(data -> delete!(data, "canopy_faces"))
    @test isempty(system.canopy_faces)
    @test !isempty(system.stations)
end

@testset "V3Kite's PSM fixture states the canopy faces of awesIO's V3 PSM document" begin
    v3_psm = joinpath(@__DIR__, "data", "v3_psm")
    system = load_authoring(joinpath(v3_psm, "struc_geometry.yaml");
                            set=kite_settings(v3_psm))
    document = structure_document(system)
    golden = YAML.load_file(joinpath(@__DIR__, "data", "v3_psm_structure.yml"))
    faces, expected = document["canopy_faces"], golden["canopy_faces"]
    @test column(faces, "name") == column(expected, "name")
    @test column(faces, "points") == column(expected, "points")
    @test unique(column(faces, "wing")) == column(document["wings"], "name")
    @test unique(column(expected, "wing")) == column(golden["wings"], "name")
    @test isnothing(only(system.wings).canopy_material)
    @test isnothing(validate(SCHEMA, JSON.parse(to_json(system))))
end

foreach(model -> unregister_model!(model[1], model[2]), SAM_MODELS)
