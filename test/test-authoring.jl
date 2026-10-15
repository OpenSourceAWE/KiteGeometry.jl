# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using JSON
using JSONSchema: Schema, validate
using KiteUtils: Settings, set_data_path
using LinearAlgebra: norm
using OrderedCollections: OrderedDict
using StaticArrays: SVector
using YAML

include("models.jl")

KITE = joinpath(@__DIR__, "data", "2plate_kite")
AUTHORING_SYSTEMS = filter(endswith("_structural_geometry.yaml"), readdir(KITE))
SCHEMA = Schema(YAML.load_file(joinpath(pkgdir(KiteGeometry), "src", "awesio",
                                         "structure_schema.yml")))

"""SAM's point columns, as a stand-in for the default point model SAM registers."""
struct SamPoint <: AbstractModel
    type::String
    body_frame_damping::Float64
    world_frame_damping::Float64
    area::Float64
    drag_coeff::Float64
end

"""`SamPoint` with the tube a point rides."""
struct RidingPoint <: AbstractModel
    type::String
    body_frame_damping::Float64
    world_frame_damping::Float64
    area::Float64
    drag_coeff::Float64
    tube::Union{Nothing, String}
end

"""SAM's station columns."""
struct SamStation <: AbstractModel
    type::String
    moment_frac::Float64
    damping::Float64
end

"""SAM's body columns."""
struct SamBody <: AbstractModel
    type::Union{Nothing, String}
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

POINT_UNITS = ("-", "N*s/m", "N*s/m", "m^2", "-")
SAM_MODELS = [(:points, "sam_point", SamPoint, POINT_UNITS, true),
              (:spanwise_stations, "sam_station", SamStation, ("-", "-", "N*m*s"), true),
              (:pulleys, "sam_pulley", SamPulley, ("-", "-"), true),
              (:wings, "sam_wing", SamWing, ("m", "-"), true),
              (:bodies, "sam_body", SamBody, ("-",), true),
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

"""The positions of the points of a `document`'s `points` table, then of its bodies."""
function positions(document)
    return SVector{3}.([column(document["points"], "pos_ENU");
                        column(document["bodies"], "pos_ENU")])
end

"""The distance between every two of `positions` [m]."""
pairwise_distances(positions) = [norm(a - b) for a in positions, b in positions]

"""The row number of each point name of `refs` among `names`."""
row_numbers(refs, names) = [findfirst(==(ref), names) for ref in refs]

"""The `connectivity_sha` of SAM's golden `document`, with the canopy `faces`."""
function golden_connectivity_sha(document, faces)
    names = column(document["points"], "name")
    segments = row_numbers.(column(document["segments"], "points"), (names,))
    return connectivity_sha((length(names), segments),
                            (length(column(document["bodies"], "name")), []),
                            (length(faces), row_numbers.(faces, (names,))))
end

"""The block each block of SAM's golden document became, where it was renamed."""
GOLDEN_BLOCKS = Dict("stations" => "spanwise_stations")

"""The column each column of SAM's golden document became, where it was renamed."""
GOLDEN_COLUMNS = Dict(("segments", "l0") => "rest_length",
                      ("points", "drag_area") => "area",
                      ("points", "drag_coefficient") => "drag_coeff")

"""The point columns SAM's point model reads, which the points a tether adds leave
unset."""
POINT_MODEL_COLUMNS = ("type", "drag_area", "drag_coefficient")

@testset "the rigid 2-plate kite has the shape of SAM's golden document, at zenith" begin
    golden = YAML.load_file(joinpath(@__DIR__, "data", "2plate_kite_structure.yml");
                            dicttype=OrderedDict{String, Any})
    system = load_kite("rigid_structural_geometry.yaml")
    document = structure_document(system)
    @test document["metadata"]["n_points"] == golden["metadata"]["n_points"]
    faces = column(document["canopy_faces"], "points")
    @test faces == [["le_left", "te_left", "te_center", "le_center"],
                    ["le_center", "te_center", "te_right", "le_right"]]
    @test document["metadata"]["connectivity_sha"] == golden_connectivity_sha(golden, faces)
    @test isapprox(pairwise_distances(positions(document)),
                   pairwise_distances(positions(golden)); rtol=0, atol=1e-12)
    unread = String[]
    for (block, expected) in golden
        block == "metadata" && continue
        isempty(expected["data"]) && continue
        for header in expected["headers"]
            table = document[get(GOLDEN_BLOCKS, block, block)]
            values = column(table, get(GOLDEN_COLUMNS, (block, header), header))
            isnothing(values) && (push!(unread, "$block.$header"); continue)
            header in ("pos_ENU", "Q_KA_to_ENU") && continue
            for (name, value, cell) in zip(column(expected, "name"), values,
                                           column(expected, header))
                isnothing(value) && block == "points" && header in POINT_MODEL_COLUMNS &&
                    continue
                cells_agree(value, cell) || @error "$block $name $header" value cell
                @test cells_agree(value, cell)
            end
        end
    end
    @test sort(unread) == ["bodies.Q_KA_to_ENU", "bodies.apparent_mass", "bodies.type",
                           "bodies.wing", "points.wing", "pulleys.efficiency",
                           "segments.density", "segments.unit_stiffness",
                           "stations.stiffness", "winches.model", "wings.aero",
                           "wings.canopy_material"]
end

@testset "each segment's material gives the golden document's density and stiffness" begin
    golden = YAML.load_file(joinpath(@__DIR__, "data", "2plate_kite_structure.yml"))
    system = load_kite("rigid_structural_geometry.yaml")
    @test [material.name for material in system.materials] == ["wing_element", "dyneema"]
    @test all(material -> material.law == "linear", system.materials)
    for (segment, row) in zip(system.segments, golden["segments"]["data"])
        parameters = system.materials[segment.material].parameters
        @test parameters["density"] == row[5]
        @test isapprox(parameters["youngs_modulus"] * π * (segment.diameter / 2)^2, row[6];
                       rtol=1e-12)
    end
end

@testset "the 2-plate kite at zenith is its authoring YAML in its wing's KA frame" begin
    for file in AUTHORING_SYSTEMS
        authored = YAML.load_file(joinpath(KITE, file))["points"]
        system = load_kite(file)
        wing = only(system.wings)
        @test wing.Q_KA2ENU == [1, 0, 0, 0]
        @test wing.pos_ENU == [0, 0, 20]
        names, pos_cad = column(authored, "name"), column(authored, "pos_cad")
        for point in system.points
            index = findfirst(==(point.name), names)
            isnothing(index) && continue
            @test isapprox(pos_KA(wing, point.pos_ENU), pos_cad[index]; rtol=0, atol=1e-12)
        end
        transforms = structure_document(system)["transforms"]
        @test only(transforms["data"])["elevation"] == 50
    end
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
    @test segment.unit_damping ≈ 10.0
    @test model_type(:segments, "spring") === SegmentSpring
end

@testset "the tether is placed at its stretched length, its points evenly along it" begin
    system = load_kite("particle_structural_geometry.yaml")
    tether = only(system.tethers)
    start_pos = system.points[tether.start_point].pos_ENU
    end_pos = system.points[tether.end_point].pos_ENU
    @test hypot((start_pos - end_pos)...) ≈ 20.0
    lengths = [system.segments[index].rest_length for index in tether.segments]
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

@testset "materials of one law and different parameters get distinct names" begin
    materials = KiteGeometry.MaterialTable(Material[], Dict{String, Any}())
    names = [KiteGeometry.line_material!(materials, "line", density, "rope")
             for density in (970.0, 1100.0, 970.0)]
    @test names == ["rope", "rope_2", "rope"]
    @test KiteGeometry.line_material!(materials, "rope", 970.0, 1.0e9) == "rope_3"
end

@testset "a breukels2011 tube's material is its own copy of SAM's constants" begin
    system = load_edited(add_timoshenko_tube!)
    parameters = system.materials[only(system.tubes).material].parameters
    @test parameters == KiteGeometry.BREUKELS2011
    @test parameters !== KiteGeometry.BREUKELS2011
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
    @test_throws "points columns body_frame_damping, world_frame_damping, area, " *
                 "drag_coeff are read by neither the loader nor a registered " *
                 "model" without_sam_models(:points) do
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
        register_model!(:points, "riding_point", RidingPoint, (POINT_UNITS..., "-");
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
        :points, "riding_point", RidingPoint, (POINT_UNITS..., "-"); default=true)
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
    @test !isempty(system.spanwise_stations)
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
