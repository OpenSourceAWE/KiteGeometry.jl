# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using JSON
using JSONSchema: Schema, validate
using KiteUtils: Settings, set_data_path
using KiteGeometry: quaternion_to_rotation_matrix
using LinearAlgebra: cross, norm
using OrderedCollections: OrderedDict
using StaticArrays: SVector
using YAML

include("models.jl")

KITE = joinpath(@__DIR__, "data", "2plate_kite")
AUTHORING_SYSTEMS = filter(endswith("_structural_geometry.yaml"), readdir(KITE))
SCHEMA = Schema(YAML.load_file(joinpath(pkgdir(KiteGeometry), "src", "awesio",
                                         "structure_schema.yml")))

"""SAM's point columns, each optional, as a stand-in for the default point model SAM
registers."""
struct SamPoint <: AbstractModel
    type::Union{Nothing, String}
    body_frame_damping::Union{Nothing, Float64}
    world_frame_damping::Union{Nothing, Float64}
    area::Union{Nothing, Float64}
    drag_coeff::Union{Nothing, Float64}
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

"""SAM's wing columns, each optional, with the columns that frame and weigh a wing's
body."""
struct SamWing <: AbstractModel
    aero_mode::Union{Nothing, String}
    angular_damping::Union{Nothing, Float64, Vector{Float64}}
    aero_scale_chord::Union{Nothing, Float64}
    aero_z_offset::Union{Nothing, Float64}
    principal_frame_method::Union{Nothing, String}
    dynamics_type::Union{Nothing, String}
    origin_idx::Any
    z_ref_points::Any
    y_ref_points::Any
    com::Union{Nothing, Vector{Float64}}
    unit_inertia::Any
    type::Union{Nothing, String}
end
KiteGeometry.model_references(::Type{SamWing}) =
    (origin_idx=WeightedRefs(:points),
     z_ref_points=(WeightedRefs(:points), WeightedRefs(:points)),
     y_ref_points=(WeightedRefs(:points), WeightedRefs(:points)))

"""The wing SAM's columns describe: of `dynamics_type`, else the deprecated `type`, rigid
where both are unset."""
function KiteGeometry.wing_spec(wing::SamWing)
    dynamics = something(wing.dynamics_type, wing.type, "RIGID_DYNAMICS")
    dynamics in ("PARTICLE_DYNAMICS", "RIGID_DYNAMICS") ||
        throw(ArgumentError("no wing dynamics type $dynamics"))
    unit_inertia = isnothing(wing.unit_inertia) ? nothing :
                   sam_unit_inertia(wing.unit_inertia)
    return WingSpec(; particle=dynamics == "PARTICLE_DYNAMICS", origin=wing.origin_idx,
                    wing.z_ref_points, wing.y_ref_points, wing.com, unit_inertia)
end

"""The 3×3 per-unit-mass inertia [m²] SAM gives as `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`."""
function sam_unit_inertia((Ixx, Iyy, Izz, Ixy, Ixz, Iyz))
    return [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz]
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
              (:segments, "spring", SamSpring, SPRING_UNITS, true),
              (:wings, "sam_wing", SamWing,
               ("-", "1/s", "-", "m", "-", "-", "-", "-", "-", "m", "m^2", "-"), true),
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

"""The 2-plate kite of the authoring `file`, loaded and placed, and its transforms."""
load_kite_transforms(file) = load_authoring(joinpath(KITE, file); set=kite_settings())

"""The 2-plate kite of the authoring `file`, loaded and placed."""
load_kite(file) = first(load_kite_transforms(file))

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
                      ("bodies", "Q_KA_to_ENU") => "Q_KA2ENU",
                      ("points", "drag_area") => "area",
                      ("points", "drag_coefficient") => "drag_coeff")

"""The point columns SAM's point model reads, which the points a tether adds leave
unset."""
POINT_MODEL_COLUMNS = ("type", "drag_area", "drag_coefficient")

@testset "the rigid 2-plate kite is SAM's golden document, placed by its transform" begin
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
            for (name, value, cell) in zip(column(expected, "name"), values,
                                           column(expected, header))
                isnothing(value) && block == "points" && header in POINT_MODEL_COLUMNS &&
                    continue
                cells_agree(value, cell) || @error "$block $name $header" value cell
                @test cells_agree(value, cell)
            end
        end
    end
    @test sort(unread) == ["bodies.apparent_mass", "bodies.type",
                           "bodies.wing", "pulleys.efficiency",
                           "segments.density", "segments.unit_stiffness",
                           "stations.stiffness", "winches.model", "wings.aero",
                           "wings.canopy_material"]
end

@testset "each segment's material gives the golden document's density and stiffness" begin
    golden = YAML.load_file(joinpath(@__DIR__, "data", "2plate_kite_structure.yml"))
    system = load_kite("rigid_structural_geometry.yaml")
    @test [material.name for material in system.materials] == [:wing_element, :dyneema]
    @test all(material -> material.law == "linear", system.materials)
    for (segment, row) in zip(system.segments, golden["segments"]["data"])
        @test system.materials[segment.material].parameters["density"] == row[5]
        @test isapprox(KiteGeometry.unit_stiffness(segment, system.materials), row[6];
                       rtol=1e-12)
    end
end

@testset "a segment of a material that is not linear has no unit_stiffness" begin
    system = load_kite("rigid_structural_geometry.yaml")
    segment = system.segments[1]
    system.materials[segment.material].law = "breukels2011"
    @test isnothing(KiteGeometry.unit_stiffness(segment, system.materials))
end

@testset "a material's unit_stiffness at a diameter allocates nothing" begin
    material = Material(; name=:line, law="linear",
                        parameters=OrderedDict{String, Any}("youngs_modulus" => 1e9))
    KiteGeometry.unit_stiffness(material, 0.004)
    @test KiteGeometry.unit_stiffness(material, 0.004) ≈ 1e9 * π * 0.002^2
    @test (@allocated KiteGeometry.unit_stiffness(material, 0.004)) == 0
end

@testset "a tether of a material that is not linear has no init_tether_force" begin
    system = load_kite("particle_structural_geometry.yaml")
    (; segments, materials) = system
    tether = only(system.tethers)
    segment = segments[first(tether.segments)]
    @test KiteGeometry.tether_unit_stiffness(tether, segments, materials) ≈
          KiteGeometry.unit_stiffness(segment, materials)
    materials[segment.material].law = "breukels2011"
    @test_throws "init_tether_force needs a linear material" (
        KiteGeometry.tether_unit_stiffness(tether, segments, materials))
end

@testset "a segment's material is what its row gives, else the settings' tether" begin
    set = kite_settings()
    @test KiteGeometry.resolve_material("segment", set; diameter=0.002,
                                        youngs_modulus=1e9) == (0.002, 1e9, set.rho_tether)
    @test KiteGeometry.resolve_material("segment", set) ==
          (0.001 * set.d_tether, set.e_tether, set.rho_tether)
end

@testset "what a tool builds a definition with is public" begin
    @test all(name -> Base.ispublic(KiteGeometry, name),
              (:unit_stiffness, :resolve_material, :expand_tether!, :MaterialTable,
               :parse_table, :tube_law_parameters, :Component, :tether_ends))
end

"""Whether each point of `system` the authoring `points` table names is at its `pos_cad`
in the KA frame of the system's wing."""
function at_pos_cad(system, points)
    wing = only(system.wings)
    names, pos_cad = column(points, "name"), column(points, "pos_cad")
    return all(system.points) do point
        index = findfirst(==(point.name), names)
        isnothing(index) ||
            isapprox(pos_KA(wing, point.pos_ENU), pos_cad[index]; atol=1e-12)
    end
end

@testset "the placed 2-plate kite is its authoring YAML in its wing's KA frame" begin
    for file in AUTHORING_SYSTEMS
        authored = YAML.load_file(joinpath(KITE, file))["points"]
        system, transforms = load_kite_transforms(file)
        @test at_pos_cad(system, authored)
        @test rad2deg(only(transforms).elevation) ≈ 50
        @test !haskey(structure_document(system), "transforms")
        transform = only(transforms)
        transform.elevation, transform.azimuth = deg2rad(35), deg2rad(20)
        transform.heading = deg2rad(10)
        @test at_pos_cad(place!(system, transforms), authored)
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

@testset "a parsed authoring document loads as its file does" begin
    path = joinpath(KITE, "rigid_structural_geometry.yaml")
    authored = YAML.load_file(path; dicttype=OrderedDict{String, Any})
    system, transforms = load_authoring(authored; set=kite_settings(),
                                        name="rigid_structural_geometry")
    from_file, file_transforms = load_kite_transforms("rigid_structural_geometry.yaml")
    @test to_yaml(system) == to_yaml(from_file)
    members(transform) = (transform.name, transform.points, transform.bodies)
    @test members.(transforms) == members.(file_transforms)
    @test transforms isa Rows{Transform}
    @test transforms[:main_transform] === transforms[1]
end

@testset "a segment carries the spring columns its row authors in the tool's model" begin
    system = load_kite("rigid_structural_geometry.yaml")
    segment = system.segments[1]
    @test segment.model == SamSpring(nothing, 0.002, 1.0, nothing)
    tether_spring = SamSpring(nothing, 0.00077, nothing, nothing)
    @test all(name -> system.segments[name].model == tether_spring,
              only(system.tethers).segments)
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

"""The 2-plate kite of the authoring `file` with `edit` applied to its parsed YAML, loaded
against `set`, and its transforms."""
function load_edited_transforms(edit, file="rigid_structural_geometry.yaml";
                                set=kite_settings())
    data = YAML.load_file(joinpath(KITE, file))
    edit(data)
    path = joinpath(mktempdir(), "edited.yaml")
    YAML.write_file(path, data)
    return load_authoring(path; set)
end

"""The 2-plate kite of the authoring `file` with `edit` applied to its parsed YAML,
loaded."""
load_edited(edit, file="rigid_structural_geometry.yaml") =
    first(load_edited_transforms(edit, file))

"""`data` with its segments giving only a line's stiffness and density."""
function without_segment_spring!(data)
    data["variables"]["bare_line"] = Dict("youngs_modulus" => 1.0e9, "density" => 724.0)
    segments = data["segments"]
    filter!(!in(("damping_per_stiffness", "compression_frac")), segments["headers"])
    for row in segments["data"]
        row[6] = "bare_line"
        pop!(row)
    end
    return data
end

@testset "with no segment model registered, a segment carries none" begin
    without_sam_models(:segments) do
        @test_throws "segments columns damping_per_stiffness, compression_frac are read " *
                     "by neither the loader nor a registered model" load_kite(
            "rigid_structural_geometry.yaml")
        @test_throws "tethers columns damping_per_stiffness are read by neither the " *
                     "loader nor a registered model" load_edited(without_segment_spring!)
        edited = load_edited() do data
            delete!(delete!(without_segment_spring!(data), "tethers"), "winches")
        end
        @test all(segment -> segment.model isa NoModel, edited.segments)
    end
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

"""`data` with a `tube` column, every `BODY_STATIC` point riding `strut`, in place of its
body unless `keep_body`."""
function add_riders!(data; keep_body=false)
    headers = data["points"]["headers"]
    type_column, body_column = indexin(("type", "body_idx"), headers)
    for row in data["points"]["data"]
        append!(row, fill(nothing, length(headers) - length(row)))
        rides = row[type_column] == "BODY_STATIC"
        rides && !keep_body && (row[body_column] = nothing)
        push!(row, rides ? "strut" : nothing)
    end
    push!(headers, "tube")
    return data
end

@testset "materials of one law and different parameters get distinct names" begin
    materials = KiteGeometry.MaterialTable(Material[], Dict{String, Any}())
    names = [KiteGeometry.line_material!(materials, "line", density, "rope")
             for density in (970.0, 1100.0, 970.0)]
    @test names == [:rope, :rope_2, :rope]
    @test KiteGeometry.line_material!(materials, "rope", 970.0, 1.0e9) == :rope_3
end

@testset "a tether of a callable stiffness expands to segments reading that force law" begin
    set_data_path(joinpath(@__DIR__, "data", "2plate_kite"))
    force_law(strain) = 1.0e5 * strain
    tether_end(name, pos) = Point(; name, extra_mass=0.0, pos_ENU=pos)
    points = [tether_end(:ground, [0.0, 0.0, 0.0]), tether_end(:kite, [0.0, 0.0, 30.0])]
    segments = Segment[]
    point_transforms = Union{Nothing, NameRef}[nothing, nothing]
    materials = KiteGeometry.MaterialTable(Material[], Dict{String, Any}())
    tether = KiteGeometry.expand_tether!(points, segments, point_transforms, :main, :ground,
                                         :kite, 3, Settings("system.yaml"), materials;
                                         init=(; init_stretched_length=33.0),
                                         model=KiteGeometry.default_row_model(:tethers),
                                         segment_model=SamSpring(5.0, nothing, nothing,
                                                                 nothing),
                                         unit_stiffness=force_law)
    @test tether.segments == [:main_seg_1, :main_seg_2, :main_seg_3]
    @test tether.init_stretched_length == 33.0
    @test length(segments) == 3
    material = only(materials.rows)
    @test material.law == "unit_stiffness"
    @test material.parameters["unit_stiffness"] === force_law
    @test all(segment -> segment.material == material.name, segments)
    @test all(segment -> segment.model.unit_damping == 5.0, segments)
    @test all(segment -> segment.rest_length ≈ 11.0, segments)
    @test isnothing(KiteGeometry.unit_stiffness(material, 0.004))
    @test length(points) == length(point_transforms) == 4
    @test all(isnothing, point_transforms)
    @test points[3].pos_ENU ≈ [0.0, 0.0, 10.0]
end

@testset "a tube's material holds its own copy of its law's parameters" begin
    system = load_edited(add_timoshenko_tube!)
    parameters = system.materials[only(system.tubes).material].parameters
    @test parameters == KiteGeometry.tube_law_parameters(:breukels2011) ==
          KiteGeometry.BREUKELS2011
    @test parameters !== KiteGeometry.tube_law_parameters("breukels2011")
    @test isempty(KiteGeometry.tube_law_parameters("timoshenko"))
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
    @test isnothing(column(document["points"], "model"))
    @test from_yaml(to_yaml(system)).points[1] isa Point{SamPoint}
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
    @test_throws "points columns colour" load_edited() do data
        push!(data["points"]["headers"], "colour")
        foreach(row -> push!(row, "red"), data["points"]["data"])
    end
end

@testset "a point names the tube it rides, not a body as well" begin
    system = load_edited(data -> add_riders!(add_timoshenko_tube!(data)))
    @test system.points[1].tube == 1
    @test isnothing(system.points[1].body)
    @test_throws "rides a `body` or a `tube`, not both" load_edited() do data
        add_riders!(add_timoshenko_tube!(data); keep_body=true)
    end
end

@testset "a block takes one default model" begin
    @test_throws "points already reads the default model sam_point" register_model!(
        :points, "other_point", SamPoint, POINT_UNITS; default=true)
    @test !haskey(KiteGeometry.MODELS, (:points, "other_point"))
end

@testset "a keyword-built component carries the model a loaded row without one gets" begin
    point = Point(; name=:a, pos_ENU=[0, 0, 1], fixed=true)
    @test point.model == KiteGeometry.authoring_model(:points, (; name="a"))
    @test point isa Point{SamPoint}
    @test Point(; name=:a, pos_ENU=[0, 0, 1], model=NoModel()) isa Point{NoModel}
    station = SpanwiseStation(; name=:s, wing=:w, points=[:le, :te])
    @test station.model == NoModel()
    without_sam_models(:points) do
        @test Point(; name=:a, pos_ENU=[0, 0, 1]).model == NoModel()
    end
end

@testset "a transform with both a base_pos and a base_transform is refused" begin
    @test_throws "has a base_pos or a base_transform: give one" load_edited() do data
        only(data["transforms"]["data"])["base_transform_idx"] = "main_transform"
    end
end

@testset "an unread block is kept as an extra block, an empty one dropped" begin
    surfaces = Dict("headers" => ["name", "point_idx", "x_airf", "y_airf", "area"],
                    "data" => [["main", "kcu", [1, 0, 0], [0, 1, 0], 10.18]])
    system = load_edited() do data
        data["surfaces"] = surfaces
        data["groups"] = Dict("headers" => ["idx", "point_idxs"], "data" => nothing)
        data["twist"] = nothing
    end
    @test collect(keys(system.extras)) == ["surfaces"]
    @test system.extras["surfaces"] == surfaces
    @test structure_document(from_yaml(to_yaml(system)))["surfaces"] == surfaces
    @test_throws "block metadata is a block of the structure document" load_edited() do data
        data["metadata"] = Dict("author" => "x")
    end
    @test_throws "version: 1 is not a block" load_edited(data -> data["version"] = 1)
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
    system, _ = load_authoring(joinpath(v3_psm, "struc_geometry.yaml");
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

"""The authoring YAML of each system SymbolicAWEModels' placed pose is recorded for."""
SAM_PLACED_SYSTEMS = Dict("rigid_structural_geometry" => KITE,
                          "particle_structural_geometry" => KITE,
                          "weighted_ref" => joinpath(@__DIR__, "data"),
                          "chained_transforms" => joinpath(@__DIR__, "data"))

"""The system `name` of `SAM_PLACED_SYSTEMS`, and its transforms, loaded and placed."""
function load_sam_placed(name)
    load() = load_authoring(joinpath(SAM_PLACED_SYSTEMS[name], "$name.yaml");
                            set=kite_settings())
    name == "chained_transforms" || return load()
    return without_sam_models(load, :points, :bodies)
end

@testset "authoring YAMLs with transforms load to the pose SymbolicAWEModels places" begin
    reference = YAML.load_file(joinpath(@__DIR__, "data", "sam_placed.yml"))
    @test sort(collect(keys(reference))) == sort(collect(keys(SAM_PLACED_SYSTEMS)))
    for (name, placed) in reference
        system, transforms = load_sam_placed(name)
        point_vel, body_vel, body_spin = transform_velocities(Pose(system), transforms)
        @test sort([String(point.name) for point in system.points]) ==
              sort(collect(keys(placed["points"])))
        for (point, vel) in zip(system.points, point_vel)
            expected = placed["points"][String(point.name)]
            @test isapprox(point.pos_ENU, expected["pos"]; rtol=0, atol=1e-12)
            @test isapprox(vel, expected["vel"]; rtol=0, atol=1e-12)
        end
        for segment in system.segments
            expected = placed["segments"][String(segment.name)]
            @test segment.rest_length ≈ expected atol=1e-12
        end
        for (index, body) in enumerate(system.bodies)
            expected = placed["bodies"][String(body.name)]
            @test isapprox(quaternion_to_rotation_matrix(body.Q_KA2ENU),
                           quaternion_to_rotation_matrix(expected["Q"]); rtol=0, atol=1e-12)
            origin_vel = body_vel[index] +
                         cross(body_spin[index], expected["pos"] - body.pos_ENU)
            @test isapprox(origin_vel, expected["vel"]; rtol=0, atol=1e-12)
        end
    end
end

SAM_CASES = joinpath(@__DIR__, "data", "sam_cases")

"""The authoring YAML `file` of SAM's tests, loaded."""
load_case(file) = first(load_authoring(joinpath(SAM_CASES, file); set=kite_settings()))

"""The particle 2-plate kite with the `wing_idx` cell of the point `name` set to `wing`,
loaded."""
function load_with_point_wing(name, wing)
    return load_edited("particle_structural_geometry.yaml") do data
        points = data["points"]
        row = points["data"][findfirst(row -> row[1] == name, points["data"])]
        row[findfirst(==("wing_idx"), points["headers"])] = wing
    end
end

@testset "0 in a reference column names nothing" begin
    for file in ("tether_wing.yaml", "wing_freefall.yaml", "wing_hanging.yaml")
        @test only(load_case(file).wings).name == :main_wing
    end
    @test structure_document(load_with_point_wing("kcu", 0)) ==
          structure_document(load_with_point_wing("kcu", nothing))
end

@testset "a bridle point keeps the wing it names, written and read back" begin
    system = load_with_point_wing("kcu", "main_wing")
    kcu = findfirst(point -> point.name == :kcu, system.points)
    @test system.wings[system.points[kcu].wing].name == :main_wing
    @test from_yaml(to_yaml(system)).points[kcu].wing == system.points[kcu].wing
    @test isnothing(load_with_point_wing("kcu", nothing).points[kcu].wing)
end

@testset "a refusal is an error naming the row" begin
    for (file, message) in (
        "cyclic.yaml" => "Cyclic variable reference: a -> b -> a",
        "cyclic_reversed.yaml" => "Cyclic variable reference: b -> a -> b",
        "shadow.yaml" => "Variable `anchor` has the same name as a component in block " *
                         "`points`",
        "mismatch.yaml" => "Variable `dyneema` defines damping_per_stiffness, " *
                           "youngs_modulus, but the columns at that position are " *
                           "youngs_modulus, compression_frac",
        "materials.yaml" => "the `materials` block was removed",
        "conflict.yaml" => "segment line: give either `unit_stiffness` or " *
                           "`youngs_modulus`, not both",
        "tether_loop.yaml" => "tether main_tether: the structure downstream of it " *
                              "connects back to its anchor",
        "tether_stacked.yaml" => "tether upper has neither endpoint on a fixed or winch " *
                                 "point")
        @test_throws ErrorException load_case(file)
        @test_throws message load_case(file)
    end
    @test_throws "point le_left is a point of station left of wing main_wing, but names " *
                 "no wing" load_with_point_wing("le_left", 0)
    @test_throws ErrorException("point kcu: no wings 7") load_with_point_wing("kcu", 7)
end

@testset "a name two rows of an authored block share is refused, naming that block" begin
    for (block, name) in (("tethers", "main_tether"), ("segments", "le_tube_left"))
        @test_throws "two $block are named $name" load_edited() do data
            rows = data[block]["data"]
            push!(rows, copy(rows[findfirst(row -> row[1] == name, rows)]))
        end
    end
end

@testset "a wing keeps the columns that frame its body in its model, as point indices" begin
    system = load_kite("particle_structural_geometry.yaml")
    wing = only(system.wings)
    point_name(index) = system.points[index].name
    @test wing.dynamics_type == "PARTICLE_DYNAMICS"
    @test point_name(wing.origin_idx) == :kcu
    @test point_name.(wing.z_ref_points) == [:kcu, :le_center]
    @test point_name.(wing.y_ref_points) == [:le_right, :le_left]
    no_origin = ArgumentError("a particle wing needs an origin")
    @test_throws no_origin load_edited("particle_structural_geometry.yaml") do data
        only(data["wings"]["data"])["origin_idx"] = nothing
    end
end

"""The type of the model of each component of each block of `system`."""
model_types(system) = [typeof.(getfield.(getfield(system, block), :model))
                       for block in keys(KiteGeometry.BLOCKS)]

@testset "a row the loader adds carries its block's default model" begin
    for system in (load_kite("particle_structural_geometry.yaml"),
                   load_case("tether_wing.yaml"))
        @test model_types(from_yaml(to_yaml(system))) == model_types(system)
        @test all(point -> point.model isa SamPoint, system.points)
        @test all(body -> body.model isa SamBody, system.bodies)
    end
end

"""SAM's wing columns of the 2-plate kite's wing, of `dynamics_type`, with the centre and
unit inertia of its mesh."""
function kite_wing_model(dynamics_type; com=nothing, unit_inertia=nothing)
    return SamWing(nothing, nothing, nothing, nothing, nothing, dynamics_type, "kcu",
                   ["kcu", "le_center"], ["le_right", "le_left"], com, unit_inertia,
                   nothing)
end

"""The 2-plate kite of `<dynamics>_structural_geometry.yaml` built row by row with keyword
constructors, every point carrying `point_mass` where it is given and its wing body
`extra_mass`, with its wings completed for `set`; `mesh` is the centre and unit inertia of
its wing's mesh."""
function keyword_kite(dynamics, set; point_mass=nothing, extra_mass=0.0, mesh...)
    rigid = dynamics == "RIGID_DYNAMICS"
    wing_body = rigid ? :main_wing : nothing
    points = Point[Point(; name, pos_ENU, body, extra_mass=something(point_mass, mass),
                         fixed=name == :ground)
                   for (name, pos_ENU, body, mass) in (
                       (:le_left, [-0.5, 1.0, 2.0], wing_body, 0.1),
                       (:te_left, [0.5, 1.0, 2.3], wing_body, 0.1),
                       (:le_center, [-0.5, 0.0, 2.5], wing_body, 0.1),
                       (:te_center, [0.5, 0.0, 2.8], wing_body, 0.1),
                       (:le_right, [-0.5, -1.0, 2.0], wing_body, 0.1),
                       (:te_right, [0.5, -1.0, 2.3], wing_body, 0.1),
                       (:steering_left, [0.0, 0.5, 0.5], nothing, 0.1),
                       (:steering_right, [0.0, -0.5, 0.5], nothing, 0.1),
                       (:kcu, [0.0, 0.0, 0.0], wing_body, 1.0),
                       (:ground, [0.0, 0.0, -20.0], nothing, 0.0))]
    segments = Segment[Segment(; name, points) for (name, points) in (
        :le_tube_left => (:te_left, :te_center), :le_tube_right => (:te_center, :te_right),
        :strut_left => (:le_left, :te_left), :strut_center => (:le_center, :te_center),
        :strut_right => (:le_right, :te_right), :te_wire_left => (:le_center, :le_right),
        :te_wire_right => (:le_center, :le_left), :diag_1 => (:te_left, :le_center),
        :diag_2 => (:le_left, :te_center), :diag_3 => (:te_right, :le_center),
        :diag_4 => (:le_right, :te_center), :le_left => (:le_left, :kcu),
        :le_center => (:le_center, :kcu), :le_right => (:le_right, :kcu),
        :te_center => (:te_center, :kcu), :le_steering_left => (:le_left, :steering_left),
        :te_steering_left => (:te_left, :steering_left),
        :kcu_steering_left => (:steering_left, :kcu),
        :le_steering_right => (:le_right, :steering_right),
        :te_steering_right => (:te_right, :steering_right),
        :kcu_steering_right => (:steering_right, :kcu))]
    materials = KiteGeometry.MaterialTable(Material[], Dict{String, Any}())
    tether = KiteGeometry.expand_tether!(
        points, segments, fill(nothing, length(points)), :main_tether, :kcu, :ground, 6,
        set, materials; init=(; init_stretched_length=20.0), model=NoModel(),
        segment_model=NoModel())
    stations = [SpanwiseStation(; name, wing=:main_wing, points=[Symbol("le_$name"),
                                                                 Symbol("te_$name")])
                for name in (:left, :center, :right)]
    faces = [CanopyFace(; name=:canopy_1, wing=:main_wing,
                        points=[:le_left, :te_left, :te_center, :le_center]),
             CanopyFace(; name=:canopy_2, wing=:main_wing,
                        points=[:le_center, :te_center, :te_right, :le_right])]
    system = SystemDefinition(;
        metadata=Metadata(; name="2plate_kite", n_points=length(points)),
        materials=materials.rows, points, segments,
        spanwise_stations=stations, tethers=[tether],
        winches=[Winch(; name=:main_winch, tethers=[:main_tether], winch_point=:ground)],
        wings=[Wing(; name=:main_wing, pos_ENU=zeros(3), body=:main_wing,
                    model=kite_wing_model(dynamics; mesh...))],
        canopy_faces=faces,
        bodies=[Body(; name=:main_wing, pos_ENU=zeros(3), extra_mass,
                     extra_inertia_KA=zeros(3, 3))])
    return complete_wings!(system, set)
end

"""`data` with every point carrying no mass of its own."""
function massless_points!(data)
    column = findfirst(==("extra_mass"), data["points"]["headers"])
    foreach(row -> row[column] = 0.0, data["points"]["data"])
end

@testset "a kite built in Julia completes to the wing load_authoring gives" begin
    unit_inertia = [0.5, 0.2, 0.6, 0.0, 0.01, 0.0]
    with_mesh(data) = merge!(only(data["wings"]["data"]),
                             Dict("com" => [0.1, 0.0, 2.4], "unit_inertia" => unit_inertia,
                                  "extra_mass" => 2.0))
    cases = Any[(dynamics, identity, 0.0, (;)) for dynamics in ("RIGID", "PARTICLE")]
    append!(cases, [(dynamics, massless_points!, 1.5, (; point_mass=0.0))
                    for dynamics in ("RIGID", "PARTICLE")])
    push!(cases, ("RIGID", with_mesh, 0.0,
                  (; extra_mass=2.0, com=[0.1, 0.0, 2.4], unit_inertia)))
    close(a, b) = isapprox(a, b; rtol=0, atol=1e-12)
    for (dynamics, edit, mass, built) in cases
        set = kite_settings()
        set.mass = mass
        file = "$(lowercase(dynamics))_structural_geometry.yaml"
        loaded, transforms = load_edited_transforms(edit, file; set)
        system = place!(keyword_kite("$(dynamics)_DYNAMICS", set; built...), transforms)
        @test [point.extra_mass for point in system.points] ==
              [point.extra_mass for point in loaded.points]
        @test all(close.(getfield.(system.points, :pos_ENU),
                         getfield.(loaded.points, :pos_ENU)))
        body, loaded_body = only(system.bodies), only(loaded.bodies)
        @test body.extra_mass == loaded_body.extra_mass
        @test close(body.pos_ENU, loaded_body.pos_ENU)
        @test close(body.Q_KA2ENU, loaded_body.Q_KA2ENU)
        @test close(body.extra_inertia_KA, loaded_body.extra_inertia_KA)
        wing, loaded_wing = only(system.wings), only(loaded.wings)
        @test close(wing.pos_ENU, loaded_wing.pos_ENU)
        @test close(wing.Q_KA2ENU, loaded_wing.Q_KA2ENU)
    end
end

@testset "a wing's centre and unit inertia each count where its model gives only one" begin
    unit_inertia = [0.5, 0.2, 0.6, 0.0, 0.01, 0.0]
    set = kite_settings()
    set.mass = 0.0
    body = only(keyword_kite("RIGID_DYNAMICS", set; point_mass=0.0, extra_mass=2.0,
                             unit_inertia).bodies)
    R = KiteGeometry.quaternion_to_rotation_matrix(body.Q_KA2ENU)
    expected = R' * (2.0 * sam_unit_inertia(unit_inertia)) * R
    @test isapprox(body.extra_inertia_KA, expected; rtol=0, atol=1e-12)
    com = [0.1, 0.0, 2.4]
    body = only(keyword_kite("RIGID_DYNAMICS", set; extra_mass=2.0, com).bodies)
    @test isapprox(body.pos_ENU, com; rtol=0, atol=1e-12)
end

@testset "a wing spec takes point indices and two reference points per axis" begin
    spec = WingSpec(; origin=9, z_ref_points=[9, [Any[1, 0.5], Any[5, 0.5]]],
                    y_ref_points=[5, [1 => 1.0]])
    @test spec.frame.origin == [9 => 1.0]
    @test spec.frame.z_ref_points == ([9 => 1.0], [1 => 0.5, 5 => 0.5])
    @test spec.frame.y_ref_points == ([5 => 1.0], [1 => 1.0])
    @test_throws ArgumentError("reference points kcu are not point indices") WingSpec(;
        origin="kcu")
    @test_throws ArgumentError WingSpec(; origin=9, z_ref_points=[9])
    @test isnothing(WingSpec(; origin=9).frame)
end

@testset "reference points weigh one index fully, a list equally, pairs by weight" begin
    origin(value) = WingSpec(; origin=value).origin
    @test origin(7) == [7 => 1.0]
    @test origin([2, 4]) == [2 => 0.5, 4 => 0.5]
    @test (@test_logs origin([Any[2, 0.25], Any[4, 0.75]])) == [2 => 0.25, 4 => 0.75]
    @test (@test_logs origin([Any[2, 0.4], Any[4, 0.6]])) == [2 => 0.4, 4 => 0.6]
    @test (@test_logs (:warn, r"sum to 4.0") origin([Any[2, 2.0], Any[4, 2.0]])) ==
          [2 => 0.5, 4 => 0.5]
    @test (@test_logs (:warn, r"sum to 4.0") origin([Any[2, 3.0], Any[4, 1.0]])) ==
          [2 => 0.75, 4 => 0.25]
    @test_throws "sum to 0.0" origin([Any[2, 0.0], Any[4, 0.0]])
    @test_throws ArgumentError("reference points Any[] name no point") origin([])
end

@testset "a wing whose model gives no wing spec is left as it is" begin
    points = [Point(; name, pos_ENU, body=:wing)
              for (name, pos_ENU) in ((:tip, [1.0, 0.0, 5.0]), (:root, [-1.0, 0.0, 5.0]))]
    system = SystemDefinition(; metadata=Metadata(; name="bare", n_points=2), points,
                              wings=[Wing(; name=:wing, pos_ENU=zeros(3), body=:wing,
                                          model=NoModel())],
                              bodies=[Body(; name=:wing, pos_ENU=zeros(3))])
    masses = [point.extra_mass for point in system.points]
    set = kite_settings()
    set.mass = 1.0
    complete_wings!(system, set)
    @test [point.extra_mass for point in system.points] == masses
    @test system.bodies[:wing].pos_ENU == zeros(3)
end

@testset "a wing with no frame points keeps its pose and weighs its unit inertia" begin
    R = KiteGeometry.quaternion_to_rotation_matrix([cos(0.3), 0.0, 0.0, sin(0.3)])
    Q_KA2ENU = KiteGeometry.rotation_matrix_to_quaternion(R)
    model = SamWing(nothing, nothing, nothing, nothing, nothing, "RIGID_DYNAMICS", nothing,
                    nothing, nothing, nothing, [0.5, 0.2, 0.5, 0.1, 0.0, 0.0], nothing)
    system = SystemDefinition(; metadata=Metadata(; name="hanging", n_points=1),
                              points=[Point(; name=:hook, pos_ENU=[0.0, 0.0, 6.0])],
                              wings=[Wing(; name=:wing, pos_ENU=zeros(3), body=:wing,
                                          model)],
                              bodies=[Body(; name=:wing, pos_ENU=[1.0, 2.0, 3.0], Q_KA2ENU,
                                           extra_mass=2.0)])
    complete_wings!(system, kite_settings())
    body = system.bodies[:wing]
    @test body.pos_ENU == [1.0, 2.0, 3.0]
    @test body.Q_KA2ENU == Q_KA2ENU
    unit_inertia = [0.5 0.1 0.0; 0.1 0.2 0.0; 0.0 0.0 0.5]
    @test isapprox(body.extra_inertia_KA, R' * (2.0 * unit_inertia) * R; rtol=0, atol=1e-12)
end

"""A station model naming the body of its flap."""
struct FlapStation <: AbstractModel
    flap_body::Any
end
KiteGeometry.model_references(::Type{FlapStation}) = (flap_body=:bodies,)

@testset "a particle wing weighs the bodies tubes join to a body its stations name" begin
    model = SamWing(nothing, nothing, nothing, nothing, nothing, "PARTICLE_DYNAMICS", "le",
                    nothing, nothing, nothing, nothing, nothing)
    points = [Point(; name, pos_ENU) for (name, pos_ENU) in
              ((:le, [0.0, 0.0, 5.0]), (:te, [1.0, 0.0, 5.0]), (:hook, [0.0, 0.0, 0.0]))]
    system = SystemDefinition(;
        metadata=Metadata(; name="flapped", n_points=3), points,
        spanwise_stations=[SpanwiseStation(; name=:mid, wing=:wing, points=[:le, :te],
                                           model=FlapStation(:flap))],
        wings=[Wing(; name=:wing, pos_ENU=zeros(3), body=:wing, model)],
        bodies=[Body(; name, pos_ENU=zeros(3), extra_mass)
                for (name, extra_mass) in ((:wing, 0.0), (:flap, 0.0), (:servo, 0.5))],
        tubes=[Tube(; name=:arm, bodies=(:flap, :servo))])
    @test KiteGeometry.particle_wing_parts(system, 1) == (Set([1, 2]), [2, 3])
    references = (:bodies, WeightedRefs(:bodies), :points)
    @test KiteGeometry.referenced_rows([3, [Any[2, 0.5], Any[4, 0.5]], 1], references,
                                       :bodies) == [3, 2, 4]
    @test isempty(KiteGeometry.referenced_rows(nothing, :bodies, :bodies))
    set = kite_settings()
    set.mass = 1.0
    complete_wings!(system, set)
    @test all(point -> isnothing(point.extra_mass) || iszero(point.extra_mass),
              system.points)
end

foreach(model -> unregister_model!(model[1], model[2]), SAM_MODELS)
