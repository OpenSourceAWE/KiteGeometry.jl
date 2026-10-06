# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using JSON
using JSONSchema: Schema, validate
using KiteUtils: Settings, set_data_path
using OrderedCollections: OrderedDict
using YAML

KITE = joinpath(@__DIR__, "data", "2plate_kite")
AUTHORING_SYSTEMS = filter(endswith("_structural_geometry.yaml"), readdir(KITE))
SCHEMA = Schema(YAML.load_file(joinpath(pkgdir(KiteGeometry), "src", "awesio",
                                         "structure_schema.yml")))

"""The settings the 2-plate kite is authored against."""
function kite_settings()
    set_data_path(KITE)
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

@testset "the rigid 2-plate kite loads as SAM's golden document describes it" begin
    golden = YAML.load_file(joinpath(@__DIR__, "data", "2plate_kite_structure.yml");
                            dicttype=OrderedDict{String, Any})
    document = structure_document(load_kite("rigid_structural_geometry.yaml"))
    @test document["metadata"]["n_points"] == golden["metadata"]["n_points"]
    @test document["metadata"]["connectivity_sha"] == golden["metadata"]["connectivity_sha"]
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
                           "stations.damping", "stations.moment_frac",
                           "stations.stiffness", "winches.model", "wings.aero"]
end

@testset "every authoring system writes a document the vendored schema accepts" begin
    @test length(AUTHORING_SYSTEMS) == 2
    for file in AUTHORING_SYSTEMS
        system = load_kite(file)
        document = JSON.parse(to_json(system))
        @test isnothing(validate(SCHEMA, document))
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

@testset "what a SystemDefinition cannot hold is refused" begin
    tube_rider = @test_throws ArgumentError load_edited() do data
        push!(data["points"]["headers"], "tube")
        foreach(row -> push!(row, "strut"), data["points"]["data"])
    end
    @test occursin("rides a tube", tube_rider.value.msg)
    chained = @test_throws ArgumentError load_edited() do data
        only(data["transforms"]["data"])["base_transform_idx"] = "main_transform"
    end
    @test occursin("chained to another", chained.value.msg)
end
