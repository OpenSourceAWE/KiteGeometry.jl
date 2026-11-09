# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: expand_tether!, MaterialTable
using KiteUtils: Settings, set_data_path
using LinearAlgebra: norm
using OrderedCollections: OrderedDict

const TETHER_LINE = (; youngs_modulus=5.5e10, density=970.0, diameter=0.002)
const STRETCHED_LENGTH, TETHER_FORCE = 20.0, 100.0

"""The settings of the 2-plate kite's system."""
function placement_settings()
    set_data_path(joinpath(@__DIR__, "data", "2plate_kite"))
    return Settings("system.yaml")
end

"""The authoring document of a tether to a kite control unit, a point riding a tube between
two bodies by a segment, and a point carried by one of those bodies."""
function authored_rider_kite(; elevation=60.0, azimuth=30.0)
    return OrderedDict{String, Any}(
        "points" => OrderedDict{String, Any}(
            "headers" => ["name", "pos_cad", "type", "transform_idx", "tube", "body_idx"],
            "data" => [Any["ground", [0.0, 0.0, 0.0], "STATIC", "tf", nothing, nothing],
                       Any["kcu", [0.0, 0.0, 10.0], "DYNAMIC", "tf", nothing, nothing],
                       Any["rider", [0.0, 0.0, 11.0], "BODY_STATIC", "tf", "strut",
                           nothing],
                       Any["carried", [0.0, 1.0, 11.0], "BODY_STATIC", "tf", nothing,
                           "strut_b"]]),
        "segments" => OrderedDict{String, Any}(
            "headers" => ["name", "point_i", "point_j", "youngs_modulus", "density",
                          "diameter_mm"],
            "data" => [Any["bridle", "kcu", "rider", TETHER_LINE.youngs_modulus,
                           TETHER_LINE.density, 1000 * TETHER_LINE.diameter]]),
        "tethers" => OrderedDict{String, Any}(
            "headers" => ["name", "start_point", "end_point", "n_segments",
                          "youngs_modulus", "density", "diameter_mm",
                          "init_stretched_length",
                          "init_tether_force"],
            "data" => [Any["main", "ground", "kcu", 3, TETHER_LINE.youngs_modulus,
                           TETHER_LINE.density, 1000 * TETHER_LINE.diameter,
                           STRETCHED_LENGTH, TETHER_FORCE]]),
        "bodies" => OrderedDict{String, Any}(
            "headers" => ["name", "extra_mass", "pos", "inertia_principal",
                          "transform_idx"],
            "data" => [Any["strut_a", 1.0, [0.0, -1.0, 11.0], [1.0, 1.0, 1.0], "tf"],
                       Any["strut_b", 1.0, [0.0, 1.0, 11.0], [1.0, 1.0, 1.0], "tf"]]),
        "tubes" => OrderedDict{String, Any}(
            "headers" => ["name", "bodies", "diameter", "pressure", "law"],
            "data" => [Any["strut", ["strut_a", "strut_b"], 0.1, 3.0e4, "elastic"]]),
        "transforms" => OrderedDict{String, Any}(
            "headers" => ["name", "elevation", "azimuth", "heading", "base_pos",
                          "base_point_idx", "rot_point_idx"],
            "data" => [Any["tf", elevation, azimuth, 0.0, [0.0, 0.0, 0.0], "ground",
                           "kcu"]]))
end

"""The kite of [`authored_rider_kite`](@ref) built row by row, in its design pose, and its
transform."""
function rider_kite_in_julia(; elevation=60.0, azimuth=30.0)
    set = placement_settings()
    materials = MaterialTable(Material[], Dict{String, Any}())
    points = Point[Point(; name=:ground, fixed=true, pos_ENU=[0.0, 0.0, 0.0],
                         extra_mass=0.0),
                   Point(; name=:kcu, pos_ENU=[0.0, 0.0, 10.0], extra_mass=0.0),
                   Point(; name=:rider, tube=:strut, pos_ENU=[0.0, 0.0, 11.0],
                         extra_mass=0.0),
                   Point(; name=:carried, body=:strut_b, pos_ENU=[0.0, 1.0, 11.0],
                         extra_mass=0.0)]
    segments = Segment[KiteGeometry.line_segment(:bridle, (:kcu, :rider), set, materials;
                                    model=NoModel(), TETHER_LINE...)]
    init = TetherInit(; stretched_length=STRETCHED_LENGTH, force=TETHER_FORCE)
    point_transforms = Any[1, 1, 1, 1]
    tether = expand_tether!(points, segments, point_transforms, :main, :ground, :kcu, 3,
                            init, set, materials; model=NoModel(), segment_model=NoModel(),
                            TETHER_LINE...)
    bodies = [Body(; name=:strut_a, pos_ENU=[0.0, -1.0, 11.0], extra_mass=1.0),
              Body(; name=:strut_b, pos_ENU=[0.0, 1.0, 11.0], extra_mass=1.0)]
    tube = Tube(; name=:strut, bodies=(:strut_a, :strut_b), diameter=0.1, pressure=3.0e4)
    system = SystemDefinition(; metadata=Metadata(; name="rider", n_points=length(points)),
                              materials=materials.rows, points, segments, tethers=[tether],
                              bodies, tubes=[tube])
    transforms = [Transform(; name=:tf, elevation=deg2rad(elevation),
                            azimuth=deg2rad(azimuth), heading=0.0,
                            base_pos=[0.0, 0.0, 0.0], base_point=1, rot_point=[2 => 1.0],
                            body=1, points=collect(eachindex(points)), bodies=[1, 2])]
    return system, transforms, [init]
end

"""Whether `a` and `b` have the same pose and rest lengths, to `atol`."""
function placed_alike(a, b; atol=1e-9)
    (pose_a, pose_b) = Pose(a), Pose(b)
    return all(isapprox.(pose_a.pos, pose_b.pos; atol)) &&
           all(isapprox.(pose_a.body_pos, pose_b.body_pos; atol)) &&
           all(isapprox.(pose_a.body_R, pose_b.body_R; atol)) &&
           all(isapprox.([s.rest_length for s in a.segments],
                         [s.rest_length for s in b.segments]; atol))
end

@testset "a structure placed by each route lands on the same pose and rest lengths" begin
    from_yaml_route, _ = load_authoring(authored_rider_kite(); set=placement_settings(),
                                        name="rider")
    in_julia, julia_transforms, tether_inits = rider_kite_in_julia()
    place!(in_julia, julia_transforms; tether_inits)
    @test placed_alike(from_yaml_route, in_julia)

    replaced, replaced_transforms = load_authoring(
        authored_rider_kite(; elevation=30.0, azimuth=-40.0); set=placement_settings(),
        name="rider")
    only(replaced_transforms).elevation = deg2rad(60)
    only(replaced_transforms).azimuth = deg2rad(30)
    place!(replaced, replaced_transforms)
    @test placed_alike(from_yaml_route, replaced)
end

@testset "a tube rider moves with the tube's bodies, on the line between them" begin
    system, _ = load_authoring(authored_rider_kite(); set=placement_settings(),
                               name="rider")
    pose = Pose(system)
    rider, carried = system.points[:rider], system.points[:carried]
    @test rider.tube == 1 && isnothing(rider.body)
    @test pose.pos[3] ≈ (pose.body_pos[1] + pose.body_pos[2]) / 2 atol=1e-9
    @test norm(rider.pos_ENU - carried.pos_ENU) ≈ 1
end

@testset "tether inits put the tether at its length and rest length" begin
    system, _ = load_authoring(authored_rider_kite(); set=placement_settings(),
                               name="rider")
    tether = only(system.tethers)
    lengths = [norm(system.points[a].pos_ENU - system.points[b].pos_ENU)
               for (a, b) in (system.segments[index].points for index in tether.segments)]
    @test sum(lengths) ≈ STRETCHED_LENGTH atol=1e-9
    stiffness = TETHER_LINE.youngs_modulus * π * (TETHER_LINE.diameter / 2)^2
    @test sum(system.segments[segment].rest_length for segment in tether.segments) ≈
          STRETCHED_LENGTH * (1 - TETHER_FORCE / stiffness) atol=1e-9
end

@testset "placing again keeps the rest lengths of a definition holding no tether init" begin
    system, transforms = load_authoring(authored_rider_kite(); set=placement_settings(),
                                        name="rider")
    rest_lengths = [segment.rest_length for segment in system.segments]
    place!(system, transforms)
    @test all(isapprox.(rest_lengths, [segment.rest_length for segment in system.segments];
                        atol=1e-12))
end

"""The authoring document of a tether from a point on a body held in the world to a point
with no transform."""
function authored_held_anchor()
    return OrderedDict{String, Any}(
        "points" => OrderedDict{String, Any}(
            "headers" => ["name", "pos_cad", "type", "body_idx"],
            "data" => [Any["pad", [0.0, 0.0, 0.0], "BODY_STATIC", "base"],
                       Any["kcu", [0.0, 0.0, 10.0], "DYNAMIC", nothing]]),
        "tethers" => OrderedDict{String, Any}(
            "headers" => ["name", "start_point", "end_point", "n_segments",
                          "init_stretched_length"],
            "data" => [Any["main", "pad", "kcu", 3, 30.0]]),
        "bodies" => OrderedDict{String, Any}(
            "headers" => ["name", "extra_mass", "pos", "inertia_principal", "type"],
            "data" => [Any["base", 1.0, [0.0, 0.0, -1.0], [1.0, 1.0, 1.0], "STATIC"]]))
end

@testset "a tether anchored on a body held in the world leaves that body where it is" begin
    system, _ = load_authoring(authored_held_anchor(); set=placement_settings(),
                               name="held")
    @test system.bodies[:base].fixed === true
    @test system.bodies[:base].pos_ENU == [0.0, 0.0, -1.0]
    @test norm(system.points[:kcu].pos_ENU - system.points[:pad].pos_ENU) ≈ 30 atol=1e-9
end

@testset "tether_inits that are not one per tether are refused" begin
    system, transforms, _ = rider_kite_in_julia()
    @test_throws "0 tether_inits for 1 tethers" place!(system, transforms;
                                                       tether_inits=TetherInit[])
end
