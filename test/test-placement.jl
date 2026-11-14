# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: expand_tether!, MaterialTable
using KiteUtils: Settings, set_data_path
using LinearAlgebra: norm, normalize
using OrderedCollections: OrderedDict

"""The line of the rider kite's tether and bridle."""
tether_line() = (; youngs_modulus=5.5e10, density=970.0, diameter=0.002)

"""The `init_*` columns of the rider kite's tether."""
tether_init() = (; init_stretched_length=20.0, init_tether_force=100.0)

"""The settings of the 2-plate kite's system."""
function placement_settings()
    set_data_path(joinpath(@__DIR__, "data", "2plate_kite"))
    return Settings("system.yaml")
end

"""The authoring document of a tether to a kite control unit, a point riding a tube between
two bodies by a segment, and a point carried by one of those bodies."""
function authored_rider_kite(; elevation=60.0, azimuth=30.0)
    line, init = tether_line(), tether_init()
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
            "data" => [Any["bridle", "kcu", "rider", line.youngs_modulus, line.density,
                           1000 * line.diameter]]),
        "tethers" => OrderedDict{String, Any}(
            "headers" => ["name", "start_point", "end_point", "n_segments",
                          "youngs_modulus", "density", "diameter_mm",
                          "init_stretched_length", "init_tether_force"],
            "data" => [Any["main", "ground", "kcu", 3, line.youngs_modulus, line.density,
                           1000 * line.diameter, init.init_stretched_length,
                           init.init_tether_force]]),
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
    line = tether_line()
    materials = MaterialTable(Material[], Dict{String, Any}())
    points = Point[Point(; name=:ground, fixed=true, pos_ENU=[0.0, 0.0, 0.0],
                         extra_mass=0.0),
                   Point(; name=:kcu, pos_ENU=[0.0, 0.0, 10.0], extra_mass=0.0),
                   Point(; name=:rider, tube=:strut, pos_ENU=[0.0, 0.0, 11.0],
                         extra_mass=0.0),
                   Point(; name=:carried, body=:strut_b, pos_ENU=[0.0, 1.0, 11.0],
                         extra_mass=0.0)]
    segments = Segment[KiteGeometry.line_segment(:bridle, (:kcu, :rider), set, materials;
                                                 model=NoModel(), line...)]
    point_transforms = Any[1, 1, 1, 1]
    tether = expand_tether!(points, segments, point_transforms, :main, :ground, :kcu, 3,
                            set, materials; init=tether_init(), model=NoModel(),
                            segment_model=NoModel(), line...)
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
    return system, transforms
end

"""The rider kite of [`authored_rider_kite`](@ref), loaded and placed, and its
transforms."""
function load_rider_kite(; elevation=60.0, azimuth=30.0)
    return load_authoring(authored_rider_kite(; elevation, azimuth);
                          set=placement_settings(), name="rider")
end

"""The rest length of each segment of `system`."""
rest_lengths(system) = [segment.rest_length for segment in system.segments]

"""Whether `a` and `b` have the same pose and rest lengths, to `atol`."""
function placed_alike(a, b; atol=1e-9)
    (pose_a, pose_b) = Pose(a), Pose(b)
    return all(isapprox.(pose_a.pos, pose_b.pos; atol)) &&
           all(isapprox.(pose_a.body_pos, pose_b.body_pos; atol)) &&
           all(isapprox.(pose_a.body_R, pose_b.body_R; atol)) &&
           all(isapprox.(rest_lengths(a), rest_lengths(b); atol))
end

"""The length of `tether` of `system` between the positions of its points [m]."""
function placed_length(system, tether)
    return sum(norm(system.points[a].pos_ENU - system.points[b].pos_ENU)
               for (a, b) in (system.segments[index].points for index in tether.segments))
end

@testset "a structure placed by each route lands on the same pose and rest lengths" begin
    from_yaml_route, _ = load_rider_kite()
    in_julia, julia_transforms = rider_kite_in_julia()
    place!(in_julia, julia_transforms)
    @test placed_alike(from_yaml_route, in_julia)

    replaced, replaced_transforms = load_rider_kite(; elevation=30.0, azimuth=-40.0)
    only(replaced_transforms).elevation = deg2rad(60)
    only(replaced_transforms).azimuth = deg2rad(30)
    place!(replaced, replaced_transforms)
    @test placed_alike(from_yaml_route, replaced)
end

@testset "a tube rider moves with the tube's bodies, on the line between them" begin
    system, _ = load_rider_kite()
    pose = Pose(system)
    rider, carried = system.points[:rider], system.points[:carried]
    @test rider.tube == 1 && isnothing(rider.body)
    @test pose.pos[3] ≈ (pose.body_pos[1] + pose.body_pos[2]) / 2 atol=1e-9
    @test norm(rider.pos_ENU - carried.pos_ENU) ≈ 1
end

@testset "a tether's init columns put it at its length and rest length" begin
    system, _ = load_rider_kite()
    tether = only(system.tethers)
    (; init_stretched_length, init_tether_force) = tether_init()
    @test tether.init_stretched_length == init_stretched_length
    @test placed_length(system, tether) ≈ init_stretched_length atol=1e-9
    line = tether_line()
    stiffness = line.youngs_modulus * π * (line.diameter / 2)^2
    @test sum(system.segments[segment].rest_length for segment in tether.segments) ≈
          init_stretched_length * (1 - init_tether_force / stiffness) atol=1e-9
end

@testset "placing again keeps the pose and rest lengths of a placed definition" begin
    system, transforms = load_rider_kite()
    placed, _ = load_rider_kite()
    place!(system, transforms)
    @test placed_alike(system, placed; atol=1e-12)
end

@testset "a tether whose row is given a new stretched length is placed at it" begin
    system, transforms = load_rider_kite()
    tether = only(system.tethers)
    tether.init_stretched_length = 25.0
    place!(system, transforms)
    @test placed_length(system, tether) ≈ 25.0 atol=1e-9
end

"""The authoring document of a tether over two segments of different lengths from a point
held in the world, giving no `init_*` column."""
function authored_uneven_tether()
    return OrderedDict{String, Any}(
        "points" => OrderedDict{String, Any}(
            "headers" => ["name", "pos_cad", "type"],
            "data" => [Any["ground", [0.0, 0.0, 0.0], "STATIC"],
                       Any["joint", [0.0, 0.0, 3.0], "DYNAMIC"],
                       Any["kcu", [0.0, 0.0, 10.0], "DYNAMIC"]]),
        "segments" => OrderedDict{String, Any}(
            "headers" => ["name", "point_i", "point_j"],
            "data" => [Any["low", "ground", "joint"], Any["high", "joint", "kcu"]]),
        "tethers" => OrderedDict{String, Any}(
            "headers" => ["name", "segment_idxs"],
            "data" => [Any["main", ["low", "high"]]]))
end

@testset "a tether given no init columns keeps its uneven rest lengths" begin
    system, _ = load_authoring(authored_uneven_tether(); set=placement_settings(),
                               name="uneven")
    @test rest_lengths(system) ≈ [3.0, 7.0] atol=1e-12
end

"""The authoring document of a body whose centre of mass is off its origin, turned by a
transform naming no `rot_point`."""
function authored_offset_body()
    return OrderedDict{String, Any}(
        "points" => OrderedDict{String, Any}(
            "headers" => ["name", "pos_cad", "type"],
            "data" => [Any["ground", [0.0, 0.0, 0.0], "STATIC"]]),
        "bodies" => OrderedDict{String, Any}(
            "headers" => ["name", "extra_mass", "pos", "inertia_principal",
                          "transform_idx", "com_offset_b"],
            "data" => [Any["boom", 1.0, [0.0, 0.0, 10.0], [1.0, 1.0, 1.0], "tf",
                           [0.0, 1.0, 0.0]]]),
        "transforms" => OrderedDict{String, Any}(
            "headers" => ["name", "elevation", "azimuth", "heading", "base_pos",
                          "base_point_idx"],
            "data" => [Any["tf", 60.0, 30.0, 0.0, [0.0, 0.0, 0.0], "ground"]]))
end

@testset "a body transform without a rot_point turns the body's centre of mass" begin
    system, _ = load_authoring(authored_offset_body(); set=placement_settings(),
                               name="boom")
    boom = system.bodies[:boom]
    radial = [cosd(60) * cosd(30), cosd(60) * sind(30), sind(60)]
    @test normalize(boom.pos_ENU) ≈ radial atol=1e-12
    @test norm(boom.pos_ENU) ≈ norm([0.0, 1.0, 10.0]) atol=1e-12
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
    @test system.bodies[:base].fixed
    @test system.bodies[:base].pos_ENU == [0.0, 0.0, -1.0]
    @test norm(system.points[:kcu].pos_ENU - system.points[:pad].pos_ENU) ≈ 30 atol=1e-9
end

@testset "only a definition holding a body in the world writes a fixed column" begin
    held, _ = load_authoring(authored_held_anchor(); set=placement_settings(), name="held")
    rider, _ = load_rider_kite()
    @test "fixed" in structure_document(held)["bodies"]["headers"]
    @test !("fixed" in structure_document(rider)["bodies"]["headers"])
    @test !any(body -> body.fixed, from_yaml(to_yaml(rider)).bodies)
end

@testset "a point riding a tube on a body or held in the world is refused" begin
    on_body = Point(; name=:rider, tube=:strut, body=:strut_a, pos_ENU=zeros(3))
    held = Point(; name=:rider, tube=:strut, fixed=true, pos_ENU=zeros(3))
    bodies = [Body(; name=:strut_a, pos_ENU=zeros(3)),
              Body(; name=:strut_b, pos_ENU=ones(3))]
    tubes = [Tube(; name=:strut, bodies=(:strut_a, :strut_b))]
    for point in (on_body, held)
        @test_throws "rides a tube" SystemDefinition(;
            metadata=Metadata(; name="rider", n_points=1), points=[point], bodies, tubes)
    end
end
