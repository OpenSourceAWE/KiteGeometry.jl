# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: quaternion_to_rotation_matrix
using KiteUtils: Settings, set_data_path
using LinearAlgebra: norm, normalize

"""The chained-transform system, loaded and placed."""
function load_chained()
    set_data_path(joinpath(@__DIR__, "data", "2plate_kite"))
    return load_authoring(joinpath(@__DIR__, "data", "chained_transforms.yaml");
                          set=Settings("system.yaml"))
end

"""The `Pose` of `system` where it stands, each body at its mass centre."""
function definition_pose(system)
    return Pose([point.pos_ENU for point in system.points],
                [body.pos_ENU for body in system.bodies],
                [quaternion_to_rotation_matrix(body.Q_KA2ENU) for body in system.bodies])
end

"""Whether the transforms `a` and `b` agree, their angles to rounding."""
function transforms_agree(a, b)
    return length(a) == length(b) &&
           all(isequal(getfield(x, field), getfield(y, field)) ||
               getfield(x, field) ≈ getfield(y, field)
               for (x, y) in zip(a, b) for field in fieldnames(Transform))
end

"""Whether the poses `a` and `b` agree to `atol`."""
function poses_agree(a, b; atol=1e-9)
    return all(isapprox.(a.pos, b.pos; atol)) && all(isapprox.(a.body_pos, b.body_pos; atol)) &&
           all(isapprox.(a.body_R, b.body_R; atol))
end

@testset "a loaded definition keeps its transforms and their members" begin
    system = load_chained()
    main_tf, kite_tilt = system.transforms
    names(indices, rows) = [rows[index].name for index in indices]
    @test names(main_tf.points, system.points) ==
          ["kcu", "nose", "main_tether_point_1", "main_tether_point_2", "main_tether_point_3"]
    @test names(kite_tilt.points, system.points) == ["top", "left", "right", "kite_origin"]
    @test isempty(main_tf.bodies) && names(kite_tilt.bodies, system.bodies) == ["kite"]
    @test kite_tilt.base_transform == 1 && kite_tilt.rot_point == 7
    @test rad2deg(kite_tilt.heading) ≈ 25
end

@testset "a structure document round-trips the transforms" begin
    system = load_chained()
    document = structure_document(system)
    @test document["transforms"]["units"][2] == "deg"
    @test !haskey(system.extras, "transforms")
    read_back = from_yaml(to_yaml(system))
    @test transforms_agree(read_back.transforms, system.transforms)
    @test !haskey(read_back.extras, "transforms")
    @test transforms_agree(from_json(to_json(system)).transforms, system.transforms)
end

@testset "a placed system repositioned to its own angles stays where it is" begin
    system = load_chained()
    pose = definition_pose(system)
    @test poses_agree(reposition!(copy(pose), system), pose)
end

@testset "repositioning to new angles turns each transform there" begin
    system = load_chained()
    main_tf, kite_tilt = system.transforms
    main_tf.elevation, main_tf.azimuth = deg2rad(40), deg2rad(-20)
    kite_tilt.heading = deg2rad(-35)
    pose = reposition!(definition_pose(system), system)
    ground, kcu, kite_origin = pose.pos[1], pose.pos[2], pose.pos[7]
    @test normalize(kcu - ground) ≈ KiteGeometry.radial_direction(main_tf)
    @test normalize(kite_origin - kcu) ≈ KiteGeometry.radial_direction(kite_tilt)
    @test calc_heading(pose.body_R[1], kite_origin - kcu) ≈ deg2rad(-35)
    @test norm(pose.pos[4] - kite_origin) ≈
          norm(system.points[4].pos_ENU - system.points[7].pos_ENU)
end

"""`system` repositioned from `pose` with every transform advanced by its rates for `dt`
[s]."""
function advanced(pose, system, dt)
    for transform in system.transforms
        transform.elevation += transform.elevation_vel * dt
        transform.azimuth += transform.azimuth_vel * dt
        transform.heading += transform.turn_rate * dt
    end
    moved = reposition!(copy(pose), system)
    for transform in system.transforms
        transform.elevation -= transform.elevation_vel * dt
        transform.azimuth -= transform.azimuth_vel * dt
        transform.heading -= transform.turn_rate * dt
    end
    return moved
end

@testset "transform velocities are the rate of change of the pose" begin
    for moving in 1:2
        system = load_chained()
        held = system.transforms[3 - moving]
        held.elevation_vel = held.azimuth_vel = held.turn_rate = 0.0
        # main_tf moves no body, so repositioning keeps no heading for an azimuth rate
        main_tf, kite_tilt = system.transforms
        main_tf.azimuth_vel, kite_tilt.azimuth_vel = 0.0, moving == 2 ? deg2rad(-1.5) : 0.0
        pose = definition_pose(system)
        point_vel, body_vel, body_spin = transform_velocities(pose, system)
        dt = 3e-3
        ahead, behind = advanced(pose, system, dt), advanced(pose, system, -dt)
        (; points, bodies) = system.transforms[moving]
        @test all(isapprox.((ahead.pos - behind.pos)[points] / 2dt, point_vel[points];
                            atol=1e-6))
        @test all(isapprox.((ahead.body_pos - behind.body_pos)[bodies] / 2dt,
                            body_vel[bodies]; atol=1e-6))
        @test iszero(point_vel[1])
        @test iszero(body_spin[1]) == (moving == 1)
    end
end
