# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry: radial_direction
using KiteUtils: Settings, set_data_path
using LinearAlgebra: norm, normalize
using YAML

"""The chained-transform system with `edit` applied to its parsed authoring YAML, loaded
and placed, and its transforms."""
function load_chained(edit=identity)
    set_data_path(joinpath(@__DIR__, "data", "2plate_kite"))
    data = YAML.load_file(joinpath(@__DIR__, "data", "chained_transforms.yaml"))
    edit(data)
    path = joinpath(mktempdir(), "chained_transforms.yaml")
    YAML.write_file(path, data)
    return load_authoring(path; set=Settings("system.yaml"))
end

"""Whether the poses `a` and `b` agree to `atol`."""
function poses_agree(a, b; atol=1e-9)
    return all(isapprox.(a.pos, b.pos; atol)) &&
           all(isapprox.(a.body_pos, b.body_pos; atol)) &&
           all(isapprox.(a.body_R, b.body_R; atol))
end

"""Tilt the kite of the chained system to 48° elevation and -35° heading."""
function tilt_kite!(transforms)
    kite_tilt = transforms[2]
    kite_tilt.elevation, kite_tilt.heading = deg2rad(48), deg2rad(-35)
    return transforms
end

"""`data`, the chained system's YAML, with its kite tilted as [`tilt_kite!`](@ref)."""
function tilt_kite_row!(data)
    data["transforms"]["data"][2][2] = 48.0
    data["transforms"]["data"][2][4] = -35.0
    return data
end

@testset "each transform holds the points and bodies naming it" begin
    system, (main_tf, kite_tilt) = load_chained()
    names(indices, rows) = [rows[index].name for index in indices]
    @test names(main_tf.points, system.points) ==
          ["kcu", "nose", ("main_tether_point_$i" for i in 1:3)...]
    @test names(kite_tilt.points, system.points) == ["top", "left", "right", "kite_origin"]
    @test isempty(main_tf.bodies) && names(kite_tilt.bodies, system.bodies) == ["kite"]
    @test kite_tilt.base_transform == 1 && kite_tilt.rot_point == [7 => 1.0]
    @test isnothing(main_tf.body) && kite_tilt.body == 1
    @test rad2deg(kite_tilt.heading) ≈ 25
end

@testset "a placed system placed again by the same transforms stays where it is" begin
    system, transforms = load_chained()
    pose = Pose(system)
    @test poses_agree(Pose(place!(system, transforms)), pose)
    @test poses_agree(reposition!(copy(pose), transforms), pose)
end

@testset "placing again at new angles is loading at them" begin
    system, transforms = load_chained()
    place!(system, tilt_kite!(transforms))
    @test poses_agree(Pose(system), Pose(first(load_chained(tilt_kite_row!))))
end

@testset "placing again carries a chained transform with its base" begin
    system, transforms = load_chained()
    main_tf, kite_tilt = transforms
    main_tf.elevation, main_tf.azimuth = deg2rad(40), deg2rad(-20)
    kite_rigid = norm(system.points[4].pos_ENU - system.points[7].pos_ENU)
    pose = Pose(place!(system, transforms))
    ground, kcu, kite_origin = pose.pos[1], pose.pos[2], pose.pos[7]
    @test normalize(kcu - ground) ≈ radial_direction(main_tf)
    @test normalize(kite_origin - kcu) ≈ radial_direction(kite_tilt)
    @test calc_heading(pose.body_R[1], kite_origin - kcu) ≈ kite_tilt.heading
    @test norm(pose.pos[4] - kite_origin) ≈ kite_rigid
end

@testset "repositioning turns each transform to its angles about its base" begin
    system, transforms = load_chained()
    pose = reposition!(Pose(system), tilt_kite!(transforms))
    kcu, kite_origin = pose.pos[2], pose.pos[7]
    @test pose.pos[2] == system.points[2].pos_ENU
    @test normalize(kite_origin - kcu) ≈ radial_direction(transforms[2])
    @test calc_heading(pose.body_R[1], kite_origin - kcu) ≈ deg2rad(-35)
end

"""`pose` repositioned with every one of `transforms` advanced by its rates for `dt` [s]."""
function advanced(pose, transforms, dt)
    for transform in transforms
        transform.elevation += transform.elevation_vel * dt
        transform.azimuth += transform.azimuth_vel * dt
        transform.heading += transform.turn_rate * dt
    end
    moved = reposition!(copy(pose), transforms)
    for transform in transforms
        transform.elevation -= transform.elevation_vel * dt
        transform.azimuth -= transform.azimuth_vel * dt
        transform.heading -= transform.turn_rate * dt
    end
    return moved
end

@testset "transform velocities are the rate of change of the pose" begin
    for moving in 1:2
        system, transforms = load_chained()
        held = transforms[3 - moving]
        held.elevation_vel = held.azimuth_vel = held.turn_rate = 0.0
        # main_tf heads no body, so repositioning keeps no heading for an azimuth rate
        main_tf, kite_tilt = transforms
        main_tf.azimuth_vel, kite_tilt.azimuth_vel = 0.0, moving == 2 ? deg2rad(-1.5) : 0.0
        pose = Pose(system)
        point_vel, body_vel, body_spin = transform_velocities(pose, transforms)
        dt = 3e-3
        ahead, behind = advanced(pose, transforms, dt), advanced(pose, transforms, -dt)
        (; points, bodies) = transforms[moving]
        @test all(isapprox.((ahead.pos - behind.pos)[points] / 2dt, point_vel[points];
                            atol=1e-6))
        @test all(isapprox.((ahead.body_pos - behind.body_pos)[bodies] / 2dt,
                            body_vel[bodies]; atol=1e-6))
        @test iszero(point_vel[1])
        @test iszero(body_spin[1]) == (moving == 1)
    end
end
