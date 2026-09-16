# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

# The 2-plate kite of SymbolicAWEModels, loaded field by field. `dyneema` is
# the material variable the file names; the stiffness a segment ends up with
# is `youngs_modulus · π(d/2)²`, and its damping `damping_per_stiffness` times
# that.
dyneema_stiffness(diameter_mm) = 55.0e9 * π * (0.0005 * diameter_mm)^2
wing_element_stiffness(diameter_mm) =
    6.3661977236758134e9 * π * (0.0005 * diameter_mm)^2

@testset "a particle-wing system loads with every reference resolved" begin
    definition = load_2plate("particle_structural_geometry.yaml")

    @test definition.name == "2plate_kite"
    @test definition isa SystemDefinition{Segment{LinearSpring},
        Tether{LinearSpring}, Winch{TorqueWinch}}

    # 10 written points plus the 5 the 6-segment tether generates.
    @test length(definition.points) == 15
    @test length(definition.segments) == 27
    @test length(definition.pulleys) == 2
    @test length(definition.tethers) == 1
    @test length(definition.winches) == 1
    @test length(definition.twist_surfaces) == 3
    @test length(definition.wings) == 1
    @test length(definition.bodies) == 1
    @test length(definition.transforms) == 1
    @test isempty(definition.joints)

    kcu = definition.points[:kcu]
    @test kcu.idx == 9
    @test kcu.pos_cad == [0.0, 0.0, 0.0]
    @test kcu.type == DYNAMIC
    @test kcu.extra_mass == 1.0
    @test kcu.body_frame_damping == [0.0, 0.0, 0.0]
    @test kcu.area == 0.1
    @test kcu.drag_coeff == 1.0
    @test kcu.wing_idx == 1
    @test kcu.transform_idx == 1
    @test kcu.body_idx == 0
    @test !kcu.is_wing_node

    le_left = definition.points[:le_left]
    @test le_left.pos_cad == [-0.5, 1.0, 2.0]
    @test le_left.body_frame_damping == [10.0, 10.0, 10.0]
    @test le_left.is_wing_node
    @test definition.points[:ground].type == STATIC
    @test !definition.points[:ground].is_wing_node

    strut = definition.segments[:strut_left]
    @test strut.point_idxs == (definition.points[:le_left].idx,
                               definition.points[:te_left].idx)
    @test strut.l0 ≈ sqrt(1.0^2 + 0.3^2)   # from the CAD geometry: `l0` is unset
    @test strut.diameter ≈ 0.001
    @test strut.density == 724.0
    @test strut.compression_frac == 1.0
    @test strut.model.unit_stiffness ≈ wing_element_stiffness(1.0)
    @test strut.model.unit_damping ≈ 0.002 * wing_element_stiffness(1.0)

    bridle = definition.segments[:le_steering_left]
    @test bridle.compression_frac == 0.01   # the `bridle_comp` variable
    @test bridle.model.unit_stiffness ≈ dyneema_stiffness(1.0)
    @test bridle.model.unit_damping ≈ 0.00077 * dyneema_stiffness(1.0)

    pulley = definition.pulleys[:left]
    @test pulley.segment_idxs == (definition.segments[:le_steering_left].idx,
                                  definition.segments[:te_steering_left].idx)
    @test pulley.type == DYNAMIC
    @test pulley.efficiency == 0.95

    tether = definition.tethers[:main_tether]
    @test tether.n_segments == 6
    @test tether.start_point_idx == definition.points[:kcu].idx
    @test tether.end_point_idx == definition.points[:ground].idx
    @test tether.init_stretched_len == 20.0
    @test tether.diameter ≈ 0.001
    @test tether.model.unit_stiffness ≈ dyneema_stiffness(1.0)

    winch = definition.winches[:main_winch]
    @test winch.tether_idxs == [tether.idx]
    @test winch.winch_point_idx == definition.points[:ground].idx
    @test winch.drum_radius == 0.110
    @test winch.gear_ratio == 1.0
    @test winch.inertia_total == 0.024
    @test winch.coulomb_friction == 122.0
    @test winch.viscous_coefficient == 30.6
    @test winch.model isa TorqueWinch
    @test winch.model.friction_epsilon == 6.0

    surface = definition.twist_surfaces[:center]
    @test surface.point_idxs == [definition.points[:le_center].idx,
                                 definition.points[:te_center].idx]
    @test surface.type == STATIC
    @test surface.moment_frac == 0.75
    @test surface.damping == 50.0

    wing = definition.wings[:main_wing]
    @test wing === definition.bodies[:main_wing]
    @test is_wing(wing)
    @test wing.dynamics_type == PARTICLE_DYNAMICS
    @test wing.type == KINEMATIC
    @test wing.aero_model == :direct
    @test wing.twist_surface_idxs == [1, 2, 3]
    @test wing.origin.ids == [definition.points[:kcu].idx]
    @test wing.z_ref_points[1].ids == [definition.points[:kcu].idx]
    @test wing.z_ref_points[2].ids == [definition.points[:le_center].idx]
    @test wing.y_ref_points[1].ids == [definition.points[:le_right].idx]
    @test wing.y_ref_points[2].ids == [definition.points[:le_left].idx]

    transform = definition.transforms[:main_transform]
    @test transform.elevation ≈ deg2rad(50)
    @test transform.azimuth == 0.0
    @test transform.heading == 0.0
    @test transform.turn_rate == 0.0
    @test transform.base_pos_ENU == [0.0, 0.0, 0.0]
    @test transform.base_point_idx == definition.points[:ground].idx
    @test transform.wing_idx == wing.idx
    @test isnothing(transform.rot_point_idx)
end

@testset "a generated tether spans its endpoints in equal segments" begin
    definition = load_2plate("particle_structural_geometry.yaml")
    tether = definition.tethers[:main_tether]
    points = definition.points

    @test length(tether.segment_idxs) == 6
    chain = [definition.segments[i] for i in tether.segment_idxs]
    @test chain[1].point_idxs[1] == points[:kcu].idx
    @test chain[end].point_idxs[2] == points[:ground].idx
    for (first_segment, next_segment) in zip(chain, chain[2:end])
        @test first_segment.point_idxs[2] == next_segment.point_idxs[1]
    end
    # The rest lengths divide the placed standoff, not the CAD distance of 20 m
    # between kcu and ground — here the two happen to agree.
    @test all(segment -> segment.l0 ≈ 20.0 / 6, chain)
    @test points[:main_tether_point_3].pos_cad ≈ [0.0, 0.0, -10.0]
    @test points[:main_tether_point_3].transform_idx == 1
    @test points[:main_tether_point_3].type == DYNAMIC
end

@testset "a tether of explicit segments takes their endpoints" begin
    yaml = """
    points:
      headers: [name, pos_cad, type]
      data:
        - [ground, [0.0, 0.0, 0.0], STATIC]
        - [middle, [0.0, 0.0, 5.0], DYNAMIC]
        - [top, [0.0, 0.0, 10.0], DYNAMIC]
    segments:
      headers: [name, point_i, point_j, diameter_mm, youngs_modulus,
                damping_per_stiffness]
      data:
        - [lower, ground, middle, 4.0, 55.0e9, 0.00077]
        - [upper, middle, top, 4.0, 55.0e9, 0.00077]
    tethers:
      headers: [name, segment_idxs]
      data:
        - [line, [lower, upper]]
    """
    definition = load_yaml(yaml)

    tether = definition.tethers[:line]
    @test tether.n_segments == 2
    @test tether.segment_idxs == [1, 2]
    # Neither endpoint is written, so they come off the ends of the chain.
    @test tether.start_point_idx == definition.points[:ground].idx
    @test tether.end_point_idx == definition.points[:top].idx
    @test isnothing(tether.init_stretched_len)
    @test tether.init_tether_force == 0.0
    # No point or segment is generated for it.
    @test length(definition.points) == 3
    @test length(definition.segments) == 2
    @test definition.segments[:upper].l0 ≈ 5.0
end

@testset "a rigid wing's nodes ride its body" begin
    definition = load_2plate("rigid_structural_geometry.yaml")

    wing = definition.wings[:main_wing]
    @test wing.dynamics_type == RIGID_DYNAMICS
    @test wing.type == DYNAMIC
    @test wing.aero_model == :linearized

    for name in (:le_left, :te_left, :le_center, :te_center, :le_right,
                 :te_right, :kcu)
        point = definition.points[name]
        @test point.type == BODY_STATIC
        @test point.body_idx == wing.idx
    end
    @test definition.points[:steering_left].type == DYNAMIC
    @test definition.points[:steering_left].body_idx == 0

    @test definition.twist_surfaces[:left].type == DYNAMIC
    @test definition.twist_surfaces[:left].moment_frac == 0.25
    @test definition.twist_surfaces[:left].damping == 100.0
end

@testset "the caller overrides the wing columns of the file" begin
    definition = load_2plate("particle_structural_geometry.yaml";
                             dynamics_type=RIGID_DYNAMICS, aero_model=:none)
    @test definition.wings[:main_wing].dynamics_type == RIGID_DYNAMICS
    @test !is_wing(definition.bodies[:main_wing])
end

@testset "an unknown reference names the component it could not find" begin
    yaml = """
    points:
      headers: [name, pos_cad, type]
      data:
        - [anchor, [0.0, 0.0, 0.0], STATIC]
    segments:
      headers: [name, point_i, point_j, diameter_mm, youngs_modulus,
                damping_per_stiffness]
      data:
        - [line, anchor, missing_point, 1.0, 55.0e9, 0.00077]
    """
    @test_throws "Unknown point name: missing_point" load_yaml(yaml)
end

@testset "a file with no points and no bodies is rejected" begin
    @test_throws "No points or bodies found" load_yaml("transforms:\n  data: []\n")
end
