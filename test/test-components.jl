# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

@testset "a collection is indexable by name and by position" begin
    points = NamedCollection([Point(:kcu, [0.0, 0.0, 0.0], DYNAMIC),
                              Point(:ground, [0.0, 0.0, -10.0], STATIC)])

    @test length(points) == 2
    @test points[1] === points[:kcu]
    @test points[:ground].type == STATIC
    @test haskey(points, :kcu)
    @test !haskey(points, :absent)
    @test points isa AbstractVector{Point}
    @test [point.type for point in points] == [DYNAMIC, STATIC]
    @test_throws "Name 'absent' not found" points[:absent]
    @test_throws "Duplicate name 'kcu'" NamedCollection(
        [Point(:kcu, [0.0, 0.0, 0.0], DYNAMIC),
         Point(:kcu, [1.0, 0.0, 0.0], DYNAMIC)])
end

@testset "a nonlinear spring carries its force law and stays concrete" begin
    force_law = strain -> 1.0e4 * strain^3
    segment = Segment(:line, :a, :b, NonlinearSpring(force_law, 12.0))

    @test segment.model.force_law === force_law
    @test segment.model.unit_damping == 12.0
    @test isconcretetype(typeof(segment))
    @test segment isa Segment{<:NonlinearSpring}
end

@testset "material keywords complete each other from the settings" begin
    set = Settings()
    set.e_tether = 5.5e10
    set.rho_tether = 724.0
    set.d_tether = 4.0        # [mm]
    set.rel_damping = 0.01

    # A diameter-independent material scales with the cross section.
    thin = Segment(:thin, set, :a, :b;
        diameter=0.001, youngs_modulus=5.5e10, damping_per_stiffness=0.001)
    thick = Segment(:thick, set, :a, :b;
        diameter=0.002, youngs_modulus=5.5e10, damping_per_stiffness=0.001)
    @test thin.model.unit_stiffness ≈ 5.5e10 * π * 0.0005^2
    @test thick.model.unit_stiffness ≈ 4 * thin.model.unit_stiffness
    @test thin.model.unit_damping ≈ 0.001 * thin.model.unit_stiffness

    # What is left out comes from the settings.
    default = Segment(:default, set, :a, :b)
    @test default.diameter ≈ 0.004
    @test default.density == 724.0
    @test default.model.unit_stiffness ≈ 5.5e10 * π * 0.002^2
    @test default.model.unit_damping ≈ 0.01 * default.model.unit_stiffness

    @test_throws "not both" Segment(:both, set, :a, :b;
        unit_stiffness=1.0e4, youngs_modulus=5.5e10)
    @test_throws "not both" Segment(:both, set, :a, :b;
        unit_damping=1.0, damping_per_stiffness=0.001)
    @test_throws "must be given explicitly" Segment(:law, set, :a, :b;
        unit_stiffness=strain -> 1.0e4 * strain)
end

@testset "a full inertia tensor diagonalises into the principal frame" begin
    inertia = [2.0 0.0 0.5; 0.0 3.0 0.0; 0.5 0.0 4.0]
    body = Body(:hub; mass=2.0, inertia=inertia, pos_cad=[0.0, 0.0, 1.0])

    rotation = body.R_KA_to_principal
    @test rotation * inertia * rotation' ≈ Diagonal(body.inertia_principal)
    @test det(rotation) ≈ 1.0
    @test sum(body.inertia_principal) ≈ sum(diag(inertia))

    # Rotating only about y is the unique answer for an xz-symmetric body.
    moments, y_rotation = principal_frame(inertia, Y_ROTATION)
    @test y_rotation[2, 2] == 1.0
    @test y_rotation * inertia * y_rotation' ≈ Diagonal(moments)

    @test_throws "not both" Body(:hub; mass=2.0, inertia=inertia,
        inertia_principal=[1.0, 1.0, 1.0], pos_cad=[0.0, 0.0, 0.0])
    @test_throws "provide `inertia_principal` or `inertia`" Body(:hub;
        mass=2.0, pos_cad=[0.0, 0.0, 0.0])
end

@testset "a point states which body it rides" begin
    @test_throws "BODY_STATIC requires" Point(:p, [0.0, 0.0, 0.0], BODY_STATIC)
    @test_throws "only valid with type BODY_STATIC" Point(:p, [0.0, 0.0, 0.0],
        DYNAMIC; body=:hub)
    @test_throws "not both" Point(:p, [0.0, 0.0, 0.0], BODY_STATIC;
        body=:hub, joint=:beam)
    @test_throws "requires type BODY_STATIC" Point(:p, [0.0, 0.0, 0.0],
        DYNAMIC; joint=:beam)

    # A scalar damping is the same on all three axes.
    @test Point(:p, [0.0, 0.0, 0.0], DYNAMIC;
        body_frame_damping=5.0).body_frame_damping == [5.0, 5.0, 5.0]
    @test Point(:p, [0.0, 0.0, 0.0], DYNAMIC;
        world_frame_damping=[1.0, 2.0, 3.0]).world_frame_damping == [1, 2, 3]
end

@testset "a tether's initial length is set one way or the other" begin
    set = Settings()
    set.rel_damping = 0.01
    @test_throws "only one of" Tether(:line, set; start_point=:a,
        end_point=:b, n_segments=2, tether_force=100.0, stretch_frac=0.9)

    slack = Tether(:line, set, 10.0; start_point=:a, end_point=:b,
        n_segments=2, stretch_frac=1.1)
    @test slack.init_stretched_len == 10.0
    @test slack.init_stretch_frac == 1.1
    @test isnothing(slack.init_tether_force)

    # Neither given is a zero initial spring force.
    @test Tether(:line, set; start_point=:a, end_point=:b,
        n_segments=2).init_tether_force == 0.0
end

@testset "a transform names one base and one rotated object" begin
    @test_throws "not both or neither" Transform(:t, 0.0, 0.0, 0.0;
        base_pos=[0.0, 0.0, 0.0], base_point=:ground)
    @test_throws "not both or neither" Transform(:t, 0.0, 0.0, 0.0;
        wing=:main, base_pos=[0.0, 0.0, 0.0], base_transform=:other)
    @test_throws "also needs a `base_point`" Transform(:t, 0.0, 0.0, 0.0;
        wing=:main, base_pos=[0.0, 0.0, 0.0])

    chained = Transform(:t, 0.1, 0.2, 0.3; rot_point=:kcu, base_transform=:root)
    @test chained.elevation == 0.1
    @test isnothing(chained.base_pos_ENU)
    @test chained.rot_point_ref == :kcu
end

@testset "reference points average and weight the frame they define" begin
    equal = WeightedRefPoints([:le, :te])
    @test equal.refs == [:le, :te]
    @test equal.weights == [0.5, 0.5]

    weighted = WeightedRefPoints([(:le, 0.7), (:te, 0.3)])
    @test weighted.weights ≈ [0.7, 0.3]
    @test (@test_logs (:warn, r"normalizing") WeightedRefPoints(
        [(:le, 7.0), (:te, 3.0)])).weights ≈ [0.7, 0.3]
    @test_throws "at least one" WeightedRefPoints(Symbol[])
end
