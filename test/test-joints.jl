# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

# Two bodies a metre apart on the x axis, the shape every joint test links.
two_bodies() = [
    Body(:root; mass=1.0, inertia_principal=[1.0, 1.0, 1.0],
         pos_CAD=[0.0, 0.0, 0.0], type=STATIC),
    Body(:tip; mass=1.0, inertia_principal=[1.0, 1.0, 1.0],
         pos_CAD=[1.0, 0.0, 0.0])]

elastic_model() = ElasticJoint(; stiffness_axial=1.0e4, stiffness_shear=2.0e3,
    stiffness_torsion=30.0, stiffness_bending=40.0)

beam_model() = TimoshenkoBeam(; EA=1.0e4, GA=2.0e3, GJ=30.0, EIy=40.0, EIz=50.0)

@testset "both joint models live in one collection of one type" begin
    definition = SystemDefinition(:beam, Settings();
        bodies=two_bodies(),
        joints=[Joint(:elastic, :root, :tip, elastic_model()),
                Joint(:beam, :root, :tip, beam_model())])

    @test length(definition.joints) == 2
    @test definition.joints[:elastic].model isa ElasticJoint{SimFloat}
    @test definition.joints[:beam].model isa TimoshenkoBeam{SimFloat}
    # Mixed models keep the abstract element type; one model narrows to it.
    @test eltype(definition.joints) == Joint

    beams = SystemDefinition(:beam, Settings();
        bodies=two_bodies(), joints=[Joint(:beam, :root, :tip, beam_model())])
    @test eltype(beams.joints) == Joint{TimoshenkoBeam{SimFloat}}
end

@testset "a joint resolves both its bodies by name" begin
    definition = SystemDefinition(:beam, Settings();
        bodies=two_bodies(), joints=[Joint(:beam, :root, :tip, beam_model();
            anchor_a=[0.1, 0.0, 0.0], damping=0.05, radius=0.02)])

    joint = definition.joints[:beam]
    @test joint.idx == 1
    @test joint.body_a_idx == definition.bodies[:root].idx
    @test joint.body_b_idx == definition.bodies[:tip].idx
    @test joint.anchor_a_KA == [0.1, 0.0, 0.0]
    @test joint.anchor_b_KA == [0.0, 0.0, 0.0]
    @test joint.damping == 0.05
    @test joint.radius == 0.02
    @test joint.model.shear_coeff ≈ 5 / 6
    @test joint.model.rest_length == 0.0
end

@testset "a nonlinear stiffness keeps the joint concrete" begin
    softening = curvature -> 40.0 / (1 + abs(curvature))
    model = TimoshenkoBeam(; EA=1.0e4, GA=2.0e3, GJ=30.0, EIy=softening,
                           EIz=50.0)
    @test model.EIy === softening
    @test model.EA == 1.0e4
    @test isconcretetype(typeof(model))

    @test_throws "must be the same type" TimoshenkoBeam(; EA=1.0e4, GA=2.0e3,
        GJ=30.0, EIy=softening, EIz=curvature -> 50.0)
end

@testset "a point rides the deformed centerline of its beam" begin
    definition = SystemDefinition(:beam, Settings();
        points=[Point(:mid, [0.25, 0.0, 0.1], BODY_STATIC; joint=:beam)],
        bodies=two_bodies(),
        joints=[Joint(:beam, :root, :tip, beam_model())])

    mid = definition.points[:mid]
    @test mid.joint_idx == 1
    @test mid.beam_frac ≈ 0.25
    @test mid.beam_offset_b ≈ [0.0, 0.0, 0.1]
end

@testset "a body-anchored point takes its anchor from the CAD geometry" begin
    definition = SystemDefinition(:beam, Settings();
        points=[Point(:tip_anchor, [1.2, 0.0, 0.3], BODY_STATIC; body=:tip)],
        bodies=two_bodies())

    anchor = definition.points[:tip_anchor]
    @test anchor.body_idx == definition.bodies[:tip].idx
    @test anchor.anchor_KA ≈ [0.2, 0.0, 0.3]
end

@testset "a body's Q_b_to_w column is its rotation into the CAD frame" begin
    yaml = """
    bodies:
      headers: [name, mass, pos, inertia_principal, Q_b_to_w]
      data:
        - [turned, 1.0, [1.0, 0.0, 0.0], [1.0, 1.0, 1.0],
           [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]]
    points:
      headers: [name, pos_cad, type, body_idx]
      data:
        - [rider, [1.0, 0.2, 0.3], BODY_STATIC, turned]
    """
    definition = load_yaml(yaml)

    @test definition.bodies[:turned].R_KA_to_CAD ≈ [0 -1 0; 1 0 0; 0 0 1]
    @test definition.points[:rider].anchor_KA ≈ [0.2, 0.0, 0.3]
end

@testset "joints load from their two YAML blocks" begin
    yaml = """
    bodies:
      headers: [name, mass, pos, inertia_principal, type]
      data:
        - [root, 1.0, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], STATIC]
        - [tip, 1.0, [1.0, 0.0, 0.0], [1.0, 1.0, 1.0], DYNAMIC]
    elastic_joints:
      headers: [name, body_a, body_b, stiffness_axial, stiffness_shear,
                stiffness_torsion, stiffness_bending, damping]
      data:
        - [hinge, root, tip, 10000.0, 2000.0, 30.0, 40.0, 0.01]
    timoshenko_joints:
      headers: [name, body_a, body_b, EA, GA, GJ, EIy, EIz, rest_length]
      data:
        - [beam, root, tip, 10000.0, 2000.0, 30.0, 40.0, 50.0, 1.0]
    """
    definition = load_yaml(yaml)

    @test length(definition.bodies) == 2
    @test definition.bodies[:root].type == STATIC
    @test definition.bodies[:tip].type == DYNAMIC
    @test !is_wing(definition.bodies[:root])

    hinge = definition.joints[:hinge]
    @test hinge.model isa ElasticJoint
    @test hinge.model.stiffness_axial == 10000.0
    @test hinge.model.stiffness_bending == 40.0
    @test hinge.damping == 0.01

    beam = definition.joints[:beam]
    @test beam.model isa TimoshenkoBeam
    @test beam.model.EIz == 50.0
    @test beam.model.rest_length == 1.0
    @test beam.body_a_idx == definition.bodies[:root].idx
    @test beam.body_b_idx == definition.bodies[:tip].idx
end

@testset "a joint row without both bodies is rejected" begin
    yaml = """
    bodies:
      headers: [name, mass, pos, inertia_principal]
      data:
        - [root, 1.0, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]
    timoshenko_joints:
      headers: [name, body_a, EA, GA, GJ, EIy, EIz]
      data:
        - [beam, root, 10000.0, 2000.0, 30.0, 40.0, 50.0]
    """
    @test_throws "requires `body_a` and `body_b`" load_yaml(yaml)
end
