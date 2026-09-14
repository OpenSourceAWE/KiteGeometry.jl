<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: MIT
-->

# Changelog

## KiteGeometry v0.1.0

### Added

- `SystemDefinition`, the shared description of an airborne wind energy
  system: `Point`, `TwistSurface`, `Segment`, `Pulley`, `Tether`, `Winch`,
  `Body`, `Joint` and `Transform`, each collection a `NamedCollection`
  indexable by name or by position.
- `load_definition`, the authoring YAML loader, including the `variables`
  block and the generation of a tether's intermediate points and segments.
- `LinearSpring` and `NonlinearSpring` under `AbstractSegmentModel`,
  `ElasticJoint` and `TimoshenkoBeam` under `AbstractJointModel`, and
  `TorqueWinch` under `AbstractWinchModel`. Each is the type parameter of the
  component that holds it, so a system sharing one model is concretely typed.
