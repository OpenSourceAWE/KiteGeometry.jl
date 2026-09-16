# KiteGeometry

[![Stable Documentation](https://img.shields.io/badge/docs-stable-blue.svg)](https://OpenSourceAWE.github.io/KiteGeometry.jl/stable)
[![Development documentation](https://img.shields.io/badge/docs-dev-blue.svg)](https://OpenSourceAWE.github.io/KiteGeometry.jl/dev)
[![Test workflow status](https://github.com/OpenSourceAWE/KiteGeometry.jl/actions/workflows/Test.yml/badge.svg?branch=main)](https://github.com/OpenSourceAWE/KiteGeometry.jl/actions/workflows/Test.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/OpenSourceAWE/KiteGeometry.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/OpenSourceAWE/KiteGeometry.jl)
[![Docs workflow Status](https://github.com/OpenSourceAWE/KiteGeometry.jl/actions/workflows/Docs.yml/badge.svg?branch=main)](https://github.com/OpenSourceAWE/KiteGeometry.jl/actions/workflows/Docs.yml?query=branch%3Amain)
[![BestieTemplate](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/JuliaBesties/BestieTemplate.jl/main/docs/src/assets/badge.json)](https://github.com/JuliaBesties/BestieTemplate.jl)

One system definition for airborne wind energy systems — topology, CAD geometry
and material — shared by the OpenSourceAWE packages.

Every package here describes the same thing and each describes it differently:
SymbolicAWEModels has a `SystemStructure` and a YAML loader, KiteModels a scalar
segment count, KiteViewers a sidecar CSV with neither names nor tethers. State
is already shared, as `SysState`; the definition is not, so nothing composes and
no viewer can draw another package's system.

This package holds that definition. The awesIO structure schema is the format it
follows and the source of truth where the two differ; the field sets here are
written by hand.

```julia
using KiteGeometry, KiteUtils

set_data_path("data/2plate_kite")
definition = load_definition("data/2plate_kite/particle_structural_geometry.yaml";
                             set=Settings("system.yaml"))

definition.points[:kcu].pos_cad
definition.segments[:strut_left].model.unit_stiffness
```

Nothing in a `SystemDefinition` is written per simulation step, so the same
definition can be handed to the package that integrates the system and to the
one that draws it.

See the [documentation](https://OpenSourceAWE.github.io/KiteGeometry.jl/stable)
for the type, the YAML dialect it is read from, and the frames it uses.

## License

MIT.
