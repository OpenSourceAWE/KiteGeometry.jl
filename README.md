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

This package is where that definition goes. The awesIO structure schema defines
it, and the component types are generated from that schema rather than written
here, so the two cannot drift apart.

See the [documentation](https://OpenSourceAWE.github.io/KiteGeometry.jl/stable)
for more information.

## License

MIT.
