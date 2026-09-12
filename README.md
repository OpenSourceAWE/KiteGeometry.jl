# KiteGeometry.jl

One system definition for airborne wind energy systems — topology, CAD geometry
and material — shared by the OpenSourceAWE packages.

Every package here describes the same thing and each describes it differently:
SymbolicAWEModels has a `SystemStructure` and a YAML loader, KiteModels a scalar
segment count, KiteViewers a sidecar CSV with neither names nor tethers. State
is already shared, as `SysState`; the definition is not, so nothing composes and
no viewer can draw another package's system.

This package holds that definition. The awesIO structure document is its
interface — YAML and JSON are two encodings of one model — and the Julia type
mirrors it one to one, so a system that can be written can be read by anything
that reads the schema.

**Nothing is here yet.** The work is scheduled as
[`plans/topology_state_split_plan.md`](https://github.com/1-Bart-1/Agents/blob/main/plans/topology_state_split_plan.md);
the units filed against this repository fill it.

## License

MIT.
