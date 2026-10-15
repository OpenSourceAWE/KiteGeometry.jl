### Added

- The `materials` block and its `Material` rows; segments, tubes and a wing's `canopy_material` name one.
- A wing's `pos_ENU` and `Q_KA2ENU`, the origin and rotation of its KA frame, and `pos_KA(wing, pos_ENU)`, which puts a world position into that frame.
- Optional awesIO columns are fields that are `nothing` where a document leaves them out, and are written only where a row fills them.

### Changed

- BREAKING: KiteGeometry follows awesIO 0.1.0 (1-Bart-1/awesIO@99a36e2) and refuses awesIO 1.0.0 documents. A point is `name`, `fixed`, `body`, `pos_ENU`; a segment `name`, `points`; `l0` is the optional `rest_length`, `Station` is `SpanwiseStation` in the `spanwise_stations` block, and a body's `Q_KA_to_ENU` is `Q_KA2ENU`. `DynamicsType`, point drag, a segment's density and `unit_stiffness`, a body's and a station's type, and a pulley's type and efficiency are no longer fields; the last four are columns of the models a tool registers.
- BREAKING: `load_authoring` writes the kite at zenith: transforms only shift their members so the base point is at its base position, each wing's KA frame is the authoring frame so shifted, and the `transforms` block is kept as an extra block. Segment stiffness and density become `linear` materials, and a tube's `breukels2011` law a material with SymbolicAWEModels' constants.
- The writer leaves out the `model` column of a table whose rows all read their block's default model, and a model column no row fills. A model column a document leaves out reads as unset where its field can be `nothing`.
