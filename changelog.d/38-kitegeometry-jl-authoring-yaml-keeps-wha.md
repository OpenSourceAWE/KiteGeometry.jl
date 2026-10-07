### Added

- `register_model!(block, name, M, units; default=true)` makes `M` the model every row of a `block` table without a `model` column reads, in `load_structure` and `load_authoring` alike; a block takes one default.

### Changed

- BREAKING: `load_authoring` refuses a filled cell that neither the loader nor the row's registered model reads, naming the block and its columns, where it used to drop it silently; a point's `tube` is read by a point model that has a `tube` field, and an `idx` column must repeat the row's position.
- BREAKING: `load_authoring` refuses a block it does not read, such as a filled `groups` table, where it used to skip it; a table without rows still loads.
