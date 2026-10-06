<!--
SPDX-FileCopyrightText: 2026 Bart van de Lint
SPDX-License-Identifier: CC-BY-4.0
-->

### Added

- `to_yaml`/`from_yaml` and `to_json`/`from_json` write and read a structure document as text, and `KiteGeometry.definition` reads the `topology` string a log carries.
- `connectivity_sha` hashes the schema's connectivity preimage; `structure_document` writes the system's own.

### Changed

- BREAKING: reading a structure document refuses one whose `connectivity_sha` does not describe its own tables, or whose `awesIO_version` is another major version, and warns on another minor version.
- BREAKING: Julia 1.11 is no longer supported; KiteGeometry needs 1.12 or newer.
