# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteUtils: Logger, SysState, load_log, log!, save_log

@testset "a log saved with a system definition reads it back" begin
    system = load_structure(joinpath(@__DIR__, "data", "v3_beam_structure.yml"))
    logger = Logger(3, 2)
    log!(logger, SysState(3))
    path = mktempdir()
    save_log(logger, system, "topology"; path, metadata=Dict("note" => "kept"))
    log = load_log("topology"; path)
    @test structure_document(KiteGeometry.definition(log)) == structure_document(system)
    @test log.metadata["note"] == "kept"
end
