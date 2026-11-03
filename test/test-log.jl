# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteUtils: Logger, SysState, load_log, log!, save_log

include("models.jl")

@testset "a log saved with a system definition reads it back" begin
    with_v3_columns() do
        system = load_structure(joinpath(@__DIR__, "data", "structure",
                                         "v3_beam_structure.yml"))
        logger = Logger(3, 2)
        log!(logger, SysState(3))
        path = mktempdir()
        save_log(logger, system, "topology"; path, metadata=Dict("note" => "kept"))
        log = load_log("topology"; path)
        @test structure_document(KiteGeometry.definition(log)) == structure_document(system)
        @test system_document(KiteGeometry.definition(log)) == system_document(system)
        @test log.metadata["note"] == "kept"
        delete!(log.metadata, "system")
        @test_throws "the log carries no system" KiteGeometry.definition(log)
    end
end
