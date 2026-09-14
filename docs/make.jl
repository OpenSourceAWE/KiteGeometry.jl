# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using KiteGeometry
using Pkg
if !("Documenter" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Documenter

DocMeta.setdocmeta!(KiteGeometry, :DocTestSetup, :(using KiteGeometry);
                    recursive=true)

makedocs(;
    modules=[KiteGeometry],
    authors="Bart van de Lint <bart@vandelint.net> and contributors",
    repo="https://github.com/OpenSourceAWE/KiteGeometry.jl/blob/{commit}{path}#{line}",
    sitename="KiteGeometry.jl",
    format=Documenter.HTML(;
        repolink="https://github.com/OpenSourceAWE/KiteGeometry.jl",
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://OpenSourceAWE.github.io/KiteGeometry.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "The YAML file" => "yaml.md",
        "Exported types" => "types.md",
        "Exported functions" => "functions.md",
        "Internals" => "internals.md",
    ],
)

deploydocs(;
    repo="github.com/OpenSourceAWE/KiteGeometry.jl",
    devbranch="main",
)
