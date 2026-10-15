### Fixed

- A model field typed `Union{Nothing, Float64, Vector{Float64}}` reads a cell holding one number or a vector; both were refused with a `MethodError`.
