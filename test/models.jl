# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

"""Take back the model registered for `block` under `name`, and its default."""
function unregister_model!(block, name)
    delete!(KiteGeometry.MODELS, (block, name))
    get(KiteGeometry.DEFAULT_MODELS, block, nothing) == name &&
        delete!(KiteGeometry.DEFAULT_MODELS, block)
end
