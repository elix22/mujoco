#!/bin/bash
# Regenerates MjBindings.cs with MuJoCo's own generator (python/mujoco/codegen/generate_cs_bindings.py)
# without Bazel. The generator reads the checked-in introspect tables (python/mujoco/introspect) and
# the public headers for #define constants. A venv with absl-py is kept in build/codegen-venv.
set -euo pipefail

MJ="$(cd "$(dirname "$0")/../.." && pwd)"
VENV="$MJ/build/codegen-venv"

if [ ! -x "$VENV/bin/python" ]; then
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install --quiet absl-py
fi

cd "$MJ"
PYTHONPATH="$MJ/python/mujoco" "$VENV/bin/python" python/mujoco/codegen/generate_cs_bindings.py \
    --header_file_list="$(ls include/mujoco/*.h | tr '\n' ' ')" \
    > bindings/csharp/MjBindings.cs
echo "wrote $MJ/bindings/csharp/MjBindings.cs ($(wc -l < bindings/csharp/MjBindings.cs) lines)"
