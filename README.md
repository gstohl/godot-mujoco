# godot-mujoco

Minimal C bridge exposing MuJoCo runtime calls for Godot integration.

## What is included

- Opaque handles for `mjModel` and `mjData` (`gmj_model`, `gmj_data`)
- Model lifecycle (`gmj_model_load_xml`, `gmj_model_free`)
- Data lifecycle (`gmj_data_create`, `gmj_data_free`, `gmj_reset_data`)
- Simulation stepping (`gmj_step`, `gmj_forward`)
- State/control getters and setters (`qpos`, `qvel`, `ctrl`)
- Body world position query (`gmj_body_world_position`)
- Lightweight error string retrieval (`gmj_last_mujoco_error`)

The API is declared in `include/godot_mujoco/gmj_bridge.h` and implemented in `src/gmj_bridge.c`.

## Build

```bash
cmake -S . -B build
cmake --build build
```

This creates `godot_mujoco_bridge` as a shared library (`.dylib`, `.so`, or `.dll`).

## Notes for Godot

- This repository provides a C ABI bridge, not a full Godot class registration layer.
- In Godot 4, use this library from a small GDExtension C++ wrapper or via C# `DllImport`.
- MuJoCo headers/libraries must be installed and discoverable by your compiler/linker for full functionality.

If MuJoCo headers are not found during compile, the bridge still builds but returns `GMJ_ERR_MUJOCO` at runtime.
