# godot-mujoco GDExtension (full in-engine MuJoCo)

This is a **bridge-free** integration: MuJoCo runs *inside* Godot as a native
[GDExtension](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/what_is_gdextension.html).
There is no separate `godot_mujoco_bridge` library to call through and **no
manual MuJoCo install** — the MuJoCo runtime is downloaded and bundled
automatically by the build, and the extension is loaded directly by the engine.

It targets the **standard Godot build** (no .NET/Mono requirement) and exposes a
native `MjWorld` node usable from GDScript *and* C#.

> Scope: **desktop** (Linux / Windows / macOS). Mobile (iOS/Android) is a
> documented follow-up — see [Mobile](#mobile-ios--android) below.

## Why this instead of the C bridge?

| | C bridge + C# P/Invoke (repo root) | This GDExtension |
| --- | --- | --- |
| Interop | managed ↔ native every call | native `mj_step` in-process, zero managed boundary |
| Runtime deps | Godot .NET build + separate MuJoCo install | standard Godot build, MuJoCo bundled |
| Usable from | C# | GDScript **and** C# |
| Manual steps | set `LD_LIBRARY_PATH`, copy libs | none — build bundles everything |

## Requirements

- CMake ≥ 3.20 and a C++17 compiler (`g++`/`clang++`/MSVC).
- Network access on first configure (to fetch MuJoCo + godot-cpp).
- A standard Godot 4.6 binary to run the demo.

Nothing else — MuJoCo is fetched automatically.

## Build

```bash
cmake -S gdextension -B gdextension/build
cmake --build gdextension/build -j
```

This will:

1. Download the pinned prebuilt **MuJoCo** release (default `3.4.0`) for your
   platform and verify its checksum.
2. Fetch and build **godot-cpp** (`godot-4.5-stable`, forward-compatible with
   Godot 4.6).
3. Build the extension and **stage everything** into
   `gdextension/demo/addons/godot_mujoco/bin/`:
   - `libgodot_mujoco.<platform>.<target>.<arch>.so|.dylib|.dll`
   - the MuJoCo runtime (`libmujoco.so.3.4.0`, etc.)

The extension's RPATH is set to `$ORIGIN` / `@loader_path`, so the bundled
MuJoCo runtime resolves next to it with **no** `LD_LIBRARY_PATH` /
`DYLD_LIBRARY_PATH` / `PATH` changes.

### Useful CMake options

- `-DMUJOCO_VERSION=3.4.0` — MuJoCo release to fetch.
- `-DGODOT_CPP_TAG=godot-4.5-stable` — godot-cpp tag to build against.
- `-DGMJ_ADDON_BIN=/path/to/bin` — where to stage the built binaries.

## Run

Visual demo (a MuJoCo-simulated pendulum driven every physics tick):

```bash
godot --path gdextension/demo
```

Headless end-to-end proof (loads the model, steps the sim, prints joint state,
exits non-zero on failure — suitable for CI):

```bash
godot --headless --path gdextension/demo res://HeadlessTest.tscn
```

## Using `MjWorld` from GDScript

```gdscript
var world := MjWorld.new()
add_child(world)
world.load_model("res://models/pendulum.xml")

var motor := world.actuator_id("hinge_motor")
world.set_ctrl(motor, 0.15)
world.step(10)                     # advance 10 MuJoCo steps
print(world.get_qpos())            # PackedFloat64Array of generalized coords
print(world.body_world_position(world.body_id("pendulum")))
```

Or with **zero code**: add an `MjWorld` node, set its `model_path` and enable
`auto_step` in the inspector.

### `MjWorld` API

- Lifecycle: `load_model(path)`, `free_model()`, `is_ready()`, `reset()`,
  `step(n=1)`, `forward()`
- Dimensions: `get_nq()`, `get_nv()`, `get_nu()`, `get_nbody()`
- Lookup: `body_id/joint_id/actuator_id(name)`, `body_name/joint_name/actuator_name(id)`
- State: `get_ctrl(i)/set_ctrl(i,v)`, `get_qpos()/set_qpos()`, `get_qvel()/set_qvel()`,
  `get_ctrl_array()/set_ctrl_array()`
- Kinematics: `body_world_position(body_index)` → `Vector3`
- Diagnostics: `get_mujoco_version()`, `get_last_error()`
- Properties: `model_path`, `steps_per_tick`, `auto_step`

> MuJoCo is Z-up; Godot is Y-up. `body_world_position` returns raw MuJoCo
> coordinates — map axes in your scene as needed (the demo maps hinge angle to a
> Z rotation).

## Mobile (iOS / Android)

Not included in this branch. MuJoCo ships **no official mobile prebuilts**, so
mobile requires cross-compiling the MuJoCo simulation core from source (Android
NDK; iOS toolchain) and linking it into a per-ABI GDExtension build (Android
`.so` per ABI; iOS static library / XCFramework). The simulation core has no
GPU/OpenGL dependency, so this is feasible as a follow-up.
