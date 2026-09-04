# godot-mujoco

**Full [MuJoCo](https://mujoco.org) physics inside Godot 4, as a native GDExtension.**

MuJoCo runs *in-engine* — there is **no bridge library** to call through and
**no manual MuJoCo install**. The MuJoCo runtime is downloaded and bundled
automatically by the build, and the extension is loaded directly by the engine.
It targets the **standard Godot build** (no .NET/Mono requirement) and exposes a
native `MjWorld` node usable from **GDScript** (and from C# via the engine's
`ClassDB` — see [Using from C#](#using-from-c)).

> Scope: **desktop first**, with **mobile compile proof**. Linux x86_64 / arm64
> and Windows are built in CI from official MuJoCo prebuilts. Android arm64 and
> iOS arm64 are cross-compiled from MuJoCo **source** in CI (no official mobile
> prebuilts exist). Device/emulator runtime is a follow-up — see
> [Mobile](#mobile-ios--android).

![Chaotic double pendulum simulated by MuJoCo inside Godot](docs/chaos_double_pendulum.gif)

*A chaotic double pendulum: MuJoCo integrates the dynamics entirely in-engine and
the joint state drives Godot nodes. See [`ChaosPendulum.tscn`](demo/ChaosPendulum.tscn).*

![Visual debug overlay: contact points, force arrows, body frames and center of mass](docs/visual_debug.gif)

*Built-in visual debug (`MjDebugDraw`): contact points, contact-force arrows, body
frames, joint axes and center of mass. See [`VisualDebug.tscn`](demo/VisualDebug.tscn).*

## Highlights

- One native node, `MjWorld`, owns a full MuJoCo `mjModel` + `mjData`.
- Zero-step packaging: `cmake --build` fetches + bundles MuJoCo and resolves it
  via `$ORIGIN` / `@loader_path` — no `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH` /
  `PATH` changes.
- Usable from GDScript (and C# via `ClassDB`); drop a node in a scene and
  simulate with zero code via the `auto_step` property.
- Batch state I/O, full body pose (position + orientation), sensors, and clock
  access for low-overhead per-tick integration.
- Built-in **visual debug** overlay (`MjDebugDraw`): contact points, force
  arrows, body frames, joint axes and center of mass.

## Requirements

- CMake ≥ 3.24 and a C++17 compiler (`g++` / `clang++` / MSVC).
- Network access on first configure (to fetch MuJoCo + godot-cpp).
- A standard Godot 4.7 binary to run the demo.

Nothing else — MuJoCo is fetched automatically.

## Build

```bash
cmake -S . -B build                 # editor / debug (default GODOTCPP_TARGET=template_debug)
cmake --build build -j
```

This will:

1. Download the pinned prebuilt **MuJoCo** release (default `3.12.0`) and verify
   its SHA-256. All shipped archives (Linux x86_64/aarch64, Windows x86_64) are
   pinned; fetching an unpinned archive fails unless `-DGMJ_ALLOW_UNVERIFIED=ON`.
2. Fetch and build **godot-cpp** (`godot-4.5-stable`, forward-compatible with
   Godot 4.6/4.7).
3. Build the extension and **stage everything** into
   `demo/addons/godot_mujoco/bin/`:
   - `libgodot_mujoco.<platform>.<target>.<arch>.so|.dylib|.dll`
   - the MuJoCo runtime (`libmujoco.so.3.12.0`, etc.)

For an **exported game** you also need the release variant (the `.gdextension`
maps release feature tags to `*.template_release.*`):

```bash
cmake -S . -B build-release -DGODOTCPP_TARGET=template_release
cmake --build build-release -j
```

> On distros whose default `cc`/`c++` points at Clang and can't find `libstdc++`,
> configure with `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`.

### Useful CMake options

- `-DMUJOCO_VERSION=3.12.0` — MuJoCo release to fetch.
- `-DGODOT_CPP_TAG=godot-4.5-stable` — godot-cpp tag to build against.
- `-DGODOTCPP_TARGET=template_debug|template_release` — build variant.
- `-DGMJ_ADDON_BIN=/path/to/bin` — where to stage the built binaries.
- `-DGMJ_MUJOCO_ROOT=/path/to/mujoco` — use a pre-extracted MuJoCo instead of
  fetching (required on macOS; also an offline / custom-version escape hatch).
- `-DGMJ_ALLOW_UNVERIFIED=ON` — permit fetching an archive with no pinned hash.
- `-DGMJ_BUILD_MUJOCO_FROM_SOURCE=ON` — compile MuJoCo from the pinned source
  tarball (automatic on Android/iOS; useful to test that path on desktop).
- `-DGMJ_BUILD_CORE_SMOKE=ON` — build the standalone `gmj_core_smoke` executable
  (automatic on Android/iOS).

## Run

Visual demo (a MuJoCo-simulated pendulum driven every physics tick):

```bash
godot --path demo
```

![MuJoCo pendulum in Godot](docs/pendulum.gif)

Headless end-to-end proof (loads the model, steps the sim, prints joint state,
sensors and body pose, exits non-zero on failure — suitable for CI):

```bash
godot --headless --path demo res://HeadlessTest.tscn
```

Chaotic **double pendulum** — a visual demo and a headless test that shows
sensitive dependence on initial conditions (a 1e-5 rad perturbation diverges to
O(1)), which also exercises multi-instance isolation:

```bash
godot --path demo res://ChaosPendulum.tscn              # visual
godot --headless --path demo res://ChaosPendulumTest.tscn  # numerical chaos test
```

## Using `MjWorld` from GDScript

```gdscript
var world := MjWorld.new()
add_child(world)
world.load_model("res://models/pendulum.xml")

var motor := world.actuator_id("hinge_motor")
world.set_ctrl(motor, 0.15)
world.step(10)                                  # advance 10 MuJoCo steps

var body := world.body_id("pendulum")
print(world.body_world_transform(body))         # full pose (Transform3D)
print(world.get_qpos())                         # PackedFloat64Array
print(world.get_sensordata())                   # all sensor readings
```

Or with **zero code**: add an `MjWorld` node, set its `model_path` and enable
`auto_step` in the inspector.

### `MjWorld` API

- Lifecycle: `load_model(path)`, `free_model()`, `is_ready()`, `reset()`,
  `step(n=1)`, `forward()`
- Dimensions: `get_nq()`, `get_nv()`, `get_nu()`, `get_nbody()`, `get_njnt()`,
  `get_nsensor()`
- Clock: `get_time()`, `get_timestep()`, `set_timestep(dt)`
- Lookup: `body_id/joint_id/actuator_id/sensor_id(name)` and the matching
  `*_name(id)` accessors
- State: `get_ctrl(i)/set_ctrl(i,v)`, `get_qpos()/set_qpos()`,
  `get_qvel()/set_qvel()`, `get_ctrl_array()/set_ctrl_array()`
- Sensors: `get_sensordata()`, `get_sensor(sensor_index)`
- Kinematics: `body_world_position(i)` → `Vector3`,
  `body_world_quaternion(i)` → `Quaternion`, `body_world_transform(i)` →
  `Transform3D`
- Diagnostics: `get_mujoco_version()`, `get_last_error()`
- Debug: `get_ncon()` (active contacts), `get_kinetic_energy()`,
  `get_potential_energy()`, `get_warnings()`, `has_warnings()`,
  `get_debug_info()` (aggregate `Dictionary` snapshot)
- Visual debug: `get_contacts()`, `get_center_of_mass()`, `get_joint_anchor(i)`,
  `get_joint_axis(i)`, plus the `MjDebugDraw` overlay node
- Properties: `model_path`, `steps_per_tick`, `auto_step`

> MuJoCo is Z-up; Godot is Y-up. Kinematics queries return raw MuJoCo world-frame
> values — remap axes in your scene as needed. `mj_step` evaluates sensors before
> integrating, so call `forward()` if you need sensor/derived data consistent
> with the post-step state.

### Debugging

`get_debug_info()` returns a snapshot you can log each frame:

```gdscript
print(world.get_debug_info())
# { "mujoco_version": "3.12.0", "time": 1.5, "timestep": 0.01,
#   "nq": 1, "nv": 1, "nu": 1, "nbody": 2, "njnt": 1, "nsensor": 2,
#   "ncon": 0, "energy": { "potential": 24.3, "kinetic": 902.5, "total": 926.8 },
#   "warnings": {  } }

if world.has_warnings():
	push_warning("MuJoCo warnings: %s" % world.get_warnings())
```

Energy is a handy correctness signal — a passive system should approximately
conserve `get_kinetic_energy() + get_potential_energy()`. `get_warnings()`
surfaces MuJoCo's solver warnings (e.g. `BADQACC`, `CONTACTFULL`) by name, and
`get_ncon()` reports the number of active contacts.

### Visual debug

`MjDebugDraw` is a drop-in overlay node that renders MuJoCo's debug geometry —
**contact points, contact-force arrows, body frames, joint axes and the center
of mass** — as lines on top of your scene:

```gdscript
var dbg := MjDebugDraw.new()
add_child(dbg)
dbg.world = world              # point it at your MjWorld
dbg.show_contact_forces = true # toggles: frames / joints / com / contacts / forces
```

The underlying geometry is also exposed directly: `get_contacts()` (each with
`pos`, world-space `normal`, `force`, and penetration `distance`),
`get_center_of_mass()`, `get_joint_anchor(i)` and `get_joint_axis(i)`. See
[`VisualDebug.tscn`](demo/VisualDebug.tscn) for a runnable example — falling
balls whose resting contact forces sum to their weight.

## Using from C#

`MjWorld` is registered engine-wide, so it is reachable from C# (`Godot.NET`)
projects through the engine's `ClassDB` — there is no generated managed type, so
you construct and call it dynamically:

```csharp
var world = (Node)ClassDB.Instantiate("MjWorld");
AddChild(world);
world.Call("load_model", "res://models/pendulum.xml");
world.Call("step", 10);
```

No P/Invoke and no separate bridge assembly. (A typed C# wrapper is not provided;
`new MjWorld()` won't compile because GDExtension classes aren't exposed as C#
types.)

## Project layout

```
CMakeLists.txt                     # fetches MuJoCo + godot-cpp, builds + bundles
src/                               # C++ GDExtension (MjWorld, registration)
demo/                              # standard Godot project (GDScript)
  addons/godot_mujoco/*.gdextension
  Main.tscn / HeadlessTest.tscn
  models/pendulum.xml
scripts/cloud_setup.sh             # idempotent CI / Cloud-Agent bring-up
```

## MuJoCo feature coverage

This binding exposes a **complete runtime-control surface** — enough to load
models, step the simulation, read/write full state, query poses and sensors, and
debug — but it does **not** wrap all of MuJoCo's very large C API. The native
`libmujoco` runtime is fully present; only the Godot-facing wrappers are curated.

**Implemented**

- Model/data lifecycle, `step`, `forward`, `reset`, multi-instance isolation
- Dimensions + name↔id lookup (bodies, joints, actuators, sensors)
- Full state I/O: `qpos`, `qvel`, `ctrl` (scalar + batch)
- Kinematics: body position, orientation, full `Transform3D`
- Sensors: all sensor data + per-sensor slices
- Clock: time / timestep
- Debug: energy, solver warnings, aggregate snapshot
- Visual debug: contact points / normals / forces, center of mass, joint axes,
  and the `MjDebugDraw` overlay

**Not yet wrapped** (native calls exist in `libmujoco`; wrappers can be added on
demand)

- External forces / applied torques (`xfrc_applied`, `qfrc_applied`), actuator
  force introspection
- Jacobians and inverse dynamics (`mj_jac*`, `mj_inverse`)
- Rich model introspection (geoms, sites, cameras, masses, joint ranges, gears)
- State save/restore (`mj_getState` / `mj_setState`), keyframes, mocap bodies
- Ray casting / collision queries (`mj_ray`), option/flag configuration
- Model editing (`mjSpec`), plugins, deformables/flex
- On-screen MuJoCo rendering (`mjr_*`) — intentionally omitted; Godot renders

Adding a wrapper is typically a few lines in `src/mj_world.cpp` plus a
`ClassDB::bind_method` entry. Open an issue for the calls you need.

## Loading models in exported games

`load_model()` reads through Godot's filesystem (`FileAccess`) and hands the
bytes to MuJoCo via its in-memory VFS, so `res://` models work in an **exported**
game, not just the editor. It also scans the model's directory and adds sibling
files to the VFS, so **multi-file MJCF** — `<include>` files and mesh/texture
assets (e.g. `meshdir`) — resolves too. See
[`demo/models/composite/`](demo/models/composite) for an `<include>` + mesh
example.

- For an exported game, add your `.xml`/`.mjcf` **and referenced assets** (meshes,
  textures, included files) to the export's non-resource filter so they're packed
  and readable by `FileAccess`.
- To load without any files, use `load_model_from_string(xml_text)`.

## Platforms

- **Linux x86_64** — built and tested in CI (debug + release, headless smoke).
- **Linux aarch64 / Windows x86_64** — official archives are checksum-pinned;
  CI builds both (arm64 also runs the headless smoke).
- **macOS** — the official release is a `.dmg` CMake can't extract; download and
  mount it, then build with `-DGMJ_MUJOCO_ROOT=/path/to/mujoco-3.12.0`. (The
  `.gdextension` currently expects a `.framework`; adjust it to the produced
  `.dylib` name if you package for macOS.)
- **Android arm64 / iOS arm64** — from-source cross-compile, proven in CI as
  binaries (see [Mobile](#mobile-ios--android)).

## Mobile (iOS / Android)

MuJoCo ships **no official mobile prebuilts**, so mobile builds compile the
simulation core **from source** (no viewer / OpenGL / GLFW) and link it into the
GDExtension. The `.gdextension` manifest already lists `android.*` and `ios.*`
library + `libmujoco` dependency entries.

What CI proves today (compile + linkage, not a phone/emulator run):

- **Android arm64-v8a** (`ubuntu-latest` + NDK r27c): `libgodot_mujoco.android.*.arm64.so`
  is AArch64, has an Android identification note, links **bionic** (`libc.so`,
  not glibc `libc.so.6`), `NEEDED`s `libmujoco`, and exports
  `godot_mujoco_library_init`. A standalone `gmj_core_smoke` executable is
  staged next to it.
- **iOS arm64** (`macos-latest` + Xcode): `libgodot_mujoco.ios.*` and
  `libmujoco` are arm64 **iphoneos** binaries (not macOS).

### Android (Linux or your Mac)

```bash
# Linux: the script downloads pinned NDK r27c if ANDROID_NDK is unset.
# macOS: install the NDK via Android Studio / sdkmanager, then:
#   export ANDROID_NDK=$HOME/Library/Android/sdk/ndk/<version>
bash scripts/build_android.sh
bash scripts/verify_android_binaries.sh
```

Runtime on a device or emulator (MuJoCo core only — no Godot):

```bash
adb push demo/addons/godot_mujoco/bin/gmj_core_smoke /data/local/tmp/
adb push demo/addons/godot_mujoco/bin/libmujoco.so /data/local/tmp/
adb push demo/addons/godot_mujoco/bin/libc++_shared.so /data/local/tmp/
adb shell "cd /data/local/tmp && LD_LIBRARY_PATH=. ./gmj_core_smoke"
# expect: CORE SMOKE: PASS
```

A full Godot Android **export** still needs the engine's Android export
templates and Gradle; that is not wired here.

### iOS (macOS + Xcode)

This Cloud Agent host is Linux, so iOS is proven on GitHub's `macos-latest`
runner. On your Mac:

```bash
bash scripts/build_ios.sh
bash scripts/verify_ios_binaries.sh
```

Requires the iPhoneOS SDK (`xcrun --sdk iphoneos --show-sdk-path`) and
**iOS 14+** (MuJoCo's thread pool uses `std::condition_variable` APIs that
Apple gated at 14.0). The extension is a `.dylib` plus `libmujoco.dylib`;
codesign / XCFramework packaging for an App Store export is still a follow-up.

## Keeping dependencies up to date

The three pinned versions (MuJoCo, Godot, godot-cpp) are the main thing to track.

- **In-repo check**: [`.github/workflows/check-upstream-versions.yml`](.github/workflows/check-upstream-versions.yml)
  runs weekly (and on demand) and opens/refreshes a tracking issue when a pin is
  behind. Run it locally any time with `bash scripts/check_upstream_versions.sh`.
- **Cursor Automation** (optional, for auto-bump PRs): Cursor has no native
  "upstream released" trigger, so create a **Scheduled** automation at
  [cursor.com/automations](https://cursor.com/automations) (or via the `/automate`
  skill), scoped to this repo + `main`, with the Pull Request tool enabled, e.g.:
  > "Run `bash scripts/check_upstream_versions.sh`. If any of MuJoCo, Godot, or
  > godot-cpp is behind, bump the pins (see the script's instructions), rebuild,
  > and open a PR. Do nothing if everything is current."
  For instant reaction instead of weekly polling, add a **Webhook** trigger to
  that automation and `POST` to it from the workflow above.

## Roadmap

Focused on being a solid MuJoCo integration for Godot:

- macOS `.dmg` extraction wired into the build (today: `-DGMJ_MUJOCO_ROOT`).
- Optional `Node3D` base so the sim can compose with a scene-graph transform.
- Android/iOS **device runtime** (Godot export + `adb`/`xcrun` smoke on an
  emulator or phone). Compile proof is already in CI — see [Mobile](#mobile-ios--android).
