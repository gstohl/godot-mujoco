# Third-party components bundled with this addon

This addon builds on and redistributes the following components. Their licenses
apply to the bundled binaries.

## MuJoCo
- Upstream: https://github.com/google-deepmind/mujoco
- License: Apache License 2.0
- Redistributed as the prebuilt runtime bundled next to the extension
  (`libmujoco.*` / `mujoco.dll`), fetched at build time.

## godot-cpp
- Upstream: https://github.com/godotengine/godot-cpp
- License: MIT
- Statically linked into the GDExtension library.

See each project's repository for full license texts. This addon itself is
distributed under the MIT license (see the repository `LICENSE`).
