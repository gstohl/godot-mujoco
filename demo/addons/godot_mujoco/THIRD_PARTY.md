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

## From-source / mobile `libmujoco` dependencies

Android and iOS (and `-DGMJ_BUILD_MUJOCO_FROM_SOURCE=ON`) compile MuJoCo's
simulation core from source. The resulting `libmujoco` statically includes
these upstream libraries — each keeps its own license:

| Library | Upstream | License |
| --- | --- | --- |
| qhull | https://github.com/qhull/qhull | Qhull License (BSD-style) |
| lodepng | https://github.com/lvandeve/lodepng | zlib |
| tinyxml2 | https://github.com/leethomason/tinyxml2 | zlib |
| libccd | https://github.com/danfis/libccd | BSD-3-Clause |
| TinyObjLoader | https://github.com/tinyobjloader/tinyobjloader | MIT |
| marchingcubecpp | (vendored by MuJoCo) | MIT |

See each project's repository for full license texts. This addon itself is
distributed under the MIT license (see the repository `LICENSE`).
