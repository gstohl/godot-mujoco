#!/usr/bin/env python3
from __future__ import annotations

import platform
import sys
from pathlib import Path


def expected_files(project_bin: Path) -> list[Path]:
    system = platform.system()
    if system == "Darwin":
        return [
            project_bin / "libgodot_mujoco_bridge.dylib",
            project_bin / "mujoco.framework" / "Versions" / "A" / "libmujoco.3.4.0.dylib",
        ]
    if system == "Linux":
        return [
            project_bin / "libgodot_mujoco_bridge.so",
            project_bin / "libmujoco.so",
        ]
    if system == "Windows":
        return [
            project_bin / "godot_mujoco_bridge.dll",
            project_bin / "mujoco.dll",
        ]
    return []


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    project_names = ["example", "godot_demo"]
    missing: list[Path] = []

    for project in project_names:
        bin_dir = repo_root / project / "bin"
        for file_path in expected_files(bin_dir):
            if not file_path.exists():
                missing.append(file_path)

    if missing:
        print("Missing runtime bundle files:")
        for item in missing:
            print(f"  - {item}")
        return 1

    print("Runtime bundle check passed for example/ and godot_demo/.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
