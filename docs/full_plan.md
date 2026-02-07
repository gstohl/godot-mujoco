# Godot MuJoCo Full Implementation Plan

## Mission

Deliver Unity-plugin-level MuJoCo usability in Godot 4 with a stable native bridge, runtime binding system, editor tooling, and release-grade packaging.

## Principles

- Keep C ABI stable and explicit.
- Minimize per-frame interop overhead with batch APIs.
- Prefer deterministic runtime behavior over hidden magic.
- Build every feature with testability and CI validation.

## Phase Plan

### Phase 0: Foundation Contract

Scope:
- Freeze and version C ABI.
- Document ownership, lifetimes, and thread assumptions.
- Define compatibility policy for function additions.

Tickets:
- P0-1: Add `gmj_api_version()`.
- P0-2: Publish API contract table.
- P0-3: Add changelog convention for ABI updates.

Acceptance:
- Contract docs cover every exported function and error code.

### Phase 1: Bridge Capability Expansion

Scope:
- Add name/id lookups and batch state IO APIs.
- Expose warning inspection and richer diagnostics.
- Keep fallback mode behavior consistent when MuJoCo is unavailable.

Tickets:
- P1-1: Add `gmj_name2id()` and `gmj_id2name()` wrappers.
- P1-2: Add array APIs for qpos/qvel/ctrl read/write slices.
- P1-3: Add warning counters API.
- P1-4: Expand C# interop declarations for new APIs.

Acceptance:
- Runtime binding can resolve IDs once and sync in batches.

### Phase 2: Godot Runtime Binding Layer

Scope:
- Introduce reusable runtime classes instead of one demo script.
- Resolve model names to IDs once at startup.
- Separate simulation stepping from node synchronization.

Tickets:
- P2-1: `MjSceneRuntime` orchestration class.
- P2-2: `MjJointBinding` and `MjActuatorBinding` classes.
- P2-3: Batched per-tick sync path.
- P2-4: Runtime validation and structured errors.

Acceptance:
- Complex sample scene runs without per-frame name lookups.

### Phase 3: Editor Import + Authoring

Scope:
- Add Godot EditorPlugin and MJCF importer.
- Generate initial scene/binding resources from MJCF.
- Support safe reimport with conflict reporting.

Tickets:
- P3-1: Base importer for `.xml/.mjcf`.
- P3-2: Generated Godot scene and metadata resource.
- P3-3: Reimport diff and rename handling.

Acceptance:
- One-click import from MJCF to runnable Godot scene.

### Phase 4: Runtime Resilience + Determinism

Scope:
- Add state snapshot/restore and rebuild flow.
- Add deterministic stepping controls and warning policies.

Tickets:
- P4-1: Snapshot/restore for qpos/qvel/ctrl.
- P4-2: Scene rebuild with state preservation.
- P4-3: Warning throttling and classification.

Acceptance:
- Rebuild preserves simulation continuity inside tolerance.

### Phase 5: Performance

Scope:
- Reduce marshaling cost via reusable pinned buffers.
- Add simple telemetry for step/sync budget visibility.

Tickets:
- P5-1: Pooled managed buffers.
- P5-2: Batch call path benchmark.
- P5-3: Runtime telemetry panel.

Acceptance:
- Stable frame budget on reference models.

### Phase 6: Packaging and Distribution

Scope:
- Deliver add-on package layout and release automation.
- Build artifacts for macOS/Linux/Windows.

Tickets:
- P6-1: Add-on folder structure in `addons/godot_mujoco`.
- P6-2: Multi-platform binary artifact workflow.
- P6-3: Versioned release + checksums.

Acceptance:
- Fresh install from release assets runs sample scene.

### Phase 7: Test Strategy

Scope:
- Add C tests for ABI behavior.
- Add Godot runtime integration tests.
- Add importer regression fixtures.

Tickets:
- P7-1: Native bridge unit test harness.
- P7-2: Headless Godot runtime test scene.
- P7-3: Import/reimport golden fixtures.

Acceptance:
- CI blocks regression on bridge, runtime, and importer paths.

## Milestones

- M1: End of Phase 2 (runtime parity baseline).
- M2: End of Phase 3 (editor pipeline baseline).
- M3: End of Phase 6 (release-grade distribution).

## Immediate Sprint (current)

1. Land Phase 1 ticket P1-1/P1-2 API additions.
2. Wire C# interop declarations for new APIs.
3. Add runtime sample usage for ID resolve + batch sync.
4. Verify with CMake build and diagnostics.
