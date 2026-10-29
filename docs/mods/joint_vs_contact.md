# Joint-vs-contact tuning

Opt-in `b3WorldDef` settings that let joints resist contacts more strongly, for example a loaded foot of a
ragdoll driven by joint springs and motors. All defaults reproduce the previous behaviour exactly.

## Solver order per substep

Source: `src/solver.c`.

1. Integrate velocities.
2. Warm start: overflow joints, overflow contacts, then each color (joint blocks before contact blocks).
3. Solve with bias (1 iteration), same order.
4. Integrate positions.
5. Relax without bias (1 iteration), same order.
6. Restitution runs once after all substeps.

There is no global joint softness. Each joint has its own `constraintHertz` (default 60) and
`constraintDampingRatio` (default 2), set up in `b3PrepareJoint` (`src/joint.c`).

## Settings

| Field | Default | Meaning |
|---|---|---|
| `jointHertz` | 0 | Overrides the constraint hertz of all joints. 0 uses the per-joint value. |
| `jointDampingRatio` | 0 | Overrides the constraint damping ratio of all joints. 0 uses the per-joint value. |
| `jointRelaxIterations` | 0 | Extra joint-only relax passes per substep, after the normal relax. Clamped to 0..16. |
| `solveJointsAfterContacts` | false | One more biased joint pass after the contact solve, so joints get the last word. |

The extra passes are new joint-only solver stages (`b3_stageSolveJoints`, `b3_stageRelaxJoints`) that skip contact
blocks. Stage count and index arithmetic in `src/solver.c` is marked `nukabox3d`.

## Test

`TestJointResistsContact` in `test/test_joint.c`: a static hip, thigh and foot. A 400 N·m motor at 1 rad/s pushes
the foot into the ground and the contact pushes back. "Lift" is the foot height above the ground top after 120
steps of 1/60 s with 4 substeps (smaller means joints resisted the contact more). It asserts that explicit zero
settings are bit identical to the defaults and that the stiff settings give less lift, both combined and each alone.

| Setting | Lift (m) | Time per 1000 runs |
|---|---|---|
| Default | 0.0039 | ~660 ms |
| `jointHertz` 1000, `jointDampingRatio` 10 | 0.0033 | no change |
| `jointRelaxIterations` 4 | 0.0035 | ~+5% |
| `solveJointsAfterContacts` | 0.0025 | ~+4% |
| All three | 0.00009 | ~+27% |

Timing covers world create, stepping and destroy of a tiny scene, so it is only indicative; measure on a real
ragdoll. A spring that lifts the foot behaves the other way: stiffer joints follow the command and lift more.

## Limitations

- Not stored in world snapshots or recordings; replays use the defaults.
- Upstream merge risks: stage count/advance arithmetic in `src/solver.c`, `b3PrepareJoint`.

## Wasm bindings

`b3WorldDef::{jointHertz, jointDampingRatio, jointRelaxIterations, solveJointsAfterContacts}` are fields of the
bound `b3WorldDef` value object, so they are set before `b3CreateWorld` and cannot be changed afterwards (the C API
has no `b3World_Set*` for them). `wasm/test/smoke.mjs` (`modWorldTuning`) builds a world with all four set and
checks a jointed body resting on the ground stays finite and settled.
