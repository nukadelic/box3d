# Joint max spring torque

Optional torque clamp on the revolute and spherical joint springs, so a position spring behaves like a
torque-limited PD. Default 0 means unclamped and reproduces the old behavior bit for bit. Before this, only
the motor joint had a spring clamp; revolute and spherical could cap torque only through the velocity motor
(`maxMotorTorque`).

## Definition field

Source: `include/box3d/types.h`, `src/joint.c`, `src/joint.h`.

| Field | Default | Meaning |
|---|---|---|
| `maxSpringTorque` | 0 | Maximum spring torque in N·m, must be >= 0 (negative values are clamped to 0). 0 = unclamped |

Present on `b3RevoluteJointDef` and `b3SphericalJointDef`. It is copied into `b3RevoluteJoint` /
`b3SphericalJoint` when the joint is created.

## API

Source: `include/box3d/box3d.h`, `src/revolute_joint.c`, `src/spherical_joint.c`.

- `b3RevoluteJoint_SetMaxSpringTorque( jointId, maxTorque )` / `b3RevoluteJoint_GetMaxSpringTorque( jointId )`
- `b3SphericalJoint_SetMaxSpringTorque( jointId, maxTorque )` / `b3SphericalJoint_GetMaxSpringTorque( jointId )`

Setters are recorded with `B3_REC` and clamp negative input to 0.

## Solver

The limit is a per-substep impulse, as in the motor joint: `maxImpulse = context->h * maxSpringTorque`. It is
applied when `maxSpringTorque > 0` to the accumulated spring impulse after the usual soft-constraint update.

- **Revolute** (`src/revolute_joint.c`): the scalar `springImpulse` is clamped to `[-maxImpulse, maxImpulse]`
  and `deltaImpulse` is rewritten to `clamped - old` so the applied impulse matches.
- **Spherical** (`src/spherical_joint.c`): `springImpulse` is a vector, clamped **per axis** in world space.
  Magnitude clamping (what `b3MotorJoint` does) is a possible later variant. The applied impulse becomes
  `clamped - old`.

The delta is rewritten only when the clamp actually changes the impulse. Recomputing `(old + delta) - old`
always would differ from `delta` by float rounding and break bit-identity for a limit that never engages.

## Recording

Source: `src/recording.c`, `src/replay.c`, `src/recording_ops.inl`, `src/recording.h`.

- Both definitions serialize `maxSpringTorque` right after `dampingRatio`.
- New ops: `0xF3` `RevoluteJointSetMaxSpringTorque`, `0xF4` `SphericalJointSetMaxSpringTorque`.
- `B3_REC_VERSION_MAJOR` bumped 6 -> 7 because the definition layout changed (replay rejects only on a major
  mismatch, so old files would otherwise misparse).
- `_Static_assert` size of `b3RevoluteJointDef` went 152 -> 160. `b3SphericalJointDef` stays 184 (the float fits
  in existing padding).

## Tests

`TestSpringTorqueClamp` in `test/test_joint.c`, for both joint types:

- a small clamp is never exceeded by the reported constraint torque, and it does limit the spring
- the setter and the definition field give identical results
- a clamp of 0 and a clamp too large to engage (1e9) are bit-identical to the old behavior
- getters, including negative input clamped to 0

Caveat: upstream `b3GetRevoluteJointTorque` counts the axial impulse twice (via `rotationAxisZ` and `axis`), so
it reports 2x the real revolute spring torque. The solver clamp is correct; the test scales by 2 for revolute.
If upstream fixes it, update that factor.
