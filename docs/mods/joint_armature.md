# Joint armature

Optional rotor inertia on `b3JointDef` for an active-ragdoll controller that needs joints to respond more
slowly than the attached bodies alone. Default 0 reproduces the old behavior bit for bit.

## Definition field

Source: `include/box3d/types.h`, `src/joint.c`.

| Field | Default | Meaning |
|---|---|---|
| `armature` | 0 | Rotor inertia in kg·m², must be >= 0 (negative values are clamped to 0) |

Only the revolute and spherical joints use it. All other joint types ignore it. It is copied into
`b3JointSim::armature` when the joint is created.

## Formula

The spring and motor rows normally use the angular effective mass `m = 1 / k`, with `k = invIA + invIB`
projected on the axis (spherical: `m = inverse( K )`, `K = invIA + invIB` as a 3x3). With armature `a`:

```
m' = 1 / ( k + a * k * k )              revolute
m' = inverse( K + a * K * K )           spherical
```

Equivalently `m' = m / ( 1 + a * k )`. Properties:

- `a = 0` gives `m' = m`; the code skips the computation, so results are bit-identical.
- `m'` is never larger than `m` and falls monotonically as `a` rises. Spring and motor impulses shrink, so the
  joint accelerates the bodies less.
- It stays stable because the velocity update still uses `k`.

Rejected alternatives: `1 / ( k + 1 / a )` is singular at `a = 0` and raises the mass as `a` grows, and
`m + a` overshoots (the velocity update uses `k`, so `m' * k > 1`) and makes the motor unstable.

Note this slows the joint's response. It is not the MuJoCo style of adding inertia to the joint space.

## Solver

Source: `src/revolute_joint.c`, `src/spherical_joint.c`, `src/joint.h`.

- Revolute: prepare computes `b3RevoluteJoint::axialDriveMass`. The spring and motor rows use it. Limit rows keep
  `axialMass`.
- Spherical: `rotationMass` is only used by the spring and motor, so prepare builds it from `K + a * K * K`.
  Swing and twist limit masses are unchanged.
- No wide/SIMD solver exists for these joint types, so there is no second path.

## Recording

Source: `src/recording.c`, `src/replay.c`, `src/recording.h`.

`armature` is written and read after `drawScale` in the shared joint def base. `B3_REC_VERSION_MAJOR` went
from 7 to 8, so older recordings no longer load. The def size asserts still hold because of padding.

## Test

`TestArmature` in `test/test_joint.c`, for both joint types:

- armature 0, unset and negative give bit-identical angular velocity and rotation on a motor driven pair.
- On a free two-body spring, relative angular speed after one step drops strictly for armature 0, 0.1, 1, 10.

The motor row is not directly checked for the drop (the test motor is torque-saturated).

## Upstream conflict risks

`b3JointDef` layout, recording version and joint base serialization, the spring and motor lines of
`b3SolveRevoluteJoint`, and the `rotationMass` computation in the spherical prepare.
