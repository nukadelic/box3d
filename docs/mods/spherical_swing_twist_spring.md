# Spherical joint swing/twist spring

Optional spring on `b3SphericalJointDef` with separate stiffness for the swing part (rotation that moves the
twist axis) and the twist part (rotation about it). Off by default, which reproduces the single
target-rotation spring exactly.

## Definition fields

Source: `include/box3d/types.h`.

| Field | Default | Meaning |
|---|---|---|
| `enableSwingTwistSpring` | false | Use the split spring instead of `enableSpring` |
| `swingHertz`, `swingDampingRatio` | 0 | Swing spring stiffness and damping |
| `twistHertz`, `twistDampingRatio` | 0 | Twist spring stiffness and damping |

While enabled, `enableSpring`, `hertz`, `dampingRatio` and `maxSpringTorque` are ignored (the torque clamp is
not applied to the split spring). `targetRotation` is still the spring target.

## Runtime API

Source: `include/box3d/box3d.h`, `src/spherical_joint.c`. Setters are recorded.

- `b3SphericalJoint_EnableSwingTwistSpring` / `b3SphericalJoint_IsSwingTwistSpringEnabled`
- `b3SphericalJoint_Set/GetSwingSpringHertz`, `b3SphericalJoint_Set/GetSwingSpringDampingRatio`
- `b3SphericalJoint_Set/GetTwistSpringHertz`, `b3SphericalJoint_Set/GetTwistSpringDampingRatio`

Toggling the mode clears the accumulated spring impulses.

## Solver

Source: `src/spherical_joint.c` (blocks marked `nukabox3d`).

- Error: the same vector as the old spring, `b3DeltaQuatToRotation( relQ, targetRotation )` rotated to world.
- Twist axis: world z-axis of frame B at the target rotation (frame A z when the target is identity).
- Rows: three scalar soft-constraint rows solved sequentially. One along the twist axis uses the twist
  softness; two along an orthonormal basis of the perpendicular plane use the swing softness. Effective mass
  per row is `1 / (n·K·n)`, with the joint armature added.
- Softness comes from `b3MakeSoft` in `b3PrepareSphericalJoint`.
- Impulses: swing accumulates in `springImpulse`, twist in `twistSpringImpulse`. Both are warm started as world
  vectors and included in `b3GetSphericalJointTorque`.

## Recording

The def serialisation gains the five fields after `targetRotation` (def size 184 -> 208 bytes),
`B3_REC_VERSION_MAJOR` 8 -> 9, and ops 0xF5-0xF9 cover the setters.

## Tests

`TestSwingTwistSpring` in `test/test_joint.c`:

- Stiff twist and soft swing (and the reverse): the stiff part's residual is under a tenth of the soft part's.
- Decoupling: a pure twist error ignores `swingHertz`, a pure swing error ignores `twistHertz`.
- Flag off: bit-identical to the old spring, whatever the new fields hold.
- Equal hertz on both parts matches the single spring within 1e-3 (cube inertia is isotropic).
- Accessors round trip.
