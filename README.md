# Moded Box3D

> Disclaimer: the code in this repo is, or will be, written by AI.

## Added

- `b3World_CastRayClosestBatch`: closest-hit ray casts for an array of rays in one call, parallelized across the world's worker threads.
- `maxSpringTorque` on revolute and spherical joints (`b3RevoluteJointDef`/`b3SphericalJointDef`, plus `b3RevoluteJoint_SetMaxSpringTorque`/`Get...` and the spherical equivalents): clamps the spring torque (N·m, per axis for spherical) so a position spring acts as a torque-limited PD. Zero keeps the old unclamped behaviour.
- `armature` on `b3JointDef` (kg·m², default 0): extra rotor inertia for revolute and spherical joints. It scales the spring and motor effective mass by 1 / (1 + armature·k), so the joint accelerates the bodies less; limits and other joint types are unaffected. Zero keeps the old behaviour.
- Swing/twist spring on `b3SphericalJointDef` (`enableSwingTwistSpring`, `swingHertz`, `swingDampingRatio`, `twistHertz`, `twistDampingRatio`, plus `b3SphericalJoint_Enable/Is/Set/Get...SwingTwistSpring`, `...SwingSpring...`, `...TwistSpring...`): replaces the single target-rotation spring with separate stiffness for the swing part (moves the twist axis) and the twist part (rotation about it). Off by default, which keeps the old behaviour.
- Joint-vs-contact tuning on `b3WorldDef` (`jointHertz`, `jointDampingRatio`, `jointRelaxIterations`, `solveJointsAfterContacts`): world-wide override of the joint constraint softness, extra joint-only relax passes per substep, and a second joint solve after the contacts, so joints can resist contacts more strongly. All default to off, which keeps the old behaviour.
