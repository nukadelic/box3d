# Plan: expose nukabox3d's mods in the wasm bindings

> **Status: done.** A1-A5 landed in this tree (`wasm/src/bindings.cpp`, `wasm/src/facade.js`,
> `wasm/test/smoke.mjs`, docs). `dist/` is rebuilt; `test/smoke.mjs` passes for both flavours and the MT build was
> checked under Node with `workerCount=4`. `wasm/package.json` bumped to 0.1.2. The sections below are kept as the
> record of what was intended and why; §5's batch-ray design and §7's checks match what shipped.

Scope: `S:\Repo\Box3D\nukabox3d` only. Goal: the embind layer
(`wasm\src\bindings.cpp` + `wasm\src\facade.js`) covers every C function/field the
fork added, so a JS consumer can reach the mods without touching C. Nothing here
is about any specific downstream app.

Status today: only `b3MotorJoint_*MaxSpringTorque` from the mod set is bound.
See `nukabox3d\CONTEXT.md` ("Our changes") for the C-side description of each mod
and `docs\mods\*.md` for the user-facing docs.

## 0. Ground rules (from bindings.cpp's header comment)

- Flat & faithful: C structs → `value_object`, C functions → `function(...)`.
  Def-taking functions get a by-value lambda wrapper; keep the param names in the
  bound name string (`--emit-tsd` uses them).
- Math is plain arrays: `b3Vec3 = [x,y,z]`, `b3Quat = [x,y,z,w]`, out-param first,
  zero-alloc via the `out_function` DSL + `g_outRegistry`. Never add an allocating
  getter variant.
- Scalars/bools/ids: bind directly (getters are plain `function(...)`).
- Bulk results: packed-tier buffer + facade reader, stride declared once in
  `namespace layout` and published by `_layoutMeta()`; add a `static_assert`
  tripwire; the facade's field-read order must match the packer by hand.
- `box3d.d.ts` is generated (`--emit-tsd`), never hand-edited. `facade.generated.js`
  is generated from `src\facade.js` by `scripts\build.mjs` — don't commit it.

## 1. Gap inventory

| Mod | C API | Bound? | Needs |
|---|---|---|---|
| maxSpringTorque (revolute) | `b3RevoluteJointDef::maxSpringTorque` (types.h:~979); `b3RevoluteJoint_{Set,Get}MaxSpringTorque` (box3d.h:1683,1686) | no | def field + 2 fns |
| maxSpringTorque (spherical) | `b3SphericalJointDef::maxSpringTorque` (types.h:~1024); `b3SphericalJoint_{Set,Get}MaxSpringTorque` (box3d.h:1816,1819) | no | def field + 2 fns |
| armature | `b3JointDef::armature` (types.h:~716) | no | def field |
| swing/twist spring | `b3SphericalJointDef::{enableSwingTwistSpring,swingHertz,swingDampingRatio,twistHertz,twistDampingRatio}` (types.h:1034-1048); `b3SphericalJoint_{Enable,Is}SwingTwistSpring`, `{Set,Get}{Swing,Twist}Spring{Hertz,DampingRatio}` (box3d.h:1766-1793) | no | 5 def fields + 9 fns |
| joint-vs-contact tuning | `b3WorldDef::{jointHertz,jointDampingRatio,jointRelaxIterations,solveJointsAfterContacts}` (types.h:187-198) | no | 4 def fields (create-time only — no `b3World_Set*` exists) |
| batch ray cast | `b3World_CastRayClosestBatch(worldId, origins, translations, count, filter, results)` (box3d.h:116) | no | packed buffer + facade reader + 1 fn |

Reference for style already in the file: `b3MotorJointDef` + its Set/Get block
(bindings.cpp:1585-1596, 1694-1713) is the exact pattern to copy for the joint
parts; `b3WorldDef` is at 743-755, `b3JointDef` at 1489-1500,
`b3RevoluteJointDef` at 1517-1528, `b3SphericalJointDef` at 1570-1583, the
revolute setters start at 1744 and the spherical ones at 1774.

## 2. Step A1 — joint def fields

- `b3JointDef` (1489-1500): add `.field( "armature", &b3JointDef::armature )`
  after `constraintDampingRatio`.
- `b3RevoluteJointDef` (1517-1528): add `maxSpringTorque` after `dampingRatio`.
- `b3SphericalJointDef` (1570-1583): add `maxSpringTorque` after `dampingRatio`;
  add `enableSwingTwistSpring`, `swingHertz`, `swingDampingRatio`, `twistHertz`,
  `twistDampingRatio` after `targetRotation` (C field order; value_object field
  order is cosmetic, but match `types.h` to keep audits trivial).
- Field order/coverage is what the C `b3Default*Def()` returns, so nothing else
  changes: `b3Create*Joint(worldId, def)` already marshals the whole struct and
  picks up new fields automatically (no wrapper edit needed).

## 3. Step A2 — joint setters/getters

- Revolute, next to `SetSpringDampingRatio` (1744):
  `b3RevoluteJoint_SetMaxSpringTorque(jointId, maxTorque)`,
  `b3RevoluteJoint_GetMaxSpringTorque(jointId)`.
- Spherical, next to `EnableSpring`/`SetTargetRotation` (1774-1780):
  `b3SphericalJoint_SetMaxSpringTorque`, `b3SphericalJoint_GetMaxSpringTorque`,
  `b3SphericalJoint_EnableSwingTwistSpring`, `b3SphericalJoint_IsSwingTwistSpringEnabled`,
  `b3SphericalJoint_{Set,Get}SwingSpringHertz`,
  `b3SphericalJoint_{Set,Get}SwingSpringDampingRatio`,
  `b3SphericalJoint_{Set,Get}TwistSpringHertz`,
  `b3SphericalJoint_{Set,Get}TwistSpringDampingRatio`.
- All signatures are plain scalars/bools → straight `&b3…` function pointers, no
  lambda needed. Keep bound names exactly the C names (1:1 rule).

## 4. Step A3 — world-def tuning fields

- `b3WorldDef` (743-755): add `jointHertz`, `jointDampingRatio`,
  `jointRelaxIterations`, `solveJointsAfterContacts` after `contactSpeed`.
- These are create-time only (the C API has no `b3World_Set*` for them), so note
  that in the smoke test and in `CONTEXT.md`. Do **not** invent setters here; if a
  runtime path is needed later, it belongs in the C API first.

## 5. Step A4 — `b3World_CastRayClosestBatch`

One implementation note learned the hard way (now documented in `CONTEXT.md` and `docs/mods/batch_ray_cast.md`):
`b3Pos` is `b3Vec3` (3 floats) unless the library is built with `BOX3D_DOUBLE_PRECISION`, so the JS origins must be
converted element-wise into a `std::vector<b3Pos>`; `reinterpret_cast<const b3Pos*>(doubleArray)` silently produces
garbage origins (the cast traverses the tree, visits a leaf, and reports no hit).

Follow the packed-tier pattern (see `PlaneResult` packing at 1014-1029 and
`b3Shape_GetSensorData` at 1305-1320 for the two shapes used):

1. `namespace layout`: add
   `constexpr int rayI32 = idW + u64W + 1 + 1 + 1 + 1 + 1; // 10: shapeId, userMaterialId(lo,hi), triangleIndex, childIndex, nodeVisits, leafVisits, hit`
   `constexpr int rayF64 = 3 + 3 + 1;                        //  7: point, normal, fraction`
   and extend the `static_assert` tripwire (`rayI32 == 10 && rayF64 == 7`).
2. Add both strides to `_layoutMeta()` (line ~660).
3. New `RayResultBuffer` struct alongside `ContactsBuffer`: `std::vector<int32_t> i32;
   std::vector<double> f64; int count;`, method
   `cast(worldId, val origins, val translations, filter)` that
   - `convertJSArrayToNumberVector<double>( origins )` (3 f64 per ray — `b3Pos`),
   - `convertJSArrayToNumberVector<float>( translations )` (3 f32 per ray),
   - asserts `originF64.size()/3 == translationF32.size()/3` (throw a JS error via
     `val::throw_`/`throw std::runtime_error` — pick one and be consistent with the file),
   - resizes a scratch `std::vector<b3RayResult>`, calls `b3World_CastRayClosestBatch`,
     then packs into the two tiers (clear-but-keep-capacity like the other buffers).
   Expose `count()`, `i32Ptr()`, `f64Ptr()`; `class_<RayResultBuffer>("RayResultBuffer")`.
4. Binding entry: `function( "b3World_CastRayClosestBatch(worldId, origins, translations, count, filter)", ... )`
   — decides `count` from the arrays (ignore the C `count` param in the bound
   signature, like the file's other `val` array bindings do) so there is no way to
   pass mismatched `count`.
5. Facade (`src/facade.js`): `createRayResultBuffer()`, `destroyRayResultBuffer()`
   (if the `class_` needs explicit delete), `getNumRayResults(buf)`,
   `createRayResult()`, `getRayResultAt(out, buf, i)` reading
   `{shapeId:{index1,world0,generation}, userMaterialId (lo|hi<<32 via >>>0),
   triangleIndex, childIndex, nodeVisits, leafVisits, hit, point:[x,y,z],
   normal:[x,y,z], fraction}` — same field names as the existing `b3RayResult`
   value_object so the two paths return identical shapes.
6. Add the new names to the `Object.assign`/export block at the bottom of
   `facade.js` (~line 523-534).

Deferrable: if batch rays are not needed yet, skip A4 entirely. It is the only
item with a new buffer tier; A1-A3 are pure field/function additions.

## 6. Build, regenerate, verify

Env (Git Bash, from the build doc):

```bash
export EMSDK=W:/re_tools/emsdk \
       EMSDK_NODE=W:/re_tools/emsdk/node/24.19.0_64bit/node.exe \
       EMSDK_PYTHON=W:/re_tools/emsdk/python/3.13.3_64bit/python.exe
export PATH=/w/re_tools/emsdk/upstream/emscripten:/w/re_tools/ninja:$PATH
cd /s/Repo/Box3D/nukabox3d/wasm
```

- `node scripts/build.mjs` — must pass the emsdk 6.0.x gate (build.mjs:41).
  The `--emit-tsd` probe link at build.mjs:223 regenerates `dist\box3d.d.ts`, so
  the new fields/functions/types appear there automatically; check that they do.
- `node test/smoke.mjs` — plain + inline flavours.
- MT build (`dist\box3d.mt.*`) shares `bindings.cpp`; run one `workerCount=4`
  world manually under Node as the build doc's status note describes. No browser
  MT test is expected (needs COOP/COEP).
- Grep the generated `dist\box3d.d.ts` for each new name; a silent embind
  registration failure otherwise shows up only at runtime.

## 7. Step A5 — smoke coverage (`test/smoke.mjs`)

Add one function per mod group, all inside a world that already exists in the
file where possible (keeps runtime small):

- `modJointTuning(b3)`: create two dynamic bodies + a revolute joint with
  `maxSpringTorque = 5`, `hertz = 4`, `base.armature = 0.01`; assert the Set/Get
  round-trips (`b3RevoluteJoint_GetMaxSpringTorque` → 5) and that the body stops
  at a lower angle than the same setup with `maxSpringTorque = 0` (the clamp
  engages). Same for spherical, plus `EnableSwingTwistSpring` + swing/twist hertz
  round-trip through the getters.
- `modWorldTuning(b3)`: build a world with `jointHertz = 60`,
  `jointDampingRatio = 1`, `jointRelaxIterations = 2`,
  `solveJointsAfterContacts = true`; assert the world is valid and steps without
  NaN (read a body position), and that a joint resists a contact better than with
  defaults (or at least that the outcome differs).
- `modBatchRay(b3)` (only with A4): 3 rays against a static box; assert
  `getNumRayResults(buf) === 3`, one hit / two misses with correct `fraction`
  ordering, and that `getRayResultAt` matches `b3World_CastRayClosest` for the
  same ray (same shapeId + point within 1e-4).
- Keep all new work in the shared `check()` path so both flavours run it.

## 8. Docs / metadata

- `nukabox3d\CONTEXT.md`: new "### wasm bindings" entry under "Our changes"
  listing the files touched (`wasm\src\bindings.cpp`, `wasm\src\facade.js`,
  `wasm\test\smoke.mjs`) and the rule "every new mod field/function must be added
  here", so a future upstream sync keeps the bindings in step.
- `docs\mods\*.md` (the four existing ones): one line each saying the wasm
  bindings expose it, naming the bound functions; for batch ray cast add
  `docs\mods\batch_ray_cast.md` (missing today) and document the buffer API there.
- `README.md` "Added" list: no change needed (features already listed) unless the
  wasm surface is mentioned; if so, add a note.
- `wasm\package.json`: bump `version` (0.1.1 → 0.1.2) once the fields ship;
  `private: true` is fine, bump is only for reproducibility of vendored copies.
- Build doc `S:\Repo\Box3D\nukabox3d-wasm-build.md`: update the "Changes relative
  to upstream box3d.js" section with a line noting the mods are now bound
  (it currently only lists renames/removals).

## 9. Order & effort

A1 (def fields) → A2 (setters) → A3 (world def) → build + d.ts check → A5 smoke →
A4 batch ray (optional, largest) → docs/version.

Rough: A1+A2+A3 ≈ an hour of edits, A5 ≈ an hour (needs a scene that visibly
changes with the clamp), A4 ≈ half a day (new tier, facade readers, d.ts check).
Verify each grouping with a build + smoke run before moving on; a broken
value_object field fails at link/registration time, not at JS call time.

## 10. Out of scope

- No new C API (e.g. `b3World_SetJointHertz`); if a runtime knob is wanted, add
  it in `src\` + `include\box3d\box3d.h` first, with a test, then bind it.
- No wide/SIMD or recording-format changes — the wasm layer only exposes what C
  already has.
- No OO/adapter layer here; consumers can wrap the flat API themselves.
- `b3World_CastRayClosestBatch` has no recording support on the C side; don't
  imply otherwise in the .d.ts comments or docs.
