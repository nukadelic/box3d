# Batch ray cast

`b3World_CastRayClosestBatch` fires an array of closest-hit rays in one call. It is equivalent to calling
`b3World_CastRayClosest` once per ray, but pays the world-id lookup and the native/JS boundary cost once instead of
`count` times, and splits the work over the world's worker threads via `b3ParallelFor`.

```c
B3_API void b3World_CastRayClosestBatch( b3WorldId worldId, const b3Pos* origins, const b3Vec3* translations,
										int count, b3QueryFilter filter, b3RayResult* results );
```

Source: `include/box3d/box3d.h` (declaration), `src/physics_world.c` (`WorldRayBatchContext`, `b3RayBatchTask`,
`b3World_CastRayClosestBatch`). Behaviour mirrors `b3World_CastRayClosest`, including the initial-overlap rule
(a ray starting inside a shape reports no hit) — if upstream changes that function, port the change here.

## Notes

- `origins`, `translations` and `results` must each point to `count` elements; a non-positive `count` or any null
  pointer returns without touching the results.
- Each `results[i]` is zeroed and then filled by the same callback the single-ray path uses
  (`b3RayCastClosestFcn`), so `nodeVisits` / `leafVisits` are per ray.
- No recording support: unlike `b3World_CastRayClosest`, the batch call is not written to a recording, so a replay
  of a session that used it will not reproduce the queries.
- Read-only: like all box3d queries, the batch traverses the broadphase without locking, so it must not run
  concurrently with a step (the task system is the only concurrency here).

## Wasm bindings

Packed-tier access, mirroring the other bulk query paths (`wasm/src/bindings.cpp`, `wasm/src/facade.js`):

```js
const rb = b3.createRayResultBuffer();      // allocate once, owns wasm storage
const hit = b3.createRayResult();           // reusable out object

const origins = new Float64Array( [ 0, 5, 0, 1, 5, 0 ] );  // 3 numbers per ray
const translations = new Float32Array( [ 0, -10, 0, 0, -10, 0 ] );
b3.castRayClosestBatch( rb, world, origins, translations );          // or ...BatchFiltered(..., filter)
for ( let i = 0, n = b3.getNumRayResults( rb ); i < n; i++ )
{
	b3.getRayResultAt( hit, rb, i );        // out-arg first, fills in place
	hit.hit; hit.shapeId; hit.point; hit.normal; hit.fraction; hit.userMaterialId; // BigInt
}
b3.destroyRayResultBuffer( rb );             // free the wasm storage (also destroy the world)
```

- The C `count` is derived from the array lengths (`min(origins/3, translations/3)`); a mismatched pair casts the
  shorter count instead of reading out of bounds.
- Results are packed into two flat tiers (`layout::rayI32` = 10 int32, `layout::rayF64` = 7 doubles per ray) read in
  place from the wasm heap, so a refill allocates nothing on the JS side. `scripts/build.mjs` publishes the strides
  through `_layoutMeta()`; keep the facade's field-read order in step with the packer.
- Ray origins are converted element-wise into `b3Pos` rather than reinterpret-cast from the `Float64Array`, because
  `b3Pos` is `b3Vec3` (floats) unless the library is built with `BOX3D_DOUBLE_PRECISION`.

## Tests

`wasm/test/smoke.mjs` (`modBatchRay`): four rays over a slab (three hits, one miss) with the expected fraction,
point and normal; parity with `b3World_CastRayClosest` for the same ray; the mismatched-length guard.
