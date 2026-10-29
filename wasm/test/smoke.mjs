// Smoke test: base API + a real "falling box" simulation, run against both
// build variants (separate-wasm and inlined single-file).

import assert from 'node:assert/strict';

function fallingBox( b3 )
{
	// world with downward gravity
	const worldDef = b3.b3DefaultWorldDef();
	worldDef.gravity = [ 0, -10, 0 ];
	const world = b3.b3CreateWorld( worldDef );
	assert.ok( b3.b3World_IsValid( world ), 'world is valid' );

	// static ground: a wide, thin box centered at the origin
	const groundBodyDef = b3.b3DefaultBodyDef();
	groundBodyDef.type = b3.b3BodyType.b3_staticBody;
	groundBodyDef.position = [ 0, 0, 0 ];
	const ground = b3.b3CreateBody( world, groundBodyDef );
	b3.b3CreateBoxShape( ground, b3.b3DefaultShapeDef(), 25, 0.5, 25 );

	// dynamic body: a sphere dropped from y = 10
	const bodyDef = b3.b3DefaultBodyDef();
	bodyDef.type = b3.b3BodyType.b3_dynamicBody;
	bodyDef.position = [ 0, 10, 0 ];
	const body = b3.b3CreateBody( world, bodyDef );
	const shapeDef = b3.b3DefaultShapeDef();
	b3.b3CreateSphereShape( body, shapeDef, { center: [ 0, 0, 0 ], radius: 0.5 } );

	// out-param reader: fill a caller-owned [x,y,z], read component 1 (y)
	const pos = [ 0, 0, 0 ];
	const startY = b3.b3Body_GetPosition( pos, body )[ 1 ];

	// simulate ~2.5 s at 60 Hz
	for ( let i = 0; i < 150; i++ )
	{
		b3.b3World_Step( world, 1 / 60, 4 );
	}

	const endY = b3.b3Body_GetPosition( pos, body )[ 1 ];

	// multi-out reader: a transform reads as two out params (position + rotation),
	// and returns them as a [position, rotation] tuple.
	const tp = [ 0, 0, 0 ], tq = [ 0, 0, 0, 1 ];
	const [ tPos, tRot ] = b3.b3Body_GetTransform( tp, tq, body );
	assert.ok( tPos === tp && tRot === tq, 'GetTransform fills the caller arrays and returns them' );
	assert.ok( Math.abs( tPos[ 1 ] - endY ) < 1e-6, 'transform position matches GetPosition' );
	assert.ok( Math.abs( Math.hypot( tRot[ 0 ], tRot[ 1 ], tRot[ 2 ], tRot[ 3 ] ) - 1 ) < 1e-3, 'transform rotation is a unit quaternion' );

	b3.b3DestroyWorld( world );
	return { startY, endY };
}

// Drop a box onto the ground, let it settle, then read the resulting contact via
// the packed buffer + facade readers (createContact/getContactAt/getManifoldAt).
function contactRead( b3 )
{
	const worldDef = b3.b3DefaultWorldDef();
	worldDef.gravity = [ 0, -10, 0 ];
	const world = b3.b3CreateWorld( worldDef );

	const groundDef = b3.b3DefaultBodyDef();
	groundDef.type = b3.b3BodyType.b3_staticBody;
	const ground = b3.b3CreateBody( world, groundDef );
	b3.b3CreateBoxShape( ground, b3.b3DefaultShapeDef(), 25, 0.5, 25 );

	const boxDef = b3.b3DefaultBodyDef();
	boxDef.type = b3.b3BodyType.b3_dynamicBody;
	boxDef.position = [ 0, 5, 0 ];
	const box = b3.b3CreateBody( world, boxDef );
	const boxShape = b3.b3CreateBoxShape( box, b3.b3DefaultShapeDef(), 0.5, 0.5, 0.5 );

	// events buffer: one step while the box is falling reliably produces a body
	// move event — read it back through the reusable, wasm-backed events buffer.
	const eb = b3.createEventsBuffer();
	const move = b3.createBodyMoveEvent();
	b3.b3World_Step( world, 1 / 60, 4 );
	b3.getEvents( eb, world );
	const moveCount = b3.getNumBodyMoveEvents( eb );
	assert.ok( moveCount >= 1, `body move events reported while falling (got ${moveCount})` );
	b3.getBodyMoveEventAt( move, eb, 0 );
	assert.ok( Number.isFinite( move.position[ 1 ] ), 'body move event decodes a finite position' );
	const q = move.rotation;
	assert.ok( Math.abs( Math.hypot( q[ 0 ], q[ 1 ], q[ 2 ], q[ 3 ] ) - 1 ) < 1e-3, 'move event rotation is a unit quaternion' );

	for ( let i = 0; i < 150; i++ ) b3.b3World_Step( world, 1 / 60, 4 );

	// reusable contacts buffer: fill it, read it, then refill after another step to
	// exercise reuse + the growth-safe view refresh.
	const cb = b3.createContactsBuffer();
	const contact = b3.createContact();
	const manifold = b3.createManifold();

	function summarize()
	{
		b3.getShapeContactData( cb, boxShape );
		const n = b3.getNumContacts( cb );
		let totalPoints = 0;
		let worstNormalErr = 0;
		for ( let i = 0; i < n; i++ )
		{
			b3.getContactAt( contact, cb, i );
			// the queried shape is one side of the contact (box3d picks the A/B order)
			assert.ok(
				contact.shapeIdA.index1 === boxShape.index1 || contact.shapeIdB.index1 === boxShape.index1,
				'queried shape id round-trips as one side of the contact',
			);
			for ( let m = 0; m < contact.manifoldCount; m++ )
			{
				b3.getManifoldAt( manifold, contact, m );
				const nrm = manifold.normal;
				worstNormalErr = Math.max( worstNormalErr, Math.abs( Math.hypot( nrm[ 0 ], nrm[ 1 ], nrm[ 2 ] ) - 1 ) );
				totalPoints += manifold.pointCount;
			}
		}
		return { n, totalPoints, worstNormalErr };
	}

	const first = summarize();
	b3.b3World_Step( world, 1 / 60, 4 );
	const second = summarize(); // still valid after a refill following a step

	assert.ok( first.n >= 1, `resting box reports a contact (got ${first.n})` );
	assert.ok( first.totalPoints >= 1, `contact has at least one manifold point (got ${first.totalPoints})` );
	assert.ok( first.worstNormalErr < 1e-3, `manifold normal is unit-length (err ${first.worstNormalErr.toExponential( 1 )})` );
	assert.ok( second.n >= 1, 'reusable buffer still valid after a refill following a step' );

	b3.destroyContactsBuffer( cb );
	b3.destroyEventsBuffer( eb );
	b3.b3DestroyWorld( world );
	return { n: first.n, totalPoints: first.totalPoints, worstNormalErr: first.worstNormalErr };
}

// Exercise the sensor-touch, and CollideMover plane-result readers — the
// browser-facing buffer paths the examples use but the contact scene doesn't hit.
function eventsAndPlanes( b3 )
{
	const world = b3.b3CreateWorld( b3.b3DefaultWorldDef() );

	// sensor box at the origin with a dynamic visitor overlapping it
	const sensorBody = b3.b3CreateBody( world, b3.b3DefaultBodyDef() );
	const sensorShapeDef = b3.b3DefaultShapeDef();
	sensorShapeDef.isSensor = true;
	sensorShapeDef.enableSensorEvents = true;
	b3.b3CreateBoxShape( sensorBody, sensorShapeDef, 2, 2, 2 );

	const visitorDef = b3.b3DefaultBodyDef();
	visitorDef.type = b3.b3BodyType.b3_dynamicBody;
	visitorDef.position = [ 0, 0, 0 ];
	const visitorBody = b3.b3CreateBody( world, visitorDef );
	const visitorShapeDef = b3.b3DefaultShapeDef();
	visitorShapeDef.enableSensorEvents = true;
	const visitorShape = b3.b3CreateBoxShape( visitorBody, visitorShapeDef, 0.5, 0.5, 0.5 );

	const eb = b3.createEventsBuffer();
	const touch = b3.createSensorTouchEvent();
	let sawSensor = false;
	for ( let i = 0; i < 10 && !sawSensor; i++ )
	{
		b3.b3World_Step( world, 1 / 60, 4 );
		b3.getEvents( eb, world );
		for ( let k = 0, n = b3.getNumSensorBeginEvents( eb ); k < n; k++ )
		{
			b3.getSensorBeginEventAt( touch, eb, k );
			if ( touch.visitorShapeId.index1 === visitorShape.index1 ) sawSensor = true;
		}
	}
	b3.destroyEventsBuffer( eb );
	assert.ok( sawSensor, 'sensor begin-touch event decoded via the events buffer' );

	// CollideMover: a mover capsule overlapping a static sphere yields collision
	// planes (placed clear of the sensor bodies above).
	const obstacleDef = b3.b3DefaultBodyDef();
	obstacleDef.position = [ 10, 1, 0 ];
	const obstacle = b3.b3CreateBody( world, obstacleDef );
	b3.b3CreateSphereShape( obstacle, b3.b3DefaultShapeDef(), { center: [ 0, 0, 0 ], radius: 0.6 } );
	b3.b3World_Step( world, 1 / 60, 4 ); // put the new shape into the broadphase
	const capsule = { center1: [ 0, -0.5, 0 ], center2: [ 0, 0.5, 0 ], radius: 0.35 };
	const planeResult = b3.createPlaneResult();
	let planeCount = 0;
	let worstNormalErr = 0;
	b3.b3World_CollideMover( world, [ 10, 1, 0 ], capsule, b3.b3DefaultQueryFilter(), ( _s, buf ) =>
	{
		for ( let i = 0, n = b3.getNumPlaneResults( buf ); i < n; i++ )
		{
			b3.getPlaneResultAt( planeResult, buf, i );
			const nrm = planeResult.plane.normal;
			worstNormalErr = Math.max( worstNormalErr, Math.abs( Math.hypot( nrm[ 0 ], nrm[ 1 ], nrm[ 2 ] ) - 1 ) );
			planeCount++;
		}
		return true;
	} );
	assert.ok( planeCount >= 1, `CollideMover reported plane results via the buffer (got ${planeCount})` );
	assert.ok( worstNormalErr < 1e-3, 'plane-result normal is unit-length' );

	b3.b3DestroyWorld( world );
	return { sawSensor, planeCount };
}

// --- nukabox3d mods ----------------------------------------------------------
// Each block exercises one fork-only addition so the bindings cannot silently
// drop a field again (embind registration failures are runtime-only).

const IDENTITY = [ 0, 0, 0, 1 ];

// Axis-angle quaternion, computed in JS (the bindings do not expose quat math).
function quatFromAxisAngle( axis, angle )
{
	const [ ax, ay, az ] = axis;
	const len = Math.hypot( ax, ay, az );
	const s = Math.sin( angle * 0.5 ) / len;
	return [ ax * s, ay * s, az * s, Math.cos( angle * 0.5 ) ];
}

// C test/test_joint.c RunSpringClampScenario, in miniature: gravity off so the
// joint spring is the only axial torque. Returns the peak constraint torque.
function springClampPeak( b3, spherical, maxSpringTorque, useSetter, armature = 0 )
{
	const worldDef = b3.b3DefaultWorldDef();
	worldDef.gravity = [ 0, 0, 0 ];
	const world = b3.b3CreateWorld( worldDef );

	const ground = b3.b3CreateBody( world, b3.b3DefaultBodyDef() );
	const bodyDef = b3.b3DefaultBodyDef();
	bodyDef.type = b3.b3BodyType.b3_dynamicBody;
	bodyDef.position = [ 0, 4, 0 ];
	const body = b3.b3CreateBody( world, bodyDef );
	const shapeDef = b3.b3DefaultShapeDef();
	shapeDef.density = 1;
	b3.b3CreateBoxShape( body, shapeDef, 0.5, 0.5, 0.5 );

	// both frames map to the same world point (the body centre)
	const frameA = { position: [ 0, 4, 0 ], quaternion: IDENTITY.slice() };
	const frameB = { position: [ 0, 0, 0 ], quaternion: IDENTITY.slice() };

	let joint;
	if ( spherical )
	{
		const def = b3.b3DefaultSphericalJointDef();
		def.base.bodyIdA = ground;
		def.base.bodyIdB = body;
		def.base.localFrameA = frameA;
		def.base.localFrameB = frameB;
		def.base.armature = armature;
		def.enableSpring = true;
		def.hertz = 5;
		def.dampingRatio = 0.5;
		def.targetRotation = quatFromAxisAngle( [ 1, 1, 1 ], 1 );
		def.maxSpringTorque = useSetter ? 0 : maxSpringTorque;
		joint = b3.b3CreateSphericalJoint( world, def );
		if ( useSetter ) b3.b3SphericalJoint_SetMaxSpringTorque( joint, maxSpringTorque );
	}
	else
	{
		const def = b3.b3DefaultRevoluteJointDef();
		def.base.bodyIdA = ground;
		def.base.bodyIdB = body;
		def.base.localFrameA = frameA;
		def.base.localFrameB = frameB;
		def.base.armature = armature;
		def.enableSpring = true;
		def.hertz = 5;
		def.dampingRatio = 0.5;
		def.targetAngle = 1;
		def.maxSpringTorque = useSetter ? 0 : maxSpringTorque;
		joint = b3.b3CreateRevoluteJoint( world, def );
		if ( useSetter ) b3.b3RevoluteJoint_SetMaxSpringTorque( joint, maxSpringTorque );
	}

	const torque = [ 0, 0, 0 ];
	let peak = 0;
	for ( let i = 0; i < 120; i++ )
	{
		b3.b3World_Step( world, 1 / 60, 4 );
		b3.b3Joint_GetConstraintTorque( torque, joint );
		peak = Math.max( peak, Math.abs( torque[ 0 ] ), Math.abs( torque[ 1 ] ), Math.abs( torque[ 2 ] ) );
	}

	const q = [ 0, 0, 0, 1 ];
	b3.b3Body_GetRotation( q, body );
	b3.b3DestroyWorld( world );
	return { peak, rotation: q };
}

function modJointSprings( b3 )
{
	for ( const spherical of [ false, true ] )
	{
		const label = spherical ? 'spherical' : 'revolute';
		const { peak: unclamped, rotation: qUnclamped } = springClampPeak( b3, spherical, 0, false );
		assert.ok( unclamped > 0, `${label}: unclamped spring reports a torque (got ${unclamped})` );

		const clamp = 0.01 * unclamped;
		// b3GetRevoluteJointTorque counts the axial impulse twice, so the revolute
		// report is 2x the spring torque (same bookkeeping as test/test_joint.c).
		const reportScale = spherical ? 1 : 2;
		const { peak: clamped, rotation: qClamped } = springClampPeak( b3, spherical, clamp, false );
		assert.ok( clamped <= reportScale * clamp * 1.001,
			`${label}: maxSpringTorque caps the spring (peak ${clamped.toFixed( 4 )} <= ${( reportScale * clamp ).toFixed( 4 )})` );
		assert.ok( clamped > 0.5 * reportScale * clamp,
			`${label}: clamp actually bites rather than killing the spring (peak ${clamped.toFixed( 4 )})` );
		assert.ok( qUnclamped.some( ( v, i ) => v !== qClamped[ i ] ), `${label}: the clamp changes the final rotation` );

		// the setter path must match the def field exactly
		const { peak: viaSetter } = springClampPeak( b3, spherical, clamp, true );
		assert.ok( Math.abs( viaSetter - clamped ) < 1e-6,
			`${label}: Set/GetMaxSpringTorque matches the def field (${viaSetter} vs ${clamped})` );

		// zero is unclamped, and a huge clamp never engages
		const { peak: zeroAgain } = springClampPeak( b3, spherical, 0, true );
		assert.ok( zeroAgain === unclamped, `${label}: maxSpringTorque 0 is unclamped (${zeroAgain} vs ${unclamped})` );
		const { peak: huge } = springClampPeak( b3, spherical, 1e9, false );
		assert.ok( huge === unclamped, `${label}: a huge clamp is identical to unclamped` );

		// armature must change the response (def field, no getter on the C side)
		const { peak: withArmature } = springClampPeak( b3, spherical, 0, false, 0.05 );
		assert.ok( withArmature < unclamped,
			`${label}: armature shrinks the spring impulse (${withArmature.toFixed( 4 )} < ${unclamped.toFixed( 4 )})` );
	}

	// swing/twist spring fields + Set/Get round-trip, and the mode actually changes
	// the response versus the single spring (the swing/twist def ignores hertz).
	{
		const worldDef = b3.b3DefaultWorldDef();
		worldDef.gravity = [ 0, 0, 0 ];
		const world = b3.b3CreateWorld( worldDef );
		const ground = b3.b3CreateBody( world, b3.b3DefaultBodyDef() );
		const bodyDef = b3.b3DefaultBodyDef();
		bodyDef.type = b3.b3BodyType.b3_dynamicBody;
		bodyDef.position = [ 0, 4, 0 ];
		const body = b3.b3CreateBody( world, bodyDef );
		const shapeDef = b3.b3DefaultShapeDef();
		shapeDef.density = 1;
		b3.b3CreateBoxShape( body, shapeDef, 0.5, 0.5, 0.5 );

		const def = b3.b3DefaultSphericalJointDef();
		def.base.bodyIdA = ground;
		def.base.bodyIdB = body;
		def.base.localFrameA = { position: [ 0, 4, 0 ], quaternion: IDENTITY.slice() };
		def.base.localFrameB = { position: [ 0, 0, 0 ], quaternion: IDENTITY.slice() };
		def.enableSpring = true;
		def.hertz = 5;
		def.dampingRatio = 0.5;
		def.targetRotation = quatFromAxisAngle( [ 0, 0, 1 ], 1.2 );
		def.enableSwingTwistSpring = true;
		def.swingHertz = 3;
		def.swingDampingRatio = 0.4;
		def.twistHertz = 2;
		def.twistDampingRatio = 0.3;
		const joint = b3.b3CreateSphericalJoint( world, def );

		assert.ok( b3.b3SphericalJoint_IsSwingTwistSpringEnabled( joint ), 'swing/twist spring is enabled from the def' );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetSwingSpringHertz( joint ) - 3 ) < 1e-6, 'swingHertz copied from the def' );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetTwistSpringHertz( joint ) - 2 ) < 1e-6, 'twistHertz copied from the def' );
		b3.b3SphericalJoint_SetSwingSpringHertz( joint, 4 );
		b3.b3SphericalJoint_SetSwingSpringDampingRatio( joint, 0.6 );
		b3.b3SphericalJoint_SetTwistSpringHertz( joint, 1.5 );
		b3.b3SphericalJoint_SetTwistSpringDampingRatio( joint, 0.2 );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetSwingSpringHertz( joint ) - 4 ) < 1e-6, 'SetSwingSpringHertz round-trips' );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetSwingSpringDampingRatio( joint ) - 0.6 ) < 1e-6, 'SetSwingSpringDampingRatio round-trips' );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetTwistSpringHertz( joint ) - 1.5 ) < 1e-6, 'SetTwistSpringHertz round-trips' );
		assert.ok( Math.abs( b3.b3SphericalJoint_GetTwistSpringDampingRatio( joint ) - 0.2 ) < 1e-6, 'SetTwistSpringDampingRatio round-trips' );

		for ( let i = 0; i < 60; i++ ) b3.b3World_Step( world, 1 / 60, 4 );
		const q = [ 0, 0, 0, 1 ];
		b3.b3Body_GetRotation( q, body );
		assert.ok( q.every( Number.isFinite ), 'swing/twist spring steps to a finite rotation' );
		// the spring pulls the body toward the target, so it must have rotated
		assert.ok( Math.abs( q[ 2 ] ) > 1e-3, `swing/twist spring rotated the body (qz ${q[ 2 ].toFixed( 4 )})` );

		b3.b3SphericalJoint_EnableSwingTwistSpring( joint, false );
		assert.ok( !b3.b3SphericalJoint_IsSwingTwistSpringEnabled( joint ), 'EnableSwingTwistSpring(false) disables it' );
		b3.b3DestroyWorld( world );
	}
}

// b3WorldDef joint-vs-contact tuning. Create-time only (the C API has no
// b3World_Set* for these), so the check is: the world builds, a jointed body
// resting on the ground stays stable and settled with the stiffer joint path on.
function modWorldTuning( b3 )
{
	const worldDef = b3.b3DefaultWorldDef();
	worldDef.gravity = [ 0, -10, 0 ];
	worldDef.jointHertz = 60;
	worldDef.jointDampingRatio = 1;
	worldDef.jointRelaxIterations = 2;
	worldDef.solveJointsAfterContacts = true;
	const world = b3.b3CreateWorld( worldDef );
	assert.ok( b3.b3World_IsValid( world ), 'world with joint-vs-contact tuning is valid' );

	const groundDef = b3.b3DefaultBodyDef();
	groundDef.position = [ 0, 0, 0 ];
	const ground = b3.b3CreateBody( world, groundDef );
	b3.b3CreateBoxShape( ground, b3.b3DefaultShapeDef(), 25, 0.5, 25 );

	// static anchor at (0, 2, 0), dynamic box hanging off a revolute joint beside it
	const anchorDef = b3.b3DefaultBodyDef();
	anchorDef.position = [ 0, 2, 0 ];
	const anchor = b3.b3CreateBody( world, anchorDef );

	const boxDef = b3.b3DefaultBodyDef();
	boxDef.type = b3.b3BodyType.b3_dynamicBody;
	boxDef.position = [ 1, 2, 0 ];
	const box = b3.b3CreateBody( world, boxDef );
	const boxShapeDef = b3.b3DefaultShapeDef();
	boxShapeDef.density = 1;
	b3.b3CreateBoxShape( box, boxShapeDef, 0.4, 0.4, 0.4 );

	const jointDef = b3.b3DefaultRevoluteJointDef();
	jointDef.base.bodyIdA = anchor;
	jointDef.base.bodyIdB = box;
	jointDef.base.localFrameA = { position: [ 1, 0, 0 ], quaternion: IDENTITY.slice() };
	jointDef.base.localFrameB = { position: [ 0, 0, 0 ], quaternion: IDENTITY.slice() };
	jointDef.base.constraintHertz = 30;
	jointDef.base.constraintDampingRatio = 0.5;
	const joint = b3.b3CreateRevoluteJoint( world, jointDef );

	const pos = [ 0, 0, 0 ], torque = [ 0, 0, 0 ];
	for ( let i = 0; i < 240; i++ ) b3.b3World_Step( world, 1 / 60, 4 );
	b3.b3Body_GetPosition( pos, box );
	b3.b3Joint_GetConstraintTorque( torque, joint );

	assert.ok( pos.every( Number.isFinite ) && torque.every( Number.isFinite ), 'tuned joint-vs-contact world stays finite' );
	assert.ok( Math.abs( pos[ 1 ] ) < 3, `jointed box did not explode (y ${pos[ 1 ].toFixed( 3 )})` );
	assert.ok( Math.abs( pos[ 0 ] ) > 0.2, `joint still holds the box away from the anchor (x ${pos[ 0 ].toFixed( 3 )})` );

	b3.b3DestroyWorld( world );
}

// b3World_CastRayClosestBatch -> reusable packed RayResultBuffer.
function modBatchRay( b3 )
{
	const worldDef = b3.b3DefaultWorldDef();
	worldDef.gravity = [ 0, 0, 0 ];
	const world = b3.b3CreateWorld( worldDef );
	const ground = b3.b3CreateBody( world, b3.b3DefaultBodyDef() );
	const groundShape = b3.b3CreateBoxShape( ground, b3.b3DefaultShapeDef(), 25, 0.5, 25 );
	b3.b3World_Step( world, 1 / 60, 1 ); // publish the shape to the broadphase

	// 3 rays straight down onto the slab, 1 that misses it entirely
	const origins = new Float64Array( [ -1, 5, 0, 0, 5, 0, 1, 5, 0, 100, 5, 100 ] );
	const translations = new Float32Array( [ 0, -10, 0, 0, -10, 0, 0, -10, 0, 0, -10, 0 ] );

	const rb = b3.createRayResultBuffer();
	const hit = b3.createRayResult();
	const n = b3.castRayClosestBatch( rb, world, origins, translations );
	assert.equal( n, 4, `batch ray cast reports one result per ray (got ${n})` );
	assert.equal( b3.getNumRayResults( rb ), 4, 'getNumRayResults matches the ray count' );

	let hits = 0;
	for ( let i = 0; i < n; i++ )
	{
		b3.getRayResultAt( hit, rb, i );
		if ( hit.hit )
		{
			hits++;
			assert.equal( hit.shapeId.index1, groundShape.index1, 'batch ray hit the slab' );
			assert.ok( Math.abs( hit.fraction - 0.45 ) < 1e-4, `batch ray fraction is 0.45 (got ${hit.fraction})` );
			assert.ok( Math.abs( hit.point[ 1 ] - 0.5 ) < 1e-3, `batch ray point sits on the slab top (y ${hit.point[ 1 ]})` );
			assert.ok( Math.abs( Math.hypot( ...hit.normal ) - 1 ) < 1e-3, 'batch ray normal is unit length' );
		}
	}
	assert.equal( hits, 3, `3 of 4 rays hit (got ${hits})` );

	// parity with the single-ray call
	{
		const single = b3.b3World_CastRayClosest( world, [ 0, 5, 0 ], [ 0, -10, 0 ], b3.b3DefaultQueryFilter() );
		b3.getRayResultAt( hit, rb, 1 );
		assert.equal( hit.shapeId.index1, single.shapeId.index1, 'batch result matches b3World_CastRayClosest shape' );
		assert.ok( Math.abs( hit.fraction - single.fraction ) < 1e-5, 'batch result matches the single ray fraction' );
		assert.ok( Math.hypot( hit.point[ 0 ] - single.point[ 0 ], hit.point[ 1 ] - single.point[ 1 ], hit.point[ 2 ] - single.point[ 2 ] ) < 1e-4,
			'batch result matches the single ray point' );
	}

	// mismatched array lengths must not read past either buffer
	{
		const short = new Float64Array( [ 0, 5, 0, 0, 5, 0 ] ); // 2 rays
		const long = new Float32Array( 9 ); // 3 rays
		long.set( [ 0, -10, 0, 0, -10, 0, 0, -10, 0 ] );
		assert.equal( b3.castRayClosestBatch( rb, world, short, long ), 2, 'mismatched lengths cast the shorter count' );
		assert.equal( b3.castRayClosestBatch( rb, world, new Float64Array( 0 ), long ), 0, 'empty origins casts nothing' );
	}

	b3.destroyRayResultBuffer( rb );
	b3.b3DestroyWorld( world );
	return hits;
}

async function check( label, importPath )
{
	const { default: Box3D } = await import( importPath );
	const b3 = await Box3D();

	const v = b3.b3GetVersion();
	const { startY, endY } = fallingBox( b3 );

	assert.ok( endY < startY - 5, `body fell (start=${startY.toFixed( 2 )} end=${endY.toFixed( 2 )})` );
	assert.ok( endY > 0.5, `body rests above the ground, not through it (end=${endY.toFixed( 2 )})` );

	const { n, totalPoints, worstNormalErr } = contactRead( b3 );
	assert.ok( n >= 1, `resting box reports a contact (got ${n})` );
	assert.ok( totalPoints >= 1, `contact has at least one manifold point (got ${totalPoints})` );
	assert.ok( worstNormalErr < 1e-3, `manifold normal is unit-length (err ${worstNormalErr.toExponential( 1 )})` );

	const { planeCount } = eventsAndPlanes( b3 );

	// nukabox3d mods: spring torque clamp, armature, swing/twist spring,
	// joint-vs-contact world tuning, batch ray cast.
	modJointSprings( b3 );
	modWorldTuning( b3 );
	const batchHits = modBatchRay( b3 );

	console.log( `  ${label}: box3d v${v.major}.${v.minor}.${v.revision} — ` +
		`sphere fell ${startY.toFixed( 2 )} -> ${endY.toFixed( 2 )} and settled; ` +
		`read ${n} contact(s)/${totalPoints} point(s), a sensor touch, and ${planeCount} mover plane(s) from the buffers; ` +
		`mods: spring clamp + armature + swing/twist, tuned joint-vs-contact world, ${batchHits}/4 batch rays` );
}

console.log( 'box3d.js smoke test' );
await check( 'separate-wasm', '../dist/box3d.mjs' );
await check( 'inline       ', '../dist/box3d.inline.mjs' );
console.log( 'OK' );
