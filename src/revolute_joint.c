// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

#include <stdio.h>

void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableSpring != joint->revoluteJoint.enableSpring )
	{
		joint->revoluteJoint.enableSpring = enableSpring;
		joint->revoluteJoint.springImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableSpring;
}

void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, b2Float hertz )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.hertz = hertz;
}

b2Float b2RevoluteJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.hertz;
}

void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, b2Float dampingRatio )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.dampingRatio = dampingRatio;
}

b2Float b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.dampingRatio;
}

b2Float b2RevoluteJoint_GetAngle( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	b2Transform transformA = b2GetBodyTransform( world, jointSim->bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, jointSim->bodyIdB );

	b2Float angle = b2RelativeAngle( transformB.q, transformA.q ) - jointSim->revoluteJoint.referenceAngle;
	angle = b2UnwindAngle( angle );
	return angle;
}

void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableLimit != joint->revoluteJoint.enableLimit )
	{
		joint->revoluteJoint.enableLimit = enableLimit;
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableLimit;
}

b2Float b2RevoluteJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.lowerAngle;
}

b2Float b2RevoluteJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.upperAngle;
}

void b2RevoluteJoint_SetLimits( b2JointId jointId, b2Float lower, b2Float upper )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( lower != joint->revoluteJoint.lowerAngle || upper != joint->revoluteJoint.upperAngle )
	{
		joint->revoluteJoint.lowerAngle = b2MinFloat( lower, upper );
		joint->revoluteJoint.upperAngle = b2MaxFloat( lower, upper );
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableMotor != joint->revoluteJoint.enableMotor )
	{
		joint->revoluteJoint.enableMotor = enableMotor;
		joint->revoluteJoint.motorImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableMotor;
}

void b2RevoluteJoint_SetMotorSpeed( b2JointId jointId, b2Float motorSpeed )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.motorSpeed = motorSpeed;
}

b2Float b2RevoluteJoint_GetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.motorSpeed;
}

b2Float b2RevoluteJoint_GetMotorTorque( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return world->inv_h * joint->revoluteJoint.motorImpulse;
}

void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, b2Float torque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.maxMotorTorque = torque;
}

b2Float b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.maxMotorTorque;
}

b2Vec2 b2GetRevoluteJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->revoluteJoint.linearImpulse );
	return force;
}

b2Float b2GetRevoluteJointTorque( b2World* world, b2JointSim* base )
{
	const b2RevoluteJoint* revolute = &base->revoluteJoint;
	b2Float torque = world->inv_h * ( revolute->motorImpulse + revolute->lowerImpulse - revolute->upperImpulse );
	return torque;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// Body State
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate the anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity. Base separation and angles need to be computed. Manifolds will be behind a frame, but that is probably best if bodies
// move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.

void b2PrepareRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2BodyArray_Get(&world->bodies, idA);
	b2Body* bodyB = b2BodyArray_Get(&world->bodies, idB);

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setA = b2SolverSetArray_Get( &world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2SolverSetArray_Get( &world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2BodySimArray_Get( &setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2BodySimArray_Get( &setB->bodySims, localIndexB );

	b2Float mA = bodySimA->invMass;
	b2Float iA = bodySimA->invInertia;
	b2Float mB = bodySimB->invMass;
	b2Float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// initial anchors in world space
	joint->anchorA = b2RotateVector( bodySimA->transform.q, b2Sub( base->localOriginAnchorA, bodySimA->localCenter ) );
	joint->anchorB = b2RotateVector( bodySimB->transform.q, b2Sub( base->localOriginAnchorB, bodySimB->localCenter ) );
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );
	joint->deltaAngle = b2RelativeAngle( bodySimB->transform.q, bodySimA->transform.q ) - joint->referenceAngle;
	joint->deltaAngle = b2UnwindAngle( joint->deltaAngle );

	b2Float k = iA + iB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	b2Float axialImpulse = joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;

	stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
	stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + axialImpulse );

	stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
	stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + axialImpulse );
}

void b2SolveRevoluteJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	b2Float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	b2Float wB = stateB->angularVelocity;

	bool fixedRotation = ( iA + iB == 0.0f );
	// const b2Float maxBias = context->maxBiasVelocity;

	// Solve spring.
	if ( joint->enableSpring && fixedRotation == false )
	{
		b2Float C = b2RelativeAngle( stateB->deltaRotation, stateA->deltaRotation ) + joint->deltaAngle;
		b2Float bias = joint->springSoftness.biasRate * C;
		b2Float massScale = joint->springSoftness.massScale;
		b2Float impulseScale = joint->springSoftness.impulseScale;

		b2Float Cdot = wB - wA;
		b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve motor constraint.
	if ( joint->enableMotor && fixedRotation == false )
	{
		b2Float Cdot = wB - wA - joint->motorSpeed;
		b2Float impulse = -joint->axialMass * Cdot;
		b2Float oldImpulse = joint->motorImpulse;
		b2Float maxImpulse = context->h * joint->maxMotorTorque;
		joint->motorImpulse = b2ClampFloat( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if ( joint->enableLimit && fixedRotation == false )
	{
		b2Float jointAngle = b2RelativeAngle( stateB->deltaRotation, stateA->deltaRotation ) + joint->deltaAngle;
		jointAngle = b2UnwindAngle( jointAngle );

		// Lower limit
		{
			b2Float C = jointAngle - joint->lowerAngle;
			b2Float bias = 0.0f;
			b2Float massScale = 1.0f;
			b2Float impulseScale = 0.0f;
			if ( C > 0.0f )
			{
				// speculation
				bias = C * context->inv_h;
			}
			else if ( useBias )
			{
				bias = context->jointSoftness.biasRate * C;
				massScale = context->jointSoftness.massScale;
				impulseScale = context->jointSoftness.impulseScale;
			}

			b2Float Cdot = wB - wA;
			b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->lowerImpulse;
			b2Float oldImpulse = joint->lowerImpulse;
			joint->lowerImpulse = b2MaxFloat( joint->lowerImpulse + impulse, 0.0f );
			impulse = joint->lowerImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			b2Float C = joint->upperAngle - jointAngle;
			b2Float bias = 0.0f;
			b2Float massScale = 1.0f;
			b2Float impulseScale = 0.0f;
			if ( C > 0.0f )
			{
				// speculation
				bias = C * context->inv_h;
			}
			else if ( useBias )
			{
				bias = context->jointSoftness.biasRate * C;
				massScale = context->jointSoftness.massScale;
				impulseScale = context->jointSoftness.impulseScale;
			}

			// sign flipped on Cdot
			b2Float Cdot = wA - wB;
			b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->lowerImpulse;
			b2Float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = b2MaxFloat( joint->upperImpulse + impulse, 0.0f );
			impulse = joint->upperImpulse - oldImpulse;

			// sign flipped on applied impulse
			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		// current anchors
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

		b2Vec2 bias = b2Vec2_zero;
		b2Float massScale = 1.0f;
		b2Float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;

			b2Vec2 separation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
			bias = b2MulSV( context->jointSoftness.biasRate, separation );
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2RevoluteJoint::Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2RevoluteJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", joint->collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", joint->localAnchorA.x, joint->localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", joint->localAnchorB.x, joint->localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", joint->referenceAngle);
	b2Dump("  jd.enableLimit = bool(%d);\n", joint->enableLimit);
	b2Dump("  jd.lowerAngle = %.9g;\n", joint->lowerAngle);
	b2Dump("  jd.upperAngle = %.9g;\n", joint->upperAngle);
	b2Dump("  jd.enableMotor = bool(%d);\n", joint->enableMotor);
	b2Dump("  jd.motorSpeed = %.9g;\n", joint->motorSpeed);
	b2Dump("  jd.maxMotorTorque = %.9g;\n", joint->maxMotorTorque);
	b2Dump("  joints[%d] = joint->world->CreateJoint(&jd);\n", joint->index);
}
#endif

void b2DrawRevoluteJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB, b2Float drawSize )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );

	b2HexColor c1 = b2_colorGray;
	b2HexColor c2 = b2_colorGreen;
	b2HexColor c3 = b2_colorRed;

	const b2Float L = drawSize;
	// draw->drawPoint(pA, 3.0f, b2_colorGray40, draw->context);
	// draw->drawPoint(pB, 3.0f, b2_colorLightBlue, draw->context);
	draw->DrawCircle( pB, L, c1, draw->context );

	b2Float angle = b2RelativeAngle( transformB.q, transformA.q );

	b2Rot rot = b2MakeRot( angle );
	b2Vec2 r = { L * rot.c, L * rot.s };
	b2Vec2 pC = b2Add( pB, r );
	draw->DrawSegment( pB, pC, c1, draw->context );

	if ( draw->drawJointExtras )
	{
		b2Float jointAngle = b2UnwindAngle( angle - joint->referenceAngle );
		char buffer[32];
		snprintf( buffer, 32, " %.1f deg", 180.0f * jointAngle / B2_PI );
		draw->DrawString( pC, buffer, b2_colorWhite, draw->context );
	}

	b2Float lowerAngle = joint->lowerAngle + joint->referenceAngle;
	b2Float upperAngle = joint->upperAngle + joint->referenceAngle;

	if ( joint->enableLimit )
	{
		b2Rot rotLo = b2MakeRot( lowerAngle );
		b2Vec2 rlo = { L * rotLo.c, L * rotLo.s };

		b2Rot rotHi = b2MakeRot( upperAngle );
		b2Vec2 rhi = { L * rotHi.c, L * rotHi.s };

		draw->DrawSegment( pB, b2Add( pB, rlo ), c2, draw->context );
		draw->DrawSegment( pB, b2Add( pB, rhi ), c3, draw->context );

		b2Rot rotRef = b2MakeRot( joint->referenceAngle );
		b2Vec2 ref = ( b2Vec2 ){ L * rotRef.c, L * rotRef.s };
		draw->DrawSegment( pB, b2Add( pB, ref ), b2_colorBlue, draw->context );
	}

	b2HexColor color = b2_colorGold;
	draw->DrawSegment( transformA.p, pA, color, draw->context );
	draw->DrawSegment( pA, pB, color, draw->context );
	draw->DrawSegment( transformB.p, pB, color, draw->context );

	// char buffer[32];
	// sprintf(buffer, "%.1f", b2Length(joint->impulse));
	// draw->DrawString(pA, buffer, draw->context);
}
