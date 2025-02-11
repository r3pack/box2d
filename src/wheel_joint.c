// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

#include <stdio.h>

void b2WheelJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );

	if ( enableSpring != joint->wheelJoint.enableSpring )
	{
		joint->wheelJoint.enableSpring = enableSpring;
		joint->wheelJoint.springImpulse = 0.0f;
	}
}

bool b2WheelJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableSpring;
}

void b2WheelJoint_SetSpringHertz( b2JointId jointId, b2Float hertz )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.hertz = hertz;
}

b2Float b2WheelJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.hertz;
}

void b2WheelJoint_SetSpringDampingRatio( b2JointId jointId, b2Float dampingRatio )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.dampingRatio = dampingRatio;
}

b2Float b2WheelJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.dampingRatio;
}

void b2WheelJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	if ( joint->wheelJoint.enableLimit != enableLimit )
	{
		joint->wheelJoint.lowerImpulse = 0.0f;
		joint->wheelJoint.upperImpulse = 0.0f;
		joint->wheelJoint.enableLimit = enableLimit;
	}
}

bool b2WheelJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableLimit;
}

b2Float b2WheelJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.lowerTranslation;
}

b2Float b2WheelJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.upperTranslation;
}

void b2WheelJoint_SetLimits( b2JointId jointId, b2Float lower, b2Float upper )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	if ( lower != joint->wheelJoint.lowerTranslation || upper != joint->wheelJoint.upperTranslation )
	{
		joint->wheelJoint.lowerTranslation = b2MinFloat( lower, upper );
		joint->wheelJoint.upperTranslation = b2MaxFloat( lower, upper );
		joint->wheelJoint.lowerImpulse = 0.0f;
		joint->wheelJoint.upperImpulse = 0.0f;
	}
}

void b2WheelJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	if ( joint->wheelJoint.enableMotor != enableMotor )
	{
		joint->wheelJoint.motorImpulse = 0.0f;
		joint->wheelJoint.enableMotor = enableMotor;
	}
}

bool b2WheelJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableMotor;
}

void b2WheelJoint_SetMotorSpeed( b2JointId jointId, b2Float motorSpeed )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.motorSpeed = motorSpeed;
}

b2Float b2WheelJoint_GetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.motorSpeed;
}

b2Float b2WheelJoint_GetMotorTorque( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return world->inv_h * joint->wheelJoint.motorImpulse;
}

void b2WheelJoint_SetMaxMotorTorque( b2JointId jointId, b2Float torque )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.maxMotorTorque = torque;
}

b2Float b2WheelJoint_GetMaxMotorTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.maxMotorTorque;
}

b2Vec2 b2GetWheelJointForce( b2World* world, b2JointSim* base )
{
	b2WheelJoint* joint = &base->wheelJoint;

	// This is a frame behind
	b2Vec2 axisA = joint->axisA;
	b2Vec2 perpA = b2LeftPerp( axisA );

	b2Float perpForce = world->inv_h * joint->perpImpulse;
	b2Float axialForce = world->inv_h * ( joint->springImpulse + joint->lowerImpulse - joint->upperImpulse );

	b2Vec2 force = b2Add( b2MulSV( perpForce, perpA ), b2MulSV( axialForce, axisA ) );
	return force;
}

b2Float b2GetWheelJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * base->wheelJoint.motorImpulse;
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2PrepareWheelJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );

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

	b2WheelJoint* joint = &base->wheelJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	b2Rot qA = bodySimA->transform.q;
	b2Rot qB = bodySimB->transform.q;

	joint->anchorA = b2RotateVector( qA, b2Sub( base->localOriginAnchorA, bodySimA->localCenter ) );
	joint->anchorB = b2RotateVector( qB, b2Sub( base->localOriginAnchorB, bodySimB->localCenter ) );
	joint->axisA = b2RotateVector( qA, joint->localAxisA );
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;

	b2Vec2 d = b2Add( joint->deltaCenter, b2Sub( rB, rA ) );
	b2Vec2 axisA = joint->axisA;
	b2Vec2 perpA = b2LeftPerp( axisA );

	// perpendicular constraint (keep wheel on line)
	b2Float s1 = b2Cross( b2Add( d, rA ), perpA );
	b2Float s2 = b2Cross( rB, perpA );

	b2Float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	joint->perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

	// spring constraint
	b2Float a1 = b2Cross( b2Add( d, rA ), axisA );
	b2Float a2 = b2Cross( rB, axisA );

	b2Float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	b2Float km = iA + iB;
	joint->motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	if ( context->enableWarmStarting == false )
	{
		joint->perpImpulse = 0.0f;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartWheelJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	b2Vec2 axisA = b2RotateVector( stateA->deltaRotation, joint->axisA );
	b2Vec2 perpA = b2LeftPerp( axisA );

	b2Float a1 = b2Cross( b2Add( d, rA ), axisA );
	b2Float a2 = b2Cross( rB, axisA );
	b2Float s1 = b2Cross( b2Add( d, rA ), perpA );
	b2Float s2 = b2Cross( rB, perpA );

	b2Float axialImpulse = joint->springImpulse + joint->lowerImpulse - joint->upperImpulse;

	b2Vec2 P = b2Add( b2MulSV( axialImpulse, axisA ), b2MulSV( joint->perpImpulse, perpA ) );
	b2Float LA = axialImpulse * a1 + joint->perpImpulse * s1 + joint->motorImpulse;
	b2Float LB = axialImpulse * a2 + joint->perpImpulse * s2 + joint->motorImpulse;

	stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, P );
	stateA->angularVelocity -= iA * LA;
	stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, P );
	stateB->angularVelocity += iB * LB;
}

void b2SolveWheelJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	b2Float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	b2Float wB = stateB->angularVelocity;

	bool fixedRotation = ( iA + iB == 0.0f );

	// current anchors
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	b2Vec2 axisA = b2RotateVector( stateA->deltaRotation, joint->axisA );
	b2Float translation = b2Dot( axisA, d );

	b2Float a1 = b2Cross( b2Add( d, rA ), axisA );
	b2Float a2 = b2Cross( rB, axisA );

	// motor constraint
	if ( joint->enableMotor && fixedRotation == false )
	{
		b2Float Cdot = wB - wA - joint->motorSpeed;
		b2Float impulse = -joint->motorMass * Cdot;
		b2Float oldImpulse = joint->motorImpulse;
		b2Float maxImpulse = context->h * joint->maxMotorTorque;
		joint->motorImpulse = b2ClampFloat( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// spring constraint
	if ( joint->enableSpring )
	{
		// This is a real spring and should be applied even during relax
		b2Float C = translation;
		b2Float bias = joint->springSoftness.biasRate * C;
		b2Float massScale = joint->springSoftness.massScale;
		b2Float impulseScale = joint->springSoftness.impulseScale;

		b2Float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
		b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		b2Vec2 P = b2MulSV( impulse, axisA );
		b2Float LA = impulse * a1;
		b2Float LB = impulse * a2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	if ( joint->enableLimit )
	{
		// Lower limit
		{
			b2Float C = translation - joint->lowerTranslation;
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

			b2Float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->lowerImpulse;
			b2Float oldImpulse = joint->lowerImpulse;
			joint->lowerImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->lowerImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axisA );
			b2Float LA = impulse * a1;
			b2Float LB = impulse * a2;

			vA = b2MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = b2MulAdd( vB, mB, P );
			wB += iB * LB;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			b2Float C = joint->upperTranslation - translation;
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
			b2Float Cdot = b2Dot( axisA, b2Sub( vA, vB ) ) + a1 * wA - a2 * wB;
			b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->upperImpulse;
			b2Float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->upperImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axisA );
			b2Float LA = impulse * a1;
			b2Float LB = impulse * a2;

			// sign flipped on applied impulse
			vA = b2MulAdd( vA, mA, P );
			wA += iA * LA;
			vB = b2MulSub( vB, mB, P );
			wB -= iB * LB;
		}
	}

	// point to line constraint
	{
		b2Vec2 perpA = b2LeftPerp( axisA );

		b2Float bias = 0.0f;
		b2Float massScale = 1.0f;
		b2Float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Float C = b2Dot( perpA, d );
			bias = context->jointSoftness.biasRate * C;
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		b2Float s1 = b2Cross( b2Add( d, rA ), perpA );
		b2Float s2 = b2Cross( rB, perpA );
		b2Float Cdot = b2Dot( perpA, b2Sub( vB, vA ) ) + s2 * wB - s1 * wA;

		b2Float impulse = -massScale * joint->perpMass * ( Cdot + bias ) - impulseScale * joint->perpImpulse;
		joint->perpImpulse += impulse;

		b2Vec2 P = b2MulSV( impulse, perpA );
		b2Float LA = impulse * s1;
		b2Float LB = impulse * s2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2WheelJoint_Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2WheelJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
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

void b2DrawWheelJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	b2WheelJoint* joint = &base->wheelJoint;

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );
	b2Vec2 axis = b2RotateVector( transformA.q, joint->localAxisA );

	b2HexColor c1 = b2_colorGray;
	b2HexColor c2 = b2_colorGreen;
	b2HexColor c3 = b2_colorRed;
	b2HexColor c4 = b2_colorDimGray;
	b2HexColor c5 = b2_colorBlue;

	draw->DrawSegment( pA, pB, c5, draw->context );

	if ( joint->enableLimit )
	{
		b2Vec2 lower = b2MulAdd( pA, joint->lowerTranslation, axis );
		b2Vec2 upper = b2MulAdd( pA, joint->upperTranslation, axis );
		b2Vec2 perp = b2LeftPerp( axis );
		draw->DrawSegment( lower, upper, c1, draw->context );
		draw->DrawSegment( b2MulSub( lower, 0.1f, perp ), b2MulAdd( lower, 0.1f, perp ), c2, draw->context );
		draw->DrawSegment( b2MulSub( upper, 0.1f, perp ), b2MulAdd( upper, 0.1f, perp ), c3, draw->context );
	}
	else
	{
		draw->DrawSegment( b2MulSub( pA, 1.0f, axis ), b2MulAdd( pA, 1.0f, axis ), c1, draw->context );
	}

	draw->DrawPoint( pA, 5.0f, c1, draw->context );
	draw->DrawPoint( pB, 5.0f, c4, draw->context );
}
