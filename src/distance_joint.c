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

void b2DistanceJoint_SetLength( b2JointId jointId, b2Float length )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;

	joint->length = b2ClampFloat( length, B2_LINEAR_SLOP, B2_HUGE );
	joint->impulse = 0.0f;
	joint->lowerImpulse = 0.0f;
	joint->upperImpulse = 0.0f;
}

b2Float b2DistanceJoint_GetLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->length;
}

void b2DistanceJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	joint->enableLimit = enableLimit;
}

bool b2DistanceJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.enableLimit;
}

void b2DistanceJoint_SetLengthRange( b2JointId jointId, b2Float minLength, b2Float maxLength )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;

	minLength = b2ClampFloat( minLength, B2_LINEAR_SLOP, B2_HUGE );
	maxLength = b2ClampFloat( maxLength, B2_LINEAR_SLOP, B2_HUGE );
	joint->minLength = b2MinFloat( minLength, maxLength );
	joint->maxLength = b2MaxFloat( minLength, maxLength );
	joint->impulse = 0.0f;
	joint->lowerImpulse = 0.0f;
	joint->upperImpulse = 0.0f;
}

b2Float b2DistanceJoint_GetMinLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->minLength;
}

b2Float b2DistanceJoint_GetMaxLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->maxLength;
}

b2Float b2DistanceJoint_GetCurrentLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );

	b2World* world = b2GetWorld( jointId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return 0.0f;
	}

	b2Transform transformA = b2GetBodyTransform( world, base->bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, base->bodyIdB );

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );
	b2Vec2 d = b2Sub( pB, pA );
	b2Float length = b2Length( d );
	return length;
}

void b2DistanceJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.enableSpring = enableSpring;
}

bool b2DistanceJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return base->distanceJoint.enableSpring;
}

void b2DistanceJoint_SetSpringHertz( b2JointId jointId, b2Float hertz )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.hertz = hertz;
}

void b2DistanceJoint_SetSpringDampingRatio( b2JointId jointId, b2Float dampingRatio )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.dampingRatio = dampingRatio;
}

b2Float b2DistanceJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->hertz;
}

b2Float b2DistanceJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->dampingRatio;
}

void b2DistanceJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	if ( enableMotor != joint->distanceJoint.enableMotor )
	{
		joint->distanceJoint.enableMotor = enableMotor;
		joint->distanceJoint.motorImpulse = 0.0f;
	}
}

bool b2DistanceJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.enableMotor;
}

void b2DistanceJoint_SetMotorSpeed( b2JointId jointId, b2Float motorSpeed )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint->distanceJoint.motorSpeed = motorSpeed;
}

b2Float b2DistanceJoint_GetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.motorSpeed;
}

b2Float b2DistanceJoint_GetMotorForce( b2JointId jointId )
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return world->inv_h * base->distanceJoint.motorImpulse;
}

void b2DistanceJoint_SetMaxMotorForce( b2JointId jointId, b2Float force )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint->distanceJoint.maxMotorForce = force;
}

b2Float b2DistanceJoint_GetMaxMotorForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.maxMotorForce;
}

b2Vec2 b2GetDistanceJointForce( b2World* world, b2JointSim* base )
{
	b2DistanceJoint* joint = &base->distanceJoint;

	b2Transform transformA = b2GetBodyTransform( world, base->bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, base->bodyIdB );

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );
	b2Vec2 d = b2Sub( pB, pA );
	b2Vec2 axis = b2Normalize( d );
	b2Float force = ( joint->impulse + joint->lowerImpulse - joint->upperImpulse + joint->motorImpulse ) * world->inv_h;
	return b2MulSV( force, axis );
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void b2PrepareDistanceJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_distanceJoint );

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

	b2DistanceJoint* joint = &base->distanceJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// initial anchors in world space
	joint->anchorA = b2RotateVector( bodySimA->transform.q, b2Sub( base->localOriginAnchorA, bodySimA->localCenter ) );
	joint->anchorB = b2RotateVector( bodySimB->transform.q, b2Sub( base->localOriginAnchorB, bodySimB->localCenter ) );
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;
	b2Vec2 separation = b2Add( b2Sub( rB, rA ), joint->deltaCenter );
	b2Vec2 axis = b2Normalize( separation );

	// compute effective mass
	b2Float crA = b2Cross( rA, axis );
	b2Float crB = b2Cross( rB, axis );
	b2Float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->distanceSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->enableWarmStarting == false )
	{
		joint->impulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
	}
}

void b2WarmStartDistanceJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	b2Vec2 ds = b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) );
	b2Vec2 separation = b2Add( joint->deltaCenter, ds );
	b2Vec2 axis = b2Normalize( separation );

	b2Float axialImpulse = joint->impulse + joint->lowerImpulse - joint->upperImpulse + joint->motorImpulse;
	b2Vec2 P = b2MulSV( axialImpulse, axis );

	stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, P );
	stateA->angularVelocity -= iA * b2Cross( rA, P );
	stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, P );
	stateB->angularVelocity += iB * b2Cross( rB, P );
}

void b2SolveDistanceJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	b2Float mA = base->invMassA;
	b2Float mB = base->invMassB;
	b2Float iA = base->invIA;
	b2Float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	b2Float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	b2Float wB = stateB->angularVelocity;

	// current anchors
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	// current separation
	b2Vec2 ds = b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) );
	b2Vec2 separation = b2Add( joint->deltaCenter, ds );

	b2Float length = b2Length( separation );
	b2Vec2 axis = b2Normalize( separation );

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if ( joint->enableSpring && ( joint->minLength < joint->maxLength || joint->enableLimit == false ) )
	{
		// spring
		if ( joint->hertz > 0.0f )
		{
			// Cdot = dot(u, v + cross(w, r))
			b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
			b2Float Cdot = b2Dot( axis, vr );
			b2Float C = length - joint->length;
			b2Float bias = joint->distanceSoftness.biasRate * C;

			b2Float m = joint->distanceSoftness.massScale * joint->axialMass;
			b2Float impulse = -m * ( Cdot + bias ) - joint->distanceSoftness.impulseScale * joint->impulse;
			joint->impulse += impulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		if ( joint->enableLimit )
		{
			// lower limit
			{
				b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
				b2Float Cdot = b2Dot( axis, vr );

				b2Float C = length - joint->minLength;

				b2Float bias = 0.0f;
				b2Float massCoeff = 1.0f;
				b2Float impulseCoeff = 0.0f;
				if ( C > 0.0f )
				{
					// speculative
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = context->jointSoftness.biasRate * C;
					massCoeff = context->jointSoftness.massScale;
					impulseCoeff = context->jointSoftness.impulseScale;
				}

				b2Float impulse = -massCoeff * joint->axialMass * ( Cdot + bias ) - impulseCoeff * joint->lowerImpulse;
				b2Float newImpulse = b2MaxFloat( 0.0f, joint->lowerImpulse + impulse );
				impulse = newImpulse - joint->lowerImpulse;
				joint->lowerImpulse = newImpulse;

				b2Vec2 P = b2MulSV( impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}

			// upper
			{
				b2Vec2 vr = b2Add( b2Sub( vA, vB ), b2Sub( b2CrossSV( wA, rA ), b2CrossSV( wB, rB ) ) );
				b2Float Cdot = b2Dot( axis, vr );

				b2Float C = joint->maxLength - length;

				b2Float bias = 0.0f;
				b2Float massScale = 1.0f;
				b2Float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculative
					bias = C * context->inv_h;
				}
				else if ( useBias )
				{
					bias = context->jointSoftness.biasRate * C;
					massScale = context->jointSoftness.massScale;
					impulseScale = context->jointSoftness.impulseScale;
				}

				b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->upperImpulse;
				b2Float newImpulse = b2MaxFloat( 0.0f, joint->upperImpulse + impulse );
				impulse = newImpulse - joint->upperImpulse;
				joint->upperImpulse = newImpulse;

				b2Vec2 P = b2MulSV( -impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}
		}

		if ( joint->enableMotor )
		{
			b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
			b2Float Cdot = b2Dot( axis, vr );
			b2Float impulse = joint->axialMass * ( joint->motorSpeed - Cdot );
			b2Float oldImpulse = joint->motorImpulse;
			b2Float maxImpulse = context->h * joint->maxMotorForce;
			joint->motorImpulse = b2ClampFloat( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint->motorImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}
	}
	else
	{
		// rigid constraint
		b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
		b2Float Cdot = b2Dot( axis, vr );

		b2Float C = length - joint->length;

		b2Float bias = 0.0f;
		b2Float massScale = 1.0f;
		b2Float impulseScale = 0.0f;
		if ( useBias )
		{
			bias = context->jointSoftness.biasRate * C;
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		b2Float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->impulse;
		joint->impulse += impulse;

		b2Vec2 P = b2MulSV( impulse, axis );
		vA = b2MulSub( vA, mA, P );
		wA -= iA * b2Cross( rA, P );
		vB = b2MulAdd( vB, mB, P );
		wB += iB * b2Cross( rB, P );
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2DistanceJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2DistanceJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Dump("  jd.length = %.9g;\n", m_length);
	b2Dump("  jd.minLength = %.9g;\n", m_minLength);
	b2Dump("  jd.maxLength = %.9g;\n", m_maxLength);
	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
	b2Dump("  jd.damping = %.9g;\n", m_damping);
	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
#endif

void b2DrawDistanceJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	b2DistanceJoint* joint = &base->distanceJoint;

	b2Vec2 pA = b2TransformPoint( transformA, base->localOriginAnchorA );
	b2Vec2 pB = b2TransformPoint( transformB, base->localOriginAnchorB );

	b2Vec2 axis = b2Normalize( b2Sub( pB, pA ) );

	if ( joint->minLength < joint->maxLength && joint->enableLimit )
	{
		b2Vec2 pMin = b2MulAdd( pA, joint->minLength, axis );
		b2Vec2 pMax = b2MulAdd( pA, joint->maxLength, axis );
		b2Vec2 offset = b2MulSV( 0.05f * b2_lengthUnitsPerMeter, b2RightPerp( axis ) );

		if ( joint->minLength > B2_LINEAR_SLOP )
		{
			// draw->DrawPoint(pMin, 4.0f, c2, draw->context);
			draw->DrawSegment( b2Sub( pMin, offset ), b2Add( pMin, offset ), b2_colorLightGreen, draw->context );
		}

		if ( joint->maxLength < B2_HUGE )
		{
			// draw->DrawPoint(pMax, 4.0f, c3, draw->context);
			draw->DrawSegment( b2Sub( pMax, offset ), b2Add( pMax, offset ), b2_colorRed, draw->context );
		}

		if ( joint->minLength > B2_LINEAR_SLOP && joint->maxLength < B2_HUGE )
		{
			draw->DrawSegment( pMin, pMax, b2_colorGray, draw->context );
		}
	}

	draw->DrawSegment( pA, pB, b2_colorWhite, draw->context );
	draw->DrawPoint( pA, 4.0f, b2_colorWhite, draw->context );
	draw->DrawPoint( pB, 4.0f, b2_colorWhite, draw->context );

	if ( joint->hertz > 0.0f && joint->enableSpring )
	{
		b2Vec2 pRest = b2MulAdd( pA, joint->length, axis );
		draw->DrawPoint( pRest, 4.0f, b2_colorBlue, draw->context );
	}
}
