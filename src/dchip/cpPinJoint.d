/*
 * Copyright (c) 2007-2013 Scott Lembcke and Howling Moon Software
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
module dchip.cpPinJoint;

import std.string;

import dchip.constraints_util;
import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.cpBody;
import dchip.cpConstraint;
import dchip.cpTransform;
import dchip.cpVect;

//~ const cpConstraintClass* cpPinJointGetClass();

/// @private
/* TODO : DELETE
struct cpPinJoint
{
    cpConstraint constraint;
    cpVect anchr1, anchr2;
    cpFloat dist = 0;

    cpVect r1, r2;
    cpVect n;
    cpFloat nMass = 0;

    cpFloat jnAcc = 0;
    cpFloat bias = 0;
}

mixin CP_DefineConstraintProperty!("cpPinJoint", cpVect, "anchr1", "Anchr1");
mixin CP_DefineConstraintProperty!("cpPinJoint", cpVect, "anchr2", "Anchr2");
mixin CP_DefineConstraintProperty!("cpPinJoint", cpFloat, "dist", "Dist");*/

void preStep(cpPinJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

	// TODO : DELETE
    /*joint.r1 = cpvrotate(joint.anchr1, a.rot);
    joint.r2 = cpvrotate(joint.anchr2, b.rot);*/
	
	// TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d and DELETE /*0*/
	/*joint.r1 = cpTransformVect(a.transform, cpvsub(joint.anchorA, a.cog));
	joint.r2 = cpTransformVect(b.transform, cpvsub(joint.anchorB, b.cog));*/
	/*0*/
	joint.r1 = cpvrotate(joint.anchorA, a.rot);
    joint.r2 = cpvrotate(joint.anchorB, b.rot);

    cpVect  delta = cpvsub(cpvadd(b.p, joint.r2), cpvadd(a.p, joint.r1));
    cpFloat dist  = cpvlength(delta);
    joint.n = cpvmult(delta, 1.0f / (dist ? dist : cast(cpFloat)INFINITY));

    // calculate mass normal
    joint.nMass = 1.0f / k_scalar(a, b, joint.r1, joint.r2, joint.n);

    // calculate bias velocity
    cpFloat maxBias = joint.constraint.maxBias;
    joint.bias = cpfclamp(-bias_coef(joint.constraint.errorBias, dt) * (dist - joint.dist) / dt, -maxBias, maxBias);
}

void applyCachedImpulse(cpPinJoint* joint, cpFloat dt_coef)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpVect j = cpvmult(joint.n, joint.jnAcc * dt_coef);
    apply_impulses(a, b, joint.r1, joint.r2, j);
}

void applyImpulse(cpPinJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;
    cpVect  n = joint.n;

    // compute relative velocity
    cpFloat vrn = normal_relative_velocity(a, b, joint.r1, joint.r2, n);

    cpFloat jnMax = joint.constraint.maxForce * dt;

    // compute normal impulse
    cpFloat jn    = (joint.bias - vrn) * joint.nMass;
    cpFloat jnOld = joint.jnAcc;
    joint.jnAcc = cpfclamp(jnOld + jn, -jnMax, jnMax);
    jn = joint.jnAcc - jnOld;

    // apply impulse
    apply_impulses(a, b, joint.r1, joint.r2, cpvmult(n, jn));
}

cpFloat getImpulse(cpPinJoint* joint)
{
    return cpfabs(joint.jnAcc);
}


__gshared cpConstraintClass klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );

/* TODO : DELETE
void _initModuleCtor_cpPinJoint()
{
    klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );
}

const(cpConstraintClass *) cpPinJointGetClass()
{
    return cast(cpConstraintClass*)&klass;
}*/

/// Allocate a pin joint.
cpPinJoint* cpPinJointAlloc()
{
    return cast(cpPinJoint*)cpcalloc(1, cpPinJoint.sizeof);
}

/// Initialize a pin joint.
// TODO : DELETE
//cpPinJoint* cpPinJointInit(cpPinJoint* joint, cpBody* a, cpBody* b, cpVect anchr1, cpVect anchr2)
cpPinJoint* cpPinJointInit(cpPinJoint* joint, cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB)
{
    cpConstraintInit(cast(cpConstraint*)joint, &klass, a, b);

    /* TODO : DELETE
	joint.anchr1 = anchr1;
    joint.anchr2 = anchr2;

    // STATIC_BODY_CHECK
    cpVect p1 = (a ? cpvadd(a.p, cpvrotate(anchr1, a.rot)) : anchr1);
    cpVect p2 = (b ? cpvadd(b.p, cpvrotate(anchr2, b.rot)) : anchr2);*/
	
	joint.anchorA = anchorA;
	joint.anchorB = anchorB;
		
	// TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d and DELETE /*0*/
	/+  

	// STATIC_BODY_CHECK
	cpVect p1 = (a ? cpTransformPoint(a.transform, anchorA) : anchorA);
	cpVect p2 = (b ? cpTransformPoint(b.transform, anchorB) : anchorB);+/

	/*0*/
    cpVect p1 = (a ? cpvadd(a.p, cpvrotate(anchorA, a.rot)) : anchorA);
    cpVect p2 = (b ? cpvadd(b.p, cpvrotate(anchorB, b.rot)) : anchorB);
	
    joint.dist = cpvlength(cpvsub(p2, p1));

    cpAssertWarn(joint.dist > 0.0, "You created a 0 length pin joint. A pivot joint will be much more stable.");

    joint.jnAcc = 0.0f;

    return joint;
}

/// Allocate and initialize a pin joint.
/* TODO : DELETE
cpConstraint* cpPinJointNew(cpBody* a, cpBody* b, cpVect anchr1, cpVect anchr2)
{
    return cast(cpConstraint*)cpPinJointInit(cpPinJointAlloc(), a, b, anchr1, anchr2);
}*/
cpConstraint* cpPinJointNew(cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB)
{
	return cast(cpConstraint*)cpPinJointInit(cpPinJointAlloc(), a, b, anchorA, anchorB);
}

/// Check if a constraint is a pin joint.
cpBool cpConstraintIsPinJoint(const cpConstraint* constraint)
{
	return (constraint.klass == &klass);
}

/// Get the location of the first anchor relative to the first body.
cpVect cpPinJointGetAnchorA(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	return (cast(cpPinJoint*)constraint).anchorA;
}

/// Set the location of the first anchor relative to the first body.
void cpPinJointSetAnchorA(cpConstraint* constraint, cpVect anchorA)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpPinJoint*)constraint).anchorA = anchorA;
}

/// Get the location of the second anchor relative to the second body.
cpVect cpPinJointGetAnchorB(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	return (cast(cpPinJoint*)constraint).anchorB;
}

/// Set the location of the second anchor relative to the second body.
void cpPinJointSetAnchorB(cpConstraint* constraint, cpVect anchorB)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpPinJoint*)constraint).anchorB = anchorB;
}

/// Get the distance the joint will maintain between the two anchors.
cpFloat cpPinJointGetDist(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	return (cast(cpPinJoint*)constraint).dist;
}

/// Set the distance the joint will maintain between the two anchors.
void cpPinJointSetDist(cpConstraint* constraint, cpFloat dist)
{
	cpAssertHard(cpConstraintIsPinJoint(constraint), "Constraint is not a pin joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpPinJoint*)constraint).dist = dist;
}

