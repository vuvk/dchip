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
module dchip.cpSlideJoint;

import std.string;

import dchip.constraints_util;
import dchip.chipmunk;

import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.cpBody;
import dchip.cpConstraint;
import dchip.cpVect;
import dchip.cpTransform;

//~ const cpConstraintClass* cpSlideJointGetClass();

/// @private
/* TODO : DELETE
struct cpSlideJoint
{
    cpConstraint constraint;
    cpVect anchr1, anchr2;
    cpFloat min = 0, max = 0;

    cpVect r1, r2;
    cpVect n;
    cpFloat nMass = 0;

    cpFloat jnAcc = 0;
    cpFloat bias = 0;
}

mixin CP_DefineConstraintProperty!("cpSlideJoint", cpVect, "anchr1", "Anchr1");
mixin CP_DefineConstraintProperty!("cpSlideJoint", cpVect, "anchr2", "Anchr2");
mixin CP_DefineConstraintProperty!("cpSlideJoint", cpFloat, "min", "Min");
mixin CP_DefineConstraintProperty!("cpSlideJoint", cpFloat, "max", "Max");*/

void preStep(cpSlideJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    /* TODO : DELETE
	joint.r1 = cpvrotate(joint.anchr1, a.rot);
    joint.r2 = cpvrotate(joint.anchr2, b.rot);*/
	
	joint.r1 = cpTransformVect(a.transform, cpvsub(joint.anchorA, a.cog));
	joint.r2 = cpTransformVect(b.transform, cpvsub(joint.anchorB, b.cog));
	
	/* TODO : DELETE
	joint.r1 = cpvrotate(joint.anchorA, a.rot);
    joint.r2 = cpvrotate(joint.anchorB, b.rot);*/
	
    cpVect  delta = cpvsub(cpvadd(b.p, joint.r2), cpvadd(a.p, joint.r1));
    cpFloat dist  = cpvlength(delta);
    cpFloat pdist = 0.0f;

    if (dist > joint.max)
    {
        pdist    = dist - joint.max;
        joint.n = cpvnormalize(delta);
    }
    else if (dist < joint.min)
    {
        pdist    = joint.min - dist;
        joint.n = cpvneg(cpvnormalize(delta));
    }
    else
    {
        joint.n     = cpvzero;
        joint.jnAcc = 0.0f;
    }

    // calculate mass normal
    joint.nMass = 1.0f / k_scalar(a, b, joint.r1, joint.r2, joint.n);

    // calculate bias velocity
    cpFloat maxBias = joint.constraint.maxBias;
    joint.bias = cpfclamp(-bias_coef(joint.constraint.errorBias, dt) * pdist / dt, -maxBias, maxBias);
}

void applyCachedImpulse(cpSlideJoint* joint, cpFloat dt_coef)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpVect j = cpvmult(joint.n, joint.jnAcc * dt_coef);
    apply_impulses(a, b, joint.r1, joint.r2, j);
}

void applyImpulse(cpSlideJoint* joint, cpFloat dt)
{
    if (cpveql(joint.n, cpvzero))
        return;                                // early exit

    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpVect n  = joint.n;
    cpVect r1 = joint.r1;
    cpVect r2 = joint.r2;

    // compute relative velocity
    cpVect  vr  = relative_velocity(a, b, r1, r2);
    cpFloat vrn = cpvdot(vr, n);

    // compute normal impulse
    cpFloat jn    = (joint.bias - vrn) * joint.nMass;
    cpFloat jnOld = joint.jnAcc;
    joint.jnAcc = cpfclamp(jnOld + jn, -joint.constraint.maxForce * dt, 0.0f);
    jn = joint.jnAcc - jnOld;

    // apply impulse
    apply_impulses(a, b, joint.r1, joint.r2, cpvmult(n, jn));
}

cpFloat getImpulse(cpConstraint* joint)
{
    return cpfabs((cast(cpSlideJoint*)joint).jnAcc);
}

__gshared cpConstraintClass klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );

/* TODO : DELETE
void _initModuleCtor_cpSlideJoint()
{
    klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );
}

const(cpConstraintClass *) cpSlideJointGetClass()
{
    return cast(cpConstraintClass*)&klass;
}*/

/// Allocate a slide joint.
cpSlideJoint* cpSlideJointAlloc()
{
    return cast(cpSlideJoint*)cpcalloc(1, cpSlideJoint.sizeof);
}

/// Initialize a slide joint.
cpSlideJoint* cpSlideJointInit(cpSlideJoint* joint, cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max)
{
    cpConstraintInit(cast(cpConstraint*)joint, &klass, a, b);

    joint.anchorA = anchorA;
    joint.anchorB = anchorB;
    joint.min    = min;
    joint.max    = max;

    joint.jnAcc = 0.0f;

    return joint;
}

/// Allocate and initialize a slide joint.
/* TODO : DELETE 
cpConstraint* cpSlideJointNew(cpBody* a, cpBody* b, cpVect anchr1, cpVect anchr2, cpFloat min, cpFloat max)
{
    return cast(cpConstraint*)cpSlideJointInit(cpSlideJointAlloc(), a, b, anchr1, anchr2, min, max);
}*/
cpConstraint* cpSlideJointNew(cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max)
{
	return cast(cpConstraint*) cpSlideJointInit(cpSlideJointAlloc(), a, b, anchorA, anchorB, min, max);
}

/// Check if a constraint is a slide joint.
cpBool cpConstraintIsSlideJoint(const cpConstraint* constraint)
{
	return (constraint.klass == &klass);
}

/// Get the location of the first anchor relative to the first body.
cpVect cpSlideJointGetAnchorA(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	return (cast(cpSlideJoint*)constraint).anchorA;
}

/// Set the location of the first anchor relative to the first body.
void cpSlideJointSetAnchorA(cpConstraint* constraint, cpVect anchorA)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpSlideJoint*)constraint).anchorA = anchorA;
}

/// Get the location of the second anchor relative to the second body.
cpVect cpSlideJointGetAnchorB(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	return (cast(cpSlideJoint*)constraint).anchorB;
}

/// Set the location of the second anchor relative to the second body.
void cpSlideJointSetAnchorB(cpConstraint* constraint, cpVect anchorB)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpSlideJoint*)constraint).anchorB = anchorB;
}

/// Get the minimum distance the joint will maintain between the two anchors.
cpFloat cpSlideJointGetMin(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	return (cast(cpSlideJoint*)constraint).min;
}

/// Set the minimum distance the joint will maintain between the two anchors.
void cpSlideJointSetMin(cpConstraint* constraint, cpFloat min)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpSlideJoint*)constraint).min = min;
}

/// Get the maximum distance the joint will maintain between the two anchors.
cpFloat cpSlideJointGetMax(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	return (cast(cpSlideJoint*)constraint).max;
}

/// Set the maximum distance the joint will maintain between the two anchors.
void cpSlideJointSetMax(cpConstraint* constraint, cpFloat max)
{
	cpAssertHard(cpConstraintIsSlideJoint(constraint), "Constraint is not a slide joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpSlideJoint*)constraint).max = max;
}
