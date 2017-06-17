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
module dchip.cpRotaryLimitJoint;

import std.string;

import dchip.constraints_util;
import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.cpBody;
import dchip.cpConstraint;
import dchip.cpVect;

//~ const cpConstraintClass* cpRotaryLimitJointGetClass();

/// @private
// TODO : DELETE
/*struct cpRotaryLimitJoint
{
    cpConstraint constraint;
    cpFloat min = 0, max = 0;

    cpFloat iSum = 0;

    cpFloat bias = 0;
    cpFloat jAcc = 0;
}

mixin CP_DefineConstraintProperty!("cpRotaryLimitJoint", cpFloat, "min", "Min");
mixin CP_DefineConstraintProperty!("cpRotaryLimitJoint", cpFloat, "max", "Max");*/

void preStep(cpRotaryLimitJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpFloat dist  = b.a - a.a;
    cpFloat pdist = 0.0f;

    if (dist > joint.max)
    {
        pdist = joint.max - dist;
    }
    else if (dist < joint.min)
    {
        pdist = joint.min - dist;
    }

    // calculate moment of inertia coefficient.
    joint.iSum = 1.0f / (a.i_inv + b.i_inv);

    // calculate bias velocity
    cpFloat maxBias = joint.constraint.maxBias;
    joint.bias = cpfclamp(-bias_coef(joint.constraint.errorBias, dt) * pdist / dt, -maxBias, maxBias);

    // If the bias is 0, the joint is not at a limit. Reset the impulse.
    if (!joint.bias)
        joint.jAcc = 0.0f;
}

void applyCachedImpulse(cpRotaryLimitJoint* joint, cpFloat dt_coef)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpFloat j = joint.jAcc * dt_coef;
    a.w -= j * a.i_inv;
    b.w += j * b.i_inv;
}

void applyImpulse(cpRotaryLimitJoint* joint, cpFloat dt)
{
    if (!joint.bias)
        return;                  // early exit

    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    // compute relative rotational velocity
    cpFloat wr = b.w - a.w;

    cpFloat jMax = joint.constraint.maxForce * dt;

    // compute normal impulse
    cpFloat j    = -(joint.bias + wr) * joint.iSum;
    cpFloat jOld = joint.jAcc;

    if (joint.bias < 0.0f)
    {
        joint.jAcc = cpfclamp(jOld + j, 0.0f, jMax);
    }
    else
    {
        joint.jAcc = cpfclamp(jOld + j, -jMax, 0.0f);
    }
    j = joint.jAcc - jOld;

    // apply impulse
    a.w -= j * a.i_inv;
    b.w += j * b.i_inv;
}

cpFloat getImpulse(cpRotaryLimitJoint* joint)
{
    return cpfabs(joint.jAcc);
}

__gshared cpConstraintClass klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );

// TODO : DELETE
/*void _initModuleCtor_cpRotaryLimitJoint()
{
    klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );
}*/

const(cpConstraintClass *) cpRotaryLimitJointGetClass()
{
    return cast(cpConstraintClass*)&klass;
}

/// Allocate a damped rotary limit joint.
cpRotaryLimitJoint* cpRotaryLimitJointAlloc()
{
    return cast(cpRotaryLimitJoint*)cpcalloc(1, cpRotaryLimitJoint.sizeof);
}

/// Initialize a damped rotary limit joint.
cpRotaryLimitJoint* cpRotaryLimitJointInit(cpRotaryLimitJoint* joint, cpBody* a, cpBody* b, cpFloat min, cpFloat max)
{
    cpConstraintInit(cast(cpConstraint*)joint, &klass, a, b);

    joint.min = min;
    joint.max = max;

    joint.jAcc = 0.0f;

    return joint;
}

/// Allocate and initialize a damped rotary limit joint.
cpConstraint* cpRotaryLimitJointNew(cpBody* a, cpBody* b, cpFloat min, cpFloat max)
{
    return cast(cpConstraint*)cpRotaryLimitJointInit(cpRotaryLimitJointAlloc(), a, b, min, max);
}

/// Check if a constraint is a damped rotary springs.
cpBool cpConstraintIsRotaryLimitJoint(const cpConstraint* constraint)
{
	return (constraint.klass == &klass);
}

/// Get the minimum distance the joint will maintain between the two anchors.
cpFloat cpRotaryLimitJointGetMin(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsRotaryLimitJoint(constraint), "Constraint is not a rotary limit joint.");
	return (cast(cpRotaryLimitJoint*)constraint).min;
}

/// Set the minimum distance the joint will maintain between the two anchors.
void cpRotaryLimitJointSetMin(cpConstraint* constraint, cpFloat min)
{
	cpAssertHard(cpConstraintIsRotaryLimitJoint(constraint), "Constraint is not a rotary limit joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpRotaryLimitJoint*)constraint).min = min;
}

/// Get the maximum distance the joint will maintain between the two anchors.
cpFloat cpRotaryLimitJointGetMax(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsRotaryLimitJoint(constraint), "Constraint is not a rotary limit joint.");
	return (cast(cpRotaryLimitJoint*)constraint).max;
}

/// Set the maximum distance the joint will maintain between the two anchors.
void cpRotaryLimitJointSetMax(cpConstraint* constraint, cpFloat max)
{
	cpAssertHard(cpConstraintIsRotaryLimitJoint(constraint), "Constraint is not a rotary limit joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpRotaryLimitJoint*)constraint).max = max;
}
