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
module dchip.cpGrooveJoint;

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

//~ const cpConstraintClass* cpGrooveJointGetClass();

/// @private
/* TODO : DELETE
struct cpGrooveJoint
{
    cpConstraint constraint;
    cpVect grv_n, grv_a, grv_b;
    cpVect anchr2;

    cpVect grv_tn;
    cpFloat clamp = 0;
    cpVect r1, r2;
    cpMat2x2 k;

    cpVect jAcc;
    cpVect bias;
}

mixin CP_DefineConstraintGetter!("cpGrooveJoint", cpVect, "grv_a", "GrooveA");
mixin CP_DefineConstraintGetter!("cpGrooveJoint", cpVect, "grv_b", "GrooveB");
mixin CP_DefineConstraintProperty!("cpGrooveJoint", cpVect, "anchr2", "Anchr2");
*/

/// Set endpoint a of a groove joint's groove
void cpGrooveJointSetGrooveA(cpConstraint* constraint, cpVect value);
/// Set endpoint b of a groove joint's groove
void cpGrooveJointSetGrooveB(cpConstraint* constraint, cpVect value);

void preStep(cpGrooveJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    // calculate endpoints in worldspace
	/* TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d and DELETE *0*
	cpVect ta = cpTransformPoint(a.transform, joint.grv_a);
	cpVect tb = cpTransformPoint(a.transform, joint.grv_b);*/
	/*0*/
    cpVect ta = cpBodyLocal2World(a, joint.grv_a);
    cpVect tb = cpBodyLocal2World(a, joint.grv_b);

    // calculate axis
	/* TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d and DELETE *0*
	cpVect n = cpTransformVect(a.transform, joint.grv_n);*/	
	/*0*/	
	cpVect  n = cpvrotate(joint.grv_n, a.rot);
    cpFloat d = cpvdot(ta, n);

    joint.grv_tn = n;
    /* TODO : DELETE
	joint.r2     = cpvrotate(joint.anchr2, b.rot); */
	/* TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d and DELETE *0* 	
	joint.r2 = cpTransformVect(b.transform, cpvsub(joint.anchorB, b.cog));*/
	/*0*/
	joint.r2 = cpvrotate (joint.anchorB, b.rot);
	
    // calculate tangential distance along the axis of r2
    cpFloat td = cpvcross(cpvadd(b.p, joint.r2), n);

    // calculate clamping factor and r2
    if (td <= cpvcross(ta, n))
    {
        joint.clamp = 1.0f;
        joint.r1    = cpvsub(ta, a.p);
    }
    else if (td >= cpvcross(tb, n))
    {
        joint.clamp = -1.0f;
        joint.r1    = cpvsub(tb, a.p);
    }
    else
    {
        joint.clamp = 0.0f;
        joint.r1    = cpvsub(cpvadd(cpvmult(cpvperp(n), -td), cpvmult(n, d)), a.p);
    }

    // Calculate mass tensor
    joint.k = k_tensor(a, b, joint.r1, joint.r2);

    // calculate bias velocity
    cpVect delta = cpvsub(cpvadd(b.p, joint.r2), cpvadd(a.p, joint.r1));
    joint.bias = cpvclamp(cpvmult(delta, -bias_coef(joint.constraint.errorBias, dt) / dt), joint.constraint.maxBias);
}

void applyCachedImpulse(cpGrooveJoint* joint, cpFloat dt_coef)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    apply_impulses(a, b, joint.r1, joint.r2, cpvmult(joint.jAcc, dt_coef));
}

cpVect grooveConstrain(cpGrooveJoint* joint, cpVect j, cpFloat dt)
{
    cpVect n      = joint.grv_tn;
    cpVect jClamp = (joint.clamp * cpvcross(j, n) > 0.0f) ? j : cpvproject(j, n);
    return cpvclamp(jClamp, joint.constraint.maxForce * dt);
}

void applyImpulse(cpGrooveJoint* joint, cpFloat dt)
{
    cpBody* a = joint.constraint.a;
    cpBody* b = joint.constraint.b;

    cpVect r1 = joint.r1;
    cpVect r2 = joint.r2;

    // compute impulse
    cpVect vr = relative_velocity(a, b, r1, r2);

    cpVect j    = cpMat2x2Transform(joint.k, cpvsub(joint.bias, vr));
    cpVect jOld = joint.jAcc;
    joint.jAcc = grooveConstrain(joint, cpvadd(jOld, j), dt);
    j = cpvsub(joint.jAcc, jOld);

    // apply impulse
    apply_impulses(a, b, joint.r1, joint.r2, j);
}

cpFloat getImpulse(cpGrooveJoint* joint)
{
    return cpvlength(joint.jAcc);
}

__gshared cpConstraintClass klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );

/* TODO : DELETE
void _initModuleCtor_cpGrooveJoint()
{
    klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );
}

const(cpConstraintClass *) cpGrooveJointGetClass()
{
    return cast(cpConstraintClass*)&klass;
}*/

/// Allocate a groove joint.
cpGrooveJoint* cpGrooveJointAlloc()
{
    return cast(cpGrooveJoint*)cpcalloc(1, cpGrooveJoint.sizeof);
}

/// Initialize a groove joint.
cpGrooveJoint* cpGrooveJointInit(cpGrooveJoint* joint, cpBody* a, cpBody* b, cpVect groove_a, cpVect groove_b, cpVect anchorB)
{
    cpConstraintInit(cast(cpConstraint*)joint, &klass, a, b);

    joint.grv_a  = groove_a;
    joint.grv_b  = groove_b;
    joint.grv_n  = cpvperp(cpvnormalize(cpvsub(groove_b, groove_a)));
    joint.anchorB = anchorB;

    joint.jAcc = cpvzero;

    return joint;
}

/// Allocate and initialize a groove joint.
/*TODO : DELETE
cpConstraint* cpGrooveJointNew(cpBody* a, cpBody* b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
{
    return cast(cpConstraint*)cpGrooveJointInit(cpGrooveJointAlloc(), a, b, groove_a, groove_b, anchr2);
}*/
cpConstraint* cpGrooveJointNew(cpBody* a, cpBody* b, cpVect groove_a, cpVect groove_b, cpVect anchorB)
{
	return cast(cpConstraint*)cpGrooveJointInit(cpGrooveJointAlloc(), a, b, groove_a, groove_b, anchorB);
}

/// Check if a constraint is a groove joint.
cpBool cpConstraintIsGrooveJoint(const cpConstraint* constraint)
{
	return (constraint.klass == &klass);
}

/// Get the first endpoint of the groove relative to the first body.
cpVect cpGrooveJointGetGrooveA(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	return (cast(cpGrooveJoint*)constraint).grv_a;
}

/// Set the first endpoint of the groove relative to the first body.
void cpGrooveJointSetGrooveA(cpConstraint* constraint, cpVect value)
{
    cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	cpGrooveJoint* g = cast(cpGrooveJoint*)constraint;
 
    g.grv_a = value;
    g.grv_n = cpvperp(cpvnormalize(cpvsub(g.grv_b, value)));

    cpConstraintActivateBodies(constraint);
}

/// Get the first endpoint of the groove relative to the first body.
cpVect cpGrooveJointGetGrooveB(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	return (cast(cpGrooveJoint*)constraint).grv_b;
}

/// Set the first endpoint of the groove relative to the first body.
void cpGrooveJointSetGrooveB(cpConstraint* constraint, cpVect value)
{
    cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	cpGrooveJoint* g = cast(cpGrooveJoint*)constraint;

    g.grv_b = value;
    g.grv_n = cpvperp(cpvnormalize(cpvsub(value, g.grv_a)));

    cpConstraintActivateBodies(constraint);
}

/// Get the location of the second anchor relative to the second body.
cpVect cpGrooveJointGetAnchorB(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	return (cast(cpGrooveJoint*)constraint).anchorB;
}

/// Set the location of the second anchor relative to the second body.
void cpGrooveJointSetAnchorB(cpConstraint* constraint, cpVect anchorB)
{
	cpAssertHard(cpConstraintIsGrooveJoint(constraint), "Constraint is not a groove joint.");
	cpConstraintActivateBodies(constraint);
	(cast(cpGrooveJoint*)constraint).anchorB = anchorB;
}
