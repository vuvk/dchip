/*
*  Copyright (c) 2007-2013 Scott Lembcke and Howling Moon Software
* 
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
* 
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
* 
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE.
*/
module dchip.cpDampedSpring;

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


//~ const cpConstraintClass* cpDampedSpringGetClass();

/*/// @private
TODO : DELETE

alias cpDampedSpringForceFunc = cpFloat function(cpConstraint* spring, cpFloat dist);

struct cpDampedSpring
{
    cpConstraint constraint;
    cpVect anchr1, anchr2;
    cpFloat restLength = 0;
    cpFloat stiffness = 0;
    cpFloat damping = 0;
    cpDampedSpringForceFunc springForceFunc;

    cpFloat target_vrn = 0;
    cpFloat v_coef = 0;

    cpVect r1, r2;
    cpFloat nMass = 0;
    cpVect n;

    cpFloat jAcc = 0;
}

mixin CP_DefineConstraintProperty!("cpDampedSpring", cpVect, "anchr1", "Anchr1");
mixin CP_DefineConstraintProperty!("cpDampedSpring", cpVect, "anchr2", "Anchr2");
mixin CP_DefineConstraintProperty!("cpDampedSpring", cpFloat, "restLength", "RestLength");
mixin CP_DefineConstraintProperty!("cpDampedSpring", cpFloat, "stiffness", "Stiffness");
mixin CP_DefineConstraintProperty!("cpDampedSpring", cpFloat, "damping", "Damping");
mixin CP_DefineConstraintProperty!("cpDampedSpring", cpDampedSpringForceFunc, "springForceFunc", "SpringForceFunc");*/


cpFloat defaultSpringForce(cpDampedSpring* spring, cpFloat dist)
{
    return (spring.restLength - dist)*  spring.stiffness;
}

void preStep(cpDampedSpring* spring, cpFloat dt)
{
    cpBody* a = spring.constraint.a;
    cpBody* b = spring.constraint.b;

	/* TODO : DELETE
    spring.r1 = cpvrotate(spring.anchr1, a.rot);
    spring.r2 = cpvrotate(spring.anchr2, b.rot);*/
	
	// TODO : UNCOMMENT AFTER ACTUALIZE cpBody.d AND DELETE* 0*
	spring.r1 = cpTransformVect(a.transform, cpvsub(spring.anchorA, a.cog));
	spring.r2 = cpTransformVect(b.transform, cpvsub(spring.anchorB, b.cog));
	/*0
    spring.r1 = cpvrotate(spring.anchorA, a.rot);
    spring.r2 = cpvrotate(spring.anchorB, b.rot);*/	

    cpVect  delta = cpvsub(cpvadd(b.p, spring.r2), cpvadd(a.p, spring.r1));
    cpFloat dist  = cpvlength(delta);
    spring.n = cpvmult(delta, 1.0f / (dist ? dist : INFINITY));

    cpFloat k = k_scalar(a, b, spring.r1, spring.r2, spring.n);
    cpAssertSoft(k != 0.0, "Unsolvable spring.");
    spring.nMass = 1.0f / k;

    spring.target_vrn = 0.0f;
    spring.v_coef     = 1.0f - cpfexp(-spring.damping*  dt*  k);

    // apply spring force
    cpFloat f_spring = spring.springForceFunc(cast(cpConstraint*)spring, dist);
    cpFloat j_spring = spring.jAcc = f_spring*  dt;
    apply_impulses(a, b, spring.r1, spring.r2, cpvmult(spring.n, j_spring));
}

void applyCachedImpulse(cpDampedSpring* spring, cpFloat dt_coef)
{
}

void applyImpulse(cpDampedSpring* spring, cpFloat dt)
{
    cpBody* a = spring.constraint.a;
    cpBody* b = spring.constraint.b;

    cpVect n  = spring.n;
    cpVect r1 = spring.r1;
    cpVect r2 = spring.r2;

    // compute relative velocity
    cpFloat vrn = normal_relative_velocity(a, b, r1, r2, n);

    // compute velocity loss from drag
    cpFloat v_damp = (spring.target_vrn - vrn)*  spring.v_coef;
    spring.target_vrn = vrn + v_damp;

    cpFloat j_damp = v_damp*  spring.nMass;
    spring.jAcc += j_damp;
    apply_impulses(a, b, spring.r1, spring.r2, cpvmult(spring.n, j_damp));
}

cpFloat getImpulse(cpDampedSpring* spring)
{
    return spring.jAcc;
}

__gshared cpConstraintClass klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );

/* TODO : DELETE
void _initModuleCtor_cpDampedSpring()
{
    klass = cpConstraintClass(
        cast(cpConstraintPreStepImpl)&preStep,
        cast(cpConstraintApplyCachedImpulseImpl)&applyCachedImpulse,
        cast(cpConstraintApplyImpulseImpl)&applyImpulse,
        cast(cpConstraintGetImpulseImpl)&getImpulse,
    );
};

const(cpConstraintClass*) cpDampedSpringGetClass()
{
    return cast(cpConstraintClass*)&klass;
}*/

/// Allocate a damped spring.
cpDampedSpring* cpDampedSpringAlloc()
{
    return cast(cpDampedSpring*)cpcalloc(1, cpDampedSpring.sizeof);
}

/// Initialize a damped spring.
/* TODO : DELETE
cpDampedSpring* cpDampedSpringInit(cpDampedSpring* spring, cpBody* a, cpBody* b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
    cpConstraintInit(cast(cpConstraint*)spring, cpDampedSpringGetClass(), a, b);

    spring.anchr1 = anchr1;
    spring.anchr2 = anchr2;*/
cpDampedSpring* cpDampedSpringInit(cpDampedSpring* spring, cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
	cpConstraintInit(cast(cpConstraint*)spring, &klass, a, b);
	
	spring.anchorA = anchorA;
	spring.anchorB = anchorB;

    spring.restLength      = restLength;
    spring.stiffness       = stiffness;
    spring.damping         = damping;
    spring.springForceFunc = cast(cpDampedSpringForceFunc)&defaultSpringForce;

    spring.jAcc = 0.0f;

    return spring;
}

/// Allocate and initialize a damped spring.
/* TODO : DELETE
cpConstraint* cpDampedSpringNew(cpBody* a, cpBody* b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
    return cast(cpConstraint*)cpDampedSpringInit(cpDampedSpringAlloc(), a, b, anchr1, anchr2, restLength, stiffness, damping);
}*/
cpConstraint* cpDampedSpringNew(cpBody* a, cpBody* b, cpVect anchorA, cpVect anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping)
{
	return cast(cpConstraint*)cpDampedSpringInit(cpDampedSpringAlloc(), a, b, anchorA, anchorB, restLength, stiffness, damping);
}

/// Check if a constraint is a Damped Spring.
cpBool cpConstraintIsDampedSpring(const cpConstraint* constraint)
{
	return (constraint.klass == &klass);
}

/// Get the location of the first anchor relative to the first body.
cpVect cpDampedSpringGetAnchorA(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).anchorA;
}

/// Set the location of the first anchor relative to the first body.
void cpDampedSpringSetAnchorA(cpConstraint* constraint, cpVect anchorA)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).anchorA = anchorA;
}

/// Get the location of the second anchor relative to the second body.
cpVect cpDampedSpringGetAnchorB(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).anchorB;
}

/// Set the location of the second anchor relative to the second body.
void cpDampedSpringSetAnchorB(cpConstraint* constraint, cpVect anchorB)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).anchorB = anchorB;
}

/// Get the rest length of the spring.
cpFloat cpDampedSpringGetRestLength(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).restLength;
}

/// Set the rest length of the spring.
void cpDampedSpringSetRestLength(cpConstraint* constraint, cpFloat restLength)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).restLength = restLength;
}

/// Get the stiffness of the spring in force/distance.
cpFloat cpDampedSpringGetStiffness(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).stiffness;
}

/// Set the stiffness of the spring in force/distance.
void cpDampedSpringSetStiffness(cpConstraint* constraint, cpFloat stiffness)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).stiffness = stiffness;
}

/// Get the damping of the spring.
cpFloat cpDampedSpringGetDamping(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).damping;
}

/// Set the damping of the spring.
void cpDampedSpringSetDamping(cpConstraint* constraint, cpFloat damping)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).damping = damping;
}

/// Get the damping of the spring.
cpDampedSpringForceFunc cpDampedSpringGetSpringForceFunc(const cpConstraint* constraint)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	return (cast(cpDampedSpring*)constraint).springForceFunc;
}

/// Set the damping of the spring.
void cpDampedSpringSetSpringForceFunc(cpConstraint* constraint, cpDampedSpringForceFunc springForceFunc)
{
	cpAssertHard(cpConstraintIsDampedSpring(constraint), "Constraint is not a damped spring.");
	cpConstraintActivateBodies(constraint);
	(cast(cpDampedSpring*)constraint).springForceFunc = springForceFunc;
}

