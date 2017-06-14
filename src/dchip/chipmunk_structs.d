/* Copyright (c) 2013 Scott Lembcke and Howling Moon Software
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

// All of the struct definitions for Chipmunk should be considered part of the private API.
// However, it is very valuable to know the struct sizes for preallocating memory.

module dchip.chipmunk_structs;

import std.string;

import dchip.cpBody;
import dchip.chipmunk;
import dchip.chipmunk_types;
import dchip.cpSpace;
import dchip.cpConstraint;

/*struct cpArray {
	int num, max;
	void **arr;
};

struct cpBody {
	// Integration functions
	cpBodyVelocityFunc velocity_func;
	cpBodyPositionFunc position_func;
	
	// mass and it's inverse
	cpFloat m;
	cpFloat m_inv;
	
	// moment of inertia and it's inverse
	cpFloat i;
	cpFloat i_inv;
	
	// center of gravity
	cpVect cog;
	
	// position, velocity, force
	cpVect p;
	cpVect v;
	cpVect f;
	
	// Angle, angular velocity, torque (radians)
	cpFloat a;
	cpFloat w;
	cpFloat t;
	
	cpTransform transform;
	
	cpDataPointer userData;
	
	// "pseudo-velocities" used for eliminating overlap.
	// Erin Catto has some papers that talk about what these are.
	cpVect v_bias;
	cpFloat w_bias;
	
	cpSpace *space;
	
	cpShape *shapeList;
	cpArbiter *arbiterList;
	cpConstraint *constraintList;
	
	struct {
		cpBody *root;
		cpBody *next;
		cpFloat idleTime;
	} sleeping;
};

enum cpArbiterState {
	// Arbiter is active and its the first collision.
	CP_ARBITER_STATE_FIRST_COLLISION,
	// Arbiter is active and its not the first collision.
	CP_ARBITER_STATE_NORMAL,
	// Collision has been explicitly ignored.
	// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
	CP_ARBITER_STATE_IGNORE,
	// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
	CP_ARBITER_STATE_CACHED,
	// Collison arbiter is invalid because one of the shapes was removed.
	CP_ARBITER_STATE_INVALIDATED,
};

struct cpArbiterThread {
	struct cpArbiter *next, *prev;
};

struct cpContact {
	cpVect r1, r2;
	
	cpFloat nMass, tMass;
	cpFloat bounce; // TODO: look for an alternate bounce solution.

	cpFloat jnAcc, jtAcc, jBias;
	cpFloat bias;
	
	cpHashValue hash;
};

struct cpCollisionInfo {
	const cpShape *a, *b;
	cpCollisionID id;
	
	cpVect n;
	
	int count;
	// TODO Should this be a unique struct type?
	struct cpContact *arr;
};

struct cpArbiter {
	cpFloat e;
	cpFloat u;
	cpVect surface_vr;
	
	cpDataPointer data;
	
	const cpShape *a, *b;
	cpBody *body_a, *body_b;
	struct cpArbiterThread thread_a, thread_b;
	
	int count;
	struct cpContact *contacts;
	cpVect n;
	
	// Regular, wildcard A and wildcard B collision handlers.
	cpCollisionHandler *handler, *handlerA, *handlerB;
	cpBool swapped;
	
	cpTimestamp stamp;
	enum cpArbiterState state;
};

struct cpShapeMassInfo {
	cpFloat m;
	cpFloat i;
	cpVect cog;
	cpFloat area;
};

typedef enum cpShapeType{
	CP_CIRCLE_SHAPE,
	CP_SEGMENT_SHAPE,
	CP_POLY_SHAPE,
	CP_NUM_SHAPES
} cpShapeType;

typedef cpBB (*cpShapeCacheDataImpl)(cpShape *shape, cpTransform transform);
typedef void (*cpShapeDestroyImpl)(cpShape *shape);
typedef void (*cpShapePointQueryImpl)(const cpShape *shape, cpVect p, cpPointQueryInfo *info);
typedef void (*cpShapeSegmentQueryImpl)(const cpShape *shape, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo *info);

typedef struct cpShapeClass cpShapeClass;

struct cpShapeClass {
	cpShapeType type;
	
	cpShapeCacheDataImpl cacheData;
	cpShapeDestroyImpl destroy;
	cpShapePointQueryImpl pointQuery;
	cpShapeSegmentQueryImpl segmentQuery;
};

struct cpShape {
	const cpShapeClass *klass;
	
	cpSpace *space;
	cpBody *body;
	struct cpShapeMassInfo massInfo;
	cpBB bb;
	
	cpBool sensor;
	
	cpFloat e;
	cpFloat u;
	cpVect surfaceV;

	cpDataPointer userData;
	
	cpCollisionType type;
	cpShapeFilter filter;
	
	cpShape *next;
	cpShape *prev;
	
	cpHashValue hashid;
};

struct cpCircleShape {
	cpShape shape;
	
	cpVect c, tc;
	cpFloat r;
};

struct cpSegmentShape {
	cpShape shape;
	
	cpVect a, b, n;
	cpVect ta, tb, tn;
	cpFloat r;
	
	cpVect a_tangent, b_tangent;
};

struct cpSplittingPlane {
	cpVect v0, n;
};

#define CP_POLY_SHAPE_INLINE_ALLOC 6

struct cpPolyShape {
	cpShape shape;
	
	cpFloat r;
	
	int count;
	// The untransformed planes are appended at the end of the transformed planes.
	struct cpSplittingPlane *planes;
	
	// Allocate a small number of splitting planes internally for simple poly.
	struct cpSplittingPlane _planes[2*CP_POLY_SHAPE_INLINE_ALLOC];
};*/


/// @private
// TODO : DELETE
/*struct cpConstraintClass
{
    cpConstraintPreStepImpl preStep;
    cpConstraintApplyCachedImpulseImpl applyCachedImpulse;
    cpConstraintApplyImpulseImpl applyImpulse;
    cpConstraintGetImpulseImpl getImpulse;
}*/

alias cpConstraintPreStepImpl = void function(cpConstraint* constraint, cpFloat dt);
alias cpConstraintApplyCachedImpulseImpl = void function(cpConstraint* constraint, cpFloat dt_coef);
alias cpConstraintApplyImpulseImpl = void function(cpConstraint* constraint, cpFloat dt);
alias cpConstraintGetImpulseImpl = cpFloat function(cpConstraint* constraint);

struct cpConstraintClass
{
    cpConstraintPreStepImpl preStep;
    cpConstraintApplyCachedImpulseImpl applyCachedImpulse;
    cpConstraintApplyImpulseImpl applyImpulse;
    cpConstraintGetImpulseImpl getImpulse;
}

struct cpConstraint
{
    version (CHIP_ALLOW_PRIVATE_ACCESS)
        /* const */ cpConstraintClass* klass;
    else
        package /* const */ cpConstraintClass* klass;
			
    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpSpace* space;
    else
        package cpSpace* space;

    /// The first body connected to this constraint.
    cpBody* a;

    /// The second body connected to this constraint.
    cpBody* b;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpConstraint* next_a;
    else
        package cpConstraint* next_a;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpConstraint* next_b;
    else
        package cpConstraint* next_b;

    /// The maximum force that this constraint is allowed to use.
    /// Defaults to infinity.
    cpFloat maxForce = 0;

    /// The rate at which joint error is corrected.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
    /// correct 10% of the error every 1/60th of a second.
    cpFloat errorBias = 0;

    /// The maximum rate at which joint error is corrected.
    /// Defaults to infinity.
    cpFloat maxBias = 0;
	
	cpBool collideBodies;

    /// Function called before the solver runs.
    /// Animate your joint anchors, update your motor torque, etc.
    cpConstraintPreSolveFunc preSolve;

    /// Function called after the solver runs.
    /// Use the applied impulse to perform effects like breakable joints.
    cpConstraintPostSolveFunc postSolve;

    /// User definable data pointer.
    /// Generally this points to your the game object class so you can access it
    /// when given a cpConstraint reference in a callback.
    cpDataPointer userData;
}

struct cpPinJoint {
	cpConstraint constraint;
	cpVect anchorA, anchorB;
	cpFloat dist = 0;
	
	cpVect r1, r2;
	cpVect n;
	cpFloat nMass = 0;
	
	cpFloat jnAcc = 0;
	cpFloat bias = 0;
}

struct cpSlideJoint
{
    cpConstraint constraint;
	cpVect anchorA, anchorB;
    cpFloat min = 0, max = 0;

    cpVect r1, r2;
    cpVect n;
    cpFloat nMass = 0;

    cpFloat jnAcc = 0;
    cpFloat bias = 0;
}

struct cpPivotJoint
{
    cpConstraint constraint;
    cpVect anchorA, anchorB;

    cpVect r1, r2;
    cpMat2x2 k;

    cpVect jAcc;
    cpVect bias;
}

struct cpGrooveJoint
{
    cpConstraint constraint;
    cpVect grv_n, grv_a, grv_b;
    cpVect anchorB;

    cpVect grv_tn;
    cpFloat clamp = 0;
    cpVect r1, r2;
    cpMat2x2 k;

    cpVect jAcc;
    cpVect bias;
}

/*struct cpDampedSpring {
	cpConstraint constraint;
	cpVect anchorA, anchorB;
	cpFloat restLength;
	cpFloat stiffness;
	cpFloat damping;
	cpDampedSpringForceFunc springForceFunc;
	
	cpFloat target_vrn;
	cpFloat v_coef;
	
	cpVect r1, r2;
	cpFloat nMass;
	cpVect n;
	
	cpFloat jAcc;
};

struct cpDampedRotarySpring {
	cpConstraint constraint;
	cpFloat restAngle;
	cpFloat stiffness;
	cpFloat damping;
	cpDampedRotarySpringTorqueFunc springTorqueFunc;
	
	cpFloat target_wrn;
	cpFloat w_coef;
	
	cpFloat iSum;
	cpFloat jAcc;
};*/

struct cpRotaryLimitJoint
{
    cpConstraint constraint;
    cpFloat min = 0, max = 0;

    cpFloat iSum = 0;

    cpFloat bias = 0;
    cpFloat jAcc = 0;
}

struct cpRatchetJoint
{
    cpConstraint constraint;
    cpFloat angle = 0, phase = 0, ratchet = 0;

    cpFloat iSum = 0;

    cpFloat bias = 0;
    cpFloat jAcc = 0;
}

struct cpGearJoint
{
    cpConstraint constraint;
    cpFloat phase = 0, ratio = 0;
    cpFloat ratio_inv = 0;

    cpFloat iSum = 0;

    cpFloat bias = 0;
    cpFloat jAcc = 0;
}

struct cpSimpleMotor 
{
	cpConstraint constraint;
	cpFloat rate = 0;
	
	cpFloat iSum = 0;
		
	cpFloat jAcc = 0;
}

/*typedef struct cpContactBufferHeader cpContactBufferHeader;
typedef void (*cpSpaceArbiterApplyImpulseFunc)(cpArbiter *arb);

struct cpSpace {
	int iterations;
	
	cpVect gravity;
	cpFloat damping;
	
	cpFloat idleSpeedThreshold;
	cpFloat sleepTimeThreshold;
	
	cpFloat collisionSlop;
	cpFloat collisionBias;
	cpTimestamp collisionPersistence;
	
	cpDataPointer userData;
	
	cpTimestamp stamp;
	cpFloat curr_dt;

	cpArray *dynamicBodies;
	cpArray *staticBodies;
	cpArray *rousedBodies;
	cpArray *sleepingComponents;
	
	cpHashValue shapeIDCounter;
	cpSpatialIndex *staticShapes;
	cpSpatialIndex *dynamicShapes;
	
	cpArray *constraints;
	
	cpArray *arbiters;
	cpContactBufferHeader *contactBuffersHead;
	cpHashSet *cachedArbiters;
	cpArray *pooledArbiters;
	
	cpArray *allocatedBuffers;
	unsigned int locked;
	
	cpBool usesWildcards;
	cpHashSet *collisionHandlers;
	cpCollisionHandler defaultHandler;
	
	cpBool skipPostStep;
	cpArray *postStepCallbacks;
	
	cpBody *staticBody;
	cpBody _staticBody;
};

typedef struct cpPostStepCallback {
	cpPostStepFunc func;
	void *key;
	void *data;
} cpPostStepCallback;*/
