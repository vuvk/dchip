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
module dchip.chipmunk_private;

import std.string;

import dchip.cpArray;
import dchip.cpBody;
import dchip.chipmunk;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.cpArbiter;
import dchip.cpConstraint;
import dchip.cpHashSet;
import dchip.cpPolyShape;
import dchip.cpShape;
import dchip.cpSpace;
import dchip.cpSpaceComponent;
import dchip.cpSpatialIndex;
import dchip.cpVect;
import dchip.util;

enum CP_ALLOW_PRIVATE_ACCESS = 1;

enum CP_HASH_COEF = 3344921057uL;

cpHashValue CP_HASH_PAIR(T1, T2)(T1 a, T2 b)
{
    return cast(cpHashValue)(cast(cpHashValue)a* CP_HASH_COEF ^ cast(cpHashValue)b* CP_HASH_COEF);
}

// TODO: Eww. Magic numbers.
enum MAGIC_EPSILON = 1e-5;


/+ TODO : DELETE??
cpHashSet* cpHashSetNew(int size, cpHashSetEqlFunc eqlFunc);
void cpHashSetSetDefaultValue(cpHashSet* set, void* default_value);

void cpHashSetFree(cpHashSet* set);

int cpHashSetCount(cpHashSet* set);
void* cpHashSetInsert(cpHashSet* set, cpHashValue hash, void* ptr, cpHashSetTransFunc trans, void* data);
void* cpHashSetRemove(cpHashSet* set, cpHashValue hash, void* ptr);
void* cpHashSetFind(cpHashSet* set, cpHashValue hash, void* ptr);

void cpHashSetEach(cpHashSet* set, cpHashSetIteratorFunc func, void* data);

void cpHashSetFilter(cpHashSet* set, cpHashSetFilterFunc func, void* data);+/

/+ TODO : DELETE???
//MARK: Bodies

void cpBodyAddShape(cpBody* body, cpShape* shape);
void cpBodyRemoveShape(cpBody* body, cpShape* shape);

//void cpBodyAccumulateMassForShape(cpBody* body, cpShape* shape);
void cpBodyAccumulateMassFromShapes(cpBody* body);

void cpBodyRemoveConstraint(cpBody* body, cpConstraint* constraint);


//MARK: Spatial Index Functions

cpSpatialIndex* cpSpatialIndexInit(cpSpatialIndex* index, cpSpatialIndexClass* klass, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex* staticIndex);



cpArbiter* cpArbiterInit(cpArbiter* arb, cpShape* a, cpShape* b);+/

//MARK: Arbiters

static cpArbiterThread* cpArbiterThreadForBody(cpArbiter* arb, cpBody* body_)
{
	return (arb.body_a == body_ ? &arb.thread_a : &arb.thread_b);
}

/+ TODO : DELETE???
void cpArbiterUnthread(cpArbiter* arb);

void cpArbiterUpdate(cpArbiter* arb, struct cpCollisionInfo* info, cpSpace* space);
void cpArbiterPreStep(cpArbiter* arb, cpFloat dt, cpFloat bias, cpFloat slop);
void cpArbiterApplyCachedImpulse(cpArbiter* arb, cpFloat dt_coef);
void cpArbiterApplyImpulse(cpArbiter* arb);


//MARK: Shapes/Collisions

cpShape* cpShapeInit(cpShape* shape, const cpShapeClass* klass, cpBody* body, struct cpShapeMassInfo massInfo);+/

static cpBool cpShapeActive(cpShape* shape)
{
	// checks if the shape is added to a shape list.
	// TODO could this just check the space now?
	return (shape.prev || (shape.body_ && shape.body_.shapeList == shape));
}

/+ TODO : DELETE
// Note: This function returns contact points with r1/r2 in absolute coordinates, not body relative.
struct cpCollisionInfo cpCollide(const cpShape* a, const cpShape* b, cpCollisionID id, struct cpContact* contacts);+/

static void CircleSegmentQuery(cpShape* shape, cpVect center, cpFloat r1, cpVect a, cpVect b, cpFloat r2, cpSegmentQueryInfo* info)
{
	cpVect da = cpvsub(a, center);
	cpVect db = cpvsub(b, center);
	cpFloat rsum = r1 + r2;
	
	cpFloat qa = cpvdot(da, da) - 2.0f*cpvdot(da, db) + cpvdot(db, db);
	cpFloat qb = cpvdot(da, db) - cpvdot(da, da);
	cpFloat det = qb*qb - qa*(cpvdot(da, da) - rsum*rsum);
	
	if(det >= 0.0f)
	{
		cpFloat t = (-qb - cpfsqrt(det))/(qa);
		if(0.0f<= t && t <= 1.0f)
		{
			cpVect n = cpvnormalize(cpvlerp(da, db, t));
			
			info.shape = shape;
			info.point = cpvsub(cpvlerp(a, b, t), cpvmult(n, r2));
			info.normal = n;
			info.alpha = t;
		}
	}
}

static cpBool cpShapeFilterReject(cpShapeFilter a, cpShapeFilter b)
{
	// Reject the collision if:
	return (
		// They are in the same non-zero group.
		(a.group != 0 && a.group == b.group) ||
		// One of the category/mask combinations fails.
		(a.categories & b.mask) == 0 ||
		(b.categories & a.mask) == 0
	);
}

/+ TODO : DELETE
void cpLoopIndexes(const cpVect* verts, int count, int* start, int* end);


// TODO naming conventions here

void cpConstraintInit(cpConstraint* constraint, const struct cpConstraintClass* klass, cpBody* a, cpBody* b);+/

//MARK: Constraints
static void cpConstraintActivateBodies(cpConstraint* constraint)
{
	cpBody* a = constraint.a; cpBodyActivate(a);
	cpBody* b = constraint.b; cpBodyActivate(b);
}

static cpVect relative_velocity(cpBody* a, cpBody* b, cpVect r1, cpVect r2)
{
	cpVect v1_sum = cpvadd(a.v, cpvmult(cpvperp(r1), a.w));
	cpVect v2_sum = cpvadd(b.v, cpvmult(cpvperp(r2), b.w));
	
	return cpvsub(v2_sum, v1_sum);
}

static cpFloat normal_relative_velocity(cpBody* a, cpBody* b, cpVect r1, cpVect r2, cpVect n)
{
	return cpvdot(relative_velocity(a, b, r1, r2), n);
}

static void apply_impulse(cpBody* body_, cpVect j, cpVect r)
{
	body_.v = cpvadd(body_.v, cpvmult(j, body_.m_inv));
	body_.w += body_.i_inv*cpvcross(r, j);
}

static void apply_impulses(cpBody* a , cpBody* b, cpVect r1, cpVect r2, cpVect j)
{
	apply_impulse(a, cpvneg(j), r1);
	apply_impulse(b, j, r2);
}

static void apply_bias_impulse(cpBody* body_, cpVect j, cpVect r)
{
	body_.v_bias = cpvadd(body_.v_bias, cpvmult(j, body_.m_inv));
	body_.w_bias += body_.i_inv*cpvcross(r, j);
}

static void apply_bias_impulses(cpBody* a, cpBody* b, cpVect r1, cpVect r2, cpVect j)
{
	apply_bias_impulse(a, cpvneg(j), r1);
	apply_bias_impulse(b, j, r2);
}

static cpFloat k_scalar_body(cpBody* body_, cpVect r, cpVect n)
{
	cpFloat rcn = cpvcross(r, n);
	return body_.m_inv + body_.i_inv*rcn*rcn;
}

static cpFloat k_scalar(cpBody* a, cpBody* b, cpVect r1, cpVect r2, cpVect n)
{
	cpFloat value = k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);
	cpAssertSoft(value != 0.0, "Unsolvable collision or constraint.");
	
	return value;
}

static cpMat2x2 k_tensor(cpBody* a, cpBody* b, cpVect r1, cpVect r2)
{
	cpFloat m_sum = a.m_inv + b.m_inv;
	
	// start with Identity*m_sum
	cpFloat k11 = m_sum, k12 = 0.0f;
	cpFloat k21 = 0.0f,  k22 = m_sum;
	
	// add the influence from r1
	cpFloat a_i_inv = a.i_inv;
	cpFloat r1xsq =  r1.x * r1.x * a_i_inv;
	cpFloat r1ysq =  r1.y * r1.y * a_i_inv;
	cpFloat r1nxy = -r1.x * r1.y * a_i_inv;
	k11 += r1ysq; k12 += r1nxy;
	k21 += r1nxy; k22 += r1xsq;
	
	// add the influnce from r2
	cpFloat b_i_inv = b.i_inv;
	cpFloat r2xsq =  r2.x * r2.x * b_i_inv;
	cpFloat r2ysq =  r2.y * r2.y * b_i_inv;
	cpFloat r2nxy = -r2.x * r2.y * b_i_inv;
	k11 += r2ysq; k12 += r2nxy;
	k21 += r2nxy; k22 += r2xsq;
	
	// invert
	cpFloat det = k11*k22 - k12*k21;
	cpAssertSoft(det != 0.0, "Unsolvable constraint.");
	
	cpFloat det_inv = 1.0f/det;
	return cpMat2x2New(
		 k22*det_inv, -k12*det_inv,
		-k21*det_inv,  k11*det_inv
 	);
}

static cpFloat bias_coef(cpFloat errorBias, cpFloat dt)
{
	return 1.0f - cpfpow(errorBias, dt);
}


//MARK: Spaces

void cpAssertSpaceUnlocked(cpSpace* space) 
{
	cpAssertHard(!space.locked, "This operation cannot be done safely during a call to cpSpaceStep() or during a query. Put these calls into a post-step callback.");
}

/+ TODO : DELETE
void cpSpaceSetStaticBody(cpSpace* space, cpBody* body);

extern cpCollisionHandler cpCollisionHandlerDoNothing;

void cpSpaceProcessComponents(cpSpace* space, cpFloat dt);

void cpSpacePushFreshContactBuffer(cpSpace* space);
struct cpContact* cpContactBufferGetArray(cpSpace* space);
void cpSpacePushContacts(cpSpace* space, int count);

cpPostStepCallback* cpSpaceGetPostStepCallback(cpSpace* space, void* key);

cpBool cpSpaceArbiterSetFilter(cpArbiter* arb, cpSpace* space);
void cpSpaceFilterArbiters(cpSpace* space, cpBody* body, cpShape* filter);

void cpSpaceActivateBody(cpSpace* space, cpBody* body);
void cpSpaceLock(cpSpace* space);
void cpSpaceUnlock(cpSpace* space, cpBool runPostStep);+/

static void cpSpaceUncacheArbiter(cpSpace* space, cpArbiter* arb)
{
	cpShape* a = arb.a;
	cpShape* b = arb.b;
	cpShape* [] shape_pair = [a, b];
	cpHashValue arbHashID = CP_HASH_PAIR(cast(cpHashValue)a, cast(cpHashValue)b);
	cpHashSetRemove(space.cachedArbiters, arbHashID, cast(cpShape*)shape_pair);
	cpArrayDeleteObj(space.arbiters, arb);
}

static cpArray* cpSpaceArrayForBodyType(cpSpace* space, cpBodyType type)
{
	return (type == cpBodyType.CP_BODY_TYPE_STATIC ? space.staticBodies : space.dynamicBodies);
}

/+ TODO : DELETE
void cpShapeUpdateFunc(cpShape* shape, void* unused);
cpCollisionID cpSpaceCollideShapes(cpShape* a, cpShape* b, cpCollisionID id, cpSpace* space);+/


//MARK: Foreach loops

static cpConstraint* cpConstraintNext(cpConstraint* node, cpBody* body_)
{
	return (node.a == body_ ? node.next_a : node.next_b);
}

template CP_BODY_FOREACH_CONSTRAINT(string bdy, string var, string code)
{
    enum CP_BODY_FOREACH_CONSTRAINT = format("for (cpConstraint* %2$s = %1$s.constraintList; %2$s; %2$s = cpConstraintNext(%2$s, %1$s)) { %3$s }",
             bdy, var, code);
}

static cpArbiter* cpArbiterNext(cpArbiter* node, cpBody* body_)
{
	return (node.body_a == body_ ? node.thread_a.next : node.thread_b.next);
}
	
template CP_BODY_FOREACH_ARBITER(string bdy, string var, string code)
{
    enum CP_BODY_FOREACH_ARBITER = format("for (cpArbiter* %2$s = %1$s.arbiterList; %2$s; %2$s = cpArbiterNext(%2$s, %1$s)) { %3$s }",
             bdy, var, code);
}
	
template CP_BODY_FOREACH_SHAPE(string body_, string var, string code)
{
    enum CP_BODY_FOREACH_SHAPE = format("for (cpShape* %2$s = %1$s.shapeList; %2$s; %2$s = %2$s.next) { %3$s }",
            body_, var, code);
}

template CP_BODY_FOREACH_COMPONENT(string root, string var, string code)
{
    enum CP_BODY_FOREACH_COMPONENT = format("for (cpBody* %2$s = %1$s; %2$s; %2$s = %2$s.sleeping.next) { %3$s }",
             root, var, code);
}


/+ TODO : DELETE
enum CP_HASH_COEF = 3344921057uL;

cpHashValue CP_HASH_PAIR(T1, T2)(T1 a, T2 b)
{
    return cast(cpHashValue)(cast(cpHashValue)a* CP_HASH_COEF ^ cast(cpHashValue)b* CP_HASH_COEF);
}

// TODO: Eww. Magic numbers.
enum MAGIC_EPSILON = 1e-5;

/* TODO : DELETE
struct cpArray
{
    int num, max;
    void** arr;
}*/

cpConstraint* cpConstraintNext(cpConstraint* node, cpBody* bdy)
{
    return (node.a == bdy ? node.next_a : node.next_b);
}

template CP_BODY_FOREACH_CONSTRAINT(string bdy, string var, string code)
{
    enum CP_BODY_FOREACH_CONSTRAINT = format("for (cpConstraint* %2$s = %1$s.constraintList; %2$s; %2$s = cpConstraintNext(%2$s, %1$s)) { %3$s }",
             bdy, var, code);
}

cpArbiter* cpArbiterNext(cpArbiter* node, cpBody* bdy)
{
    return (node.body_a == bdy ? node.thread_a.next : node.thread_b.next);
}

template CP_BODY_FOREACH_ARBITER(string bdy, string var, string code)
{
    enum CP_BODY_FOREACH_ARBITER = format("for (cpArbiter* %2$s = %1$s.arbiterList; %2$s; %2$s = cpArbiterNext(%2$s, %1$s)) { %3$s }",
             bdy, var, code);
}

template CP_BODY_FOREACH_SHAPE(string bdy, string var, string code)
{
    enum CP_BODY_FOREACH_SHAPE = format("for (cpShape* %2$s = %1$s.shapeList; %2$s; %2$s = %2$s.next) { %3$s }",
            bdy, var, code);
}

template CP_BODY_FOREACH_COMPONENT(string root, string var, string code)
{
    enum CP_BODY_FOREACH_COMPONENT = format("for (cpBody* %2$s = %1$s; %2$s; %2$s = %2$s.node.next) { %3$s }",
             root, var, code);
}

alias cpHashSetEqlFunc = cpBool function(void* ptr, void* elt);
alias cpHashSetTransFunc = void* function(void* ptr, void* data);

alias cpHashSetIteratorFunc = void function(void* elt, void* data);
alias cpHashSetFilterFunc = cpBool function(void* elt, void* data);

// TODO should move this to the cpVect API. It's pretty useful.
cpVect cpClosetPointOnSegment(const cpVect p, const cpVect a, const cpVect b)
{
    cpVect  delta = cpvsub(a, b);
    cpFloat t     = cpfclamp01(cpvdot(delta, cpvsub(p, b)) / cpvlengthsq(delta));
    return cpvadd(b, cpvmult(delta, t));
}

cpBool cpShapeActive(cpShape* shape)
{
    return shape.prev || (shape.body_ && shape.body_.shapeList == shape);
}

void CircleSegmentQuery(cpShape* shape, cpVect center, cpFloat r, cpVect a, cpVect b, cpSegmentQueryInfo* info)
{
    cpVect da = cpvsub(a, center);
    cpVect db = cpvsub(b, center);

    cpFloat qa = cpvdot(da, da) - 2.0f* cpvdot(da, db) + cpvdot(db, db);
    cpFloat qb = -2.0f* cpvdot(da, da) + 2.0f* cpvdot(da, db);
    cpFloat qc = cpvdot(da, da) - r* r;

    cpFloat det = qb* qb - 4.0f* qa* qc;

    if (det >= 0.0f)
    {
        cpFloat t = (-qb - cpfsqrt(det)) / (2.0f* qa);

        if (0.0f <= t && t <= 1.0f)
        {
            info.shape = shape;
            info.t     = t;
            info.n     = cpvnormalize(cpvlerp(da, db, t));
        }
    }
}

// TODO doesn't really need to be inline, but need a better place to put this function
cpSplittingPlane cpSplittingPlaneNew(cpVect a, cpVect b)
{
    cpVect n = cpvnormalize(cpvperp(cpvsub(b, a)));
    cpSplittingPlane plane = { n, cpvdot(n, a) };
    return plane;
}

cpFloat cpSplittingPlaneCompare(cpSplittingPlane plane, cpVect v)
{
    return cpvdot(plane.n, v) - plane.d;
}

/* TODO : DELETE
struct cpPostStepCallback
{
    cpPostStepFunc func;
    void* key;
    void* data;
}*/

cpCollisionHandler* cpSpaceLookupHandler(cpSpace* space, cpCollisionType a, cpCollisionType b)
{
    cpCollisionType[2] types = void;
    types[0] = a;
    types[1] = b;
    return cast(cpCollisionHandler*)cpHashSetFind(space.collisionHandlers, CP_HASH_PAIR(a, b), types.ptr);
}

//MARK: Constraints
// TODO naming conventions here

static void cpConstraintActivateBodies (cpConstraint* constraint)
{
	cpBody* a = constraint.a; cpBodyActivate(a);
	cpBody* b = constraint.b; cpBodyActivate(b);
}

void cpSpaceUncacheArbiter(cpSpace* space, cpArbiter* arb)
{
    cpShape* a = arb.a, b = arb.b;
    cpShape*[2] shape_pair = void;
    shape_pair[0] = a;
    shape_pair[1] = b;
    cpHashValue arbHashID = CP_HASH_PAIR(cast(cpHashValue)a, cast(cpHashValue)b);
    cpHashSetRemove(space.cachedArbiters, arbHashID, shape_pair.ptr);
    cpArrayDeleteObj(space.arbiters, arb);
}

/* TODO : DELETE
struct cpContact
{
    cpVect p, n;
    cpFloat dist = 0;

    cpVect r1, r2;
    cpFloat nMass = 0, tMass = 0, bounce = 0;

    cpFloat jnAcc = 0, jtAcc = 0, jBias = 0;
    cpFloat bias = 0;

    cpHashValue hash;
}*/

void cpArbiterCallSeparate(cpArbiter* arb, cpSpace* space)
{
    // The handler needs to be looked up again as the handler cached on the arbiter may have been deleted since the last step.
    cpCollisionHandler* handler = cpSpaceLookupHandler(space, arb.a.collision_type, arb.b.collision_type);
    handler.separate(arb, space, handler.data);
}

cpArbiterThread* cpArbiterThreadForBody(cpArbiter* arb, cpBody* bdy)
{
    return (arb.body_a == bdy ? &arb.thread_a : &arb.thread_b);
}+/
