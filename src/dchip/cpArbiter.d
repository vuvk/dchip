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
module dchip.cpArbiter;

import std.string;

import dchip.cpBody;
import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.constraints_util;
import dchip.cpSpace;
import dchip.cpSpatialIndex;
import dchip.cpShape;
import dchip.cpVect;
import dchip.util;


/// The cpArbiter struct tracks pairs of colliding shapes.
/// They are also used in conjuction with collision handler callbacks
/// allowing you to retrieve information on the collision or change it.
/// A unique arbiter value is used for each pair of colliding objects. It persists until the shapes separate.

enum CP_MAX_CONTACTS_PER_ARBITER = 2;

/// A macro shortcut for defining and retrieving the shapes from an arbiter.
template CP_ARBITER_GET_SHAPES(string arb, string a, string b)
{
    enum CP_ARBITER_GET_SHAPES = q{
        cpShape*  %2$s;
        cpShape*  %3$s;
        cpArbiterGetShapes(%1$s, &%2$s, &%3$s);
    }.format(arb, a, b);
}

/// A macro shortcut for defining and retrieving the bodies from an arbiter.
template CP_ARBITER_GET_BODIES(string arb, string a, string b)
{
    enum CP_ARBITER_GET_BODIES = q{
        cpBody*  %2$s;
        cpBody*  %3$s;
        cpArbiterGetBodies(%1$s, &%2$s, &%3$s);
    }.format(arb, a, b);
}

/// A struct that wraps up the important collision data for an arbiter.
struct cpContactPointSet 
{
	/// The number of contact points in the set.
	int count;
	
	/// The normal of the collision.
	cpVect normal;
	
	/// The array of contact points.
	struct Point 
	{
		/// The position of the contact on the surface of each shape.
		cpVect pointA, pointB;
		/// Penetration distance of the two shapes. Overlapping means it will be negative.
		/// This value is calculated as cpvdot(cpvsub(point2, point1), normal) and is ignored by cpArbiterSetContactPointSet().
		cpFloat distance;
	}	
	
	Point[CP_MAX_CONTACTS_PER_ARBITER] points;
}


// TODO: make this generic so I can reuse it for constraints also.
static void unthreadHelper(cpArbiter* arb, cpBody* body_)
{
	cpArbiterThread* thread = cpArbiterThreadForBody(arb, body_);
	cpArbiter* prev = thread.prev;
	cpArbiter* next = thread.next;
	
	if(prev)
	{
		cpArbiterThreadForBody(prev, body_).next = next;
	} 
	else 
		if(body_.arbiterList == arb) 
		{
			// IFF prev is null and body_.arbiterList == arb, is arb at the head of the list.
			// This function may be called for an arbiter that was never in a list.
			// In that case, we need to protect it from wiping out the body_.arbiterList pointer.
			body_.arbiterList = next;
		}
	
	if(next) 
		cpArbiterThreadForBody(next, body_).prev = prev;
	
	thread.prev = null;
	thread.next = null;
}

void cpArbiterUnthread(cpArbiter* arb)
{
	unthreadHelper(arb, arb.body_a);
	unthreadHelper(arb, arb.body_b);
}

cpBool cpArbiterIsFirstContact(const cpArbiter* arb)
{
	return arb.state == CP_ARBITER_STATE_FIRST_COLLISION;
}

cpBool cpArbiterIsRemoval(const cpArbiter* arb)
{
	return arb.state == CP_ARBITER_STATE_INVALIDATED;
}

int cpArbiterGetCount(const cpArbiter* arb)
{
	// Return 0 contacts if we are in a separate callback.
	return (arb.state < CP_ARBITER_STATE_CACHED ? arb.count : 0);
}

cpVect cpArbiterGetNormal(const cpArbiter* arb)
{
	return cpvmult(arb.n, arb.swapped ? -1.0f : 1.0);
}

cpVect cpArbiterGetPointA(const cpArbiter* arb, int i)
{
	cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	return cpvadd(arb.body_a.p, arb.contacts[i].r1);
}

cpVect cpArbiterGetPointB(const cpArbiter* arb, int i)
{
	cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	return cpvadd(arb.body_b.p, arb.contacts[i].r2);
}

cpFloat cpArbiterGetDepth(const cpArbiter* arb, int i)
{
	cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");
	
	cpContact* con = &arb.contacts[i];
	return cpvdot(cpvadd(cpvsub(con.r2, con.r1), cpvsub(arb.body_b.p, arb.body_a.p)), arb.n);
}

cpContactPointSet cpArbiterGetContactPointSet(const cpArbiter* arb)
{
	cpContactPointSet set;
	set.count = cpArbiterGetCount(arb);
	
	cpBool swapped = arb.swapped;
	cpVect n = arb.n;
	set.normal = (swapped ? cpvneg(n) : n);
	
	for(int i=0; i<set.count; i++)
	{
		// Contact points are relative to body CoGs;
		cpVect p1 = cpvadd(arb.body_a.p, arb.contacts[i].r1);
		cpVect p2 = cpvadd(arb.body_b.p, arb.contacts[i].r2);
		
		set.points[i].pointA = (swapped ? p2 : p1);
		set.points[i].pointB = (swapped ? p1 : p2);
		set.points[i].distance = cpvdot(cpvsub(p2, p1), n);
	}
	
	return set;
}

void cpArbiterSetContactPointSet(cpArbiter* arb, cpContactPointSet* set)
{
	int count = set.count;
	cpAssertHard(count == arb.count, "The number of contact points cannot be changed.");
	
	cpBool swapped = arb.swapped;
	arb.n = (swapped ? cpvneg(set.normal) : set.normal);
	
	for(int i=0; i<count; i++)
	{
		// Convert back to CoG relative offsets.
		cpVect p1 = set.points[i].pointA;
		cpVect p2 = set.points[i].pointB;
		
		arb.contacts[i].r1 = cpvsub(swapped ? p2 : p1, arb.body_a.p);
		arb.contacts[i].r2 = cpvsub(swapped ? p1 : p2, arb.body_b.p);
	}
}

cpVect cpArbiterTotalImpulse(const cpArbiter* arb)
{
	cpContact* contacts = arb.contacts;
	cpVect n = arb.n;
	cpVect sum = cpvzero;
	
	for(int i=0, count=cpArbiterGetCount(arb); i<count; i++)
	{
		cpContact* con = &contacts[i];
		sum = cpvadd(sum, cpvrotate(n, cpv(con.jnAcc, con.jtAcc)));
	}
		
	return (arb.swapped ? sum : cpvneg(sum));
	return cpvzero;
}

cpFloat cpArbiterTotalKE(const cpArbiter* arb)
{
	cpFloat eCoef = (1 - arb.e)/(1 + arb.e);
	cpFloat sum = 0.0;
	
	cpContact* contacts = arb.contacts;
	for(int i=0, count=cpArbiterGetCount(arb); i<count; i++)
	{
		cpContact* con = &contacts[i];
		cpFloat jnAcc = con.jnAcc;
		cpFloat jtAcc = con.jtAcc;
		
		sum += eCoef*jnAcc*jnAcc/con.nMass + jtAcc*jtAcc/con.tMass;
	}
	
	return sum;
}

cpBool cpArbiterIgnore(cpArbiter* arb)
{
	arb.state = CP_ARBITER_STATE_IGNORE;
	return cpFalse;
}

cpFloat cpArbiterGetRestitution(const cpArbiter* arb)
{
	return arb.e;
}

void cpArbiterSetRestitution(cpArbiter* arb, cpFloat restitution)
{
	arb.e = restitution;
}

cpFloat cpArbiterGetFriction(const cpArbiter* arb)
{
	return arb.u;
}

void cpArbiterSetFriction(cpArbiter* arb, cpFloat friction)
{
	arb.u = friction;
}

cpVect cpArbiterGetSurfaceVelocity(cpArbiter* arb)
{
	return cpvmult(arb.surface_vr, arb.swapped ? -1.0f : 1.0);
}

void cpArbiterSetSurfaceVelocity(cpArbiter* arb, cpVect vr)
{
	arb.surface_vr = cpvmult(vr, arb.swapped ? -1.0f : 1.0);
}

cpDataPointer cpArbiterGetUserData(const cpArbiter* arb)
{
	return arb.data;
}

void cpArbiterSetUserData(cpArbiter* arb, cpDataPointer userData)
{
	arb.data = userData;
}

void cpArbiterGetShapes(const cpArbiter* arb, cpShape** a, cpShape** b)
{
	if(arb.swapped)
	{
		(*a) = cast(cpShape*)arb.b;
		(*b) = cast(cpShape*)arb.a;
	} 
	else 
	{
		(*a) = cast(cpShape*)arb.a;
		(*b) = cast(cpShape*)arb.b;
	}
}

void cpArbiterGetBodies(const cpArbiter* arb, cpBody* *a, cpBody* *b)
{
	CP_ARBITER_GET_SHAPES(arb, shape_a, shape_b);
	(*a) = shape_a.body_;
	(*b) = shape_b.body_;
}

cpBool cpArbiterCallWildcardBeginA(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerA;
	return handler.beginFunc(arb, space, handler.userData);
}

cpBool cpArbiterCallWildcardBeginB(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerB;
	arb.swapped = !arb.swapped;
	cpBool retval = handler.beginFunc(arb, space, handler.userData);
	arb.swapped = !arb.swapped;
	return retval;
}

cpBool cpArbiterCallWildcardPreSolveA(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerA;
	return handler.preSolveFunc(arb, space, handler.userData);
}

cpBool cpArbiterCallWildcardPreSolveB(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerB;
	arb.swapped = !arb.swapped;
	cpBool retval = handler.preSolveFunc(arb, space, handler.userData);
	arb.swapped = !arb.swapped;
	return retval;
}

void cpArbiterCallWildcardPostSolveA(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerA;
	handler.postSolveFunc(arb, space, handler.userData);
}

void cpArbiterCallWildcardPostSolveB(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerB;
	arb.swapped = !arb.swapped;
	handler.postSolveFunc(arb, space, handler.userData);
	arb.swapped = !arb.swapped;
}

void cpArbiterCallWildcardSeparateA(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerA;
	handler.separateFunc(arb, space, handler.userData);
}

void cpArbiterCallWildcardSeparateB(cpArbiter* arb, cpSpace* space)
{
	cpCollisionHandler* handler = arb.handlerB;
	arb.swapped = !arb.swapped;
	handler.separateFunc(arb, space, handler.userData);
	arb.swapped = !arb.swapped;
}

cpArbiter* cpArbiterInit(cpArbiter* arb, cpShape* a, cpShape* b)
{
	arb.handler = null;
	arb.swapped = cpFalse;
	
	arb.handler = null;
	arb.handlerA = null;
	arb.handlerB = null;
	
	arb.e = 0.0f;
	arb.u = 0.0f;
	arb.surface_vr = cpvzero;
	
	arb.count = 0;
	arb.contacts = null;
	
	arb.a = a; arb.body_a = a.body_;
	arb.b = b; arb.body_b = b.body_;
	
	arb.thread_a.next = null;
	arb.thread_b.next = null;
	arb.thread_a.prev = null;
	arb.thread_b.prev = null;
	
	arb.stamp = 0;
	arb.state = CP_ARBITER_STATE_FIRST_COLLISION;
	
	arb.data = null;
	
	return arb;
}

static cpCollisionHandler* cpSpaceLookupHandler(cpSpace* space, cpCollisionType a, cpCollisionType b, cpCollisionHandler* defaultValue)
{
	cpCollisionType[] types = {a, b};
	cpCollisionHandler* handler = cast(cpCollisionHandler*)cpHashSetFind(space.collisionHandlers, CP_HASH_PAIR(a, b), types);
	return (handler ? handler : defaultValue);
}

void cpArbiterUpdate(cpArbiter* arb, cpCollisionInfo* info, cpSpace* space)
{
	immutable cpShape* a = info.a, b = info.b;
	
	// For collisions between two similar primitive types, the order could have been swapped since the last frame.
	arb.a = a; arb.body_a = a.body_;
	arb.b = b; arb.body_b = b.body_;
	
	// Iterate over the possible pairs to look for hash value matches.
	for(int i=0; i<info.count; i++)
	{
		cpContact* con = &info.arr[i];
		
		// r1 and r2 store absolute offsets at init time.
		// Need to convert them to relative offsets.
		con.r1 = cpvsub(con.r1, a.body_.p);
		con.r2 = cpvsub(con.r2, b.body_.p);
		
		// Cached impulses are not zeroed at init time.
		con.jnAcc = con.jtAcc = 0.0f;
		
		for(int j=0; j<arb.count; j++)
		{
			cpContact* old = &arb.contacts[j];
			
			// This could trigger false positives, but is fairly unlikely nor serious if it does.
			if(con.hash == old.hash)
			{
				// Copy the persistant contact information.
				con.jnAcc = old.jnAcc;
				con.jtAcc = old.jtAcc;
			}
		}
	}
	
	arb.contacts = info.arr;
	arb.count = info.count;
	arb.n = info.n;
	
	arb.e = a.e*  b.e;
	arb.u = a.u*  b.u;
	
	cpVect surface_vr = cpvsub(b.surfaceV, a.surfaceV);
	arb.surface_vr = cpvsub(surface_vr, cpvmult(info.n, cpvdot(surface_vr, info.n)));
	
	cpCollisionType typeA = info.a.type, typeB = info.b.type;
	cpCollisionHandler* defaultHandler = &space.defaultHandler;
	cpCollisionHandler* handler = arb.handler = cpSpaceLookupHandler(space, typeA, typeB, defaultHandler);
	
	// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
	cpBool swapped = arb.swapped = (typeA != handler.typeA && handler.typeA != CP_WILDCARD_COLLISION_TYPE);
	
	if(handler != defaultHandler || space.usesWildcards)
	{
		// The order of the main handler swaps the wildcard handlers too. Uffda.
		arb.handlerA = cpSpaceLookupHandler(space, (swapped ? typeB : typeA), CP_WILDCARD_COLLISION_TYPE, &cpCollisionHandlerDoNothing);
		arb.handlerB = cpSpaceLookupHandler(space, (swapped ? typeA : typeB), CP_WILDCARD_COLLISION_TYPE, &cpCollisionHandlerDoNothing);
	}
		
	// mark it as new if it's been cached
	if(arb.state == CP_ARBITER_STATE_CACHED) 
		arb.state = CP_ARBITER_STATE_FIRST_COLLISION;
}

void cpArbiterPreStep(cpArbiter* arb, cpFloat dt, cpFloat slop, cpFloat bias)
{
	cpBody* a = arb.body_a;
	cpBody* b = arb.body_b;
	cpVect n = arb.n;
	cpVect body_delta = cpvsub(b.p, a.p);
	
	for(int i=0; i<arb.count; i++)
	{
		cpContact* con = &arb.contacts[i];
		
		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0f/k_scalar(a, b, con.r1, con.r2, n);
		con.tMass = 1.0f/k_scalar(a, b, con.r1, con.r2, cpvperp(n));
				
		// Calculate the target bias velocity.
		cpFloat dist = cpvdot(cpvadd(cpvsub(con.r2, con.r1), body_delta), n);
		con.bias = -bias*cpfmin(0.0f, dist + slop)/dt;
		con.jBias = 0.0f;
		
		// Calculate the target bounce velocity.
		con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, n)*arb.e;
	}
}

void cpArbiterApplyCachedImpulse(cpArbiter* arb, cpFloat dt_coef)
{
	if(cpArbiterIsFirstContact(arb)) return;
	
	cpBody* a = arb.body_a;
	cpBody* b = arb.body_b;
	cpVect n = arb.n;
	
	for(int i=0; i<arb.count; i++)
	{
		cpContact* con = &arb.contacts[i];
		cpVect j = cpvrotate(n, cpv(con.jnAcc, con.jtAcc));
		apply_impulses(a, b, con.r1, con.r2, cpvmult(j, dt_coef));
	}
}

// TODO: is it worth splitting velocity/position correction?

void cpArbiterApplyImpulse(cpArbiter* arb)
{
	cpBody* a = arb.body_a;
	cpBody* b = arb.body_b;
	cpVect n = arb.n;
	cpVect surface_vr = arb.surface_vr;
	cpFloat friction = arb.u;

	for(int i=0; i<arb.count; i++)
	{
		cpContact* con = &arb.contacts[i];
		cpFloat nMass = con.nMass;
		cpVect r1 = con.r1;
		cpVect r2 = con.r2;
		
		cpVect vb1 = cpvadd(a.v_bias, cpvmult(cpvperp(r1), a.w_bias));
		cpVect vb2 = cpvadd(b.v_bias, cpvmult(cpvperp(r2), b.w_bias));
		cpVect vr = cpvadd(relative_velocity(a, b, r1, r2), surface_vr);
		
		cpFloat vbn = cpvdot(cpvsub(vb2, vb1), n);
		cpFloat vrn = cpvdot(vr, n);
		cpFloat vrt = cpvdot(vr, cpvperp(n));
		
		cpFloat jbn = (con.bias - vbn)*nMass;
		cpFloat jbnOld = con.jBias;
		con.jBias = cpfmax(jbnOld + jbn, 0.0f);
		
		cpFloat jn = -(con.bounce + vrn)*nMass;
		cpFloat jnOld = con.jnAcc;
		con.jnAcc = cpfmax(jnOld + jn, 0.0f);
		
		cpFloat jtMax = friction*con.jnAcc;
		cpFloat jt = -vrt*con.tMass;
		cpFloat jtOld = con.jtAcc;
		con.jtAcc = cpfclamp(jtOld + jt, -jtMax, jtMax);
		
		apply_bias_impulses(a, b, r1, r2, cpvmult(n, con.jBias - jbnOld));
		apply_impulses(a, b, r1, r2, cpvrotate(n, cpv(con.jnAcc - jnOld, con.jtAcc - jtOld)));
	}
}



/+ 
TODO : DELETE
/// Collision begin event function callback type.
/// Returning false from a begin callback causes the collision to be ignored until
/// the the separate callback is called when the objects stop colliding.
alias cpCollisionBeginFunc = cpBool function(cpArbiter* arb, cpSpace* space, void* data);

/// Collision pre-solve event function callback type.
/// Returning false from a pre-step callback causes the collision to be ignored until the next step.
alias cpCollisionPreSolveFunc = cpBool function(cpArbiter* arb, cpSpace* space, void* data);

/// Collision post-solve event function callback type.
alias cpCollisionPostSolveFunc = void function(cpArbiter* arb, cpSpace* space, void* data);

/// Collision separate event function callback type.
alias cpCollisionSeparateFunc = void function(cpArbiter* arb, cpSpace* space, void* data);

/// @private
struct cpCollisionHandler
{
    cpCollisionType a;
    cpCollisionType b;
    cpCollisionBeginFunc begin;
    cpCollisionPreSolveFunc preSolve;
    cpCollisionPostSolveFunc postSolve;
    cpCollisionSeparateFunc separate;
    void* data;
}

enum CP_MAX_CONTACTS_PER_ARBITER = 2;

/// @private
enum cpArbiterState
{
    // Arbiter is active and its the first collision.
    cpArbiterStateFirstColl,

    // Arbiter is active and its not the first collision.
    cpArbiterStateNormal,

    // Collision has been explicitly ignored.
    // Either by returning false from a begin collision handler or calling cpArbiterIgnore().
    cpArbiterStateIgnore,

    // Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
    cpArbiterStateCached,
}

///
mixin _ExportEnumMembers!cpArbiterState;

/// @private
struct cpArbiterThread
{
    // Links to next and previous arbiters in the contact graph.
    cpArbiter* next;
    cpArbiter* prev;
}

/// A colliding pair of shapes.
struct cpArbiter
{
    /// Calculated value to use for the elasticity coefficient.
    /// Override in a pre-solve collision handler for custom behavior.
    cpFloat e = 0;

    /// Calculated value to use for the friction coefficient.
    /// Override in a pre-solve collision handler for custom behavior.
    cpFloat u = 0;

    /// Calculated value to use for applying surface velocities.
    /// Override in a pre-solve collision handler for custom behavior.
    cpVect surface_vr;

    /// User definable data pointer.
    /// The value will persist for the pair of shapes until the separate() callback is called.
    /// NOTE: If you need to clean up this pointer, you should implement the separate() callback to do it.
    cpDataPointer data;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpShape*  a;
    else
        package cpShape*  a;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpShape*  b;
    else
        package cpShape*  b;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpBody*  body_a;
    else
        package cpBody*  body_a;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpBody*  body_b;
    else
        package cpBody*  body_b;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpArbiterThread thread_a;
    else
        package cpArbiterThread thread_a;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpArbiterThread thread_b;
    else
        package cpArbiterThread thread_b;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        int numContacts;
    else
        package int numContacts;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpContact*  contacts;
    else
        package cpContact*  contacts;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpTimestamp stamp;
    else
        package cpTimestamp stamp;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpCollisionHandler*  handler;
    else
        package cpCollisionHandler*  handler;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpBool swappedColl;
    else
        package cpBool swappedColl;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpArbiterState state;
    else
        package cpArbiterState state;
}

mixin template CP_DefineArbiterStructGetter(type, string member, string name)
{
    mixin(q{
        type cpArbiterGet%s(const cpArbiter*  arb) { return cast(typeof(return))arb.%s; }
    }.format(name, member));
}

mixin template CP_DefineArbiterStructSetter(type, string member, string name)
{
    mixin(q{
        void cpArbiterSet%s(cpArbiter*  arb, type value) { arb.%s = value; }
    }.format(name, member));
}

mixin template CP_DefineArbiterStructProperty(type, string member, string name)
{
    mixin CP_DefineArbiterStructGetter!(type, member, name);
    mixin CP_DefineArbiterStructSetter!(type, member, name);
}

mixin CP_DefineArbiterStructProperty!(cpFloat, "e", "Elasticity");
mixin CP_DefineArbiterStructProperty!(cpFloat, "u", "Friction");

mixin CP_DefineArbiterStructProperty!(cpDataPointer, "data", "UserData");

/// Return the colliding shapes involved for this arbiter.
/// The order of their cpSpace.collision_type values will match
/// the order set when the collision handler was registered.
void cpArbiterGetShapes(const cpArbiter* arb, cpShape** a, cpShape** b)
{
    if (arb.swappedColl)
    {
        (*a) = cast(typeof(*a))arb.b;
        (*b) = cast(typeof(*a))arb.a;
    }
    else
    {
        (*a) = cast(typeof(*a))arb.a;
        (*b) = cast(typeof(*b))arb.b;
    }
}

/// A macro shortcut for defining and retrieving the shapes from an arbiter.
string CP_ARBITER_GET_SHAPES(string arb, string a, string b)()
{
    return q{
        cpShape*  %2$s;
        cpShape*  %3$s;
        cpArbiterGetShapes(%1$s, &%2$s, &%3$s);
    }.format(arb, a, b);
}

/// Return the colliding bodies involved for this arbiter.
/// The order of the cpSpace.collision_type the bodies are associated with values will match
/// the order set when the collision handler was registered.
void cpArbiterGetBodies(const cpArbiter* arb, cpBody** a, cpBody** b)
{
    mixin(CP_ARBITER_GET_SHAPES!("arb", "shape_a", "shape_b"));
    (*a) = shape_a.body_;
    (*b) = shape_b.body_;
}

/// A macro shortcut for defining and retrieving the bodies from an arbiter.
template CP_ARBITER_GET_BODIES(string arb, string a, string b)
{
    enum CP_ARBITER_GET_BODIES = q{
        cpBody*  %2$s;
        cpBody*  %3$s;
        cpArbiterGetBodies(%1$s, &%2$s, &%3$s);
    }.format(arb, a, b);
}

/// A struct that wraps up the important collision data for an arbiter.
struct cpContactPointSet
{
    /// The number of contact points in the set.
    int count;

    /// The array of contact points.
    struct Point
    {
        /// The position of the contact point.
        cpVect point;

        /// The normal of the contact point.
        cpVect normal;

        /// The depth of the contact point.
        cpFloat dist = 0;
    }

    Point[CP_MAX_CONTACTS_PER_ARBITER] points;
}

cpContact* cpContactInit(cpContact* con, cpVect p, cpVect n, cpFloat dist, cpHashValue hash)
{
    con.p    = p;
    con.n    = n;
    con.dist = dist;

    con.jnAcc = 0.0f;
    con.jtAcc = 0.0f;
    con.jBias = 0.0f;

    con.hash = hash;

    return con;
}

// TODO make this generic so I can reuse it for constraints also.
void unthreadHelper(cpArbiter* arb, cpBody* body_)
{
    cpArbiterThread* thread = cpArbiterThreadForBody(arb, body_);
    cpArbiter* prev = thread.prev;
    cpArbiter* next = thread.next;

    if (prev)
    {
        cpArbiterThreadForBody(prev, body_).next = next;
    }
    else if (body_.arbiterList == arb)
    {
        // IFF prev is null and body_.arbiterList == arb, is arb at the head of the list.
        // This function may be called for an arbiter that was never in a list.
        // In that case, we need to protect it from wiping out the body_.arbiterList pointer.
        body_.arbiterList = next;
    }

    if (next)
        cpArbiterThreadForBody(next, body_).prev = prev;

    thread.prev = null;
    thread.next = null;
}

void cpArbiterUnthread(cpArbiter* arb)
{
    unthreadHelper(arb, arb.body_a);
    unthreadHelper(arb, arb.body_b);
}

cpBool cpArbiterIsFirstContact(const cpArbiter* arb)
{
    return arb.state == cpArbiterStateFirstColl;
}

int cpArbiterGetCount(const cpArbiter* arb)
{
    // Return 0 contacts if we are in a separate callback.
    return (arb.state != cpArbiterStateCached ? arb.numContacts : 0);
}

cpVect cpArbiterGetNormal(const cpArbiter* arb, int i)
{
    cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");

    cpVect n = arb.contacts[i].n;
    return arb.swappedColl ? cpvneg(n) : n;
}

cpVect cpArbiterGetPoint(const cpArbiter* arb, int i)
{
    cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");

    return arb.contacts[i].p;
}

cpFloat cpArbiterGetDepth(const cpArbiter* arb, int i)
{
    cpAssertHard(0 <= i && i < cpArbiterGetCount(arb), "Index error: The specified contact index is invalid for this arbiter");

    return arb.contacts[i].dist;
}

cpContactPointSet cpArbiterGetContactPointSet(const cpArbiter* arb)
{
    cpContactPointSet set;
    set.count = cpArbiterGetCount(arb);

    for (int i = 0; i < set.count; i++)
    {
        set.points[i].point  = arb.contacts[i].p;
        set.points[i].normal = arb.contacts[i].n;
        set.points[i].dist   = arb.contacts[i].dist;
    }

    return set;
}

void cpArbiterSetContactPointSet(cpArbiter* arb, cpContactPointSet* set)
{
    int count = set.count;
    cpAssertHard(count == arb.numContacts, "The number of contact points cannot be changed.");

    for (int i = 0; i < count; i++)
    {
        arb.contacts[i].p    = set.points[i].point;
        arb.contacts[i].n    = set.points[i].normal;
        arb.contacts[i].dist = set.points[i].dist;
    }
}

cpVect cpArbiterTotalImpulse(const cpArbiter* arb)
{
    cpContact* contacts = cast(cpContact*)arb.contacts;
    cpVect sum = cpvzero;

    for (int i = 0, count = cpArbiterGetCount(arb); i < count; i++)
    {
        cpContact* con = &contacts[i];
        sum = cpvadd(sum, cpvmult(con.n, con.jnAcc));
    }

    return (arb.swappedColl ? sum : cpvneg(sum));
}

cpVect cpArbiterTotalImpulseWithFriction(const cpArbiter* arb)
{
    cpContact* contacts = cast(cpContact*)arb.contacts;
    cpVect sum = cpvzero;

    for (int i = 0, count = cpArbiterGetCount(arb); i < count; i++)
    {
        cpContact* con = &contacts[i];
        sum = cpvadd(sum, cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc)));
    }

    return (arb.swappedColl ? sum : cpvneg(sum));
}

cpFloat cpArbiterTotalKE(const cpArbiter* arb)
{
    cpFloat eCoef = (1 - arb.e) / (1 + arb.e);
    cpFloat sum   = 0.0;

    cpContact* contacts = cast(cpContact*)arb.contacts;

    for (int i = 0, count = cpArbiterGetCount(arb); i < count; i++)
    {
        cpContact* con = &contacts[i];
        cpFloat jnAcc  = con.jnAcc;
        cpFloat jtAcc  = con.jtAcc;

        sum += eCoef*  jnAcc*  jnAcc / con.nMass + jtAcc*  jtAcc / con.tMass;
    }

    return sum;
}

void cpArbiterIgnore(cpArbiter* arb)
{
    arb.state = cpArbiterStateIgnore;
}

cpVect cpArbiterGetSurfaceVelocity(cpArbiter* arb)
{
    return cpvmult(arb.surface_vr, arb.swappedColl ? -1.0f : 1.0);
}

void cpArbiterSetSurfaceVelocity(cpArbiter* arb, cpVect vr)
{
    arb.surface_vr = cpvmult(vr, arb.swappedColl ? -1.0f : 1.0);
}

cpArbiter* cpArbiterInit(cpArbiter* arb, cpShape* a, cpShape* b)
{
    arb.handler     = null;
    arb.swappedColl = cpFalse;

    arb.e = 0.0f;
    arb.u = 0.0f;
    arb.surface_vr = cpvzero;

    arb.numContacts = 0;
    arb.contacts    = null;

    arb.a      = a;
    arb.body_a = a.body_;
    arb.b      = b;
    arb.body_b = b.body_;

    arb.thread_a.next = null;
    arb.thread_b.next = null;
    arb.thread_a.prev = null;
    arb.thread_b.prev = null;

    arb.stamp = 0;
    arb.state = cpArbiterStateFirstColl;

    arb.data = null;

    return arb;
}

void cpArbiterUpdate(cpArbiter* arb, cpContact* contacts, int numContacts, cpCollisionHandler* handler, cpShape* a, cpShape* b)
{
    // Iterate over the possible pairs to look for hash value matches.
    for (int i = 0; i < numContacts; i++)
    {
        cpContact* con = &contacts[i];

        for (int j = 0; j < arb.numContacts; j++)
        {
            cpContact* old = &arb.contacts[j];

            // This could trigger false positives, but is fairly unlikely nor serious if it does.
            if (con.hash == old.hash)
            {
                // Copy the persistant contact information.
                con.jnAcc = old.jnAcc;
                con.jtAcc = old.jtAcc;
            }
        }
    }

    arb.contacts    = contacts;
    arb.numContacts = numContacts;

    arb.handler     = handler;
    arb.swappedColl = (a.collision_type != handler.a);

    arb.e = a.e*  b.e;
    arb.u = a.u*  b.u;

    // Currently all contacts will have the same normal.
    // This may change in the future.
    cpVect n = (numContacts ? contacts[0].n : cpvzero);
    cpVect surface_vr = cpvsub(a.surface_v, b.surface_v);
    arb.surface_vr = cpvsub(surface_vr, cpvmult(n, cpvdot(surface_vr, n)));

    // For collisions between two similar primitive types, the order could have been swapped.
    arb.a      = a;
    arb.body_a = a.body_;
    arb.b      = b;
    arb.body_b = b.body_;

    // mark it as new if it's been cached
    if (arb.state == cpArbiterStateCached)
        arb.state = cpArbiterStateFirstColl;
}

void cpArbiterPreStep(cpArbiter* arb, cpFloat dt, cpFloat slop, cpFloat bias)
{
    cpBody* a = arb.body_a;
    cpBody* b = arb.body_b;

    for (int i = 0; i < arb.numContacts; i++)
    {
        cpContact* con = &arb.contacts[i];

        // Calculate the offsets.
        con.r1 = cpvsub(con.p, a.p);
        con.r2 = cpvsub(con.p, b.p);

        // Calculate the mass normal and mass tangent.
        con.nMass = 1.0f / k_scalar(a, b, con.r1, con.r2, con.n);
        con.tMass = 1.0f / k_scalar(a, b, con.r1, con.r2, cpvperp(con.n));

        // Calculate the target bias velocity.
        con.bias  = -bias* cpfmin(0.0f, con.dist + slop) / dt;
        con.jBias = 0.0f;

        // Calculate the target bounce velocity.
        con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, con.n)*  arb.e;
    }
}

void cpArbiterApplyCachedImpulse(cpArbiter* arb, cpFloat dt_coef)
{
    if (cpArbiterIsFirstContact(arb))
        return;

    cpBody* a = arb.body_a;
    cpBody* b = arb.body_b;

    for (int i = 0; i < arb.numContacts; i++)
    {
        cpContact* con = &arb.contacts[i];
        cpVect j       = cpvrotate(con.n, cpv(con.jnAcc, con.jtAcc));
        apply_impulses(a, b, con.r1, con.r2, cpvmult(j, dt_coef));
    }
}

// TODO is it worth splitting velocity/position correction?

void cpArbiterApplyImpulse(cpArbiter* arb)
{
    cpBody* a = arb.body_a;
    cpBody* b = arb.body_b;
    cpVect  surface_vr = arb.surface_vr;
    cpFloat friction   = arb.u;

    for (int i = 0; i < arb.numContacts; i++)
    {
        cpContact* con = &arb.contacts[i];
        cpFloat nMass  = con.nMass;
        cpVect  n      = con.n;
        cpVect  r1     = con.r1;
        cpVect  r2     = con.r2;

        cpVect vb1 = cpvadd(a.v_bias, cpvmult(cpvperp(r1), a.w_bias));
        cpVect vb2 = cpvadd(b.v_bias, cpvmult(cpvperp(r2), b.w_bias));
        cpVect vr  = cpvadd(relative_velocity(a, b, r1, r2), surface_vr);

        cpFloat vbn = cpvdot(cpvsub(vb2, vb1), n);
        cpFloat vrn = cpvdot(vr, n);
        cpFloat vrt = cpvdot(vr, cpvperp(n));

        cpFloat jbn    = (con.bias - vbn)*  nMass;
        cpFloat jbnOld = con.jBias;
        con.jBias = cpfmax(jbnOld + jbn, 0.0f);

        cpFloat jn    = -(con.bounce + vrn)*  nMass;
        cpFloat jnOld = con.jnAcc;
        con.jnAcc = cpfmax(jnOld + jn, 0.0f);

        cpFloat jtMax = friction*  con.jnAcc;
        cpFloat jt    = -vrt*  con.tMass;
        cpFloat jtOld = con.jtAcc;
        con.jtAcc = cpfclamp(jtOld + jt, -jtMax, jtMax);

        apply_bias_impulses(a, b, r1, r2, cpvmult(n, con.jBias - jbnOld));
        apply_impulses(a, b, r1, r2, cpvrotate(n, cpv(con.jnAcc - jnOld, con.jtAcc - jtOld)));
    }
}+/
