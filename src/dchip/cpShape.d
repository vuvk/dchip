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
module dchip.cpShape;

import std.string;


import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;
import dchip.cpArbiter;

import dchip.util;
import dchip.cpBB;
import dchip.cpBody;
import dchip.cpSpace;
import dchip.cpVect;

/+ TODO : DELETE
/// The cpShape struct defines the shape of a rigid body_.

/// Segment query info struct.
struct cpSegmentQueryInfo
{
    /// The shape that was hit, null if no collision occured.  
	 TODO : DELETE
	cpShape* shape;

    /// The normalized distance along the query segment in the range [0, 1].
    cpFloat t = 0;

    /// The normal of the surface hit.
    cpVect n;
}

/// Nearest point query info struct.
struct cpNearestPointQueryInfo
{
    /// The nearest shape, null if no shape was within range.
    cpShape* shape;

    /// The closest point on the shape's surface. (in world space coordinates)
    cpVect p;

    /// The distance to the point. The distance is negative if the point is inside the shape.
    cpFloat d = 0;

    /// The gradient of the signed distance function.
    /// The same as info.p/info.d, but accurate even for very small values of info.d.
    cpVect g;
}


/// @private
enum cpShapeType
{
    CP_CIRCLE_SHAPE,
    CP_SEGMENT_SHAPE,
    CP_POLY_SHAPE,
    CP_NUM_SHAPES
}

///
mixin _ExportEnumMembers!cpShapeType;

alias cpShapeCacheDataImpl = cpBB function(cpShape* shape, cpVect p, cpVect rot);
alias cpShapeDestroyImpl = void function(cpShape* shape);
alias cpShapeNearestPointQueryImpl = void function(cpShape* shape, cpVect p, cpNearestPointQueryInfo* info);
alias cpShapeSegmentQueryImpl = void function(cpShape* shape, cpVect a, cpVect b, cpSegmentQueryInfo* info);

/// @private
struct cpShapeClass
{
    cpShapeType type;

    cpShapeCacheDataImpl cacheData;
    cpShapeDestroyImpl destroy;
    cpShapeNearestPointQueryImpl nearestPointQuery;
    cpShapeSegmentQueryImpl segmentQuery;
}

/// Opaque collision shape struct.
struct cpShape
{
    version (CHIP_ALLOW_PRIVATE_ACCESS)
        /* const* / cpShapeClass*  klass;
    else
        package /* const* / cpShapeClass*  klass;

    /// The rigid body_ this collision shape is attached to.
    cpBody* body_;

    /// The current bounding box of the shape.
    cpBB bb;

    /// Sensor flag.
    /// Sensor shapes call collision callbacks but don't produce collisions.
    cpBool sensor;

    /// Coefficient of restitution. (elasticity)
    cpFloat e = 0;

    /// Coefficient of friction.
    cpFloat u = 0;

    /// Surface velocity used when solving for friction.
    cpVect surface_v;

    /// User definable data pointer.
    /// Generally this points to your the game object class so you can access it
    /// when given a cpShape reference in a callback.
    cpDataPointer data;

    /// Collision type of this shape used when picking collision handlers.
    cpCollisionType collision_type;

    /// Group of this shape. Shapes in the same group don't collide.
    cpGroup group;

    // Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
    cpLayers layers;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpSpace*  space;
    else
        package cpSpace*  space;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpShape*  next;
    else
        package cpShape*  next;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpShape*  prev;
    else
        package cpShape*  prev;

    version (CHIP_ALLOW_PRIVATE_ACCESS)
        cpHashValue hashid;
    else
        package cpHashValue hashid;
}

/// Get the hit point for a segment query.
cpVect cpSegmentQueryHitPoint(const cpVect start, const cpVect end, const cpSegmentQueryInfo info)
{
    return cpvlerp(start, end, info.t);
}

/// Get the hit distance for a segment query.
cpFloat cpSegmentQueryHitDist(const cpVect start, const cpVect end, const cpSegmentQueryInfo info)
{
    return cpvdist(start, end)*  info.t;
}

mixin template CP_DefineShapeStructGetter(type, string member, string name)
{
    mixin(q{
        type cpShapeGet%s(const cpShape*  shape) { return cast(typeof(return))shape.%s; }
    }.format(name, member));
}

mixin template CP_DefineShapeStructSetter(type, string member, string name, bool activates)
{
    mixin(q{
        void cpShapeSet%s(cpShape*  shape, type value)
        {
            %s

            shape.%s = value;
        }
    }.format(name, activates ? "if (shape.body_) cpBodyActivate(shape.body_);" : "", member));
}

mixin template CP_DefineShapeStructProperty(type, string member, string name, bool activates)
{
    mixin CP_DefineShapeStructGetter!(type, member, name);
    mixin CP_DefineShapeStructSetter!(type, member, name, activates);
}

mixin CP_DefineShapeStructGetter!(cpSpace*, "space", "Space");

mixin CP_DefineShapeStructGetter!(cpBody*, "body_", "Body");

mixin CP_DefineShapeStructGetter!(cpBB, "bb", "BB");
mixin CP_DefineShapeStructProperty!(cpBool, "sensor", "Sensor", cpTrue);
mixin CP_DefineShapeStructProperty!(cpFloat, "e", "Elasticity", cpFalse);
mixin CP_DefineShapeStructProperty!(cpFloat, "u", "Friction", cpTrue);
mixin CP_DefineShapeStructProperty!(cpVect, "surface_v", "SurfaceVelocity", cpTrue);
mixin CP_DefineShapeStructProperty!(cpDataPointer, "data", "UserData", cpFalse);
mixin CP_DefineShapeStructProperty!(cpCollisionType, "collision_type", "CollisionType", cpTrue);
mixin CP_DefineShapeStructProperty!(cpGroup, "group", "Group", cpTrue);
mixin CP_DefineShapeStructProperty!(cpLayers, "layers", "Layers", cpTrue);

mixin template CP_DeclareShapeGetter(type, string struct_, string member, string name)
{
    mixin(q{
        type %sGet%s(const cpShape*  shape)
    }.format(struct_, name, member));
}

/// @private
struct cpCircleShape
{
    cpShape shape;

    cpVect c, tc;
    cpFloat r = 0;
}

/// @private
struct cpSegmentShape
{
    cpShape shape;

    cpVect a, b, n;
    cpVect ta, tb, tn;
    cpFloat r = 0;

    cpVect a_tangent, b_tangent;
}

__gshared cpHashValue cpShapeIDCounter = 0;

void cpResetShapeIdCounter()
{
    cpShapeIDCounter = 0;
}

cpShape* cpShapeInit(cpShape* shape, const cpShapeClass* klass, cpBody* body_)
{
    shape.klass = cast(typeof(shape.klass))klass;

    shape.hashid = cpShapeIDCounter;
    cpShapeIDCounter++;

    shape.body_   = body_;
    shape.sensor = 0;

    shape.e         = 0.0f;
    shape.u         = 0.0f;
    shape.surface_v = cpvzero;

    shape.collision_type = 0;
    shape.group  = CP_NO_GROUP;
    shape.layers = CP_ALL_LAYERS;

    shape.data = null;

    shape.space = null;

    shape.next = null;
    shape.prev = null;

    return shape;
}

void cpShapeDestroy(cpShape* shape)
{
    if (shape.klass && shape.klass.destroy)
        shape.klass.destroy(shape);
}

void cpShapeFree(cpShape* shape)
{
    if (shape)
    {
        cpShapeDestroy(shape);
        cpfree(shape);
    }
}

void cpShapeSetBody(cpShape* shape, cpBody* body_)
{
    cpAssertHard(!cpShapeActive(shape), "You cannot change the body_ on an active shape. You must remove the shape from the space before changing the body_.");
    shape.body_ = body_;
}

cpBB cpShapeCacheBB(cpShape* shape)
{
    cpBody* body_ = shape.body_;
    return cpShapeUpdate(shape, body_.p, body_.rot);
}

cpBB cpShapeUpdate(cpShape* shape, cpVect pos, cpVect rot)
{
    return (shape.bb = shape.klass.cacheData(shape, pos, rot));
}

cpBool cpShapePointQuery(cpShape* shape, cpVect p)
{
    cpNearestPointQueryInfo info = { null, cpvzero, INFINITY, cpvzero };
    cpShapeNearestPointQuery(shape, p, &info);

    return (info.d < 0.0f);
}

cpFloat cpShapeNearestPointQuery(cpShape* shape, cpVect p, cpNearestPointQueryInfo* info)
{
    cpNearestPointQueryInfo blank = { null, cpvzero, INFINITY, cpvzero };

    if (info)
    {
        (*info) = blank;
    }
    else
    {
        info = &blank;
    }

    shape.klass.nearestPointQuery(shape, p, info);
    return info.d;
}

cpBool cpShapeSegmentQuery(cpShape* shape, cpVect a, cpVect b, cpSegmentQueryInfo* info)
{
    cpSegmentQueryInfo blank = { null, 1.0f, cpvzero };

    if (info)
    {
        (*info) = blank;
    }
    else
    {
        info = &blank;
    }

    cpNearestPointQueryInfo nearest;
    shape.klass.nearestPointQuery(shape, a, &nearest);

    if (nearest.d <= 0.0)
    {
        info.shape = shape;
        info.t     = 0.0;
        info.n     = cpvnormalize(cpvsub(a, nearest.p));
    }
    else
    {
        shape.klass.segmentQuery(shape, a, b, info);
    }

    return (info.shape != null);
}

cpCircleShape* cpCircleShapeAlloc()
{
    return cast(cpCircleShape*)cpcalloc(1, cpCircleShape.sizeof);
}

cpBB cpCircleShapeCacheData(cpCircleShape* circle, cpVect p, cpVect rot)
{
    cpVect c = circle.tc = cpvadd(p, cpvrotate(circle.c, rot));
    return cpBBNewForCircle(c, circle.r);
}

void cpCicleShapeNearestPointQuery(cpCircleShape* circle, cpVect p, cpNearestPointQueryInfo* info)
{
    cpVect  delta = cpvsub(p, circle.tc);
    cpFloat d     = cpvlength(delta);
    cpFloat r     = circle.r;

    info.shape = cast(cpShape*)circle;
    info.p     = cpvadd(circle.tc, cpvmult(delta, r / d)); // TODO div/0
    info.d     = d - r;

    // Use up for the gradient if the distance is very small.
    info.g = (d > MAGIC_EPSILON ? cpvmult(delta, 1.0f / d) : cpv(0.0f, 1.0f));
}

void cpCircleShapeSegmentQuery(cpCircleShape* circle, cpVect a, cpVect b, cpSegmentQueryInfo* info)
{
    CircleSegmentQuery(cast(cpShape*)circle, circle.tc, circle.r, a, b, info);
}

cpCircleShape* cpCircleShapeInit(cpCircleShape* circle, cpBody* body_, cpFloat radius, cpVect offset)
{
    circle.c = offset;
    circle.r = radius;

    cpShapeInit(cast(cpShape*)circle, &cpCircleShapeClass, body_);

    return circle;
}

cpShape* cpCircleShapeNew(cpBody* body_, cpFloat radius, cpVect offset)
{
    return cast(cpShape*)cpCircleShapeInit(cpCircleShapeAlloc(), body_, radius, offset);
}

cpSegmentShape* 
cpSegmentShapeAlloc()
{
    return cast(cpSegmentShape*)cpcalloc(1, cpSegmentShape.sizeof);
}

cpBB cpSegmentShapeCacheData(cpSegmentShape* seg, cpVect p, cpVect rot)
{
    seg.ta = cpvadd(p, cpvrotate(seg.a, rot));
    seg.tb = cpvadd(p, cpvrotate(seg.b, rot));
    seg.tn = cpvrotate(seg.n, rot);

    cpFloat l = 0, r = 0, b = 0, t = 0;

    if (seg.ta.x < seg.tb.x)
    {
        l = seg.ta.x;
        r = seg.tb.x;
    }
    else
    {
        l = seg.tb.x;
        r = seg.ta.x;
    }

    if (seg.ta.y < seg.tb.y)
    {
        b = seg.ta.y;
        t = seg.tb.y;
    }
    else
    {
        b = seg.tb.y;
        t = seg.ta.y;
    }

    cpFloat rad = seg.r;
    return cpBBNew(l - rad, b - rad, r + rad, t + rad);
}

void cpSegmentShapeNearestPointQuery(cpSegmentShape* seg, cpVect p, cpNearestPointQueryInfo* info)
{
    cpVect closest = cpClosetPointOnSegment(p, seg.ta, seg.tb);

    cpVect  delta = cpvsub(p, closest);
    cpFloat d     = cpvlength(delta);
    cpFloat r     = seg.r;
    cpVect  g     = cpvmult(delta, 1.0f / d);

    info.shape = cast(cpShape*)seg;
    info.p     = (d ? cpvadd(closest, cpvmult(g, r)) : closest);
    info.d     = d - r;

    // Use the segment's normal if the distance is very small.
    info.g = (d > MAGIC_EPSILON ? g : seg.n);
}

void cpSegmentShapeSegmentQuery(cpSegmentShape* seg, cpVect a, cpVect b, cpSegmentQueryInfo* info)
{
    cpVect  n = seg.tn;
    cpFloat d = cpvdot(cpvsub(seg.ta, a), n);
    cpFloat r = seg.r;

    cpVect flipped_n  = (d > 0.0f ? cpvneg(n) : n);
    cpVect seg_offset = cpvsub(cpvmult(flipped_n, r), a);

    // Make the endpoints relative to 'a' and move them by the thickness of the segment.
    cpVect seg_a = cpvadd(seg.ta, seg_offset);
    cpVect seg_b = cpvadd(seg.tb, seg_offset);
    cpVect delta = cpvsub(b, a);

    if (cpvcross(delta, seg_a)*  cpvcross(delta, seg_b) <= 0.0f)
    {
        cpFloat d_offset = d + (d > 0.0f ? -r : r);
        cpFloat ad       = -d_offset;
        cpFloat bd       = cpvdot(delta, n) - d_offset;

        if (ad*  bd < 0.0f)
        {
            info.shape = cast(cpShape*)seg;
            info.t     = ad / (ad - bd);
            info.n     = flipped_n;
        }
    }
    else if (r != 0.0f)
    {
        cpSegmentQueryInfo info1 = { null, 1.0f, cpvzero };
        cpSegmentQueryInfo info2 = { null, 1.0f, cpvzero };
        CircleSegmentQuery(cast(cpShape*)seg, seg.ta, seg.r, a, b, &info1);
        CircleSegmentQuery(cast(cpShape*)seg, seg.tb, seg.r, a, b, &info2);

        if (info1.t < info2.t)
        {
            (*info) = info1;
        }
        else
        {
            (*info) = info2;
        }
    }
}

__gshared cpShapeClass cpSegmentShapeClass;
__gshared cpShapeClass cpCircleShapeClass;

void _initModuleCtor_cpShape()
{
    cpSegmentShapeClass = cpShapeClass(
        CP_SEGMENT_SHAPE,
        cast(cpShapeCacheDataImpl)&cpSegmentShapeCacheData,
        null,
        cast(cpShapeNearestPointQueryImpl)&cpSegmentShapeNearestPointQuery,
        cast(cpShapeSegmentQueryImpl)&cpSegmentShapeSegmentQuery,
    );

    cpCircleShapeClass = cpShapeClass(
        CP_CIRCLE_SHAPE,
        cast(cpShapeCacheDataImpl)&cpCircleShapeCacheData,
        null,
        cast(cpShapeNearestPointQueryImpl)&cpCicleShapeNearestPointQuery,
        cast(cpShapeSegmentQueryImpl)&cpCircleShapeSegmentQuery,
    );
}

cpSegmentShape* cpSegmentShapeInit(cpSegmentShape* seg, cpBody* body_, cpVect a, cpVect b, cpFloat r)
{
    seg.a = a;
    seg.b = b;
    seg.n = cpvperp(cpvnormalize(cpvsub(b, a)));

    seg.r = r;

    seg.a_tangent = cpvzero;
    seg.b_tangent = cpvzero;

    cpShapeInit(cast(cpShape*)seg, &cpSegmentShapeClass, body_);

    return seg;
}

cpShape* cpSegmentShapeNew(cpBody* body_, cpVect a, cpVect b, cpFloat r)
{
    return cast(cpShape*)cpSegmentShapeInit(cpSegmentShapeAlloc(), body_, a, b, r);
}

void cpSegmentShapeSetNeighbors(cpShape* shape, cpVect prev, cpVect next)
{
    cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
    cpSegmentShape* seg = cast(cpSegmentShape*)shape;

    seg.a_tangent = cpvsub(prev, seg.a);
    seg.b_tangent = cpvsub(next, seg.b);
}

// Unsafe API (chipmunk_unsafe.h)

void cpCircleShapeSetRadius(cpShape* shape, cpFloat radius)
{
    cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
    cpCircleShape* circle = cast(cpCircleShape*)shape;

    circle.r = radius;
}

void cpCircleShapeSetOffset(cpShape* shape, cpVect offset)
{
    cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
    cpCircleShape* circle = cast(cpCircleShape*)shape;

    circle.c = offset;
}

void cpSegmentShapeSetEndpoints(cpShape* shape, cpVect a, cpVect b)
{
    cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
    cpSegmentShape* seg = cast(cpSegmentShape*)shape;

    seg.a = a;
    seg.b = b;
    seg.n = cpvperp(cpvnormalize(cpvsub(b, a)));
}

void cpSegmentShapeSetRadius(cpShape* shape, cpFloat radius)
{
    cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
    cpSegmentShape* seg = cast(cpSegmentShape*)shape;

    seg.r = radius;
}+/


/// Point query info struct.
struct cpPointQueryInfo 
{
	/// The nearest shape, NULL if no shape was within range.
	cpShape* shape;
	/// The closest point on the shape's surface. (in world space coordinates)
	cpVect point;
	/// The distance to the point. The distance is negative if the point is inside the shape.
	cpFloat distance = 0;
	/// The gradient of the signed distance function.
	/// The value should be similar to info.p/info.d, but accurate even for very small values of info.d.
	cpVect gradient;
} 

/// Segment query info struct.
struct cpSegmentQueryInfo 
{
	/// The shape that was hit, or NULL if no collision occured.
	cpShape* shape;
	/// The point of impact.
	cpVect point;
	/// The normal of the surface hit.
	cpVect normal;
	/// The normalized distance along the query segment in the range [0, 1].
	cpFloat alpha = 0;
} 

/// Fast collision filtering type that is used to determine if two objects collide before calling collision or query callbacks.
struct cpShapeFilter 
{
	/// Two objects with the same non-zero group value do not collide.
	/// This is generally used to group objects in a composite object together to disable self collisions.
	cpGroup group;
	/// A bitmask of user definable categories that this object belongs to.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	cpBitmask categories;
	/// A bitmask of user definable category types that this object object collides with.
	/// The category/mask combinations of both objects in a collision must agree for a collision to occur.
	cpBitmask mask;
} 

/*
/// Collision filter value for a shape that will collide with anything except CP_SHAPE_FILTER_NONE.
static const cpShapeFilter CP_SHAPE_FILTER_ALL = {CP_NO_GROUP, CP_ALL_CATEGORIES, CP_ALL_CATEGORIES};
/// Collision filter value for a shape that does not collide with anything.
static const cpShapeFilter CP_SHAPE_FILTER_NONE = {CP_NO_GROUP, ~CP_ALL_CATEGORIES, ~CP_ALL_CATEGORIES};*/

/// Create a new collision filter.
static cpShapeFilter cpShapeFilterNew(cpGroup group, cpBitmask categories, cpBitmask mask)
{
	cpShapeFilter filter = {group, categories, mask};
	return filter;
}


cpShape* cpShapeInit(cpShape* shape, const cpShapeClass* klass, cpBody* body_, cpShapeMassInfo massInfo)
{
	shape.klass = klass;
	
	shape.body_ = body_;
	shape.massInfo = massInfo;
	
	shape.sensor = 0;
	
	shape.e = 0.0f;
	shape.u = 0.0f;
	shape.surfaceV = cpvzero;
	
	shape.type = 0;
	shape.filter.group = CP_NO_GROUP;
	shape.filter.categories = CP_ALL_CATEGORIES;
	shape.filter.mask = CP_ALL_CATEGORIES;
	
	shape.userData = null;
	
	shape.space = null;
	
	shape.next = null;
	shape.prev = null;
	
	return shape;
}

/// Destroy a shape.
void cpShapeDestroy(cpShape* shape)
{
	if(shape.klass && shape.klass.destroy) 
		shape.klass.destroy(shape);
}

/// Destroy and Free a shape.
void cpShapeFree(cpShape* shape)
{
	if(shape)
	{
		cpShapeDestroy(shape);
		cpfree(shape);
	}
}

/// Update, cache and return the bounding box of a shape based on the body it's attached to.
cpBB cpShapeCacheBB(cpShape* shape)
{
	return cpShapeUpdate(shape, shape.body_.transform);
}

/// Update, cache and return the bounding box of a shape with an explicit transformation.
cpBB cpShapeUpdate(cpShape* shape, cpTransform transform)
{
	return (shape.bb = shape.klass.cacheData(shape, transform));
}

/// Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
/// The value returned is the distance between the points. A negative distance means the point is inside the shape.
cpFloat cpShapePointQuery(const cpShape* shape, cpVect p, cpPointQueryInfo* info)
{
	cpPointQueryInfo blank = {NULL, cpvzero, INFINITY, cpvzero};
	if(info)
	{
		(*info) = blank;
	} 
	else 
	{
		info = &blank;
	}
	
	shape.klass.pointQuery(shape, p, info);
	return info.distance;
}

/// Perform a segment query against a shape. @c info must be a pointer to a valid cpSegmentQueryInfo structure.
cpBool cpShapeSegmentQuery(const cpShape* shape, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo* info)
{
	cpSegmentQueryInfo blank = {NULL, b, cpvzero, 1.0f};
	if(info)
	{
		(*info) = blank;
	} 
	else 
	{
		info = &blank;
	}
	
	cpPointQueryInfo nearest;
	shape.klass.pointQuery(shape, a, &nearest);
	if(nearest.distance <= radius)
	{
		info.shape = shape;
		info.alpha = 0.0;
		info.normal = cpvnormalize(cpvsub(a, nearest.point));
	} 
	else 
	{
		shape.klass.segmentQuery(shape, a, b, radius, info);
	}
	
	return (info.shape != NULL);
}

/// Return contact information about two shapes.
cpContactPointSet cpShapesCollide(const cpShape* a, const cpShape* b)
{
	cpContact[CP_MAX_CONTACTS_PER_ARBITER] contacts;
	cpCollisionInfo info = cpCollide(a, b, 0, contacts);
	
	cpContactPointSet set;
	set.count = info.count;
	
	// cpCollideShapes() may have swapped the contact order. Flip the normal.
	cpBool swapped = (a != info.a);
	set.normal = (swapped ? cpvneg(info.n) : info.n);
	
	for(int i=0; i<info.count; i++)
	{
		// cpCollideShapesInfo() returns contacts with absolute positions.
		cpVect p1 = contacts[i].r1;
		cpVect p2 = contacts[i].r2;
		
		set.points[i].pointA = (swapped ? p2 : p1);
		set.points[i].pointB = (swapped ? p1 : p2);
		set.points[i].distance = cpvdot(cpvsub(p2, p1), set.normal);
	}
	
	return set;
}

/// The cpSpace this body is added to.
cpSpace* cpShapeGetSpace(const cpShape* shape)
{
	return shape.space;
}

/// The cpBody this shape is connected to.
cpBody* cpShapeGetBody(const cpShape* shape)
{
	return shape.body_;
}

/// Set the cpBody this shape is connected to.
/// Can only be used if the shape is not currently added to a space.
void cpShapeSetBody(cpShape* shape, cpBody* body_)
{
	cpAssertHard(!cpShapeActive(shape), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
	shape.body_ = body_;
}

/// Get the mass of the shape if you are having Chipmunk calculate mass properties for you.
cpFloat cpShapeGetMass(cpShape* shape)
{ 
	return shape.massInfo.m; 
}

/// Set the mass of this shape to have Chipmunk calculate mass properties for you.
void cpShapeSetMass(cpShape* shape, cpFloat mass)
{
	cpBody* body_ = shape.body_;
	cpBodyActivate(body_);
	
	shape.massInfo.m = mass;
	cpBodyAccumulateMassFromShapes(body_);
}

/// Get the density of the shape if you are having Chipmunk calculate mass properties for you.
cpFloat cpShapeGetDensity(cpShape* shape)
{ 
	return shape.massInfo.m/shape.massInfo.area; 
}

/// Set the density  of this shape to have Chipmunk calculate mass properties for you.
void cpShapeSetDensity(cpShape* shape, cpFloat density)
{ 
	cpShapeSetMass(shape, density*shape.massInfo.area); 
}

/// Get the calculated moment of inertia for this shape.
cpFloat cpShapeGetMoment(cpShape* shape)
{
	return shape.massInfo.m*shape.massInfo.i;
}

/// Get the calculated area of this shape.
cpFloat cpShapeGetArea(cpShape* shape)
{
	return shape.massInfo.area;
}

/// Get the centroid of this shape.
cpVect cpShapeGetCenterOfGravity(cpShape* shape)
{
	return shape.massInfo.cog;
}

/// Get the bounding box that contains the shape given it's current position and angle.
cpBB cpShapeGetBB(const cpShape* shape)
{
	return shape.bb;
}

/// Get if the shape is set to be a sensor or not.
cpBool cpShapeGetSensor(const cpShape* shape)
{
	return shape.sensor;
}

/// Set if the shape is a sensor or not.
void cpShapeSetSensor(cpShape* shape, cpBool sensor)
{
	cpBodyActivate(shape.body_);
	shape.sensor = sensor;
}

/// Get the elasticity of this shape.
cpFloat cpShapeGetElasticity(const cpShape* shape)
{
	return shape.e;
}

/// Set the elasticity of this shape.
void cpShapeSetElasticity(cpShape* shape, cpFloat elasticity)
{
	cpAssertHard(elasticity >= 0.0f, "Elasticity must be positive.");
	cpBodyActivate(shape.body_);
	shape.e = elasticity;
}

/// Get the friction of this shape.
cpFloat cpShapeGetFriction(const cpShape* shape)
{
	return shape.u;
}

/// Set the friction of this shape.
void cpShapeSetFriction(cpShape* shape, cpFloat friction)
{
	cpAssertHard(friction >= 0.0f, "Friction must be postive.");
	cpBodyActivate(shape.body_);
	shape.u = friction;
}

/// Get the surface velocity of this shape.
cpVect cpShapeGetSurfaceVelocity(const cpShape* shape)
{
	return shape.surfaceV;
}

/// Set the surface velocity of this shape.
void cpShapeSetSurfaceVelocity(cpShape* shape, cpVect surfaceVelocity)
{
	cpBodyActivate(shape.body_);
	shape.surfaceV = surfaceVelocity;
}

/// Get the user definable data pointer of this shape.
cpDataPointer cpShapeGetUserData(const cpShape* shape)
{
	return shape.userData;
}

/// Set the user definable data pointer of this shape.
void cpShapeSetUserData(cpShape* shape, cpDataPointer userData)
{
	shape.userData = userData;
}

/// Set the collision type of this shape.
cpCollisionType cpShapeGetCollisionType(const cpShape* shape)
{
	return shape.type;
}

/// Get the collision type of this shape.
void cpShapeSetCollisionType(cpShape* shape, cpCollisionType collisionType)
{
	cpBodyActivate(shape.body_);
	shape.type = collisionType;
}

/// Get the collision filtering parameters of this shape.
cpShapeFilter cpShapeGetFilter(const cpShape* shape)
{
	return shape.filter;
}

/// Set the collision filtering parameters of this shape.
void cpShapeSetFilter(cpShape* shape, cpShapeFilter filter)
{
	cpBodyActivate(shape.body_);
	shape.filter = filter;
}

/// Allocate a circle shape.
cpCircleShape* cpCircleShapeAlloc()
{
	return cast(cpCircleShape*)cpcalloc(1, cpCircleShape.sizeof);
}

static cpBB cpCircleShapeCacheData(cpCircleShape* circle, cpTransform transform)
{
	cpVect c = circle.tc = cpTransformPoint(transform, circle.c);
	return cpBBNewForCircle(c, circle.r);
}
static void cpCircleShapePointQuery(cpCircleShape* circle, cpVect p, cpPointQueryInfo* info)
{
	cpVect delta = cpvsub(p, circle.tc);
	cpFloat d = cpvlength(delta);
	cpFloat r = circle.r;
	
	info.shape = cast(cpShape*)circle;
	info.point = cpvadd(circle.tc, cpvmult(delta, r/d)); // TODO: div/0
	info.distance = d - r;
	
	// Use up for the gradient if the distance is very small.
	info.gradient = (d > MAGIC_EPSILON ? cpvmult(delta, 1.0f/d) : cpv(0.0f, 1.0f));
}

static void cpCircleShapeSegmentQuery(cpCircleShape* circle, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo* info)
{
	CircleSegmentQuery(cast(cpShape*)circle, circle.tc, circle.r, a, b, radius, info);
}

static cpShapeMassInfo cpCircleShapeMassInfo(cpFloat mass, cpFloat radius, cpVect center)
{
	cpShapeMassInfo info = {
		mass, cpMomentForCircle(1.0f, 0.0f, radius, cpvzero),
		center,
		cpAreaForCircle(0.0f, radius),
	};
	
	return info;
}

__gshared cpShapeClass cpCircleShapeClass = cpShapeClass(		
		cpShapeType.CP_CIRCLE_SHAPE,
		cast(cpShapeCacheDataImpl)&cpCircleShapeCacheData,
		null,
		cast(cpShapePointQueryImpl)&cpCircleShapePointQuery,
		cast(cpShapeSegmentQueryImpl)&cpCircleShapeSegmentQuery,
    );

/// Initialize a circle shape.
cpCircleShape* cpCircleShapeInit(cpCircleShape* circle, cpBody* body_, cpFloat radius, cpVect offset)
{
	circle.c = offset;
	circle.r = radius;
	
	cpShapeInit(cast(cpShape*)circle, &cpCircleShapeClass, body_, cpCircleShapeMassInfo(0.0f, radius, offset));
	
	return circle;
}

/// Allocate and initialize a circle shape.
cpShape* cpCircleShapeNew(cpBody* body_, cpFloat radius, cpVect offset)
{
	return cast(cpShape*)cpCircleShapeInit(cpCircleShapeAlloc(), body_, radius, offset);
}

/// Get the offset of a circle shape.
cpVect cpCircleShapeGetOffset(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	return (cast(cpCircleShape*)shape).c;
}

/// Get the radius of a circle shape.
cpFloat cpCircleShapeGetRadius(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	return (cast(cpCircleShape*)shape).r;
}

	
/// Allocate a segment shape.
cpSegmentShape* cpSegmentShapeAlloc()
{
	return cast(cpSegmentShape*)cpcalloc(1, cpSegmentShape.sizeof);
}

static cpBB cpSegmentShapeCacheData(cpSegmentShape* seg, cpTransform transform)
{
	seg.ta = cpTransformPoint(transform, seg.a);
	seg.tb = cpTransformPoint(transform, seg.b);
	seg.tn = cpTransformVect(transform, seg.n);
	
	cpFloat l,r,b,t;
	
	if(seg.ta.x < seg.tb.x){
		l = seg.ta.x;
		r = seg.tb.x;
	} else {
		l = seg.tb.x;
		r = seg.ta.x;
	}
	
	if(seg.ta.y < seg.tb.y){
		b = seg.ta.y;
		t = seg.tb.y;
	} else {
		b = seg.tb.y;
		t = seg.ta.y;
	}
	
	cpFloat rad = seg.r;
	return cpBBNew(l - rad, b - rad, r + rad, t + rad);
}

static void cpSegmentShapePointQuery(cpSegmentShape* seg, cpVect p, cpPointQueryInfo* info)
{
	cpVect closest = cpClosetPointOnSegment(p, seg.ta, seg.tb);
	
	cpVect delta = cpvsub(p, closest);
	cpFloat d = cpvlength(delta);
	cpFloat r = seg.r;
	cpVect g = cpvmult(delta, 1.0f/d);
	
	info.shape = cast(cpShape*)seg;
	info.point = (d ? cpvadd(closest, cpvmult(g, r)) : closest);
	info.distance = d - r;
	
	// Use the segment's normal if the distance is very small.
	info.gradient = (d > MAGIC_EPSILON ? g : seg.n);
}

static void cpSegmentShapeSegmentQuery(cpSegmentShape* seg, cpVect a, cpVect b, cpFloat r2, cpSegmentQueryInfo* info)
{
	cpVect n = seg.tn;
	cpFloat d = cpvdot(cpvsub(seg.ta, a), n);
	cpFloat r = seg.r + r2;
	
	cpVect flipped_n = (d > 0.0f ? cpvneg(n) : n);
	cpVect seg_offset = cpvsub(cpvmult(flipped_n, r), a);
	
	// Make the endpoints relative to 'a' and move them by the thickness of the segment.
	cpVect seg_a = cpvadd(seg.ta, seg_offset);
	cpVect seg_b = cpvadd(seg.tb, seg_offset);
	cpVect delta = cpvsub(b, a);
	
	if(cpvcross(delta, seg_a)*cpvcross(delta, seg_b) <= 0.0f)
	{
		cpFloat d_offset = d + (d > 0.0f ? -r : r);
		cpFloat ad = -d_offset;
		cpFloat bd = cpvdot(delta, n) - d_offset;
		
		if(ad*bd < 0.0f)
		{
			cpFloat t = ad/(ad - bd);
			
			info.shape = cast(cpShape*)seg;
			info.point = cpvsub(cpvlerp(a, b, t), cpvmult(flipped_n, r2));
			info.normal = flipped_n;
			info.alpha = t;
		}
	} 
	else 
		if(r != 0.0f)
		{
			cpSegmentQueryInfo info1 = {NULL, b, cpvzero, 1.0f};
			cpSegmentQueryInfo info2 = {NULL, b, cpvzero, 1.0f};
			CircleSegmentQuery(cast(cpShape*)seg, seg.ta, seg.r, a, b, r2, &info1);
			CircleSegmentQuery(cast(cpShape*)seg, seg.tb, seg.r, a, b, r2, &info2);
		
			if(info1.alpha < info2.alpha)
			{
				(*info) = info1;
			} 
			else 
			{
				(*info) = info2;
			}
		}
}

static cpShapeMassInfo cpSegmentShapeMassInfo(cpFloat mass, cpVect a, cpVect b, cpFloat r)
{
	cpShapeMassInfo info = {
		mass, cpMomentForBox(1.0f, cpvdist(a, b) + 2.0f*r, 2.0f*r), // TODO is an approximation.
		cpvlerp(a, b, 0.5f),
		cpAreaForSegment(a, b, r),
	};
	
	return info;
}

__gshared cpShapeClass cpSegmentShapeClass = cpShapeClass(		
		cpShapeType.CP_SEGMENT_SHAPE,
		cast(cpShapeCacheDataImpl)&cpSegmentShapeCacheData,
		null,
		cast(cpShapePointQueryImpl)&cpSegmentShapePointQuery,
		cast(cpShapeSegmentQueryImpl)&cpSegmentShapeSegmentQuery,
    );
	
/// Initialize a segment shape.
cpSegmentShape* cpSegmentShapeInit(cpSegmentShape* seg, cpBody* body_, cpVect a, cpVect b, cpFloat radius)
{
	seg.a = a;
	seg.b = b;
	seg.n = cpvrperp(cpvnormalize(cpvsub(b, a)));
	
	seg.r = r;
	
	seg.a_tangent = cpvzero;
	seg.b_tangent = cpvzero;
	
	cpShapeInit(cast(cpShape*)seg, &cpSegmentShapeClass, body_, cpSegmentShapeMassInfo(0.0f, a, b, r));
	
	return seg;
}

/// Allocate and initialize a segment shape.
cpShape* cpSegmentShapeNew(cpBody* body_, cpVect a, cpVect b, cpFloat radius)
{
	return cast(cpShape*)cpSegmentShapeInit(cpSegmentShapeAlloc(), body_, a, b, r);
}

/// Let Chipmunk know about the geometry of adjacent segments to avoid colliding with endcaps.
void cpSegmentShapeSetNeighbors(cpShape* shape, cpVect prev, cpVect next)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	cpSegmentShape* seg = cast(cpSegmentShape*)shape;
	
	seg.a_tangent = cpvsub(prev, seg.a);
	seg.b_tangent = cpvsub(next, seg.b);
}

/// Get the first endpoint of a segment shape.
cpVect cpSegmentShapeGetA(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	return (cast(cpSegmentShape*)shape).a;
}

/// Get the second endpoint of a segment shape.
cpVect cpSegmentShapeGetB(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	return (cast(cpSegmentShape*)shape).b;
}

/// Get the normal of a segment shape.
cpVect cpSegmentShapeGetNormal(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	return (cast(cpSegmentShape*)shape).n;
}

/// Get the first endpoint of a segment shape.
cpFloat cpSegmentShapeGetRadius(const cpShape* shape)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	return (cast(cpSegmentShape*)shape).r;
}


// Unsafe API (chipmunk_unsafe.h)

// TODO setters should wake the shape up?

void cpCircleShapeSetRadius(cpShape* shape, cpFloat radius)
{
	cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	cpCircleShape* circle = cast(cpCircleShape*)shape;
	
	circle.r = radius;
	
	cpFloat mass = shape.massInfo.m;
	shape.massInfo = cpCircleShapeMassInfo(mass, circle.r, circle.c);
	if(mass > 0.0f) 
		cpBodyAccumulateMassFromShapes(shape.body_);
}

void cpCircleShapeSetOffset(cpShape* shape, cpVect offset)
{
	cpAssertHard(shape.klass == &cpCircleShapeClass, "Shape is not a circle shape.");
	cpCircleShape* circle = cast(cpCircleShape*)shape;
	
	circle.c = offset;

	cpFloat mass = shape.massInfo.m;
	shape.massInfo = cpCircleShapeMassInfo(shape.massInfo.m, circle.r, circle.c);
	if(mass > 0.0f) 
		cpBodyAccumulateMassFromShapes(shape.body_);
}

void cpSegmentShapeSetEndpoints(cpShape* shape, cpVect a, cpVect b)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	cpSegmentShape* seg = cast(cpSegmentShape*)shape;
	
	seg.a = a;
	seg.b = b;
	seg.n = cpvperp(cpvnormalize(cpvsub(b, a)));

	cpFloat mass = shape.massInfo.m;
	shape.massInfo = cpSegmentShapeMassInfo(shape.massInfo.m, seg.a, seg.b, seg.r);
	if(mass > 0.0f) 
		cpBodyAccumulateMassFromShapes(shape.body_);
}

void cpSegmentShapeSetRadius(cpShape* shape, cpFloat radius)
{
	cpAssertHard(shape.klass == &cpSegmentShapeClass, "Shape is not a segment shape.");
	cpSegmentShape* seg = cast(cpSegmentShape*)shape;
	
	seg.r = radius;

	cpFloat mass = shape.massInfo.m;
	shape.massInfo = cpSegmentShapeMassInfo(shape.massInfo.m, seg.a, seg.b, seg.r);
	if(mass > 0.0f)
		cpBodyAccumulateMassFromShapes(shape.body_);
}
