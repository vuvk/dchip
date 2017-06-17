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
module dchip.cpCollision;

import core.stdc.stdlib : alloca;

import std.string;

import dchip.cpArbiter;
import dchip.cpBB;
import dchip.cpBody;
import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;

import dchip.cpRobust;
import dchip.cpPolyShape;
import dchip.cpShape;
import dchip.cpSpace;
import dchip.cpVect;

import dchip.util;

/+ #if DEBUG && 0
#include "ChipmunkDemo.h"
#define DRAW_ALL 0
#define DRAW_GJK (0 || DRAW_ALL)
#define DRAW_EPA (0 || DRAW_ALL)
#define DRAW_CLOSEST (0 || DRAW_ALL)
#define DRAW_CLIP (0 || DRAW_ALL)

#define PRINT_LOG 0
#endif +/

/* TODO : DELETE
enum ENABLE_CACHING = 1;*/

enum MAX_GJK_ITERATIONS = 30;
enum MAX_EPA_ITERATIONS = 30;
enum WARN_GJK_ITERATIONS = 20;
enum WARN_EPA_ITERATIONS = 20;

/* TODO : DELETE
// Add contact points for circle to circle collisions.
// Used by several collision tests.
// TODO should accept hash parameter
int CircleToCircleQuery(const cpVect p1, const cpVect p2, const cpFloat r1, const cpFloat r2, cpHashValue hash, cpContact* con)
{
    cpFloat mindist = r1 + r2;
    cpVect  delta   = cpvsub(p2, p1);
    cpFloat distsq  = cpvlengthsq(delta);

    if (distsq < mindist * mindist)
    {
        cpFloat dist = cpfsqrt(distsq);
        cpVect  n    = (dist ? cpvmult(delta, 1.0f / dist) : cpv(1.0f, 0.0f));
        cpContactInit(con, cpvlerp(p1, p2, r1 / (r1 + r2)), n, dist - mindist, hash);

        return 1;
    }
    else
    {
        return 0;
    }
}*/

static void cpCollisionInfoPushContact(cpCollisionInfo* info, cpVect p1, cpVect p2, cpHashValue hash)
{
	cpAssertSoft(info.count <= CP_MAX_CONTACTS_PER_ARBITER, "Internal error: Tried to push too many contacts.");
	
	cpContact* con = &info.arr[info.count];
	con.r1 = p1;
	con.r2 = p2;
	con.hash = hash;
	
	info.count++;
}

//MARK: Support Points and Edges:

// Support points are the maximal points on a shape's perimeter along a certain axis.
// The GJK and EPA algorithms use support points to iteratively sample the surface of the two shapes' minkowski difference.
/* TODO : DELETE
int PolySupportPointIndex(const int count, const cpVect* verts, const cpVect n)*/
static int PolySupportPointIndex(const int count, const cpSplittingPlane* planes, const cpVect n)
{
    cpFloat max = -INFINITY;
    int index   = 0;

    for (int i = 0; i < count; i++)
    {
        /* TODO : DELETE
		cpVect  v = verts[i];
        cpFloat d = cpvdot(v, n);*/
		
		cpVect v = planes[i].v0;
		cpFloat d = cpvdot(v, n);

        if (d > max)
        {
            max   = d;
            index = i;
        }
    }

    return index;
}

struct SupportPoint
{
    cpVect p;
    /* TODO : DELETE
	cpCollisionID id;*/
	
	// Save an index of the point so it can be cheaply looked up as a starting point for the next frame.
	cpCollisionID index;
}

/* TODO : DELETE
SupportPoint SupportPointNew(cpVect p, cpCollisionID id)
{
    SupportPoint point = { p, id };*/
static SupportPoint SupportPointNew(cpVect p, cpCollisionID index)
{
	SupportPoint point = {p, index};
	return point;
}

alias SupportPointFunc = SupportPoint function(const cpShape* shape, const cpVect n);

static SupportPoint CircleSupportPoint(const cpCircleShape* circle, const cpVect n)
{
    return SupportPointNew(circle.tc, 0);
}

static SupportPoint SegmentSupportPoint(const cpSegmentShape* seg, const cpVect n)
{
    if (cpvdot(seg.ta, n) > cpvdot(seg.tb, n))
    {
        return SupportPointNew(seg.ta, 0);
    }
    else
    {
        return SupportPointNew(seg.tb, 1);
    }
}

static SupportPoint PolySupportPoint(const cpPolyShape* poly, const cpVect n)
{
	/* TODO : DELETE
    const cpVect* verts = poly.tVerts;
    int i = PolySupportPointIndex(poly.numVerts, verts, n);
    return SupportPointNew(verts[i], i);*/
	
	cpSplittingPlane* planes = cast(cpSplittingPlane*)poly.planes;
	int i = PolySupportPointIndex(poly.count, planes, n);
	return SupportPointNew(planes[i].v0, i);
}

// A point on the surface of two shape's minkowski difference.
struct MinkowskiPoint
{
	// Cache the two original support points.
    cpVect a, b;
	// b - a
    cpVect ab;
	// Concatenate the two support point indexes.
    cpCollisionID id;
}

static MinkowskiPoint MinkowskiPointNew(const SupportPoint a, const SupportPoint b)
{
	/* TODO : DELETE
    MinkowskiPoint point = { a.p, b.p, cpvsub(b.p, a.p), (a.id & 0xFF) << 8 | (b.id & 0xFF) };*/
	MinkowskiPoint point = { a.p, b.p, cpvsub(b.p, a.p), (a.index & 0xFF)<<8 | (b.index & 0xFF) };	
    return point;
}

struct SupportContext
{
    cpShape* shape1;
    cpShape* shape2;
    SupportPointFunc func1, func2;
}

// Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
static MinkowskiPoint Support(const SupportContext* ctx, const cpVect n)
{
    SupportPoint a = ctx.func1(ctx.shape1, cpvneg(n));
    SupportPoint b = ctx.func2(ctx.shape2, n);
    return MinkowskiPointNew(a, b);
}

struct EdgePoint
{
    cpVect p;
	// Keep a hash value for Chipmunk's collision hashing mechanism.
    cpHashValue hash;
}

// Support edges are the edges of a polygon or segment shape that are in contact.
struct Edge
{
    EdgePoint a, b;
    cpFloat r = 0;
    cpVect n;
}

/* TODO : DELETE
Edge EdgeNew(cpVect va, cpVect vb, cpHashValue ha, cpHashValue hb, cpFloat r)
{
    Edge edge = { { va, ha }, { vb, hb }, r, cpvnormalize(cpvperp(cpvsub(vb, va))) };
    return edge;
}*/

static Edge SupportEdgeForPoly(const cpPolyShape* poly, const cpVect n)
{
	/* TODO : DELETE
    int numVerts = poly.numVerts;
    int i1       = PolySupportPointIndex(poly.numVerts, poly.tVerts, n);

    // TODO get rid of mod eventually, very expensive on ARM
    int i0 = (i1 - 1 + numVerts) % numVerts;
    int i2 = (i1 + 1) % numVerts;

    cpVect* verts = cast(cpVect*)poly.tVerts;

    if (cpvdot(n, poly.tPlanes[i1].n) > cpvdot(n, poly.tPlanes[i2].n))
    {
        Edge edge = { { verts[i0], CP_HASH_PAIR(poly, i0) }, { verts[i1], CP_HASH_PAIR(poly, i1) }, poly.r, poly.tPlanes[i1].n };
        return edge;
    }
    else
    {
        Edge edge = { { verts[i1], CP_HASH_PAIR(poly, i1) }, { verts[i2], CP_HASH_PAIR(poly, i2) }, poly.r, poly.tPlanes[i2].n };
        return edge;
    }*/
	
	int count = poly.count;
	int i1 = PolySupportPointIndex(poly.count, poly.planes, n);
	
	// TODO: get rid of mod eventually, very expensive on ARM
	int i0 = (i1 - 1 + count)%count;
	int i2 = (i1 + 1)%count;
	
	cpSplittingPlane* planes = cast(cpSplittingPlane*)poly.planes;
	cpHashValue hashid = poly.shape.hashid;
	if(cpvdot(n, planes[i1].n) > cpvdot(n, planes[i2].n))
	{
		Edge edge = {{planes[i0].v0, CP_HASH_PAIR(hashid, i0)}, {planes[i1].v0, CP_HASH_PAIR(hashid, i1)}, poly.r, planes[i1].n};
		return edge;
	} 
	else 
	{
		Edge edge = {{planes[i1].v0, CP_HASH_PAIR(hashid, i1)}, {planes[i2].v0, CP_HASH_PAIR(hashid, i2)}, poly.r, planes[i2].n};
		return edge;
	}
}

static Edge SupportEdgeForSegment(const cpSegmentShape* seg, const cpVect n)
{
	/* TODO : DELETE
    if (cpvdot(seg.tn, n) > 0.0)
    {
        Edge edge = { { seg.ta, CP_HASH_PAIR(seg, 0) }, { seg.tb, CP_HASH_PAIR(seg, 1) }, seg.r, seg.tn };
        return edge;
    }
    else
    {
        Edge edge = { { seg.tb, CP_HASH_PAIR(seg, 1) }, { seg.ta, CP_HASH_PAIR(seg, 0) }, seg.r, cpvneg(seg.tn) };
        return edge;
    }*/
	
	cpHashValue hashid = seg.shape.hashid;
	if (cpvdot(seg.tn, n) > 0.0)
	{
		Edge edge = {{seg.ta, CP_HASH_PAIR(hashid, 0)}, {seg.tb, CP_HASH_PAIR(hashid, 1)}, seg.r, seg.tn};
		return edge;
	}
	else 
	{
		Edge edge = {{seg.tb, CP_HASH_PAIR(hashid, 1)}, {seg.ta, CP_HASH_PAIR(hashid, 0)}, seg.r, cpvneg(seg.tn)};
		return edge;
	}
}

// Find the closest p(t) to (0, 0) where p(t) = a*(1-t)/2 + b*(1+t)/2
// The range for t is [-1, 1] to avoid floating point issues if the parameters are swapped.
static cpFloat ClosestT(const cpVect a, const cpVect b)
{
    cpVect delta = cpvsub(b, a);
    return -cpfclamp(cpvdot(delta, cpvadd(a, b)) / cpvlengthsq(delta), -1.0f, 1.0f);
}

// Basically the same as cpvlerp(), except t = [-1, 1]
static cpVect LerpT(const cpVect a, const cpVect b, const cpFloat t)
{
    cpFloat ht = 0.5f * t;
    return cpvadd(cpvmult(a, 0.5f - ht), cpvmult(b, 0.5f + ht));
}

// Closest points on the surface of two shapes.
struct ClosestPoints
{
    // Surface points in absolute coordinates.
	cpVect a, b;
	// Minimum separating axis of the two shapes.
	cpVect n;
	// Signed distance between the points.
	cpFloat d = 0;
	// Concatenation of the id's of the minkoski points.
	cpCollisionID id;
}

// Calculate the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
static ClosestPoints ClosestPointsNew(const MinkowskiPoint v0, const MinkowskiPoint v1)
{
    // Find the closest p(t) on the minkowski difference to (0, 0)
	cpFloat t = ClosestT(v0.ab, v1.ab);
    cpVect  p = LerpT(v0.ab, v1.ab, t);

    // Interpolate the original support points using the same 't' value as above.
	// This gives you the closest surface points in absolute coordinates. NEAT!
	cpVect pa        = LerpT(v0.a, v1.a, t);
    cpVect pb        = LerpT(v0.b, v1.b, t);
    cpCollisionID id = (v0.id & 0xFFFF) << 16 | (v1.id & 0xFFFF);

    // First try calculating the MSA from the minkowski difference edge.
	// This gives us a nice, accurate MSA when the surfaces are close together.
	cpVect  delta = cpvsub(v1.ab, v0.ab);
    /* TODO : DELETE
	cpVect  n     = cpvnormalize(cpvperp(delta));
    cpFloat d     = -cpvdot(n, p);

    if (d <= 0.0f || (0.0f < t && t < 1.0f))
    {
        ClosestPoints points = { pa, pb, cpvneg(n), d, id };
        return points;
    }
    else
    {
        cpFloat d2 = cpvlength(p);
        cpVect  n_  = cpvmult(p, 1.0f / (d2 + CPFLOAT_MIN));

        ClosestPoints points = { pa, pb, n_, d2, id };
        return points;
    }*/	
	cpVect n = cpvnormalize(cpvrperp(delta));
	cpFloat d = cpvdot(n, p);
	
	if(d <= 0.0f || (-1.0f < t && t < 1.0f))
	{
		// If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
		ClosestPoints points = {pa, pb, n, d, id};
		return points;
	} 
	else
	{
		// Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
		cpFloat d2 = cpvlength(p);
		cpVect n2 = cpvmult(p, 1.0f/(d2 + CPFLOAT_MIN));
		
		ClosestPoints points = {pa, pb, n2, d2, id};
		return points;
	}
}

//MARK: EPA Functions

static cpFloat ClosestDist(const cpVect v0, const cpVect v1)
{
    return cpvlengthsq(LerpT(v0, v1, ClosestT(v0, v1)));
}

// Recursive implementation of the EPA loop.
// Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
static ClosestPoints EPARecurse(const SupportContext* ctx, const int count, const MinkowskiPoint* hull, const int iteration)
{
    int mini        = 0;
    cpFloat minDist = INFINITY;

    // TODO: precalculate this when building the hull and save a step.
    // Find the closest segment hull[i] and hull[i + 1] to (0, 0)
	for (int j = 0, i = count - 1; j < count; i = j, j++)
    {
        cpFloat d = ClosestDist(hull[i].ab, hull[j].ab);

        if (d < minDist)
        {
            minDist = d;
            mini    = i;
        }
    }

    MinkowskiPoint v0 = hull[mini];
    MinkowskiPoint v1 = hull[(mini + 1) % count];
    cpAssertSoft(!cpveql(v0.ab, v1.ab), "!cpveql(v0.ab, v1.ab)", "Internal Error: EPA vertexes are the same (%d and %d)", mini, (mini + 1) % count);

    // Check if there is a point on the minkowski difference beyond this edge.
	MinkowskiPoint p = Support(ctx, cpvperp(cpvsub(v1.ab, v0.ab)));

/+  TODO : DELETE
	#if DRAW_EPA
    cpVect verts[count];

    for (int i = 0; i < count; i++)
        verts[i] = hull[i].ab;

    ChipmunkDebugDrawPolygon(count, verts, RGBAColor(1, 1, 0, 1), RGBAColor(1, 1, 0, 0.25));
    ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 0, 0, 1));

    ChipmunkDebugDrawPoints(5, 1, (cpVect[]){ p.ab }, RGBAColor(1, 1, 1, 1));
#endif +/

/+ #if DRAW_EPA
	cpVect verts[count];
	for(int i=0; i<count; i++) verts[i] = hull[i].ab;
	
	ChipmunkDebugDrawPolygon(count, verts, 0.0, RGBAColor(1, 1, 0, 1), RGBAColor(1, 1, 0, 0.25));
	ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 0, 0, 1));
	
	ChipmunkDebugDrawDot(5, p.ab, LAColor(1, 1));
#endif +/

	/* TODO : DELETE
    cpFloat area2x = cpvcross(cpvsub(v1.ab, v0.ab), cpvadd(cpvsub(p.ab, v0.ab), cpvsub(p.ab, v1.ab)));

    if (area2x > 0.0f && iteration < MAX_EPA_ITERATIONS)
    {*/
	// The usual exit condition is a duplicated vertex.
	// Much faster to check the ids than to check the signed area.
	cpBool duplicate = (p.id == v0.id || p.id == v1.id);
	
	if(!duplicate && cpCheckPointGreater(v0.ab, v1.ab, p.ab) && iteration < MAX_EPA_ITERATIONS)
	{
		// Rebuild the convex hull by inserting p.
		MinkowskiPoint* hull2 = cast(MinkowskiPoint*)alloca((count + 1)*MinkowskiPoint.sizeof);
		int count2 = 1;		
        hull2[0] = p;

        for (int i = 0; i < count; i++)
        {
            int index = (mini + 1 + i) % count;

            cpVect h0 = hull2[count2 - 1].ab;
            cpVect h1 = hull[index].ab;
            cpVect h2 = (i + 1 < count ? hull[(index + 1) % count] : p).ab;

			/* TODO : DELETE
            // TODO: Should this be changed to an area2x check?
            if (cpvcross(cpvsub(h2, h0), cpvsub(h1, h0)) > 0.0f)
            {*/
			if(cpCheckPointGreater(h0, h2, h1))
			{
                hull2[count2] = hull[index];
                count2++;
            }
        }

        return EPARecurse(ctx, count2, hull2, iteration + 1);
    }
    else
    {
        // Could not find a new point to insert, so we have found the closest edge of the minkowski difference.
		cpAssertWarn(iteration < WARN_EPA_ITERATIONS, "High EPA iterations: %d", iteration);
        return ClosestPointsNew(v0, v1);
    }
}

// Find the closest points on the surface of two overlapping shapes using the EPA algorithm.
// EPA is called from GJK when two shapes overlap.
// This is a moderately expensive step! Avoid it by adding radii to your shapes so their inner polygons won't overlap.
static ClosestPoints EPA(const SupportContext* ctx, const MinkowskiPoint v0, const MinkowskiPoint v1, const MinkowskiPoint v2)
{
    // TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse?
	MinkowskiPoint[3] hull;
    hull[0] = v0;
    hull[1] = v1;
    hull[2] = v2;
    return EPARecurse(ctx, 3, hull.ptr, 1);
}

//MARK: GJK Functions.

// Recursive implementation of the GJK loop.
static ClosestPoints GJKRecurse(const SupportContext* ctx, const MinkowskiPoint v0, const MinkowskiPoint v1, const int iteration)
{
    if (iteration > MAX_GJK_ITERATIONS)
    {
        cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: %d", iteration);
        return ClosestPointsNew(v0, v1);
    }

    /* TODO : DELETE
	cpVect delta = cpvsub(v1.ab, v0.ab);

    if (cpvcross(delta, cpvadd(v0.ab, v1.ab)) > 0.0f)
    {
        // Origin is behind axis. Flip and try again.
        return GJKRecurse(ctx, v1, v0, iteration + 1);
    }*/
	if(cpCheckPointGreater(v1.ab, v0.ab, cpvzero))
	{
		// Origin is behind axis. Flip and try again.
		return GJKRecurse(ctx, v1, v0, iteration);
	}
    else
    {
        cpFloat t = ClosestT(v0.ab, v1.ab);
        /* TODO : DELETE
		cpVect  n = (-1.0f < t && t < 1.0f ? cpvperp(delta) : cpvneg(LerpT(v0.ab, v1.ab, t)));*/
        cpVect n = (-1.0f < t && t < 1.0f ? cpvperp(cpvsub(v1.ab, v0.ab)) : cpvneg(LerpT(v0.ab, v1.ab, t)));
		MinkowskiPoint p = Support(ctx, n);

/+ TODO : DELETE
	#if DRAW_GJK
        ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 1, 1, 1));
        cpVect c = cpvlerp(v0.ab, v1.ab, 0.5);
        ChipmunkDebugDrawSegment(c, cpvadd(c, cpvmult(cpvnormalize(n), 5.0)), RGBAColor(1, 0, 0, 1));

        ChipmunkDebugDrawPoints(5.0, 1, &p.ab, RGBAColor(1, 1, 1, 1));
#endif +/

/* #if DRAW_GJK
		ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 1, 1, 1));
		cpVect c = cpvlerp(v0.ab, v1.ab, 0.5);
		ChipmunkDebugDrawSegment(c, cpvadd(c, cpvmult(cpvnormalize(n), 5.0)), RGBAColor(1, 0, 0, 1));
		
		ChipmunkDebugDrawDot(5.0, p.ab, LAColor(1, 1));
#endif */

        /* TODO : DELETE
		if (
            cpvcross(cpvsub(v1.ab, p.ab), cpvadd(v1.ab, p.ab)) > 0.0f &&
            cpvcross(cpvsub(v0.ab, p.ab), cpvadd(v0.ab, p.ab)) < 0.0f
            )
        {
            cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK.EPA iterations: %d", iteration);
		*/
		if(cpCheckPointGreater(p.ab, v0.ab, cpvzero) && cpCheckPointGreater(v1.ab, p.ab, cpvzero))
		{
			// The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
			cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK->EPA iterations: %d", iteration);
			
            // The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
            return EPA(ctx, v0, p, v1);
        }
        else
        {
			/* TODO : DELETE
            // The new point must be farther along the normal than the existing points.
            if (cpvdot(p.ab, n) <= cpfmax(cpvdot(v0.ab, n), cpvdot(v1.ab, n)))
            {*/
			if(cpCheckAxis(v0.ab, v1.ab, p.ab, n))
			{
				// The edge v0, v1 that we already have is the closest to (0, 0) since p was not closer.
				cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: %d", iteration);
                return ClosestPointsNew(v0, v1);
            }
            else
            {
                // p was closer to the origin than our existing edge.
				// Need to figure out which existing point to drop.
				if (ClosestDist(v0.ab, p.ab) < ClosestDist(p.ab, v1.ab))
                {
                    return GJKRecurse(ctx, v0, p, iteration + 1);
                }
                else
                {
                    return GJKRecurse(ctx, p, v1, iteration + 1);
                }
            }
        }
    }
}

// Get a SupportPoint from a cached shape and index.
static SupportPoint ShapePoint(const cpShape* shape, const int i)
{
    switch (shape.klass.type)
    {
        case cpShapeType.CP_CIRCLE_SHAPE:
        {
            return SupportPointNew((cast(cpCircleShape*)shape).tc, 0);
        }

        case cpShapeType.CP_SEGMENT_SHAPE:
        {
            cpSegmentShape* seg = cast(cpSegmentShape*)shape;
            return SupportPointNew(i == 0 ? seg.ta : seg.tb, i);
        }

        case cpShapeType.CP_POLY_SHAPE:
        {
            cpPolyShape* poly = cast(cpPolyShape*)shape;

            // Poly shapes may change vertex count.
            /* TODO : DELETE
			int index = (i < poly.numVerts ? i : 0);
            return SupportPointNew(poly.tVerts[index], index);*/
			int index = (i < poly.count ? i : 0);
			return SupportPointNew(poly.planes[index].v0, index);
        }

        default:
        {
            return SupportPointNew(cpvzero, 0);
        }
    }
}

// Find the closest points between two shapes using the GJK algorithm.
static ClosestPoints GJK(const SupportContext* ctx, cpCollisionID* id)
{
/+ TODO : DELETE
	#if DRAW_GJK || DRAW_EPA

    // draw the minkowski difference origin
    cpVect origin = cpvzero;
    ChipmunkDebugDrawPoints(5.0, 1, &origin, RGBAColor(1, 0, 0, 1));

    int mdiffCount     = ctx.count1 * ctx.count2;
    cpVect* mdiffVerts = alloca(mdiffCount * sizeof(cpVect));

    for (int i = 0; i < ctx.count1; i++)
    {
        for (int j = 0; j < ctx.count2; j++)
        {
            cpVect v1 = ShapePoint(ctx.count1, ctx.verts1, i).p;
            cpVect v2 = ShapePoint(ctx.count2, ctx.verts2, j).p;
            mdiffVerts[i * ctx.count2 + j] = cpvsub(v2, v1);
        }
    }

    cpVect* hullVerts = alloca(mdiffCount * sizeof(cpVect));
    int hullCount     = cpConvexHull(mdiffCount, mdiffVerts, hullVerts, null, 0.0);

    ChipmunkDebugDrawPolygon(hullCount, hullVerts, RGBAColor(1, 0, 0, 1), RGBAColor(1, 0, 0, 0.25));
    ChipmunkDebugDrawPoints(2.0, mdiffCount, mdiffVerts, RGBAColor(1, 0, 0, 1));
#endif +/

/+ #if DRAW_GJK || DRAW_EPA
	int count1 = 1;
	int count2 = 1;
	
	switch(ctx->shape1->klass->type){
		case CP_SEGMENT_SHAPE: count1 = 2; break;
		case CP_POLY_SHAPE: count1 = ((cpPolyShape *)ctx->shape1)->count; break;
		default: break;
	}
	
	switch(ctx->shape2->klass->type){
		case CP_SEGMENT_SHAPE: count1 = 2; break;
		case CP_POLY_SHAPE: count2 = ((cpPolyShape *)ctx->shape2)->count; break;
		default: break;
	}
	
	
	// draw the minkowski difference origin
	cpVect origin = cpvzero;
	ChipmunkDebugDrawDot(5.0, origin, RGBAColor(1,0,0,1));
	
	int mdiffCount = count1*count2;
	cpVect *mdiffVerts = alloca(mdiffCount*sizeof(cpVect));
	
	for(int i=0; i<count1; i++){
		for(int j=0; j<count2; j++){
			cpVect v = cpvsub(ShapePoint(ctx->shape2, j).p, ShapePoint(ctx->shape1, i).p);
			mdiffVerts[i*count2 + j] = v;
			ChipmunkDebugDrawDot(2.0, v, RGBAColor(1, 0, 0, 1));
		}
	}
	 
	cpVect *hullVerts = alloca(mdiffCount*sizeof(cpVect));
	int hullCount = cpConvexHull(mdiffCount, mdiffVerts, hullVerts, null, 0.0);
	
	ChipmunkDebugDrawPolygon(hullCount, hullVerts, 0.0, RGBAColor(1, 0, 0, 1), RGBAColor(1, 0, 0, 0.25));
#endif +/

    MinkowskiPoint v0, v1;

    // TODO : DELETE if (*id && ENABLE_CACHING)

    if (*id)
    {
        // Use the minkowski points from the last frame as a starting point using the cached indexes.
		v0 = MinkowskiPointNew(ShapePoint(ctx.shape1, (*id >> 24) & 0xFF), ShapePoint(ctx.shape2, (*id >> 16) & 0xFF));
        v1 = MinkowskiPointNew(ShapePoint(ctx.shape1, (*id >> 8) & 0xFF), ShapePoint(ctx.shape2, (*id    ) & 0xFF));
    }
    else
    {
        // No cached indexes, use the shapes' bounding box centers as a guess for a starting axis.
		cpVect axis = cpvperp(cpvsub(cpBBCenter(ctx.shape1.bb), cpBBCenter(ctx.shape2.bb)));
        v0 = Support(ctx, axis);
        v1 = Support(ctx, cpvneg(axis));
    }

    ClosestPoints points = GJKRecurse(ctx, v0, v1, 1);
    *id = points.id;
    return points;
}

//MARK: Contact Clipping

/+ TODO : DELETE
void Contact1(cpFloat dist, cpVect a, cpVect b, cpFloat refr, cpFloat incr, cpVect n, cpHashValue hash, cpContact* arr)
{
    cpFloat rsum  = refr + incr;
    cpFloat alpha = (rsum > 0.0f ? refr / rsum : 0.5f);
    cpVect  point = cpvlerp(a, b, alpha);

    cpContactInit(arr, point, n, dist - rsum, hash);
}

int Contact2(cpVect refp, cpVect inca, cpVect incb, cpFloat refr, cpFloat incr, cpVect refn, cpVect n, cpHashValue hash, cpContact* arr)
{
    cpFloat cian = cpvcross(inca, refn);
    cpFloat cibn = cpvcross(incb, refn);
    cpFloat crpn = cpvcross(refp, refn);
    cpFloat t    = 1.0f - cpfclamp01((cibn - crpn) / (cibn - cian));

    cpVect  point = cpvlerp(inca, incb, t);
    cpFloat pd    = cpvdot(cpvsub(point, refp), refn);

    if (t > 0.0f && pd <= 0.0f)
    {
        cpFloat rsum  = refr + incr;
        cpFloat alpha = (rsum > 0.0f ? incr * (1.0f - (rsum + pd) / rsum) : -0.5f * pd);

        cpContactInit(arr, cpvadd(point, cpvmult(refn, alpha)), n, pd, hash);
        return 1;
    }
    else
    {
        return 0;
    }
}

int ClipContacts(const Edge ref_, const Edge inc, const ClosestPoints points, const cpFloat nflip, cpContact* arr)
{
    cpVect inc_offs = cpvmult(inc.n, inc.r);
    cpVect ref_offs = cpvmult(ref_.n, ref_.r);

    cpVect inca = cpvadd(inc.a.p, inc_offs);
    cpVect incb = cpvadd(inc.b.p, inc_offs);

    cpVect closest_inca = cpClosetPointOnSegment(inc.a.p, ref_.a.p, ref_.b.p);
    cpVect closest_incb = cpClosetPointOnSegment(inc.b.p, ref_.a.p, ref_.b.p);

    cpVect  msa    = cpvmult(points.n, nflip * points.d);
    cpFloat cost_a = cpvdistsq(cpvsub(inc.a.p, closest_inca), msa);
    cpFloat cost_b = cpvdistsq(cpvsub(inc.b.p, closest_incb), msa);

/* #if DRAW_CLIP
    ChipmunkDebugDrawSegment(ref_.a.p, ref_.b.p, RGBAColor(1, 0, 0, 1));
    ChipmunkDebugDrawSegment(inc.a.p, inc.b.p, RGBAColor(0, 1, 0, 1));
    ChipmunkDebugDrawSegment(inca, incb, RGBAColor(0, 1, 0, 1));

    cpVect cref = cpvlerp(ref_.a.p, ref_.b.p, 0.5);
    ChipmunkDebugDrawSegment(cref, cpvadd(cref, cpvmult(ref_.n, 5.0)), RGBAColor(1, 0, 0, 1));

    cpVect cinc = cpvlerp(inc.a.p, inc.b.p, 0.5);
    ChipmunkDebugDrawSegment(cinc, cpvadd(cinc, cpvmult(inc.n, 5.0)), RGBAColor(1, 0, 0, 1));

    ChipmunkDebugDrawPoints(5.0, 2, (cpVect[]){ ref_.a.p, inc.a.p }, RGBAColor(1, 1, 0, 1));
    ChipmunkDebugDrawPoints(5.0, 2, (cpVect[]){ ref_.b.p, inc.b.p }, RGBAColor(0, 1, 1, 1));

    if (cost_a < cost_b)
    {
        ChipmunkDebugDrawSegment(closest_inca, inc.a.p, RGBAColor(1, 0, 1, 1));
    }
    else
    {
        ChipmunkDebugDrawSegment(closest_incb, inc.b.p, RGBAColor(1, 0, 1, 1));
    }
#endif */

    cpHashValue hash_iarb = CP_HASH_PAIR(inc.a.hash, ref_.b.hash);
    cpHashValue hash_ibra = CP_HASH_PAIR(inc.b.hash, ref_.a.hash);

    if (cost_a < cost_b)
    {
        cpVect refp = cpvadd(ref_.a.p, ref_offs);
        Contact1(points.d, closest_inca, inc.a.p, ref_.r, inc.r, points.n, hash_iarb, arr);
        return Contact2(refp, inca, incb, ref_.r, inc.r, ref_.n, points.n, hash_ibra, arr + 1) + 1;
    }
    else
    {
        cpVect refp = cpvadd(ref_.b.p, ref_offs);
        Contact1(points.d, closest_incb, inc.b.p, ref_.r, inc.r, points.n, hash_ibra, arr);
        return Contact2(refp, incb, inca, ref_.r, inc.r, ref_.n, points.n, hash_iarb, arr + 1) + 1;
    }
} +/

/* TODO : DELETE
int ContactPoints(const Edge e1, const Edge e2, const ClosestPoints points, cpContact* arr)
{
    cpFloat mindist = e1.r + e2.r;

    if (points.d <= mindist)
    {
        cpFloat pick = cpvdot(e1.n, points.n) + cpvdot(e2.n, points.n);

        if (
            (pick != 0.0f && pick > 0.0f) ||

            // If the edges are both perfectly aligned weird things happen.
            // This is *very* common at the start of a simulation.
            // Pick the longest edge as the reference to break the tie.
            (pick == 0.0f && (cpvdistsq(e1.a.p, e1.b.p) > cpvdistsq(e2.a.p, e2.b.p)))
            )
        {
            return ClipContacts(e1, e2, points, 1.0f, arr);
        }
        else
        {
            return ClipContacts(e2, e1, points, -1.0f, arr);
        }
    }
    else
    {
        return 0;
    }
}*/

// Given two support edges, find contact point pairs on their surfaces.
static void ContactPoints(const Edge e1, const Edge e2, const ClosestPoints points, cpCollisionInfo* info)
{
	cpFloat mindist = e1.r + e2.r;
	if(points.d <= mindist)
	{
/+ #ifdef DRAW_CLIP
	ChipmunkDebugDrawFatSegment(e1.a.p, e1.b.p, e1.r, RGBAColor(0, 1, 0, 1), LAColor(0, 0));
	ChipmunkDebugDrawFatSegment(e2.a.p, e2.b.p, e2.r, RGBAColor(1, 0, 0, 1), LAColor(0, 0));
#endif +/
		cpVect n = info.n = points.n;
		
		// Distances along the axis parallel to n
		cpFloat d_e1_a = cpvcross(e1.a.p, n);
		cpFloat d_e1_b = cpvcross(e1.b.p, n);
		cpFloat d_e2_a = cpvcross(e2.a.p, n);
		cpFloat d_e2_b = cpvcross(e2.b.p, n);
		
		// TODO + min isn't a complete fix.
		cpFloat e1_denom = 1.0f/(d_e1_b - d_e1_a + CPFLOAT_MIN);
		cpFloat e2_denom = 1.0f/(d_e2_b - d_e2_a + CPFLOAT_MIN);
		
		// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
		// Compare the projected points to the collision normal to see if the shapes overlap there.
		{
			cpVect p1 = cpvadd(cpvmult(n,  e1.r), cpvlerp(e1.a.p, e1.b.p, cpfclamp01((d_e2_b - d_e1_a)*e1_denom)));
			cpVect p2 = cpvadd(cpvmult(n, -e2.r), cpvlerp(e2.a.p, e2.b.p, cpfclamp01((d_e1_a - d_e2_a)*e2_denom)));
			cpFloat dist = cpvdot(cpvsub(p2, p1), n);
			if(dist <= 0.0f)
			{
				cpHashValue hash_1a2b = CP_HASH_PAIR(e1.a.hash, e2.b.hash);
				cpCollisionInfoPushContact(info, p1, p2, hash_1a2b);
			}
		}
		{
			cpVect p1 = cpvadd(cpvmult(n,  e1.r), cpvlerp(e1.a.p, e1.b.p, cpfclamp01((d_e2_a - d_e1_a)*e1_denom)));
			cpVect p2 = cpvadd(cpvmult(n, -e2.r), cpvlerp(e2.a.p, e2.b.p, cpfclamp01((d_e1_b - d_e2_a)*e2_denom)));
			cpFloat dist = cpvdot(cpvsub(p2, p1), n);
			if(dist <= 0.0f)
			{
				cpHashValue hash_1b2a = CP_HASH_PAIR(e1.b.hash, e2.a.hash);
				cpCollisionInfoPushContact(info, p1, p2, hash_1b2a);
			}
		}
	}
}

//MARK: Collision Functions

/* TODO : DELETE
alias CollisionFunc = int function(const cpShape* a, const cpShape* b, cpCollisionID* id, cpContact* arr);*/
alias CollisionFunc = void function(const cpShape* a, const cpShape* b, cpCollisionInfo* info);

/* TODO : DELETE
// Collide circle shapes.
int CircleToCircle(const cpCircleShape* c1, const cpCircleShape* c2, cpCollisionID* id, cpContact* arr)
{
    return CircleToCircleQuery(c1.tc, c2.tc, c1.r, c2.r, 0, arr);
}

int CircleToSegment(const cpCircleShape* circleShape, const cpSegmentShape* segmentShape, cpCollisionID* id, cpContact* con)
{
    cpVect seg_a  = segmentShape.ta;
    cpVect seg_b  = segmentShape.tb;
    cpVect center = circleShape.tc;

    cpVect  seg_delta = cpvsub(seg_b, seg_a);
    cpFloat closest_t = cpfclamp01(cpvdot(seg_delta, cpvsub(center, seg_a)) / cpvlengthsq(seg_delta));
    cpVect  closest   = cpvadd(seg_a, cpvmult(seg_delta, closest_t));

    if (CircleToCircleQuery(center, closest, circleShape.r, segmentShape.r, 0, con))
    {
        cpVect n = con[0].n;

        // Reject endcap collisions if tangents are provided.
        if (
            (closest_t != 0.0f || cpvdot(n, cpvrotate(segmentShape.a_tangent, segmentShape.shape.body_.rot)) >= 0.0) &&
            (closest_t != 1.0f || cpvdot(n, cpvrotate(segmentShape.b_tangent, segmentShape.shape.body_.rot)) >= 0.0)
            )
        {
            return 1;
        }
    }

    return 0;
}*/
// Collide circle shapes.
static void CircleToCircle(const cpCircleShape* c1, const cpCircleShape* c2, cpCollisionInfo* info)
{
	cpFloat mindist = c1.r + c2.r;
	cpVect delta = cpvsub(c2.tc, c1.tc);
	cpFloat distsq = cpvlengthsq(delta);
	
	if(distsq < mindist*mindist)
	{
		cpFloat dist = cpfsqrt(distsq);
		cpVect n = info.n = (dist ? cpvmult(delta, 1.0f/dist) : cpv(1.0f, 0.0f));
		cpCollisionInfoPushContact(info, cpvadd(c1.tc, cpvmult(n, c1.r)), cpvadd(c2.tc, cpvmult(n, -c2.r)), 0);
	}
}

static void CircleToSegment(const cpCircleShape* circle, const cpSegmentShape* segment, cpCollisionInfo* info)
{
	cpVect seg_a = segment.ta;
	cpVect seg_b = segment.tb;
	cpVect center = circle.tc;
	
	// Find the closest point on the segment to the circle.
	cpVect seg_delta = cpvsub(seg_b, seg_a);
	cpFloat closest_t = cpfclamp01(cpvdot(seg_delta, cpvsub(center, seg_a))/cpvlengthsq(seg_delta));
	cpVect closest = cpvadd(seg_a, cpvmult(seg_delta, closest_t));
	
	// Compare the radii of the two shapes to see if they are colliding.
	cpFloat mindist = circle.r + segment.r;
	cpVect delta = cpvsub(closest, center);
	cpFloat distsq = cpvlengthsq(delta);
	if(distsq < mindist*mindist)
	{
		cpFloat dist = cpfsqrt(distsq);
		// Handle coincident shapes as gracefully as possible.
		cpVect n = info.n = (dist ? cpvmult(delta, 1.0f/dist) : segment.tn);
		
		// Reject endcap collisions if tangents are provided.
		cpVect rot = cpBodyGetRotation(segment.shape.body_);
		if((closest_t != 0.0f || cpvdot(n, cpvrotate(segment.a_tangent, rot)) >= 0.0) &&
		   (closest_t != 1.0f || cpvdot(n, cpvrotate(segment.b_tangent, rot)) >= 0.0))
		{
			cpCollisionInfoPushContact(info, cpvadd(center, cpvmult(n, circle.r)), cpvadd(closest, cpvmult(n, -segment.r)), 0);
		}
	}
}

/* TODO : DELETE
int SegmentToSegment(const cpSegmentShape* seg1, const cpSegmentShape* seg2, cpCollisionID* id, cpContact* arr)*/
static void SegmentToSegment(const cpSegmentShape* seg1, const cpSegmentShape* seg2, cpCollisionInfo* info)
{
    SupportContext context = { cast(cpShape*)seg1, cast(cpShape*)seg2, safeCast!SupportPointFunc(&SegmentSupportPoint), safeCast!SupportPointFunc(&SegmentSupportPoint) };
    /* TODO : DELETE
	ClosestPoints  points  = GJK(&context, id);*/
	ClosestPoints points = GJK(&context, &info.id);


/+ #if DRAW_CLOSEST
#if PRINT_LOG
    //	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif

    ChipmunkDebugDrawDot(6.0, points.a, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawDot(6.0, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif +/

    cpVect n    = points.n;
    /* TODO : DELETE
	cpVect rot1 = seg1.shape.body_.rot;
    cpVect rot2 = seg2.shape.body_.rot;

    if (
        points.d <= (seg1.r + seg2.r) &&
        (*/
	cpVect rot1 = cpBodyGetRotation(seg1.shape.body_);
	cpVect rot2 = cpBodyGetRotation(seg2.shape.body_);
	
	// If the closest points are nearer than the sum of the radii...
	if((points.d <= (seg1.r + seg2.r)) && 
		// Reject endcap collisions if tangents are provided.
		((!cpveql(points.a, seg1.ta) || cpvdot(n, cpvrotate(seg1.a_tangent, rot1)) <= 0.0) &&
         (!cpveql(points.a, seg1.tb) || cpvdot(n, cpvrotate(seg1.b_tangent, rot1)) <= 0.0) &&
         (!cpveql(points.b, seg2.ta) || cpvdot(n, cpvrotate(seg2.a_tangent, rot2)) >= 0.0) &&
         (!cpveql(points.b, seg2.tb) || cpvdot(n, cpvrotate(seg2.b_tangent, rot2)) >= 0.0)))
    /* TODO : DELETE
	{
        return ContactPoints(SupportEdgeForSegment(seg1, n), SupportEdgeForSegment(seg2, cpvneg(n)), points, arr);
    }
    else
    {
        return 0;
    }*/
	{
		ContactPoints(SupportEdgeForSegment(seg1, n), SupportEdgeForSegment(seg2, cpvneg(n)), points, info);
	}
}


/* TODO : DELETE
int PolyToPoly(const cpPolyShape* poly1, const cpPolyShape* poly2, cpCollisionID* id, cpContact* arr)*/
static void PolyToPoly(const cpPolyShape* poly1, const cpPolyShape* poly2, cpCollisionInfo* info)
{
    SupportContext context = { cast(cpShape*)poly1, cast(cpShape*)poly2, safeCast!SupportPointFunc(&PolySupportPoint), safeCast!SupportPointFunc(&PolySupportPoint) };
    /* TODO : DELETE
	ClosestPoints  points  = GJK(&context, id);*/
	ClosestPoints points = GJK(&context, &info.id);
	

/+ #if DRAW_CLOSEST
#if PRINT_LOG
    //	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif

    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif +/


	/* TODO : DELETE
    if (points.d - poly1.r - poly2.r <= 0.0)
    {
        return ContactPoints(SupportEdgeForPoly(poly1, points.n), SupportEdgeForPoly(poly2, cpvneg(points.n)), points, arr);
    }
    else
    {
        return 0;
    }*/
	
	// If the closest points are nearer than the sum of the radii...
	if (points.d - poly1.r - poly2.r <= 0.0)
	{
		ContactPoints(SupportEdgeForPoly(poly1, points.n), SupportEdgeForPoly(poly2, cpvneg(points.n)), points, info);
	}
}

/* TODO : DELETE
int SegmentToPoly(const cpSegmentShape* seg, const cpPolyShape* poly, cpCollisionID* id, cpContact* arr)*/
static void SegmentToPoly(const cpSegmentShape* seg, const cpPolyShape* poly, cpCollisionInfo* info)
{
    SupportContext context = { cast(cpShape*)seg, cast(cpShape*)poly, safeCast!SupportPointFunc(&SegmentSupportPoint), safeCast!SupportPointFunc(&PolySupportPoint) };
    /* TODO : DELETE
	ClosestPoints  points  = GJK(&context, id);*/
	ClosestPoints points = GJK(&context, &info.id);	

/+ #if DRAW_CLOSEST
#if PRINT_LOG
    //	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif

    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif +/

    // Reject endcap collisions if tangents are provided.
    cpVect n   = points.n;
    /* TODO : DELETE
	cpVect rot = seg.shape.body_.rot;

    if (
        points.d - seg.r - poly.r <= 0.0 &&
        (
            (!cpveql(points.a, seg.ta) || cpvdot(n, cpvrotate(seg.a_tangent, rot)) <= 0.0) &&
            (!cpveql(points.a, seg.tb) || cpvdot(n, cpvrotate(seg.b_tangent, rot)) <= 0.0)
        )
        )
    {
        return ContactPoints(SupportEdgeForSegment(seg, n), SupportEdgeForPoly(poly, cpvneg(n)), points, arr);
    }
    else
    {
        return 0;
    }*/
	cpVect rot = cpBodyGetRotation(seg.shape.body_);
	
	if((points.d - seg.r - poly.r <= 0.0) && // If the closest points are nearer than the sum of the radii...
		// Reject endcap collisions if tangents are provided.
		((!cpveql(points.a, seg.ta) || cpvdot(n, cpvrotate(seg.a_tangent, rot)) <= 0.0) && 
		 (!cpveql(points.a, seg.tb) || cpvdot(n, cpvrotate(seg.b_tangent, rot)) <= 0.0)))
	{
		ContactPoints(SupportEdgeForSegment(seg, n), SupportEdgeForPoly(poly, cpvneg(n)), points, info);
	}
}

/* TODO : DELETE
// This one is less gross, but still gross.
// TODO: Comment me!
int CircleToPoly(const cpCircleShape* circle, const cpPolyShape* poly, cpCollisionID* id, cpContact* con)*/
static void CircleToPoly(const cpCircleShape* circle, const cpPolyShape* poly, cpCollisionInfo* info)
{
    SupportContext context = { cast(cpShape*)circle, cast(cpShape*)poly, safeCast!SupportPointFunc(&CircleSupportPoint), safeCast!SupportPointFunc(&PolySupportPoint) };
    /* TODO : DELETE
	ClosestPoints  points  = GJK(&context, id);*/
	ClosestPoints points = GJK(&context, &info.id);
	
/+ #if DRAW_CLOSEST
    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif +/

	/* TODO : DELETE
    cpFloat mindist = circle.r + poly.r;

    if (points.d - mindist <= 0.0)
    {
        cpVect p = cpvlerp(points.a, points.b, circle.r / (mindist));
        cpContactInit(con, p, points.n, points.d - mindist, 0);
        return 1;
    }
    else
    {
        return 0;
    }*/
	// If the closest points are nearer than the sum of the radii...
	if(points.d <= circle.r + poly.r)
	{
		cpVect n = info.n = points.n;
		cpCollisionInfoPushContact(info, cpvadd(points.a, cpvmult(n, circle.r)), cpvadd(points.b, cpvmult(n, poly.r)), 0);
	}
}

static void CollisionError(const cpShape* circle, const cpShape* poly, cpCollisionInfo* info)
{
	cpAssertHard(cpFalse, "Internal Error: Shape types are not sorted.");
}


__gshared CollisionFunc[9] BuiltinCollisionFuncs;
__gshared CollisionFunc* CollisionFuncs;
/* TODO DELETE
__gshared CollisionFunc[9] segmentCollisions; */

void _initModuleCtor_cpCollision()
{
    BuiltinCollisionFuncs = [

		safeCast!CollisionFunc(&CircleToCircle),
	
		safeCast!CollisionFunc(&CollisionError),
		safeCast!CollisionFunc(&CollisionError),
	
		safeCast!CollisionFunc(&CircleToSegment),
		safeCast!CollisionFunc(&SegmentToSegment),
	
		safeCast!CollisionFunc(&CollisionError),
	
		safeCast!CollisionFunc(&CircleToPoly),
		safeCast!CollisionFunc(&SegmentToPoly),
		safeCast!CollisionFunc(&PolyToPoly),
    ];

    CollisionFuncs = BuiltinCollisionFuncs.ptr;

	/* TODO : DELETE
    segmentCollisions = [
        safeCast!CollisionFunc(&CircleToCircle),
        null,
        null,
        safeCast!CollisionFunc(&CircleToSegment),
        safeCast!CollisionFunc(&SegmentToSegment),
        null,
        safeCast!CollisionFunc(&CircleToPoly),
        safeCast!CollisionFunc(&SegmentToPoly),
        safeCast!CollisionFunc(&PolyToPoly),
    ];*/
}

/* TODO : DELETE
void cpEnableSegmentToSegmentCollisions()
{
    colfuncs = segmentCollisions.ptr;
}

int cpCollideShapes(const cpShape* a, const cpShape* b, cpCollisionID* id, cpContact* arr)
{
    // Their shape types must be in order.
    cpAssertSoft(a.klass.type <= b.klass.type, "Internal Error: Collision shapes passed to cpCollideShapes() are not sorted.");

    CollisionFunc cfunc = colfuncs[a.klass.type + b.klass.type * CP_NUM_SHAPES];

    int numContacts = (cfunc ? cfunc(a, b, id, arr) : 0);
    cpAssertSoft(numContacts <= CP_MAX_CONTACTS_PER_ARBITER, "Internal error: Too many contact points returned.");

    return numContacts;
}*/

/// Note: This function returns contact points with r1/r2 in absolute coordinates, not body relative.
cpCollisionInfo cpCollide(cpShape* a, cpShape* b, cpCollisionID id, cpContact* contacts)
{
	cpCollisionInfo info = {a, b, id, cpvzero, 0, contacts};
	
	// Make sure the shape types are in order.
	if(a.klass.type > b.klass.type)
	{
		info.a = b;
		info.b = a;
	}
	
	CollisionFuncs[info.a.klass.type + info.b.klass.type*cpShapeType.CP_NUM_SHAPES](info.a, info.b, &info);
	
//	if(0){
//		for(int i=0; i<info.count; i++){
//			cpVect r1 = info.arr[i].r1;
//			cpVect r2 = info.arr[i].r2;
//			cpVect mid = cpvlerp(r1, r2, 0.5f);
//			
//			ChipmunkDebugDrawSegment(r1, mid, RGBAColor(1, 0, 0, 1));
//			ChipmunkDebugDrawSegment(r2, mid, RGBAColor(0, 0, 1, 1));
//		}
//	}
	
	return info;
}
