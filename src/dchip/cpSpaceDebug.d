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
module dchip.cpSpaceDebug;

import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;

import dchip.cpShape;
import dchip.cpSpace;


static immutable cpVect[] spring_verts = 
[
	cpVect(0.00f, 0.0f),
	cpVect(0.20f, 0.0f),
	cpVect(0.25f, 3.0f),
	cpVect(0.30f,-6.0f),
	cpVect(0.35f, 6.0f),
	cpVect(0.40f,-6.0f),
	cpVect(0.45f, 6.0f),
	cpVect(0.50f,-6.0f),
	cpVect(0.55f, 6.0f),
	cpVect(0.60f,-6.0f),
	cpVect(0.65f, 6.0f),
	cpVect(0.70f,-3.0f),
	cpVect(0.75f, 6.0f),
	cpVect(0.80f, 0.0f),
	cpVect(1.00f, 0.0f)
];
static immutable int spring_count = spring_verts.sizeof / cpVect.sizeof;


/* TODO : UNCOMMENT AFTER ACTUALIZE cpSpace.d and struct cpSpaceDebugDrawOptions

static void cpSpaceDebugDrawShape(cpShape* shape, cpSpaceDebugDrawOptions* options)
{
	cpBody* body_ = shape.body_;
	cpDataPointer data = options.data;
	
	cpSpaceDebugColor outline_color = options.shapeOutlineColor;
	cpSpaceDebugColor fill_color = options.colorForShape(shape, data);
	
	switch(shape.klass.type)
	{
		case CP_CIRCLE_SHAPE: 
		{
			cpCircleShape* circle = cast(cpCircleShape*)shape;
			options.drawCircle(circle.tc, body_.a, circle.r, outline_color, fill_color, data);
			break;
		}
		
		case CP_SEGMENT_SHAPE: 
		{
			cpSegmentShape* seg = cast(cpSegmentShape*)shape;
			options.drawFatSegment(seg.ta, seg.tb, seg.r, outline_color, fill_color, data);
			break;
		}
		
		case CP_POLY_SHAPE: 
		{
			cpPolyShape* poly = cast(cpPolyShape*)shape;
			
			int count = poly.count;
			cpSplittingPlane* planes = poly.planes;
			cpVect* verts = cast(cpVect*)alloca(count*cpVect.sizeof);
			
			for (int i = 0; i < count; i++) 
				verts[i] = planes[i].v0;
			options.drawPolygon(count, verts, poly.r, outline_color, fill_color, data);
			break;
		}
		
		default: break;
	}
}



static void cpSpaceDebugDrawConstraint(cpConstraint* constraint, cpSpaceDebugDrawOptions* options)
{
	cpDataPointer data = options.data;
	cpSpaceDebugColor color = options.constraintColor;
	
	cpBody* body_a = constraint.a;
	cpBody* body_b = constraint.b;

	if(cpConstraintIsPinJoint(constraint))
	{
		cpPinJoint* joint = cast(cpPinJoint*)constraint;
		
		cpVect a = cpTransformPoint(body_a.transform, joint.anchorA);
		cpVect b = cpTransformPoint(body_b.transform, joint.anchorB);
		
		options.drawDot(5, a, color, data);
		options.drawDot(5, b, color, data);
		options.drawSegment(a, b, color, data);
	} 
	else 
		if(cpConstraintIsSlideJoint(constraint))
		{
			cpSlideJoint* joint = cast(cpSlideJoint*)constraint;
	
			cpVect a = cpTransformPoint(body_a.transform, joint.anchorA);
			cpVect b = cpTransformPoint(body_b.transform, joint.anchorB);
		
			options.drawDot(5, a, color, data);
			options.drawDot(5, b, color, data);
			options.drawSegment(a, b, color, data);
		}
		else 
			if(cpConstraintIsPivotJoint(constraint))
			{
				cpPivotJoint* joint = cast(cpPivotJoint*)constraint;
	
				cpVect a = cpTransformPoint(body_a.transform, joint.anchorA);
				cpVect b = cpTransformPoint(body_b.transform, joint.anchorB);

				options.drawDot(5, a, color, data);
				options.drawDot(5, b, color, data);
			} 
			else 
				if(cpConstraintIsGrooveJoint(constraint))
				{
					cpGrooveJoint* joint = cast(cpGrooveJoint*)constraint;
	
					cpVect a = cpTransformPoint(body_a.transform, joint.grv_a);
					cpVect b = cpTransformPoint(body_a.transform, joint.grv_b);
					cpVect c = cpTransformPoint(body_b.transform, joint.anchorB);
		
					options.drawDot(5, c, color, data);
					options.drawSegment(a, b, color, data);
				}
				else 
					if(cpConstraintIsDampedSpring(constraint))
					{
						cpDampedSpring* spring = cast(cpDampedSpring*)constraint;
						cpDataPointer data = options.data;
						cpSpaceDebugColor color = options.constraintColor;
		
						cpVect a = cpTransformPoint(body_a.transform, spring.anchorA);
						cpVect b = cpTransformPoint(body_b.transform, spring.anchorB);
		
						options.drawDot(5, a, color, data);
						options.drawDot(5, b, color, data);

						cpVect delta = cpvsub(b, a);
						cpFloat cos = delta.x;
						cpFloat sin = delta.y;
						cpFloat s = 1.0f/cpvlength(delta);
		
						cpVect r1 = cpv(cos, -sin*s);
						cpVect r2 = cpv(sin,  cos*s);
		
						cpVect* verts = cast(cpVect*)alloca(spring_count*sizeof(cpVect));
						for(int i=0; i<spring_count; i++)
						{
							cpVect v = spring_verts[i];
							verts[i] = cpv(cpvdot(v, r1) + a.x, cpvdot(v, r2) + a.y);
						}
		
						for(int i=0; i<spring_count-1; i++)
						{
							options.drawSegment(verts[i], verts[i + 1], color, data);
						}
					}
}

void cpSpaceDebugDraw(cpSpace* space, cpSpaceDebugDrawOptions* options)
{
	if(options.flags & CP_SPACE_DEBUG_DRAW_SHAPES)
	{
		cpSpaceEachShape(space, cast(cpSpaceShapeIteratorFunc)cpSpaceDebugDrawShape, options);
	}
	
	if(options.flags & CP_SPACE_DEBUG_DRAW_CONSTRAINTS)
	{
		cpSpaceEachConstraint(space, cast(cpSpaceConstraintIteratorFunc)cpSpaceDebugDrawConstraint, options);
	}
	
	if(options.flags & CP_SPACE_DEBUG_DRAW_COLLISION_POINTS)
	{
		cpArray* arbiters = space.arbiters;
		cpSpaceDebugColor color = options.collisionPointColor;
		cpSpaceDebugDrawSegmentImpl draw_seg = options.drawSegment;
		cpDataPointer data = options.data;
		
		for(int i=0; i<arbiters.num; i++)
		{
			cpArbiter* arb = cast(cpArbiter*)arbiters.arr[i];
			cpVect n = arb.n;
			
			for(int j=0; j<arb.count; j++){
				cpVect p1 = cpvadd(arb.body_a.p, arb.contacts[j].r1);
				cpVect p2 = cpvadd(arb.body_b.p, arb.contacts[j].r2);
				
				cpFloat d = 2.0f;
				cpVect a = cpvadd(p1, cpvmult(n, -d));
				cpVect b = cpvadd(p2, cpvmult(n,  d));
				draw_seg(a, b, color, data);
			}
		}
	}
}*/
