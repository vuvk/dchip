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
module dchip.cpSpaceComponent;

import core.stdc.string;

import dchip.chipmunk;
import dchip.chipmunk_types;
import dchip.chipmunk_private;
import dchip.chipmunk_structs;
import dchip.cpArray;
import dchip.cpArbiter;
import dchip.cpBody;
import dchip.cpConstraint;
import dchip.cpHashSet;
import dchip.cpShape;
import dchip.cpSpace;
import dchip.cpSpaceStep;
import dchip.cpSpaceQuery;
import dchip.cpSpatialIndex;
import dchip.cpVect;
import dchip.util;

//MARK: Sleeping Functions

void cpSpaceActivateBody(cpSpace* space, cpBody* body_)
{
	/* TODO : DELETE
    cpAssertHard(!cpBodyIsRogue(body_), "Internal error: Attempting to activate a rogue body_.");*/
	cpAssertHard(cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_DYNAMIC, "Internal error: Attempting to activate a non-dynamic body.");
	
    if (space.locked)
    {
        // cpSpaceActivateBody() is called again once the space is unlocked
        if (!cpArrayContains(space.rousedBodies, body_))
            cpArrayPush(space.rousedBodies, body_);
    }
    else
    {
		/* TODO : DELETE
        cpAssertSoft(body_.node.root == null && body_.node.next == null, "Internal error: Activating body_ non-null node pointers.");
        cpArrayPush(space.bodies, body_);*/
		cpAssertSoft(body_.sleeping.root == null && body_.sleeping.next == null, "Internal error: Activating body non-null node pointers.");
		cpArrayPush(space.dynamicBodies, body_);


        mixin(CP_BODY_FOREACH_SHAPE!("body_", "shape", q{
            cpSpatialIndexRemove(space.staticShapes, shape, shape.hashid);
            cpSpatialIndexInsert(space.dynamicShapes, shape, shape.hashid);		
        }));

        mixin(CP_BODY_FOREACH_ARBITER!("body_", "arb", q{
            cpBody* bodyA = arb.body_a;

            // Arbiters are shared between two bodies that are always woken up together.
            // You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
            // The edge case is when static bodies are involved as the static bodies never actually sleep.
            // If the static body_ is bodyB then all is good. If the static body_ is bodyA, that can easily be checked.
            /* TODO : DELETE			
			if (body_ == bodyA || cpBodyIsStatic(bodyA))
            {
                int numContacts     = arb.numContacts;
                cpContact* contacts = arb.contacts;

                // Restore contact values back to the space's contact buffer memory
                arb.contacts = cpContactBufferGetArray(space);
                memcpy(arb.contacts, contacts, numContacts*  cpContact.sizeof);
                cpSpacePushContacts(space, numContacts);

                // Reinsert the arbiter into the arbiter cache
                cpShape* a = arb.a;
                cpShape* b = arb.b;
                cpShape*[2] shape_pair;
                shape_pair[0] = a;
                shape_pair[1] = b;
                cpHashValue arbHashID = CP_HASH_PAIR(cast(cpHashValue)a, cast(cpHashValue)b);
                
				// TODO : DELETE
				//cpHashSetInsert(space.cachedArbiters, arbHashID, shape_pair.ptr, arb, null);
				cpHashSetInsert(space.cachedArbiters, arbHashID, shape_pair.ptr, null, arb);

                // Update the arbiter's state
                arb.stamp   = space.stamp;
                arb.handler = cpSpaceLookupHandler(space, a.collision_type, b.collision_type);
                cpArrayPush(space.arbiters, arb);

                cpfree(contacts);
            }*/
			if(body_ == bodyA || cpBodyGetType(bodyA) == cpBodyType.CP_BODY_TYPE_STATIC)
			{
				int numContacts = arb.count;
				cpContact* contacts = arb.contacts;
				
				// Restore contact values back to the space's contact buffer memory
				arb.contacts = cpContactBufferGetArray(space);
				memcpy(arb.contacts, contacts, numContacts*cpContact.sizeof);
				cpSpacePushContacts(space, numContacts);
				
				// Reinsert the arbiter into the arbiter cache
				cpShape* a = arb.a;
				cpShape* b = arb.b;
				cpShape*[] shape_pair = [a, b];
				cpHashValue arbHashID = CP_HASH_PAIR(cast(cpHashValue)a, cast(cpHashValue)b);
				cpHashSetInsert(space.cachedArbiters, arbHashID, cast(cpDataPointer)shape_pair, null, arb);
				
				// Update the arbiter's state
				arb.stamp = space.stamp;
				cpArrayPush(space.arbiters, arb);
				
				cpfree(contacts);
			}
        }));

        mixin(CP_BODY_FOREACH_CONSTRAINT!("body_", "constraint", q{
            cpBody* bodyA = constraint.a;

			/* TODO : DELETE
            if (body_ == bodyA || cpBodyIsStatic(bodyA))
                cpArrayPush(space.constraints, constraint);*/
			if(body_ == bodyA || cpBodyGetType(bodyA) == cpBodyType.CP_BODY_TYPE_STATIC) 
				cpArrayPush(space.constraints, constraint);
        }));
    }
}

void cpSpaceDeactivateBody(cpSpace* space, cpBody* body_)
{
	/* TODO : DELETE
    cpAssertHard(!cpBodyIsRogue(body_), "Internal error: Attempting to deactivate a rouge body_.");
	cpArrayDeleteObj(space.bodies, body_);*/
	
	cpAssertHard(cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_DYNAMIC, "Internal error: Attempting to deactivate a non-dynamic body.");
	cpArrayDeleteObj(space.dynamicBodies, body_);	

    mixin(CP_BODY_FOREACH_SHAPE!("body_", "shape", q{	
        cpSpatialIndexRemove(space.dynamicShapes, shape, shape.hashid);
        cpSpatialIndexInsert(space.staticShapes, shape, shape.hashid);
    }));

    mixin(CP_BODY_FOREACH_ARBITER!("body_", "arb", q{
        cpBody* bodyA = arb.body_a;

		/* TODO : DELETE
        if (body_ == bodyA || cpBodyIsStatic(bodyA))
        {
            cpSpaceUncacheArbiter(space, arb);

            // Save contact values to a new block of memory so they won't time out
            size_t bytes        = arb.numContacts*  cpContact.sizeof;
            cpContact* contacts = cast(cpContact*)cpcalloc(1, bytes);
            memcpy(contacts, arb.contacts, bytes);
            arb.contacts = contacts;
        }*/
		if(body_ == bodyA || cpBodyGetType(bodyA) == cpBodyType.CP_BODY_TYPE_STATIC){
			cpSpaceUncacheArbiter(space, arb);
			
			// Save contact values to a new block of memory so they won't time out
			size_t bytes = arb.count*cpContact.sizeof;
			cpContact* contacts = cast(cpContact*)cpcalloc(1, bytes);
			memcpy(contacts, arb.contacts, bytes);
			arb.contacts = contacts;
		}
    }));

    mixin(CP_BODY_FOREACH_CONSTRAINT!("body_", "constraint", q{
        cpBody* bodyA = constraint.a;

		/* TODO : DELETE
        if (body_ == bodyA || cpBodyIsStatic(bodyA))
            cpArrayDeleteObj(space.constraints, constraint);*/
		if(body_ == bodyA || cpBodyGetType(bodyA) == cpBodyType.CP_BODY_TYPE_STATIC) 
			cpArrayDeleteObj(space.constraints, constraint);	
    }));
}

cpBody* ComponentRoot(cpBody* body_)
{
	/* TODO : DELETE
    return (body_ ? body_.node.root : null);*/
    return (body_ ? body_.sleeping.root : null);
}

/* TODO : DELETE
void ComponentActivate(cpBody* root)
{
    if (!root || !cpBodyIsSleeping(root))
        return;
    cpAssertHard(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body_.");

    cpSpace* space = root.space;
    cpBody*  body_  = root;

    while (body_)
    {
        cpBody* next = body_.node.next;

        body_.node.idleTime = 0.0f;
        body_.node.root     = null;
        body_.node.next     = null;
        cpSpaceActivateBody(space, body_);

        body_ = next;
    }

    cpArrayDeleteObj(space.sleepingComponents, root);
}*/

/// Wake up a sleeping or idle body_.
void cpBodyActivate(cpBody* body_)
{
	if(body_ != null && cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_DYNAMIC)
	{
		body_.sleeping.idleTime = 0.0f;
		
		cpBody* root = ComponentRoot(body_);
		if(root && cpBodyIsSleeping(root))
		{
			// TODO should cpBodyIsSleeping(root) be an assertion?
			cpAssertSoft(cpBodyGetType(root) == cpBodyType.CP_BODY_TYPE_DYNAMIC, "Internal Error: Non-dynamic body component root detected.");
			
			cpSpace* space = root.space;
			cpBody* bdy = root;
			while(bdy)
			{
				cpBody* next = bdy.sleeping.next;
				
				bdy.sleeping.idleTime = 0.0f;
				bdy.sleeping.root = null;
				bdy.sleeping.next = null;
				cpSpaceActivateBody(space, body_);
				
				bdy = next;
			}
			
			cpArrayDeleteObj(space.sleepingComponents, root);
		}
		
		mixin(CP_BODY_FOREACH_ARBITER!("body_", "arb", q{
			// Reset the idle timer of things the body is touching as well.
			// That way things don't get left hanging in the air.
			cpBody* other = (arb.body_a == body_ ? arb.body_b : arb.body_a);
			if(cpBodyGetType(other) != cpBodyType.CP_BODY_TYPE_STATIC) 
				other.sleeping.idleTime = 0.0f;
		}));
	}
}

/// Wake up any sleeping or idle bodies touching a static body_.
void cpBodyActivateStatic(cpBody* body_, cpShape* filter)
{
    cpAssertHard(cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_STATIC, "cpBodyActivateStatic() called on a non-static body.");
	
    mixin(CP_BODY_FOREACH_ARBITER!("body_", "arb", q{
        if (!filter || filter == arb.a || filter == arb.b)
        {
            cpBodyActivate(arb.body_a == body_ ? arb.body_b : arb.body_a);
        }
    }));

	// TODO: should also activate joints?
}

static void cpBodyPushArbiter(cpBody* body_, cpArbiter* arb)
{
    cpAssertSoft(cpArbiterThreadForBody(arb, body_).next == null, "Internal Error: Dangling contact graph pointers detected. (A)");
    cpAssertSoft(cpArbiterThreadForBody(arb, body_).prev == null, "Internal Error: Dangling contact graph pointers detected. (B)");

    cpArbiter* next = body_.arbiterList;
    cpAssertSoft(next == null || cpArbiterThreadForBody(next, body_).prev == null, "Internal Error: Dangling contact graph pointers detected. (C)");
    cpArbiterThreadForBody(arb, body_).next = next;

    if (next)
        cpArbiterThreadForBody(next, body_).prev = arb;
    body_.arbiterList = arb;
}

static void ComponentAdd(cpBody* root, cpBody* body_)
{
	body_.sleeping.root = root;

	if(body_ != root)
	{
		body_.sleeping.next = root.sleeping.next;
		root.sleeping.next = body_;
	}
}

static void FloodFillComponent(cpBody* root, cpBody* body_)
{
	/* TODO : DELETE
    // Rogue bodies cannot be put to sleep and prevent bodies they are touching from sleepining anyway.
    // Static bodies (which are a type of rogue body_) are effectively sleeping all the time.
    if (!cpBodyIsRogue(body_))*/
	
	// Kinematic bodies cannot be put to sleep and prevent bodies they are touching from sleeping.
	// Static bodies are effectively sleeping all the time.
	if(cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_DYNAMIC)
	{
        cpBody* other_root = ComponentRoot(body_);

        if (other_root == null)
        {
            ComponentAdd(root, body_);
            mixin(CP_BODY_FOREACH_ARBITER!("body_", "arb", "FloodFillComponent(root, (body_ == arb.body_a ? arb.body_b : arb.body_a));"));
            mixin(CP_BODY_FOREACH_CONSTRAINT!("body_", "constraint", "FloodFillComponent(root, (body_ == constraint.a ? constraint.b : constraint.a));"));
        }
        else
        {
            cpAssertSoft(other_root == root, "Internal Error: Inconsistency dectected in the contact graph.");
        }
    }
}

static cpBool ComponentActive(cpBody* root, cpFloat threshold)
{
    mixin(CP_BODY_FOREACH_COMPONENT!("root", "body_", q{
        if (body_.sleeping.idleTime < threshold)
            return cpTrue;
    }));

    return cpFalse;
}

void cpSpaceProcessComponents(cpSpace* space, cpFloat dt)
{
    cpBool sleep    = (space.sleepTimeThreshold != INFINITY);
    cpArray* bodies = space.dynamicBodies;	

    version (CHIP_ENABLE_WARNINGS)
    {
        for (int i = 0; i < bodies.num; i++)
        {
            cpBody* body_ = cast(cpBody*)bodies.arr[i];
			/* TODO : DELETE
            cpAssertSoft(body_.node.next == null, "Internal Error: Dangling next pointer detected in contact graph.");
            cpAssertSoft(body_.node.root == null, "Internal Error: Dangling root pointer detected in contact graph.");*/
			cpAssertSoft(body_.sleeping.next == null, "Internal Error: Dangling next pointer detected in contact graph.");
			cpAssertSoft(body_.sleeping.root == null, "Internal Error: Dangling root pointer detected in contact graph.");
		}
    }

    // Calculate the kinetic energy of all the bodies.
    if (sleep)
    {
        cpFloat dv   = space.idleSpeedThreshold;
        cpFloat dvsq = (dv ? dv*  dv : cpvlengthsq(space.gravity)*  dt*  dt);

        // update idling and reset component nodes
        for (int i = 0; i < bodies.num; i++)
        {
            cpBody* body_ = cast(cpBody*)bodies.arr[i];
			/* TODO : DELETE
            // Need to deal with infinite mass objects
            cpFloat keThreshold = (dvsq ? body_.m*  dvsq : 0.0f);
            body_.node.idleTime = (cpBodyKineticEnergy(body_) > keThreshold ? 0.0f : body_.node.idleTime + dt);*/
			
			// TODO should make a separate array for kinematic bodies.
			if(cpBodyGetType(body_) != cpBodyType.CP_BODY_TYPE_DYNAMIC) continue;
			
			// Need to deal with infinite mass objects
			cpFloat keThreshold = (dvsq ? body_.m*dvsq : 0.0f);
			body_.sleeping.idleTime = (cpBodyKineticEnergy(body_) > keThreshold ? 0.0f : body_.sleeping.idleTime + dt);
        }
    }

    // Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
    cpArray* arbiters = space.arbiters;

    for (int i = 0, count = arbiters.num; i < count; i++)
    {
        cpArbiter* arb = cast(cpArbiter*)arbiters.arr[i];
        cpBody* a      = arb.body_a;
        cpBody* b      = arb.body_b;

        if (sleep)
        {	
			/* TODO : DELETE
            if ((cpBodyIsRogue(b) && !cpBodyIsStatic(b)) || cpBodyIsSleeping(a))
                cpBodyActivate(a);

            if ((cpBodyIsRogue(a) && !cpBodyIsStatic(a)) || cpBodyIsSleeping(b))
                cpBodyActivate(b);*/
			
			// TODO checking cpBodyIsSleepin() redundant?
			if(cpBodyGetType(b) == cpBodyType.CP_BODY_TYPE_KINEMATIC || cpBodyIsSleeping(a)) 
				cpBodyActivate(a);
			if(cpBodyGetType(a) == cpBodyType.CP_BODY_TYPE_KINEMATIC || cpBodyIsSleeping(b)) 
				cpBodyActivate(b);		
        }

        cpBodyPushArbiter(a, arb);
        cpBodyPushArbiter(b, arb);
    }

    if (sleep)
    {
        // Bodies should be held active if connected by a joint to a kinematic.
		cpArray* constraints = space.constraints;

        for (int i = 0; i < constraints.num; i++)
        {
            cpConstraint* constraint = cast(cpConstraint*)constraints.arr[i];
            cpBody* a = constraint.a;
            cpBody* b = constraint.b;

			/* TODO : DELETE
            if (cpBodyIsRogue(b) && !cpBodyIsStatic(b))
                cpBodyActivate(a);
            if (cpBodyIsRogue(a) && !cpBodyIsStatic(a))
                cpBodyActivate(b);*/
				
			if(cpBodyGetType(b) == cpBodyType.CP_BODY_TYPE_KINEMATIC) 
				cpBodyActivate(a);
			if(cpBodyGetType(a) == cpBodyType.CP_BODY_TYPE_KINEMATIC) 
				cpBodyActivate(b);		
        }

        // Generate components and deactivate sleeping ones
        for (int i = 0; i < bodies.num; )
        {
            cpBody* body_ = cast(cpBody*)bodies.arr[i];

            if (ComponentRoot(body_) == null)
            {
                // Body not in a component yet. Perform a DFS to flood fill mark
                // the component in the contact graph using this body_ as the root.
                FloodFillComponent(body_, body_);

                // Check if the component should be put to sleep.
                if (!ComponentActive(body_, space.sleepTimeThreshold))
                {
                    cpArrayPush(space.sleepingComponents, body_);
                    mixin(CP_BODY_FOREACH_COMPONENT!("body_", "other", "cpSpaceDeactivateBody(space, other);"));

                    // cpSpaceDeactivateBody() removed the current body_ from the list.
                    // Skip incrementing the index counter.
                    continue;
                }
            }

            i++;

            // Only sleeping bodies retain their component node pointers.
            body_.sleeping.root = null;
            body_.sleeping.next = null;
        }
    }
}

/// Force a body_ to fall asleep immediately.
void cpBodySleep(cpBody* body_)
{
    cpBodySleepWithGroup(body_, null);
}

/// Force a body_ to fall asleep immediately along with other bodies in a group.
void cpBodySleepWithGroup(cpBody* body_, cpBody* group){
	cpAssertHard(cpBodyGetType(body_) == cpBodyType.CP_BODY_TYPE_DYNAMIC, "Non-dynamic bodies cannot be put to sleep.");
	
	cpSpace* space = body_.space;
	cpAssertHard(!cpSpaceIsLocked(space), "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
	cpAssertHard(cpSpaceGetSleepTimeThreshold(space) < INFINITY, "Sleeping is not enabled on the space. You cannot sleep a body_ without setting a sleep time threshold on the space.");
	cpAssertHard(group == null || cpBodyIsSleeping(group), "Cannot use a non-sleeping body_ as a group identifier.");
	
	if(cpBodyIsSleeping(body_))
	{
		cpAssertHard(ComponentRoot(body_) == ComponentRoot(group), "The body_ is already sleeping and it's group cannot be reassigned.");
		return;
	}
	
	mixin(CP_BODY_FOREACH_SHAPE!("body_", "shape", "cpShapeCacheBB(shape);"));
	cpSpaceDeactivateBody(space, body_);
	
	if(group)
	{
		cpBody* root = ComponentRoot(group);
		
		body_.sleeping.root = root;
		body_.sleeping.next = root.sleeping.next;
		body_.sleeping.idleTime = 0.0f;
		
		root.sleeping.next = body_;
	}
	else
	{
		body_.sleeping.root = body_;
		body_.sleeping.next = null;
		body_.sleeping.idleTime = 0.0f;
		
		cpArrayPush(space.sleepingComponents, body_);
	}
	
	cpArrayDeleteObj(space.dynamicBodies, body_);
}

/* TODO : DELETE
void activateTouchingHelper(cpShape* shape, cpContactPointSet* points, cpShape* other)
{
    cpBodyActivate(shape.body_);
}

void cpSpaceActivateShapesTouchingShape(cpSpace* space, cpShape* shape)
{
    if (space.sleepTimeThreshold != INFINITY)
    {
        cpSpaceShapeQuery(space, shape, safeCast!cpSpaceShapeQueryFunc(&activateTouchingHelper), shape);
    }
}*/
