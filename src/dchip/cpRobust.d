module dchip.cpRobust;

import dchip.cpVect;
import dchip.chipmunk_types;

// This is a private header for functions (currently just one) that need strict floating point results.
// It was easier to put this in it's own file than to fiddle with 4 different compiler specific pragmas or attributes.
// "Fast math" should be disabled here.

// Check if c is to the left of segment (a, b).
cpBool cpCheckPointGreater(const cpVect a, const cpVect b, const cpVect c)
{
	return (b.y - a.y)*(a.x + b.x - 2*c.x) > (b.x - a.x)*(a.y + b.y - 2*c.y);
}

// Check if p is behind one of v0 or v1 on axis n.
cpBool cpCheckAxis(cpVect v0, cpVect v1, cpVect p, cpVect n)
{
	return cpvdot(p, n) <= cpfmax(cpvdot(v0, n), cpvdot(v1, n));
}
