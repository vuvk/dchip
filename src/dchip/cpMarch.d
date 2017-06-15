// Copyright 2013 Howling Moon Software. All rights reserved.
// See http://chipmunk2d.net/legal.php for more information.
module dchip.cpMarch;

import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_structs;
import dchip.chipmunk_types;
import dchip.cpBB;
import dchip.cpVect;
import dchip.cpArray;
import dchip.cpHashSet;
import dchip.cpSpatialIndex;
import dchip.util;



/// Function type used as a callback from the marching squares algorithm to sample an image function.
/// It passes you the point to sample and your context pointer, and you return the density.
// TODO : DELETE
alias cpMarchSampleFunc = cpFloat function(cpVect point, void* data);

/// Function type used as a callback from the marching squares algorithm to output a line segment.
/// It passes you the two endpoints and your context pointer.
// TODO : DELETE
alias cpMarchSegmentFunc = void function(cpVect v0, cpVect v1, void* data);


// The looping and sample caching code is shared between cpMarchHard() and cpMarchSoft().
static void cpMarchCells (cpBB bb, ulong x_samples, ulong y_samples, cpFloat t,
  						  cpMarchSegmentFunc segment, void* segment_data,
						  cpMarchSampleFunc sample, void* sample_data,
						  cpMarchCellFunc cell)
{
	cpFloat x_denom = 1.0/cast(cpFloat)(x_samples - 1);
	cpFloat y_denom = 1.0/cast(cpFloat)(y_samples - 1);
	
	// TODO range assertions and short circuit for 0 sized windows.
	
	// Keep a copy of the previous row to avoid double lookups.
	cpFloat* buffer = cast(cpFloat*)cpcalloc(cast(uint)x_samples, cpFloat.sizeof);
	for (uint i=0; i<x_samples; i++) 
		buffer[i] = sample(cpv(cpflerp(bb.l, bb.r, i*x_denom), bb.b), sample_data);
	
	for (uint j=0; j<y_samples-1; j++)
	{
		cpFloat y0 = cpflerp(bb.b, bb.t, (j+0)*y_denom);
		cpFloat y1 = cpflerp(bb.b, bb.t, (j+1)*y_denom);
		
		cpFloat a, b = buffer[0];
		cpFloat c, d = sample(cpv(bb.l, y1), sample_data);
		buffer[0] = d;
		
		for (uint i=0; i<x_samples-1; i++)
		{
			cpFloat x0 = cpflerp(bb.l, bb.r, (i+0)*x_denom);
			cpFloat x1 = cpflerp(bb.l, bb.r, (i+1)*x_denom);
			
			a = b, b = buffer[i + 1];
			c = d, d = sample(cpv(x1, y1), sample_data);
			buffer[i + 1] = d;
			
			cell(t, a, b, c, d, x0, x1, y0, y1, segment, segment_data);
		}
	}
	
	cpfree(buffer);
}

alias cpMarchCellFunc = void function (cpFloat t, cpFloat a, cpFloat b, cpFloat c, cpFloat d,
									   cpFloat x0, cpFloat x1, cpFloat y0, cpFloat y1,
									   cpMarchSegmentFunc segment, void* segment_data);


// TODO should flip this around eventually.
static void seg(cpVect v0, cpVect v1, cpMarchSegmentFunc f, void* data)
{
	if(!cpveql(v0, v1)) f(v1, v0, data);
}

// Lerps between two positions based on their sample values.
static cpFloat midlerp(cpFloat x0, cpFloat x1, cpFloat s0, cpFloat s1, cpFloat t)
{
	return cpflerp(x0, x1, (t - s0)/(s1 - s0));
}

static void cpMarchCellSoft (cpFloat t, cpFloat a, cpFloat b, cpFloat c, cpFloat d,
							 cpFloat x0, cpFloat x1, cpFloat y0, cpFloat y1,
							 cpMarchSegmentFunc segment, void* segment_data)
{
	// TODO this switch part is super expensive, can it be NEONized?
	switch((a>t)<<0 | (b>t)<<1 | (c>t)<<2 | (d>t)<<3)
	{
		case 0x1: seg(cpv(x0, midlerp(y0,y1,a,c,t)), cpv(midlerp(x0,x1,a,b,t), y0), segment, segment_data); break;
		case 0x2: seg(cpv(midlerp(x0,x1,a,b,t), y0), cpv(x1, midlerp(y0,y1,b,d,t)), segment, segment_data); break;
		case 0x3: seg(cpv(x0, midlerp(y0,y1,a,c,t)), cpv(x1, midlerp(y0,y1,b,d,t)), segment, segment_data); break;
		case 0x4: seg(cpv(midlerp(x0,x1,c,d,t), y1), cpv(x0, midlerp(y0,y1,a,c,t)), segment, segment_data); break;
		case 0x5: seg(cpv(midlerp(x0,x1,c,d,t), y1), cpv(midlerp(x0,x1,a,b,t), y0), segment, segment_data); break;
		case 0x6: seg(cpv(midlerp(x0,x1,a,b,t), y0), cpv(x1, midlerp(y0,y1,b,d,t)), segment, segment_data);
				  seg(cpv(midlerp(x0,x1,c,d,t), y1), cpv(x0, midlerp(y0,y1,a,c,t)), segment, segment_data); break;
		case 0x7: seg(cpv(midlerp(x0,x1,c,d,t), y1), cpv(x1, midlerp(y0,y1,b,d,t)), segment, segment_data); break;
		case 0x8: seg(cpv(x1, midlerp(y0,y1,b,d,t)), cpv(midlerp(x0,x1,c,d,t), y1), segment, segment_data); break;
		case 0x9: seg(cpv(x0, midlerp(y0,y1,a,c,t)), cpv(midlerp(x0,x1,a,b,t), y0), segment, segment_data);
				  seg(cpv(x1, midlerp(y0,y1,b,d,t)), cpv(midlerp(x0,x1,c,d,t), y1), segment, segment_data); break;
		case 0xA: seg(cpv(midlerp(x0,x1,a,b,t), y0), cpv(midlerp(x0,x1,c,d,t), y1), segment, segment_data); break;
		case 0xB: seg(cpv(x0, midlerp(y0,y1,a,c,t)), cpv(midlerp(x0,x1,c,d,t), y1), segment, segment_data); break;
		case 0xC: seg(cpv(x1, midlerp(y0,y1,b,d,t)), cpv(x0, midlerp(y0,y1,a,c,t)), segment, segment_data); break;
		case 0xD: seg(cpv(x1, midlerp(y0,y1,b,d,t)), cpv(midlerp(x0,x1,a,b,t), y0), segment, segment_data); break;
		case 0xE: seg(cpv(midlerp(x0,x1,a,b,t), y0), cpv(x0, midlerp(y0,y1,a,c,t)), segment, segment_data); break;
		default: break; // 0x0 and 0xF
	}	
}
alias cpMarchCellSoftFunc = void function (cpFloat t, cpFloat a, cpFloat b, cpFloat c, cpFloat d,
										   cpFloat x0, cpFloat x1, cpFloat y0, cpFloat y1,
							 			   cpMarchSegmentFunc segment, void* segment_data);

/// Trace an anti-aliased contour of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
void cpMarchSoft (cpBB bb, ulong x_samples, ulong y_samples, cpFloat threshold,
  				  cpMarchSegmentFunc segment, void* segment_data,
  				  cpMarchSampleFunc sample, void* sample_data)
{
	cpMarchCells(bb, x_samples, y_samples, threshold, segment, segment_data, sample, sample_data, safeCast!cpMarchCellSoftFunc(&cpMarchCellSoft));
}


// TODO should flip this around eventually.
static void segs(cpVect a, cpVect b, cpVect c, cpMarchSegmentFunc f, void* data)
{
	seg(b, c, f, data);
	seg(a, b, f, data);
}

static void cpMarchCellHard (cpFloat t, cpFloat a, cpFloat b, cpFloat c, cpFloat d,
							 cpFloat x0, cpFloat x1, cpFloat y0, cpFloat y1,
							 cpMarchSegmentFunc segment, void* segment_data)
{
	// midpoints
	cpFloat xm = cpflerp(x0, x1, 0.5f);
	cpFloat ym = cpflerp(y0, y1, 0.5f);
	
	switch((a>t)<<0 | (b>t)<<1 | (c>t)<<2 | (d>t)<<3){
		case 0x1: segs(cpv(x0, ym), cpv(xm, ym), cpv(xm, y0), segment, segment_data); break;
		case 0x2: segs(cpv(xm, y0), cpv(xm, ym), cpv(x1, ym), segment, segment_data); break;
		case 0x3: seg(cpv(x0, ym), cpv(x1, ym), segment, segment_data); break;
		case 0x4: segs(cpv(xm, y1), cpv(xm, ym), cpv(x0, ym), segment, segment_data); break;
		case 0x5: seg(cpv(xm, y1), cpv(xm, y0), segment, segment_data); break;
		case 0x6: segs(cpv(xm, y0), cpv(xm, ym), cpv(x0, ym), segment, segment_data);
		          segs(cpv(xm, y1), cpv(xm, ym), cpv(x1, ym), segment, segment_data); break;
		case 0x7: segs(cpv(xm, y1), cpv(xm, ym), cpv(x1, ym), segment, segment_data); break;
		case 0x8: segs(cpv(x1, ym), cpv(xm, ym), cpv(xm, y1), segment, segment_data); break;
		case 0x9: segs(cpv(x1, ym), cpv(xm, ym), cpv(xm, y0), segment, segment_data);
		          segs(cpv(x0, ym), cpv(xm, ym), cpv(xm, y1), segment, segment_data); break;
		case 0xA: seg(cpv(xm, y0), cpv(xm, y1), segment, segment_data); break;
		case 0xB: segs(cpv(x0, ym), cpv(xm, ym), cpv(xm, y1), segment, segment_data); break;
		case 0xC: seg(cpv(x1, ym), cpv(x0, ym), segment, segment_data); break;
		case 0xD: segs(cpv(x1, ym), cpv(xm, ym), cpv(xm, y0), segment, segment_data); break;
		case 0xE: segs(cpv(xm, y0), cpv(xm, ym), cpv(x0, ym), segment, segment_data); break;
		default: break; // 0x0 and 0xF
	}
}
alias cpMarchCellHardFunc = void function (cpFloat t, cpFloat a, cpFloat b, cpFloat c, cpFloat d,
							 			   cpFloat x0, cpFloat x1, cpFloat y0, cpFloat y1,
							 			   cpMarchSegmentFunc segment, void* segment_data);

/// Trace an aliased curve of an image along a particular threshold.
/// The given number of samples will be taken and spread across the bounding box area using the sampling function and context.
/// The segment function will be called for each segment detected that lies along the density contour for @c threshold.
void cpMarchHard (cpBB bb, ulong x_samples, ulong y_samples, cpFloat threshold,
  				  cpMarchSegmentFunc segment, void* segment_data,
  				  cpMarchSampleFunc sample, void* sample_data)
{
	cpMarchCells(bb, x_samples, y_samples, threshold, segment, segment_data, sample, sample_data, safeCast!cpMarchCellHardFunc(&cpMarchCellHard));
}
