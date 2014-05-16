/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#include <Box2D/Common/OpenCL/b2CLTypeDefOCL.h>

//==============================================================================================
// Sub-functions
//==============================================================================================

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
static float b2clEdgeSeparation(const b2clPolygonShape* poly1, const b2clTransform* xf1, int edge1,
							  const b2clPolygonShape* poly2, const b2clTransform* xf2, float4 *test)
{
	const float2* vertices1 = poly1->m_vertices;
	const float2* normals1 = poly1->m_normals;

	int count2 = poly2->m_vertexCount;
	const float2* vertices2 = poly2->m_vertices;

	//b2Assert(0 <= edge1 && edge1 < poly1->m_vertexCount);

	// Convert normal from poly1's frame into poly2's frame.
	float2 normal1World = b2clMul_Rotate(xf1->q, normals1[edge1]);
	float2 normal1 = b2clMulT_Rotate(xf2->q, normal1World);

	// Find support vertex on poly2 for -normal.
	int index = 0;
	float minDot = MAXFLOAT;

	for (int i = 0; i < count2; ++i)
	{
		float dotProduct = dot(vertices2[i], normal1);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			index = i;
		}
	}

	float2 v1 = b2clMul_Transform(xf1, vertices1[edge1]);
	float2 v2 = b2clMul_Transform(xf2, vertices2[index]);
	float separation = dot(v2 - v1, normal1World);
	(*test).xy = vertices2[index];
	(*test).zw = index;
	return separation;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
float b2clFindMaxSeparation(int* edgeIndex,
							const b2clPolygonShape* poly1, const b2clTransform* xf1,
							const b2clPolygonShape* poly2, const b2clTransform* xf2,
							float4 *test)
{
	int count1 = poly1->m_vertexCount;
	const float2* normals1 = poly1->m_normals;

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(xf2, poly2->m_centroid) - b2clMul_Transform(xf1, poly1->m_centroid);
	float2 dLocal1 = b2clMulT_Rotate(xf1->q, d);

	(*test).x = poly1->m_centroid.x;
	(*test).y = poly1->m_centroid.y;
	(*test).z = poly2->m_centroid.x;
	(*test).w = poly2->m_centroid.y;

	// Find edge normal on poly1 that has the largest projection onto d.
	int edge = 0;
	float maxDot = -MAXFLOAT;
	for (int i = 0; i < count1; ++i)
	{
		float dotProduct = dot(normals1[i], dLocal1);
		if (dotProduct > maxDot)
		{
			maxDot = dotProduct;
			edge = i;
		}
	}

	float4 temp;
	// Get the separation for the edge normal.
	float s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2, &temp);

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	float sPrev = b2clEdgeSeparation(poly1, xf1, prevEdge, poly2, xf2, &temp);

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	float sNext = b2clEdgeSeparation(poly1, xf1, nextEdge, poly2, xf2, &temp);

	// Find the best edge and the search direction.
	int bestEdge;
	float bestSeparation;
	int increment;
	if (sPrev > s && sPrev > sNext)
	{
		increment = -1;
		bestEdge = prevEdge;
		bestSeparation = sPrev;
	}
	else if (sNext > s)
	{
		increment = 1;
		bestEdge = nextEdge;
		bestSeparation = sNext;
	}
	else
	{
		*edgeIndex = edge;
		return s;
	}

	// Perform a local search for the best edge normal.
	for ( ; ; )
	{
		if (increment == -1)
			edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		else
			edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2, &temp);

		if (s > bestSeparation)
		{
			bestEdge = edge;
			bestSeparation = s;
		}
		else
		{
			break;
		}
	}

	*edgeIndex = bestEdge;

	return bestSeparation;
}

static void b2clFindIncidentEdge(b2clClipVertex c[2],
							 const b2clPolygonShape* poly1, const b2clTransform* xf1, int edge1,
							 const b2clPolygonShape* poly2, const b2clTransform* xf2)
{
	const float2* normals1 = poly1->m_normals;

	int count2 = poly2->m_vertexCount;
	const float2* vertices2 = poly2->m_vertices;
	const float2* normals2 = poly2->m_normals;

	//b2Assert(0 <= edge1 && edge1 < poly1->m_vertexCount);

	// Get the normal of the reference edge in poly2's frame.
	float2 normal1 = b2clMulT_Rotate(xf2->q, b2clMul_Rotate(xf1->q, normals1[edge1]));

	// Find the incident edge on poly2.
	int index = 0;
	float minDot = MAXFLOAT;
	for (int i = 0; i < count2; ++i)
	{
		float dotProduct = dot(normal1, normals2[i]);
		if (dotProduct < minDot)
		{
			minDot = dotProduct;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int i1 = index;
	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2clMul_Transform(xf2, vertices2[i1]);
	c[0].id.cf.indexA = (uint)edge1;
	c[0].id.cf.indexB = (uint)i1;
	c[0].id.cf.typeA = 1;
	c[0].id.cf.typeB = 0;

	c[1].v = b2clMul_Transform(xf2, vertices2[i2]);
	c[1].id.cf.indexA = (uint)edge1;
	c[1].id.cf.indexB = (uint)i2;
	c[1].id.cf.typeA = 1;
	c[1].id.cf.typeB = 0;
}

// Sutherland-Hodgman clipping.
int b2clClipSegmentToLine(b2clClipVertex vOut[2], const b2clClipVertex vIn[2],
							const float2 normal, float offset, int vertexIndexA)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = dot(normal, vIn[0].v) - offset;
	float distance1 = dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		// VertexA is hitting edgeB.
		vOut[numOut].id.cf.indexA = vertexIndexA;
		vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
		vOut[numOut].id.cf.typeA = 0;
		vOut[numOut].id.cf.typeB = 1;
		++numOut;
	}

	return numOut;
}

//==============================================================================================
// Main kernel function
//==============================================================================================

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
__kernel void b2clCollidePolygons(__global b2clManifold* manifolds, // output
					  const __global b2clPolygonShape* polyGlobal, const __global b2clTransform* xfGlobal,
					  const __global int4* indices,
					  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold;

	manifold.pointCount = 0;
	int type;

	int4 index;
	b2clPolygonShape polyA, polyB;
	b2clTransform xfA, xfB;

	index = indices[i];
	polyA = polyGlobal[index.x];
	polyB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];

	float totalRadius = polyA.m_radius + polyB.m_radius;

	float4 test;

	int edgeA = 0;
	float separationA = b2clFindMaxSeparation(&edgeA, &polyA, &xfA, &polyB, &xfB, &test);

	if (separationA > totalRadius)
	{
		manifolds[i] = manifold;
		return;
	}

	int edgeB = 0;
	float separationB = b2clFindMaxSeparation(&edgeB, &polyB, &xfB, &polyA, &xfA, &test);
	if (separationB > totalRadius)
	{
		manifolds[i] = manifold;
		return;
	}

	const b2clPolygonShape* poly1;	// reference polygon
	const b2clPolygonShape* poly2;	// incident polygon
	b2clTransform *xf1, *xf2;
	int edge1;		// reference edge
	uint flip;
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;

	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = &polyB;
		poly2 = &polyA;
		xf1 = &xfB;
		xf2 = &xfA;
		edge1 = edgeB;
		manifold.type = 2;
		flip = 1;
	}
	else
	{
		poly1 = &polyA;
		poly2 = &polyB;
		xf1 = &xfA;
		xf2 = &xfB;
		edge1 = edgeA;
		manifold.type = 1;
		flip = 0;
	}

	b2clClipVertex incidentEdge[2];
	b2clFindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int count1 = poly1->m_vertexCount;
	const float2* vertices1 = poly1->m_vertices;

	int iv1 = edge1;
	int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	float2 v11 = vertices1[iv1];
	float2 v12 = vertices1[iv2];

	float2 localTangent = v12 - v11;
	localTangent = normalize(localTangent);
	
	float2 localNormal = b2clCross_VS(localTangent, 1.0f);
	float2 planePoint = 0.5f * (v11 + v12);

	float2 tangent = b2clMul_Rotate(xf1->q, localTangent);
	float2 normal = b2clCross_VS(tangent, 1.0f);
	
	v11 = b2clMul_Transform(xf1, v11);
	v12 = b2clMul_Transform(xf1, v12);

	// Face offset.
	float frontOffset = dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	float sideOffset1 = -dot(tangent, v11) + totalRadius;
	float sideOffset2 = dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	b2clClipVertex clipPoints1[2];
	b2clClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = b2clClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

	if (np < 2)
	{
		manifolds[i] = manifold;
		return;
	}

	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		manifolds[i] = manifold;
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal = localNormal;
	manifold.localPoint = planePoint;

	int pointCount = 0;
	for (int k = 0; k < b2cl_maxManifoldPoints; ++k)
	{
		float separation = dot(normal, clipPoints2[k].v) - frontOffset;

		if (separation <= totalRadius)
		{
			b2clManifoldPoint* cp = manifold.points + pointCount;
			cp->localPoint = b2clMulT_Transform(xf2, clipPoints2[k].v);
			cp->id = clipPoints2[k].id;
			if (flip)
			{
				// Swap features
				b2clContactFeature cf = cp->id.cf;
				cp->id.cf.indexA = cf.indexB;
				cp->id.cf.indexB = cf.indexA;
				cp->id.cf.typeA = cf.typeB;
				cp->id.cf.typeB = cf.typeA;
			}
			++pointCount;
		}
	}

	manifold.pointCount = pointCount;

	for (int k = 0; k < manifold.pointCount; ++k)
	{
		b2clManifoldPoint* mp2 = manifold.points + k;
		mp2->normalImpulse = 0.0f;
		mp2->tangentImpulse = 0.0f;
		int key2 = mp2->id.key;

		b2clManifold oldManifold = manifolds[i];
		for (int j = 0; j < oldManifold.pointCount; ++j)
		{
			b2clManifoldPoint* mp1 = oldManifold.points + j;

			if (mp1->id.key == key2)
			{
				mp2->normalImpulse = mp1->normalImpulse;
				mp2->tangentImpulse = mp1->tangentImpulse;
				break;
			}
		}
		//manifold.test.x = oldManifold.pointCount;
	}

	manifolds[i] = manifold;
}

__kernel void b2clCollideCircles(__global b2clManifold* manifolds, // output
					  const __global b2clPolygonShape* polyGlobal, const __global b2clTransform* xfGlobal,
					  const __global int4* global_indices,
					  const __global int* pair_indices,
					  const int maxContactNum,
					  const int contactNum)
{
    unsigned int i = get_global_id(0);

	if (i>=contactNum)
		return;

	b2clManifold manifold/* = manifolds[i]*/;

	manifold.pointCount = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape circleA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i]; // 0 is contact type for circle-circle
	index = global_indices[pair_index];
	circleA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];

	float totalRadius = circleA.m_radius + circleB.m_radius;
	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(&xfA, circleA.m_centroid) - b2clMul_Transform(&xfB, circleB.m_centroid);
	float distSqr = b2clDot(d, d);

	if (distSqr > totalRadius * totalRadius)
	{
		manifolds[pair_index] = manifold;
		return;
	}

	manifold.type = 0; // b2Manifold::e_circles
	manifold.localPoint = circleA.m_centroid;
	manifold.localNormal = 0;
	manifold.pointCount = 1;

	manifold.points[0].localPoint = circleB.m_centroid;
	manifold.points[0].id.key = 0;

	manifolds[pair_index] = manifold;
}

__kernel void b2clCompactForOneContact(
					const __global int *CompactIn_data, // input
					__global int *CompactOut_data, // output
					__global int *numValidData // output
					)
{
	*numValidData = CompactIn_data[0];
	if (*numValidData)
		CompactOut_data[0] = 0;
}