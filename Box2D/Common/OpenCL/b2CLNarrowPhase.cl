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
// Structures
//==============================================================================================

// This structure is used to keep track of the best separating axis.
typedef struct
{
	int type;
	int index;
	float separation;
} b2clEPAxis;

// This holds polygon B expressed in frame A.
typedef struct
{
	float2 vertices[b2cl_maxPolygonVertices];
	float2 normals[b2cl_maxPolygonVertices];
	int count;
} b2clTempPolygon;

// Reference face used for clipping
typedef struct
{
	int i1, i2;
	
	float2 v1, v2;
	
	float2 normal;
	
	float2 sideNormal1;
	float sideOffset1;
	
	float2 sideNormal2;
	float sideOffset2;
} b2clReferenceFace;

//==============================================================================================
// Sub-functions
//==============================================================================================

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
static float b2clEdgeSeparation(const b2clPolygonShape* poly1, const b2clTransform* xf1, int edge1,
							  const b2clPolygonShape* poly2, const b2clTransform* xf2/*, float4 *test*/)
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
	//(*test).xy = vertices2[index];
	//(*test).zw = index;
	return separation;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
float b2clFindMaxSeparation(int* edgeIndex,
							const b2clPolygonShape* poly1, const b2clTransform* xf1,
							const b2clPolygonShape* poly2, const b2clTransform* xf2
							/*, float4 *test*/)
{
	int count1 = poly1->m_vertexCount;
	const float2* normals1 = poly1->m_normals;

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(xf2, poly2->m_centroid) - b2clMul_Transform(xf1, poly1->m_centroid);
	float2 dLocal1 = b2clMulT_Rotate(xf1->q, d);

	//(*test).x = poly1->m_centroid.x;
	//(*test).y = poly1->m_centroid.y;
	//(*test).z = poly2->m_centroid.x;
	//(*test).w = poly2->m_centroid.y;

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

	//float4 temp;
	// Get the separation for the edge normal.
	float s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2/*, &temp*/);

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	float sPrev = b2clEdgeSeparation(poly1, xf1, prevEdge, poly2, xf2/*, &temp*/);

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	float sNext = b2clEdgeSeparation(poly1, xf1, nextEdge, poly2, xf2/*, &temp*/);

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

		s = b2clEdgeSeparation(poly1, xf1, edge, poly2, xf2/*, &temp*/);

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

inline b2clEPAxis b2clComputeEdgeSeparation(bool m_front, b2clTempPolygon *m_polygonB, float2 m_normal, float2 m_v1)
{
	b2clEPAxis axis;
	axis.type = 1; //b2EPAxis::e_edgeA;
	axis.index = m_front ? 0 : 1;
	axis.separation = MAXFLOAT;
	
	for (int i = 0; i < m_polygonB->count; ++i)
	{
		float s = b2clDot(m_normal, m_polygonB->vertices[i] - m_v1);
		if (s < axis.separation)
		{
			axis.separation = s;
		}
	}
	
	return axis;
}

inline b2clEPAxis b2clComputePolygonSeparation(float2 m_normal, b2clTempPolygon *m_polygonB, float2 m_v1, float2 m_v2, float2 m_upperLimit, float2 m_lowerLimit, float m_radius)
{
	b2clEPAxis axis;
	axis.type = 0; //b2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -MAXFLOAT;

	float2 perp = (float2)(-m_normal.y, m_normal.x);

	for (int i = 0; i < m_polygonB->count; ++i)
	{
		float2 n = -m_polygonB->normals[i];
		
		float s1 = b2clDot(n, m_polygonB->vertices[i] - m_v1);
		float s2 = b2clDot(n, m_polygonB->vertices[i] - m_v2);
		float s = min(s1, s2);
		
		if (s > m_radius)
		{
			// No collision
			axis.type = 2; //b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
			return axis;
		}
		
		// Adjacency
		if (b2clDot(n, perp) >= 0.0f)
		{
			if (b2clDot(n - m_upperLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		else
		{
			if (b2clDot(n - m_lowerLimit, m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		
		if (s > axis.separation)
		{
			axis.type = 2; //b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}
	
	return axis;
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
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
					  const __global b2clPolygonShape* polyGlobal, 
					  const __global b2clTransform* xfGlobal,
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
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape polyA, polyB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*2]; // 2 is contact type for polygon-polygon
	index = global_indices[pair_index];
	polyA = polyGlobal[index.x];
	polyB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = polyA.m_bIsSensor || polyB.m_bIsSensor;

	float totalRadius = polyA.m_radius + polyB.m_radius;

	//float4 test;

	int edgeA = 0;
	float separationA = b2clFindMaxSeparation(&edgeA, &polyA, &xfA, &polyB, &xfB/*, &test*/);

	if (separationA > totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	int edgeB = 0;
	float separationB = b2clFindMaxSeparation(&edgeB, &polyB, &xfB, &polyA, &xfA/*, &test*/);
	if (separationB > totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
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
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
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

	//for (int k = 0; k < manifold.pointCount; ++k)
	//{
	//	b2clManifoldPoint* mp2 = manifold.points + k;
	//	mp2->normalImpulse = 0.0f;
	//	mp2->tangentImpulse = 0.0f;
	//	int key2 = mp2->id.key;

	//	b2clManifold oldManifold = manifolds[i];
	//	for (int j = 0; j < oldManifold.pointCount; ++j)
	//	{
	//		b2clManifoldPoint* mp1 = oldManifold.points + j;

	//		if (mp1->id.key == key2)
	//		{
	//			mp2->normalImpulse = mp1->normalImpulse;
	//			mp2->tangentImpulse = mp1->tangentImpulse;
	//			break;
	//		}
	//	}
	//	manifold.test.x = oldManifold.pointCount;
	//}

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = pointCount>0 ? !bSensor : 0;
}

__kernel void b2clCollideCircles(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
					  const __global b2clPolygonShape* polyGlobal, 
					  const __global b2clTransform* xfGlobal,
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
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
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
	bool bSensor = circleA.m_bIsSensor || circleB.m_bIsSensor;

	float totalRadius = circleA.m_radius + circleB.m_radius;
	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	float2 d = b2clMul_Transform(&xfA, circleA.m_centroid) - b2clMul_Transform(&xfB, circleB.m_centroid);
	float distSqr = b2clDot(d, d);

	if (distSqr > totalRadius * totalRadius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}

	manifold.type = 0; // b2Manifold::e_circles
	manifold.localPoint = circleA.m_centroid;
	manifold.localNormal = 0;
	manifold.pointCount = 1;

	manifold.points[0].localPoint = circleB.m_centroid;
	manifold.points[0].id.key = 0;

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollidePolygonAndCircle(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
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
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape polyA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*1]; // 1 is contact type for polygon-circle
	index = global_indices[pair_index];
	polyA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = polyA.m_bIsSensor || circleB.m_bIsSensor;

	// Compute circle position in the frame of the polygon.
	float2 c = b2clMul_Transform(&xfB, circleB.m_centroid);
	float2 cLocal = b2clMulT_Transform(&xfA, c);

	// Find the min separating edge.
	int normalIndex = 0;
	float separation = -b2_maxFloat;
	float radius = polyA.m_radius + circleB.m_radius;
	int vertexCount = polyA.m_vertexCount;
	const float2* vertices = polyA.m_vertices;
	const float2* normals = polyA.m_normals;

	for (int i = 0; i < vertexCount; ++i)
	{
		float s = b2clDot(normals[i], cLocal - vertices[i]);

		if (s > radius)
		{
			// Early out.
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			//printf("early out, pair index: %d\n", pair_index);
			return;
		}

		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	int vertIndex1 = normalIndex;
	int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	float2 v1 = vertices[vertIndex1];
	float2 v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < b2_epsilon)
	{
		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normals[normalIndex];
		manifold.localPoint = 0.5f * (v1 + v2);
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}

	// Compute barycentric coordinates
	float u1 = b2clDot(cLocal - v1, v2 - v1);
	float u2 = b2clDot(cLocal - v2, v1 - v2);
	if (u1 <= 0.0f)
	{
		if (b2clDistanceSquared(cLocal, v1) > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normalize(cLocal - v1);
		manifold.localPoint = v1;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}
	else if (u2 <= 0.0f)
	{
		if (b2clDistanceSquared(cLocal, v2) > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normalize(cLocal - v2);
		manifold.localPoint = v2;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}
	else
	{
		float2 faceCenter = 0.5f * (v1 + v2);
		float separation = b2clDot(cLocal - faceCenter, normals[vertIndex1]);
		if (separation > radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}

		manifold.pointCount = 1;
		manifold.type = 1; //b2Manifold::e_faceA
		manifold.localNormal = normals[vertIndex1];
		manifold.localPoint = faceCenter;
		manifold.points[0].localPoint = circleB.m_centroid;
		manifold.points[0].id.key = 0;
	}

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollideEdgeAndCircle(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
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
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape edgeA, circleB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*3]; // 3 is contact type for edge-circle
	index = global_indices[pair_index];
	edgeA = polyGlobal[index.x];
	circleB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = edgeA.m_bIsSensor || circleB.m_bIsSensor;

	// Compute circle in frame of edge
	float2 Q = b2clMulT_Transform(&xfA, b2clMul_Transform(&xfB, circleB.m_centroid));
	
	float2 A = edgeA.m_vertices[0], B = edgeA.m_vertices[1];
	float2 e = B - A;
	
	// Barycentric coordinates
	float u = b2clDot(e, B - Q);
	float v = b2clDot(e, Q - A);
	
	float radius = edgeA.m_radius + circleB.m_radius;
	
	b2clContactFeature cf;
	cf.indexB = 0;
	cf.typeB = 0; //b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		float2 P = A;
		float2 d = Q - P;
		float dd = b2clDot(d, d);
		if (dd > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}
		
		// Is there an edge connected to A?
		if (edgeA.m_centroid.x) // m_centroid.x is used for m_hasVertex0
		{
			float2 A1 = edgeA.m_vertices[2]; // m_vertices[2] is used for m_vertex0
			float2 B1 = A;
			float2 e1 = B1 - A1;
			float u1 = b2clDot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
			{
				manifolds[pair_index] = manifold;
				manifoldBinaryBits[pair_index] = 0;
				return;
			}
		}
		
		cf.indexA = 0;
		cf.typeA = 0; //b2ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = 0; //b2Manifold::e_circles;
		manifold.localNormal = 0;
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_centroid;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		float2 P = B;
		float2 d = Q - P;
		float dd = b2clDot(d, d);
		if (dd > radius * radius)
		{
			manifolds[pair_index] = manifold;
			manifoldBinaryBits[pair_index] = 0;
			return;
		}
		
		// Is there an edge connected to B?
		if (edgeA.m_centroid.y) // m_centroid.y is used for m_hasVertex3
		{
			float2 B2 = edgeA.m_vertices[3];
			float2 A2 = B;
			float2 e2 = B2 - A2;
			float v2 = b2clDot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
			{
				manifolds[pair_index] = manifold;
				manifoldBinaryBits[pair_index] = 0;
				return;
			}
		}
		
		cf.indexA = 1;
		cf.typeA = 0; //b2ContactFeature::e_vertex;
		manifold.pointCount = 1;
		manifold.type = 0; //b2Manifold::e_circles;
		manifold.localNormal = 0;
		manifold.localPoint = P;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB.m_centroid;

		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = !bSensor;
		return;
	}
	
	// Region AB
	float den = b2clDot(e, e);
	float2 P = (1.0f / den) * (u * A + v * B);
	float2 d = Q - P;
	float dd = b2clDot(d, d);
	if (dd > radius * radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	float2 n = (float2)(-e.y, e.x);
	if (b2clDot(n, Q - A) < 0.0f)
	{
		float2 nn = (float2)(-n.x, -n.y);
		n = nn;
	}
	n = normalize(n);
	
	cf.indexA = 0;
	cf.typeA = 1; //b2ContactFeature::e_face;
	manifold.pointCount = 1;
	manifold.type = 1; //b2Manifold::e_faceA;
	manifold.localNormal = n;
	manifold.localPoint = A;
	manifold.points[0].id.key = 0;
	manifold.points[0].id.cf = cf;
	manifold.points[0].localPoint = circleB.m_centroid;

	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
}

__kernel void b2clCollideEdgeAndPolygon(__global b2clManifold* manifolds, // output
								  __global int* manifoldBinaryBits, // output
								  //__global uint* impulsesKeys, // output
								  const __global b2clPolygonShape* polyGlobal, 
								  const __global b2clTransform* xfGlobal,
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
    manifold.points[0].normalImpulse = 0;
    manifold.points[0].tangentImpulse = 0;
    manifold.points[1].normalImpulse = 0;
    manifold.points[1].tangentImpulse = 0;
	int type;

	int pair_index;
	int4 index;
	b2clPolygonShape edgeA, polygonB;
	b2clTransform xfA, xfB;

	pair_index = pair_indices[i + maxContactNum*4]; // 4 is contact type for edge-polygon
	index = global_indices[pair_index];
	edgeA = polyGlobal[index.x];
	polygonB = polyGlobal[index.y];
	xfA = xfGlobal[index.z];
	xfB = xfGlobal[index.w];
	bool bSensor = edgeA.m_bIsSensor || polygonB.m_bIsSensor;

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
	b2clTempPolygon m_polygonB;

	b2clTransform m_xf;
	float2 m_centroidB;
	float2 m_v0, m_v1, m_v2, m_v3;
	float2 m_normal0, m_normal1, m_normal2;
	float2 m_normal;
	int m_type1, m_type2;
	float2 m_lowerLimit, m_upperLimit;
	float m_radius;
	bool m_front;

	m_xf = b2clMulT_TwoTransform(&xfA, &xfB);
	
	m_centroidB = b2clMul_Transform(&m_xf, polygonB.m_centroid);
	
	m_v0 = edgeA.m_vertices[2]; // m_vertices[2] is used for m_vertex0
	m_v1 = edgeA.m_vertices[0]; // m_vertices[0] is used for m_vertex1
	m_v2 = edgeA.m_vertices[1]; // m_vertices[1] is used for m_vertex2
	m_v3 = edgeA.m_vertices[3]; // m_vertices[3] is used for m_vertex3
	
	bool hasVertex0 = edgeA.m_centroid.x; // m_centroid.x is used for m_hasVertex0
	bool hasVertex3 = edgeA.m_centroid.y; // m_centroid.y is used for m_hasVertex3
	
	float2 edge1 = m_v2 - m_v1;
	edge1 = normalize(edge1);
	m_normal1 = (float2)(edge1.y, -edge1.x);
	float offset1 = b2clDot(m_normal1, m_centroidB - m_v1);
	float offset0 = 0.0f, offset2 = 0.0f;
	bool convex1 = false, convex2 = false;
	
	// Is there a preceding edge?
	if (hasVertex0)
	{
		float2 edge0 = m_v1 - m_v0;
		edge0 = normalize(edge0);
		m_normal0 = (float2)(edge0.y, -edge0.x);
		convex1 = b2clCross_VV(edge0, edge1) >= 0.0f;
		offset0 = b2clDot(m_normal0, m_centroidB - m_v0);
	}
	
	// Is there a following edge?
	if (hasVertex3)
	{
		float2 edge2 = m_v3 - m_v2;
		edge2 = normalize(edge2);
		m_normal2 = (float2)(edge2.y, -edge2.x);
		convex2 = b2clCross_VV(edge1, edge2) > 0.0f;
		offset2 = b2clDot(m_normal2, m_centroidB - m_v2);
	}
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		if (convex1 && convex2)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex1)
		{
			m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex2)
		{
			m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex0)
	{
		if (convex1)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex3)
	{
		if (convex2)
		{
			m_front = offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
		}
		else
		{
			m_front = offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = m_normal1;
			}
		}		
	}
	else
	{
		m_front = offset1 >= 0.0f;
		if (m_front)
		{
			m_normal = m_normal1;
			m_lowerLimit = -m_normal1;
			m_upperLimit = -m_normal1;
		}
		else
		{
			m_normal = -m_normal1;
			m_lowerLimit = m_normal1;
			m_upperLimit = m_normal1;
		}
	}
	
	// Get polygonB in frameA
	m_polygonB.count = polygonB.m_vertexCount;
	for (int i = 0; i < polygonB.m_vertexCount; ++i)
	{
		m_polygonB.vertices[i] = b2clMul_Transform(&m_xf, polygonB.m_vertices[i]);
		m_polygonB.normals[i] = b2clMul_Rotate(m_xf.q, polygonB.m_normals[i]);
	}
	
	m_radius = 2.0f * b2_polygonRadius;
	
	manifold.pointCount = 0;
	
	b2clEPAxis edgeAxis = b2clComputeEdgeSeparation(m_front, &m_polygonB, m_normal, m_v1);
	
	// If no valid normal can be found than this edge should not collide.
	if (edgeAxis.type == 0 /*b2EPAxis::e_unknown*/) // ??? Never true?
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	if (edgeAxis.separation > m_radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	b2clEPAxis polygonAxis = b2clComputePolygonSeparation(m_normal, &m_polygonB, m_v1, m_v2, m_upperLimit, m_lowerLimit, m_radius);
	if (polygonAxis.type != 0 /*b2EPAxis::e_unknown*/ && polygonAxis.separation > m_radius)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;
	
	b2clEPAxis primaryAxis;
	if (polygonAxis.type == 0 /*b2EPAxis::e_unknown*/)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}
	
	b2clClipVertex ie[2];
	b2clReferenceFace rf;
	if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
	{
		manifold.type = 1; //b2Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		int bestIndex = 0;
		float bestValue = b2clDot(m_normal, m_polygonB.normals[0]);
		for (int i = 1; i < m_polygonB.count; ++i)
		{
			float value = b2clDot(m_normal, m_polygonB.normals[i]);
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}
		
		int i1 = bestIndex;
		int i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;
		
		ie[0].v = m_polygonB.vertices[i1];
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = i1;
		ie[0].id.cf.typeA = 1; //b2ContactFeature::e_face;
		ie[0].id.cf.typeB = 0; //b2ContactFeature::e_vertex;
		
		ie[1].v = m_polygonB.vertices[i2];
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = i2;
		ie[1].id.cf.typeA = 1; //b2ContactFeature::e_face;
		ie[1].id.cf.typeB = 0; //b2ContactFeature::e_vertex;
		
		if (m_front)
		{
			rf.i1 = 0;
			rf.i2 = 1;
			rf.v1 = m_v1;
			rf.v2 = m_v2;
			rf.normal = m_normal1;
		}
		else
		{
			rf.i1 = 1;
			rf.i2 = 0;
			rf.v1 = m_v2;
			rf.v2 = m_v1;
			rf.normal = -m_normal1;
		}		
	}
	else
	{
		manifold.type = 2; //b2Manifold::e_faceB;
		
		ie[0].v = m_v1;
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = primaryAxis.index;
		ie[0].id.cf.typeA = 0; //b2ContactFeature::e_vertex;
		ie[0].id.cf.typeB = 1; //b2ContactFeature::e_face;
		
		ie[1].v = m_v2;
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = primaryAxis.index;		
		ie[1].id.cf.typeA = 0; //b2ContactFeature::e_vertex;
		ie[1].id.cf.typeB = 1; //b2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
		rf.v1 = m_polygonB.vertices[rf.i1];
		rf.v2 = m_polygonB.vertices[rf.i2];
		rf.normal = m_polygonB.normals[rf.i1];
	}
	
	rf.sideNormal1 = (float2)(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = b2clDot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2clDot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	b2clClipVertex clipPoints1[2];
	b2clClipVertex clipPoints2[2];
	int np;
	
	// Clip to box side 1
	np = b2clClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
	
	if (np < b2cl_maxManifoldPoints)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
	
	if (np < b2cl_maxManifoldPoints)
	{
		manifolds[pair_index] = manifold;
		manifoldBinaryBits[pair_index] = 0;
		return;
	}
	
	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
	{
		manifold.localNormal = rf.normal;
		manifold.localPoint = rf.v1;
	}
	else
	{
		manifold.localNormal = polygonB.m_normals[rf.i1];
		manifold.localPoint = polygonB.m_vertices[rf.i1];
	}
	
	int pointCount = 0;
	for (int i = 0; i < b2cl_maxManifoldPoints; ++i)
	{
		float separation;
		
		separation = b2clDot(rf.normal, clipPoints2[i].v - rf.v1);
		
		if (separation <= m_radius)
		{
			b2clManifoldPoint* cp = manifold.points + pointCount;
			
			if (primaryAxis.type == 1 /*b2EPAxis::e_edgeA*/)
			{
				cp->localPoint = b2clMulT_Transform(&m_xf, clipPoints2[i].v);
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint = clipPoints2[i].v;
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}
			
			++pointCount;
		}
	}
	
	manifold.pointCount = pointCount;
	manifolds[pair_index] = manifold;
	manifoldBinaryBits[pair_index] = !bSensor;
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