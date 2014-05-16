
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

typedef struct 
{
	float m_min[2];
	float m_max[2];
	uchar m_sType;
	uchar m_bType;
	//uchar mask[2];
} b2clAABB;

//http://stereopsis.com/radix.html

uint FloatFlip(float fl)
{
	uint f = as_uint(fl);
	uint mask = -(int)(f >> 31) | 0x80000000;
	return f ^ mask;
}

float IFloatFlip(uint f)
{
	uint mask = ((f >> 31) - 1) | 0x80000000;
	uint fl = f ^ mask;
	return as_float(fl);
}

bool TestAabbAgainstAabb2(const b2clAABB* aabb1, __local const b2clAABB* aabb2)
{
	bool overlap = true;
	overlap = (aabb1->m_min[0] > aabb2->m_max[0] || aabb1->m_max[0] < aabb2->m_min[0]) ? false : overlap;
	overlap = (aabb1->m_min[1] > aabb2->m_max[1] || aabb1->m_max[1] < aabb2->m_min[1]) ? false : overlap;
	return overlap;
}

//bool TestAabbAgainstAabb2GlobalGlobal(__global const b2clAABB* aabb1, __global const b2clAABB* aabb2)
bool TestAabbAgainstAabb2GlobalGlobal(const b2clAABB* aabb1, __global const b2clAABB* aabb2)
{
	bool overlap = true;
	//if ((aabb1->mask[0] ^ aabb2->mask[0]) && (aabb1->mask[0] ^ aabb2->mask[1]) &&
	//	(aabb1->mask[1] ^ aabb2->mask[0]) && (aabb1->mask[1] ^ aabb2->mask[1]))
	//	return false;
	overlap = (aabb1->m_min[0] > aabb2->m_max[0] || aabb1->m_max[0] < aabb2->m_min[0]) ? false : overlap;
	overlap = (aabb1->m_min[1] > aabb2->m_max[1] || aabb1->m_max[1] < aabb2->m_min[1]) ? false : overlap;
	return overlap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ShouldCollideJoints
//
// Determines whether bodies A and B should collide or not.
// If A and B are connected by a joint which does not allow collisions between connected bodies,
// this function returns false.
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ShouldCollideJoints(int bodyA, int bodyB, const __global int* connectedBodyIndicesA, const __global int* connectedBodyIndicesB)
{
	int bodyIndex;
	const __global int* connectedBodyIndices;
	if (bodyA < bodyB)
	{
		bodyIndex = bodyB;
		connectedBodyIndices = connectedBodyIndicesA;
	}
	else
	{
		bodyIndex = bodyA;
		connectedBodyIndices = connectedBodyIndicesB;
	}

	for (int i = 0; i < MAX_CONNECTED_BODY_INDICES; ++i)
	{
		if (connectedBodyIndices[i] == bodyIndex)
			return false;
		if (connectedBodyIndices[i] == -1)
			return true;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ShouldCollide
//
// Determines whether bodies A and B should collide or not from their category bits, mask bits, and group indices.
// It follows the collision filtering rule in the Box2D manual 6.2.
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ShouldCollide(ushort categoryBitsA, ushort maskBitsA, short groupIndexA, 
				   ushort categoryBitsB, ushort maskBitsB, short groupIndexB)
{
	if (groupIndexA == groupIndexB && groupIndexA != 0)
	{
		return groupIndexA > 0;
	}

	bool collide = (maskBitsA & categoryBitsB) != 0 && (categoryBitsA & maskBitsB) != 0;
	return collide;
}

__kernel void ComputeAABBs(
		const __global b2clPolygonShape* polyGlobal,
		const __global b2clBodyStatic* bodyStaticListBuffer,
		const __global b2clTransform* xfGlobal,
		__global b2clAABB* AABBs, // output
		const __global int* shapeToBodyMap,
		int numShapes)
{
	int i = get_global_id(0);

	if (i>=numShapes)
		return;

	b2clPolygonShape poly = polyGlobal[i];
	int j = shapeToBodyMap[i];
	b2clTransform xf = xfGlobal[j];
	b2clBodyStatic bs = bodyStaticListBuffer[j];

	const float2* vertices = poly.m_vertices;

	float2 lower = b2clMul_Transform(&xf, vertices[0]);
	float2 upper = lower;

	if (poly.m_type>0) // edge (m_type==1) or polygon (m_type==2)
	{
		float2 v = b2clMul_Transform(&xf, vertices[1]);
		lower = b2clMin(lower, v);
		upper = b2clMax(upper, v);
	}

	// In Box2D 2.1.2, polygon is m_type==1
	if (poly.m_type==2) // polygon (m_type==2)
	{
		for (int k = 2; k < poly.m_vertexCount; ++k)
		{
			float2 v = b2clMul_Transform(&xf, vertices[k]);
			lower = b2clMin(lower, v);
			upper = b2clMax(upper, v);
		}
	}

	float r = poly.m_radius;

	AABBs[i].m_min[0] = lower.x - r;
	AABBs[i].m_min[1] = lower.y - r;
	AABBs[i].m_max[0] = upper.x + r;
	AABBs[i].m_max[1] = upper.y + r;
	
	AABBs[i].m_sType = poly.m_type;
	AABBs[i].m_bType = bs.m_type;
}

__kernel void ComputeAABBsTOI(
		const __global b2clPolygonShape* polyGlobal,
		const __global b2clBodyStatic* bodyStaticListBuffer,
	    const __global b2clBodyDynamic* bodyDynamicListBuffer,
		const __global b2clTransform* xfGlobal,
		__global b2clAABB* AABBs, // output
		const __global int* shapeToBodyMap,
		int numShapes)
{
	int i = get_global_id(0);

	if (i>=numShapes)
		return;

	b2clPolygonShape poly = polyGlobal[i];
	int j = shapeToBodyMap[i];
	b2clTransform xf = xfGlobal[j];
	b2clBodyStatic bs = bodyStaticListBuffer[j];
	b2clBodyDynamic bd = bodyDynamicListBuffer[j];

	const float2* vertices = poly.m_vertices;

	float2 lower = b2clMul_Transform(&xf, vertices[0]);
	float2 upper = lower;

	if (poly.m_type>0) // edge (m_type==1) or polygon (m_type==2)
	{
		float2 v = b2clMul_Transform(&xf, vertices[1]);
		lower = b2clMin(lower, v);
		upper = b2clMax(upper, v);
	}

	// In Box2D 2.1.2, polygon is m_type==1
	if (poly.m_type==2) // polygon (m_type==2)
	{
		for (int k = 2; k < poly.m_vertexCount; ++k)
		{
			float2 v = b2clMul_Transform(&xf, vertices[k]);
			lower = b2clMin(lower, v);
			upper = b2clMax(upper, v);
		}
	}

	float r = poly.m_radius;

	// synchronize xf for each body
	float a0, a;
	float2 c0, c;
	float2 p0, q0, p, q, d;

	a0 = bd.m_sweep.a0;
	a = bd.m_sweep.a;
	c0 = bd.m_sweep.c0;
	c = bd.m_sweep.c;

	float sina, cosa;
	
	sina = sincos(a0, &cosa);
	q0.x = sina;
	q0.y = cosa;
	//q0.x = /*native_*/sin(a0);
	//q0.y = /*native_*/cos(a0);
	//q0.y = cos_wrapper(a0);
	p0 = c0 - b2clMul_Rotate(q0, bs.m_localCenter);

	sina = sincos(a, &cosa);
	q.x = sina;
	q.y = cosa;
	//q.x = /*native_*/sin(a);
	//q.y = /*native_*/cos(a);
	//q.y = cos_wrapper(a);
	p = c - b2clMul_Rotate(q, bs.m_localCenter);

	d = (p - p0) * b2cl_aabbMultiplier;

	//if (!(d.x<0.0f) && !(d.x>=0.0f))
	//{
	//	printf("%d, %f\n", i, d.x);
	//	printf("a0: %f, c0: (%f, %f), a: %f, c: (%f, %f)\n", a0, c0.x, c0.y, a, c.x, c.y);
	//}
	
	// Extend AABB with d
	if (d.x < 0.0f)
	{
		lower.x += d.x;
	}
	else
	{
		upper.x += d.x;
	}

	if (d.y < 0.0f)
	{
		lower.y += d.y;
	}
	else
	{
		upper.y += d.y;
	}

	AABBs[i].m_min[0] = lower.x - r;
	AABBs[i].m_min[1] = lower.y - r;
	AABBs[i].m_max[0] = upper.x + r;
	AABBs[i].m_max[1] = upper.y + r;

	//if (i==0)
	//{
	//	printf("AABB %d: (%f, %f)-(%f, %f)\n", i, lower.x, lower.y, upper.x, upper.y);
	//	printf("d %d: (%f, %f)\n", i, d.x, d.y);
	//}

	AABBs[i].m_sType = poly.m_type;
	AABBs[i].m_bType = bs.m_type;
}

__kernel void PrepareSumVariance( __global const b2clAABB* AABBs, 
			__global float4* sum, 
			//__global float4* sum2, 
			int numAabbs)
{
	int i = get_global_id(0);
	if (i>numAabbs)
		return;
	float2 s, s2;
	s.x = (AABBs[i].m_max[0]+AABBs[i].m_min[0])*0.5f;
	s.y = (AABBs[i].m_max[1]+AABBs[i].m_min[1])*0.5f;
	s2 = s*s;
	sum[i] = (float4)(s, s2);
	//sum2[i] = s*s;	
}

__kernel void PrepareSumVarianceNoFloat4( __global const b2clAABB* AABBs, 
			__global float* sumX, 
			__global float* sumY, 
			__global float* sumX2, 
			__global float* sumY2, 
			int numAabbs)
{
	int i = get_global_id(0);
	if (i>numAabbs)
		return;
	float2 s, s2;
	s.x = (AABBs[i].m_max[0]+AABBs[i].m_min[0])*0.5f;
	s.y = (AABBs[i].m_max[1]+AABBs[i].m_min[1])*0.5f;
	s2 = s*s;
	sumX[i] = s.x;
	sumY[i] = s.y;
	sumX2[i] = s2.x;
	sumY2[i] = s2.y;
}

__kernel void InitSortingKeys(__global const b2clAABB* AABBs, 
		__global uint* keysAABB, // output
		__global uint* indicesAABB, // output
		int numAabbs,
		int axis, 
		int sortCount)
{
	int i = get_global_id(0);
	if (i>=numAabbs)
	{
		if (i<sortCount)
			keysAABB[i] = 0;
		return;
	}

	keysAABB[i] = FloatFlip(AABBs[i].m_max[axis]); // use max_x/y for sorting (descending), convert it to int
	indicesAABB[i] = i;
}

__kernel void ComputePairs( 
		const __global b2clAABB* aabbs, 
//		__global b2clAABB* test, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		volatile __global int* pairIndices, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		volatile __global int* pairCounts, // output
		volatile __global int* pairTotalCount, // output
		int numObjects, 
		int axis, 
		int maxPairs)
{
	int i = get_global_id(0);

	if (i>=numObjects)
		return;

	int shape_i, shape_j, body_i, body_j, type_i, type_j;
	b2clAABB aabb_i, aabb_j;
	shape_i = indicesAABB[i];
	body_i = shapeToBodyMap[shape_i];
	aabb_i = aabbs[shape_i];
	type_i = aabbs[shape_i].m_bType;

	//if (i==0)
	//{
	//	printf("aabb[0]: (%f, %f)-(%f, %f)\n", aabbs[0].m_min[0], aabbs[0].m_min[1], aabbs[0].m_max[0], aabbs[0].m_max[1]);
	//}

//	int localPairCount = 0;
//	
//#define max_pair_per_shape 10
//	int4 localPairIndicesBuffer[max_pair_per_shape]; // assume there will be less than max_pair_per_shape pair for each shape

	unsigned short categoryBitsA = shapeListBuffer[shape_i].m_filter.categoryBits;
	unsigned short maskBitsA = shapeListBuffer[shape_i].m_filter.maskBits;
	short groupIndexA = shapeListBuffer[shape_i].m_filter.groupIndex;

	for (int j=i+1; j<numObjects; j++)
	{
		shape_j = indicesAABB[j];
		//aabb_j = aabbs[shape_j];
		if(aabb_i.m_min[axis] > (aabbs[shape_j].m_max[axis])) 
		{
			break;
		}

		body_j = shapeToBodyMap[shape_j];
		type_j = aabbs[shape_j].m_bType;

		if (body_i==body_j)
			continue;

		bool bIsIntersection = TestAabbAgainstAabb2GlobalGlobal(&aabb_i, &aabbs[shape_j]) && (type_i==2 || type_j==2);

		// check joint collide connected
		bIsIntersection &= ShouldCollideJoints(body_i, body_j, bodyStaticListBuffer[body_i].m_connectedBodyIndices, bodyStaticListBuffer[body_j].m_connectedBodyIndices);

		// check user filtering
		unsigned short categoryBitsB = shapeListBuffer[shape_j].m_filter.categoryBits;
		unsigned short maskBitsB = shapeListBuffer[shape_j].m_filter.maskBits;
		short groupIndexB = shapeListBuffer[shape_j].m_filter.groupIndex;
		bIsIntersection &= ShouldCollide(categoryBitsA, maskBitsA, groupIndexA, categoryBitsB, maskBitsB, groupIndexB);

        int4 currentPairIndices;
        int c_type;
        int curPair, curTotalPair;
		if (bIsIntersection)
		{
			//printf("i: %d, shape_i: %d, shape_j: %d\n", i, shape_i, shape_j);
			//printf("\tAABB[%d]: (%f, %f)-(%f, %f)\n", shape_i, aabbs[shape_i].m_min[0], aabbs[shape_i].m_min[1], aabbs[shape_i].m_max[0], aabbs[shape_i].m_max[1]);
			//printf("\tAABB[%d]: (%f, %f)-(%f, %f)\n", shape_j, aabbs[shape_j].m_min[0], aabbs[shape_j].m_min[1], aabbs[shape_j].m_max[0], aabbs[shape_j].m_max[1]);

			currentPairIndices.x = shape_i;
			currentPairIndices.y = shape_j;
			currentPairIndices.z = body_i;
			currentPairIndices.w = body_j;

			int s_type_i = aabb_i.m_sType;
			int s_type_j = aabbs[shape_j].m_sType;

			// Set c_type according to s_type_i and s_type_j
			// Suppose only have circle (0), edge (1), and polygon (2) at this time
			// 0: circle-circle
			// 1: circle-polygon (A is polygon and B is circle)
			// 2: polygon-polygon
			// 3: edge-circle (A is edge and B is circle)
			// 4: edge-polygon (A is edge and B is polygon)
			// note that edge-edge is NOT supported in Box2D
			if (s_type_i==0) // A is circle
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 0;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 3;
					// swap the two shpaes to make sure A is edge and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 1;
					// swap the two shpaes to make sure A is polygon and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
			}
			else if (s_type_i==1) // A is edge
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 3;
				}
				else if (s_type_j==1) // B is edge
				{
					// This should never happen
				}
				else // B is polygon
				{
					c_type = 4;
				}
			}
			else // A is polygon
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 1;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 4;
					// swap the two shpaes to make sure A is edge and B is polygon
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 2;
				}
			}

			// the following two atomic_inc may have some problem?
			// check it later!!!
			curPair = atomic_inc(pairCounts+c_type);
			curTotalPair = atomic_inc(pairTotalCount);
			if (curTotalPair<maxPairs)
			{
				globalIndices[curTotalPair] = currentPairIndices; //flush to main memory
				pairIndices[maxPairs*c_type+curPair] = curTotalPair; //flush to main memory
			}
			//printf("pairCount: %d, shape_i: %d, shape_j: %d, body_i: %d, body_j: %d\n", curPair, shape_i, shape_j, body_i, body_j);
		}
	}
	//int curPair = atomic_add(pairCount, localPairCount);
	//if (curPair+localPairCount<maxPairs)
	//{
	//	for (int k=0; k<localPairCount; k++)
	//	{
	//		pairIndices[curPair+k] = localPairIndicesBuffer[k]; //flush to main memory
	//	}
	//}
}

__kernel void ComputePairsNoAtomic( 
		const __global b2clAABB* aabbs, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		__global int* pairIndices, // output
		__global int* pairIndicesBinaryBits, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		int numObjects, 
		int axis, 
		int maxPairsPerFixture,
		int maxPairs)
{
	int i = get_global_id(0);

	if (i>=numObjects)
		return;

	int base_address = i*maxPairsPerFixture;

	int shape_i, shape_j, body_i, body_j, type_i, type_j;
	shape_i = indicesAABB[i];
	body_i = shapeToBodyMap[shape_i];
	type_i = aabbs[shape_i].m_bType;

//	int localPairCount = 0;
//	
//#define max_pair_per_shape 10
//	int4 localPairIndicesBuffer[max_pair_per_shape]; // assume there will be less than max_pair_per_shape pair for each shape

	unsigned short categoryBitsA = shapeListBuffer[shape_i].m_filter.categoryBits;
	unsigned short maskBitsA = shapeListBuffer[shape_i].m_filter.maskBits;
	short groupIndexA = shapeListBuffer[shape_i].m_filter.groupIndex;

	int num_pairs = 0;
	// currently only support 5 types of contacts
	int type_counts[5];
	for (int k=0; k<5; k++)
		type_counts[k] = 0;

	for (int j=i+1; j<numObjects; j++)
	{
		shape_j = indicesAABB[j];
		if(aabbs[shape_i].m_min[axis] > (aabbs[shape_j].m_max[axis])) 
		{
			break;
		}

		body_j = shapeToBodyMap[shape_j];
		type_j = aabbs[shape_j].m_bType;

		if (body_i==body_j)
			continue;

		bool bIsIntersection = 1;//TestAabbAgainstAabb2GlobalGlobal(&aabbs[shape_i], &aabbs[shape_j]) && (type_i==2 || type_j==2);

		// check joint collide connected
		bIsIntersection &= ShouldCollideJoints(body_i, body_j, bodyStaticListBuffer[body_i].m_connectedBodyIndices, bodyStaticListBuffer[body_j].m_connectedBodyIndices);

		// check user filtering
		unsigned short categoryBitsB = shapeListBuffer[shape_j].m_filter.categoryBits;
		unsigned short maskBitsB = shapeListBuffer[shape_j].m_filter.maskBits;
		short groupIndexB = shapeListBuffer[shape_j].m_filter.groupIndex;
		bIsIntersection &= ShouldCollide(categoryBitsA, maskBitsA, groupIndexA, categoryBitsB, maskBitsB, groupIndexB);

        int4 currentPairIndices;
        int c_type;
		if (bIsIntersection)
		{
			currentPairIndices.x = shape_i;
			currentPairIndices.y = shape_j;
			currentPairIndices.z = body_i;
			currentPairIndices.w = body_j;

			int s_type_i = aabbs[shape_i].m_sType;
			int s_type_j = aabbs[shape_j].m_sType;

			// Set c_type according to s_type_i and s_type_j
			// Suppose only have circle (0), edge (1), and polygon (2) at this time
			// 0: circle-circle
			// 1: circle-polygon (A is polygon and B is circle)
			// 2: polygon-polygon
			// 3: edge-circle (A is edge and B is circle)
			// 4: edge-polygon (A is edge and B is polygon)
			// note that edge-edge is NOT supported in Box2D
			if (s_type_i==0) // A is circle
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 0;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 3;
					// swap the two shpaes to make sure A is edge and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 1;
					// swap the two shpaes to make sure A is polygon and B is circle
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
			}
			else if (s_type_i==1) // A is edge
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 3;
				}
				else if (s_type_j==1) // B is edge
				{
					// This should never happen
				}
				else // B is polygon
				{
					c_type = 4;
				}
			}
			else // A is polygon
			{
				if (s_type_j==0) // B is circle
				{
					c_type = 1;
				}
				else if (s_type_j==1) // B is edge
				{
					c_type = 4;
					// swap the two shpaes to make sure A is edge and B is polygon
					currentPairIndices.x = shape_j;
					currentPairIndices.y = shape_i;
					currentPairIndices.z = body_j;
					currentPairIndices.w = body_i;
				}
				else // B is polygon
				{
					c_type = 2;
				}
			}

			if (num_pairs<maxPairs)
			{
				globalIndices[base_address + num_pairs] = currentPairIndices;
				pairIndices[maxPairs*c_type + base_address + type_counts[c_type]] = base_address + num_pairs;
				pairIndicesBinaryBits[maxPairs*c_type + base_address + type_counts[c_type]] = 1;
				num_pairs++;
				type_counts[c_type]++;
			}
		}
	}
}

#define WORKGROUP_SIZE 256
//#define WORKGROUP_SIZE 1024

__kernel void computePairsLocalMemory( 
		__global const b2clAABB* aabbs, 
		const __global b2clBodyStatic* bodyStaticListBuffer, // only need body type here, should extract it later
		const __global b2clPolygonShape* shapeListBuffer,
		__global int4* globalIndices, // output
		__global int4* pairIndices, // output
		const __global uint* indicesAABB,
		const __global int* shapeToBodyMap,
		volatile __global int* pairCount, // output
		volatile __global int* pairTotalCount, // output
		int numObjects, 
		int axis, 
		int maxPairs)
{
	int i = get_global_id(0);

	//if (i>=numObjects)
	//	return;

	int localId = get_local_id(0);

	__local int numActiveWgItems[1];
	__local int breakRequest[1];
	__local int localShapeI[WORKGROUP_SIZE*2]; // = indicesAABB[i];
	__local b2clAABB localAabbs[WORKGROUP_SIZE*2]; // = aabbs[shape_i];
	__local int localBodyI[WORKGROUP_SIZE*2]; // = shapeToBodyMap[shape_i];
	__local int localBodyType[WORKGROUP_SIZE*2]; // = bodyStaticListBuffer[shapeToBodyMap[shape_i]].m_type;

	if (localId==0)
	{
		numActiveWgItems[0] = 0;
		breakRequest[0] = 0;
	}
	int localCount=0;
	int block=0;

	int shape_0, shape_i, body_i, shape_i2, body_i2;
	b2clAABB myAabb;
	int myShapeI, myBodyI, myType;

	// load shape_i
	shape_0 = indicesAABB[0];
	shape_i = i<numObjects ? indicesAABB[i] : shape_0;
	myShapeI = shape_i;
	localShapeI[localId] = myShapeI;
	shape_i2 = (i+WORKGROUP_SIZE)<numObjects ? indicesAABB[i+WORKGROUP_SIZE]: shape_0;
	localShapeI[localId+WORKGROUP_SIZE] = shape_i2;

	// load AABB
	myAabb = aabbs[shape_i];
	float testValue = 	myAabb.m_max[axis];
	localAabbs[localId] = myAabb;
	localAabbs[localId+WORKGROUP_SIZE] = aabbs[shape_i2];

	// load body_i
	body_i = shapeToBodyMap[shape_i];
	myBodyI = body_i;
	localBodyI[localId] = myBodyI;
	body_i2 = shapeToBodyMap[shape_i2];
	localBodyI[localId+WORKGROUP_SIZE] = body_i2;

	// load type_i
	myType = bodyStaticListBuffer[body_i].m_type;
	localBodyType[localId] = myType;
	localBodyType[localId+WORKGROUP_SIZE] = bodyStaticListBuffer[body_i2].m_type;

	barrier(CLK_LOCAL_MEM_FENCE);
	atomic_inc(numActiveWgItems);
	barrier(CLK_LOCAL_MEM_FENCE);
	int localBreak = 0;
	
	int j=i+1;
	do
	{
		barrier(CLK_LOCAL_MEM_FENCE);
	
		if (j<numObjects)
		{
	  		if(testValue < (localAabbs[localCount+localId+1].m_min[axis])) 
			{
				if (!localBreak)
				{
					atomic_inc(breakRequest);
					localBreak = 1;
				}
			}
		}
		
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (j>=numObjects && !localBreak)
		{
			atomic_inc(breakRequest);
			localBreak = 1;
		}
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if (!localBreak)
		{
			bool bIsIntersection = TestAabbAgainstAabb2(&myAabb,&localAabbs[localCount+localId+1]) &&
				(myType!=0 || localBodyType[localCount+localId+1]!=0);
			if (bIsIntersection)
			{
				int4 currentPairIndices;
				currentPairIndices.x = myShapeI;
				currentPairIndices.y = localShapeI[localCount+localId+1];
				currentPairIndices.z = myBodyI;
				currentPairIndices.w = localBodyI[localCount+localId+1];
				int curPair = atomic_inc (pairCount);
				if (curPair<maxPairs)
				{
						pairIndices[curPair] = currentPairIndices; //flush to main memory
				}
			}
		}
		
		barrier(CLK_LOCAL_MEM_FENCE);

		localCount++;
		if (localCount==WORKGROUP_SIZE)
		{
			localCount = 0;
			block+=WORKGROUP_SIZE;			

			// load shape_i
			shape_i = (i+block)<numObjects ? indicesAABB[i+block] : shape_0;
			localShapeI[localId] = shape_i;
			shape_i2 = (i+block+WORKGROUP_SIZE)<numObjects ? indicesAABB[i+block+WORKGROUP_SIZE]: shape_0;
			localShapeI[localId+WORKGROUP_SIZE] = shape_i2;

			// load AABB
			localAabbs[localId] = aabbs[shape_i];
			localAabbs[localId+WORKGROUP_SIZE] = aabbs[shape_i2];

			// load body_i
			body_i = shapeToBodyMap[shape_i];
			localBodyI[localId] = body_i;
			body_i2 = shapeToBodyMap[shape_i2];
			localBodyI[localId+WORKGROUP_SIZE] = body_i2;

			// load type_i
			localBodyType[localId] = bodyStaticListBuffer[body_i].m_type;
			localBodyType[localId+WORKGROUP_SIZE] = bodyStaticListBuffer[body_i2].m_type;
		}
		j++;
		
	} while (breakRequest[0]<numActiveWgItems[0]);
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ComputeAABBIntersection
//
// Kernel for AABBQuery user callback.
// Check intersections between the input AABB and existing AABBs.
// Indices of intersecting AABBs are stored in intersectingShapeIndices.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void ComputeAABBIntersection( 
		float4 inputAabb,
		const __global b2clAABB* aabbs, 
		const __global uint* indicesAABB,
		int numObjects,
		volatile __global int* intersectionCounts, // for each shape type
		volatile __global int* intersectionTotalCount,
		__global unsigned int* intersectingShapeIndices,
		__global unsigned int* intersectingshapeTypes)
{
	int i = get_global_id(0);

	if (i >= numObjects)
		return;

	unsigned int shape_i = indicesAABB[i];

	b2clAABB aabb;
	aabb.m_min[0] = inputAabb.x;
	aabb.m_min[1] = inputAabb.y;
	aabb.m_max[0] = inputAabb.z;
	aabb.m_max[1] = inputAabb.w;

	if(aabbs[shape_i].m_min[0] > aabb.m_max[0] || aabbs[shape_i].m_max[0] < aabb.m_min[0] ||
	   aabbs[shape_i].m_min[1] > aabb.m_max[1] || aabbs[shape_i].m_max[1] < aabb.m_min[1])
	   return;

	unsigned int s_type_i = aabbs[shape_i].m_sType;

	int curIntersection = atomic_inc(intersectionCounts + s_type_i);
	int curTotalIntersection = atomic_inc(intersectionTotalCount);

	intersectingShapeIndices[curTotalIntersection] = shape_i; 
	intersectingshapeTypes[curTotalIntersection] = s_type_i;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayCircleIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and circle shapes.
// The algorithm is from b2CircleShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayCircleIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	float2 position = b2clMul_Transform(&xf, shape.m_centroid);
	float2 s = (float2)(ray.x, ray.y) - position;
	float b = b2clDot(s, s) - shape.m_radius * shape.m_radius;

	// Solve quadratic equation.
	float2 r = (float2)(ray.z, ray.w) - (float2)(ray.x, ray.y);
	float c =  b2clDot(s, r);
	float rr = b2clDot(r, r);
	float sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0f || rr < b2_epsilon)
	{
		output->isCollide = 0;
		return;
	}

	// Find the point of intersection of the line with the circle.
	float a = -(c + sqrt(sigma));

	// Is the intersection point on the segment?
	if (0.0f <= a && a <= 1.0f * rr)
	{
		a /= rr;
		output->fraction = a;
		float2 normal = normalize(s + a * r);
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
		output->isCollide = 1;
	}
	else
	{
		output->isCollide = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayEdgeIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and edge shapes.
// The algorithm is from b2EdgeShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayEdgeIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	// Put the ray into the edge's frame of reference.
	float2 p1 = b2clMulT_Rotate(xf.q, (float2)(ray.x, ray.y) - xf.p);
	float2 p2 = b2clMulT_Rotate(xf.q, (float2)(ray.z, ray.w) - xf.p);
	float2 d = p2 - p1;

	float2 v1 = shape.m_vertices[0];
	float2 v2 = shape.m_vertices[1];
	float2 e = v2 - v1;
	float2 normal = normalize((float2)(e.y, -e.x));

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float numerator = b2clDot(normal, v1 - p1);
	float denominator = b2clDot(normal, d);

	if (denominator == 0.0f)
	{
		output->isCollide = 0;
		return;
	}

	float t = numerator / denominator;
	if (t < 0.0f || 1.0f < t)
	{
		output->isCollide = 0;
		return;
	}

	float2 q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	float2 r = v2 - v1;
	float rr = b2clDot(r, r);
	if (rr == 0.0f)
	{
		output->isCollide = 0;
		return;
	}

	float s = b2clDot(q - v1, r) / rr;
	if (s < 0.0f || 1.0f < s)
	{
		output->isCollide = 0;
		return;
	}

	output->fraction = t;
	if (numerator > 0.0f)
	{
		output->normal[0] = -normal.x;
		output->normal[1] = -normal.y;
	}
	else
	{
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
	}
	output->isCollide = 1;
	
	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayPolygonIntersection
//
// Kernel for RayCast user callback.
// Check intersection between the input ray and polygon shapes.
// The algorithm is from b2PolygonShape::RayCast()
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayPolygonIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	unsigned int shapeIndex = shapeIndices[i];
	const b2clPolygonShape shape = shapeListBuffer[shapeIndex];
	unsigned int bodyIndex = shapeToBodyMap[shapeIndex];
	const b2clTransform xf = xfGlobal[bodyIndex];

	output->shapeIndex = shapeIndex;

	// Put the ray into the polygon's frame of reference.
	float2 p1 = b2clMulT_Rotate(xf.q, (float2)(ray.x, ray.y) - xf.p);
	float2 p2 = b2clMulT_Rotate(xf.q, (float2)(ray.z, ray.w) - xf.p);
	float2 d = p2 - p1;

	float lower = 0.0f, upper = 1.0f;

	int index = -1;

	for (int i = 0; i < shape.m_vertexCount; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2clDot(shape.m_normals[i], shape.m_vertices[i] - p1);
		float denominator = b2clDot(shape.m_normals[i], d);

		if (denominator == 0.0f)
		{	
			if (numerator < 0.0f)
			{
				output->isCollide = 0;
				return;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			output->isCollide = 0;
			return;
		}
	}

	//b2Assert(0.0f <= lower && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		float2 normal = b2clMul_Rotate(xf.q, shape.m_normals[index]);
		output->normal[0] = normal.x;
		output->normal[1] = normal.y;
		output->isCollide = 1;
	}
	else
	{
		output->isCollide = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RayChainIntersection
//
// Kernel for RayCast user callback.
// But this function is never called and edge shape is used for chains.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void RayChainIntersection( 
		float4 ray,
		const __global b2clPolygonShape* shapeListBuffer,
		const __global unsigned int* shapeIndices,
		const __global b2clTransform* xfGlobal,
		const __global int* shapeToBodyMap,
		int offset,
		int numObjects,
		__global b2clRayCastOutput* outputBuffer)
{
	int i = get_global_id(0) + offset;

	if (i >= offset + numObjects)
		return;

	__global b2clRayCastOutput* output = outputBuffer + i;

	output->isCollide = 0;
}