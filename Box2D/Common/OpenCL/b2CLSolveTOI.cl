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
//#include <Box2D/Common/OpenCL/b2CLNarrowPhase.cl>
#if 0
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

enum SeparationType
{
	e_points,
	e_faceA,
	e_faceB
};

typedef struct
{
	const b2clDistanceProxy* m_proxyA;
	const b2clDistanceProxy* m_proxyB;
	b2clSweep m_sweepA, m_sweepB;
	int m_type;
	float2 m_localPoint;
	float2 m_axis;
} b2clSeparationFunction;

float b2clSeparationFunctionInitialize(b2clSeparationFunction* sf, const b2clSimplexCache* cache,
	const b2clDistanceProxy* proxyA, const b2clSweep* sweepA,
	const b2clDistanceProxy* proxyB, const b2clSweep* sweepB,
	float t1)
{
	sf->m_proxyA = proxyA;
	sf->m_proxyB = proxyB;
	int count = cache->count;

	sf->m_sweepA = *sweepA;
	sf->m_sweepB = *sweepB;

	b2clTransform xfA, xfB;
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t1);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t1);

	//printf ("cache->indexA: %d cache->indexB: %d \n ", cache->indexA[0], cache->indexA[1]);
	//printf ("cache->indexA: %d \n ", );
	if (count == 1)
	{
		sf->m_type = e_points;
		float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[0]);
		float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, cache->indexB[0]);
		float2 pointA = b2clMul_Transform(&xfA, localPointA);
		float2 pointB = b2clMul_Transform(&xfB, localPointB);
		sf->m_axis = pointB - pointA;
		float s = b2clNormalize(&sf->m_axis);
		return s;
	}
	else if (cache->indexA[0] == cache->indexA[1])
	{
		// Two points on B and one on A.
		sf->m_type = e_faceB;
		float2 localPointB1 = b2clDistanceProxyGetVertex(proxyB, cache->indexB[0]);
		float2 localPointB2 = b2clDistanceProxyGetVertex(proxyB, cache->indexB[1]);

		sf->m_axis = b2clCross_VS(localPointB2 - localPointB1, 1.0f);
		sf->m_axis = normalize(sf->m_axis);
		float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);

		sf->m_localPoint = 0.5f * (localPointB1 + localPointB2);
		float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);

		float2 localPointA = b2clDistanceProxyGetVertex(proxyA, cache->indexA[0]);
		float2 pointA = b2clMul_Transform(&xfA, localPointA);

		float s = b2clDot(pointA - pointB, normal);
		if (s < 0.0f)
		{
			sf->m_axis = -sf->m_axis;
			s = -s;
		}
		return s;
	}
	else
	{
		// Two points on A and one or two points on B.
		sf->m_type = e_faceA;
		float2 localPointA1 = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[0]);
		float2 localPointA2 = b2clDistanceProxyGetVertex(sf->m_proxyA, cache->indexA[1]);
			
		sf->m_axis = b2clCross_VS(localPointA2 - localPointA1, 1.0f);
		sf->m_axis = normalize(sf->m_axis);
		float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);

		sf->m_localPoint = 0.5f * (localPointA1 + localPointA2);
		float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

		float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, cache->indexB[0]);
		float2 pointB = b2clMul_Transform(&xfB, localPointB);

		float s = b2clDot(pointB - pointA, normal);
		if (s < 0.0f)
		{
			sf->m_axis = -sf->m_axis;
			s = -s;
		}
		return s;
	}
}

float b2clSeparationFunctionFindMinSeparation(b2clSeparationFunction* sf, int* indexA, int* indexB, float t, int numLoops)
{
	b2clTransform xfA, xfB;
	
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t);

	
	switch (sf->m_type)
	{
	case e_points:
		{
			float2 axisA = b2clMulT_Rotate(xfA.q,  sf->m_axis);
			float2 axisB = b2clMulT_Rotate(xfB.q, -sf->m_axis);

			*indexA = b2clDistanceProxyGetSupport(sf->m_proxyA, axisA);
			*indexB = b2clDistanceProxyGetSupport(sf->m_proxyB, axisB);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, *indexA);
			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, *indexB);
				
			float2 pointA = b2clMul_Transform(&xfA, localPointA);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, sf->m_axis);
			return separation;
		}

	case e_faceA:
		{
		   // if (numLoops == 36) {
		//		printf ("got in numLoops == 36 \n"); 
		//	}
		 //   if (numLoops == 36) return ; 
			float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);
			float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

			float2 axisB = b2clMulT_Rotate(xfB.q, -normal);
				
			*indexA = -1;
			*indexB = b2clDistanceProxyGetSupport(sf->m_proxyB, axisB);

			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, *indexB);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, normal);
			return separation;
		}

	case e_faceB:
		{
		    
			float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);
		//	if (numLoops == 36) {
		//	printf ("sf->m_localPoint for 36: %f \n", sf->m_localPoint.y); 
		//	}
		//	if (numLoops == 36) return ; 
			float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);
			
			float2 axisA = b2clMulT_Rotate(xfA.q, -normal);

			*indexB = -1;
			*indexA = b2clDistanceProxyGetSupport(sf->m_proxyA, axisA);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, *indexA);
			float2 pointA = b2clMul_Transform(&xfA, localPointA);

			float separation = b2clDot(pointA - pointB, normal);
			return separation;
		}

	default:
		*indexA = -1;
		*indexB = -1;
		return 0.0f;
	}
}

float b2clSeparationFunctionEvaluate(b2clSeparationFunction* sf, int indexA, int indexB, float t)
{
	b2clTransform xfA, xfB;
	b2clSweepGetTransform(&sf->m_sweepA, &xfA, t);
	b2clSweepGetTransform(&sf->m_sweepB, &xfB, t);

	switch (sf->m_type)
	{
	case e_points:
		{
			float2 axisA = b2clMulT_Rotate(xfA.q,  sf->m_axis);
			float2 axisB = b2clMulT_Rotate(xfB.q, -sf->m_axis);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, indexA);
			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, indexB);

			float2 pointA = b2clMul_Transform(&xfA, localPointA);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);
			float separation = b2clDot(pointB - pointA, sf->m_axis);

			return separation;
		}

	case e_faceA:
		{
			float2 normal = b2clMul_Rotate(xfA.q, sf->m_axis);
			float2 pointA = b2clMul_Transform(&xfA, sf->m_localPoint);

			float2 axisB = b2clMulT_Rotate(xfB.q, -normal);

			float2 localPointB = b2clDistanceProxyGetVertex(sf->m_proxyB, indexB);
			float2 pointB = b2clMul_Transform(&xfB, localPointB);

			float separation = b2clDot(pointB - pointA, normal);
			return separation;
		}

	case e_faceB:
		{
			float2 normal = b2clMul_Rotate(xfB.q, sf->m_axis);
			float2 pointB = b2clMul_Transform(&xfB, sf->m_localPoint);

			float2 axisA = b2clMulT_Rotate(xfA.q, -normal);

			float2 localPointA = b2clDistanceProxyGetVertex(sf->m_proxyA, indexA);
			float2 pointA = b2clMul_Transform(&xfA, localPointA);

			float separation = b2clDot(pointA - pointB, normal);
			return separation;
		}

	default:
		return 0.0f;
	}
}

float SimplexGetMetric(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 0:
		return 0.0;

	case 1:
		return 0.0f;

	case 2:
		return b2clDistance(simplex->m_v1.w, simplex->m_v2.w);

	case 3:
		return b2clCross_VV(simplex->m_v2.w - simplex->m_v1.w, simplex->m_v3.w - simplex->m_v1.w);

	default:
		return 0.0f;
	}
}

void SimplexReadCache(b2clSimplex* simplex, const b2clSimplexCache* cache,
					const b2clDistanceProxy* proxyA, const b2clTransform* transformA,
					const b2clDistanceProxy* proxyB, const b2clTransform* transformB)
{
		
	// Copy data from cache.
	simplex->m_count = cache->count;
	b2clSimplexVertex* vertices = &simplex->m_v1;
	for (int i = 0; i < simplex->m_count; ++i)
	{
		b2clSimplexVertex* v = vertices + i;
		v->indexA = cache->indexA[i];
		v->indexB = cache->indexB[i];
		float2 wALocal = proxyA->m_vertices[v->indexA];
		float2 wBLocal = proxyB->m_vertices[v->indexB];
		v->wA = b2clMul_Transform(transformA, wALocal);
		v->wB = b2clMul_Transform(transformB, wBLocal);
		v->w = v->wB - v->wA;
		v->a = 0.0f;
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if (simplex->m_count > 1)
	{
		float metric1 = cache->metric;
		float metric2 = SimplexGetMetric(simplex);
		if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < b2_epsilon)
		{
			// Reset the simplex.
			simplex->m_count = 0;
		}
	}

	// If the cache is empty or invalid ...
	if (simplex->m_count == 0)
	{
		b2clSimplexVertex* v = vertices + 0;
		v->indexA = 0;
		v->indexB = 0;
		float2 wALocal = proxyA->m_vertices[0];
		float2 wBLocal = proxyB->m_vertices[0];
		v->wA = b2clMul_Transform(transformA, wALocal);
		v->wB = b2clMul_Transform(transformB, wBLocal);
		v->w = v->wB - v->wA;
		simplex->m_count = 1;
	}
}

void SimplexWriteCache(b2clSimplex* simplex, b2clSimplexCache* cache)
{
	cache->metric = SimplexGetMetric(simplex);
	cache->count = (short)(simplex->m_count);
	const b2clSimplexVertex* vertices = &simplex->m_v1;
	for (int i = 0; i < simplex->m_count; ++i)
	{
		cache->indexA[i] = (unsigned char)(vertices[i].indexA);
		cache->indexB[i] = (unsigned char)(vertices[i].indexB);
	}
}

float2 SimplexGetSearchDirection(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 1:
		return -simplex->m_v1.w;

	case 2:
		{
			float2 e12 = simplex->m_v2.w - simplex->m_v1.w;
			float sgn = b2clCross_VV(e12, -simplex->m_v1.w);
			if (sgn > 0.0f)
			{
				// Origin is left of e12.
				return b2clCross_SV(1.0f, e12);
			}
			else
			{
				// Origin is right of e12.
				return b2clCross_VS(e12, 1.0f);
			}
		}

	default:
		return (float2)(0.0f, 0.0f);
	}
}

float2 SimplexGetClosestPoint(b2clSimplex* simplex)
{
	switch (simplex->m_count)
	{
	case 0:
		return (float2)(0.0f, 0.0f);

	case 1:
		return simplex->m_v1.w;

	case 2:
		return simplex->m_v1.a * simplex->m_v1.w + simplex->m_v2.a * simplex->m_v2.w;

	case 3:
		return (float2)(0.0f, 0.0f);

	default:
		return (float2)(0.0f, 0.0f);
	}
}

void SimplexGetWitnessPoints(b2clSimplex* simplex, float2* pA, float2* pB)
{
	switch (simplex->m_count)
	{
	case 0:
		break;

	case 1:
		*pA = simplex->m_v1.wA;
		*pB = simplex->m_v1.wB;
		break;

	case 2:
		*pA = simplex->m_v1.a * simplex->m_v1.wA + simplex->m_v2.a * simplex->m_v2.wA;
		*pB = simplex->m_v1.a * simplex->m_v1.wB + simplex->m_v2.a * simplex->m_v2.wB;
		break;

	case 3:
		*pA = simplex->m_v1.a * simplex->m_v1.wA + simplex->m_v2.a * simplex->m_v2.wA + simplex->m_v3.a * simplex->m_v3.wA;
		*pB = *pA;
		break;

	default:
		break;
	}
}

void SimplexSolve2(b2clSimplex* simplex)
{
	float2 w1 = simplex->m_v1.w;
	float2 w2 = simplex->m_v2.w;
	float2 e12 = w2 - w1;

	// w1 region
	float d12_2 = -b2clDot(w1, e12);
	if (d12_2 <= 0.0f)
	{
		// a2 <= 0, so we clamp it to 0
		simplex->m_v1.a = 1.0f;
		simplex->m_count = 1;
		return;
	}

	// w2 region
	float d12_1 = b2clDot(w2, e12);
	if (d12_1 <= 0.0f)
	{
		// a1 <= 0, so we clamp it to 0
		simplex->m_v2.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v2;
		return;
	}

	// Must be in e12 region.
	float inv_d12 = 1.0f / (d12_1 + d12_2);
	simplex->m_v1.a = d12_1 * inv_d12;
	simplex->m_v2.a = d12_2 * inv_d12;
	simplex->m_count = 2;
}

void SimplexSolve3(b2clSimplex* simplex)
{
	float2 w1 = simplex->m_v1.w;
	float2 w2 = simplex->m_v2.w;
	float2 w3 = simplex->m_v3.w;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	float2 e12 = w2 - w1;
	float w1e12 = b2clDot(w1, e12);
	float w2e12 = b2clDot(w2, e12);
	float d12_1 = w2e12;
	float d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	float2 e13 = w3 - w1;
	float w1e13 = b2clDot(w1, e13);
	float w3e13 = b2clDot(w3, e13);
	float d13_1 = w3e13;
	float d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	float2 e23 = w3 - w2;
	float w2e23 = b2clDot(w2, e23);
	float w3e23 = b2clDot(w3, e23);
	float d23_1 = w3e23;
	float d23_2 = -w2e23;
	
	// Triangle123
	float n123 = b2clCross_VV(e12, e13);

	float d123_1 = n123 * b2clCross_VV(w2, w3);
	float d123_2 = n123 * b2clCross_VV(w3, w1);
	float d123_3 = n123 * b2clCross_VV(w1, w2);

	// w1 region
	if (d12_2 <= 0.0f && d13_2 <= 0.0f)
	{
		simplex->m_v1.a = 1.0f;
		simplex->m_count = 1;
		return;
	}

	// e12
	if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
	{
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		simplex->m_v1.a = d12_1 * inv_d12;
		simplex->m_v2.a = d12_2 * inv_d12;
		simplex->m_count = 2;
		return;
	}

	// e13
	if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
	{
		float inv_d13 = 1.0f / (d13_1 + d13_2);
		simplex->m_v1.a = d13_1 * inv_d13;
		simplex->m_v3.a = d13_2 * inv_d13;
		simplex->m_count = 2;
		simplex->m_v2 = simplex->m_v3;
		return;
	}

	// w2 region
	if (d12_1 <= 0.0f && d23_2 <= 0.0f)
	{
		simplex->m_v2.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v2;
		return;
	}

	// w3 region
	if (d13_1 <= 0.0f && d23_1 <= 0.0f)
	{
		simplex->m_v3.a = 1.0f;
		simplex->m_count = 1;
		simplex->m_v1 = simplex->m_v3;
		return;
	}

	// e23
	if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
	{
		float inv_d23 = 1.0f / (d23_1 + d23_2);
		simplex->m_v2.a = d23_1 * inv_d23;
		simplex->m_v3.a = d23_2 * inv_d23;
		simplex->m_count = 2;
		simplex->m_v1 = simplex->m_v3;
		return;
	}

	// Must be in triangle123
	float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
	simplex->m_v1.a = d123_1 * inv_d123;
	simplex->m_v2.a = d123_2 * inv_d123;
	simplex->m_v3.a = d123_3 * inv_d123;
	simplex->m_count = 3;
}

// Compute the closest points between two shapes. Supports any combination of:
// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
// On the first call set b2SimplexCache.count to zero.
void b2clShapeDistance(b2clDistanceOutput* output, b2clSimplexCache* cache, const b2clDistanceInput* input)
{

    
	const b2clDistanceProxy* proxyA = &input->proxyA;
	const b2clDistanceProxy* proxyB = &input->proxyB;

	b2clTransform transformA = input->transformA;
	b2clTransform transformB = input->transformB;

	// Initialize the simplex.
	b2clSimplex simplex;
	SimplexReadCache(&simplex, cache, proxyA, &transformA, proxyB, &transformB);

	// Get simplex vertices as an array.
	b2clSimplexVertex* vertices = &simplex.m_v1;
	const int k_maxIters = 20;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	int saveA[3], saveB[3];
	int saveCount = 0;

	float2 closestPoint = SimplexGetClosestPoint(&simplex);
	float distanceSqr1 = b2clLengthSquared(closestPoint);
	float distanceSqr2 = distanceSqr1;

	// Main iteration loop.
	int iter = 0;
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (int i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		switch (simplex.m_count)
		{
		case 1:
			break;

		case 2:
			SimplexSolve2(&simplex);
			break;

		case 3:
			SimplexSolve3(&simplex);
			break;

		default:
			break;
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.m_count == 3)
		{
			break;
		}

		// Compute closest point.
		float2 p = SimplexGetClosestPoint(&simplex);
		distanceSqr2 = b2clLengthSquared(p);

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;

		// Get search direction.
		float2 d = SimplexGetSearchDirection(&simplex);

		// Ensure the search direction is numerically fit.
		if (b2clLengthSquared(d) < b2_epsilon * b2_epsilon)
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b2clSimplexVertex* vertex = vertices + simplex.m_count;
		vertex->indexA = b2clDistanceProxyGetSupport(proxyA, b2clMulT_Rotate(transformA.q, -d));
		vertex->wA = b2clMul_Transform(&transformA, (b2clDistanceProxyGetVertex(proxyA, vertex->indexA)));

		//float2 wBLocal;
		vertex->indexB = b2clDistanceProxyGetSupport(proxyB, b2clMulT_Rotate(transformB.q, d));
		vertex->wB = b2clMul_Transform(&transformB, (b2clDistanceProxyGetVertex(proxyB, vertex->indexB)));
		vertex->w = vertex->wB - vertex->wA;

		// Iteration count is equated to the number of support point calls.
		++iter;

		// Check for duplicate support points. This is the main termination criteria.
		bool duplicate = false;
		for (int i = 0; i < saveCount; ++i)
		{
			if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++simplex.m_count;
	}
	
	// Prepare output.
	SimplexGetWitnessPoints(&simplex, &output->pointA, &output->pointB);
	//float dis = b2clDistance(output->pointA, output->pointB);
	output->ndistance = b2clDistance(output->pointA, output->pointB);
	output->iterations = iter;
	
	// Cache the simplex.
	SimplexWriteCache(&simplex, cache);
	
	// Apply radii if requested.
	if (input->useRadii)
	{
		float rA = proxyA->m_radius;
		float rB = proxyB->m_radius;

		if (output->ndistance > rA + rB && output->ndistance > b2_epsilon)
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output->ndistance -= rA + rB;
			float2 normal = normalize(output->pointB - output->pointA);
			output->pointA += rA * normal;
			output->pointB -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			float2 p = 0.5f * (output->pointA + output->pointB);
			output->pointA = p;
			output->pointB = p;
			output->ndistance = 0.0f;
		}
	}
}

void b2clTimeOfImpact(b2clTOIOutput* output, const b2clTOIInput* input, int numLoops)
{
	output->state = e_unknown;
	output->t = input->tMax;

	b2clDistanceProxy proxyA = input->proxyA;
	b2clDistanceProxy proxyB = input->proxyB;

	b2clSweep sweepA = input->sweepA;
	b2clSweep sweepB = input->sweepB;

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	b2clSweepNormalize(&sweepA);
	b2clSweepNormalize(&sweepB);

	float tMax = input->tMax;

	float totalRadius = proxyA.m_radius + proxyB.m_radius;
	float target = max(b2_linearSlop, totalRadius - 3.0f * b2_linearSlop);
	float tolerance = 0.25f * b2_linearSlop;

	float t1 = 0.0f;
	const int k_maxIterations = 20;	// TODO_ERIN b2Settings
	int iter = 0;

	// Prepare input for distance query.
	b2clSimplexCache cache;
	cache.count = 0;
	b2clDistanceInput distanceInput;
	distanceInput.proxyA = proxyA;
	distanceInput.proxyB = proxyB;
	distanceInput.useRadii = false;


	

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	//for(;;)
	//Warning: I can not use for(;;). Otherwise, clBuildProgram will return error! 
	for (int i = 0 ; i < 100 ; i ++ )
	//while(1)
	//for (;;)
	{
		b2clTransform xfA, xfB;
		b2clSweepGetTransform(&sweepA, &xfA, t1);
		b2clSweepGetTransform(&sweepB, &xfB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2clDistanceOutput distanceOutput;

		b2clShapeDistance(&distanceOutput, &cache, &distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.ndistance <= 0.0f)
		{
			// Failure!
			output->state = e_overlapped;
			output->t = 0.0f;
			break;
		}

		if (distanceOutput.ndistance < target + tolerance)
		{
			// Victory!
			output->state = e_touching;
			output->t = t1;
			break;
		}

		// Initialize the separating axis.
		b2clSeparationFunction fcn;
		b2clSeparationFunctionInitialize(&fcn, &cache, &proxyA, &sweepA, &proxyB, &sweepB, t1);

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		bool done = false;
		float t2 = tMax;
		int pushBackIter = 0;
		
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			int indexA, indexB;
			//if (numLoops == 36 && ) return ; 
			float s2 = b2clSeparationFunctionFindMinSeparation(&fcn, &indexA, &indexB, t2, numLoops);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output->state = e_separated;
				output->t = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			float s1 = b2clSeparationFunctionEvaluate(&fcn, indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output->state = e_failed;
				output->t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output->state = e_touching;
				output->t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			int rootIterCount = 0;
			float a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float t;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = 0.5f * (a1 + a2);
				}

				float s = b2clSeparationFunctionEvaluate(&fcn, indexA, indexB, t);

				if (b2clAbs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				++rootIterCount;

				if (rootIterCount == 50)
				{
					break;
				}
			}

			++pushBackIter;

			if (pushBackIter == b2cl_maxPolygonVertices)
			{
				break;
			}
		}

		++iter;

		if (done)
		{
			break;
		}

		if (iter == k_maxIterations)
		{
			// Root finder got stuck. Semi-victory.
			output->state = e_failed;
			output->t = t1;
			break;
		}
	}
	//printf ("output.t: %f \n", output->t); 
}

__kernel void b2clComputeTOI(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	const __global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global unsigned int* contactTOIs,
	__global int* contactIndices,
	int resetAlpha0,
	__global int* toiTimesBuffer,
	int numLoops 
	)
{

 
	int i = get_global_id(0);

	if (i >= numContacts)
		return;

	float alpha = 1.0f;

	contactIndices[i] = i;
	contactTOIs[i] = FloatFlip(alpha);

	// TODO: Disabled contact checking
	/*
	// Is this contact disabled?
	if (c->IsEnabled() == false)
	{
		continue;
	}
	*/

	
	int4 currentIndices = globalIndices[i];

	b2clBodyStatic bA = bodyStaticListBuffer[currentIndices.z];
	b2clBodyStatic bB = bodyStaticListBuffer[currentIndices.w];

	b2clPolygonShape shapeA = polyGlobal[currentIndices.x];
	b2clPolygonShape shapeB = polyGlobal[currentIndices.y];

	// Is there a sensor?
	if (shapeA.m_bIsSensor || shapeB.m_bIsSensor)
		return;
   if (toiTimesBuffer[i] > 8 ) return ; 
	int typeA = bA.m_type;
	int typeB = bB.m_type;

	//printf ("Contact id: %d, TypeA: %d TypeB: %d \n", i, typeA, typeB);
	
	// Is at least one body active (awake and dynamic or kinematic)?
	if (typeA == 0 && typeB == 0) // 0 : b2_staticBody
	{
		return;
	}
	 
	bool collideA = bA.m_bIsBullet || typeA != 2; // 2 : b2_dynamicBody
	bool collideB = bB.m_bIsBullet || typeB != 2; // 2 : b2_dynamicBody

	// Are these two non-bullet dynamic bodies?
	if (collideA == false && collideB == false)
	{
		return;
	}

	b2clBodyDynamic bdA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bdB = bodyDynamicListBuffer[currentIndices.w];

	//printf ("Contact id: %d: Index A: %d , Index B: %d \n", i , currentIndices.z , currentIndices.w); 

	if (resetAlpha0)
	{	
		bdA.m_sweep.alpha0 = 0.0f;
		bdB.m_sweep.alpha0 = 0.0f;
	}

	// Compute the TOI for this contact.
	// Put the sweeps onto the same time interval.
	float alpha0 = bdA.m_sweep.alpha0;


	if (bdA.m_sweep.alpha0 < bdB.m_sweep.alpha0)
	{
		alpha0 = bdB.m_sweep.alpha0;
		b2clSweepAdvance(&bdA.m_sweep, alpha0);
	}
	else if (bdB.m_sweep.alpha0 < bdA.m_sweep.alpha0)
	{
		alpha0 = bdA.m_sweep.alpha0;
		b2clSweepAdvance(&bdB.m_sweep, alpha0);
	}


	// Compute the time of impact in interval [0, minTOI]
	b2clTOIInput input;
	b2clDistanceProxySet(&input.proxyA, &shapeA);
	b2clDistanceProxySet(&input.proxyB, &shapeB);
	input.sweepA = bdA.m_sweep;
	input.sweepB = bdB.m_sweep;
	input.tMax = 1.0f;

	float test1 = bdA.m_sweep.c.y ;
	float test2 = bdB.m_sweep.c.y ;  
	
	//if (numLoops == 36 && i == 0 ) return ; 

	b2clTOIOutput output;
	b2clTimeOfImpact(&output, &input , numLoops) ; 				

	
	// Beta is the fraction of the remaining portion of the .

	float beta = output.t;
		//printf ("beta %f, \n",  beta); 
	if (output.state == e_touching)
	{
		alpha = min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
		//printf ("MyAlpha %f \n", alpha); 
		//printf ("if alpha0 %f, \n",  alpha0); 
	}
	else
	{
		alpha = 1.0f;
	}
	//printf ("Contact id: %d: Index A: %d , alpha: %f \n", i , currentIndices.z , alpha); 
	 
	contactTOIs[i] = FloatFlip(alpha);
}





void b2clOnePairCollidePolygons(b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB){
/*
	manifold->pointCount = 0 ;
	// Not sure if I need them yet.  
	//manifold.points[0].normalImpulse = 0;
    //manifold.points[0].tangentImpulse = 0;
    //manifold.points[1].normalImpulse = 0;
    //manifold.points[1].tangentImpulse = 0;
	float totalRadius = polyA->m_radius + polyB->m_radius; 
	int edgeA = 0 ; 
	float separationA = b2clFindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > totalRadius) return ; 
	int edgeB = 0 ; 
	float separationB = b2clFindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > totalRadius) return;

	const b2clPolygonShape* poly1;	// reference polygon
	const b2clPolygonShape* poly2;	// incident polygon
	const b2clTransform *xf1;
	const b2clTransform *xf2;
	int edge1;		// reference edge
	uint flip;
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;
	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifold->type = 2;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifold->type = 1;
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
		return;
	}

	// Clip to negative box side 1
	np = b2clClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		return;
	}
    manifold->localNormal = localNormal;
	manifold->localPoint = planePoint;
	int pointCount = 0;
	for (int k = 0; k < b2cl_maxManifoldPoints; ++k)
	{
		float separation = dot(normal, clipPoints2[k].v) - frontOffset;

		if (separation <= totalRadius)
		{
			b2clManifoldPoint* cp = manifold->points + pointCount;
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

	manifold->pointCount = pointCount;
*/
}

void ContactEvaluate (b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB) {

    int contactType = manifold->type ; 
	switch (contactType)
			{
			case 0: // circle-circle
				//collideKernel = collideCirclesKernel;
				break;
			case 1: // circle-polygon
				//collideKernel = collidePolygonAndCircleKernel;
				break;
			case 2: // polygon-polygon
				b2clOnePairCollidePolygons ( manifold, polyA, xfA, polyB, xfB ) ; 
				break;
			case 3: // edge-circle
				//collideKernel = collideEdgeAndCircleKernel;
				break;
			case 4: // edge-polygon
				//collideKernel = collideEdgeAndPolygonKernel;
				break;
			default:
				//printf("Error! OpenCL: ContactEvaluate!\n");
				return; 
			}
}
void ContactUpdate  (b2clManifold* manifold, const b2clPolygonShape* polyA, const b2clTransform* xfA, const b2clPolygonShape* polyB, const b2clTransform* xfB) {
	b2clManifold oldManifold = *manifold; 
	// Re-enable this contact ; Do I need to do this?
	bool touching = false ; 
	// wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag; Do I need this? 
	//Not implemented for sensor
	//bool sensorA = m_fixtureA->IsSensor(); bool sensorB = m_fixtureB->IsSensor(); bool sensor = sensorA || sensorB; 
	//if (sensor){}
	ContactEvaluate ( manifold, polyA, xfA, polyB, xfB ) ;  
	touching = manifold->pointCount > 0 ; 
	for (int i = 0; i < manifold->pointCount; ++i)
	{
		b2clManifoldPoint* mp2 = manifold->points + i;
		mp2->normalImpulse = 0.0f;
		mp2->tangentImpulse = 0.0f;
		b2clContactID id2 = mp2->id;

		for (int j = 0; j < oldManifold.pointCount; ++j)
		{
			b2clManifoldPoint* mp1 = oldManifold.points + j;
			if (mp1->id.key == id2.key)
			{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
			}
		}
	}




}
void b2clBodySynchronizeTransform ( b2clTransform* xf, b2clSweep* sweep ) {
	float sina, cosa;
	sina = sincos(sweep->a, &cosa);
	xf->q.x = sina;
	xf->q.y = cosa;
	//xf->q.x = sin (sweep->a);
	//xf->q.y = cos (sweep->a);
	//xf->q.y = cos_wrapper(sweep->a);
	xf->p = sweep->c - b2clMul_Rotate (xf->q, sweep->localCenter); 
}


//void b2clAdvanceOneBody 

__kernel void b2clAdvanceBodiesKernel(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	__global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global b2clTransform* xfGlobal,
	__global b2clManifold* manifoldList ,  
	__global unsigned int* contactTOIs,
	__global int* contactIndices){

	int idxContact = get_global_id(0);

	// Currently, we only handle one contact.
	if (idxContact >= 1) return;
	idxContact = contactIndices[idxContact] ; 
	float minAlpha = IFloatFlip (contactTOIs[idxContact]);  
	int4 currentIndices = globalIndices[idxContact] ; 
	b2clBodyDynamic bA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bB = bodyDynamicListBuffer[currentIndices.w];
	b2clPolygonShape polyA = polyGlobal[currentIndices.z];
	b2clPolygonShape polyB = polyGlobal[currentIndices.w]; 
	b2clManifold manifold = manifoldList[idxContact]; 

	//b2clSweep backupSweepA = bA.m_sweep;
	//b2clSweep backupSweepB = bB.m_sweep; 
	b2clTransform xfA, xfB ; 

	b2clBodySweepAdvance (&bA, &xfA, minAlpha);
	b2clBodySweepAdvance (&bB, &xfB, minAlpha);

	ContactUpdate ( &manifold, &polyA, &xfA, &polyB, &xfB ) ; 
	if (manifold.pointCount == 0) {
		//bA.m_sweep = backupSweepA ; 
		//bB.m_sweep = backupSweepB ; 
		//b2clBodySynchronizeTransform ( &xfA, &bA.m_sweep) ; 
		//b2clBodySynchronizeTransform ( &xfB, &bB.m_sweep);
		return ; 
	}

    bodyDynamicListBuffer[currentIndices.z] = bA;
	bodyDynamicListBuffer[currentIndices.w] = bB;
	xfGlobal[currentIndices.z] = xfA ; 
	xfGlobal[currentIndices.w] = xfB; 

	//minContact->Update(m_contactManager.m_contactListener);

}




__kernel void b2clMarkConnectedContactsKernel(const __global int4* globalIndices,
	int numContacts,
	const __global b2clBodyStatic* bodyStaticListBuffer,
	__global b2clBodyDynamic* bodyDynamicListBuffer,
	const __global b2clPolygonShape* polyGlobal,
	__global b2clTransform* xfGlobal,
	__global b2clManifold* manifoldList ,  
	__global unsigned int* contactTOIs,
	__global int* contactIndices,
	__global int* isConnectedBuffer, 
	int seedIndex ) {

	int idxContact = get_global_id(0) ; 
	if (idxContact >= numContacts) return ; 
	idxContact = contactIndices[idxContact]; 
	seedIndex = contactIndices[seedIndex]; 
	if (idxContact == seedIndex) {isConnectedBuffer[idxContact] = 1 ; return ; }
	int4 seedIndices = globalIndices[seedIndex];
	int4 currentIndices = globalIndices[idxContact]; 

	bool findA = false; bool findB = false ; int idxShare, idxOther ; 
	if (seedIndices.z == currentIndices.z || seedIndices.z == currentIndices.w) findA = true ;  
	if (seedIndices.w == currentIndices.w || seedIndices.w == currentIndices.w) findB = true ;
	if (findA == false && findB == false ) return ; 
    if (findA == true) {idxShare = seedIndices.z; idxOther = (seedIndices.z == currentIndices.z)? currentIndices.w: currentIndices.z;}
	else {idxShare = seedIndices.w; idxOther = (seedIndices.w == currentIndices.w)? currentIndices.z:currentIndices.w; }

	// Has this contact already been added to the island? 
	if (isConnectedBuffer[idxContact] == 1) return ; 

	


	b2clBodyStatic staticShare = bodyStaticListBuffer[idxShare];
	b2clBodyStatic staticOther = bodyStaticListBuffer[idxOther];

	// Only add static, kinematic, or bullet bodies.
	// b2_dynamicbody = 2
	if (staticOther.m_type == 2 && staticShare.m_bIsBullet == 0 && staticOther.m_bIsBullet == 0 ) return ; 
	//To do: skip Sensors: 

	// Tentatively advance the body to the TOI.
	float minAlpha = IFloatFlip (contactTOIs[seedIndex]);  
	b2clBodyDynamic bA = bodyDynamicListBuffer[currentIndices.z];
	b2clBodyDynamic bB = bodyDynamicListBuffer[currentIndices.w];
	b2clTransform xfA = xfGlobal[currentIndices.z];
	b2clTransform xfB = xfGlobal[currentIndices.w];
	b2clPolygonShape polyA = polyGlobal[currentIndices.z];
	b2clPolygonShape polyB = polyGlobal[currentIndices.w]; 
	b2clManifold manifold = manifoldList[idxContact];
	
    b2clBodyDynamic* bodyOther; b2clTransform* xfOther;
	if (idxOther == currentIndices.z) {bodyOther = &bA; xfOther = &xfA;}
	else {bodyOther =&bB; xfOther = &xfB;}; 
	
	b2clBodySweepAdvance (bodyOther, xfOther, minAlpha);
	ContactUpdate ( &manifold, &polyA, &xfA, &polyB, &xfB ) ; 
	if (manifold.pointCount == 0) {
		return ; 
	}
    bodyDynamicListBuffer[idxOther] = *bodyOther;
	xfGlobal[idxOther] = *xfOther ;  
	isConnectedBuffer[idxContact] = 1 ; 


}
#endif


__kernel void b2clsyncMovedBodyKernel (
 const __global float* movedBodyBuffer, 
int numBodies, 
__global b2clTransform* xfGlobal,
__global clb2Velocity* velocityBuffer,
__global clb2Position* positionBuffer
)
{
  int idxThread = get_global_id (0);
  if (idxThread >= numBodies) return; 

  b2clTransform xf;
  clb2Velocity vel;
  clb2Position pos;

  int offset = idxThread* 11 ; 
  //float testFloat = movedBodyBuffer[offset+0]; 
    
  int idxBody = (int) movedBodyBuffer[offset+0]; 
  vel.vx = movedBodyBuffer[offset+1];
  vel.vy = movedBodyBuffer[offset+2];
  vel.w = movedBodyBuffer[offset+3];
  pos.cx = movedBodyBuffer[offset+4];
  pos.cy = movedBodyBuffer[offset+5];
  pos.a  = movedBodyBuffer[offset+6];
  xf.p.x = movedBodyBuffer[offset+7];
  xf.p.y = movedBodyBuffer[offset+8];
  xf.q.x = movedBodyBuffer[offset+9];
  xf.q.y = movedBodyBuffer[offset+10];
 

  xfGlobal[idxBody] = xf ; 
  velocityBuffer[idxBody] = vel ; 
  positionBuffer[idxBody] = pos ; 

}