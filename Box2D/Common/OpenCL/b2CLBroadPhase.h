/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLBroadPhase_h
#define Box2D_b2CLBroadPhase_h

#include <list>
#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Common/OpenCL/b2CLCompactionFuncitons.h>
#include <Box2D/Common/OpenCL/b2CLSort.h>
#include <Box2D/Collision/b2DynamicTree.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

//class b2World;
class b2RayCastCallback;
class b2QueryCallback;

#include <sys/stat.h>
#include <algorithm>
#include <math.h>
#include <fcntl.h>

#ifdef _DEBUG_TIME_BROADPHASE
#include <Box2D/Common/b2Timer.h>
#endif

class b2CLBroadPhase
{
public: 
    b2CLBroadPhase();
    ~b2CLBroadPhase();

	void CreateGPUBuffers(int shape_num);
	void ComputeAABBs(int shape_num);
	void ComputeAABBsTOI(int shape_num);
	void PrepareSumVariance(int shape_num);
	void InitSortingKeys(int shape_num);
	void SortAABBs(int shape_num);
	void ComputePairs(int shape_num, int* pTotalContactCount, int* pContactCounts, const b2World *pWorld);
	void ComputePairsNoAtomic(int shape_num, int* pTotalContactCount, int* pContactCounts);

	template<typename T>
	void RayCast(T* callback, const b2RayCastInput& input, int shape_num, bool bUseListener) const;
	template<typename T>
	void Query(T* callback, const b2AABB& inputAabb, int shape_num, bool bUseListener) const;

private:
	void ComputeAABBsIntersection(const float* aabb, int shape_num) const;
	void CheckRayShapesCollision(const b2RayCastInput& input, int intersectionTotalCount, int* intersectionCounts) const;


    cl_program broadPhaseProgram;
    cl_kernel computeAABBsKernel;
    cl_kernel computeAABBsTOIKernel;
	cl_kernel prepareSumVarianceKernel;
	cl_kernel initSortingKeysKernel;
    cl_kernel computePairsKernel;
    cl_kernel computePairsNoAtomicKernel;
	cl_kernel computeAABBIntersectionKernel;
	cl_kernel rayShapeIntersectionKernel[b2Shape::e_typeCount];
    
    size_t maxWorkGroupSizeForComputeAABBs,
		maxWorkGroupSizeForComputeAABBsTOI,
		maxWorkGroupSizeForPrepareSumVariance,
		maxWorkGroupSizeForInitSortingKeys,
		maxWorkGroupSizeForComputePairs,
		maxWorkGroupSizeForComputePairsNoAtomic,
		maxWorkGroupSizeForComputeAABBIntersection,
		maxWorkGroupSizeForRayShapeIntersection[b2Shape::e_typeCount];

	int sortCount, oldSortCount;
	int old_shape_num;
	int sweep_axis;

	cl_mem aabbListBuffer;
#ifdef NO_FLOAT4
	cl_mem sumXBuffer;
	cl_mem sumYBuffer;
	cl_mem sumX2Buffer;
	cl_mem sumY2Buffer;
#else
	cl_mem sumBuffer;
#endif
	cl_mem keysAABBListBuffer;
	cl_mem indicesAABBListBuffer;
};

// AABBQuery user callback
template<typename T>
void b2CLBroadPhase::Query(T* callback, const b2AABB& inputAabb, int shape_num, bool bUseListener) const
{
#if defined(BROADPHASE_OPENCL)
	if (bUseListener == false)
	{
		printf("b2World::SetUseListener(true) should be called to use AABBQuery!\n");
		return;
	}

	if (shape_num == 0)
		return;

	// assume AABB were computed in the last frame

	// create GPU buffer
	int maxSortCount;
#if defined(USE_CPU_SORT)
	maxSortCount = shape_num;
#else
	if (shape_num < BITONIC_SORT_INTEL_MINNUM)
		maxSortCount = BITONIC_SORT_INTEL_MINNUM;
	else
	{
		// compute the least power-of-2 which >= m_contactCount
		int exp;
		frexp((float)shape_num, &exp);
		maxSortCount = 1 << (exp-1);
		if (maxSortCount < shape_num)
			maxSortCount <<= 1;
	}
#endif
	if (b2CLCommonData::instance().oldIntersectionBufferSize < maxSortCount)
	{
		if (b2CLCommonData::instance().intersectingShapeIndices)
			b2CLDevice::instance().freeArray(b2CLCommonData::instance().intersectingShapeIndices);
		if (b2CLCommonData::instance().intersectingShapeTypes)
			b2CLDevice::instance().freeArray(b2CLCommonData::instance().intersectingShapeTypes);
		b2CLCommonData::instance().intersectingShapeIndices = b2CLDevice::instance().allocateArray(sizeof(unsigned int) * maxSortCount);
		b2CLCommonData::instance().intersectingShapeTypes = b2CLDevice::instance().allocateArray(sizeof(unsigned int) * maxSortCount);
		b2CLCommonData::instance().oldIntersectionBufferSize = maxSortCount;

		delete[] b2CLCommonData::instance().intersectingShapeIndicesData;
		b2CLCommonData::instance().intersectingShapeIndicesData = new unsigned int[maxSortCount];
	}

	// gather overapping AABBs
	float aabb[4] = { inputAabb.lowerBound.x, inputAabb.lowerBound.y, inputAabb.upperBound.x, inputAabb.upperBound.y };
	ComputeAABBsIntersection(aabb, shape_num);

	// read back number of intersections
	int intersectionTotalCount;
	b2CLDevice::instance().copyArrayFromDevice(&intersectionTotalCount, b2CLCommonData::instance().intersectionTotalCount, 0, sizeof(int), true);
	if (intersectionTotalCount == 0)
		return;

	// retrieve result to CPU
	unsigned int* intersectingShapeIndicesData = b2CLCommonData::instance().intersectingShapeIndicesData;
	b2CLDevice::instance().copyArrayFromDevice(intersectingShapeIndicesData, b2CLCommonData::instance().intersectingShapeIndices, 0, sizeof(unsigned int) * intersectionTotalCount, true);

	// retrieve transforms

	// run the callback function for fixtures that the AABB intersects
	for (int i = 0; i < intersectionTotalCount; ++i)
	{
		unsigned int shapeIndex = intersectingShapeIndicesData[i];

		callback->QueryCallback(shapeIndex);
	}
#endif
}

// RayCast user callback
template<typename T>
void b2CLBroadPhase::RayCast(T* callback, const b2RayCastInput& input, int shape_num, bool bUseListener) const
{
#if defined(BROADPHASE_OPENCL)
	if (bUseListener == false)
	{
		printf("b2World::SetUseListener(true) should be called to use RayCast!\n");
		return;
	}

	if (shape_num == 0)
		return;

	// assume AABB were computed in the last frame

	// create GPU buffer
	int maxSortCount;
#if defined(USE_CPU_SORT)
	maxSortCount = shape_num;
#else
	if (shape_num < BITONIC_SORT_INTEL_MINNUM)
		maxSortCount = BITONIC_SORT_INTEL_MINNUM;
	else
	{
		// compute the least power-of-2 which >= m_contactCount
		int exp;
		frexp((float)shape_num, &exp);
		maxSortCount = 1 << (exp-1);
		if (maxSortCount < shape_num)
			maxSortCount <<= 1;
	}
#endif
	if (b2CLCommonData::instance().oldIntersectionBufferSize < maxSortCount)
	{
		if (b2CLCommonData::instance().intersectingShapeIndices)
			b2CLDevice::instance().freeArray(b2CLCommonData::instance().intersectingShapeIndices);
		if (b2CLCommonData::instance().intersectingShapeTypes)
			b2CLDevice::instance().freeArray(b2CLCommonData::instance().intersectingShapeTypes);
		b2CLCommonData::instance().intersectingShapeIndices = b2CLDevice::instance().allocateArray(sizeof(unsigned int) * maxSortCount);
		b2CLCommonData::instance().intersectingShapeTypes = b2CLDevice::instance().allocateArray(sizeof(unsigned int) * maxSortCount);
		b2CLCommonData::instance().oldIntersectionBufferSize = maxSortCount;

		delete[] b2CLCommonData::instance().intersectingShapeIndicesData;
		b2CLCommonData::instance().intersectingShapeIndicesData = new unsigned int[maxSortCount];
	}

	// gather overapping AABBs with the ray
	const b2Vec2& from = input.p1;
	const b2Vec2 to = from + input.maxFraction * (input.p2 - from);
	float aabb[4] = { b2Min(from.x, to.x), b2Min(from.y, to.y), b2Max(from.x, to.x), b2Max(from.y, to.y)};	
	ComputeAABBsIntersection(aabb, shape_num);

	// read back number of intersections
	int intersectionTotalCount;
	int intersectionCounts[b2Shape::e_typeCount];
	b2CLDevice::instance().copyArrayFromDevice(&intersectionTotalCount, b2CLCommonData::instance().intersectionTotalCount, 0, sizeof(int), false);
	b2CLDevice::instance().copyArrayFromDevice(intersectionCounts, b2CLCommonData::instance().intersectionCounts, 0, sizeof(int) * b2Shape::e_typeCount, false);
	b2CLDevice::instance().finishCommandQueue();
	if (intersectionTotalCount == 0)
		return;

	//READ_CL_MEMORY(0, b2CLCommonData::instance().intersectingShapeIndices, unsigned int, maxSortCount);
	//READ_CL_MEMORY(1, b2CLCommonData::instance().intersectingShapeTypes, unsigned int, maxSortCount);

	// sort AABBs by shape types
	int numTypes = 0;
	for (int i = 0; i < b2Shape::e_typeCount; ++i)
	{
		if (intersectionCounts[i] > 0)
			++numTypes;
	}
	if (numTypes > 1)
	{
		// sort
#if defined(USE_CPU_SORT)
		b2CLSort::instance().stlSort(b2CLCommonData::instance().intersectingShapeTypes, b2CLCommonData::instance().intersectingShapeIndices, intersectionTotalCount, 1);
#else
		int sortCount;
		if (intersectionTotalCount < BITONIC_SORT_INTEL_MINNUM)
			sortCount = BITONIC_SORT_INTEL_MINNUM;
		else
		{
			// compute the least power-of-2 which >= m_contactCount
			int exp;
			frexp((float)shape_num, &exp);
			sortCount = 1 << (exp-1);
			if (sortCount < intersectionTotalCount)
				sortCount <<= 1;
		}
		unsigned int *zeroBuffer = new unsigned int[sortCount - intersectionTotalCount];
		memset(zeroBuffer, 0xFF, sizeof(unsigned int) * (sortCount - intersectionTotalCount));
		b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().intersectingShapeTypes, zeroBuffer, 
												 sizeof(unsigned int) *  intersectionTotalCount, sizeof(unsigned int) * (sortCount - intersectionTotalCount), true);
		delete [] zeroBuffer;
		b2CLSort::instance().bitonicSort_Intel(b2CLCommonData::instance().intersectingShapeTypes, b2CLCommonData::instance().intersectingShapeIndices, sortCount, 1);
#endif
	}

	// collision checking between the ray and shapes
	CheckRayShapesCollision(input, intersectionTotalCount, intersectionCounts);

	// retrieve result to CPU
	b2clRayCastOutput* rayCastOutputListData = b2CLCommonData::instance().rayCastOutputListData;
	b2CLDevice::instance().copyArrayFromDevice(rayCastOutputListData, b2CLCommonData::instance().rayCastOutputBuffer, 0, sizeof(b2clRayCastOutput) * intersectionTotalCount, true);

	// run the callback function for fixtures that the ray intersects
	float maxFraction = 1.0f;
	for (int i = 0; i < intersectionTotalCount; ++i)
	{
		const b2clRayCastOutput& output = rayCastOutputListData[i];
		if (output.isCollide == 0)
			continue;

		if (output.fraction > maxFraction)
			continue;

		b2Fixture* fixture = b2CLCommonData::instance().fixture_address[output.shapeIndex];

		float32 fraction = output.fraction;
		b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
		b2Vec2 normal(output.normal[0], output.normal[1]);

		float value = callback->callback->ReportFixture(fixture, point, normal, fraction);
		if (value == 0.0f)
		{
			// The client has terminated the ray cast.
			break;
		}
		if (value > 0.0f)
			maxFraction = value;
	}
#endif
}

#endif
