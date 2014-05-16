/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#include <iostream>
#include <Box2D/Common/OpenCL/b2CLBroadPhase.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/OpenCL/b2CLScan.h>
//#include <errno.h>

b2CLBroadPhase::b2CLBroadPhase()
{
#if defined(BROADPHASE_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		printf("Initializing b2CLBroadPhase...\n");

		int err;
    
		//load opencl programs from files
		char* broadPhaseSource = 0;
		size_t broadPhaseSourceLen = 0;

		shrLog("...loading b2CLBroadPhase.cl\n");
    #ifdef linux
    	broadPhaseSource = b2clLoadProgSource(shrFindFilePath("/opt/usr/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLBroadPhase.cl", NULL), "// My comment\n", &broadPhaseSourceLen);
	#elif defined (_WIN32)
		broadPhaseSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLBroadPhase.cl", NULL), "// My comment\n", &broadPhaseSourceLen);
	#else
//        FILE * pFile;
//        pFile = fopen ("/usr/local/include/Box2D/Common/OpenCL/b2CLBroadPhase.cl","r");
//        if (pFile == NULL) {
//            printf("fopen failed, errno = %d\n", errno);
//        }
        
		broadPhaseSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLBroadPhase.cl", NULL), "// My comment\n", &broadPhaseSourceLen);
	#endif
		if(broadPhaseSource == NULL)
		{
			b2Log("Could not load program source, is path 'b2CLBroadPhase.cl' correct?");
		}

		//create the compute program from source kernel code
		broadPhaseProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**) &broadPhaseSource, NULL, &err);
		if (!broadPhaseProgram)
		{
			printf("Error: Failed to create compute program!\n");
			exit(1);
		}
    
		//build the program
		err = clBuildProgram(broadPhaseProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			size_t len;
			char buffer[20480];
        
			printf("Error: Failed to build program executable!\n");
			clGetProgramBuildInfo(broadPhaseProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
			printf("%s\n", buffer);
			exit(1);
		}
    
		//create the compute kernel
		computeAABBsKernel=clCreateKernel(broadPhaseProgram, "ComputeAABBs", &err);
		if (!computeAABBsKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(computeAABBsKernel, maxWorkGroupSizeForComputeAABBs);

		computeAABBsTOIKernel=clCreateKernel(broadPhaseProgram, "ComputeAABBsTOI", &err);
		if (!computeAABBsTOIKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(computeAABBsTOIKernel, maxWorkGroupSizeForComputeAABBsTOI);

#ifdef NO_FLOAT4
		prepareSumVarianceKernel=clCreateKernel(broadPhaseProgram, "PrepareSumVarianceNoFloat4", &err);
#else
		prepareSumVarianceKernel=clCreateKernel(broadPhaseProgram, "PrepareSumVariance", &err);
#endif
		if (!prepareSumVarianceKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(prepareSumVarianceKernel, maxWorkGroupSizeForPrepareSumVariance);

		initSortingKeysKernel=clCreateKernel(broadPhaseProgram, "InitSortingKeys", &err);
		if (!initSortingKeysKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(initSortingKeysKernel, maxWorkGroupSizeForInitSortingKeys);

		computePairsKernel=clCreateKernel(broadPhaseProgram, "ComputePairs", &err);
 		//computePairsKernel=clCreateKernel(broadPhaseProgram, "computePairsLocalMemory", &err);
		if (!computePairsKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(computePairsKernel, maxWorkGroupSizeForComputePairs);

		computePairsNoAtomicKernel=clCreateKernel(broadPhaseProgram, "ComputePairsNoAtomic", &err);
		if (!computePairsNoAtomicKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(computePairsNoAtomicKernel, maxWorkGroupSizeForComputePairsNoAtomic);

		computeAABBIntersectionKernel=clCreateKernel(broadPhaseProgram, "ComputeAABBIntersection", &err);
		if (!computeAABBIntersectionKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute AABB intersection kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(computeAABBIntersectionKernel, maxWorkGroupSizeForComputeAABBIntersection);

		const char* rayShapeIntersectionKernelNames[] = {"RayCircleIntersection", "RayEdgeIntersection", "RayPolygonIntersection", "RayChainIntersection"};
		for (int i = 0; i < b2Shape::e_typeCount; ++i)
		{
			rayShapeIntersectionKernel[i]=clCreateKernel(broadPhaseProgram, rayShapeIntersectionKernelNames[i], &err);
			if (!rayShapeIntersectionKernel[i] || err != CL_SUCCESS)
			{
				printf("Error: Failed to create compute ray shape intersection kernel!\n");
				exit(1);
			}
			b2CLDevice::instance().getMaximumKernelWorkGroupSize(rayShapeIntersectionKernel[i], maxWorkGroupSizeForRayShapeIntersection[i]);
		}
	}
#endif

	oldSortCount = 0;
	old_shape_num = 0;
	sweep_axis = 0;
	aabbListBuffer = keysAABBListBuffer = indicesAABBListBuffer = NULL;

#ifdef NO_FLOAT4
	sumXBuffer = sumYBuffer = sumX2Buffer = sumY2Buffer = NULL;
#else
	sumBuffer = NULL;
#endif
}

b2CLBroadPhase::~b2CLBroadPhase()
{
}

void b2CLBroadPhase::CreateGPUBuffers(int shape_num)
{
	if (old_shape_num<shape_num)
	{
		if (aabbListBuffer)
		    b2CLDevice::instance().freeArray(aabbListBuffer);
	    aabbListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clAABB) * shape_num);

#ifdef NO_FLOAT4
		if (sumXBuffer)
		    b2CLDevice::instance().freeArray(sumXBuffer);
	    sumXBuffer = b2CLDevice::instance().allocateArray(sizeof(float) * (shape_num+1));
		if (sumYBuffer)
		    b2CLDevice::instance().freeArray(sumYBuffer);
	    sumYBuffer = b2CLDevice::instance().allocateArray(sizeof(float) * (shape_num+1));
		if (sumX2Buffer)
		    b2CLDevice::instance().freeArray(sumX2Buffer);
	    sumX2Buffer = b2CLDevice::instance().allocateArray(sizeof(float) * (shape_num+1));
		if (sumY2Buffer)
		    b2CLDevice::instance().freeArray(sumY2Buffer);
	    sumY2Buffer = b2CLDevice::instance().allocateArray(sizeof(float) * (shape_num+1));
#else
		if (sumBuffer)
		    b2CLDevice::instance().freeArray(sumBuffer);
	    sumBuffer = b2CLDevice::instance().allocateArray(sizeof(float) * 4 * (shape_num+1));
#endif
		old_shape_num = shape_num;
	}

#if 0//defined(USE_CPU_SORT)
	sortCount = shape_num;
#else
	if (shape_num<BITONIC_SORT_INTEL_MINNUM)
		sortCount = BITONIC_SORT_INTEL_MINNUM;
	else
	{
		// compute the least power-of-2 which >= m_contactCount
		int exp;
		frexp((float)shape_num, &exp);
		sortCount = 1 << (exp-1);
		if (sortCount<shape_num)
			sortCount <<= 1;
	}
#endif

	if (oldSortCount<sortCount)
	{
        if (keysAABBListBuffer)
            b2CLDevice::instance().freeArray(keysAABBListBuffer);
        keysAABBListBuffer=b2CLDevice::instance().allocateArray(sizeof(unsigned int) * sortCount);
        if (indicesAABBListBuffer)
            b2CLDevice::instance().freeArray(indicesAABBListBuffer);
        indicesAABBListBuffer=b2CLDevice::instance().allocateArray(sizeof(unsigned int) * sortCount);

		oldSortCount = sortCount;
	}
}

void b2CLBroadPhase::ComputeAABBs(int shape_num)
{
	if (shape_num<=0)
		return;

	//// for debug
	//b2clPolygonShape * testPoly = new b2clPolygonShape[shape_num];
	//b2clTransform * testXf = new b2clTransform[shape_num];
	//int * testMap = new int[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(testPoly, b2CLCommonData::instance().shapeListBuffer, 0, sizeof(b2clPolygonShape)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testXf, b2CLCommonData::instance().xfListBuffer, 0, sizeof(b2clTransform)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("testPoly %d: v0(%f, %f), r=%f\n", i, testPoly[i].m_vertices[0][0], testPoly[i].m_vertices[0][1], testPoly[i].m_radius);
	//	int j = testMap[i];
	//	printf("testXf %d: p(%f, %f), q(%f, %f)\n", j, testXf[j].p[0], testXf[j].p[1], testXf[j].q[0], testXf[j].q[1]);
	//	printf("testMap %d: %d\n", i, testMap[i]);
	//}
	//delete [] testPoly;
	//delete [] testXf;
	//delete [] testMap;

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
	err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
	err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyStaticListBuffer));
    err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
    err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
	err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeToBodyMapBuffer));
	err |= clSetKernelArg(computeAABBsKernel,  a++, sizeof(int), &shape_num);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", "computeAABBsKernel");
        return;
    }

	int group_num = (sortCount + maxWorkGroupSizeForComputeAABBs-1)/maxWorkGroupSizeForComputeAABBs;
        
    size_t global = group_num * maxWorkGroupSizeForComputeAABBs;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), computeAABBsKernel, 1, NULL, &global, &maxWorkGroupSizeForComputeAABBs, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute AABBs: Failed to execute kernel!\n");
        return;
    }

#ifdef _DEBUG
	//// for debug
	//#define DeltaY 25
	//#define DeltaY2 (DeltaY*2)

	//b2clAABB* aabb = new b2clAABB[shape_num];
	//unsigned int* keysBefore = new unsigned int[sortCount];
	//unsigned int* indicesBefore = new unsigned int[sortCount];
	//b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	//for (int i=0; i<shape_num; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//	printf("aabb %d: %d, %d\n", i, aabb[i].m_sType, aabb[i].m_bType);
	//	unsigned char mask[2];
	//	mask[0] = int(aabb[i].m_min[1])/DeltaY & 1;
	//	mask[0] |= (int(aabb[i].m_min[1])/DeltaY2 & 1)<<1;
	//	mask[1] = int(aabb[i].m_max[1])/DeltaY & 1;
	//	mask[1] |= (int(aabb[i].m_max[1])/DeltaY2 & 1)<<1;
	//	printf("computed mask: %d, %d\n", mask[0], mask[1]);
	//}
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//}
	//b2CLDevice::instance().copyArrayFromDevice(keysBefore, keysAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(indicesBefore, indicesAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//delete [] keysBefore;
	//delete [] indicesBefore;
	//delete [] aabb;
#endif
}

void b2CLBroadPhase::ComputeAABBsTOI(int shape_num)
{
	if (shape_num<=0)
		return;

	//// for debug
	//b2clPolygonShape * testPoly = new b2clPolygonShape[shape_num];
	//b2clTransform * testXf = new b2clTransform[shape_num];
	//int * testMap = new int[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(testPoly, b2CLCommonData::instance().shapeListBuffer, 0, sizeof(b2clPolygonShape)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testXf, b2CLCommonData::instance().xfListBuffer, 0, sizeof(b2clTransform)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("testPoly %d: v0(%f, %f), r=%f\n", i, testPoly[i].m_vertices[0][0], testPoly[i].m_vertices[0][1], testPoly[i].m_radius);
	//	int j = testMap[i];
	//	printf("testXf %d: p(%f, %f), q(%f, %f)\n", j, testXf[j].p[0], testXf[j].p[1], testXf[j].q[0], testXf[j].q[1]);
	//	printf("testMap %d: %d\n", i, testMap[i]);
	//}
	//delete [] testPoly;
	//delete [] testXf;
	//delete [] testMap;

#ifdef BOX2D_OPENCL_ON_CPU
	// Initialize the buffer to 0
	// OCL-GPU will do it automatically
	unsigned int *zeroBuffer = new unsigned int[sortCount];
	memset(zeroBuffer, 0, sizeof(unsigned int)*sortCount);
	b2CLDevice::instance().copyArrayToDevice(keysAABBListBuffer, zeroBuffer, 0, sizeof(unsigned int)*sortCount, true);
	delete [] zeroBuffer;
#endif

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
	err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
	err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyStaticListBuffer));
	err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyDynamicListBuffer));
    err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
    err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
	err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeToBodyMapBuffer));
	err |= clSetKernelArg(computeAABBsTOIKernel,  a++, sizeof(int), &shape_num);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", "computeAABBsKernel");
        return;
    }

	int group_num = (sortCount + maxWorkGroupSizeForComputeAABBsTOI-1)/maxWorkGroupSizeForComputeAABBsTOI;
        
    size_t global = group_num * maxWorkGroupSizeForComputeAABBsTOI;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), computeAABBsTOIKernel, 1, NULL, &global, &maxWorkGroupSizeForComputeAABBsTOI, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute AABBsTOI: Failed to execute kernel!\n");
        return;
    }

#ifdef _DEBUG
	//// for debug
	//#define DeltaY 25
	//#define DeltaY2 (DeltaY*2)

	//b2clAABB* aabb = new b2clAABB[shape_num];
	//unsigned int* keysBefore = new unsigned int[sortCount];
	//unsigned int* indicesBefore = new unsigned int[sortCount];
	//b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	//for (int i=0; i<50; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//	printf("aabb %d: %d, %d\n", i, aabb[i].m_sType, aabb[i].m_bType);
	//	unsigned char mask[2];
	//	mask[0] = int(aabb[i].m_min[1])/DeltaY & 1;
	//	mask[0] |= (int(aabb[i].m_min[1])/DeltaY2 & 1)<<1;
	//	mask[1] = int(aabb[i].m_max[1])/DeltaY & 1;
	//	mask[1] |= (int(aabb[i].m_max[1])/DeltaY2 & 1)<<1;
	//	printf("computed mask: %d, %d\n", mask[0], mask[1]);
	//}
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//}
	//b2CLDevice::instance().copyArrayFromDevice(keysBefore, keysAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(indicesBefore, indicesAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//delete [] keysBefore;
	//delete [] indicesBefore;
	//delete [] aabb;
#endif
}

void b2CLBroadPhase::PrepareSumVariance(int shape_num)
{
	if (shape_num<=0)
		return;

	//// for debug
	//b2clPolygonShape * testPoly = new b2clPolygonShape[shape_num];
	//b2clTransform * testXf = new b2clTransform[shape_num];
	//int * testMap = new int[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(testPoly, b2CLCommonData::instance().shapeListBuffer, 0, sizeof(b2clPolygonShape)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testXf, b2CLCommonData::instance().xfListBuffer, 0, sizeof(b2clTransform)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("testPoly %d: v0(%f, %f), r=%f\n", i, testPoly[i].m_vertices[0][0], testPoly[i].m_vertices[0][1], testPoly[i].m_radius);
	//	int j = testMap[i];
	//	printf("testXf %d: p(%f, %f), q(%f, %f)\n", j, testXf[j].p[0], testXf[j].p[1], testXf[j].q[0], testXf[j].q[1]);
	//	printf("testMap %d: %d\n", i, testMap[i]);
	//}
	//delete [] testPoly;
	//delete [] testXf;
	//delete [] testMap;

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
#ifdef NO_FLOAT4
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &sumXBuffer);
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &sumYBuffer);
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &sumX2Buffer);
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &sumY2Buffer);
#else
    err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(cl_mem), &sumBuffer);
#endif
	err |= clSetKernelArg(prepareSumVarianceKernel,  a++, sizeof(int), &shape_num);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", "prepareSumVarianceKernel");
        return;
    }

	int group_num = (sortCount + maxWorkGroupSizeForPrepareSumVariance-1)/maxWorkGroupSizeForPrepareSumVariance;
        
    size_t global = group_num * maxWorkGroupSizeForPrepareSumVariance;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), prepareSumVarianceKernel, 1, NULL, &global, &maxWorkGroupSizeForPrepareSumVariance, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute AABBs: Failed to execute kernel!\n");
        return;
    }

#ifdef _DEBUG
	//// for debug
	//float* testSum = new float[shape_num*4];
	//float* testSum2 = new float[shape_num*4];
	//b2CLDevice::instance().copyArrayFromDevice(testSum, sumBuffer, 0, sizeof(float)*4*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testSum2, sum2Buffer, 0, sizeof(float)*4*shape_num, true);
	//delete [] testSum;
	//delete [] testSum2;
#endif

	//// for debug
	//int num = 10;
	//float *fillnums = new float[num*4];
	//float *input = new float[num*4];
	//float *result = new float[num*4];
	//for (int i=0; i<num; i++)
	//{
	//	fillnums[i*4] = i;
	//	fillnums[i*4+1] = i;
	//	fillnums[i*4+2] = i;
	//	fillnums[i*4+3] = i;
	//}
	//b2CLDevice::instance().copyArrayToDevice(sumBuffer, fillnums, 0, sizeof(float)*4*num, true);
	//b2CLDevice::instance().copyArrayFromDevice(input, sumBuffer, 0, sizeof(float)*4*num, true);
	//b2CLScanFloat4::instance().Scan(sumBuffer, num);
	//clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScanFloat4::instance().scanResultsBuffer, true, 0/*sizeof(int)*4*(shape_num-1)*/, sizeof(float)*4*num, result, 0, NULL, NULL);

	sweep_axis = 0;
	if (shape_num>1)
	{
		b2Vec2 s, s2;
#if defined(USE_CPU_SCAN)
		float *sumList = new float[shape_num*4];
		float *scanResult = new float[(shape_num+1)*4];
		b2CLDevice::instance().copyArrayFromDevice(sumList, sumBuffer, 0, sizeof(float)*4*shape_num, true);
		scanResult[0] = scanResult[1] = scanResult[2] = scanResult[3] = 0;
		for (int i=1; i<shape_num+1; i++)
		{
			scanResult[i*4] = scanResult[(i-1)*4] + sumList[(i-1)*4];
			scanResult[i*4+1] = scanResult[(i-1)*4+1] + sumList[(i-1)*4+1];
			scanResult[i*4+2] = scanResult[(i-1)*4+2] + sumList[(i-1)*4+2];
			scanResult[i*4+3] = scanResult[(i-1)*4+3] + sumList[(i-1)*4+3];
		}
		s = b2Vec2(scanResult[shape_num*4], scanResult[shape_num*4+1]);
		s2 = b2Vec2(scanResult[shape_num*4+2], scanResult[shape_num*4+3]);
		delete [] sumList;
		delete [] scanResult;
#else
		float scanResult[4];
	#ifdef NO_FLOAT4
		b2CLScan::instance().ScanCLPP(sumXBuffer, shape_num+1);
		clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScan::instance().scanResultsBuffer, true, sizeof(float)*shape_num, sizeof(float), scanResult, 0, NULL, NULL);
		b2CLScan::instance().ScanCLPP(sumYBuffer, shape_num+1);
		clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScan::instance().scanResultsBuffer, true, sizeof(float)*shape_num, sizeof(float), &(scanResult[1]), 0, NULL, NULL);
		b2CLScan::instance().ScanCLPP(sumX2Buffer, shape_num+1);
		clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScan::instance().scanResultsBuffer, true, sizeof(float)*shape_num, sizeof(float), &(scanResult[2]), 0, NULL, NULL);
		b2CLScan::instance().ScanCLPP(sumY2Buffer, shape_num+1);
		clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScan::instance().scanResultsBuffer, true, sizeof(float)*shape_num, sizeof(float), &(scanResult[3]), 0, NULL, NULL);
	#else
		b2CLScanFloat4::instance().Scan(sumBuffer, shape_num+1);
		clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScanFloat4::instance().scanResultsBuffer, true, sizeof(float)*4*shape_num, sizeof(float)*2, scanResult, 0, NULL, NULL);
		//b2CLScanFloat4::instance().Scan(sum2Buffer, shape_num+1);
		//clEnqueueReadBuffer(b2CLDevice::instance().GetCommandQueue(), b2CLScanFloat4::instance().scanResultsBuffer, true, sizeof(float)*4*shape_num, sizeof(float)*2, &s2, 0, NULL, NULL);
	#endif
		s = b2Vec2(scanResult[0], scanResult[1]);
		s2 = b2Vec2(scanResult[2], scanResult[3]);
#endif
        
        //printf("s: (%f, %f), s2: (%f, %f)\n", s.x, s.y, s2.x, s2.y);
        //printf("centroid: (%f, %f)\n", s.x/shape_num, s.y/shape_num);

		b2Vec2 v;
		v.x = s.x * s.x;
		v.y = s.y * s.y;
		v *= 1/(float)shape_num;
		v = s2 - v;

		if(v(1) > v(0)) 
			sweep_axis = 1;
	}

	//printf("sweep_axis: %d\n", sweep_axis);
}

void b2CLBroadPhase::InitSortingKeys(int shape_num)
{
	if (shape_num<=0)
		return;

	//// for debug
	//b2clPolygonShape * testPoly = new b2clPolygonShape[shape_num];
	//b2clTransform * testXf = new b2clTransform[shape_num];
	//int * testMap = new int[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(testPoly, b2CLCommonData::instance().shapeListBuffer, 0, sizeof(b2clPolygonShape)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testXf, b2CLCommonData::instance().xfListBuffer, 0, sizeof(b2clTransform)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(testMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("testPoly %d: v0(%f, %f), r=%f\n", i, testPoly[i].m_vertices[0][0], testPoly[i].m_vertices[0][1], testPoly[i].m_radius);
	//	int j = testMap[i];
	//	printf("testXf %d: p(%f, %f), q(%f, %f)\n", j, testXf[j].p[0], testXf[j].p[1], testXf[j].q[0], testXf[j].q[1]);
	//	printf("testMap %d: %d\n", i, testMap[i]);
	//}
	//delete [] testPoly;
	//delete [] testXf;
	//delete [] testMap;

#ifdef BOX2D_OPENCL_ON_CPU
	// Initialize the buffer to 0
	// OCL-GPU will do it automatically
	unsigned int *zeroBuffer = new unsigned int[sortCount];
	memset(zeroBuffer, 0, sizeof(unsigned int)*sortCount);
	b2CLDevice::instance().copyArrayToDevice(keysAABBListBuffer, zeroBuffer, 0, sizeof(unsigned int)*sortCount, true);
	delete [] zeroBuffer;
#endif

	int axis = sweep_axis;

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
    err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
    err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(cl_mem), &keysAABBListBuffer);
    err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(cl_mem), &indicesAABBListBuffer);
	err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(int), &shape_num);
	err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(int), &axis);
 	err |= clSetKernelArg(initSortingKeysKernel,  a++, sizeof(int), &sortCount);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) initSortingKeysKernel);
        return;
    }

	int group_num = (sortCount + maxWorkGroupSizeForInitSortingKeys-1)/maxWorkGroupSizeForInitSortingKeys;
        
    size_t global = group_num * maxWorkGroupSizeForInitSortingKeys;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), initSortingKeysKernel, 1, NULL, &global, &maxWorkGroupSizeForInitSortingKeys, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute AABBs: Failed to execute kernel!\n");
        return;
    }

#ifdef _DEBUG
	//// for debug
	//#define DeltaY 25
	//#define DeltaY2 (DeltaY*2)

	//b2clAABB* aabb = new b2clAABB[shape_num];
	//unsigned int* keysBefore = new unsigned int[sortCount];
	//unsigned int* indicesBefore = new unsigned int[sortCount];
	//b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	//for (int i=0; i<50; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//	printf("aabb %d: %d, %d\n", i, aabb[i].m_sType, aabb[i].m_bType);
	//	unsigned char mask[2];
	//	mask[0] = int(aabb[i].m_min[1])/DeltaY & 1;
	//	mask[0] |= (int(aabb[i].m_min[1])/DeltaY2 & 1)<<1;
	//	mask[1] = int(aabb[i].m_max[1])/DeltaY & 1;
	//	mask[1] |= (int(aabb[i].m_max[1])/DeltaY2 & 1)<<1;
	//	printf("computed mask: %d, %d\n", mask[0], mask[1]);
	//}
	//for (int i=shape_num-6; i<shape_num; i++)
	//{
	//	printf("aabb %d: (%f, %f, %f, %f)\n", i, aabb[i].m_min[0], aabb[i].m_min[1], aabb[i].m_max[0], aabb[i].m_max[1]);
	//}
	//b2CLDevice::instance().copyArrayFromDevice(keysBefore, keysAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(indicesBefore, indicesAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//delete [] keysBefore;
	//delete [] indicesBefore;
	//delete [] aabb;
#endif
}


float FFlip (unsigned int fflip)
{
	unsigned int mask = ((fflip >> 31) - 1) | 0x80000000;
	unsigned int fl = fflip ^ mask;
	return *((float*)(&fl));
}

void b2CLBroadPhase::SortAABBs(int shape_num)
{
#ifdef _DEBUG
	// for debug
	b2clAABB* aabb = new b2clAABB[shape_num];
	unsigned int* keysBefore = new unsigned int[sortCount];
	unsigned int* keysAfter = new unsigned int[sortCount];
	unsigned int* indicesBefore = new unsigned int[sortCount];
	unsigned int* indicesAfter = new unsigned int[sortCount];
	b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	b2CLDevice::instance().copyArrayFromDevice(keysBefore, keysAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	b2CLDevice::instance().copyArrayFromDevice(indicesBefore, indicesAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);

	float *keysFBefore = new float[sortCount];
	float *keysFAfter = new float[sortCount];
	for (int i=0; i<sortCount; i++)
		keysFBefore[i] = FFlip(keysBefore[i]);
#endif

#if defined(USE_CPU_SORT)
	b2CLSort::instance().stlSort(keysAABBListBuffer, indicesAABBListBuffer, sortCount, 0);
#else
	b2CLSort::instance().bitonicSort_Intel(keysAABBListBuffer, indicesAABBListBuffer, sortCount, 0);
#endif

#ifdef _DEBUG
	//// for debug
	//b2CLDevice::instance().copyArrayFromDevice(keysAfter, keysAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(indicesAfter, indicesAABBListBuffer, 0, sizeof(unsigned int)*sortCount, true);

	//for (int i=0; i<sortCount; i++)
	//	keysFAfter[i] = FFlip(keysAfter[i]);

	//float *sortedKeys = new float[shape_num];
	//for (int i = 0; i<shape_num; i++)
	//{
	//	int j = indicesAfter[i];
	//	if (j>shape_num || j<0)
	//		int a = 1;
	//	sortedKeys[i] = aabb[j].m_min[0];
	//}
	//delete [] sortedKeys;
	//delete [] keysBefore;
	//delete [] keysAfter;
	//delete [] indicesBefore;
	//delete [] indicesAfter;
	//delete [] aabb;
	//delete [] keysFBefore;
	//delete [] keysFAfter;
#endif
}

void b2CLBroadPhase::ComputePairs(int shape_num, int* pTotalContactCount, int* pContactCounts, const b2World *pWorld)
{
	if (shape_num<=0)
		return;

	int axis = sweep_axis;
	int maxPairs = shape_num * MAX_CONTACT_PER_FIXTURE;
	// clear pairCountBuffer to 0
	int fill_num = 0;
	// APP Profiler said no need for block
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().pairTotalCountBuffer, &fill_num, 0, sizeof(int), true);

	int *fill_nums = new int[b2Shape::contact_type_num];
	memset(fill_nums, 0, sizeof(int)*b2Shape::contact_type_num);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().pairCountsBuffer, fill_nums, 0, sizeof(int)*b2Shape::contact_type_num, true);
    delete [] fill_nums;

#ifdef _DEBUG
	//// for debug
	//b2clAABB* aabb = new b2clAABB[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	//delete [] aabb;
	////cl_mem testBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clAABB) * shape_num);

	//// for debug
	//int n = min(shape_num*shape_num, shape_num*200);
	//int* indices = new int[n*4];
	//b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*n*4, true);
	//delete [] indices;

	//// for debug
	//int *indicesAABB = new int[shape_num];
	//int *shapeToBodyMap = new int[shape_num];;
	//b2CLDevice::instance().copyArrayFromDevice(indicesAABB, indicesAABBListBuffer, 0, sizeof(int)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(shapeToBodyMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//delete indicesAABB;
	//delete shapeToBodyMap;
#endif

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
    err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
	//err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &testBuffer);
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyStaticListBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().globalIndicesBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairIndicesBuffer));
    err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &indicesAABBListBuffer);
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeToBodyMapBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairCountsBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairTotalCountBuffer));
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(int), &shape_num);
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(int), &axis);
	err |= clSetKernelArg(computePairsKernel,  a++, sizeof(int), &maxPairs);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) computeAABBsKernel);
        return;
    }

	//maxWorkGroupSizeForComputePairs = 64;

	int group_num = (shape_num + maxWorkGroupSizeForComputePairs-1)/maxWorkGroupSizeForComputePairs;
        
    size_t global = group_num * maxWorkGroupSizeForComputePairs;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), computePairsKernel, 1, NULL, &global, &maxWorkGroupSizeForComputePairs, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute Pairs: Failed to execute kernel!\n");
        return;
    }

	// read back number of contacts
	int pairTotalCount;
	b2CLDevice::instance().copyArrayFromDevice(&pairTotalCount, b2CLCommonData::instance().pairTotalCountBuffer, 0, sizeof(int), true);
	*pTotalContactCount = pairTotalCount;
	b2CLDevice::instance().copyArrayFromDevice(pContactCounts, b2CLCommonData::instance().pairCountsBuffer, 0, sizeof(int)*b2Shape::contact_type_num, true);

	//printf("Pair count: %d\n", pairTotalCount);
	
#ifdef _DEBUG
	//// for debug
	////b2CLDevice::instance().copyArrayFromDevice(aabb, testBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	////printf("aabb[71]: %f, %f, %f, %f\n", aabb[71].m_min[0], aabb[71].m_min[1], aabb[71].m_max[0], aabb[71].m_max[1]);

	//// for debug
	//if (pairTotalCount>0)
	//	int a = 0;
	//int n = shape_num*MAX_CONTACT_PER_FIXTURE;
	//int* global_indices = new int[n*4];
	//int* indices = new int[n * b2Shape::contact_type_num];
	//b2CLDevice::instance().copyArrayFromDevice(global_indices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*4*n, true);
    //for (int i=0; i<n; i++)
    //{
    //    int indexA = global_indices[i*4+2];
    //    int indexB = global_indices[i*4+3];
    //    int body_num = pWorld->GetBodyCount();
    //    if (indexA>=body_num)
    //        printf("Error! indexA>=body_num! indexA=%d, body_num=%d\n", indexA, body_num);
    //    if (indexB>=body_num)
    //        printf("Error! indexB>=body_num! indexB=%d, body_num=%d\n", indexB, body_num);
    //}
	//b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*n*b2Shape::contact_type_num, true);
	//int *test = new int[pairTotalCount];
	//memset(test, 0, sizeof(int)*pairTotalCount);
	//for (int i=0; i<pairTotalCount; i++)
	//	test[i] = 0;
	//for (int i=0; i<b2Shape::contact_type_num; i++)
	//{
	//	for (int j=0; j<pContactCounts[i]; j++)
	//	{
	//		test[indices[j+n*i]] = 1;
	//	}
	//}
	//for (int i=0; i<pairTotalCount; i++)
	//{
	//	if (test[i]==0)
	//		int a = 1;
	//}
	//delete [] test;
	//delete [] global_indices;
	//delete [] indices;
#endif
}

void b2CLBroadPhase::ComputePairsNoAtomic(int shape_num, int* pTotalContactCount, int* pContactCounts)
{
	if (shape_num<=0)
		return;

	int axis = 0;
	int maxPairs = shape_num * MAX_CONTACT_PER_FIXTURE;
	int maxPairsPerFixutre = MAX_CONTACT_PER_FIXTURE;
	// clear pairCountBuffer to 0
	int fill_num = 0;
	// APP Profiler said no need for block
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().pairTotalCountBuffer, &fill_num, 0, sizeof(int), true);

	int *fill_nums = new int[b2Shape::contact_type_num];
	memset(fill_nums, 0, sizeof(int)*b2Shape::contact_type_num);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().pairCountsBuffer, fill_nums, 0, sizeof(int)*b2Shape::contact_type_num, true);
    delete [] fill_nums;

#ifdef _DEBUG
	//// for debug
	//b2clAABB* aabb = new b2clAABB[shape_num];
	//b2CLDevice::instance().copyArrayFromDevice(aabb, aabbListBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	//delete [] aabb;
	////cl_mem testBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clAABB) * shape_num);

	//// for debug
	//int n = min(shape_num*shape_num, shape_num*200);
	//int* indices = new int[n*4];
	//b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*n*4, true);
	//delete [] indices;

	//// for debug
	//int *indicesAABB = new int[shape_num];
	//int *shapeToBodyMap = new int[shape_num];;
	//b2CLDevice::instance().copyArrayFromDevice(indicesAABB, indicesAABBListBuffer, 0, sizeof(int)*shape_num, true);
	//b2CLDevice::instance().copyArrayFromDevice(shapeToBodyMap, b2CLCommonData::instance().shapeToBodyMapBuffer, 0, sizeof(int)*shape_num, true);
	//delete indicesAABB;
	//delete shapeToBodyMap;
#endif

	unsigned int a = 0;
        
    int err = CL_SUCCESS;
    err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
	//err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &testBuffer);
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyStaticListBuffer));
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().globalIndicesBuffer));
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairIndicesBuffer));
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairIndicesBinaryBitsBuffer));
    err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &indicesAABBListBuffer);
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeToBodyMapBuffer));
	//err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairCountsBuffer));
	//err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairTotalCountBuffer));
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(int), &shape_num);
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(int), &axis);
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(int), &maxPairsPerFixutre);
	err |= clSetKernelArg(computePairsNoAtomicKernel,  a++, sizeof(int), &maxPairs);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) computePairsNoAtomicKernel);
        return;
    }

	//maxWorkGroupSizeForComputePairs = 64;

	int group_num = (shape_num + maxWorkGroupSizeForComputePairsNoAtomic-1)/maxWorkGroupSizeForComputePairsNoAtomic;
        
    size_t global = group_num * maxWorkGroupSizeForComputePairs;
	//cout << contactNum << ", " << group_num << endl;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), computePairsNoAtomicKernel, 1, NULL, &global, &maxWorkGroupSizeForComputePairs, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Compute Pairs: Failed to execute kernel!\n");
        return;
    }

	//// for debug
	//{
		//int *binaryArray = new int[sizeof(int)*maxPairs*b2Shape::contact_type_num];
		//int *globalIndices = new int[sizeof(int) * 4 * maxPairs];
		//int *pairIndices = new int[sizeof(int) * maxPairs * b2Shape::contact_type_num];
		//b2CLDevice::instance().copyArrayFromDevice(binaryArray, b2CLCommonData::instance().pairIndicesBinaryBitsBuffer, 0, sizeof(int)*maxPairs*b2Shape::contact_type_num, true);
		//b2CLDevice::instance().copyArrayFromDevice(globalIndices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*4*maxPairs, true);
		//b2CLDevice::instance().copyArrayFromDevice(pairIndices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*maxPairs*b2Shape::contact_type_num, true);
		//delete [] binaryArray;
		//delete [] globalIndices;
		//delete [] pairIndices;
	//}

	// scan and compact pair_indices for each type
	// we can merge the following code using segmented scan later

	int pairCount;
	cl_mem resultBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * maxPairs);
	// first segment (type 0)
	b2CLScan::instance().PreScanBuffer(b2CLCommonData::instance().pairIndicesBinaryBitsBuffer, maxPairs);
	b2CLScan::instance().ParallelCompactGeneral(resultBuffer, 
		b2CLCommonData::instance().pairIndicesBuffer, 
		b2CLCommonData::instance().pairIndicesBinaryBitsBuffer, 
		b2CLScan::instance().scanResultsBuffer, 
		maxPairs);
	b2CLDevice::instance().copyArrayInsideDevice(resultBuffer, b2CLCommonData::instance().pairIndicesBuffer, 0, 0, sizeof(int)*maxPairs);
	b2CLDevice::instance().copyArrayFromDevice(&pairCount, b2CLScan::instance().numValidDataBuffer, 0, sizeof(cl_uint), true);
	pContactCounts[0] = pairCount;
	*pTotalContactCount = pairCount;

	// other segments (types>0)
	//cl_mem subBufferBinary = b2CLDevice::instance().allocateArray(sizeof(int) * maxPairs);
	//cl_mem subBufferIndices = b2CLDevice::instance().allocateArray(sizeof(int) * maxPairs);
	//for (int i=1; i<b2Shape::contact_type_num; i++)
	//{
	//	b2CLDevice::instance().copyArrayInsideDevice(b2CLCommonData::instance().pairIndicesBinaryBitsBuffer, subBufferBinary, sizeof(int)*i*maxPairs, 0, sizeof(int)*maxPairs);
	//	b2CLDevice::instance().copyArrayInsideDevice(b2CLCommonData::instance().pairIndicesBuffer, subBufferIndices, sizeof(int)*i*maxPairs, 0, sizeof(int)*maxPairs);

	//	// for debug
	//	int* indices = new int[sizeof(int) * maxPairs * b2Shape::contact_type_num];
	//	b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*maxPairs*b2Shape::contact_type_num, true);
	//	b2CLDevice::instance().copyArrayFromDevice(indices, subBufferIndices, 0, sizeof(int)*maxPairs, true);

	//	b2CLScan::instance().PreScanBuffer(subBufferBinary, maxPairs);
	//	b2CLScan::instance().ParallelCompactGeneral(resultBuffer, 
	//		subBufferIndices, 
	//		subBufferBinary, 
	//		b2CLScan::instance().scanResultsBuffer, 
	//		maxPairs);

	//	b2CLDevice::instance().copyArrayInsideDevice(resultBuffer, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*i*maxPairs, sizeof(int)*maxPairs);

	//	b2CLDevice::instance().copyArrayFromDevice(&pairCount, b2CLScan::instance().numValidDataBuffer, 0, sizeof(cl_uint), true);
	//	pContactCounts[i] = pairCount;
	//	*pTotalContactCount += pairCount;
	//}
	b2CLDevice::instance().freeArray(resultBuffer);
	//b2CLDevice::instance().freeArray(subBufferBinary);
	//b2CLDevice::instance().freeArray(subBufferIndices);

	//printf("Pair count: %d\n", *pTotalContactCount);
	
	*pTotalContactCount = maxPairs;

#ifdef _DEBUG
	//// for debug
	////b2CLDevice::instance().copyArrayFromDevice(aabb, testBuffer, 0, sizeof(b2clAABB)*shape_num, true);
	////printf("aabb[71]: %f, %f, %f, %f\n", aabb[71].m_min[0], aabb[71].m_min[1], aabb[71].m_max[0], aabb[71].m_max[1]);

	//// for debug
	//int n = shape_num*MAX_CONTACT_PER_FIXTURE;
	//int* global_indices = new int[n*4];
	//int* indices = new int[n * b2Shape::contact_type_num];
	//b2CLDevice::instance().copyArrayFromDevice(global_indices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*4*n, true);
	//b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*n*b2Shape::contact_type_num, true);
	//int *test = new int[pairTotalCount];
	//memset(test, 0, sizeof(int)*pairTotalCount);
	//for (int i=0; i<pairTotalCount; i++)
	//	test[i] = 0;
	//for (int i=0; i<b2Shape::contact_type_num; i++)
	//{
	//	for (int j=0; j<pContactCounts[i]; j++)
	//	{
	//		test[indices[j+n*i]] = 1;
	//	}
	//}
	//for (int i=0; i<pairTotalCount; i++)
	//{
	//	if (test[i]==0)
	//		int a = 1;
	//}
	//delete [] test;
	//delete [] global_indices;
	//delete [] indices;
#endif
}

// Compute the list of intersecting AABBs with the input AABB.
void b2CLBroadPhase::ComputeAABBsIntersection(const float* aabb, int shape_num) const
{
	int fill_num = 0;
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().intersectionTotalCount, &fill_num, 0, sizeof(int), false);

	int fill_nums[b2Shape::e_typeCount];
	memset(fill_nums, 0, sizeof(int) * b2Shape::e_typeCount);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().intersectionCounts, fill_nums, 0, sizeof(int) * b2Shape::e_typeCount, false);
	b2CLDevice::instance().finishCommandQueue();

	unsigned int a = 0;
    int err = CL_SUCCESS;
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(float) * 4, aabb);
    err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &aabbListBuffer);
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &indicesAABBListBuffer);
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(int), &shape_num);
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().intersectionCounts));
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().intersectionTotalCount));
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().intersectingShapeIndices));
	err |= clSetKernelArg(computeAABBIntersectionKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().intersectingShapeTypes));
    if (err != CL_SUCCESS)
    {
		printf("Error: %s: Failed to set kernel arguments!\n", (char *) computeAABBIntersectionKernel);
        return;
    }

	int group_num = ComputeGroupNum(shape_num, maxWorkGroupSizeForComputeAABBIntersection);
    size_t global = group_num * maxWorkGroupSizeForComputeAABBIntersection;
    err = CL_SUCCESS;
	err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), computeAABBIntersectionKernel, 1, NULL, &global, &maxWorkGroupSizeForComputeAABBIntersection, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", (char *) computeAABBIntersectionKernel);
        return;
    }
}

// Compute the list of intersecting AABBs with the input ray.
void b2CLBroadPhase::CheckRayShapesCollision(const b2RayCastInput& input, int intersectionTotalCount, int* intersectionCounts) const
{
	if (b2CLCommonData::instance().oldRayCastOutBufferSize < intersectionTotalCount)
	{
		if (b2CLCommonData::instance().rayCastOutputBuffer)
			b2CLDevice::instance().freeArray(b2CLCommonData::instance().rayCastOutputBuffer);
		delete[] b2CLCommonData::instance().rayCastOutputListData;

		b2CLCommonData::instance().rayCastOutputBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clRayCastOutput) * intersectionTotalCount);
		b2CLCommonData::instance().rayCastOutputListData = new b2clRayCastOutput[intersectionTotalCount];

		b2CLCommonData::instance().oldRayCastOutBufferSize = intersectionTotalCount;
	}

	float ray[4] = { input.p1.x, input.p1.y, input.p2.x, input.p2.y };

	int offset = 0;
	for (int i = 0; i < b2Shape::e_typeCount; ++i)
	{
		if (intersectionCounts[i] == 0)
			continue;

		unsigned int a = 0;
		int err = CL_SUCCESS;
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(float) * 4, ray);
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(cl_mem), &(b2CLCommonData::instance().intersectingShapeIndices));
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeToBodyMapBuffer));
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(int), &offset);
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(int), &intersectionCounts[i]);
		err |= clSetKernelArg(rayShapeIntersectionKernel[i],  a++, sizeof(cl_mem), &(b2CLCommonData::instance().rayCastOutputBuffer));
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", (char *) rayShapeIntersectionKernel[i]);
			return;
		}

		int group_num = ComputeGroupNum(intersectionCounts[i], maxWorkGroupSizeForRayShapeIntersection[i]);
		size_t global = group_num * maxWorkGroupSizeForRayShapeIntersection[i];
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), rayShapeIntersectionKernel[i], 1, NULL, &global, &maxWorkGroupSizeForRayShapeIntersection[i], 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to execute kernel!\n", (char *) rayShapeIntersectionKernel[i]);
			return;
		}

		offset += intersectionCounts[i];
	}
}
