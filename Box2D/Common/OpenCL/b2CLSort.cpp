/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/

#include <algorithm>
#include <Box2D/Common/OpenCL/b2CLSort.h>
#include <Box2D/Common/OpenCL/b2CLDevice.h>
#ifdef linux
#include <limits.h>
#include <cstdlib>
#endif

const char* b2CLSort::SortKernelNames_NV[] = 
{
	"bitonicSortLocal",
	"bitonicSortLocal1",
	"bitonicMergeGlobal",
	"bitonicMergeLocal"
};

b2CLSort::b2CLSort()
{
    printf("Initializing b2CLSort...\n");
    
    int err;
    
    //load opencl programs from files
    char* sortKernelSource = 0;
    size_t sortKernelSourceLen = 0;

#ifndef USE_CPU_SORT
	// Load and create Bitonic Sort kernels from NVIDIA
    shrLog("...loading b2CLBitonicSort_NV.cl\n");
#ifdef linux
    sortKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/usr/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLBitonicSort_NV.cl", NULL), "// My comment\n", &sortKernelSourceLen);
#elif defined (_WIN32)
	sortKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLBitonicSort_NV.cl", NULL), "// My comment\n", &sortKernelSourceLen);
#else
    sortKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLBitonicSort_NV.cl", NULL), "// My comment\n", &sortKernelSourceLen);
#endif
	if(sortKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLBitonicSort_NV.cl' correct?");
	}

    //create the compute program from source kernel code
    sortProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&sortKernelSource, NULL, &err);
    if (!sortProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(sortProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(sortProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
    
    //create the compute kernel
	kernel_work_group_size_NV = INT_MAX;
    for(int i = 0; i < SORTKERNELCOUNT; i++)
    {    
        // Create each kernel from within the compactionProgram
        sortKernels_NV[i] = clCreateKernel(sortProgram, SortKernelNames_NV[i], &err);
        if (!sortKernels_NV[i] || err != CL_SUCCESS)
        {
            printf("Error: Failed to create sort kernel!\n");
            exit(1);
        }
		size_t size;
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(sortKernels_NV[i], size);
		kernel_work_group_size_NV = kernel_work_group_size_NV<size ? size : kernel_work_group_size_NV;
    }

	// Load and create Bitonic Sort kernel from Intel
    shrLog("...loading b2CLBitonicSort_Intel.cl\n");
#ifdef linux
    sortKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/usr/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLBitonicSort_Intel.cl", NULL), "// My comment\n", &sortKernelSourceLen);    
#elif defined (_WIN32)    
	sortKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLBitonicSort_Intel.cl", NULL), "// My comment\n", &sortKernelSourceLen);
#else
    sortKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLBitonicSort_Intel.cl", NULL), "// My comment\n", &sortKernelSourceLen);
#endif
	if(sortKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLBitonicSort_Intel.cl' correct?");
	}

    //create the compute program from source kernel code
    sortProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&sortKernelSource, NULL, &err);
    if (!sortProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(sortProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(sortProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
    
    //create the compute kernel
    sortKernel_Intel = clCreateKernel(sortProgram, "BitonicSort", &err);
    if (!sortKernel_Intel || err != CL_SUCCESS)
    {
        printf("Error: Failed to create compute kernel!\n");
        exit(1);
    }
    b2CLDevice::instance().getMaximumKernelWorkGroupSize(sortKernel_Intel, kernel_work_group_size_Intel);
#else
	old_sort_num = 0;
	keys = values = NULL;
	elements = NULL;
#endif
}

b2CLSort::~b2CLSort()
{
}

b2CLSort& b2CLSort::instance()
{
	static b2CLSort inst;
	return inst;
}
#define LOCAL_SIZE_LIMIT 512U
void b2CLSort::bitonicSort_NV(
    cl_mem d_DstKey,
    cl_mem d_DstVal,
    cl_mem d_SrcKey,
    cl_mem d_SrcVal,
    unsigned int batch,
    unsigned int arrayLength,
    unsigned int dir
)
{
	if(arrayLength < 2)
		return;

	//Only power-of-two array lengths are supported so far
	//cl_uint log2L;
	//cl_uint factorizationRemainder = factorRadix2(log2L, arrayLength);
	//b2clCheckError( factorizationRemainder == 1, shrTRUE );
	b2clCheckError( IsPowerOfTwo(arrayLength), shrTRUE );

	dir = (dir != 0);

	cl_int ciErrNum;
	size_t localWorkSize, globalWorkSize;
	unsigned int k, a;

	if(arrayLength <= LOCAL_SIZE_LIMIT)
	{
		//printf("SMALL array\n");
		b2clCheckError( (batch * arrayLength) % LOCAL_SIZE_LIMIT == 0, shrTRUE );

		//Launch bitonicSortLocal
		k = BITONICSORTLOCAL;
		a = 0;
		ciErrNum  = clSetKernelArg(sortKernels_NV[k],  a++,   sizeof(cl_mem), (void *)&d_DstKey);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,   sizeof(cl_mem), (void *)&d_DstVal);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,   sizeof(cl_mem), (void *)&d_SrcKey);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,   sizeof(cl_mem), (void *)&d_SrcVal);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_uint), (void *)&arrayLength);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_uint), (void *)&dir);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		localWorkSize  = LOCAL_SIZE_LIMIT / 2;
		globalWorkSize = batch * arrayLength / 2;
		ciErrNum = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernels_NV[k], 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);
	}
	else
	{
		//printf("BIG array\n");
		//Launch bitonicSortLocal1
		k = BITONICSORTLOCAL1;
		a = 0;
		ciErrNum  = clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstKey);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstVal);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_SrcKey);
		ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_SrcVal);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		localWorkSize  = LOCAL_SIZE_LIMIT / 2;
		globalWorkSize = batch * arrayLength / 2;
		ciErrNum = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernels_NV[k], 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		for(unsigned int size = 2 * LOCAL_SIZE_LIMIT; size <= arrayLength; size <<= 1)
		{
			for(unsigned stride = size / 2; stride > 0; stride >>= 1)
			{
				if(stride >= LOCAL_SIZE_LIMIT)
				{
					//Launch bitonicMergeGlobal
					k = BITONICMERGEGLOBAL;
					a = 0;
					ciErrNum  = clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&arrayLength);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&size);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&stride);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&dir);
					b2clCheckError(ciErrNum, CL_SUCCESS);

					localWorkSize  = LOCAL_SIZE_LIMIT / 2;
					globalWorkSize = batch * arrayLength / 2;

					ciErrNum = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernels_NV[k], 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					b2clCheckError(ciErrNum, CL_SUCCESS);

				}
				else
				{
					//Launch bitonicMergeLocal
					k = BITONICMERGELOCAL;
					a = 0;
					ciErrNum  = clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&arrayLength);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&stride);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&size);
					ciErrNum |= clSetKernelArg(sortKernels_NV[k],  a++, sizeof(cl_uint), (void *)&dir);
					b2clCheckError(ciErrNum, CL_SUCCESS);

					localWorkSize  = LOCAL_SIZE_LIMIT / 2;
					globalWorkSize = batch * arrayLength / 2;

					ciErrNum = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernels_NV[k], 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					b2clCheckError(ciErrNum, CL_SUCCESS);
					break;
				}
			}
			int aaa = 0;
		}
	}
}

void b2CLSort::bitonicSort_Intel(cl_mem keyBuffer, cl_mem valueBuffer, cl_int arraySize, cl_uint sortAscending)
{
	//// for debug
	//unsigned int *tempKeyBefore = new unsigned int[arraySize];
	//unsigned int *tempKeyAfter = new unsigned int[arraySize];
	//unsigned int *tempValueBefore = new unsigned int[arraySize];
	//unsigned int *tempValueAfter = new unsigned int[arraySize];
	//for (int i=0; i<arraySize; i++)
	//{
	//	if (i<2210)
	//	{
	//		tempKeyBefore[i] = i;
	//		tempValueBefore[i] = arraySize-i-1;
	//	}
	//	else
	//	{
	//		tempKeyBefore[i] = 0;
	//		tempValueBefore[i] = 0;
	//	}
	//}
	//tempKeyBefore[0] =
	//	tempKeyBefore[1] =
	//	tempKeyBefore[2] =
	//	tempKeyBefore[3] =
	//	tempKeyBefore[4] =
	//	tempKeyBefore[5] = 5;

	//b2CLDevice::instance().copyArrayToDevice(keyBuffer, tempKeyBefore, 0, sizeof(unsigned int)*arraySize, true);
	//b2CLDevice::instance().copyArrayToDevice(valueBuffer, tempValueBefore, 0, sizeof(unsigned int)*arraySize, true);
	//b2CLDevice::instance().copyArrayFromDevice(tempKeyBefore, keyBuffer, 0, sizeof(unsigned int)*arraySize, true);
	//b2CLDevice::instance().copyArrayFromDevice(tempValueBefore, valueBuffer, 0, sizeof(unsigned int)*arraySize, true);
	//sortAscending = 1;

    cl_int err = CL_SUCCESS;
    cl_int numStages = 0;
    cl_uint temp;

    cl_int stage;
    cl_int passOfStage;

    for (temp = arraySize; temp > 2; temp >>= 1)
    {
        numStages++;
    }

    err  = clSetKernelArg(sortKernel_Intel, 0, sizeof(cl_mem), (void *) &keyBuffer);
    err |= clSetKernelArg(sortKernel_Intel, 1, sizeof(cl_mem), (void *) &valueBuffer);
    err |= clSetKernelArg(sortKernel_Intel, 4, sizeof(cl_uint), (void *) &sortAscending);
    if (err != CL_SUCCESS)
    {
        printf("ERROR: Failed to set input kernel arguments\n");
        return;
    }

    for (stage = 0; stage < numStages; stage++)
    {
        if ( CL_SUCCESS != clSetKernelArg(sortKernel_Intel, 2, sizeof(cl_uint), (void *) &stage) )
            return;

        for (passOfStage = stage; passOfStage >= 0; passOfStage--)
        {
            if ( CL_SUCCESS != clSetKernelArg(sortKernel_Intel, 3, sizeof(cl_uint), (void *) &passOfStage) )
                return;

            // set work-item dimensions
            size_t gsz = arraySize / (2*4);
			int numWorkItems = passOfStage ? gsz : gsz << 1;
		    int numBlocks = (numWorkItems + kernel_work_group_size_Intel-1)/kernel_work_group_size_Intel;
		    size_t global = numBlocks * kernel_work_group_size_Intel;

            size_t global_work_size[1] = { passOfStage ? gsz : gsz << 1 };	//number of quad items in input array
            size_t local_work_size[1]= { 128 };					//valid WG sizes are 1:1024

            if ( CL_SUCCESS != clSetKernelArg(sortKernel_Intel, 5, sizeof(cl_uint), (void *) &numWorkItems) )
                return;
            // execute kernel
			//err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernel_Intel, 1, NULL, global_work_size, local_work_size, 0, NULL, NULL);
			err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), sortKernel_Intel, 1, NULL, &global, &kernel_work_group_size_Intel, 0, NULL, NULL);
            if (CL_SUCCESS != err)
            {
                printf("ERROR: Failed to execute sorting kernel\n");
                return;
            }
			//b2CLDevice::instance().finishCommandQueue();
			//int a = 1;
        }
    }

	//// for debug
	//b2CLDevice::instance().copyArrayFromDevice(tempKeyAfter, keyBuffer, 0, sizeof(unsigned int)*arraySize, true);
	//b2CLDevice::instance().copyArrayFromDevice(tempValueAfter, valueBuffer, 0, sizeof(unsigned int)*arraySize, true);
	//delete [] tempKeyBefore;
	//delete [] tempKeyAfter;
	//delete [] tempValueBefore;
	//delete [] tempValueAfter;
}

#if defined(USE_CPU_SORT)

void b2CLSort::stlSort(cl_mem keyBuffer, cl_mem valueBuffer, cl_int arraySize, cl_uint sortAscending, cl_uint storeKeys)
{
	if (old_sort_num<arraySize)
	{
		if (keys)
			delete [] keys;
		keys = new unsigned int[arraySize];
		if (values)
			delete [] values;
		values = new unsigned int[arraySize];
		if (elements)
			delete [] elements;
		elements = new element_type[arraySize];

		old_sort_num = arraySize;
	}
	b2CLDevice::instance().copyArrayFromDevice(keys, keyBuffer, 0, sizeof(unsigned int)*arraySize, true);
	b2CLDevice::instance().copyArrayFromDevice(values, valueBuffer, 0, sizeof(unsigned int)*arraySize, true);
	for (int i=0; i<arraySize; i++)
	{
		elements[i].key = keys[i];
		elements[i].value = values[i];
	}

	std::sort(elements, elements+arraySize, CompareElementsForSort);

	for (int i=0; i<arraySize; i++)
	{
		if (sortAscending)
		{
			values[i] = elements[i].value;
			if (storeKeys)
				keys[i] = elements[i].key;
		}
		else
		{
			values[i] = elements[arraySize-i-1].value;
			if (storeKeys)
				keys[i] = elements[arraySize-i-1].key;
		}
	}
	// APP Profiler said no need for block
	b2CLDevice::instance().copyArrayToDevice(valueBuffer, values, 0, sizeof(unsigned int)*arraySize, false);
	if (storeKeys)
		b2CLDevice::instance().copyArrayToDevice(keyBuffer, keys, 0, sizeof(unsigned int)*arraySize, false);
}

#endif