/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/



#include "b2CLDevice.h"
#include <fcntl.h>
#include <sys/stat.h>

//Note: logically shared with BitonicSort_b.cl!

b2CLDevice& b2CLDevice::instance()
{
	static b2CLDevice device;
	return device;
}

b2CLDevice::b2CLDevice()
{
	startupOpenCL();
	//initBitonicSort();
}

b2CLDevice::~b2CLDevice()
{
    free( cdDevices );
	//closeBitonicSort();
}

void b2CLDevice::startupOpenCL()
{
	cl_uint uiNumPlatforms;
	cl_platform_id* cpPlatforms;
	cl_uint uiTargetPlatform;
	cl_uint  uiNumDevices;
	cl_int  ciErrNum;
	char infoStr[1024];
	size_t infoLen;
	cl_device_type infoType;
    int firstCPU, firstGPU;
    //cl_int  status;

    firstCPU = firstGPU = -1;
    
	// Get the platform
	shrLog("clGetPlatformIDs...\n");
	clGetPlatformIDs(0, NULL, &uiNumPlatforms);

	cpPlatforms = (cl_platform_id *)malloc(uiNumPlatforms * sizeof(cl_platform_id));

	ciErrNum = clGetPlatformIDs(uiNumPlatforms, cpPlatforms, &uiNumPlatforms);
	b2clCheckError(ciErrNum, CL_SUCCESS);

	shrLog("Total number of platforms: %d\n", uiNumPlatforms);

	for (int i=0; i<uiNumPlatforms; i++)
	{
		clGetPlatformInfo(cpPlatforms[i], CL_PLATFORM_VENDOR, sizeof(infoStr), infoStr, &infoLen);
		shrLog("Platform %d vendor: %s\n", i, infoStr);
		clGetPlatformInfo(cpPlatforms[i], CL_PLATFORM_NAME, sizeof(infoStr), infoStr, &infoLen);
		shrLog("Platform %d name: %s\n\n", i, infoStr);
	}

#if defined(_INTEL_PLATFORM)
	uiTargetPlatform = 1;
#else
	uiTargetPlatform = 0;
#endif
	shrLog("Using Platform %u, ", uiTargetPlatform);

    //---------------------------------------------------------------
    // Discover and initialize devices
    //---------------------------------------------------------------
	shrLog("clGetDeviceIDs...\n\n");

	ciErrNum = clGetDeviceIDs(cpPlatforms[uiTargetPlatform], CL_DEVICE_TYPE_ALL, 0, NULL, &uiNumDevices);
    
	b2clCheckError(ciErrNum, CL_SUCCESS);
    
    // Allocate space for each device
	cdDevices = (cl_device_id *)malloc(uiNumDevices * sizeof(cl_device_id) );

	ciErrNum = clGetDeviceIDs(cpPlatforms[uiTargetPlatform], CL_DEVICE_TYPE_ALL, uiNumDevices, cdDevices, NULL);

	b2clCheckError(ciErrNum, CL_SUCCESS);

	shrLog("Total number of devices: %d\n", uiNumDevices);
	for (int i=0; i<uiNumDevices; i++)
	{
		shrLog("Device #%d is " , i);
		clGetDeviceInfo(cdDevices[i], CL_DEVICE_TYPE, sizeof(infoType), &infoType, &infoLen);
		if (infoType & CL_DEVICE_TYPE_CPU)
		{
			shrLog("CPU\n\t");
            if (firstCPU<0)
                firstCPU = i;
		}
		if (infoType & CL_DEVICE_TYPE_GPU)
		{
			shrLog("GPU\n\t");
            if (firstGPU<0)
                firstGPU = i;
		}
		clGetDeviceInfo(cdDevices[i], CL_DEVICE_VENDOR, sizeof(infoStr), infoStr, &infoLen);
		shrLog("Device vendor: %s\n\t", infoStr);
		clGetDeviceInfo(cdDevices[i], CL_DEVICE_NAME, sizeof(infoStr), infoStr, &infoLen);
		shrLog("Device name: %s\n\t", infoStr);

		clGetDeviceInfo(cdDevices[i], CL_DEVICE_VERSION, sizeof(infoStr), infoStr, &infoLen);
		shrLog("Supported OpenCL version: %s\n\t", infoStr);

		size_t returned_size = 0;
		size_t max_workgroup_size = 0;
		clGetDeviceInfo(cdDevices[i], CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &max_workgroup_size, &returned_size);
		shrLog("Max work group size: %d\n\t", max_workgroup_size);
        
        size_t max_compute_units=0;
        clGetDeviceInfo(cdDevices[i], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(size_t), &max_compute_units, &returned_size);
        shrLog("Max number of compute units: %d\n", max_compute_units);
	}
	shrLog("\n");

	assert( firstCPU>=0 || firstGPU>=0 );

	// Set target device and Query number of compute units on uiTargetDevice
#ifdef BOX2D_OPENCL_ON_CPU
	if (firstCPU<0)
	{
		printf("The platform does not support OpenCL on CPU!\n");
		exit(1);
	}
    uiTargetDevice = firstCPU;
#else
	if (firstGPU<0)
	{
		printf("The platform does not support OpenCL on GPU!\n");
		exit(1);
	}
	uiTargetDevice = firstGPU;
#endif
	shrLog("Using Device %u, ", uiTargetDevice);
//#ifdef SHR_UTILS_H
//	b2clPrintDevName(LOGBOTH, cdDevices[uiTargetDevice]);
//#endif

	currentDevice = cdDevices[uiTargetDevice];

	clGetDeviceInfo(currentDevice, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &m_deviceMaxWorkGroupSize, NULL);

	// Create the context
	shrLog("\n\nclCreateContext...\n\n");
	m_context = clCreateContext(0, 1, &currentDevice, NULL, NULL, &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);

	//Create a command-queue
	shrLog("clCreateCommandQueue...\n\n");
	m_commandQueue = clCreateCommandQueue(m_context, currentDevice, 0, &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);

    free(cpPlatforms);
}

void b2CLDevice::initBitonicSort()
{
	cl_int ciErrNum;
	size_t kernelLength;    
    
	shrLog("...loading BitonicSort_b.cl\n");

#ifdef linux
    char* cBitonicSort = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/BitonicSort_b.cl", NULL), "// My comment\n", &kernelLength);
#elif defined (_WIN32)
    char* cBitonicSort = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/BitonicSort_b.cl", NULL), "// My comment\n", &kernelLength);
#elif defined (__EMSCRIPTEN__)
    char* cBitonicSort = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/BitonicSort_b.cl", NULL), "// My comment\n", &kernelLength);
#else
    char* cBitonicSort = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/BitonicSort_b.cl", NULL), "// My comment\n", &kernelLength);
#endif
    
	if(cBitonicSort == NULL)
	{
		b2Log("Could not load program source, is path '../../Box2D/Common/OpenCL/BitonicSort_b.cl' correct?");
	}

	b2clCheckError(cBitonicSort != NULL, shrTRUE);

	shrLog("...creating bitonic sort program\n");
	m_bitonicSortProgram = clCreateProgramWithSource(m_context, 1, (const char **)&cBitonicSort, &kernelLength, &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);

	shrLog("...building bitonic sort program\n");
	ciErrNum = clBuildProgram(m_bitonicSortProgram, 0, NULL, NULL, NULL, NULL);
	if (ciErrNum != CL_SUCCESS)
	{
//#ifdef SHR_UTILS_H
		// write out standard error, Build Log and PTX, then cleanup and exit
//		shrLogEx(LOGBOTH | ERRORMSG, ciErrNum, STDERROR);
//		b2clLogBuildInfo(m_bitonicSortProgram, b2clGetFirstDev(m_context));
//		b2clLogPtx(m_bitonicSortProgram, b2clGetFirstDev(m_context), "b2clBitonicSort.ptx");
//#endif
		b2clCheckError(ciErrNum, CL_SUCCESS);
	}

	shrLog("...creating bitonic sort kernels\n");
	m_bitonicSortLocal = clCreateKernel(m_bitonicSortProgram, "bitonicSortLocal", &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);
	m_bitonicSortLocal1 = clCreateKernel(m_bitonicSortProgram, "bitonicSortLocal1", &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);
	m_bitonicMergeGlobal = clCreateKernel(m_bitonicSortProgram, "bitonicMergeGlobal", &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);
	m_bitonicMergeLocal = clCreateKernel(m_bitonicSortProgram, "bitonicMergeLocal", &ciErrNum);
	b2clCheckError(ciErrNum, CL_SUCCESS);
    
    getMaximumKernelWorkGroupSize(m_bitonicSortLocal, maxWorkGroupSizeForBitonicSort);
    size_t maxWorkGroupSizeForBitonicSortTmp;
    getMaximumKernelWorkGroupSize(m_bitonicSortLocal1, maxWorkGroupSizeForBitonicSortTmp);
    maxWorkGroupSizeForBitonicSort=maxWorkGroupSizeForBitonicSort<maxWorkGroupSizeForBitonicSortTmp ? maxWorkGroupSizeForBitonicSort : maxWorkGroupSizeForBitonicSortTmp;
    getMaximumKernelWorkGroupSize(m_bitonicMergeGlobal, maxWorkGroupSizeForBitonicSortTmp);
    maxWorkGroupSizeForBitonicSort=maxWorkGroupSizeForBitonicSort<maxWorkGroupSizeForBitonicSortTmp ? maxWorkGroupSizeForBitonicSort : maxWorkGroupSizeForBitonicSortTmp;
    getMaximumKernelWorkGroupSize(m_bitonicMergeLocal, maxWorkGroupSizeForBitonicSortTmp);
    maxWorkGroupSizeForBitonicSort=maxWorkGroupSizeForBitonicSort<maxWorkGroupSizeForBitonicSortTmp ? maxWorkGroupSizeForBitonicSort : maxWorkGroupSizeForBitonicSortTmp;
    
	//Discard temp storage
#ifndef __APPLE__
	free(cBitonicSort);
#endif
}

void b2CLDevice::closeBitonicSort()
{
	cl_int ciErrNum;
	ciErrNum  = clReleaseKernel(m_bitonicMergeLocal);
	ciErrNum |= clReleaseKernel(m_bitonicMergeGlobal);
	ciErrNum |= clReleaseKernel(m_bitonicSortLocal1);
	ciErrNum |= clReleaseKernel(m_bitonicSortLocal);
	ciErrNum |= clReleaseProgram(m_bitonicSortProgram);
	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::getMaximumKernelWorkGroupSize(cl_kernel& kernelName, size_t& workGroupSize)
{
    cl_int error = clGetKernelWorkGroupInfo(kernelName,cdDevices[uiTargetDevice],CL_KERNEL_WORK_GROUP_SIZE,sizeof(size_t),&workGroupSize,NULL);
    b2clCheckError(error, CL_SUCCESS);
}

cl_mem b2CLDevice::allocateArray(size_t size, bool print_log)
{
	cl_int error;
	if(print_log) shrLog(" clCreateBuffer (GPU GMEM, %u bytes)...\n\n", size);
	cl_mem result = clCreateBuffer(m_context, CL_MEM_READ_WRITE, size, NULL, &error);
	b2clCheckError(error, CL_SUCCESS);
	return result;
}

cl_mem b2CLDevice::allocateArray(size_t size, void* host_buffer, bool print_log)
{
	cl_int error;
	if(print_log) shrLog(" clCreateBuffer (GPU GMEM, %u bytes)...\n\n", size);
	cl_mem result = clCreateBuffer(m_context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, size, host_buffer, &error);
	b2clCheckError(error, CL_SUCCESS);
	return result;
}

void b2CLDevice::freeArray(cl_mem memObj)
{
	cl_int ciErrNum;
	ciErrNum = clReleaseMemObject(memObj);
	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayFromDevice(void *hostPtr, cl_mem memObj, unsigned int vbo, size_t size)
{
	cl_int ciErrNum;
	assert( vbo == 0 );
	ciErrNum = clEnqueueReadBuffer(m_commandQueue, memObj, CL_TRUE, 0, size, hostPtr, 0, NULL, NULL);
	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayFromDevice(void *hostPtr, cl_mem memObj, unsigned int vbo, size_t size, bool blocking)
{
	cl_int ciErrNum;
	assert( vbo == 0 );

	ciErrNum = clEnqueueReadBuffer(m_commandQueue, memObj, blocking ? CL_TRUE : CL_FALSE, 0, size, hostPtr, 0, NULL, NULL);
	//void *clientPtr = clEnqueueMapBuffer(m_commandQueue, memObj, blocking ? CL_TRUE : CL_FALSE, CL_MAP_READ, 0, size, 0, NULL, NULL, &ciErrNum);
	//memcpy(hostPtr, clientPtr, size);
	//clEnqueueUnmapMemObject(m_commandQueue, memObj, clientPtr, 0, NULL, NULL);

	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayInsideDevice(cl_mem src_memObj, cl_mem dst_memObj, size_t size)
{
	cl_int ciErrNum;

	ciErrNum = clEnqueueCopyBuffer(m_commandQueue, src_memObj, dst_memObj, 0, 0, size, 0, NULL, NULL );

	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayInsideDevice(cl_mem src_memObj, cl_mem dst_memObj, size_t src_offset, size_t dst_offset, size_t size)
{
	cl_int ciErrNum;

	ciErrNum = clEnqueueCopyBuffer(m_commandQueue, src_memObj, dst_memObj, src_offset, dst_offset, size, 0, NULL, NULL );

	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayToDevice(cl_mem memObj, const void *hostPtr, size_t offset, size_t size)
{
	cl_int ciErrNum;
	ciErrNum = clEnqueueWriteBuffer(m_commandQueue, memObj, CL_TRUE, 0, size, hostPtr, offset, NULL, NULL);
	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::copyArrayToDevice(cl_mem memObj, const void *hostPtr, size_t offset, size_t size, bool blocking)
{
	cl_int ciErrNum;
	ciErrNum = clEnqueueWriteBuffer(m_commandQueue, memObj, blocking ? CL_TRUE : CL_FALSE, offset, size, hostPtr, 0, NULL, NULL);
	b2clCheckError(ciErrNum, CL_SUCCESS);
}

void b2CLDevice::finishCommandQueue()
{
	cl_int error = clFinish(m_commandQueue);
	b2clCheckError(error, CL_SUCCESS);
}

static cl_uint factorRadix2(cl_uint& log2L, cl_uint L)
{
    if(!L)
    {
        log2L = 0;
        return 0;
    }
    else
    {
        for(log2L = 0; (L & 1) == 0; L >>= 1, log2L++); // empty for
        return L;
    }
}

void b2CLDevice::bitonicSort(
//    cl_command_queue cqCommandQueue,
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
	cl_uint log2L;
	cl_uint factorizationRemainder = factorRadix2(log2L, arrayLength);
	b2clCheckError( factorizationRemainder == 1, shrTRUE );

//	if(!cqCommandQueue)
//		cqCommandQueue = m_commandQueue;

	dir = (dir != 0);

	cl_int ciErrNum;
	size_t localWorkSize, globalWorkSize;

	if(arrayLength <= maxWorkGroupSizeForBitonicSort)
	{
		b2clCheckError( (batch * arrayLength) % maxWorkGroupSizeForBitonicSort == 0, shrTRUE );
		//Launch bitonicSortLocal
		ciErrNum  = clSetKernelArg(m_bitonicSortLocal, 0,   sizeof(cl_mem), (void *)&d_DstKey);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal, 1,   sizeof(cl_mem), (void *)&d_DstVal);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal, 2,   sizeof(cl_mem), (void *)&d_SrcKey);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal, 3,   sizeof(cl_mem), (void *)&d_SrcVal);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal, 4,  sizeof(cl_uint), (void *)&arrayLength);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal, 5,  sizeof(cl_uint), (void *)&dir);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		localWorkSize  = maxWorkGroupSizeForBitonicSort / 2;
		globalWorkSize = batch * arrayLength / 2;
		ciErrNum = clEnqueueNDRangeKernel(m_commandQueue, m_bitonicSortLocal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);
	}
	else
	{
		//Launch bitonicSortLocal1
		ciErrNum  = clSetKernelArg(m_bitonicSortLocal1, 0,  sizeof(cl_mem), (void *)&d_DstKey);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal1, 1,  sizeof(cl_mem), (void *)&d_DstVal);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal1, 2,  sizeof(cl_mem), (void *)&d_SrcKey);
		ciErrNum |= clSetKernelArg(m_bitonicSortLocal1, 3,  sizeof(cl_mem), (void *)&d_SrcVal);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		localWorkSize  = maxWorkGroupSizeForBitonicSort / 2;
		globalWorkSize = batch * arrayLength / 2;
		ciErrNum = clEnqueueNDRangeKernel(m_commandQueue, m_bitonicSortLocal1, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);

		for(unsigned int size = 2 * maxWorkGroupSizeForBitonicSort; size <= arrayLength; size <<= 1)
		{
			for(unsigned stride = size / 2; stride > 0; stride >>= 1)
			{
				if(stride >= maxWorkGroupSizeForBitonicSort)
				{
					//Launch bitonicMergeGlobal
					ciErrNum  = clSetKernelArg(m_bitonicMergeGlobal, 0,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 1,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 2,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 3,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 4, sizeof(cl_uint), (void *)&arrayLength);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 5, sizeof(cl_uint), (void *)&size);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 6, sizeof(cl_uint), (void *)&stride);
					ciErrNum |= clSetKernelArg(m_bitonicMergeGlobal, 7, sizeof(cl_uint), (void *)&dir);
					b2clCheckError(ciErrNum, CL_SUCCESS);

					localWorkSize  = maxWorkGroupSizeForBitonicSort / 4;
					globalWorkSize = batch * arrayLength / 2;

					ciErrNum = clEnqueueNDRangeKernel(m_commandQueue, m_bitonicMergeGlobal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					b2clCheckError(ciErrNum, CL_SUCCESS);
				}
				else
				{
					//Launch bitonicMergeLocal
					ciErrNum  = clSetKernelArg(m_bitonicMergeLocal, 0,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 1,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 2,  sizeof(cl_mem), (void *)&d_DstKey);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 3,  sizeof(cl_mem), (void *)&d_DstVal);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 4, sizeof(cl_uint), (void *)&arrayLength);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 5, sizeof(cl_uint), (void *)&stride);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 6, sizeof(cl_uint), (void *)&size);
					ciErrNum |= clSetKernelArg(m_bitonicMergeLocal, 7, sizeof(cl_uint), (void *)&dir);
					b2clCheckError(ciErrNum, CL_SUCCESS);

					localWorkSize  = maxWorkGroupSizeForBitonicSort / 2;
					globalWorkSize = batch * arrayLength / 2;

					ciErrNum = clEnqueueNDRangeKernel(m_commandQueue, m_bitonicMergeLocal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					b2clCheckError(ciErrNum, CL_SUCCESS);
					break;
				}
			}
		}
	}
}
