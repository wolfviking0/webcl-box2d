/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLSort_h
#define Box2D_b2CLSort_h

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Common/OpenCL/b2CLCommonData.h>

#ifdef _DEBUG_TIME_NARROWPHASE
#include <Box2D/Common/b2Timer.h>
#endif

//#define min(A,B) ((A) < (B) ? (A) : (B))
#if defined (_WIN32)
#define fmax(A,B) ((A) > (B) ? (A) : (B))
#endif

#define NUM_BANKS       (16)

#if defined(USE_CPU_SORT)
	typedef struct{
		unsigned int key;
		unsigned int value;
	}element_type;

	/// This is used to sort pairs.
	inline bool CompareElementsForSort(const element_type& e1, const element_type& e2)
	{
		return e1.key<e2.key;
	}
#endif

class b2CLSort
{
	enum SortKernelMethods
	{
		BITONICSORTLOCAL,
		BITONICSORTLOCAL1,
		BITONICMERGEGLOBAL,
		BITONICMERGELOCAL,
		SORTKERNELCOUNT
	};

public: 
    b2CLSort();
    ~b2CLSort();

	static b2CLSort& instance();
	void bitonicSort_NV(cl_mem d_DstKey, cl_mem d_DstVal, cl_mem d_SrcKey, cl_mem d_SrcVal, unsigned int batch, unsigned int arrayLength, unsigned int dir);
	void bitonicSort_Intel(cl_mem keyBuffer, cl_mem valueBuffer, cl_int arraySize, cl_uint sortAscending);
	void stlSort(cl_mem keyBuffer, cl_mem valueBuffer, cl_int arraySize, cl_uint sortAscending, cl_uint storeKeys = 0);
	//cl_mem scanResultsBuffer;
	//cl_mem numValidDataBuffer;

private:
	static const char* SortKernelNames_NV[];

	cl_program sortProgram;
    cl_kernel sortKernels_NV[SORTKERNELCOUNT];
	cl_kernel sortKernel_Intel;
	size_t kernel_work_group_size_NV, kernel_work_group_size_Intel;

	inline bool IsPowerOfTwo(int n) { return ((n&(n-1))==0); }

#if defined(USE_CPU_SORT)
	int old_sort_num;
	unsigned int *keys, *values;
	element_type *elements;
#endif
};

#endif
