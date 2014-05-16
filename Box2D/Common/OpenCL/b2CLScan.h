/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLScan_h
#define Box2D_b2CLScan_h

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

class b2CLScan
{
	enum ScanKernelMethods
	{
		PRESCAN,
		PRESCAN_STORE_SUM,
		PRESCAN_STORE_SUM_NON_POWER_OF_TWO,
		PRESCAN_NON_POWER_OF_TWO,
		UNIFORM_ADD,
		PARALLEL_COMPACT,
		PARALLEL_COMPACTINDICES,
		PARALLEL_COMPATCGENERAL,
		SCANKERNELCOUNT
	};

public: 
    b2CLScan();
    ~b2CLScan();

	static b2CLScan& instance();
	void PreScanBuffer(cl_mem input_data, int element_count);
	void PreScanBuffer(cl_mem output_data, cl_mem input_data, int element_count);
	int ParallelCompact(cl_mem output_data, cl_mem input_data, cl_mem scan_result, int element_count, cl_mem num_usefuldata=NULL);
	int ParallelCompactIndices(cl_mem output_data, cl_mem input_data, cl_mem scan_result, int element_count, cl_mem num_usefuldata=NULL);
	int ParallelCompactGeneral(cl_mem output_data, cl_mem input_data, cl_mem bit_data, cl_mem scan_result, int element_count, cl_mem num_usefuldata=NULL);

	void ScanCLPP(cl_mem input_data, int element_count);
    
#if defined(USE_CPU_SCAN)
    int GetElementsAllocated() { return ElementsAllocated; };
#endif

	cl_mem scanResultsBuffer;
	cl_mem numValidDataBuffer;

private:
	static const char* ScanKernelNames[];

	cl_program scanProgram;
    cl_kernel scanKernels[SCANKERNELCOUNT];
	size_t kernel_work_group_size;

	cl_kernel scanKernelCLPP, uniformAddKernelCLPP;
	size_t kernel_work_group_size_scan_CLPP, kernel_work_group_size_add_CLPP;

    cl_mem* ScanPartialSums;
	int LevelsAllocated;
	int ElementsAllocated;
	int *SumsSizes;

	inline bool IsPowerOfTwo(int n) { return ((n&(n-1))==0); }
	inline int floorPow2(int n) 
	{
		int exp;
		frexp((float)n, &exp);
		return 1 << (exp - 1);
	}
	static size_t toMultipleOf(size_t N, size_t base) 
	{
		return (ceil((double)N / (double)base) * base);
	}

	int CreatePartialSumBuffers(unsigned int count);
	void ReleasePartialSums(void);
	int PreScan(size_t *global, size_t *local, size_t shared, cl_mem output_data, cl_mem input_data, unsigned int n,int group_index, int base_index);
	int PreScanStoreSum(size_t *global, size_t *local, size_t shared, cl_mem output_data, cl_mem input_data, cl_mem partial_sums, unsigned int n, int group_index, int base_index);
	int PreScanStoreSumNonPowerOfTwo(size_t *global, size_t *local, size_t shared, cl_mem output_data, cl_mem input_data, cl_mem partial_sums, unsigned int n, int group_index, int base_index);
	int PreScanNonPowerOfTwo(size_t *global, size_t *local, size_t shared, cl_mem output_data, cl_mem input_data, unsigned int n, int group_index, int base_index);
	int UniformAdd(size_t *global, size_t *local, cl_mem output_data, cl_mem partial_sums, unsigned int n, unsigned int group_offset, unsigned int base_index);

	int PreScanBufferRecursive(cl_mem output_data, cl_mem input_data, unsigned int max_group_size, unsigned int max_work_item_count, unsigned int element_count, int level);
};

class b2CLScanFloat4
{
	enum
	{
		BLOCK_SIZE = 128
	};

	enum ScanKernelMethods
	{
		LOCALSCAN,
		ADDOFFSET,
		TOPLEVELSCAN,
		SCANKERNELCOUNT
	};

public: 
	static const char* ScanKernelNames[];

    b2CLScanFloat4();
    ~b2CLScanFloat4();

	int Scan(cl_mem input_data, int element_count);

	static b2CLScanFloat4& instance();
	cl_mem scanResultsBuffer;
	cl_mem numValidDataBuffer;

private:
	cl_program scanProgram;
    cl_kernel scanKernels[SCANKERNELCOUNT];
	size_t kernel_work_group_size;

    cl_mem WorkBuffer;
	int old_workbuffer_size;

	template<class T>
	T nextPowerOf2(T n)
	{
		n -= 1;
		for(int i=0; i<sizeof(T)*8; i++)
			n = n | (n>>i);
		return n+1;
	}

	int Scan();
};

#endif
