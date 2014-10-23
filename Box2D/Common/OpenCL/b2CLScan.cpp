/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#include <Box2D/Common/OpenCL/b2CLScan.h>
#include <Box2D/Common/OpenCL/b2CLDevice.h>
#ifdef linux
#include <cstdlib>
#include <string.h>
#include <limits.h>
#endif

const char* b2CLScan::ScanKernelNames[] = 
{
	"PreScanKernel",
	"PreScanStoreSumKernel",
	"PreScanStoreSumNonPowerOfTwoKernel",
	"PreScanNonPowerOfTwoKernel",
	"UniformAddKernel",
	"ParallelCompactKernel",
	"ParallelCompactIndicesKernel",
	"ParallelCompactGeneralKernel"
};

b2CLScan::b2CLScan()
{
#if defined(SCAN_OPENCL)
    printf("Initializing b2CLScan...\n");
    
    int err;
    
    //load opencl programs from files
    char* scanKernelSource = 0;
    size_t scanKernelSourceLen = 0;

	// Load kernels from Apple
    shrLog("...loading b2CLScan.cl\n");

#ifdef linux
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLScan.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (_WIN32)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLScan.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLScan.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#else
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLScan.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#endif
    
	if(scanKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLScan.cl' correct?");
	}

    //create the compute program from source kernel code
    scanProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&scanKernelSource, NULL, &err);
    if (!scanProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(scanProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(scanProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
    
    //create the compute kernel
	kernel_work_group_size = INT_MAX;
    for(int i = 0; i < SCANKERNELCOUNT; i++)
    {    
        // Create each kernel from within the compactionProgram
        scanKernels[i] = clCreateKernel(scanProgram, ScanKernelNames[i], &err);
        if (!scanKernels[i] || err != CL_SUCCESS)
        {
            printf("Error: Failed to create compute kernel!\n");
            exit(1);
        }
		size_t size;
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(scanKernels[i], size);
		kernel_work_group_size = kernel_work_group_size>size ? size : kernel_work_group_size;
    }

	// Load kernels from CLPP
    shrLog("...loading b2CLScan_CLPP.cl\n");
    
#ifdef linux
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLScan_CLPP.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (_WIN32)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLScan_CLPP.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLScan_CLPP.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#else
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLScan_CLPP.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#endif
    
	if(scanKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLScan_CLPP.cl' correct?");
	}

    //create the compute program from source kernel code
    scanProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&scanKernelSource, NULL, &err);
    if (!scanProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(scanProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(scanProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
    
    //create the compute kernel
    scanKernelCLPP = clCreateKernel(scanProgram, "kernel__ExclusivePrefixScan", &err);
    if (!scanKernelCLPP || err != CL_SUCCESS)
    {
        printf("Error: Failed to create compute kernel kernel__ExclusivePrefixScan!\n");
        exit(1);
    }
    b2CLDevice::instance().getMaximumKernelWorkGroupSize(scanKernelCLPP, kernel_work_group_size_scan_CLPP);

    uniformAddKernelCLPP = clCreateKernel(scanProgram, "kernel__UniformAdd", &err);
    if (!uniformAddKernelCLPP || err != CL_SUCCESS)
    {
        printf("Error: Failed to create compute kernel kernel__UniformAdd!\n");
        exit(1);
    }
    b2CLDevice::instance().getMaximumKernelWorkGroupSize(uniformAddKernelCLPP, kernel_work_group_size_add_CLPP);

	ElementsAllocated = LevelsAllocated = 0;

	scanResultsBuffer = NULL;

	numValidDataBuffer = NULL;
#endif
}

b2CLScan::~b2CLScan()
{
	ReleasePartialSums();
}

b2CLScan& b2CLScan::instance()
{
	static b2CLScan inst;
	return inst;
}

int b2CLScan::CreatePartialSumBuffers(unsigned int count)
{
	if (ElementsAllocated>0)
		ReleasePartialSums();

    ElementsAllocated = count;

	unsigned int group_size = kernel_work_group_size;
    unsigned int element_count = count;

    int level = 0;

    do
    {       
        unsigned int group_count = (int)fmax(1, (int)ceil((float)element_count / (2.0f * group_size)));
        if (group_count > 1)
        {
            level++;
        }
        element_count = group_count;
        
    } while (element_count > 1);

    ScanPartialSums = (cl_mem*) malloc(level * sizeof(cl_mem));
    LevelsAllocated = level;
    memset(ScanPartialSums, 0, sizeof(cl_mem) * level);
    
    element_count = count;
    level = 0;
    
    do
    {       
        unsigned int group_count = (int)fmax(1, (int)ceil((float)element_count / (2.0f * group_size)));
        if (group_count > 1) 
        {
            size_t buffer_size = group_count * sizeof(float);
            ScanPartialSums[level++] = b2CLDevice::instance().allocateArray(buffer_size);
        }

        element_count = group_count;

    } while (element_count > 1);

	// allocate a buffer to store the scan results
	if (scanResultsBuffer)
		b2CLDevice::instance().freeArray(scanResultsBuffer);
	scanResultsBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*count);

    return CL_SUCCESS;
}

void b2CLScan::ReleasePartialSums(void)
{
    unsigned int i;
    for (i = 0; i < LevelsAllocated; i++)
    {
        clReleaseMemObject(ScanPartialSums[i]);
    }    
    
    free(ScanPartialSums);
    ScanPartialSums = 0;
    ElementsAllocated = 0;
    LevelsAllocated = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int b2CLScan::PreScan(
    size_t *global, 
    size_t *local, 
    size_t shared, 
    cl_mem output_data, 
    cl_mem input_data, 
    unsigned int n,
    int group_index, 
    int base_index)
{
    unsigned int k = PRESCAN;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, shared,         0);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int b2CLScan::PreScanStoreSum(
    size_t *global, 
    size_t *local, 
    size_t shared, 
    cl_mem output_data, 
    cl_mem input_data, 
    cl_mem partial_sums,
    unsigned int n,
    int group_index, 
    int base_index)
{
    unsigned int k = PRESCAN_STORE_SUM;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(scanKernels[k],  a++, shared,         0);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }
    
    return CL_SUCCESS;
}

int b2CLScan::PreScanStoreSumNonPowerOfTwo(
    size_t *global, 
    size_t *local, 
    size_t shared, 
    cl_mem output_data, 
    cl_mem input_data, 
    cl_mem partial_sums,
    unsigned int n,
    int group_index, 
    int base_index)
{
    unsigned int k = PRESCAN_STORE_SUM_NON_POWER_OF_TWO;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(scanKernels[k],  a++, shared,         0);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int b2CLScan::PreScanNonPowerOfTwo(
    size_t *global, 
    size_t *local, 
    size_t shared, 
    cl_mem output_data, 
    cl_mem input_data, 
    unsigned int n,
    int group_index, 
    int base_index)
{
    unsigned int k = PRESCAN_NON_POWER_OF_TWO;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, shared,         0);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }
    return CL_SUCCESS;
}

int b2CLScan::UniformAdd(
    size_t *global, 
    size_t *local, 
    cl_mem output_data, 
    cl_mem partial_sums, 
    unsigned int n, 
    unsigned int group_offset, 
    unsigned int base_index)
{
    unsigned int k = UNIFORM_ADD;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(float),  0);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &group_offset);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int b2CLScan::PreScanBufferRecursive(
                        cl_mem output_data, 
                        cl_mem input_data, 
                        unsigned int max_group_size,
                        unsigned int max_work_item_count,
                        unsigned int element_count, 
                        int level)
{
    unsigned int group_size = max_group_size; 
    unsigned int group_count = (int)fmax(1.0f, (int)ceil((float)element_count / (2.0f * group_size)));
    unsigned int work_item_count = 0;

    if (group_count > 1)
        work_item_count = group_size;
    else if (IsPowerOfTwo(element_count))
        work_item_count = element_count / 2;
    else
        work_item_count = floorPow2(element_count);
        
    work_item_count = (work_item_count > max_work_item_count) ? max_work_item_count : work_item_count;

    unsigned int element_count_per_group = work_item_count * 2;
    unsigned int last_group_element_count = element_count - (group_count-1) * element_count_per_group;
    unsigned int remaining_work_item_count = (int)fmax(1.0f, last_group_element_count / 2);
    remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
    unsigned int remainder = 0;
    size_t last_shared = 0;

	//// for debug
	//float *temp = new float[element_count];
    
    if (last_group_element_count != element_count_per_group)
    {
        remainder = 1;

        if(!IsPowerOfTwo(last_group_element_count))
            remaining_work_item_count = floorPow2(last_group_element_count);    
        
        remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
        unsigned int padding = (2 * remaining_work_item_count) / NUM_BANKS;
        last_shared = sizeof(float) * (2 * remaining_work_item_count + padding);
    }

    remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
    size_t global[] = { (int)fmax(1, group_count - remainder) * work_item_count, 1 };
    size_t local[]  = { work_item_count, 1 };  

    unsigned int padding = element_count_per_group / NUM_BANKS;
    size_t shared = sizeof(float) * (element_count_per_group + padding);
    
    cl_mem partial_sums = ScanPartialSums[level];
    int err = CL_SUCCESS;
    
    if (group_count > 1)
    {
        err = PreScanStoreSum(global, local, shared, output_data, input_data, partial_sums, work_item_count * 2, 0, 0);

        if(err != CL_SUCCESS)
            return err;
            
        if (remainder)
        {
            size_t last_global[] = { 1 * remaining_work_item_count, 1 };
            size_t last_local[]  = { remaining_work_item_count, 1 };  

            err = PreScanStoreSumNonPowerOfTwo(
                    last_global, last_local, last_shared, 
                    output_data, input_data, partial_sums,
                    last_group_element_count, 
                    group_count - 1, 
                    element_count - last_group_element_count);    
        
            if(err != CL_SUCCESS)
                return err;			
			
        }

        err = PreScanBufferRecursive(partial_sums, partial_sums, max_group_size, max_work_item_count, group_count, level + 1);
        if(err != CL_SUCCESS)
            return err;
            
        err = UniformAdd(global, local, output_data, partial_sums,  element_count - last_group_element_count, 0, 0);
        if(err != CL_SUCCESS)
            return err;
        
        if (remainder)
        {
            size_t last_global[] = { 1 * remaining_work_item_count, 1 };
            size_t last_local[]  = { remaining_work_item_count, 1 };  

            err = UniformAdd(
                    last_global, last_local, 
                    output_data, partial_sums,
                    last_group_element_count, 
                    group_count - 1, 
                    element_count - last_group_element_count);
                
            if(err != CL_SUCCESS)
                return err;
        }
    }
    else if (IsPowerOfTwo(element_count))
    {
        err = PreScan(global, local, shared, output_data, input_data, work_item_count * 2, 0, 0);
        if(err != CL_SUCCESS)
            return err;
    }
    else
    {
        err = PreScanNonPowerOfTwo(global, local, shared, output_data, input_data, element_count, 0, 0);
        if(err != CL_SUCCESS)
            return err;
    }

	//// for debug
	//b2CLDevice::instance().copyArrayFromDevice(temp, input_data, 0, sizeof(float)*element_count, true);
	//b2CLDevice::instance().copyArrayFromDevice(temp, output_data, 0, sizeof(float)*element_count, true);
	////b2CLDevice::instance().copyArrayFromDevice(temp, partial_sums, 0, sizeof(float)*group_count, true);
	//delete [] temp;

    return CL_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Use internal buffer to restore results
void b2CLScan::PreScanBuffer(
                cl_mem input_data,
                int element_count)
{
	if (ElementsAllocated<element_count)
		CreatePartialSumBuffers(element_count);
    PreScanBufferRecursive(scanResultsBuffer, input_data, kernel_work_group_size, kernel_work_group_size, element_count, 0);
}

// Use user's buffer to restore results
void b2CLScan::PreScanBuffer(
                cl_mem output_data,
                cl_mem input_data,
                int element_count)
{
	if (ElementsAllocated<element_count)
		CreatePartialSumBuffers(element_count);
    PreScanBufferRecursive(output_data, input_data, kernel_work_group_size, kernel_work_group_size, element_count, 0);
}

// input_data is data to be compacted, -1 means invalid data, >=0 means valid data
int b2CLScan::ParallelCompact(
                cl_mem output_data,
                cl_mem input_data,
				cl_mem scan_result,
                int element_count,
				cl_mem num_validdata)
{
    unsigned int k = PARALLEL_COMPACT;
    unsigned int a = 0;

	if (num_validdata==NULL) // use internal buffer to store number of valid data
	{
		if (!numValidDataBuffer)
			numValidDataBuffer = b2CLDevice::instance().allocateArray(sizeof(cl_uint));
	}

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &scan_result);
	if (num_validdata)
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &num_validdata);
	else
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &numValidDataBuffer );
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &element_count);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

	int group = (element_count+kernel_work_group_size-1)/kernel_work_group_size;
	size_t global[] = { kernel_work_group_size * group };
	size_t local[] = { kernel_work_group_size };
	err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }
    return CL_SUCCESS;
}

// input_data is (binary) data for scan input, 0 means invalid data, 1 means valid data
// indices (0, 1, 2, ...) is data to be compact, implicitly defined in kernel
int b2CLScan::ParallelCompactIndices(
                cl_mem output_data,
                cl_mem input_data,
				cl_mem scan_result,
                int element_count,
				cl_mem num_validdata)
{
    unsigned int k = PARALLEL_COMPACTINDICES;
    unsigned int a = 0;

	if (num_validdata==NULL) // use internal buffer to store number of valid data
	{
		if (!numValidDataBuffer)
			numValidDataBuffer = b2CLDevice::instance().allocateArray(sizeof(cl_uint));
	}

	//// for debug
	//int *testScanResult = new int[element_count];
	//b2CLDevice::instance().copyArrayFromDevice(testScanResult, scan_result, 0, sizeof(int)*element_count, true);
	//delete [] testScanResult;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &scan_result);
	if (num_validdata)
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &num_validdata);
	else
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &numValidDataBuffer );
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &element_count);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

	int group = (element_count+kernel_work_group_size-1)/kernel_work_group_size;
	size_t global[] = { kernel_work_group_size * group };
	size_t local[] = { kernel_work_group_size };
	//printf("element_count: %d, group: %d\n", element_count, group);
	err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }
    return CL_SUCCESS;
}

// input_data is (binary) data for scan input, 0 means invalid data, 1 means valid data
// indices (0, 1, 2, ...) is data to be compact, implicitly defined in kernel
int b2CLScan::ParallelCompactGeneral(
                cl_mem output_data,
                cl_mem input_data,
				cl_mem bit_data,
				cl_mem scan_result,
                int element_count,
				cl_mem num_validdata)
{
    unsigned int k = PARALLEL_COMPATCGENERAL;
    unsigned int a = 0;

	if (num_validdata==NULL) // use internal buffer to store number of valid data
	{
		if (!numValidDataBuffer)
			numValidDataBuffer = b2CLDevice::instance().allocateArray(sizeof(cl_uint));
	}

	//// for debug
	//int *testScanResult = new int[element_count];
	//b2CLDevice::instance().copyArrayFromDevice(testScanResult, scan_result, 0, sizeof(int)*element_count, true);
	//delete [] testScanResult;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &output_data);  
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &bit_data);
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &scan_result);
	if (num_validdata)
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &num_validdata);
	else
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &numValidDataBuffer );
    err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int), &element_count);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }

	int group = (element_count+kernel_work_group_size-1)/kernel_work_group_size;
	size_t global[] = { kernel_work_group_size * group };
	size_t local[] = { kernel_work_group_size };
	//printf("element_count: %d, group: %d\n", element_count, group);
	err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
        return EXIT_FAILURE;
    }
    return CL_SUCCESS;
}

void b2CLScan::ScanCLPP(cl_mem input_data, int element_count)
{
    //printf("ElementsAllocated: %d, LevelsAllocated: %d\n", ElementsAllocated, LevelsAllocated);
	if (ElementsAllocated<element_count)
	{
        if (LevelsAllocated>0)
        {
            unsigned int i;
            for (i = 0; i < LevelsAllocated; i++)
            {
                //printf("array released\n");
                clReleaseMemObject(ScanPartialSums[i]);
            }
            delete [] SumsSizes;
            free(ScanPartialSums);
            ScanPartialSums = 0;
            ElementsAllocated = 0;
            LevelsAllocated = 0;
        }
        
        // Compute the number of buffers we need for the scan
        LevelsAllocated = 0;
        unsigned int n = element_count;
        do
        {
            n = (n + kernel_work_group_size_scan_CLPP - 1) / kernel_work_group_size_scan_CLPP; // round up
            LevelsAllocated++;
        }
        while(n > 1);
        
		// Allocate the arrays
		ScanPartialSums = (cl_mem*) malloc(LevelsAllocated * sizeof(cl_mem));
		SumsSizes = new int[LevelsAllocated + 1];

		// Compute the block-sum sizes
		n = element_count;
		for(unsigned int i = 0; i < LevelsAllocated; i++)
		{
			SumsSizes[i] = n;

            //printf("array alocated: %d\n", sizeof(int) * n);
			ScanPartialSums[i] = b2CLDevice::instance().allocateArray(sizeof(int) * n);

			n = (n + kernel_work_group_size_scan_CLPP - 1) / kernel_work_group_size_scan_CLPP; // round up
		}
		SumsSizes[LevelsAllocated] = n;
        
        ElementsAllocated=element_count;

        // increase the size of scanResultsBuffer
        if (scanResultsBuffer)
            b2CLDevice::instance().freeArray(scanResultsBuffer);
        scanResultsBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*element_count);
	}

    // copy input_data into scanResultsBuffer because the scan kernels will update input values
	b2CLDevice::instance().copyArrayInsideDevice(input_data, scanResultsBuffer, sizeof(int) * element_count);

	//---- Apply the scan to each level
	int err = CL_SUCCESS;
	err |= clSetKernelArg(scanKernelCLPP,  1, kernel_work_group_size_scan_CLPP * sizeof(int) + kernel_work_group_size_scan_CLPP/16, 0);

	cl_mem clValues = scanResultsBuffer;
	for(unsigned int i = 0; i < LevelsAllocated; i++)
	{
		size_t globalWorkSize = {toMultipleOf((SumsSizes[i]+1) / 2, kernel_work_group_size_scan_CLPP / 2)};
		size_t localWorkSize = {kernel_work_group_size_scan_CLPP / 2};

		err = clSetKernelArg(scanKernelCLPP, 0, sizeof(cl_mem), &clValues);
		err |= clSetKernelArg(scanKernelCLPP, 2, sizeof(cl_mem), &ScanPartialSums[i]);
		err |= clSetKernelArg(scanKernelCLPP, 3, sizeof(int), &SumsSizes[i]);
		if (err != CL_SUCCESS)
		{
			printf("Error: ScanCLPP: Failed to set kernel arguments!\n");
			return;
		}

		err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernelCLPP, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: ScanCLPP Kernel: Failed to execute kernel!\n");
			return;
		}

		clValues = ScanPartialSums[i];
    }

	//---- Uniform addition
	for(int i = LevelsAllocated - 2; i >= 0; i--)
	{
		size_t globalWorkSize = {toMultipleOf((SumsSizes[i]+1) / 2, kernel_work_group_size_scan_CLPP / 2)};
		size_t localWorkSize = {kernel_work_group_size_scan_CLPP / 2};

        cl_mem dest = (i > 0) ? ScanPartialSums[i-1] : scanResultsBuffer;

		err = clSetKernelArg(uniformAddKernelCLPP, 0, sizeof(cl_mem), &dest);
		err |= clSetKernelArg(uniformAddKernelCLPP, 1, sizeof(cl_mem), &ScanPartialSums[i]);
		err |= clSetKernelArg(uniformAddKernelCLPP, 2, sizeof(int), &SumsSizes[i]);
		if (err != CL_SUCCESS)
		{
			printf("Error: AddCLPP: Failed to set kernel arguments!\n");
			return;
		}

		err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), uniformAddKernelCLPP, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: AddCLPP Kernel: Failed to execute kernel!\n");
			return;
		}
    }
}

///////////////////////////////////////////////////////
const char* b2CLScanFloat4::ScanKernelNames[] = 
{
	"LocalScanKernel",
	"AddOffsetKernel",
	"TopLevelScanKernel",
};

b2CLScanFloat4::b2CLScanFloat4()
{
#if defined(SCAN_OPENCL)
    printf("Initializing b2CLScanFloat4...\n");
    
    int err;
    
    //load opencl programs from files
    char* scanKernelSource = 0;
    size_t scanKernelSourceLen = 0;

    shrLog("...loading b2CLPrefixScanFloat4.cl\n");
    
#ifdef linux
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLPrefixScanFloat4.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (_WIN32)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLPrefixScanFloat4", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLPrefixScanFloat4", NULL), "// My comment\n", &scanKernelSourceLen);
#else
    scanKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLPrefixScanFloat4", NULL), "// My comment\n", &scanKernelSourceLen);
#endif
    
	if(scanKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLPrefixScanFloat4.cl' correct?");
	}

    //create the compute program from source kernel code
    scanProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&scanKernelSource, NULL, &err);
    if (!scanProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(scanProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(scanProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
    
    //create the compute kernel
	kernel_work_group_size = INT_MAX;
    for(int i = 0; i < SCANKERNELCOUNT; i++)
    {    
        // Create each kernel from within the compactionProgram
        scanKernels[i] = clCreateKernel(scanProgram, ScanKernelNames[i], &err);
        if (!scanKernels[i] || err != CL_SUCCESS)
        {
            printf("Error: Failed to create compute kernel!\n");
            exit(1);
        }
		size_t size;
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(scanKernels[i], size);
		kernel_work_group_size = kernel_work_group_size>size ? size : kernel_work_group_size;
    }

	WorkBuffer = scanResultsBuffer = NULL;
	old_workbuffer_size = 0;

	numValidDataBuffer = NULL;
#endif
}

b2CLScanFloat4::~b2CLScanFloat4()
{
}

b2CLScanFloat4& b2CLScanFloat4::instance()
{
	static b2CLScanFloat4 inst;
	return inst;
}

int b2CLScanFloat4::Scan(cl_mem input_data, int element_count)
{
	const unsigned int numBlocks = (const unsigned int)( (element_count+BLOCK_SIZE*2-1)/(BLOCK_SIZE*2) );

	if (old_workbuffer_size<element_count)
	{
		if (WorkBuffer)
		    b2CLDevice::instance().freeArray(WorkBuffer);
	    WorkBuffer = b2CLDevice::instance().allocateArray(sizeof(float) * 4 * element_count);

		if (scanResultsBuffer)
		    b2CLDevice::instance().freeArray(scanResultsBuffer);
	    scanResultsBuffer = b2CLDevice::instance().allocateArray(sizeof(float) * 4 * element_count);

		old_workbuffer_size = element_count;
	}

	cl_uint4 constBuffer;
	constBuffer.s[0] = element_count;
	constBuffer.s[1] = numBlocks;
	constBuffer.s[2] = nextPowerOf2( numBlocks );

	// LocalScanKernel
	{
		unsigned int k = LOCALSCAN;
		unsigned int a = 0;

		int err = CL_SUCCESS;
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &scanResultsBuffer);
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &input_data);  
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &WorkBuffer);
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_uint4), &constBuffer);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}

		size_t local[] = {BLOCK_SIZE, 1};
		size_t global[] = {numBlocks*BLOCK_SIZE, 1};
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}

		//// for debug
		//float *input = new float[element_count*4];
		//float *result = new float[element_count*4];
		//float *work = new float[element_count*4];
		//b2CLDevice::instance().copyArrayFromDevice(input, input_data, 0, sizeof(float)*4*element_count, true);
		//b2CLDevice::instance().copyArrayFromDevice(result, scanResultsBuffer, 0, sizeof(float)*4*element_count, true);
		//b2CLDevice::instance().copyArrayFromDevice(work, WorkBuffer, 0, sizeof(float)*4*element_count, true);
		//delete [] input;
		//delete [] result;
		//delete [] work;
	}

	// TopLevelScanKernel
	{
		unsigned int k = TOPLEVELSCAN;
		unsigned int a = 0;

		int err = CL_SUCCESS;
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &WorkBuffer);
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int4), &constBuffer);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}

		size_t local[] = {BLOCK_SIZE, 1};
		size_t global[] = {BLOCK_SIZE, 1};
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}
	}
	
	// AddOffsetKernel
	if( numBlocks > 1 )
	{
		unsigned int k = ADDOFFSET;
		unsigned int a = 0;

		int err = CL_SUCCESS;
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &scanResultsBuffer);
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_mem), &WorkBuffer);
		err |= clSetKernelArg(scanKernels[k],  a++, sizeof(cl_int4), &constBuffer);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}

		size_t local[] = {BLOCK_SIZE, 1};
		size_t global[] = {(numBlocks-1)*BLOCK_SIZE, 1};
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), scanKernels[k], 1, NULL, global, local, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to execute kernel!\n", ScanKernelNames[k]);
			return EXIT_FAILURE;
		}
	}
	return CL_SUCCESS;
}
