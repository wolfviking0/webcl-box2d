/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef simple_BVH_compaction_funcitons_h
#define simple_BVH_compaction_funcitons_h

#include <Box2D/Common/OpenCL/b2CLDevice.h>

#include <sys/stat.h>
#include <algorithm>
#include <math.h>
#if defined linux
#include <string.h>
#endif
#include <fcntl.h>
////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEBUG_INFO      (0)
#define NUM_BANKS       (16)
#define MAX_ERROR       (1e-7)
#define SEPARATOR       ("----------------------------------------------------------------------\n")

//#define min(A,B) ((A) < (B) ? (A) : (B))
#if defined (_WIN32)
#define fmax(A,B) ((A) > (B) ? (A) : (B))
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////

enum CompactionKernelMethods
{
    PRESCAN_FIRST_LEVEL                 = 0,
    PRESCAN                             = 1,
    PRESCAN_STORE_SUM_FIRST_LEVEL       = 2,
    PRESCAN_STORE_SUM                   = 3,
    PRESCAN_STORE_SUM_NON_POWER_OF_TWO_FIRST_LEVEL  = 4,
    PRESCAN_STORE_SUM_NON_POWER_OF_TWO  = 5,
    PRESCAN_NON_POWER_OF_TWO_FIRST_LEVEL= 6,
    PRESCAN_NON_POWER_OF_TWO            = 7,
    UNIFORM_ADD                         = 8,
    PARALLEL_COMPACT                    = 9,
    PARALLEL_COMPACT_FINAL              = 10
};

static const char* CompactionKernelNames[] =
{
    "PreScanKernelFirstLevel",
    "PreScanKernel",
    "PreScanStoreSumFirstLevelKernel",
    "PreScanStoreSumKernel",
    "PreScanStoreSumNonPowerOfTwoFirstLevelKernel",
    "PreScanStoreSumNonPowerOfTwoKernel",
    "PreScanNonPowerOfTwoKernelFirstLevel",
    "PreScanNonPowerOfTwoKernel",
    "UniformAddKernel",
    "ParallelCompact",
    "ParallelCompactFinal"
};

static const unsigned int CompactionKernelCount = sizeof(CompactionKernelNames) / sizeof(char *);

class b2CLCompactionFunctions
{

    cl_mem* ScanPartialSums;
    unsigned int ElementsAllocated;
    unsigned int LevelsAllocated;
    cl_program compactionProgram;
    cl_kernel compactionKernels[CompactionKernelCount];
    
public:
    
    size_t maxWorkGroupSize;
    
    b2CLCompactionFunctions()
    :ScanPartialSums(0),ElementsAllocated(0),LevelsAllocated(0),maxWorkGroupSize(0)
    {
#if defined(BROADPHASE_OPENCL)
        printf("Initializing b2CLCompactionFunctions...\n");
        
        int err;
        
        //load opencl programs from files
        char* scanKernelSource=0;
        size_t scanKernelSourceLen=0;
    
        shrLog("...loading b2CLScanKernel.cl\n");

#ifdef linux
        scanKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLScanKernel.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (_WIN32)
        scanKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLScanKernel.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
        scanKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLScanKernel.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#else
        scanKernelSource = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLScanKernel.cl", NULL), "// My comment\n", &scanKernelSourceLen);
#endif
        
        if(scanKernelSource == NULL)
        {
            b2Log("Could not load program source, is path 'b2CLScanKernel.cl' correct?");
        }
        
        //create the compute program from source kernel code
        compactionProgram=clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&scanKernelSource, NULL, &err);
        if (!compactionProgram)
        {
            printf("Error: Failed to create compute program!\n");
            exit(1);
        }
        
        //build the program
        err=clBuildProgram(compactionProgram,0,NULL,NULL,NULL,NULL);
        if (err != CL_SUCCESS)
        {
            size_t len;
            char buffer[2048];
            
            printf("Error: Failed to build program executable!\n");
            clGetProgramBuildInfo(compactionProgram,b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
            printf("%s\n", buffer);
            exit(1);
        }
        
        //create the compute kernel
        for(int i = 0; i < static_cast<int>(CompactionKernelCount); i++)
        {    
            // Create each kernel from within the compactionProgram
            compactionKernels[i] = clCreateKernel(compactionProgram, CompactionKernelNames[i], &err);
            if (!compactionKernels[i] || err != CL_SUCCESS)
            {
                printf("Error: Failed to create compute kernel!\n");
                exit(1);
            }
            size_t maxWorkGroupSizeTmp;
            b2CLDevice::instance().getMaximumKernelWorkGroupSize(compactionKernels[i], maxWorkGroupSizeTmp);
            if(!maxWorkGroupSize || maxWorkGroupSizeTmp<maxWorkGroupSize) maxWorkGroupSize=maxWorkGroupSizeTmp;
        }
#endif
    }
    
    bool IsPowerOfTwo(int n)
    {
        return ((n&(n-1))==0) ;
    }
    
    int floorPow2(int n)
    {
        int exp;
        frexp((float)n, &exp);
        return 1 << (exp - 1);
    }
    
    int 
    CreatePartialSumBuffers(unsigned int count)
    {
        ElementsAllocated = count;
        
        unsigned int group_size = maxWorkGroupSize;
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
        
        return CL_SUCCESS;
    }
    
    void 
    ReleasePartialSums(void)
    {
        unsigned int i;
        for (i = 0; i < LevelsAllocated; i++)
        {
            b2CLDevice::instance().freeArray(ScanPartialSums[i]);
        }
        
        free(ScanPartialSums);
        ScanPartialSums = 0;
        ElementsAllocated = 0;
        LevelsAllocated = 0;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void 
    ParallelCompactBuffer(
                          cl_mem CompactIn_data,
                          cl_mem CompactOut_data,
                          cl_mem numUsefulNodes,
                          cl_mem ScanResult_data, 
                          cl_mem LevelLength_data,
                          int max_level_length,
                          int num_move,
                          bool final=false)
    {
        unsigned int k = final?PARALLEL_COMPACT_FINAL:PARALLEL_COMPACT;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &CompactIn_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &CompactOut_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &numUsefulNodes);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &ScanResult_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &LevelLength_data);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &max_level_length);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return;
        }
        
        size_t global[] = { static_cast<size_t>(max_level_length), static_cast<size_t>(num_move) };
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 2, NULL, global, NULL, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return;
        }
        
        return;
    }
    
    int
    PreScan(
            size_t *global, 
            size_t *local, 
            size_t shared, 
            cl_mem output_data, 
            cl_mem input_data, 
            unsigned int n,
            int group_index, 
            int base_index,
            int level,
            int addr_offset)
    {
        if(shared==0) return CL_SUCCESS;
        
#if DEBUG_INFO
        printf("PreScan: Global[%4d] Local[%4d] Shared[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n", 
               (int)global[0], (int)local[0], (int)shared, group_index, base_index, n);
#endif
        
        unsigned int k;
        k = level==0 ? PRESCAN_FIRST_LEVEL : PRESCAN;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &output_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &input_data);
        err |= clSetKernelArg(compactionKernels[k],  a++, shared,         0);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &group_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &base_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &n);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &addr_offset);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 1, NULL, global, local, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        return CL_SUCCESS;
    }
    
    int
    PreScanStoreSum(
                    size_t *global, 
                    size_t *local, 
                    size_t shared, 
                    cl_mem output_data, 
                    cl_mem input_data, 
                    cl_mem partial_sums,
                    unsigned int n,
                    int group_index, 
                    int base_index,
                    int level,
                    int addr_offset)
    { 
        if(shared==0) return CL_SUCCESS;
        
#if DEBUG_INFO
        printf("PreScan: Global[%4d] Local[%4d] Shared[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n", 
               (int)global[0], (int)local[0], (int)shared, group_index, base_index, n);
#endif
        
        unsigned int k;
        k = level==0 ? PRESCAN_STORE_SUM_FIRST_LEVEL : PRESCAN_STORE_SUM;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &output_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &input_data);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &partial_sums);
        err |= clSetKernelArg(compactionKernels[k],  a++, shared,         0);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &group_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &base_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &n);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &addr_offset);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 1, NULL, global, local, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        return CL_SUCCESS;
    }
    
    int
    PreScanStoreSumNonPowerOfTwo(
                                 size_t *global, 
                                 size_t *local, 
                                 size_t shared, 
                                 cl_mem output_data, 
                                 cl_mem input_data, 
                                 cl_mem partial_sums,
                                 unsigned int n,
                                 int group_index, 
                                 int base_index,
                                 int level,
                                 int addr_offset)
    {
        if(shared==0) return CL_SUCCESS;
#if DEBUG_INFO
        printf("PreScanStoreSumNonPowerOfTwo: Global[%4d] Local[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n", 
               (int)global[0], (int)local[0], group_index, base_index, n);
#endif
        
        unsigned int k;
        k = level==0 ? PRESCAN_STORE_SUM_NON_POWER_OF_TWO_FIRST_LEVEL : PRESCAN_STORE_SUM_NON_POWER_OF_TWO;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &output_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &input_data);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &partial_sums);
        err |= clSetKernelArg(compactionKernels[k],  a++, shared,         0);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &group_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &base_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &n);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &addr_offset);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 1, NULL, global, local, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        return CL_SUCCESS;
    }
    
    int
    PreScanNonPowerOfTwo(
                         size_t *global, 
                         size_t *local, 
                         size_t shared, 
                         cl_mem output_data, 
                         cl_mem input_data, 
                         unsigned int n,
                         int group_index, 
                         int base_index,
                         int level,
                         int addr_offset)
    {
        if(shared==0) return CL_SUCCESS;
#if DEBUG_INFO
        printf("PreScanNonPowerOfTwo: Global[%4d] Local[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n", 
               (int)global[0], (int)local[0], group_index, base_index, n);
#endif
        
        unsigned int k;
        k = level==0 ? PRESCAN_NON_POWER_OF_TWO_FIRST_LEVEL : PRESCAN_NON_POWER_OF_TWO;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &output_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &input_data);
        err |= clSetKernelArg(compactionKernels[k],  a++, shared,         0);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &group_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &base_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &n);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &addr_offset);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 1, NULL, global, local, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        return CL_SUCCESS;
    }
    
    int
    UniformAdd(
               size_t *global, 
               size_t *local, 
               cl_mem output_data, 
               cl_mem partial_sums, 
               unsigned int n, 
               unsigned int group_offset, 
               unsigned int base_index,
               unsigned int addr_offset)
    {
#if DEBUG_INFO
        printf("UniformAdd: Global[%4d] Local[%4d] BlockOffset[%4d] BaseIndex[%4d] Entries[%d]\n", 
               (int)global[0], (int)local[0], group_offset, base_index, n);
#endif
        
        unsigned int k = UNIFORM_ADD;
        unsigned int a = 0;
        
        int err = CL_SUCCESS;
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &output_data);  
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_mem), &partial_sums);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(float),  0);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &group_offset);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &base_index);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &n);
        err |= clSetKernelArg(compactionKernels[k],  a++, sizeof(cl_int), &addr_offset);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to set kernel arguments!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        err = CL_SUCCESS;
        err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactionKernels[k], 1, NULL, global, local, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            printf("Error: %s: Failed to execute kernel!\n", CompactionKernelNames[k]);
            return EXIT_FAILURE;
        }
        
        return CL_SUCCESS;
    }
    
    int 
    PreScanBufferRecursive(
                           cl_mem output_data, 
                           cl_mem input_data, 
                           unsigned int max_group_size,
                           unsigned int max_work_item_count,
                           unsigned int element_count, 
                           unsigned int addr_offset,
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
            err = PreScanStoreSum(global, local, shared, output_data, input_data, partial_sums, work_item_count * 2, 0, 0,level,addr_offset);
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
                                                   element_count - last_group_element_count,level,addr_offset);    
                
                if(err != CL_SUCCESS)
                    return err;			
                
            }
            
            err = PreScanBufferRecursive(partial_sums, partial_sums, max_group_size, max_work_item_count, group_count, 0, level + 1);
            if(err != CL_SUCCESS)
                return err;
            
            err = UniformAdd(global, local, output_data, partial_sums,  element_count - last_group_element_count, 0, 0, addr_offset);
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
                                 element_count - last_group_element_count, addr_offset);
                
                if(err != CL_SUCCESS)
                    return err;
            }
        }
        else if (IsPowerOfTwo(element_count))
        {
            err = PreScan(global, local, shared, output_data, input_data, work_item_count * 2, 0, 0, level, addr_offset);
            if(err != CL_SUCCESS)
                return err;
        }
        else
        {
            err = PreScanNonPowerOfTwo(global, local, shared, output_data, input_data, element_count, 0, 0, level, addr_offset);
            if(err != CL_SUCCESS)
                return err;
        }
        
        return CL_SUCCESS;
    }    
    
    void 
    PreScanBuffer(
                  cl_mem CompactIn_data,
                  cl_mem CompactOut_data,
                  cl_mem clNumUsefulNodesOutput,
                  cl_mem ScanResult_data, 
                  cl_mem clNumUsefulNodesInput,
                  int max_group_size,
                  int max_work_item_count,
                  int *clNumUsefulNodes,
                  int clMaxNumUsefulNodes,
                  int num_move,
                  bool final=false)
    {
        // Scan for each row. Should be able to use segmented scan to scan all rows together!
        for (int i=0; i<num_move; i++)
            PreScanBufferRecursive(ScanResult_data, CompactIn_data, max_group_size, max_work_item_count, clNumUsefulNodes[i], clMaxNumUsefulNodes*i, 0);
		clFinish(b2CLDevice::instance().GetCommandQueue());
        ParallelCompactBuffer(CompactIn_data, CompactOut_data, clNumUsefulNodesOutput, ScanResult_data, clNumUsefulNodesInput, clMaxNumUsefulNodes, num_move, final);
    }    
};
#endif
