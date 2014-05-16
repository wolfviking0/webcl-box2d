//------------------------------------------------------------
// Purpose :
// ---------
// Prefix sum or prefix scan is an operation where each output element contains the sum of all input elements preceding it.
//
// Algorithm :
// -----------
// The parallel prefix sum has two principal parts, the reduce phase (also known as the up-sweep phase) and the down-sweep phase.
//
// In the up-sweep reduction phase we traverse the computation tree from bottom to top, computing partial sums.
// After this phase, the last element of the array contains the total sum.
//
// During the down-sweep phase, we traverse the tree from the root and use the partial sums to build the scan in place.
//
// Because the scan pictured is an exclusive sum, a zero is inserted into the last element before the start of the down-sweep phase.
// This zero is then propagated back to the first element.
//
// In our implementation, each compute unit loads and sums up two elements (for the deepest depth). Each subsequent depth during the up-sweep
// phase is processed by half of the compute units from the deeper level and the other way around for the down-sweep phase.
//
// In order to be able to scan large arrays, i.e. arrays that have many more elements than the maximum size of a work-group, the prefix sum has to be decomposed.
// Each work-group computes the prefix scan of its sub-range and outputs a single number representing the sum of all elements in its sub-range.
// The workgroup sums are scanned using exactly the same algorithm.
// When the number of work-group results reaches the size of a work-group, the process is reversed and the work-group sums are
// propagated to the sub-ranges, where each work-group adds the incoming sum to all its elements, thus producing the final scanned array.
//
// References :
// ------------
// NVIDIA Mark Harris. Parallel prefix sum (scan) with CUDA. April 2007
// http://developer.download.nvidia.com/compute/cuda/1_1/Website/projects/scan/doc/scan.pdf
// http://graphics.idav.ucdavis.edu/publications/print_pub?pub_id=915
//
// Other references :
// ------------------
// http://developer.nvidia.com/node/57
//------------------------------------------------------------

#pragma OPENCL EXTENSION cl_amd_printf : enable
#define T float
#define SUPPORT_AVOID_BANK_CONFLICT

//------------------------------------------------------------
// kernel__ExclusivePrefixScanSmall
//
// Purpose : do a fast scan on a small chunck of data.
//------------------------------------------------------------

__kernel 
void kernel__ExclusivePrefixScanSmall(
	__global T* input,
	__global T* output,
	__local  T* block,
	const uint length)
{
	int tid = get_local_id(0);
	
	int offset = 1;

    // Cache the computational window in shared memory
	block[2*tid]     = input[2*tid];
	block[2*tid + 1] = input[2*tid + 1];	

    // Build the sum in place up the tree
	for(int d = length>>1; d > 0; d >>=1)
	{
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if(tid<d)
		{
			int ai = offset*(2*tid + 1) - 1;
			int bi = offset*(2*tid + 2) - 1;
			
			block[bi] += block[ai];
		}
		offset *= 2;
	}

    // scan back down the tree

    // Clear the last element
	if(tid == 0)
		block[length - 1] = 0;

    // traverse down the tree building the scan in the place
	for(int d = 1; d < length ; d *= 2)
	{
		offset >>=1;
		barrier(CLK_LOCAL_MEM_FENCE);
		
		if(tid < d)
		{
			int ai = offset*(2*tid + 1) - 1;
			int bi = offset*(2*tid + 2) - 1;
			
			float t = block[ai];
			block[ai] = block[bi];
			block[bi] += t;
		}
	}
	
	barrier(CLK_LOCAL_MEM_FENCE);

    // write the results back to global memory
	output[2*tid]     = block[2*tid];
	output[2*tid + 1] = block[2*tid + 1];
}

//------------------------------------------------------------
// kernel__ExclusivePrefixScan
//
// Purpose : do a scan on a chunck of data.
//------------------------------------------------------------

// Define this to more rigorously avoid bank conflicts, even at the lower (root) levels of the tree.
// To avoid bank conflicts during the tree traversal, we need to add padding to the shared memory array every NUM_BANKS (16) elements.
// Note that due to the higher addressing overhead, performance is lower with ZERO_BANK_CONFLICTS enabled.
// It is provided as an example.
//#define ZERO_BANK_CONFLICTS 

// 16 banks on G80
#define NUM_BANKS 16
#define LOG_NUM_BANKS 4

#ifdef ZERO_BANK_CONFLICTS
	#define CONFLICT_FREE_OFFSET(index) ((index) >> LOG_NUM_BANKS + (index) >> (2*LOG_NUM_BANKS))
#else
	#define CONFLICT_FREE_OFFSET(index) ((index) >> LOG_NUM_BANKS)
#endif

//#define CONFLICT_FREE_OFFSET(index) 0

__kernel
void kernel__ExclusivePrefixScan(
	__global T* dataSet,
	
	__local T* localBuffer,
	
	__global T* blockSums,
	const uint blockSumsSize
	)
{
	const uint gid = get_global_id(0);
	const uint tid = get_local_id(0);
	const uint bid = get_group_id(0);
	const uint lwz  = get_local_size(0);
	
	// The local buffer has 2x the size of the local-work-size, because we manage 2 scans at a time.
    const uint localBufferSize = lwz << 1;
    int offset = 1;
	
    const int tid2_0 = tid << 1;
    const int tid2_1 = tid2_0 + 1;
	
	const int gid2_0 = gid << 1;
    const int gid2_1 = gid2_0 + 1;

	// Cache the datas in local memory

#ifdef SUPPORT_AVOID_BANK_CONFLICT
	uint ai = tid;
	uint bi = tid + lwz;
	uint gai = tid + bid*lwz*2;
	uint gbi = gai + lwz;
	uint bankOffsetA = CONFLICT_FREE_OFFSET(ai); 
	uint bankOffsetB = CONFLICT_FREE_OFFSET(bi);
	localBuffer[ai + bankOffsetA] = (gai < blockSumsSize) ? dataSet[gai] : 0; 
	localBuffer[bi + bankOffsetB] = (gbi < blockSumsSize) ? dataSet[gbi] : 0;
#else
	localBuffer[tid2_0] = (gid2_0 < blockSumsSize) ? dataSet[gid2_0] : 0;
	localBuffer[tid2_1] = (gid2_1 < blockSumsSize) ? dataSet[gid2_1] : 0;
#endif
	
    // bottom-up
    for(uint d = lwz; d > 0; d >>= 1)
	{
        barrier(CLK_LOCAL_MEM_FENCE);
		
        if (tid < d)
		{
#ifdef SUPPORT_AVOID_BANK_CONFLICT
			//uint ai = mad24(offset, (tid2_1+0), -1);	// offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            //uint bi = mad24(offset, (tid2_1+1), -1);	// offset*(tid2_1+1)-1;			
			uint i = 2 * offset * tid;
			uint ai = i + offset - 1;
			uint bi = ai + offset;
			ai += CONFLICT_FREE_OFFSET(ai);	// ai += ai / NUM_BANKS;
			bi += CONFLICT_FREE_OFFSET(bi);	// bi += bi / NUM_BANKS;
#else
            const uint ai = mad24(offset, (tid2_1+0), -1);	// offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            const uint bi = mad24(offset, (tid2_1+1), -1);	// offset*(tid2_1+1)-1;
#endif

            localBuffer[bi] += localBuffer[ai];
        }
        offset <<= 1;
    }

    barrier(CLK_LOCAL_MEM_FENCE);
	
	/*
	if (tid < 1)
		blockSums[bid] = localBuffer[localBufferSize-1];
		
	barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);
	
	if (tid < 1)
		localBuffer[localBufferSize - 1] = 0;
	*/
	
	//// for debug
	//uint iii = localBufferSize-1;
	//iii += CONFLICT_FREE_OFFSET(iii);
	//printf("iii: %d, localBuffer[iii]: %d\n", iii, localBuffer[iii]);

    if (tid < 1)
	{
#ifdef SUPPORT_AVOID_BANK_CONFLICT
		uint index = localBufferSize-1;
		index += CONFLICT_FREE_OFFSET(index);
		blockSums[bid] = localBuffer[index];
		localBuffer[index] = 0;
		//printf("bid: %d, lwz: %d, blockSums[bid]: %d\n", bid, lwz, blockSums[bid]);
#else
		// We store the biggest value (the last) to the sum-block for later use.
        blockSums[bid] = localBuffer[localBufferSize-1];		
		//barrier(CLK_LOCAL_MEM_FENCE | CLK_GLOBAL_MEM_FENCE);		
		// Clear the last element
        localBuffer[localBufferSize - 1] = 0;
#endif
    }

    // top-down
    for(uint d = 1; d < localBufferSize; d <<= 1)
	{
        offset >>= 1;
        barrier(CLK_LOCAL_MEM_FENCE);
		
        if (tid < d)
		{
#ifdef SUPPORT_AVOID_BANK_CONFLICT
			//uint ai = mad24(offset, (tid2_1+0), -1);	// offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            //uint bi = mad24(offset, (tid2_1+1), -1);	// offset*(tid2_1+1)-1;			
			uint i = 2 * offset * tid;
			uint ai = i + offset - 1;
			uint bi = ai + offset;
			ai += CONFLICT_FREE_OFFSET(ai);	// Apply an offset to the __local memory
			bi += CONFLICT_FREE_OFFSET(bi);
#else
            const uint ai = mad24(offset, (tid2_1+0), -1); // offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            const uint bi = mad24(offset, (tid2_1+1), -1); // offset*(tid2_1+1)-1;
#endif

            T tmp = localBuffer[ai];
            localBuffer[ai] = localBuffer[bi];
            localBuffer[bi] += tmp;
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);

    // Copy back from the local buffer to the output array
	
#ifdef SUPPORT_AVOID_BANK_CONFLICT
	//dataSet[gai] = (gai < blockSumsSize) * localBuffer[ai + bankOffsetA];		
	//dataSet[gbi] = (gbi < blockSumsSize) * localBuffer[bi + bankOffsetB];		

	//printf("gai: %d, gbi: %d, blockSumsSize: %d\n", gai, gbi, blockSumsSize);
	//printf("ai: %d, bankOffsetA: %d, bi: %d, bankOffsetB: %d\n", ai, bankOffsetA, bi, bankOffsetB);
	if (gai < blockSumsSize)
		dataSet[gai] = localBuffer[ai + bankOffsetA];		
	if (gbi < blockSumsSize)
		dataSet[gbi] = localBuffer[bi + bankOffsetB];		
#else
	if (gid2_0 < blockSumsSize)
		dataSet[gid2_0] = localBuffer[tid2_0];
	if (gid2_1 < blockSumsSize)
		dataSet[gid2_1] = localBuffer[tid2_1];
#endif

}

//------------------------------------------------------------
// kernel__ExclusivePrefixScan
//
// Purpose :
// Final step of large-array scan: combine basic inclusive scan with exclusive scan of top elements of input arrays.
//------------------------------------------------------------

__kernel
void kernel__UniformAdd(
	__global T* output,
	__global const T* blockSums,
	const uint outputSize
	)
{
    uint gid = get_global_id(0) * 2;
    const uint tid = get_local_id(0);
    const uint blockId = get_group_id(0);
	
	// Intel SDK fix
	//output[gid] += blockSums[blockId];
	//output[gid+1] += blockSums[blockId];

    __local T localBuffer[1];

#ifdef SUPPORT_AVOID_BANK_CONFLICT
	//uint blockOffset = 1024 - 1;
    if (tid < 1)
	{
        localBuffer[0] = blockSums[blockId/* + blockOffset*/];
		//printf("localBuffer: %d, blockId: %d\n", localBuffer[0], blockId);
	}
#else
    if (tid < 1)
        localBuffer[0] = blockSums[blockId];
#endif

    barrier(CLK_LOCAL_MEM_FENCE);
	
#ifdef SUPPORT_AVOID_BANK_CONFLICT
	unsigned int address = blockId * get_local_size(0) * 2 + get_local_id(0); 
	
	//output[address] += localBuffer[0];
 //   output[address + get_local_size(0)] += (get_local_id(0) + get_local_size(0) < outputSize) * localBuffer[0];

	//printf("address: %d, output[address] before: %d, localBuffer[0]: %d\n", address, output[address], localBuffer[0]);
	if (address < outputSize)
		output[address] += localBuffer[0];
	if (address + get_local_size(0) < outputSize)
		output[address + get_local_size(0)] += localBuffer[0];
#else
	//printf("gid: %d, output[gid] before: %d, localBuffer[0]: %d\n", gid, output[gid], localBuffer[0]);
	if (gid < outputSize)
		output[gid] += localBuffer[0];
	gid++;
	if (gid < outputSize)
		output[gid] += localBuffer[0];
#endif
}