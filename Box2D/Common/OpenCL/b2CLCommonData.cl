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

////////////////////////////////////////////////////////////////////////////////////////////////////
// ReadLastJointImpulses
// 
// Read joint impulses of the last frame for warm starting.
// b2clJointImpulseNode contains impulses of the last frame.
// Joint index is used to find the position of the impulse of a joint in the array.
////////////////////////////////////////////////////////////////////////////////////////////////////
__kernel void ReadLastJointImpulses(
                                 __global b2clJoint* joints, // output
								 const __global b2clJointImpulseNode* lastJointImpulses, // input, joint impulses of the last frame
								 const __global int* jointImpulseKeys,
								 const __global int* jointImpulseGlobalIndices,
								 const unsigned int jointCount,
								 const unsigned int lastjointCount)
{
    unsigned int jointIndex = get_global_id(0);

    if (jointIndex >= jointCount) return;

	__global b2clJoint* currentJoint = joints + jointIndex;
	int currentId = currentJoint->index;

	// binary search
	int lb = 0; // low bound
	int ub = lastjointCount-1; // upper bound
	int mid;
	bool bFound = false;
	uint midId;
	
	while (lb <= ub)
	{
		mid = (lb+ub)/2;
		
		midId = jointImpulseKeys[mid];

		if (midId == currentId)
		{
			bFound = true;
			break;
		}
		else if (midId < currentId)
		{
			ub = mid - 1;
		}
		else
		{
			lb = mid + 1;
		}
	}

	if (bFound)
	{
		const __global b2clJointImpulseNode* storedNode = lastJointImpulses + jointImpulseGlobalIndices[mid];
		currentJoint->a.x.impulse[0] = storedNode->nimpulse[0];
		currentJoint->a.x.impulse[1] = storedNode->nimpulse[1];
		currentJoint->a.x.impulse[2] = storedNode->nimpulse[2];
		currentJoint->motorImpulse = storedNode->nimpulse[3];
	}
	else
	{
		currentJoint->a.x.impulse[0] = 0.0f;
		currentJoint->a.x.impulse[1] = 0.0f;
		currentJoint->a.x.impulse[2] = 0.0f;
		currentJoint->motorImpulse = 0.0f;
	}
}
