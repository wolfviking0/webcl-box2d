/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLNarrowPhase_h
#define Box2D_b2CLNarrowPhase_h

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Common/OpenCL/b2CLCommonData.h>

#ifdef _DEBUG_TIME_NARROWPHASE
#include <Box2D/Common/b2Timer.h>
#endif

class b2World;
class b2Contact;
class b2PolygonShape;
class b2ContactListener;

class b2CLNarrowPhase
{
public: 
    b2CLNarrowPhase();
    ~b2CLNarrowPhase();

	//void CreateXfBuffer(b2World *m_pWorld);
	void InitializeGPUData(b2World *m_pWorld, b2Contact *m_contactList, int32 *m_pContactCounts);
	void UpdateContactPairs(int contactNum, int *pContactNums, int maxContactNum/*, b2ContactListener* listener*/);
	void CompactContactPairs(int contactNum);
	void CompactEnabledContactPairs(int contactNum);
	void ReadbackGPUData(b2World *m_pWorld, b2Contact *m_contactList, b2ContactListener* listener);
	void ReadbackGPUDataForListener(b2World *m_pWorld, b2Contact **m_contactList, b2ContactListener* listener, int *enableBitArray, int *temp);

private:
    cl_program narrowPhaseProgram;
    cl_kernel collidePolygonsKernel, collideCirclesKernel, collidePolygonAndCircleKernel, collideEdgeAndCircleKernel, collideEdgeAndPolygonKernel;
	cl_kernel compactForOneContactKernel;
	size_t kernel_work_group_size, kernel_preferred_work_group_size_multiple;

	int32 /*old_xf_num, */old_contact_num, manifold_nums[2];
    
	b2clTransform *xfListData;
	int32 *globalIndicesData, *pairIndicesData;

	//cl_mem xfListBuffer;
	//cl_mem manifoldListBuffer;
	//cl_mem pairIndicesBuffer;
};
#endif
