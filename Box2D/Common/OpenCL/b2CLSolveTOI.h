/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/



#ifndef B2CLSOLVETOI_H
#define B2CLSOLVETOI_H

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Common/OpenCL/b2CLSolver.h>
#include <Box2D/Common/OpenCL/b2CLCommonData.h>
#include <Box2D/Dynamics/b2TimeStep.h>
#include <Box2D/Dynamics/b2Body.h>
#include <set>

class b2CLSolveTOI
{
public: 
    b2CLSolveTOI();
    ~b2CLSolveTOI();

	static b2CLSolveTOI& instance();

	bool ComputeMinTOI(int numContacts, bool firstTime, int numLoop);
	void AdvanceBodies();
	void GetAllConnectedBodies (int numContactFromBF) ; 
	void GenerateTimeStep (b2TimeStep & subStep, b2TimeStep step);
	void CopyUpdatedBodytoDevice (b2Body* b, b2CLSolver* solver); 
	void CopyAllBodytoDevice (b2Body* b, b2CLSolver* solver, int bodyNum); 
	void syncMovedBodytoDevice (std::set<b2Body*> & movedBodySet, b2CLSolver* solver); 
		int numValidContacts; 
			int indexMinContact ; 
	float minTOI ; 
	cl_mem TOITimesBuffer ; 

private:

	cl_program SolveTOIProgram;

    cl_kernel ComputeTOIKernel;
	size_t ComputeTOIKernelWorkGroupSize;

	cl_kernel AdvanceBodiesKernel; 
	cl_kernel MarkConnectedContactsKernel; 

	cl_kernel syncMovedBodyKernel ; 

	int lastNumContacts;
	cl_mem ContactTOIsBuffer;
	cl_mem ContactIndicesBuffer;

	cl_mem movedBodyBuffer ; 
	float* movedBodyList ; 
	int bodyListSize ; 
	
	int sortCount;

	b2Velocity* copyAllVelocityList;
	b2Position* copyAllPositionList;
	b2Transform* copyAllXFList ; 
	int copyBodyNum ; 


};

#endif