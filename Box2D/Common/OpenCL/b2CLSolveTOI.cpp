/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/



#include <Box2D/Common/OpenCL/b2CLSolveTOI.h>
#include <Box2D/Common/OpenCL/b2CLCommonData.h>
#include <Box2D/Common/OpenCL/b2CLScan.h>
#include <Box2D/Common/OpenCL/b2CLSort.h>
#include <Box2D/Common/OpenCL/b2CLSolver.h>
#include<set>

// In b2CLBroadPhase.cpp

extern float FFlip (unsigned int fflip);
b2CLSolveTOI::b2CLSolveTOI()
{
	printf("Initializing b2CLSolveTOI...\n");
    
    int err;
    
    //load opencl programs from files
    char* TOIKernelSource = 0;
    size_t TOIKernelSourceLen = 0;

    shrLog("...loading b2CLSolveTOI.cl\n");
#ifdef linux
    TOIKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLSolveTOI.cl", NULL), "// My comment\n", &TOIKernelSourceLen);
#elif defined (_WIN32)    
	TOIKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLSolveTOI.cl", NULL), "// My comment\n", &TOIKernelSourceLen);
#elif defined (__APPLE__)
    TOIKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLSolveTOI.cl", NULL), "// My comment\n", &TOIKernelSourceLen);
#else
    TOIKernelSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLSolveTOI.cl", NULL), "// My comment\n", &TOIKernelSourceLen);
#endif
	if(TOIKernelSource == NULL)
	{
		b2Log("Could not load program source, is path 'b2CLSolveTOI.cl' correct?");
	}

    //create the compute program from source kernel code
    SolveTOIProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&TOIKernelSource, NULL, &err);
    if (!SolveTOIProgram)
    {
        printf("Error: Failed to create compute program!\n");
        exit(1);
    }
    
    //build the program
    err = clBuildProgram(SolveTOIProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[204800];
        
        printf("Error: Failed to build program executable!\n");
        clGetProgramBuildInfo(SolveTOIProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        printf("%s\n", buffer);
        exit(1);
    }
#if 0	
	//create the compute kernel
	ComputeTOIKernel = clCreateKernel(SolveTOIProgram, "b2clComputeTOI", &err);
	if (!ComputeTOIKernel || err != CL_SUCCESS)
	{
		printf("Error: Failed to create compute kernel!\n");
		exit(1);
	}
	b2CLDevice::instance().getMaximumKernelWorkGroupSize(ComputeTOIKernel, ComputeTOIKernelWorkGroupSize);
		lastNumContacts = 0;
	sortCount = 0;
	ContactTOIsBuffer = NULL;
	ContactIndicesBuffer = NULL;

	//create the AdvanceBodiesKernel
	AdvanceBodiesKernel = clCreateKernel (SolveTOIProgram, "b2clAdvanceBodiesKernel", &err); 
	if (!AdvanceBodiesKernel || err != CL_SUCCESS)
	{
		printf("Error: Failed to create AdvanceBodiesKernel kernel!\n");
		exit(1);
	}
    b2CLDevice::instance().getMaximumKernelWorkGroupSize(AdvanceBodiesKernel, ComputeTOIKernelWorkGroupSize);

	this->MarkConnectedContactsKernel = clCreateKernel (SolveTOIProgram, "b2clMarkConnectedContactsKernel", &err); 
	if (!MarkConnectedContactsKernel || err != CL_SUCCESS) {
		printf("Error: Failed to create MarkConnectedContactsKernel kernel!\n");
		exit(1);
	}
#endif
	syncMovedBodyKernel = clCreateKernel (SolveTOIProgram, "b2clsyncMovedBodyKernel", &err); 
	if (!syncMovedBodyKernel || err != CL_SUCCESS) {
		printf("Error: Failed to create syncMovedBodyKernel kernel!\n");
		exit(1);
	}
	b2CLDevice::instance().getMaximumKernelWorkGroupSize(syncMovedBodyKernel, ComputeTOIKernelWorkGroupSize);
	movedBodyList = NULL ; bodyListSize = 0 ; 
	this->copyAllPositionList = NULL ; this->copyAllVelocityList = NULL; this->copyAllXFList = NULL ; copyBodyNum = 0 ; 

}

b2CLSolveTOI::~b2CLSolveTOI()
{
	if (ContactTOIsBuffer)
		b2CLDevice::instance().freeArray(ContactTOIsBuffer);
	if (ContactIndicesBuffer)
		b2CLDevice::instance().freeArray(ContactIndicesBuffer);
	if (movedBodyList != NULL ) delete [] movedBodyList ; 
}

b2CLSolveTOI& b2CLSolveTOI::instance()
{
	static b2CLSolveTOI inst;
	return inst;
}

void b2CLSolveTOI::CopyUpdatedBodytoDevice (b2Body* b, b2CLSolver* solver){


	b2clTransform        xfListData;
	b2clBodyDynamic      bodyDynamicListData;

	int body_index = b->m_uid;
	


	xfListData.p[0] = b->m_xf.p.x;
	xfListData.p[1] = b->m_xf.p.y;
	xfListData.q[0] = b->m_xf.q.s;
	xfListData.q[1]  = b->m_xf.q.c;
	
	bodyDynamicListData.m_sweep.localCenter[0] = b->m_sweep.localCenter.x;
	bodyDynamicListData.m_sweep.localCenter[1] = b->m_sweep.localCenter.y;
	bodyDynamicListData.m_sweep.c0[0] = b->m_sweep.c0.x;
	bodyDynamicListData.m_sweep.c0[1] = b->m_sweep.c0.y;
	bodyDynamicListData.m_sweep.c[0] = b->m_sweep.c.x;
	bodyDynamicListData.m_sweep.c[1] = b->m_sweep.c.y;
	bodyDynamicListData.m_sweep.a0 = b->m_sweep.a0;
	bodyDynamicListData.m_sweep.a = b->m_sweep.a;
	bodyDynamicListData.m_sweep.alpha0 = b->m_sweep.alpha0;

	bodyDynamicListData.m_linearVelocity[0] = b->m_linearVelocity.x;
	bodyDynamicListData.m_linearVelocity[1] = b->m_linearVelocity.y;
	bodyDynamicListData.m_force[0] = b->m_force.x;
	bodyDynamicListData.m_force[1] = b->m_force.y;
	bodyDynamicListData.m_angularVelocity = b->m_angularVelocity;
	bodyDynamicListData.m_torque = b->m_torque;

	bodyDynamicListData.m_last_uid = b->m_last_uid;

	b2Velocity velocity ;
	velocity.v = b->m_linearVelocity ; 
	velocity.w = b->m_angularVelocity ; 
	b2Position pos ; 
	pos.c = b->m_sweep.c ; 
	pos.a = b->m_sweep.a ; 

	// update position and velocity
	b2CLDevice::instance().copyArrayToDevice(solver->clb2VelocitiesBuffer, &velocity, sizeof(float32)*3 * body_index, sizeof(float32)*3,true);
    b2CLDevice::instance().copyArrayToDevice(solver->clb2PositionsBuffer,  &pos,      sizeof(float32)*3 * body_index, sizeof(float32)*3,true);


	// Copy data from CPU to GPU
	//if (! b2CLCommonData::instance().xfListBuffer) {
	//	 printf ("?"); 
	//}
	b2CLDevice::instance().copyArrayToDevice( b2CLCommonData::instance().xfListBuffer, &xfListData, sizeof(b2clTransform)* body_index, sizeof(b2clTransform) * 1, true);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().bodyDynamicListBuffer, &bodyDynamicListData, sizeof(b2clBodyDynamic)* body_index, sizeof(b2clBodyDynamic) * 1, true);
	

}

void b2CLSolveTOI::syncMovedBodytoDevice( std::set<b2Body*> & movedBodySet, b2CLSolver* solver){
	if (movedBodySet.size() == 0) return ; 
	if (movedBodySet.size () > this->bodyListSize ) {
		if (this->movedBodyList != NULL) { 
			 delete [] this->movedBodyList;
			 b2CLDevice::instance().freeArray ( movedBodyBuffer ) ; 
		}
		movedBodyList = new float [movedBodySet.size() * 11]; 
		this->movedBodyBuffer = b2CLDevice::instance().allocateArray(sizeof(float32) * movedBodySet.size()*11);
	}
	int bodySize = movedBodySet.size() ;
	//for (int i = 0 ; i < bodySize ; i ++ ) {
	int i = 0 ; 
	for (std::set<b2Body*>::iterator iter = movedBodySet.begin(); iter != movedBodySet.end(); iter ++) {
		b2Body* b = *iter ; 
		movedBodyList[i*11+0] = (float) (b->m_uid);
		movedBodyList[i*11+1] = b->m_linearVelocity.x;
		movedBodyList[i*11+2] = b->m_linearVelocity.y;
		movedBodyList[i*11+3] = b->m_angularVelocity;
		movedBodyList[i*11+4] = b->m_sweep.c.x ;
		movedBodyList[i*11+5] = b->m_sweep.c.y ;
		movedBodyList[i*11+6] = b->m_sweep.a;
		movedBodyList[i*11+7] = b->m_xf.p.x;
		movedBodyList[i*11+8] = b->m_xf.p.y;
		movedBodyList[i*11+9] = b->m_xf.q.s;
		movedBodyList[i*11+10] = b->m_xf.q.c;
		i ++ ; 
	}


	
	b2CLDevice::instance().copyArrayToDevice( movedBodyBuffer, movedBodyList, 0, sizeof(float) * 11 * bodySize, true);
	//b2CLDevice::instance().copyArrayToDevice( movedBodyBuffer, movedBodyList, 0, 1, true);

	// set arguments
	int err = CL_SUCCESS;
	int a = 0;

	err |= clSetKernelArg(this->syncMovedBodyKernel, a++, sizeof(cl_mem), &movedBodyBuffer);
	err |= clSetKernelArg(this->syncMovedBodyKernel, a++, sizeof(int), &bodySize);
	err |= clSetKernelArg(this->syncMovedBodyKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().xfListBuffer);
	err |= clSetKernelArg(this->syncMovedBodyKernel, a++, sizeof(cl_mem), &solver->clb2VelocitiesBuffer);
	err |= clSetKernelArg(this->syncMovedBodyKernel,  a++, sizeof(cl_mem), &solver->clb2PositionsBuffer);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) this->syncMovedBodyKernel);
        exit(1);
    }
	
	// kernel call
	int numBlocks=(bodySize + ComputeTOIKernelWorkGroupSize - 1)/ComputeTOIKernelWorkGroupSize;
	size_t global = numBlocks*ComputeTOIKernelWorkGroupSize;
	err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), this->syncMovedBodyKernel, 1, NULL, &global, &ComputeTOIKernelWorkGroupSize, 0, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to execute b2clComputeMinTOI kernel!\n");
		exit(1);
	}

}

void b2CLSolveTOI::CopyAllBodytoDevice (b2Body* bList, b2CLSolver* solver,  int bodyNum ) {

/*
	b2clTransform        xfListData[;
	b2clBodyDynamic      bodyDynamicListData;

	int body_index = b->m_uid;
	


	xfListData.p[0] = b->m_xf.p.x;
	xfListData.p[1] = b->m_xf.p.y;
	xfListData.q[0] = b->m_xf.q.s;
	xfListData.q[1]  = b->m_xf.q.c;
	
	bodyDynamicListData.m_sweep.localCenter[0] = b->m_sweep.localCenter.x;
	bodyDynamicListData.m_sweep.localCenter[1] = b->m_sweep.localCenter.y;
	bodyDynamicListData.m_sweep.c0[0] = b->m_sweep.c0.x;
	bodyDynamicListData.m_sweep.c0[1] = b->m_sweep.c0.y;
	bodyDynamicListData.m_sweep.c[0] = b->m_sweep.c.x;
	bodyDynamicListData.m_sweep.c[1] = b->m_sweep.c.y;
	bodyDynamicListData.m_sweep.a0 = b->m_sweep.a0;
	bodyDynamicListData.m_sweep.a = b->m_sweep.a;
	bodyDynamicListData.m_sweep.alpha0 = b->m_sweep.alpha0;

	bodyDynamicListData.m_linearVelocity[0] = b->m_linearVelocity.x;
	bodyDynamicListData.m_linearVelocity[1] = b->m_linearVelocity.y;
	bodyDynamicListData.m_force[0] = b->m_force.x;
	bodyDynamicListData.m_force[1] = b->m_force.y;
	bodyDynamicListData.m_angularVelocity = b->m_angularVelocity;
	bodyDynamicListData.m_torque = b->m_torque;

	bodyDynamicListData.m_last_uid = b->m_last_uid;
*/

	//b2Velocity velocity[9000] ;
	//b2Position pos[9000] ; 
	//b2Transform xf[9000] ; 

	if (bodyNum == 0) return ; 
	if (bodyNum > copyBodyNum) {
		if (copyBodyNum > 0 ) {
			delete [] this->copyAllPositionList;
			delete [] this->copyAllVelocityList ;
			delete [] this->copyAllXFList ; 
		}
		this->copyAllPositionList = new b2Position [bodyNum];
		this->copyAllVelocityList = new b2Velocity [bodyNum];
		this->copyAllXFList = new b2Transform [bodyNum];
		copyBodyNum = bodyNum ; 
	}

	int totalNum = 0 ; 


	for ( b2Body* b = bList ; b; b = b->GetNext()  ) {
		this->copyAllVelocityList[totalNum].v = b->m_linearVelocity ; 
		this->copyAllVelocityList[totalNum].w = b->m_angularVelocity ; 
		this->copyAllPositionList[totalNum].c = b->m_sweep.c ; 
		this->copyAllPositionList[totalNum].a = b->m_sweep.a ; 
		this->copyAllXFList[totalNum].p = b->m_xf.p ; 
		this->copyAllXFList[totalNum].q = b->m_xf.q ; 
		totalNum ++ ; 
	}
	


	// update position and velocity
	b2CLDevice::instance().copyArrayToDevice(solver->clb2VelocitiesBuffer, this->copyAllVelocityList, 0, sizeof(float32)*3 * totalNum,true);
    b2CLDevice::instance().copyArrayToDevice(solver->clb2PositionsBuffer,  this->copyAllPositionList,      0, sizeof(float32)*3 * totalNum,true);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().xfListBuffer, this->copyAllXFList,      0, sizeof(b2Transform) * totalNum,true);



	//b2CLDevice::instance().copyArrayToDevice( b2CLCommonData::instance().xfListBuffer, &xfListData, sizeof(b2clTransform)* body_index, sizeof(b2clTransform) * 1, true);
	//b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().bodyDynamicListBuffer, &bodyDynamicListData, sizeof(b2clBodyDynamic)* body_index, sizeof(b2clBodyDynamic) * 1, true);
}

bool b2CLSolveTOI::ComputeMinTOI(int numContacts, bool firstTime, int numLoop)
{
	if (numContacts == 0)
		return false;

	if (numContacts > lastNumContacts)
	{
#if defined(USE_CPU_SORT)
		//sortCount = shape_num;
#else
		if (numContacts < BITONIC_SORT_INTEL_MINNUM)
			sortCount = BITONIC_SORT_INTEL_MINNUM;
		else
		{
			// compute the least power-of-2 which >= m_contactCount
			int exp;
			frexp((float)numContacts, &exp);
			sortCount = 1 << (exp-1);
			if (sortCount < numContacts)
				sortCount <<= 1;
		}
#endif

		if (ContactTOIsBuffer)
			b2CLDevice::instance().freeArray(ContactTOIsBuffer);
		ContactTOIsBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * sortCount);

		if (ContactIndicesBuffer)
			b2CLDevice::instance().freeArray(ContactIndicesBuffer);
		ContactIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * sortCount);

		unsigned int *zeroBuffer = new unsigned int[sortCount - numContacts];
		memset(zeroBuffer, 0xFF, sizeof(int) * (sortCount - numContacts));
		b2CLDevice::instance().copyArrayToDevice(ContactTOIsBuffer, zeroBuffer, 
												 sizeof(int) *  numContacts, sizeof(int) * (sortCount - numContacts), true);
		delete [] zeroBuffer;

		lastNumContacts = numContacts;
	}

	int err = CL_SUCCESS;
	int a = 0;

	int resetAlpha0 = firstTime ? 1 : 0;

	// set arguments
	err |= clSetKernelArg(ComputeTOIKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().globalIndicesBuffer);
	err |= clSetKernelArg(ComputeTOIKernel, a++, sizeof(int), &numContacts);
	err |= clSetKernelArg(ComputeTOIKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyStaticListBuffer);
	err |= clSetKernelArg(ComputeTOIKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyDynamicListBuffer);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().shapeListBuffer);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(cl_mem), &ContactTOIsBuffer);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(cl_mem), &ContactIndicesBuffer);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(int), &resetAlpha0);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(cl_mem), &TOITimesBuffer);
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(int), &numLoop);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) ComputeTOIKernel);
        exit(1);
    }
	

	// kernel call
	int numBlocks=(numContacts + ComputeTOIKernelWorkGroupSize - 1)/ComputeTOIKernelWorkGroupSize;
	size_t global = numBlocks*ComputeTOIKernelWorkGroupSize;
	err = clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), ComputeTOIKernel, 1, NULL, &global, &ComputeTOIKernelWorkGroupSize, 0, NULL, NULL);
	if (err != CL_SUCCESS)
	{
		printf("Error: Failed to execute b2clComputeMinTOI kernel!\n");
		exit(1);
	}

	// Sort to find min TOI and contact index.
	// TODO : should be changed to parallel reduction to improve the performance
#if defined(USE_CPU_SORT)
	b2CLSort::instance().stlSort(ContactTOIsBuffer, ContactIndicesBuffer, sortCount, 1);
#else
	b2CLSort::instance().bitonicSort_Intel(ContactTOIsBuffer, ContactIndicesBuffer, sortCount, 1);
#endif

	// The first element of ContactTOIsBuffer is minTOI
	// The first element of ContactIndicesBuffer is contact index of minTOI

	// return false if first TOI >= 1.0
	unsigned int minTOI;
	b2CLDevice::instance().copyArrayFromDevice(&minTOI, ContactTOIsBuffer, 0, sizeof(unsigned int), true);
	if (FFlip(minTOI) >= 1.0f)
		return false;

	this->minTOI = FFlip(minTOI);
	b2CLDevice::instance().copyArrayFromDevice (&this->indexMinContact, this->ContactIndicesBuffer, 0, sizeof(int), true) ; 
	return true;
}



void b2CLSolveTOI::AdvanceBodies()
{
	int numContacts = 1 ; 

	int err = CL_SUCCESS;
	int a = 0;
	err |= clSetKernelArg(AdvanceBodiesKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().globalIndicesBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel, a++, sizeof(int), &numContacts);
	err |= clSetKernelArg(AdvanceBodiesKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyStaticListBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel, a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyDynamicListBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().shapeListBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
	err |= clSetKernelArg(ComputeTOIKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().currentManifoldBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel,  a++, sizeof(cl_mem), &ContactTOIsBuffer);
	err |= clSetKernelArg(AdvanceBodiesKernel,  a++, sizeof(cl_mem), &ContactIndicesBuffer);

	int numBlocks=(numContacts + ComputeTOIKernelWorkGroupSize - 1)/ComputeTOIKernelWorkGroupSize;
	size_t global = numBlocks * ComputeTOIKernelWorkGroupSize; 
	err = clEnqueueNDRangeKernel ( b2CLDevice::instance().GetCommandQueue(), AdvanceBodiesKernel,1, NULL, &global, &ComputeTOIKernelWorkGroupSize, 0, NULL, NULL ) ; 
	if (err != CL_SUCCESS)
	{
		printf ("Error:Failed to execute b2clAdvanceBodies kernel! \n");
		exit(1); 
	}
	return ; 
}

void b2CLSolveTOI::GetAllConnectedBodies (int numContactsFromBF) {
	int numSeedContacts = 1 ; 
	cl_mem isConnectedBuffer ; 
	//b2CLCommonData::all
	isConnectedBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * numContactsFromBF);
	int* zeroBuffer = new int[numContactsFromBF];
	memset (zeroBuffer, 0, sizeof(int)* numContactsFromBF); 
	b2CLDevice::instance().copyArrayToDevice ( isConnectedBuffer, zeroBuffer, 0, sizeof(int) * numContactsFromBF, true ) ; 
	delete [] zeroBuffer;
	for (int i = 0 ; i < numSeedContacts; i ++ ) {
		int seedIndex = 0 ; 
		int err = CL_SUCCESS;
		int a = 0;
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().globalIndicesBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(int), &numContactsFromBF);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyStaticListBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().bodyDynamicListBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &b2CLCommonData::instance().shapeListBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &ContactTOIsBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &ContactIndicesBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(cl_mem), &isConnectedBuffer);
		err |= clSetKernelArg(MarkConnectedContactsKernel,  a++, sizeof(int), &seedIndex); 
		int numBlocks = (numContactsFromBF + ComputeTOIKernelWorkGroupSize - 1)/ComputeTOIKernelWorkGroupSize;
		size_t global = numBlocks * ComputeTOIKernelWorkGroupSize; 
		err = clEnqueueNDRangeKernel ( b2CLDevice::instance().GetCommandQueue(), MarkConnectedContactsKernel,1, NULL, &global, &ComputeTOIKernelWorkGroupSize, 0, NULL, NULL ) ; 
		if (err != CL_SUCCESS)
		{
			printf ("Error:Failed to execute b2clAdvanceBodies kernel! \n");
			exit(1); 
		}
	}
	
	int* cpuIsConnectedBuffer = new int [numContactsFromBF]; 
	b2CLDevice::instance().copyArrayFromDevice(cpuIsConnectedBuffer, isConnectedBuffer, 0, sizeof(int)* numContactsFromBF, true ) ; 
	numValidContacts = 0 ;
	int maxConnectPerPair = 5 ; 
	int* cpuValidContactsIndicesBuffer = new int [numSeedContacts * maxConnectPerPair]; 
	for (int i = 0 ; i < numContactsFromBF; i ++ ){
		if (cpuIsConnectedBuffer == 0 ) continue ; 
		cpuValidContactsIndicesBuffer[numValidContacts] = i; 
		numValidContacts ++ ; 
	}
	b2CLDevice::instance().copyArrayToDevice (b2CLCommonData::instance().validContactIndicesBuffer,cpuValidContactsIndicesBuffer,0,sizeof(int)*numSeedContacts * maxConnectPerPair, true ) ; 
	b2CLDevice::instance().copyArrayToDevice (b2CLScan::instance().numValidDataBuffer, &numValidContacts, 0, sizeof(unsigned int), true );
	delete [] cpuIsConnectedBuffer ; 

}

void b2CLSolveTOI::GenerateTimeStep(b2TimeStep & subStep, b2TimeStep step ){
	
	unsigned int minTOI;
	b2CLDevice::instance().copyArrayFromDevice(&minTOI, ContactTOIsBuffer, 0, sizeof(unsigned int), true);
	float minalpha = FFlip (minTOI);
	subStep.dt = (1.0f - minalpha ) * step.dt; 
	subStep.inv_dt = 1.0f / subStep.dt;
	subStep.dtRatio = 1.0f ; 
	subStep.positionIterations = 20 ; 
	subStep.velocityIterations = step.velocityIterations ; 
	subStep.warmStarting = false ; 
}

