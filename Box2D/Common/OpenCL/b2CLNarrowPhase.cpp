/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#include <iostream>
#include <Box2D/Common/OpenCL/b2CLNarrowPhase.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Dynamics/Contacts/b2PolygonContact.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Common/OpenCL/b2CLScan.h>

b2CLNarrowPhase::b2CLNarrowPhase()
{
#if defined(NARROWPHASE_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		printf("Initializing b2CLNarrowPhase...\n");
    
		int err;
    
		//load opencl programs from files
		char* narrowPhaseKernelSource = 0;
		size_t narrowPhaseKernelSourceLen = 0;

		shrLog("...loading b2CLNarrowPhase.cl\n");
	#if defined(SCAN_OPENCL)

#ifdef linux
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLNarrowPhase.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#elif defined (_WIN32)
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLNarrowPhase.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLNarrowPhase.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#else
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLNarrowPhase.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#endif
        
	#else
        
#ifdef linux
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLNarrowPhase_Alone.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#elif defined (_WIN32)
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLNarrowPhase_Alone.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#elif defined (__EMSCRIPTEN__)
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLNarrowPhase_Alone.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#else
        narrowPhaseKernelSource = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLNarrowPhase_Alone.cl", NULL), "// My comment\n", &narrowPhaseKernelSourceLen);
#endif
        
	#endif
		if(narrowPhaseKernelSource == NULL)
		{
			b2Log("Could not load program source, is path 'b2CLNarrowPhase.cl' correct?");
		}

		//create the compute program from source kernel code
		narrowPhaseProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**)&narrowPhaseKernelSource, NULL, &err);
		if (!narrowPhaseProgram)
		{
			printf("Error: Failed to create compute program!\n");
			exit(1);
		}
    
		//build the program
		err = clBuildProgram(narrowPhaseProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			size_t len;
			char buffer[204800];
        
			printf("Error: Failed to build program executable!\n");
			clGetProgramBuildInfo(narrowPhaseProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
			printf("%s\n", buffer);
			exit(1);
		}
    
		//create the compute kernel
		collidePolygonsKernel = clCreateKernel(narrowPhaseProgram, "b2clCollidePolygons", &err);
		if (!collidePolygonsKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		collideCirclesKernel = clCreateKernel(narrowPhaseProgram, "b2clCollideCircles", &err);
		if (!collideCirclesKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		collidePolygonAndCircleKernel = clCreateKernel(narrowPhaseProgram, "b2clCollidePolygonAndCircle", &err);
		if (!collidePolygonAndCircleKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		collideEdgeAndCircleKernel = clCreateKernel(narrowPhaseProgram, "b2clCollideEdgeAndCircle", &err);
		if (!collideEdgeAndCircleKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		collideEdgeAndPolygonKernel = clCreateKernel(narrowPhaseProgram, "b2clCollideEdgeAndPolygon", &err);
		if (!collideEdgeAndPolygonKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}

		b2CLDevice::instance().getMaximumKernelWorkGroupSize(collidePolygonsKernel, kernel_work_group_size);
		//clGetKernelWorkGroupInfo(collidePolygonsKernel, b2CLDevice::instance().GetCurrentDevice(), CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(size_t), &kernel_preferred_work_group_size_multiple, NULL);

		//create the compute kernel
		compactForOneContactKernel = clCreateKernel(narrowPhaseProgram, "b2clCompactForOneContact", &err);
		if (!compactForOneContactKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}

		/*old_xf_num = */old_contact_num = 0;
		xfListData = NULL;
		b2CLCommonData::instance().manifoldListData = NULL;
		globalIndicesData = pairIndicesData = NULL;
	#if !defined(BROADPHASE_OPENCL)
		b2CLCommonData::instance().manifoldListBuffers[0] = 
			b2CLCommonData::instance().manifoldListBuffers[1] =
			b2CLCommonData::instance().pairIndicesBuffer = 
	#endif
	#if defined(SCAN_OPENCL)
			b2CLCommonData::instance().manifoldBinaryBitListBuffer = 
			b2CLCommonData::instance().validContactIndicesBuffer = 
	#endif
			NULL;
		
		b2CLScan::instance(); // initializing b2CLScan
	}
#endif
	b2CLCommonData::instance().currentManifoldBuffer = 0;
	manifold_nums[0] = 0;
	manifold_nums[1] = 0;
}

b2CLNarrowPhase::~b2CLNarrowPhase()
{
}

//void b2CLNarrowPhase::CreateXfBuffer(b2World *m_pWorld)
//{
//	// Initialize data on CPU
//	
//	if (old_xf_num < m_pWorld->m_bodyCount)
//	{
//		if (xfListData)
//			delete [] xfListData;
//		xfListData = new b2clTransform[m_pWorld->m_bodyCount];
//
//		if (b2CLCommonData::instance().xfListBuffer)
//		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().xfListBuffer);
//	    b2CLCommonData::instance().xfListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clTransform) * m_pWorld->m_bodyCount);
//
//		old_xf_num = m_pWorld->m_bodyCount;
//		//cout << "New xf list" << endl;
//	}
//}

void b2CLNarrowPhase::InitializeGPUData(b2World *m_pWorld, b2Contact *m_contactList, int32 *m_pContactCounts)
{
	int contactCount = m_pWorld->m_contactManager.m_contactCount;
	if (old_contact_num < contactCount)
	{
#if !defined(SOLVER_OPENCL)
		if (b2CLCommonData::instance().manifoldListData)
			delete [] b2CLCommonData::instance().manifoldListData;
		b2CLCommonData::instance().manifoldListData = new b2clManifold[contactCount];
#else
		if (m_pWorld->m_uListenerCallback)
		{
			if (b2CLCommonData::instance().manifoldListData)
				delete [] b2CLCommonData::instance().manifoldListData;
			b2CLCommonData::instance().manifoldListData = new b2clManifold[contactCount];

			if (b2CLCommonData::instance().globalIndices)
				delete [] b2CLCommonData::instance().globalIndices;
			b2CLCommonData::instance().globalIndices = new int[contactCount*4];
		}
#endif

#if !defined(BROADPHASE_OPENCL)
		if (globalIndicesData)
			delete [] globalIndicesData;
		globalIndicesData = new int32[contactCount * 4];

		if (pairIndicesData)
			delete [] pairIndicesData;
		pairIndicesData = new int32[contactCount * b2Shape::contact_type_num];
#endif

#if defined(SCAN_OPENCL)
		if (b2CLCommonData::instance().manifoldBinaryBitListBuffer)
		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().manifoldBinaryBitListBuffer);
		b2CLCommonData::instance().manifoldBinaryBitListBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * contactCount);

		if (b2CLCommonData::instance().validContactIndicesBuffer)
		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().validContactIndicesBuffer);
		b2CLCommonData::instance().validContactIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * contactCount);
#endif

		//if (b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer])
		//    b2CLDevice::instance().freeArray(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]);
		//b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer] = b2CLDevice::instance().allocateArray(sizeof(b2clManifold) * contactCount);

#if !defined(BROADPHASE_OPENCL)
		if (b2CLCommonData::instance().globalIndicesBuffer)
		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().globalIndicesBuffer);
		b2CLCommonData::instance().globalIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int32) * contactCount * 4);

		if (b2CLCommonData::instance().pairIndicesBuffer)
		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().pairIndicesBuffer);
		b2CLCommonData::instance().pairIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int32) * contactCount * b2Shape::contact_type_num);
#endif

		old_contact_num = contactCount;
		//cout << "New contact list" << endl;
	}

	// switch current manifold buffer
	b2CLCommonData::instance().currentManifoldBuffer = 1-b2CLCommonData::instance().currentManifoldBuffer;

	if (manifold_nums[b2CLCommonData::instance().currentManifoldBuffer] < contactCount)
	{
		if (b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer])
		    b2CLDevice::instance().freeArray(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]);
		b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer] = b2CLDevice::instance().allocateArray(sizeof(b2clManifold) * contactCount);

		manifold_nums[b2CLCommonData::instance().currentManifoldBuffer] = contactCount;
	}

#if !defined(BROADPHASE_OPENCL) // possible problem for (BP && !SOLVER)
	b2Contact* c = m_contactList;
	int contact_index = 0;
	int maxPairs = m_pWorld->GetProxyCount() * MAX_CONTACT_PER_FIXTURE;
	memset(m_pContactCounts, 0, sizeof(int)*b2Shape::contact_type_num);

	while (c)
	{
		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();

		// Set c_type according to s_type_i and s_type_j
		// Suppose only have circle (0), edge (1), and polygon (2) at this time
		// 0: circle-circle
		// 1: circle-polygon (A is polygon and B is circle)
		// 2: polygon-polygon
		// 3: edge-circle (A is edge and B is circle)
		// 4: edge-polygon (A is edge and B is polygon)
		// note that edge-edge is NOT supported in Box2D
		int c_type;
		b2Shape::Type typeA = fixtureA->GetType();
		b2Shape::Type typeB = fixtureB->GetType();

		if (typeA==b2Shape::e_chain)
			globalIndicesData[contact_index*4] = fixtureA->m_proxies[c->m_indexA].m_uid;
		else
			globalIndicesData[contact_index*4] = fixtureA->m_uid;
		if (typeB==b2Shape::e_chain)
			globalIndicesData[contact_index*4+1] = fixtureB->m_proxies[c->m_indexB].m_uid;	
		else
			globalIndicesData[contact_index*4+1] = fixtureB->m_uid;	
		globalIndicesData[contact_index*4+2] = bodyA->m_uid;
		globalIndicesData[contact_index*4+3] = bodyB->m_uid;

		// treat b2Shape::e_chain as b2Shape::e_edge
		if (typeA==b2Shape::e_chain) typeA = b2Shape::e_edge;
		if (typeB==b2Shape::e_chain) typeB = b2Shape::e_edge;

		if (typeA == b2Shape::e_circle)
		{
			if (typeB == b2Shape::e_circle)
			{
				c_type = 0;
			}
			else if (typeB == b2Shape::e_edge)
			{
				c_type = 3;
				// swap the two shpaes to make sure A is edge and B is circle
				globalIndicesData[contact_index*4] = fixtureB->m_uid;
				globalIndicesData[contact_index*4+1] = fixtureA->m_uid;
				globalIndicesData[contact_index*4+2] = bodyB->m_uid;
				globalIndicesData[contact_index*4+3] = bodyA->m_uid;
			}
			else // typeB == b2Shape::e_polygon
			{
				c_type = 1;
				// swap the two shpaes to make sure A is polygon and B is circle
				globalIndicesData[contact_index*4] = fixtureB->m_uid;
				globalIndicesData[contact_index*4+1] = fixtureA->m_uid;
				globalIndicesData[contact_index*4+2] = bodyB->m_uid;
				globalIndicesData[contact_index*4+3] = bodyA->m_uid;
			}
		}
		else if (typeA == b2Shape::e_edge)
		{
			if (typeB == b2Shape::e_circle)
			{
				c_type = 3;
			}
			else if (typeB == b2Shape::e_edge)
			{
				assert(0); // this should never happen
			}
			else // typeB == b2Shape::e_polygon
			{
				c_type = 4;
			}
		}
		else // typeA == b2Shape::e_polygon
		{
			if (typeB == b2Shape::e_circle)
			{
				c_type = 1;
			}
			else if (typeB == b2Shape::e_edge)
			{
				c_type = 2;
				// swap the two shpaes to make sure A is edge and B is polygon
				globalIndicesData[contact_index*4] = fixtureB->m_uid;
				globalIndicesData[contact_index*4+1] = fixtureA->m_uid;
				globalIndicesData[contact_index*4+2] = bodyB->m_uid;
				globalIndicesData[contact_index*4+3] = bodyA->m_uid;
			}
			else // typeB == b2Shape::e_polygon
			{
				c_type = 2;
			}
		}
		pairIndicesData[contactCount*c_type + m_pContactCounts[c_type]] = contact_index;
		m_pContactCounts[c_type]++;

	#if !defined(SOLVER_OPENCL)
		b2clManifold *manifoldListData = b2CLCommonData::instance().manifoldListData;
		for (int i=0; i<c->m_manifold.pointCount; i++)
		{
			manifoldListData[contact_index].points[i].id.key = c->m_manifold.points[i].id.key;
			manifoldListData[contact_index].points[i].normalImpulse = c->m_manifold.points[i].normalImpulse;
			manifoldListData[contact_index].points[i].tangentImpulse = c->m_manifold.points[i].tangentImpulse;
		}
		manifoldListData[contact_index].pointCount = c->m_manifold.pointCount;
	#endif

		c = c->GetNext();
		contact_index++;
	}
	assert(contactCount == contact_index);

#if !defined(SOLVER_OPENCL)
	// Copy data from CPU to GPU
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 
		b2CLCommonData::instance().manifoldListData, 0, sizeof(b2clManifold) * contactCount);
#endif

	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().globalIndicesBuffer, globalIndicesData, 0, sizeof(int32) * contactCount * 4);
	b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().pairIndicesBuffer, pairIndicesData, 0, sizeof(int32) * contactCount * b2Shape::contact_type_num);
#endif
}

void b2CLNarrowPhase::UpdateContactPairs(int contactNum, int *pContactNums, int maxContactNum/*, b2ContactListener* listener*/)
{
	//// for debug
	//int *fill_data = new int[contactNum];
	//for (int i=0; i<contactNum; i++)
	//	fill_data[i] = 231;
	//b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().manifoldBinaryBitListBuffer, fill_data, 0, sizeof(int) * contactNum);
	//delete [] fill_data;

	//// for debug
	//int* globalIndices = new int[50*4];
	//b2CLDevice::instance().copyArrayFromDevice(globalIndices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*4*50, true);
	//int* indices = new int[200];
	//b2CLDevice::instance().copyArrayFromDevice(indices, b2CLCommonData::instance().pairIndicesBuffer, 0, sizeof(int)*200, true);
	//int *test = new int[contactNum];
 //	memset(test, 0, sizeof(int)*contactNum);
	//for (int i=0; i<contactNum; i++)
	//	test[indices[i]] = 1;
	//for (int i=0; i<contactNum; i++)
	//	if (test[i]==0)
	//		int a = 1;
	//delete [] test;
	//delete [] indices;
	//delete [] globalIndices;

  	for (int contactType=0; contactType<b2Shape::contact_type_num; contactType++)
	{
		if (pContactNums[0]!=0)
			int a = 0;
		if (pContactNums[contactType]>0)
		{
			unsigned int a = 0;

			cl_kernel collideKernel;

			switch (contactType)
			{
			case 0: // circle-circle
				collideKernel = collideCirclesKernel;
				break;
			case 1: // circle-polygon
				collideKernel = collidePolygonAndCircleKernel;
				break;
			case 2: // polygon-polygon
				collideKernel = collidePolygonsKernel;
				break;
			case 3: // edge-circle
				collideKernel = collideEdgeAndCircleKernel;
				break;
			case 4: // edge-polygon
				collideKernel = collideEdgeAndPolygonKernel;
				break;
			default:
				printf("Error! Unsupported contact type: %d!\n", contactType);
				exit(0);
			}
        
			int err = CL_SUCCESS;
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
		#if defined(SCAN_OPENCL)
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().manifoldBinaryBitListBuffer));
		#endif
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().shapeListBuffer));
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().xfListBuffer));
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().globalIndicesBuffer));
			err |= clSetKernelArg(collideKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().pairIndicesBuffer));
			err |= clSetKernelArg(collideKernel,  a++, sizeof(int), &maxContactNum);
			err |= clSetKernelArg(collideKernel,  a++, sizeof(int), pContactNums+contactType);
			if (err != CL_SUCCESS)
			{
				printf("Error: %s: Failed to set kernel arguments!\n", (char *) collidePolygonsKernel);
				return;
			}

			int group_num = (pContactNums[contactType] + kernel_work_group_size-1)/kernel_work_group_size;
        
			size_t global = group_num * kernel_work_group_size;
			//cout << contactNum << ", " << group_num << endl;
			err = CL_SUCCESS;
			err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), collideKernel, 1, NULL, &global, &kernel_work_group_size, 0, NULL, NULL);
			if (err != CL_SUCCESS)
			{
				printf("Error: Collide Kernel: Failed to execute kernel!\n");
				return;
			}

		#ifdef _DEBUG
			//// for debug
			//b2clManifold *testManifold = new b2clManifold[contactNum];
			//b2CLDevice::instance().copyArrayFromDevice(testManifold, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 0, sizeof(b2clManifold)*contactNum,true);

			//int* input = new int[contactNum];
			//b2CLDevice::instance().copyArrayFromDevice(input, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(int)*contactNum, true);
			//for (int i=0; i<contactNum; i++)
			//{
			//	if (input[i]!=0 && input[i]!=1)
			//		int a = 0;
			//}

			//delete [] testManifold;
			//delete [] input;
		#endif
		}
	}
}

void b2CLNarrowPhase::CompactContactPairs(int contactNum)
{
#if defined(SCAN_OPENCL)
	if (contactNum==1)
	{
		unsigned int a = 0;
        
		int err = CL_SUCCESS;
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().manifoldBinaryBitListBuffer));
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().validContactIndicesBuffer));
		if (!(b2CLScan::instance().numValidDataBuffer))
			b2CLScan::instance().numValidDataBuffer = b2CLDevice::instance().allocateArray(sizeof(cl_uint));
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLScan::instance().numValidDataBuffer));
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", (char *) collidePolygonsKernel);
			return;
		}

		size_t global = 1;
		//cout << contactNum << ", " << group_num << endl;
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactForOneContactKernel, 1, NULL, &global, NULL, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: Compact for One Contact: Failed to execute kernel!\n");
			return;
		}
	}
	else
	{
		//// for debug
		//int* input = new int[contactNum];
		//int* input2 = new int[contactNum];
		//b2CLDevice::instance().copyArrayFromDevice(input, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(input2, b2CLCommonData::instance().indicesToBeCompactListBuffer, 0, sizeof(int)*contactNum, true);
		//int n=0;
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (input2[i]>-1)
		//		n++;
		//}
		//
		////cl_mem tempBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);
		//cl_mem tempBuffer2 = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);

		//int *test = new int[contactNum];
		//for (int i=0; i<contactNum; i++)
		//	test[i] = -2;
		//b2CLDevice::instance().copyArrayToDevice(tempBuffer2, test, 0, sizeof(int)*contactNum);

		//cl_mem tempNum = b2CLDevice::instance().allocateArray(sizeof(cl_uint));

		//printf("\tCompactContactPairs: scan on CPU\n");
		//printf("\tcontactNum: %d\n", contactNum);

#if defined(USE_CPU_SCAN)
		int *manifoldBinaryBitList = new int[contactNum];
		int *scanResult = new int[contactNum];
		b2CLDevice::instance().copyArrayFromDevice(manifoldBinaryBitList, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(int)*contactNum, true);
		//for (int k=0; k<contactNum; k++)
		//{
		//	if (manifoldBinaryBitList[k]!=0 && manifoldBinaryBitList[k]!=1)
		//	{
		//		printf("ERROR FOUND!!!\n");
		//		exit(0);
		//	}
		//}
		scanResult[0] = 0;
		for (int i=1; i<contactNum; i++)
		{
			scanResult[i] = scanResult[i-1] + manifoldBinaryBitList[i-1];
			//if (scanResult[i]>i)
			//	int a = 0;
		}
        if (b2CLScan::instance().GetElementsAllocated()<contactNum)
        {
            if (b2CLScan::instance().scanResultsBuffer)
                b2CLDevice::instance().freeArray(b2CLScan::instance().scanResultsBuffer);
            b2CLScan::instance().scanResultsBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);
        }
		b2CLDevice::instance().copyArrayToDevice(b2CLScan::instance().scanResultsBuffer, scanResult, 0, sizeof(int)*contactNum, true);
		delete [] manifoldBinaryBitList;
		delete [] scanResult;
#else
		b2CLScan::instance().PreScanBuffer(b2CLCommonData::instance().manifoldBinaryBitListBuffer, contactNum);
		//b2CLScan::instance().ScanCLPP(b2CLCommonData::instance().manifoldBinaryBitListBuffer, contactNum);
#endif
		//printf("\tCompactContactPairs: scan on CPU finished\n");

		//// for debug
		//int* output = new int[contactNum];
		//int* output2 = new int[contactNum];
		//b2CLDevice::instance().copyArrayFromDevice(output, b2CLScan::instance().scanResultsBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(output2, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (output[i]>contactNum)
		//		int a = 0;
		//}

		//printf("\tCompactContactPairs: compact on GPU\n");
		b2CLScan::instance().ParallelCompactIndices(b2CLCommonData::instance().validContactIndicesBuffer, 
			b2CLCommonData::instance().manifoldBinaryBitListBuffer, 
			b2CLScan::instance().scanResultsBuffer, 
			contactNum);
        
		//// for debug
		//int* output = new int[contactNum];
		//int* output2 = new int[contactNum];
		//cl_uint nValidNum = 0;
		//b2CLDevice::instance().copyArrayFromDevice(output, b2CLScan::instance().scanResultsBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(output2, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(&nValidNum, b2CLScan::instance().numValidDataBuffer, 0, sizeof(cl_uint), true);
		//if (nValidNum>contactNum)
		//	int a = 0;
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (i<nValidNum && output[i]!=i)
		//		int a = 0;
		//}
		//printf("valid contactNum: %d\n", nValidNum);
		//delete [] output;
		//delete [] output2;
	}
#endif
}

void b2CLNarrowPhase::CompactEnabledContactPairs(int contactNum)
{
#if defined(SCAN_OPENCL)
	if (contactNum==1)
	{
		unsigned int a = 0;
        
		int err = CL_SUCCESS;
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().manifoldBinaryBitListBuffer));
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLCommonData::instance().validContactIndicesBuffer));
		if (!(b2CLScan::instance().numValidDataBuffer))
			b2CLScan::instance().numValidDataBuffer = b2CLDevice::instance().allocateArray(sizeof(cl_uint));
		err |= clSetKernelArg(compactForOneContactKernel,  a++, sizeof(cl_mem), &(b2CLScan::instance().numValidDataBuffer));
		if (err != CL_SUCCESS)
		{
			printf("Error: %s: Failed to set kernel arguments!\n", (char *) collidePolygonsKernel);
			return;
		}

		size_t global = 1;
		//cout << contactNum << ", " << group_num << endl;
		err = CL_SUCCESS;
		err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), compactForOneContactKernel, 1, NULL, &global, NULL, 0, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			printf("Error: Compact for One Contact: Failed to execute kernel!\n");
			return;
		}
	}
	else
	{
		//// for debug
		//int* input = new int[contactNum];
		//int* input2 = new int[contactNum];
		//b2CLDevice::instance().copyArrayFromDevice(input, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(input2, b2CLCommonData::instance().indicesToBeCompactListBuffer, 0, sizeof(int)*contactNum, true);
		//int n=0;
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (input2[i]>-1)
		//		n++;
		//}
		//
		////cl_mem tempBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);
		//cl_mem tempBuffer2 = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);

		//int *test = new int[contactNum];
		//for (int i=0; i<contactNum; i++)
		//	test[i] = -2;
		//b2CLDevice::instance().copyArrayToDevice(tempBuffer2, test, 0, sizeof(int)*contactNum);

		//cl_mem tempNum = b2CLDevice::instance().allocateArray(sizeof(cl_uint));

		//printf("\tCompactContactPairs: scan on CPU\n");
		//printf("\tcontactNum: %d\n", contactNum);

#if defined(USE_CPU_SCAN)
		int *manifoldBinaryBitList = new int[contactNum];
		int *scanResult = new int[contactNum];
		b2CLDevice::instance().copyArrayFromDevice(manifoldBinaryBitList, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(int)*contactNum, true);
		//for (int k=0; k<contactNum; k++)
		//{
		//	if (manifoldBinaryBitList[k]!=0 && manifoldBinaryBitList[k]!=1)
		//	{
		//		printf("ERROR FOUND!!!\n");
		//		exit(0);
		//	}
		//}
		scanResult[0] = 0;
		for (int i=1; i<contactNum; i++)
		{
			scanResult[i] = scanResult[i-1] + manifoldBinaryBitList[i-1];
			//if (scanResult[i]>i)
			//	int a = 0;
		}
        if (b2CLScan::instance().GetElementsAllocated()<contactNum)
        {
            if (b2CLScan::instance().scanResultsBuffer)
                b2CLDevice::instance().freeArray(b2CLScan::instance().scanResultsBuffer);
            b2CLScan::instance().scanResultsBuffer = b2CLDevice::instance().allocateArray(sizeof(int)*contactNum);
        }
		b2CLDevice::instance().copyArrayToDevice(b2CLScan::instance().scanResultsBuffer, scanResult, 0, sizeof(int)*contactNum, true);
		delete [] manifoldBinaryBitList;
		delete [] scanResult;
#else
		b2CLScan::instance().PreScanBuffer(b2CLCommonData::instance().manifoldBinaryBitListBuffer, contactNum);
#endif
		//printf("\tCompactContactPairs: scan on CPU finished\n");

		//// for debug
		//int* output = new int[contactNum];
		//int* output2 = new int[contactNum];
		//b2CLDevice::instance().copyArrayFromDevice(output, b2CLScan::instance().scanResultsBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(output2, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (output[i]>contactNum)
		//		int a = 0;
		//}

		//printf("\tCompactContactPairs: compact on GPU\n");
		cl_mem lastValidContactIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * contactNum); // can be optimized further by initialize it only once!!!
		b2CLDevice::instance().copyArrayInsideDevice(b2CLCommonData::instance().validContactIndicesBuffer, lastValidContactIndicesBuffer, sizeof(int)*contactNum);

		//// for debug
		//int *indicesBefore = new int[contactNum];
		//int *indicesAfter = new int[contactNum];
		//b2CLDevice::instance().copyArrayFromDevice(indicesBefore, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);

		b2CLScan::instance().ParallelCompactGeneral(b2CLCommonData::instance().validContactIndicesBuffer, 
			lastValidContactIndicesBuffer, 
			b2CLCommonData::instance().manifoldBinaryBitListBuffer, 
			b2CLScan::instance().scanResultsBuffer, 
			contactNum);
		b2CLDevice::instance().freeArray(lastValidContactIndicesBuffer);
        
		//// for debug
		//b2CLDevice::instance().copyArrayFromDevice(indicesAfter, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);
		//delete [] indicesBefore;
		//delete [] indicesAfter;

		//// for debug
		//int* output = new int[contactNum];
		//int* output2 = new int[contactNum];
		//cl_uint nValidNum = 0;
		//b2CLDevice::instance().copyArrayFromDevice(output, b2CLScan::instance().scanResultsBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(output2, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactNum, true);
		//b2CLDevice::instance().copyArrayFromDevice(&nValidNum, b2CLScan::instance().numValidDataBuffer, 0, sizeof(cl_uint), true);
		//if (nValidNum>contactNum)
		//	int a = 0;
		//for (int i=0; i<contactNum; i++)
		//{
		//	if (i<nValidNum && output[i]!=i)
		//		int a = 0;
		//}
		//printf("valid contactNum: %d\n", nValidNum);
		//delete [] output;
		//delete [] output2;
	}
#endif
}

void b2CLNarrowPhase::ReadbackGPUData(b2World *m_pWorld, b2Contact *m_contactList, b2ContactListener* listener)
{
	int contactCount = m_pWorld->m_contactManager.m_contactCount;
	b2clManifold *manifoldListData = b2CLCommonData::instance().manifoldListData;

	b2CLDevice::instance().finishCommandQueue();
    b2CLDevice::instance().copyArrayFromDevice(manifoldListData, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 
		0, sizeof(b2clManifold) * contactCount);

	// when is "m_contactList" created??? CHECK!!!
	b2Contact* c = m_contactList;
	int contact_index = 0, valid_contact_num = 0;
	bool touching;
	while (c)
	{
		touching = false;

		switch (manifoldListData[contact_index].type)
		{
		case 0:
			c->m_manifold.type = b2Manifold::e_circles;
			break;
		case 1:
			c->m_manifold.type = b2Manifold::e_faceA;
			break;
		case 2:
			c->m_manifold.type = b2Manifold::e_faceB;
			break;
		}
		c->m_manifold.pointCount = manifoldListData[contact_index].pointCount;
		c->m_manifold.localPoint.x = manifoldListData[contact_index].localPoint[0];
		c->m_manifold.localPoint.y = manifoldListData[contact_index].localPoint[1];
		c->m_manifold.localNormal.x = manifoldListData[contact_index].localNormal[0];
		c->m_manifold.localNormal.y = manifoldListData[contact_index].localNormal[1];
		for (int i=0; i<c->m_manifold.pointCount; i++)
		{
			c->m_manifold.points[i].id.key = manifoldListData[contact_index].points[i].id.key;
			c->m_manifold.points[i].localPoint.x = manifoldListData[contact_index].points[i].localPoint[0];
			c->m_manifold.points[i].localPoint.y = manifoldListData[contact_index].points[i].localPoint[1];
			c->m_manifold.points[i].normalImpulse = manifoldListData[contact_index].points[i].normalImpulse;
			c->m_manifold.points[i].tangentImpulse = manifoldListData[contact_index].points[i].tangentImpulse;
		}

		touching = c->m_manifold.pointCount>0;
		if (touching)
		{
			c->m_flags |= b2Contact::e_touchingFlag;
			valid_contact_num++;
		}
		else
		{
			c->m_flags &= ~b2Contact::e_touchingFlag;
		}

		c = c->GetNext();
		contact_index++;
	}
	assert(contactCount == contact_index);
}

void b2CLNarrowPhase::ReadbackGPUDataForListener(b2World *m_pWorld, b2Contact **m_contactList, b2ContactListener* listener, int *enableBitArray, int *temp)
{
#if defined(SOLVER_OPENCL)
	if (m_pWorld->m_contactManager.m_contactCount <=0 )
		return;

	int totalContactCount = m_pWorld->m_contactManager.m_contactCount;

	b2Contact* pc = *m_contactList;
	while (pc)
	{
		// set an unused bit for all contacts
		// clear the bit later when visited the contact
		// if the bit is still set after this function, it is not longer valid and should be removed
		pc->m_flags |= 0x40;
		pc = pc->GetNext();
	}
	
	b2Manifold oldManifold;

	if (totalContactCount>0)
	{
		cl_uint *manifoldBinaryBitList;
		if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
		{
			assert(enableBitArray); // if PreSolve is used, enableBitArray should not be NULL!
			manifoldBinaryBitList = new cl_uint[totalContactCount];
			b2CLDevice::instance().copyArrayFromDevice(manifoldBinaryBitList, b2CLCommonData::instance().manifoldBinaryBitListBuffer, 0, sizeof(cl_uint)*totalContactCount, true);
		}

		b2clManifold *manifoldListData = b2CLCommonData::instance().manifoldListData;
		b2CLDevice::instance().copyArrayFromDevice(manifoldListData, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 
			0, sizeof(b2clManifold) * totalContactCount);

		int *globalIndices = b2CLCommonData::instance().globalIndices;
		b2CLDevice::instance().copyArrayFromDevice(globalIndices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*totalContactCount*4, true);

		// Mimic b2ContactManager::AddPair (which is called in b2BroadPhase::UpdatePairs)
		// to insert contacts into m_contactList
		int fixtureIndexA, fixtureIndexB;
		for (int i=0; i<totalContactCount; i++)
		{
			if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
			{
				// only valid contacts can be enabled for Solver
				enableBitArray[i] = manifoldBinaryBitList[i];
			}

			int globalIndex = i; //validContactIndices[i];
			if (manifoldListData[globalIndex].pointCount==0)
			{
				// assert(0);
				// We cannot use validContactCount here. A contact with non-zero pointCount may be invalid, if one body is a sensor!
				continue;
			}

			fixtureIndexA = globalIndices[globalIndex*4];
			fixtureIndexB = globalIndices[globalIndex*4+1];

			b2Fixture* fixtureA = b2CLCommonData::instance().fixture_address[fixtureIndexA];
			b2Fixture* fixtureB = b2CLCommonData::instance().fixture_address[fixtureIndexB];

			int32 indexA = b2CLCommonData::instance().fixture_child[fixtureIndexA];
			int32 indexB = b2CLCommonData::instance().fixture_child[fixtureIndexB];

			b2Body* bodyA = fixtureA->GetBody();
			b2Body* bodyB = fixtureB->GetBody();

			// Are the fixtures on the same body?
			if (bodyA == bodyB)
			{
				assert(0); // should not happen at all
				continue;
			}

			b2ContactEdge* edge = bodyB->GetContactList();
			bool bFoundSameContact = false;
			while (edge)
			{
				if (edge->other == bodyA)
				{
					b2Fixture* fA = edge->contact->GetFixtureA();
					b2Fixture* fB = edge->contact->GetFixtureB();
					int32 iA = edge->contact->GetChildIndexA();
					int32 iB = edge->contact->GetChildIndexB();

					if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
					{
						// A contact already exists.
						bFoundSameContact = true;
						break;
					}

					if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
					{
						// A contact already exists.
						bFoundSameContact = true;
						break;
					}
				}

				edge = edge->next;
			}

			if (bFoundSameContact)
			{
				oldManifold = edge->contact->m_manifold;

				// update manifold information for exisiting contact
				b2Contact * c = edge->contact;
				c->m_manifold.pointCount = manifoldListData[i].pointCount;
				c->m_manifold.localPoint.x = manifoldListData[i].localPoint[0];
				c->m_manifold.localPoint.y = manifoldListData[i].localPoint[1];
				c->m_manifold.localNormal.x = manifoldListData[i].localNormal[0];
				c->m_manifold.localNormal.y = manifoldListData[i].localNormal[1];
				c->m_manifold.type = b2Manifold::Type(manifoldListData[i].type);
				for (int k=0; k<c->m_manifold.pointCount; k++)
				{
					c->m_manifold.points[k].id.key = manifoldListData[i].points[k].id.key;
					c->m_manifold.points[k].localPoint.x = manifoldListData[i].points[k].localPoint[0];
					c->m_manifold.points[k].localPoint.y = manifoldListData[i].points[k].localPoint[1];
					c->m_manifold.points[k].normalImpulse = manifoldListData[i].points[k].normalImpulse;
					c->m_manifold.points[k].tangentImpulse = manifoldListData[i].points[k].tangentImpulse;
				}

				c->m_flags &= ~0x40;
				c->m_flags |= b2Contact::e_touchingFlag;
				c->m_flags |= b2Contact::e_enabledFlag; // Re-enable this contact.

				if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
				{
					listener->PreSolve(c, &oldManifold);
					enableBitArray[i] &= (c->m_flags & b2Contact::e_enabledFlag)>0;
				}

				continue;
			}

			// Does a joint override collision? Is at least one body dynamic?
			if (bodyB->ShouldCollide(bodyA) == false)
			{
				assert(0);
				continue;
			}

			// Check user filtering.
			if (m_pWorld->m_contactManager.m_contactFilter && 
				m_pWorld->m_contactManager.m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
			{
				assert(0);
				continue;
			}
			// Call the factory.
			b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_pWorld->m_contactManager.m_allocator);
			if (c == NULL)
			{
				printf("Allocate memory error in b2CLNarrowPhase::ReadbackGPUDataForListener!!!\n");
				exit(0);
			}

			// Contact creation may swap fixtures.
			fixtureA = c->GetFixtureA();
			fixtureB = c->GetFixtureB();
			indexA = c->GetChildIndexA();
			indexB = c->GetChildIndexB();
			bodyA = fixtureA->GetBody();
			bodyB = fixtureB->GetBody();

			// Insert into the world.
			c->m_prev = NULL;
			c->m_next = *m_contactList;
			if (*m_contactList != NULL)
			{
				(*m_contactList)->m_prev = c;
			}
			*m_contactList = c;

			// Connect to island graph.

			// Connect to body A
			c->m_nodeA.contact = c;
			c->m_nodeA.other = bodyB;

			c->m_nodeA.prev = NULL;
			c->m_nodeA.next = bodyA->m_contactList;
			if (bodyA->m_contactList != NULL)
			{
				bodyA->m_contactList->prev = &c->m_nodeA;
			}
			bodyA->m_contactList = &c->m_nodeA;

			// Connect to body B
			c->m_nodeB.contact = c;
			c->m_nodeB.other = bodyA;

			c->m_nodeB.prev = NULL;
			c->m_nodeB.next = bodyB->m_contactList;
			if (bodyB->m_contactList != NULL)
			{
				bodyB->m_contactList->prev = &c->m_nodeB;
			}
			bodyB->m_contactList = &c->m_nodeB;

			oldManifold = c->m_manifold;

			// fill in manifold information
			c->m_manifold.pointCount = manifoldListData[i].pointCount;
			c->m_manifold.localPoint.x = manifoldListData[i].localPoint[0];
			c->m_manifold.localPoint.y = manifoldListData[i].localPoint[1];
			c->m_manifold.localNormal.x = manifoldListData[i].localNormal[0];
			c->m_manifold.localNormal.y = manifoldListData[i].localNormal[1];
			c->m_manifold.type = b2Manifold::Type(manifoldListData[i].type);
			for (int k=0; k<c->m_manifold.pointCount; k++)
			{
				c->m_manifold.points[k].id.key = manifoldListData[i].points[k].id.key;
				c->m_manifold.points[k].localPoint.x = manifoldListData[i].points[k].localPoint[0];
				c->m_manifold.points[k].localPoint.y = manifoldListData[i].points[k].localPoint[1];
				c->m_manifold.points[k].normalImpulse = manifoldListData[i].points[k].normalImpulse;
				c->m_manifold.points[k].tangentImpulse = manifoldListData[i].points[k].tangentImpulse;
			}

			c->m_flags &= ~0x40;
			c->m_flags |= b2Contact::e_touchingFlag;
			c->m_flags |= b2Contact::e_enabledFlag; // Re-enable this contact.

			// call BeginContact
			listener->BeginContact(c);

			if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
			{
				listener->PreSolve(c, &oldManifold);
				enableBitArray[i] &= (c->m_flags & b2Contact::e_enabledFlag)>0;
			}
		}

		if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
		{
			delete [] manifoldBinaryBitList;
		}
	}
#endif
}