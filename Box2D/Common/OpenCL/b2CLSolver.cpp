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
#include <Box2D/Common/OpenCL/b2CLSolver.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/OpenCL/b2CLScan.h>
#include <Box2D/Common/OpenCL/b2CLSort.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

////////////////////////////////////PROFILE INFO
float32 b2CLSolver::totalTimeForComputingColoring;
int b2CLSolver::totalCountForComputingColoring;
float32 b2CLSolver::totalTimeForSolvingVelocityConstraint;
int b2CLSolver::totalCountForSolvingVelocityConstraint;
float32 b2CLSolver::totalTimeForCPUGPUCommunication;
int b2CLSolver::totalCountForCPUGPUCommunication;
////////////////////////////////////////////////

b2CLSolver::b2CLSolver()
{
#if defined(SOLVER_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		printf("Initializing b2CLSolver...\n");
    
		int err;
    
		//load opencl programs from files
		char* source=0;
		size_t sourceLen=0;
    
	#if !defined(NARROWPHASE_OPENCL)
        
#ifdef linux
        source = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (_WIN32)
        source = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (__EMSCRIPTEN__)
        source = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl", NULL), "// My comment\n", &sourceLen);
#else
        source = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLSolveVelocityConstraint_Alone.cl", NULL), "// My comment\n", &sourceLen);
#endif
        
	#else
        
#ifdef linux
        source = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLSolveVelocityConstraint.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (_WIN32)
        source = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLSolveVelocityConstraint.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (__EMSCRIPTEN__)
        source = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLSolveVelocityConstraint.cl", NULL), "// My comment\n", &sourceLen);
#else
        source = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLSolveVelocityConstraint.cl", NULL), "// My comment\n", &sourceLen);
#endif
        
	#endif
		//printf("lqiu debug constraint solver source is \n %s \n",velocityConstraintSolverSource);
		if(source == NULL)
		{
			b2Log("Could not load program source, is path 'b2CLSolveVelocityConstraint.cl' correct?");
		}
    
		//create the compute program from source kernel code
		velocityConstraintSolverProgram=clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**) &source, NULL, &err);
		if (!velocityConstraintSolverProgram)
		{
			printf("Error: Failed to create compute program!\n");
			exit(1);
		}
    
		//build the program
		err=clBuildProgram(velocityConstraintSolverProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			size_t len;
			char buffer[20480];
        
			printf("Error: Failed to build program executable!\n");
			clGetProgramBuildInfo(velocityConstraintSolverProgram,b2CLDevice::instance().GetCurrentDevice(),CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
			printf("%s\n", buffer);
			exit(1);
		}
//#if defined (SPLIT_OPENCL)

		splitVelocityConstraintSolverKernel = clCreateKernel ( velocityConstraintSolverProgram, "SolveSplitImpulseVelocityConstraint", &err ) ; 
		if (!splitVelocityConstraintSolverKernel || err != CL_SUCCESS) {
			printf ("Error: Failed to create compute kernel! \n") ; 
			exit(1);
		}

		mergeVelocityConstraintSolverKernel = clCreateKernel ( velocityConstraintSolverProgram, "SolveMergeVelocityConstraint", &err ); 
		if (!mergeVelocityConstraintSolverKernel || err != CL_SUCCESS) {
			printf ("Error: Failed to create compute kernel! \n") ; 
			exit(1);
		}
		countContactNum4EachBodyKernel = clCreateKernel ( velocityConstraintSolverProgram, "CountContactNum4EachBodyConstraint", &err ); 
		if (!countContactNum4EachBodyKernel || err != CL_SUCCESS) {
			printf ("Error: Failed to create compute kernel! \n") ; 
			exit(1);
		}
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(countContactNum4EachBodyKernel, maxWorkGroupSizeForCountContactNum4EachBody ) ;
/*
		updateImpulseConstraintSolverKernel = clCreateKernel ( velocityConstraintSolverProgram, "SolveUpdateImpulseConstraint", &err ); 
		if (!mergeVelocityConstraintSolverKernel || err != CL_SUCCESS) {
			printf ("Error: Failed to create compute kernel! \n") ; 
			exit(1);
		}
*/
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(splitVelocityConstraintSolverKernel, maxWorkGroupSizeForVelocityConstraintSolver ) ; 
//#else
		//create the compute kernel
		velocityConstraintSolverKernel = clCreateKernel(velocityConstraintSolverProgram, "SolveVelocityConstraint", &err);
		//velocityConstraintSolverKernel = clCreateKernel(velocityConstraintSolverProgram, "SolveVelocityConstraintComplex", &err);
		if (!velocityConstraintSolverKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel SolveVelocityConstraint!\n");

			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(velocityConstraintSolverKernel, maxWorkGroupSizeForVelocityConstraintSolver);
//#endif

	    for ( int i = 1 ; i < 11 ; i ++ )
		{
			switch (i) 
			{
				case e_distanceJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveDistanceJointVelocityConstraint", &err);
					break;
				}

				case e_revoluteJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveRevoluteJointVelocityConstraint", &err);
					break ; 
				}

				case e_prismaticJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolvePrismaticJointVelocityConstraint", &err);
					break ; 
				}

				case e_gearJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveGearJointVelocityConstraint", &err);
					break ; 
				}
				case e_pulleyJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolvePulleyJointVelocityConstraint", &err);
					break ; 
				}
				case e_ropeJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveRopeJointVelocityConstraint", &err);
					break ; 
				}

				case e_wheelJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveWheelJointVelocityConstraint", &err);
					break ; 
				}

				case e_weldJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveWeldJointVelocityConstraint", &err);
					break ; 
				}

				case e_mouseJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveMouseJointVelocityConstraint", &err);
					break ; 
				}

				case e_frictionJoint:
				{
					jointVelocityConstraintSolverKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "SolveFrictionJointVelocityConstraint", &err);
					break ; 
				}
			}

		    if (! jointVelocityConstraintSolverKernel[i] || err != CL_SUCCESS)
			{
				printf("Error: Failed to create compute kernel SolveVelocityConstraint!\n");

				exit(1);
			}
			 
			b2CLDevice::instance().getMaximumKernelWorkGroupSize(jointVelocityConstraintSolverKernel[i], maxWorkGroupSizeForJointVelocityConstraintSolver[i]);
		}

		
    
	#if defined(NARROWPHASE_OPENCL)
		velocityConstraintInitializeKernel = clCreateKernel(velocityConstraintSolverProgram, "InitializeVelocityConstraint", &err);
		if (!velocityConstraintInitializeKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel InitializeVelocityConstraint!, %d \n", err );

			exit(1);
		}
      for ( int i = 1 ; i < 11 ; i ++ ) {  // i == 11 for all cases
			switch (i) {
				case e_distanceJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeDistanceJointVelocityConstraint", &err);
					break;
				}
				case e_revoluteJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeRevoluteJointVelocityConstraint", &err);
					break;
				}
				case e_prismaticJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializePrismaticJointVelocityConstraint", &err);
					break;
				}
				case e_gearJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeGearJointVelocityConstraint", &err);
					break;
				}
				case e_pulleyJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializePulleyJointVelocityConstraint", &err);
					break;
				}
				case e_ropeJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeRopeJointVelocityConstraint", &err);
					break;
				}
				case e_wheelJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeWheelJointVelocityConstraint", &err);
					break;
				}
				case e_weldJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeWeldJointVelocityConstraint", &err);
					break;
				}
				case e_mouseJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeMouseJointVelocityConstraint", &err);
					break;
				}
				case e_frictionJoint:
				{
					jointVelocityConstraintInitializeKernel[i] = clCreateKernel(velocityConstraintSolverProgram, "InitializeFrictionJointVelocityConstraint", &err);
					break;
				}
			}
		     if (! jointVelocityConstraintInitializeKernel[i] || err != CL_SUCCESS)
			{
				printf("Error: Failed to create compute kernel SolveVelocityConstraint!\n");

				exit(1);
			}
		}		

		velocityConstraintInitializeKernel_HasSplit = clCreateKernel(velocityConstraintSolverProgram, "InitializeVelocityConstraint_HasSplit", &err);
		if (!velocityConstraintInitializeKernel_HasSplit || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel InitializeVelocityConstraint!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(velocityConstraintInitializeKernel_HasSplit, maxWorkGroupSizeForVelocityConstraintInitialize);

    
		bodyStateInitializeKernel = clCreateKernel(velocityConstraintSolverProgram, "InitializeBodyState", &err);
		if (!bodyStateInitializeKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(bodyStateInitializeKernel, maxWorkGroupSizeForBodyStateInitializeKernel);
    
		bodyStateInitializeFirstFrameKernel = clCreateKernel(velocityConstraintSolverProgram, "InitializeBodyStateFirstFrame", &err);
		if (!bodyStateInitializeFirstFrameKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(bodyStateInitializeFirstFrameKernel, maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel);
    
		warmStartKernel = clCreateKernel(velocityConstraintSolverProgram, "WarmStart", &err);
		if (!warmStartKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(warmStartKernel, maxWorkGroupSizeForWarmStartKernel);
    
		warmStartWithColoringKernel = clCreateKernel(velocityConstraintSolverProgram, "WarmStartWithColoring", &err);
		if (!warmStartWithColoringKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(warmStartWithColoringKernel, maxWorkGroupSizeForWarmStartWithColoringKernel);
    
		//warmStartSplitKernel = clCreateKernel(velocityConstraintSolverProgram, "WarmStartSplitWithColoring", &err);
		warmStartSplitKernel = clCreateKernel(velocityConstraintSolverProgram, "WarmStartSplit", &err);
		if (!warmStartSplitKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		testWarmstartKernel = clCreateKernel(velocityConstraintSolverProgram, "WarmStartSplitWithColoring", &err);
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(warmStartSplitKernel, maxWorkGroupSizeForWarmStartSplitKernel);
        b2CLDevice::instance().getMaximumKernelWorkGroupSize(warmStartSplitKernel, maxWorkGroupSizeForWarmStartSplitKernel);

		synchronizeXfKernel = clCreateKernel(velocityConstraintSolverProgram, "SynchronizeXf", &err);
		if (!synchronizeXfKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(synchronizeXfKernel, maxWorkGroupSizeForSynchronizeXfKernel);

		readLastImpulsesKernel = clCreateKernel(velocityConstraintSolverProgram, "ReadLastImpulses", &err);
		if (!readLastImpulsesKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(readLastImpulsesKernel, maxWorkGroupSizeForReadLastImpulsesKernel);

		readLastImpulsesFirstFrameKernel = clCreateKernel(velocityConstraintSolverProgram, "ReadLastImpulsesFirstFrame", &err);
		if (!readLastImpulsesFirstFrameKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(readLastImpulsesFirstFrameKernel, maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel);

		storeImpulsesKernel = clCreateKernel(velocityConstraintSolverProgram, "StoreImpulses", &err);
		if (!storeImpulsesKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(storeImpulsesKernel, maxWorkGroupSizeForStoreImpulsesKernel);
	#endif
        
#ifdef linux
        source = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLSolvePositionConstraint.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (_WIN32)
        source = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLSolvePositionConstraint.cl", NULL), "// My comment\n", &sourceLen);
#elif defined (__EMSCRIPTEN__)
        source = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLSolvePositionConstraint.cl", NULL), "// My comment\n", &sourceLen);
#else
        source = b2clLoadProgSource(shrFindFilePath("../../../Box2D/Common/OpenCL/b2CLSolvePositionConstraint.cl", NULL), "// My comment\n", &sourceLen);
#endif
        
		if(source == NULL)
		{
			b2Log("Could not load program source, is path 'b2CLIntegratePositions.cl' correct?");
		}
    
		//create the compute program from source kernel code
		positionConstraintSolverProgram=clCreateProgramWithSource(b2CLDevice::instance().GetContext(),1,(const char**) &source,NULL,&err);
		if (!positionConstraintSolverProgram)
		{
			printf("Error: Failed to create initialize position constraint program!\n");
			exit(1);
		}
    
		//build the program
		err=clBuildProgram(positionConstraintSolverProgram,0,NULL,OPENCL_BUILD_PATH,NULL,NULL);
		if (err != CL_SUCCESS)
		{
			size_t len;
			char buffer[30000];
        
			printf("Error: Failed to build program executable!\n");
			clGetProgramBuildInfo(positionConstraintSolverProgram,b2CLDevice::instance().GetCurrentDevice(),CL_PROGRAM_BUILD_LOG, sizeof(buffer),buffer,&len);
			printf("%s\n", buffer);
			exit(1);
		}

		//create the compute kernel
		integratePositionKernel=clCreateKernel(positionConstraintSolverProgram,"IntegratePosition",&err);
		if (!integratePositionKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create integrate positions kernel!\n");
			exit(1);
		}
		positionConstraintSolverKernel=clCreateKernel(positionConstraintSolverProgram,"SolvePositionConstraint",&err);
		if (!positionConstraintSolverKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create solve position constraint kernel!\n");
			exit(1);
		}
		splitPositionConstraintSolverKernel=clCreateKernel(positionConstraintSolverProgram,"SolveSplitPositionConstraint",&err);
		if (!splitPositionConstraintSolverKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create solve position constraint kernel!\n");
			exit(1);
		}
		positionConstraintSolverKernel_MergeSplittedMass=clCreateKernel(positionConstraintSolverProgram,"SolvePositionConstraint_MergeSplittedMass",&err);
		if (!positionConstraintSolverKernel_MergeSplittedMass || err != CL_SUCCESS)
		{
			printf("Error: Failed to create solve position constraint kernel!\n");
			exit(1);
		}
		for ( int i = 1 ; i < 11 ; i ++ ) // i == 11 for all cases
		{
			switch (i) 
			{
				case e_distanceJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveDistanceJointPositionConstraint", &err);
					break;
				}

				case e_revoluteJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveRevoluteJointPositionConstraint", &err);
					break ; 
				}

				case e_prismaticJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolvePrismaticJointPositionConstraint", &err);
					break ; 
				}

				case e_gearJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveGearJointPositionConstraint", &err);
					break ; 
				}
				case e_pulleyJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolvePulleyJointPositionConstraint", &err);
					break ; 
				}
				case e_ropeJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveRopeJointPositionConstraint", &err);
					break ; 
				}

				case e_wheelJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveWheelJointPositionConstraint", &err);
					break ; 
				}

				case e_weldJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveWeldJointPositionConstraint", &err);
					break ; 
				}

				case e_mouseJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveMouseJointPositionConstraint", &err);
					break ; 
				}

				case e_frictionJoint:
				{
					jointPositionConstraintSolverKernel[i] = clCreateKernel(positionConstraintSolverProgram, "SolveFrictionJointPositionConstraint", &err);
					break ; 
				}
			}

		    if (! jointPositionConstraintSolverKernel[i] || err != CL_SUCCESS)
			{
				printf("Error: Failed to create compute kernel SolvePositionConstraint!\n");

				exit(1);
			}
			 
			b2CLDevice::instance().getMaximumKernelWorkGroupSize(jointPositionConstraintSolverKernel[i], maxWorkGroupSizeForJointPositionConstraintSolver[i]);
		}
		collectStaticDynamicPairKernel = clCreateKernel(positionConstraintSolverProgram,"CollectStaticDynamicPairKernel",&err);
		if (!collectStaticDynamicPairKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create Collect Static Dynamic Pair kernel!\n");
			exit(1);
		}	
		initSDBodyKernel  = clCreateKernel(positionConstraintSolverProgram,"InitStaticDynamicBodyKernel",&err);
		if (!initSDBodyKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to initSDBodyKernel !\n");
			exit(1);
		}
		solveSDVelocityKernel  = clCreateKernel(positionConstraintSolverProgram,"SolveSDVelocity",&err);
		if (!solveSDVelocityKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create SolveSDVelocityKernel kernel!\n");
			exit(1);
		}	
		solveSDPositionKernel  = clCreateKernel(positionConstraintSolverProgram,"SolveSDPosition",&err);
		if (!solveSDPositionKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create SolveSDPositionKernel kernel!\n");
			exit(1);
		}

		syncSDBodyKernel  = clCreateKernel(positionConstraintSolverProgram,"syncSDBody",&err);
		if (!syncSDBodyKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create syncSDBodyKernel kernel!\n");
			exit(1);
		}

		testKernel=clCreateKernel(positionConstraintSolverProgram,"TestKernel",&err);
		if (!testKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create solve position constraint kernel!\n");
			exit(1);
		}
    
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(integratePositionKernel, maxWorkGroupSizeForIntegratingPositions);
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(positionConstraintSolverKernel, maxWorkGroupSizeForPositionConstraintSolver);
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(testKernel, maxWorkGroupSizeForTestKernel);
    
		old_body_num = old_contact_num = 0;
		old_valid_contact_num = old_global_contact_num = 0;
		oldSortCount = 0;
		//islandManifoldIndicesData = NULL;
		//islandManifoldIndicesBuffer = NULL;

		lastContactCount = 0;
    
		clb2ContactsList = NULL; 
		clb2PointsList = NULL;
		clb2ImpulsesList = NULL;
		clb2ManifoldsList = NULL;
		contactIndexToColoredContactIndexMap = NULL;
		coloredContactIndexToContactIndexMap = NULL;
		colorOffsets = NULL;
		colorLengths = NULL;
		validContactIndices = NULL;
		globalIndices = NULL;
    
		clb2VelocitiesBuffer =  clb2PositionsBuffer =

		clb2ContactsBuffer = clb2PointsBuffer = clb2ImpulsesBuffer = clb2ManifoldsBuffer =
		manifoldKeysBuffer = globalIndicesBuffer = 
		coloredContactIndexToContactIndexMapBuffer = NULL;

		clb2SplitVelocitiesBuffer = clb2OldVelocitiesBuffer =clb2NumContactEachBody = clb2Contact2BodySplitVelocitiesIndex = clb2OldImpulsesBuffer = NULL ; 

		sdIndexBufferSize = this->sdIndexNum  = 0 ; 	sdManifoldDataBuffer = sdBodyDataBuffer = sdContactDataBuffer= sdManifoldBuffer = sdContactBuffer=  sdPointBuffer = sdImpulseBuffer =  NULL; 

		// for debug
		clb2TestPositionsBuffer = NULL;
		clb2TestVelocitiesBuffer = NULL ; 
	}
#endif
	colorTime = 0 ; 
}

b2CLSolver::~b2CLSolver()
{
#if defined(SOLVER_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		//if(islandManifoldIndicesData) 
		//    delete [] islandManifoldIndicesData;
		if(clb2ContactsList)
			delete [] clb2ContactsList;
		if(clb2PointsList)
			delete [] clb2PointsList;
		if(clb2ImpulsesList)
			delete [] clb2ImpulsesList;
		if(clb2ManifoldsList)
			delete [] clb2ManifoldsList;
		if(contactIndexToColoredContactIndexMap)
			delete [] contactIndexToColoredContactIndexMap;
		if(coloredContactIndexToContactIndexMap)
			delete [] coloredContactIndexToContactIndexMap;
		if(colorOffsets)
			delete [] colorOffsets;
		if(colorLengths)
			delete [] colorLengths;
    
		//if(islandManifoldIndicesBuffer)
		//    b2CLDevice::instance().freeArray(islandManifoldIndicesBuffer);
		if(clb2VelocitiesBuffer)
			b2CLDevice::instance().freeArray(clb2VelocitiesBuffer);
		if(clb2PositionsBuffer)
			b2CLDevice::instance().freeArray(clb2PositionsBuffer);

		// for debug
		if(clb2TestPositionsBuffer)
			b2CLDevice::instance().freeArray(clb2TestPositionsBuffer);

		if(clb2ContactsBuffer)
			b2CLDevice::instance().freeArray(clb2ContactsBuffer);
		if(clb2PointsBuffer)
			b2CLDevice::instance().freeArray(clb2PointsBuffer);
		if(clb2ImpulsesBuffer)
			b2CLDevice::instance().freeArray(clb2ImpulsesBuffer);
		if(clb2ManifoldsBuffer)
			b2CLDevice::instance().freeArray(clb2ManifoldsBuffer);
		if(coloredContactIndexToContactIndexMapBuffer)
			b2CLDevice::instance().freeArray(coloredContactIndexToContactIndexMapBuffer);
		// Split
		if (clb2NumContactEachBody)
			b2CLDevice::instance().freeArray(clb2NumContactEachBody);
		if (clb2Contact2BodySplitVelocitiesIndex)
			b2CLDevice::instance().freeArray(clb2Contact2BodySplitVelocitiesIndex);
		if (clb2SplitVelocitiesBuffer)
			b2CLDevice::instance().freeArray(clb2SplitVelocitiesBuffer);
		if (clb2OldVelocitiesBuffer)
			b2CLDevice::instance().freeArray(clb2OldVelocitiesBuffer);
		if (clb2OldImpulsesBuffer)
			b2CLDevice::instance().freeArray(clb2OldImpulsesBuffer);
		if (sdManifoldDataBuffer != NULL )
			b2CLDevice::instance().freeArray(sdManifoldDataBuffer);
		if (sdBodyDataBuffer != NULL )
			b2CLDevice::instance().freeArray(sdBodyDataBuffer);
		if (sdManifoldBuffer != NULL)
			b2CLDevice::instance().freeArray(sdManifoldBuffer);
		if (sdContactBuffer != NULL)
			b2CLDevice::instance().freeArray(sdContactBuffer);
		if (sdContactDataBuffer != NULL )
			b2CLDevice::instance().freeArray(sdContactDataBuffer);
		if (sdPointBuffer != NULL)
			b2CLDevice::instance().freeArray(sdPointBuffer);
		if (sdImpulseBuffer != NULL)
			b2CLDevice::instance().freeArray(sdImpulseBuffer); 
	} 
#endif
}

void b2CLSolver::SetValues(int bodyCount, int contactCount, b2Vec2 gravity, float32 m_dtInput, float32 dtRatio)
{
	m_bodyCount = bodyCount;
	m_contactCount = contactCount;
	m_gravity[0] = gravity.x;
	m_gravity[1] = gravity.y;
	m_dt = m_dtInput;
	m_dtRatio = dtRatio;

}

void b2CLSolver::StoreImpulses()
{
    for (int32 i = 0; i < m_contactCount; ++i)
	{
#if defined(NARROWPHASE_OPENCL)
		int globalIndex = validContactIndices[i];
		b2Manifold* manifold = m_contacts[globalIndex]->GetManifold();
#else
		b2Manifold* manifold = m_contacts[i]->GetManifold();
#endif
        
        manifold->points[0].normalImpulse = clb2ImpulsesList[contactIndexToColoredContactIndexMap[i]].normalImpulse1;
        manifold->points[0].tangentImpulse = clb2ImpulsesList[contactIndexToColoredContactIndexMap[i]].tangentImpulse1;
        if (manifold->pointCount > 1) 
        {
            manifold->points[1].normalImpulse = clb2ImpulsesList[contactIndexToColoredContactIndexMap[i]].normalImpulse2;
            manifold->points[1].tangentImpulse = clb2ImpulsesList[contactIndexToColoredContactIndexMap[i]].tangentImpulse2;
        }
	}
}

void b2CLSolver::b2CLInitializeSolver(b2Body** m_bodiesInput, b2Contact** m_contactsInput,
                                      b2Velocity* m_velocitiesInput, b2Position* m_positionsInput,
                                      int32 m_bodyCountInput, int32 m_contactCountInput, b2Vec2 m_gravityInput, float32 m_dtInput, 
                                      const b2ContactManager& contactManager, b2Contact** m_islandContacts,
                                      b2ContactVelocityConstraint* m_velocitiesConstraintsInput, b2ContactPositionConstraint* m_positionsConstraintsInput)
{
    //SetValues(m_bodyCountInput, m_contactCountInput, m_gravityInput, m_dtInput);
    
    if (m_bodyCount<=0) return;
    
	m_positions=m_positionsInput;
    m_velocities=m_velocitiesInput;
    m_bodies = m_bodiesInput;
    
    CreateBuffersForBodies();
#if !defined(NARROWPHASE_OPENCL)
    CopyDataToGPUForBodies();
#endif
    
    if(m_contactCount<=0) return;
    
#if defined(NARROWPHASE_OPENCL)
	//b2Contact* c = contactManager.m_contactList;
	//int contact_index = 0;
	//while (c)
	//{
	//	c->m_uid = contact_index;
	//	c = c->GetNext();
	//	contact_index++;
	//}
 //   
	//for (int32 i = 0; i < m_contactCount; ++i)
	//{
	//	c = m_islandContacts[i];
	//	islandManifoldIndicesData[i] = c->m_uid;
	//}
 //   
	//// Copy data from CPU to GPU
 //   b2CLDevice::instance().copyArrayToDevice(islandManifoldIndicesBuffer, islandManifoldIndicesData, 0, sizeof(int32) * m_contactCount);
#endif
    
	m_contacts = m_contactsInput;
    m_velocitiesConstraints = m_velocitiesConstraintsInput;
    m_positionsConstraints = m_positionsConstraintsInput;
    
    CreateBuffersForContacts();
    
#if defined(NARROWPHASE_OPENCL)
	#if defined(_DEBUG_TIME_SOLVER)
	    b2Timer testColorTimer;
	#endif
#if defined (SPLIT_OPENCL)
  memcpy (coloredContactIndexToContactIndexMap, validContactIndices, sizeof(int)*m_contactCount) ; 
#else
 ComputeColoringOfContacts(m_islandContacts);
#endif

//	#if defined(_DEBUG_TIME_SOLVER)
//		this->colorTime += testColorTimer.GetMilliseconds();
//	#endif

#else
	ComputeColoringOfContacts();
#endif
    
   
#if !defined(NARROWPHASE_OPENCL) && defined(SOLVER_OPENCL)
    for (int32 i=0; i<m_contactCount; i++) 
	{
        int coloredContactIndex = contactIndexToColoredContactIndexMap[i]; 
        
        clb2ContactsList[coloredContactIndex].indexA = m_velocitiesConstraintsInput[i].indexA;
        clb2ContactsList[coloredContactIndex].indexB = m_velocitiesConstraintsInput[i].indexB;
        clb2ContactsList[coloredContactIndex].normal[0] = m_velocitiesConstraintsInput[i].normal.x;
        clb2ContactsList[coloredContactIndex].normal[1] = m_velocitiesConstraintsInput[i].normal.y;
        clb2ContactsList[coloredContactIndex].friction = m_velocitiesConstraintsInput[i].friction;
        clb2ContactsList[coloredContactIndex].invMassA = m_velocitiesConstraintsInput[i].invMassA;
        clb2ContactsList[coloredContactIndex].invMassB = m_velocitiesConstraintsInput[i].invMassB;
        clb2ContactsList[coloredContactIndex].invIA = m_velocitiesConstraintsInput[i].invIA;
        clb2ContactsList[coloredContactIndex].invIB = m_velocitiesConstraintsInput[i].invIB;
        clb2PointsList[coloredContactIndex].rA1[0] = m_velocitiesConstraintsInput[i].points[0].rA.x;
        clb2PointsList[coloredContactIndex].rA1[1] = m_velocitiesConstraintsInput[i].points[0].rA.y;
        clb2PointsList[coloredContactIndex].rB1[0] = m_velocitiesConstraintsInput[i].points[0].rB.x;
        clb2PointsList[coloredContactIndex].rB1[1] = m_velocitiesConstraintsInput[i].points[0].rB.y;
        clb2PointsList[coloredContactIndex].normalMass1 = m_velocitiesConstraintsInput[i].points[0].normalMass;
        clb2PointsList[coloredContactIndex].tangentMass1 = m_velocitiesConstraintsInput[i].points[0].tangentMass;
        clb2PointsList[coloredContactIndex].velocityBias1 = m_velocitiesConstraintsInput[i].points[0].velocityBias;
        
        clb2ImpulsesList[coloredContactIndex].normalImpulse1 = m_velocitiesConstraintsInput[i].points[0].normalImpulse;
        clb2ImpulsesList[coloredContactIndex].tangentImpulse1 = m_velocitiesConstraintsInput[i].points[0].tangentImpulse;
        
        bool isOnePointContact = (m_velocitiesConstraintsInput[i].pointCount == 1);
        
        clb2PointsList[coloredContactIndex].rA2[0] = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].rA.x;
        clb2PointsList[coloredContactIndex].rA2[1] = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].rA.y;
        clb2PointsList[coloredContactIndex].rB2[0] = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].rB.x;
        clb2PointsList[coloredContactIndex].rB2[1] = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].rB.y;
        clb2PointsList[coloredContactIndex].normalMass2 = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].normalMass;
        clb2PointsList[coloredContactIndex].tangentMass2 = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].tangentMass;
        clb2PointsList[coloredContactIndex].velocityBias2 = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].velocityBias;
        
        clb2ImpulsesList[coloredContactIndex].normalImpulse2 = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].normalImpulse;
        clb2ImpulsesList[coloredContactIndex].tangentImpulse2 = isOnePointContact ? -1 : m_velocitiesConstraintsInput[i].points[1].tangentImpulse;  
        
        clb2ManifoldsList[coloredContactIndex].localNormal=m_positionsConstraintsInput[i].localNormal;
        clb2ManifoldsList[coloredContactIndex].localPoint=m_positionsConstraintsInput[i].localPoint;
        clb2ManifoldsList[coloredContactIndex].localPoints1=m_positionsConstraintsInput[i].localPoints[0];
        if (m_positionsConstraintsInput[i].pointCount>=2) {
            clb2ManifoldsList[coloredContactIndex].localPoints2=m_positionsConstraintsInput[i].localPoints[1];
        }
        clb2ManifoldsList[coloredContactIndex].pointCount=m_positionsConstraintsInput[i].pointCount;
        clb2ManifoldsList[coloredContactIndex].type=m_positionsConstraintsInput[i].type;
        clb2ManifoldsList[coloredContactIndex].radiusA=m_positionsConstraintsInput[i].radiusA;
        clb2ManifoldsList[coloredContactIndex].radiusB=m_positionsConstraintsInput[i].radiusB;
        clb2ManifoldsList[coloredContactIndex].localCenterA=m_positionsConstraintsInput[i].localCenterA;
        clb2ManifoldsList[coloredContactIndex].localCenterB=m_positionsConstraintsInput[i].localCenterB;
    }    
#endif
    
    CopyDataToGPUForContacts();
}

void b2CLSolver::ComputeColoringOfContacts(b2Contact** m_islandContacts)
{
    bool* freezed_body = new bool[m_bodyCount];
    int* contactColors = new int[m_contactCount];
    memset(freezed_body, 0, m_bodyCount*sizeof(bool));
    memset(contactColors, 0, m_contactCount*sizeof(int));
    
    unsigned int color = 1;unsigned int coloredContactIndex = 0;
    colorOffsets[0] = 0;

	//bool exchange[2] = {false, false}; 
   // printf ("exchange in coloring"); 
    while (coloredContactIndex < m_contactCount) {
        
        for (int32 i = 0; i < m_contactCount; i++) {
            if (contactColors[i]) {
                continue;
            }
		#if defined(NARROWPHASE_OPENCL)
			int globalIndex = validContactIndices[i];
			#if defined(BROADPHASE_OPENCL)
				int32 indexA = globalIndices[globalIndex*4+2];
				int32 indexB = globalIndices[globalIndex*4+3];
			#else
				b2Contact* contact = m_islandContacts[globalIndex];

				b2Fixture* fixtureA = contact->m_fixtureA;
				b2Fixture* fixtureB = contact->m_fixtureB;
				b2Body* bodyA = fixtureA->GetBody();
				b2Body* bodyB = fixtureB->GetBody();

				int32 indexA = bodyA->m_islandIndex;
				int32 indexB = bodyB->m_islandIndex;
              //  if (i == 0 && indexA == 2 && indexB == 1) exchange[0] = true ; 
			//	if (i == 1 && indexA == 1 && indexB == 0) exchange[1] = true ; 
			#endif
        #else
            int32 indexA = m_contacts[i]->m_fixtureA->GetBody()->m_islandIndex;
            int32 indexB = m_contacts[i]->m_fixtureB->GetBody()->m_islandIndex;
		#endif
            if (freezed_body[indexA] || freezed_body[indexB]){
                continue;
            }
            
            contactColors[i] = color;
            contactIndexToColoredContactIndexMap[i] = coloredContactIndex;
		#if defined(NARROWPHASE_OPENCL)
            coloredContactIndexToContactIndexMap[coloredContactIndex] = globalIndex;
		#else
            coloredContactIndexToContactIndexMap[coloredContactIndex] = i;
		#endif
            coloredContactIndex++;
            
            if(m_bodies[indexA]->GetType() == b2_dynamicBody){
                freezed_body[indexA] = true;
            }
            if(m_bodies[indexB]->GetType() == b2_dynamicBody){
                freezed_body[indexB] = true;
            }
        }
        color++;
        memset(freezed_body, 0, m_bodyCount*sizeof(bool));
        
        colorOffsets[color-1] = coloredContactIndex;
    }
    max_color = color-1;
    
    colorOffsets[max_color] = m_contactCount;
    for (int i = 1; i<= max_color; i++) {
        colorLengths[i-1] = colorOffsets[i]-colorOffsets[i-1];
    } 
    delete [] contactColors;
    delete [] freezed_body;
}

void b2CLSolver::CreateBuffersForBodies()
{
	if (old_body_num < m_bodyCount)
	{
		if (clb2VelocitiesBuffer)
			b2CLDevice::instance().freeArray(clb2VelocitiesBuffer);
		clb2VelocitiesBuffer = b2CLDevice::instance().allocateArray(sizeof(float32) * 3 * m_bodyCount);
		if (clb2PositionsBuffer)
			b2CLDevice::instance().freeArray(clb2PositionsBuffer);
		clb2PositionsBuffer = b2CLDevice::instance().allocateArray(sizeof(float32) * 3 * m_bodyCount);
        
		// for debug
		if (clb2TestPositionsBuffer)
			b2CLDevice::instance().freeArray(clb2TestPositionsBuffer);
		clb2TestPositionsBuffer = b2CLDevice::instance().allocateArray(sizeof(float32) * 3 * m_bodyCount);

		old_body_num = m_bodyCount;
	}
}

void b2CLSolver::CreateBuffersForContacts()
{
    if (old_contact_num < m_contactCount)
	{
		if (clb2ContactsList)
			delete [] clb2ContactsList;
		clb2ContactsList = new clb2ContactNode[m_contactCount];
		if (clb2PointsList)
			delete [] clb2PointsList;
		clb2PointsList = new clb2PointsNode[m_contactCount];
		if (clb2ImpulsesList)
			delete [] clb2ImpulsesList;
		clb2ImpulsesList = new clb2ImpulseNode[m_contactCount];
        if (clb2ManifoldsList)
            delete [] clb2ManifoldsList;
        clb2ManifoldsList = new clb2Manifold[m_contactCount];
		if (contactIndexToColoredContactIndexMap)
			delete [] contactIndexToColoredContactIndexMap;
		contactIndexToColoredContactIndexMap = new int[m_contactCount];
		if (coloredContactIndexToContactIndexMap)
			delete [] coloredContactIndexToContactIndexMap;
		coloredContactIndexToContactIndexMap = new int[m_contactCount];
		if (colorOffsets)
			delete [] colorOffsets;
		colorOffsets = new unsigned int[m_contactCount+1];
		if (colorLengths)
			delete [] colorLengths;
		colorLengths = new unsigned int[m_contactCount];
        
		if (clb2ContactsBuffer)
			b2CLDevice::instance().freeArray(clb2ContactsBuffer);
		clb2ContactsBuffer = b2CLDevice::instance().allocateArray(sizeof(struct clb2ContactNode) * m_contactCount);
		if (clb2PointsBuffer)
			b2CLDevice::instance().freeArray(clb2PointsBuffer);
		clb2PointsBuffer = b2CLDevice::instance().allocateArray(sizeof(struct clb2PointsNode) * m_contactCount);
        if (clb2ManifoldsBuffer)
            b2CLDevice::instance().freeArray(clb2ManifoldsBuffer);
        clb2ManifoldsBuffer=b2CLDevice::instance().allocateArray(sizeof(struct clb2Manifold) * m_contactCount);
		if (clb2ImpulsesBuffer)
			b2CLDevice::instance().freeArray(clb2ImpulsesBuffer);
		clb2ImpulsesBuffer = b2CLDevice::instance().allocateArray(sizeof(struct clb2ImpulseNode) * m_contactCount);

#if defined(NARROWPHASE_OPENCL)
		if (coloredContactIndexToContactIndexMapBuffer)
			b2CLDevice::instance().freeArray(coloredContactIndexToContactIndexMapBuffer);
		coloredContactIndexToContactIndexMapBuffer = b2CLDevice::instance().allocateArray(sizeof(int32) * m_contactCount);

		//if (islandManifoldIndicesData)
		//	delete [] islandManifoldIndicesData;
		//islandManifoldIndicesData = new int32[m_contactCount];
		//if (islandManifoldIndicesBuffer)
		//	b2CLDevice::instance().freeArray(islandManifoldIndicesBuffer);
		//islandManifoldIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int32) * m_contactCount);
#endif
		old_contact_num = m_contactCount;
	}
}

void b2CLSolver::CreateBuffersForSortingManifolds()
{
	if (m_contactCount<512)
		sortCount = 512;
	else
	{
		// compute the least power-of-2 which >= m_contactCount
		int exp;
		frexp((float)m_contactCount, &exp);
		sortCount = 1 << (exp-1);
		if (sortCount<m_contactCount)
			sortCount <<= 1;
	}

	if (oldSortCount<sortCount)
	{
        if (manifoldKeysBuffer)
            b2CLDevice::instance().freeArray(manifoldKeysBuffer);
        manifoldKeysBuffer=b2CLDevice::instance().allocateArray(sizeof(unsigned int) * sortCount);
        if (globalIndicesBuffer)
            b2CLDevice::instance().freeArray(globalIndicesBuffer);
        globalIndicesBuffer=b2CLDevice::instance().allocateArray(sizeof(unsigned int) * sortCount);

		oldSortCount = sortCount;
	}

#ifndef USE_CPU_SORT
	unsigned int *zeroBuffer = new unsigned int[sortCount];
	memset(zeroBuffer, 0, sizeof(unsigned int)*sortCount);
	b2CLDevice::instance().copyArrayToDevice(manifoldKeysBuffer, zeroBuffer, 0, sizeof(unsigned int)*sortCount, true);
	delete [] zeroBuffer;
#endif
}

void b2CLSolver::CopyDataToGPUForBodies()
{
    b2CLDevice::instance().copyArrayToDevice(clb2VelocitiesBuffer, m_velocities, 0, sizeof(float32)*3*m_bodyCount,true);
    b2CLDevice::instance().copyArrayToDevice(clb2PositionsBuffer, m_positions, 0, sizeof(float32)*3*m_bodyCount,true);
}

void b2CLSolver::CopyDataToGPUForContacts()
{
#if !defined(NARROWPHASE_OPENCL)
    b2CLDevice::instance().copyArrayToDevice(clb2ContactsBuffer, clb2ContactsList, 0, sizeof(struct clb2ContactNode)*m_contactCount, true);
    b2CLDevice::instance().copyArrayToDevice(clb2PointsBuffer, clb2PointsList, 0, sizeof(struct clb2PointsNode)*m_contactCount, true);
    b2CLDevice::instance().copyArrayToDevice(clb2ImpulsesBuffer, clb2ImpulsesList, 0, sizeof(struct clb2ImpulseNode)*m_contactCount, true);
    b2CLDevice::instance().copyArrayToDevice(clb2ManifoldsBuffer, clb2ManifoldsList, 0, sizeof(struct clb2Manifold)*m_contactCount,true);
#endif
#if defined(NARROWPHASE_OPENCL)
    b2CLDevice::instance().copyArrayToDevice(coloredContactIndexToContactIndexMapBuffer, coloredContactIndexToContactIndexMap, 0, sizeof(int32)*m_contactCount, true);
#endif
}

void b2CLSolver::CopyResultsFromGPUForBodies()
{
    if(m_bodyCount<=0) return;
    
    b2CLDevice::instance().copyArrayFromDevice(m_positions, clb2PositionsBuffer, 0, sizeof(float32)*3*m_bodyCount,true);
    b2CLDevice::instance().copyArrayFromDevice(m_velocities, clb2VelocitiesBuffer, 0, sizeof(float32)*3*m_bodyCount, true);
}

void b2CLSolver::CopyResultsFromGPUForContacts()
{
    if(m_contactCount<=0) return;
    
    b2CLDevice::instance().copyArrayFromDevice(clb2ImpulsesList, clb2ImpulsesBuffer, 0, sizeof(struct clb2ImpulseNode)*m_contactCount, true);
}

void b2CLSolver::b2CLInitializeVelocityConstraint_HasSplit(bool bWarmStarting, float dtRatio)
{
    //if(m_contactCount<=0) return;


	// Copy from Initialize SplitBufferParallel
		int maxContactNumPerBody = 20 ;
		if (!clb2SplitVelocitiesBuffer) {
			if (clb2SplitVelocitiesBuffer) b2CLDevice::instance().freeArray(clb2SplitVelocitiesBuffer);
			clb2SplitVelocitiesBuffer = b2CLDevice::instance().allocateArray ( sizeof(float) * 3 * m_bodyCount * maxContactNumPerBody ) ; 
			
			if (!clb2HasSplitVelocitiesContactsBuffer) b2CLDevice::instance().freeArray(clb2HasSplitVelocitiesContactsBuffer);
			unsigned int* allZeroArray = new unsigned int [m_bodyCount*maxContactNumPerBody];
			memset ( allZeroArray, 0 , sizeof (unsigned int) * m_bodyCount*maxContactNumPerBody ) ; 
			clb2HasSplitVelocitiesContactsBuffer = b2CLDevice::instance().allocateArray ( sizeof (unsigned int) * m_bodyCount * maxContactNumPerBody ) ; 
			b2CLDevice::instance().copyArrayToDevice(clb2HasSplitVelocitiesContactsBuffer, allZeroArray, 0, sizeof(unsigned int)* m_bodyCount * maxContactNumPerBody, true);
			delete [] allZeroArray ;
			 
			
			if (clb2NumContactEachBody) b2CLDevice::instance().freeArray(clb2NumContactEachBody);
	        clb2NumContactEachBody = b2CLDevice::instance().allocateArray (sizeof(unsigned int) * m_bodyCount);
		}
		if (m_contactCount == 0 ) return ; 


		int numElement = m_contactCount ; 
		if (clb2Contact2BodySplitVelocitiesIndex) b2CLDevice::instance().freeArray(clb2Contact2BodySplitVelocitiesIndex);
		clb2Contact2BodySplitVelocitiesIndex = b2CLDevice::instance().allocateArray ( sizeof(unsigned int)*m_contactCount * 2 ) ;    
		this->InitializeSplitBufferParallel ( true ) ; 


       
    // set arguments of kernel
    int a=0;
    int err;

    err=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&clb2ContactsBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&clb2PointsBuffer);
    err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++, sizeof(cl_mem), &clb2ManifoldsBuffer);
    err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(const unsigned int),&m_contactCount);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().fixtureStaticListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&coloredContactIndexToContactIndexMapBuffer);
	//err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&validContactIndices);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().shapeListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(int),&bWarmStarting);
	err|=clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,a++,sizeof(float),&dtRatio);
	err |= clSetKernelArg (velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&this->clb2NumContactEachBody);
	err |= clSetKernelArg (velocityConstraintInitializeKernel_HasSplit,a++,sizeof(cl_mem),&this->clb2Contact2BodySplitVelocitiesIndex);
	//err |= clSetKernelArg(velocityConstraintInitializeKernel_HasSplit,  a++, sizeof(unsigned int), &numBody);

    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
	//// for debug
	//clb2PointsNode* points1 = new clb2PointsNode[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(points1, clb2PointsBuffer, 0, sizeof(clb2PointsNode)*m_contactCount, true);
    
	//execute the kernel
    int numBlocks = (numElement + maxWorkGroupSizeForVelocityConstraintInitialize-1)/maxWorkGroupSizeForVelocityConstraintInitialize;
    size_t global = numBlocks * maxWorkGroupSizeForVelocityConstraintInitialize;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), velocityConstraintInitializeKernel_HasSplit, 1, NULL, 
                               &global, &maxWorkGroupSizeForVelocityConstraintInitialize, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute velocity constraint solver kernel!\n");
        exit(1);
    }
	//printf (""); 
    

}


void b2CLSolver::b2CLInitializeJointVelocityConstraint(bool bWarmStarting, float dt)
{
	cl_event events[2]; int curr=0, last=1; int a=0;   bool noqueue = true ; 
	for ( int i = 1 ; i < 11 ; i ++ ) {
		int numContact = b2CLCommonData::instance().numJoints[i] ;  
		//To do: update the number of numContact. 
		if (numContact == 0) continue ; 
		int a = 0 ; int err; 
		err =clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&clb2PositionsBuffer);
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer) );
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer) );
		err |= clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&(b2CLCommonData::instance().jointListBuffer) );
		//err |= clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(int),& numContact );
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(int),&bWarmStarting);
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(float),&dt);
		err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(float),&m_dtRatio);
		//err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a++,sizeof(cl_mem),&testBuffer);

		int maxcolor = b2CLCommonData::instance().jointMaxColor[i];
		for(int colorIndex=0; colorIndex < maxcolor; colorIndex++) 
		{    
			unsigned int offset = b2CLCommonData::instance().jointColorOffsets[i][colorIndex];
			unsigned int length = b2CLCommonData::instance().jointColorOffsets[i][colorIndex+1] - offset; 	

			cl_event* curEvent = &events[curr];
			if (!bWarmStarting)
			{
						length = numContact;
						curEvent = NULL;
			}

			err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a,sizeof(const unsigned int),&offset);
			err|=clSetKernelArg(jointVelocityConstraintInitializeKernel[i],a+1,sizeof(const unsigned int),&length);
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
				exit(1);
			}

					//execute the kernel
			int numBlocks=(length+maxWorkGroupSizeForVelocityConstraintInitialize-1)/maxWorkGroupSizeForVelocityConstraintInitialize;
			size_t global=numBlocks*maxWorkGroupSizeForVelocityConstraintInitialize;

			cl_uint num_events_in_wait_list = (noqueue) ? 0 : 1;
			const cl_event* event_wait_list = (noqueue) ? NULL : &events[last];

			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), jointVelocityConstraintInitializeKernel[i], 1, NULL, 
						&global, &maxWorkGroupSizeForVelocityConstraintInitialize, num_events_in_wait_list, event_wait_list, curEvent);
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute jointVelocityConstraintInitializeKernel[%d]!\n", i);
				exit(1);
			}	

			if (!bWarmStarting)
				break;

			if (!noqueue)
				clReleaseEvent(events[last]);
			noqueue = false; 

			curr = last;
			last = 1-last;
					
		}// end of color index			 

		
	} // end of for (i)
	if (noqueue == false )
    	clReleaseEvent(events[last]);
}



void b2CLSolver::b2CLInitializeVelocityConstraint(bool bWarmStarting, float dtRatio)
{
    if(m_contactCount<=0) return;
    
    // set arguments of kernel
    int a=0;
    int err;
    err=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
    err|=clSetKernelArg(velocityConstraintInitializeKernel,a++, sizeof(cl_mem), &clb2ManifoldsBuffer);
    //err|=clSetKernelArg(velocityConstraintInitializeKernel,a++, sizeof(cl_mem), &manifoldKeysBuffers[0]);
    //err|=clSetKernelArg(velocityConstraintInitializeKernel,a++, sizeof(cl_mem), &globalIndicesBuffers[0]);
    err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(const unsigned int),&m_contactCount);
    
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().fixtureStaticListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
    //err|=clSetKernelArg(velocityConstraintInitializeKernel, a++, sizeof(cl_mem), &(b2CLCommonData::instance().bodyDynamicListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
	//err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&islandManifoldIndicesBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&coloredContactIndexToContactIndexMapBuffer);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().shapeListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(int),&bWarmStarting);
	err|=clSetKernelArg(velocityConstraintInitializeKernel,a++,sizeof(float),&dtRatio);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
        exit(1);
    }

    int numBlocks = (m_contactCount + maxWorkGroupSizeForVelocityConstraintInitialize-1)/maxWorkGroupSizeForVelocityConstraintInitialize;
    size_t global = numBlocks * maxWorkGroupSizeForVelocityConstraintInitialize;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), velocityConstraintInitializeKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForVelocityConstraintInitialize, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute velocity constraint solver kernel!\n");
        exit(1);
    }
    
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
	//clb2PointsNode* points2 = new clb2PointsNode[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(points2, clb2PointsBuffer, 0, sizeof(clb2PointsNode)*m_contactCount, true);
	//delete [] points1;
	//delete [] points2;

	//// for debug
	//unsigned int *keysBefore = new unsigned int[sortCount];
	//unsigned int *valuesBefore = new unsigned int[sortCount];
 //   b2CLDevice::instance().copyArrayFromDevice(keysBefore, manifoldKeysBuffers[0], 0, sizeof(unsigned int)*sortCount,true);
 //   b2CLDevice::instance().copyArrayFromDevice(valuesBefore, globalIndicesBuffers[0], 0, sizeof(unsigned int)*sortCount,true);
	//delete [] keysBefore;
	//delete [] valuesBefore;
//    typedef struct{
//        float normalImpulse1;
//        float tangentImpulse1;
//        
//        float normalImpulse2;
//        float tangentImpulse2;
//    }clb2Impulse;
//	clb2Impulse * testImpulse = new clb2Impulse[m_contactCount];
//	b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2Impulse)*m_contactCount, true);
//    for (int j=0; j<m_contactCount; j++)
//    {
//        if (testImpulse[j].normalImpulse1>10 || testImpulse[j].tangentImpulse1>10 || testImpulse[j].normalImpulse2>10 || testImpulse[j].tangentImpulse2>10)
//            printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", j, testImpulse[j].normalImpulse1, testImpulse[j].tangentImpulse1, testImpulse[j].normalImpulse2, testImpulse[j].tangentImpulse2);
//    }
//    delete [] testImpulse;
}


void b2CLSolver::b2CLReadCompactedContacts(int& contactCount)
{
	b2CLDevice::instance().copyArrayFromDevice(&m_validContactCount, b2CLScan::instance().numValidDataBuffer, 0, sizeof(cl_uint), true);

	if (old_valid_contact_num < m_validContactCount)
	{
		if (validContactIndices)
			delete [] validContactIndices;
		validContactIndices = new int[m_validContactCount];

		old_valid_contact_num = m_validContactCount;
	}

#if defined(BROADPHASE_OPENCL)
	if (old_global_contact_num < m_contactCount)
	{
		if (globalIndices)
			delete [] globalIndices;
		globalIndices = new int[m_contactCount*4];

		old_global_contact_num = m_contactCount;
	}
#endif

	//printf("contact number in Solver: %d\n", m_validContactCount);
	if (m_validContactCount>0)
	{
		b2CLDevice::instance().copyArrayFromDevice(validContactIndices, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*m_validContactCount, true);

	#if defined(BROADPHASE_OPENCL)
		b2CLDevice::instance().copyArrayFromDevice(globalIndices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*m_contactCount*4, true);
	#endif
	}

	// for debug
	//for (int i=0; i<m_validContactCount; i++)
	//{
	//	int globalIndex = validContactIndices[i];
	//	int indexA = globalIndices[globalIndex*4+2];
	//	int indexB = globalIndices[globalIndex*4+3];
	//	if (indexA != 16 && indexB != 16)
	//		int a = 1;
	//}

	m_totalContactCount = m_contactCount;

	contactCount = m_validContactCount;
	m_contactCount = m_validContactCount;
}

void b2CLSolver::b2CLInitializeBodyState()
{
    if(m_bodyCount<=0) return;
    
    //	// for debug
//    typedef struct{
//        float cx;
//        float cy;
//        float a;
//    }clb2Position;	
//	clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);

    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
    err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(int),&m_bodyCount);
	err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyDynamicListBuffer));
	err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(float)*2, &m_gravity);
	err|=clSetKernelArg(bodyStateInitializeKernel,a++,sizeof(float), &m_dt);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
    //execute the kernel
    int numBlocks=(m_bodyCount + maxWorkGroupSizeForBodyStateInitializeKernel-1)/maxWorkGroupSizeForBodyStateInitializeKernel;
    size_t global=numBlocks*maxWorkGroupSizeForBodyStateInitializeKernel;
	err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), bodyStateInitializeKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForBodyStateInitializeKernel, 0, NULL, NULL);
	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute velocity constraint solver kernel!\n");
        exit(1);
    }
    
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());
    
//	//// for debug
//	clb2Position * testPos = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	printf("\tResult of b2CLInitializeBodyState()\n");
//	for (int i=0; i<m_bodyCount; i++)
//	{
//        if (testPos[i].cy>1)
//        {
//            printf("Pos %d: c=(%f, %f), a=%f\n", i, testPos[i].cx, testPos[i].cy, testPos[i].a);
//            printf("Pos before: c=(%f, %f)\n", testPosBefore[i].cx, testPosBefore[i].cy);
//        }
//	}
//    delete [] testPos;
//    delete [] testPosBefore;
}

void b2CLSolver::b2CLInitializeBodyStateFirstFrame()
{
    if(m_bodyCount<=0) return;
    
    //	// for debug
//    typedef struct{
//        float cx;
//        float cy;
//        float a;
//    }clb2Position;	
//    typedef struct{
//        float vx;
//        float vy;
//        float w;
//    }clb2Velocity;
//	clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	clb2Velocity * testVBefore = new clb2Velocity[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testVBefore, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);

    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(int),&m_bodyCount);
	err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyDynamicListBuffer));
	err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(float)*2, &m_gravity);
	err|=clSetKernelArg(bodyStateInitializeFirstFrameKernel,a++,sizeof(float), &m_dt);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
    //execute the kernel
    int numBlocks=(m_bodyCount + maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel-1)/maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel;
    size_t global=numBlocks*maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel;
	err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), bodyStateInitializeFirstFrameKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel, 0, NULL, NULL);
	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute velocity constraint solver kernel!\n");
        exit(1);
    }
    
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
//	clb2Position * testPos = new clb2Position[m_bodyCount];
//	clb2Velocity * testV = new clb2Velocity[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testV, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
//	printf("\tResult of b2CLInitializeBodyStateFirstFrame()\n");
//	for (int i=0; i<m_bodyCount; i++)
//	{
//        if (testV[i].w>100)
//        {
//            printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//            printf("V Before %d: v=(%f, %f), w=%f\n", i, testVBefore[i].vx, testVBefore[i].vy, testVBefore[i].w);
//        }
//	}
//    delete [] testPos;
//    delete [] testPosBefore;
}

void b2CLSolver::b2CLWarmStart()
{
    if(m_contactCount<=0) return;
    
    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(warmStartKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(warmStartKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(warmStartKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(warmStartKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
    err|=clSetKernelArg(warmStartKernel,a++,sizeof(const unsigned int), &m_contactCount);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
 
    //execute the kernel
    int numBlocks=(m_contactCount + maxWorkGroupSizeForWarmStartKernel-1)/maxWorkGroupSizeForWarmStartKernel;
    size_t global=numBlocks * maxWorkGroupSizeForWarmStartKernel;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartKernel, 1, NULL, &global, &maxWorkGroupSizeForWarmStartKernel, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute velocity constraint solver kernel!\n");
        exit(1);
    }
    
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());
    
	//// for debug
	//b2Velocity* result = new b2Velocity[m_bodyCount];
	//result[0].v.x = 0;
	//result[0].v.y = 0;
	//result[0].w = 0;
    //   b2CLDevice::instance().copyArrayFromDevice(result, clb2VelocitiesBuffer, 0, sizeof(float32)*3*m_bodyCount, true);
	//int abc = 0;
	//delete [] result;
}

void b2CLSolver::b2CLWarmStartWithColoring()
{
    if(m_contactCount<=0) return;
    
//typedef struct{
//    float vx;
//    float vy;
//    float w;
//}clb2Velocity;
//typedef struct{
//    float normalImpulse1;
//    float tangentImpulse1;
//    
//    float normalImpulse2;
//    float tangentImpulse2;
//}clb2Impulse;
//
//	// for debug
//	clb2Velocity * testV = new clb2Velocity[m_bodyCount];
//	clb2Impulse * testImpulse = new clb2Impulse[m_contactCount];
//	b2CLDevice::instance().copyArrayFromDevice(testV, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2Impulse)*m_contactCount, true);
//	printf("\tInput of b2CLWarmStartWithColoring()\n");
//	for (int i=m_bodyCount-6; i<m_bodyCount; i++)
//	{
//		printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//	}
//	for (int i=m_contactCount-6; i<m_contactCount; i++)
//	{
//		printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", i, testImpulse[i].normalImpulse1, testImpulse[i].tangentImpulse1, testImpulse[i].normalImpulse2, testImpulse[i].tangentImpulse2);
//	}
//	delete [] testV;
//	delete [] testImpulse;
	
	cl_event events[2];
    int curr=0, last=1;
    
	//// for debug
	//clb2Velocity* velocitiesBefore = new clb2Velocity[m_bodyCount];
	//clb2Velocity* velocitiesAfter = new clb2Velocity[m_bodyCount];
	//clb2Impulse* impulses = new clb2Impulse[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(velocitiesBefore, clb2VelocitiesBuffer, 0, sizeof(b2Velocity)*m_bodyCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(impulses, clb2ImpulsesBuffer, 0, sizeof(clb2Impulse)*m_contactCount, true);

	// set some of the arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(warmStartWithColoringKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(warmStartWithColoringKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(warmStartWithColoringKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(warmStartWithColoringKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
	for(int colorIndex=0; colorIndex<max_color; colorIndex++) 
	{    
		// set other arguments of kernel
		err|=clSetKernelArg(warmStartWithColoringKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
		err|=clSetKernelArg(warmStartWithColoringKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
		if (err != CL_SUCCESS)
		{
			printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
			exit(1);
		}
            
		//execute the kernel
		int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForWarmStartWithColoringKernel-1)/maxWorkGroupSizeForWarmStartWithColoringKernel;
		size_t global=numBlocks*maxWorkGroupSizeForWarmStartWithColoringKernel;
		if(colorIndex==0){
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartWithColoringKernel, 1, NULL, 
                                        &global, &maxWorkGroupSizeForWarmStartWithColoringKernel, 0, 
                                        NULL, &events[curr]);
			curr = last;;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
		else 
		{
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartWithColoringKernel, 1, NULL,
                                        &global, &maxWorkGroupSizeForWarmStartWithColoringKernel, 1, 
                                        &events[last], &events[curr]);
 			clReleaseEvent(events[last]);
			curr = last;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
	}
    
	clReleaseEvent(events[last]);
	//wait for commands to finish
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
	//b2CLDevice::instance().copyArrayFromDevice(velocitiesAfter, clb2VelocitiesBuffer, 0, sizeof(b2Velocity)*m_bodyCount, true);
	//delete [] velocitiesBefore;
	//delete [] velocitiesAfter;
	//delete [] impulses;

	//// for debug
	//clb2Velocity * testVAfter = new clb2Velocity[m_bodyCount];
	//b2CLDevice::instance().copyArrayFromDevice(testVAfter, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
	//printf("\tResult of b2CLWarmStartWithColoring()\n");
	//for (int i=m_bodyCount-6; i<m_bodyCount; i++)
	//{
	//	printf("VAfter %d: v=(%f, %f), w=%f\n", i, testVAfter[i].vx, testVAfter[i].vy, testVAfter[i].w);
	//}
	//delete [] testVAfter;
}


void b2CLSolver::b2CLWarmStartSplit()
{
    if(m_contactCount<=0) return;
	
	cl_event events[2];
	int curr=0, last=1;
 /*   
	float beforeArray[15000] ; 
	b2CLDevice::instance().copyArrayFromDevice(beforeArray, clb2VelocitiesBuffer, 0, sizeof(float)*3*m_bodyCount, true);
	unsigned int beforenumArray[15000] ; 
	b2CLDevice::instance().copyArrayFromDevice(beforenumArray, clb2NumContactEachBody, 0, sizeof(unsigned int)*m_bodyCount, true);

		float beforesplitArray [50000] ; 
	b2CLDevice::instance().copyArrayFromDevice(beforesplitArray, clb2SplitVelocitiesBuffer, 0, sizeof(float)*3*m_bodyCount * 20, true);
	printf ("beforesplitArray 780: %f \n", beforesplitArray[780*3*20+0]);
	//printf ("beforeArray 780: %f \n", beforeArray[780*3+0]);
	 // float testBeforeArray [10] ; 
	 // b2CLDevice::instance().copyArrayFromDevice(testBeforeArray, clb2VelocitiesBuffer, 0, sizeof(float)*3, true);
	 // printf (""); 
*/
	unsigned int numContacts = m_contactCount ; unsigned int numBody = m_bodyCount ; 
	int a=0;
    int err;
	err=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2SplitVelocitiesBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
	err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2HasSplitVelocitiesContactsBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);

    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
    err|= clSetKernelArg(warmStartSplitKernel,  a++, sizeof(unsigned int), &numContacts);
	err|= clSetKernelArg(warmStartSplitKernel,  a++, sizeof(unsigned int), &numBody);

    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set Failed to execute warm start split  arguments! %d\n", err);
        exit(1);
    }
    
	//execute the kernel

	int numElement = numContacts > numBody ? numContacts: numBody ; 
	int numBlocks=(numElement+maxWorkGroupSizeForWarmStartSplitKernel-1)/maxWorkGroupSizeForWarmStartSplitKernel;
	size_t global=numBlocks*maxWorkGroupSizeForWarmStartSplitKernel;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartSplitKernel, 1, NULL, 
                              &global, &maxWorkGroupSizeForWarmStartSplitKernel, 0, NULL, &events[curr]);

    if (err != CL_SUCCESS)
	{
		printf("Error: Failed to execute warm start split kernel!\n");
		exit(1);
	}

	
  //float testVelocity[3000];
  //b2CLDevice::instance().copyArrayFromDevice(testVelocity, clb2SplitVelocitiesBuffer, 0, sizeof(float)*3 *m_bodyCount*20, true);
  //for (int i = 0 ; i < 14 ; i ++ ) {
  //  printf ( "WarmStart Split Velcoity %d :  %f , %f , %f , %f \n", i, testVelocity[i*60+1], testVelocity[i*60+4], testVelocity[i*60+7], testVelocity[i*60+11]);
  //}
	curr = last ; 
	last = 1- last; 

	a=0; err = 0 ;

    err=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
	err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++, sizeof(cl_mem), &clb2SplitVelocitiesBuffer);
    err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
	err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
	err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
	err |= clSetKernelArg(mergeVelocityConstraintSolverKernel,  a++, sizeof(cl_int), &this->m_bodyCount);
		numBlocks = (this->m_bodyCount+maxWorkGroupSizeForVelocityConstraintSolver-1)/maxWorkGroupSizeForVelocityConstraintSolver; 
		global = numBlocks * maxWorkGroupSizeForVelocityConstraintSolver ; 
		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), mergeVelocityConstraintSolverKernel, 1, NULL, 
			&global, &maxWorkGroupSizeForVelocityConstraintSolver, 1, &events[last], &events[curr]) ;
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		clReleaseEvent (events[last]);
		//b2CLDevice::instance().copyArrayFromDevice(testArray, clb2VelocitiesBuffer, 0, sizeof(float)*3*m_bodyCount, true);
		curr = last ; 
		last = 1- last; 
		clReleaseEvent(events[last]);


/*
	unsigned int  numArray[15000] ; 
	b2CLDevice::instance().copyArrayFromDevice(numArray, clb2NumContactEachBody, 0, sizeof(unsigned int)*m_bodyCount, true);

	float velocityArray [15000] ; 
	b2CLDevice::instance().copyArrayFromDevice(velocityArray, clb2VelocitiesBuffer, 0, sizeof(float)*3*numBody, true);
	float testArray [15000] ; 
	b2CLDevice::instance().copyArrayFromDevice(testArray, clb2TestVelocitiesBuffer, 0, sizeof(float)*3*numBody, true);

	float splitArray [50000] ; 
	b2CLDevice::instance().copyArrayFromDevice(splitArray, clb2SplitVelocitiesBuffer, 0, sizeof(float)*3*numBody * 20, true);
	//printf ("i = : %d , j = : %d velocityvalue :  %f , testvalue: % f  \n", i,j, velocityArray[i*3+j], testArray[i*3+j]); 
	
	for (int i = 0; i < numBody ; i ++ ) {
		for ( int j = 0 ; j < 3 ; j ++ ) {
			if ( abs (velocityArray[i*3+j] - testArray[i*3+j]) > 0.005 ) {
				printf ("i = : %d , j = : %d velocityvalue :  %f , testvalue: % f  \n", i,j, velocityArray[i*3+j], testArray[i*3+j]); 
			}
		}		
	}


	float testArray [500] ; 
	b2CLDevice::instance().copyArrayFromDevice(testArray, clb2VelocitiesBuffer, 0, sizeof(float)*3*10, true);
	for (int i = 0 ; i < 30 ; i ++ ) {
		printf ("testArray %d : %f \n", i , testArray[i]) ; 
	}
	printf ("end"); 
*/
	//clReleaseEvent(events[last]);
}

void b2CLSolver::b2CLTestWarmStart () {
	 if(m_contactCount<=0) return;
	
	cl_event events[2];
	int curr=0, last=1;
    
	if ( clb2TestVelocitiesBuffer ) 
		b2CLDevice::instance().freeArray( clb2TestVelocitiesBuffer ) ; 
	clb2TestVelocitiesBuffer =  b2CLDevice::instance().allocateArray ( sizeof(float32) * 3 * m_bodyCount  ) ; 
	b2CLDevice::instance().copyArrayInsideDevice ( clb2VelocitiesBuffer,clb2TestVelocitiesBuffer , sizeof(float32) * 3 * m_bodyCount ) ;

	// set some of the arguments of kernel
	//b2CLDevice::instance().copyArrayInsideDevice ( this->clb2VelocitiesBuffer, this->clb2OldVelocitiesBuffer, sizeof(float32) * 3 * m_bodyCount ) ;

	int a=0;
    int err;
	err=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2TestVelocitiesBuffer);
    //err=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2OldVelocitiesBuffer);
    err|=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
	err|=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
    err|=clSetKernelArg(testWarmstartKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
	 if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }  

//	  float testBeforeArray [10] ; 
//	  b2CLDevice::instance().copyArrayFromDevice(testBeforeArray, clb2VelocitiesBuffer, 0, sizeof(float)*3, true);
//	  printf (""); 
	for(int colorIndex=0; colorIndex<max_color; colorIndex++) 
	{    
		// set other arguments of kernel
		err|=clSetKernelArg(testWarmstartKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
		err|=clSetKernelArg(testWarmstartKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
		if (err != CL_SUCCESS)
		{
			printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
			exit(1);
		}
            
		//execute the kernel
		int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForWarmStartSplitKernel-1)/maxWorkGroupSizeForWarmStartSplitKernel;
		size_t global=numBlocks*maxWorkGroupSizeForWarmStartSplitKernel;
		if(colorIndex==0){
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), testWarmstartKernel, 1, NULL, 
                                        &global, &maxWorkGroupSizeForWarmStartSplitKernel, 0, 
                                        NULL, &events[curr]);
			curr = last;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
		else 
		{
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), testWarmstartKernel, 1, NULL,
                                        &global, &maxWorkGroupSizeForWarmStartSplitKernel, 1, 
                                        &events[last], &events[curr]);
 			clReleaseEvent(events[last]);
			curr = last;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
	 // float testArray [10] ; 
	 // b2CLDevice::instance().copyArrayFromDevice(testArray, clb2VelocitiesBuffer, 0, sizeof(float)*3, true);
	//  printf (""); 
	}
    
	b2CLDevice::instance().copyArrayInsideDevice ( clb2TestVelocitiesBuffer,clb2VelocitiesBuffer , sizeof(float32) * 3 * m_bodyCount ) ;

	clReleaseEvent(events[last]);
}
/*
void b2CLSolver::b2CLWarmStartSplit()
{
    if(m_contactCount<=0) return;
	
	cl_event events[2];
	int curr=0, last=1;
    

	// set some of the arguments of kernel
	//b2CLDevice::instance().copyArrayInsideDevice ( this->clb2VelocitiesBuffer, this->clb2OldVelocitiesBuffer, sizeof(float32) * 3 * m_bodyCount ) ;

	int a=0;
    int err;
	err=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    //err=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2OldVelocitiesBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
	err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
    err|=clSetKernelArg(warmStartSplitKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
	 if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }  

//	  float testBeforeArray [10] ; 
//	  b2CLDevice::instance().copyArrayFromDevice(testBeforeArray, clb2VelocitiesBuffer, 0, sizeof(float)*3, true);
//	  printf (""); 
	for(int colorIndex=0; colorIndex<max_color; colorIndex++) 
	{    
		// set other arguments of kernel
		err|=clSetKernelArg(warmStartSplitKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
		err|=clSetKernelArg(warmStartSplitKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
		if (err != CL_SUCCESS)
		{
			printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
			exit(1);
		}
            
		//execute the kernel
		int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForWarmStartSplitKernel-1)/maxWorkGroupSizeForWarmStartSplitKernel;
		size_t global=numBlocks*maxWorkGroupSizeForWarmStartSplitKernel;
		if(colorIndex==0){
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartSplitKernel, 1, NULL, 
                                        &global, &maxWorkGroupSizeForWarmStartSplitKernel, 0, 
                                        NULL, &events[curr]);
			curr = last;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
		else 
		{
			err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), warmStartSplitKernel, 1, NULL,
                                        &global, &maxWorkGroupSizeForWarmStartSplitKernel, 1, 
                                        &events[last], &events[curr]);
 			clReleaseEvent(events[last]);
			curr = last;
			last = 1-last;
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to execute warm start with coloring kernel!\n");
				exit(1);
			}
		}
	 // float testArray [10] ; 
	 // b2CLDevice::instance().copyArrayFromDevice(testArray, clb2VelocitiesBuffer, 0, sizeof(float)*3, true);
	//  printf (""); 
	}
    
	clReleaseEvent(events[last]);

}
*/
void b2CLSolver::b2CLSolveVelocityConstraint(int numberOfIterations)
{
    if(m_contactCount<=0) return;
    
//typedef struct{
//    float cx;
//    float cy;
//    float a;
//}clb2Position;	
//typedef struct{
//float vx;
//float vy;
//float w;
//}clb2Velocity;
//typedef struct{
//float normalImpulse1;
//float tangentImpulse1;
//
//float normalImpulse2;
//float tangentImpulse2;
//}clb2Impulse;
//typedef struct{
//    float rA1[2];
//    float rB1[2];
//    float normalMass1;
//    float tangentMass1;
//    
//    float rA2[2];
//    float rB2[2];
//    float normalMass2;
//    float tangentMass2;
//    
//    float velocityBias1;
//    float velocityBias2;
//}clb2Points;
//typedef struct{
//    int color;
//    int indexA;
//    int indexB;
//    float friction;
//    float normal[2];
//    float invMassA;
//    float invIA;
//    float invMassB;
//    float invIB;
//}clb2Contact;
    //	
//	// for debug
//	//clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	clb2Velocity * testV = new clb2Velocity[m_bodyCount];
//	clb2Impulse * testImpulse = new clb2Impulse[m_contactCount];
//    clb2Points * testPoint = new clb2Points[m_contactCount];
//    clb2Contact * testContact = new clb2Contact[m_contactCount];
//	//b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testV, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2Impulse)*m_contactCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testPoint, clb2PointsBuffer, 0, sizeof(clb2Points)*m_contactCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testContact, clb2ContactsBuffer, 0, sizeof(clb2Contact)*m_contactCount, true);
//	printf("\tInput of b2CLSolveVelocityConstraint()\n");
//	for (int i=0; i<m_bodyCount; i++)
//	{
//		//printf("Pos %d: c=(%f, %f), a=%f\n", i, testPosBefore[i].cx, testPosBefore[i].cy, testPosBefore[i].a);
//		printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//	}
//	for (int i=m_contactCount-6; i<m_contactCount; i++)
//	{
//		printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", i, testImpulse[i].normalImpulse1, testImpulse[i].tangentImpulse1, testImpulse[i].normalImpulse2, testImpulse[i].tangentImpulse2);
//	}
//	delete [] testV;
//	//delete [] testPosBefore;
//	delete [] testImpulse;

	cl_event events[2];
    int curr=0, last=1;
    
    // set some of the arguments of kernel

	//float testArray [500];
	//cl_mem testBuffer ; 
	//testBuffer = b2CLDevice::instance().allocateArray ( sizeof(float) * 500 ) ; 
	//b2CLDevice::instance().copyArrayToDevice(testBuffer, testArray, 0, sizeof(float)*500, true);
    
	int a=0;

    int err=clSetKernelArg(velocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(velocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    err|=clSetKernelArg(velocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
    err|=clSetKernelArg(velocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
	//err|=clSetKernelArg(velocityConstraintSolverKernel,a++,sizeof(cl_mem),&testBuffer);


    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    //// for debug
    // b2Timer intertime;
//	 float testbefore[3000];
//  b2CLDevice::instance().copyArrayFromDevice(testbefore, clb2VelocitiesBuffer, 0, sizeof(float)*3 *m_bodyCount, true);
//   for (int i = 0 ; i < 14 ; i ++ ) {
//    printf ( "Before Velcoity %d :  %f  \n", i, testbefore[i*3+1]);
 // }
    for(int iter=0;iter<numberOfIterations;iter++) 
	{
  		        //// for debug
                        //if (iter != 0 ) { 
			//   b2CLDevice::instance().finishCommandQueue();
                        //   float itertime = intertime.GetMilliseconds();
                        //   printf ("Iter %d: time: %f. \n",iter-1, itertime );
                        //   intertime.Reset() ; 
                        //}

		for(int colorIndex=0; colorIndex<max_color; colorIndex++) 
		{    
                        
			// set other arguments of kernel
			err=clSetKernelArg(velocityConstraintSolverKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
			err|=clSetKernelArg(velocityConstraintSolverKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
				exit(1);
			}
            
			//execute the kernel
			int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForVelocityConstraintSolver-1)/maxWorkGroupSizeForVelocityConstraintSolver;
			size_t global=numBlocks*maxWorkGroupSizeForVelocityConstraintSolver;
			if(colorIndex==0){
				err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), velocityConstraintSolverKernel, 1, NULL, 
                                           &global, &maxWorkGroupSizeForVelocityConstraintSolver, iter?1:0, 
                                           iter?&events[last]:NULL, &events[curr]);
				if (iter)
				{
					clReleaseEvent(events[last]);
				}
				curr = last;;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute velocity constraint solver kernel!\n");
					exit(1);
				}
			}
			else 
			{
				err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), velocityConstraintSolverKernel, 1, NULL,
                                           &global, &maxWorkGroupSizeForVelocityConstraintSolver, 1, 
                                           &events[last], &events[curr]);
 				clReleaseEvent(events[last]);
				curr = last;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute velocity constraint solver kernel!\n");
					exit(1);
				}
			}
			//b2CLDevice::instance().copyArrayFromDevice(testArray, testBuffer, 0, sizeof(float)*500, true);
 			//printf ("for debug \n" ); 
			
            //// for debug
            ////clb2Position * testPosBefore = new clb2Position[m_bodyCount];
            //clb2Velocity * testVAfter = new clb2Velocity[m_bodyCount];
            ////b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
            //b2CLDevice::instance().copyArrayFromDevice(testVAfter, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
            //printf("\tResult of b2CLSolveVelocityConstraint()\n");
//            for (int i=0; i<m_bodyCount; i++)
//            {
//                //printf("Pos %d: c=(%f, %f), a=%f\n", i, testPosBefore[i].cx, testPosBefore[i].cy, testPosBefore[i].a);
//                if (testVAfter[i].vx>1 || testVAfter[i].vx<-1 || testVAfter[i].vy>1 || testVAfter[i].vy<-1 || testVAfter[i].w>1 || testVAfter[i].w<-1)
//                {
//                    printf("V %d: v=(%f, %f), w=%f\n", i, testVAfter[i].vx, testVAfter[i].vy, testVAfter[i].w);
//                    
//                    for (int j=0; j<m_contactCount; j++)
//                    {
//                        if (testImpulse[j].normalImpulse1>10 || testImpulse[j].tangentImpulse1>10 || testImpulse[j].normalImpulse2>10 || testImpulse[j].tangentImpulse2>10)
//                            printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", j, testImpulse[j].normalImpulse1, testImpulse[j].tangentImpulse1, testImpulse[j].normalImpulse2, testImpulse[j].tangentImpulse2);
//                    }
//                    
//                    printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//                    printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", i, testImpulse[i].normalImpulse1, testImpulse[i].tangentImpulse1, testImpulse[i].normalImpulse2, testImpulse[i].tangentImpulse2);
//                    printf("Point %d: rA1(%f, %f), rB1(%f, %f), normalMass1: %f, tangentMass1: %f, velocityBias1: %f\n", i, testPoint[i].rA1[0], testPoint[i].rA1[1], testPoint[i].rB1[0], testPoint[i].rB1[1], testPoint[i].normalMass1, testPoint[i].tangentMass1, testPoint[i].velocityBias1);
//                    printf("Point %d: rA2(%f, %f), rB2(%f, %f), normalMass2: %f, tangentMass2: %f, velocityBias2: %f\n", i, testPoint[i].rA2[0], testPoint[i].rA2[1], testPoint[i].rB2[0], testPoint[i].rB2[1], testPoint[i].normalMass2, testPoint[i].tangentMass2, testPoint[i].velocityBias2);
//                    printf("Contact %d: color: %d, indexA: %d, indexB: %d, friction: %f, normal(%f, %f), iMA: %f, iIA: %f, iMB: %f, iIB: %f\n", i, testContact[i].color, testContact[i].indexA, testContact[i].indexB, testContact[i].friction, testContact[i].normal[0], testContact[i].normal[1], testContact[i].invMassA, testContact[i].invIA, testContact[i].invMassB, testContact[i].invIB);
//                }
//            }
//            delete [] testVAfter;
            ////delete [] testPosBefore;
		}
		   //                    float testmerge[3000];
            //           b2CLDevice::instance().copyArrayFromDevice(testmerge, clb2VelocitiesBuffer, 0, sizeof(float)*3 *m_bodyCount, true);
             //          for (int i = 0 ; i < 14 ; i ++ ) {
               //                 printf ( "After Velcoity %d :  %f  \n", i, testmerge[i*3+1]);
                 //       }
					//	printf (" "); 
                       
	}
    
	//// for debug
	//b2Velocity* velocities = new b2Velocity[m_bodyCount];
	//b2CLDevice::instance().copyArrayFromDevice(velocities, clb2VelocitiesBuffer, 0, sizeof(b2Velocity)*m_bodyCount, true);
	//delete [] velocities;

	clReleaseEvent(events[last]);
	//delete [] testArray  ; 
	//wait for commands to finish
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

}


void b2CLSolver::b2CLSolveJointVelocityConstraint(int numberOfIterations)
{

	cl_event events[2]; int curr=0, last=1; int a=0;   bool noqueue = true ; int err ; 
	if (b2CLCommonData::instance().numTotalJoints == 0) return ; 						    
    for(int iter=0;iter<numberOfIterations;iter++) 
	{
		for ( int jointType = 1 ; jointType < 11 ; jointType ++) {
			int numContact = b2CLCommonData::instance().numJoints[jointType] ; 
			if (numContact == 0) continue ;   
			int maxcolor = b2CLCommonData::instance().jointMaxColor[jointType] ; 
			a= 0 ; 
			err=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a++,sizeof(cl_mem),&clb2VelocitiesBuffer);						    
			err|=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a++,sizeof(cl_mem),& (b2CLCommonData::instance().jointListBuffer) );
			err|=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a++,sizeof(float),&m_dt);
			
			for(int colorIndex=0; colorIndex < maxcolor; colorIndex++) 
			{    
                     		   
					unsigned int length = b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex+1] - b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex]; 
					unsigned int offset = b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex];
					err|=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a,sizeof(const unsigned int),&offset );	
					err|=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a+1,sizeof(const unsigned int),&length);

					//err|=clSetKernelArg(jointVelocityConstraintSolverKernel[jointType],a++,sizeof(cl_mem),&testBuffer);
					if (err != CL_SUCCESS)
					{
						printf("Error: Failed to set velocity constraint solver arguments! %d\n", err); exit(1);
					}
            
					//execute the kernel
					int numBlocks=(length+maxWorkGroupSizeForJointVelocityConstraintSolver[jointType]-1)/maxWorkGroupSizeForJointVelocityConstraintSolver[jointType];
					size_t global=numBlocks*maxWorkGroupSizeForJointVelocityConstraintSolver[jointType];
					if(colorIndex==0){
							err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), jointVelocityConstraintSolverKernel[jointType], 1, NULL, 
                                           &global, &maxWorkGroupSizeForJointVelocityConstraintSolver[jointType], iter?1:0, 
                                           iter?&events[last]:NULL, &events[curr]);
						if (noqueue == false)
							clReleaseEvent(events[last]);
						curr = last;
						last = 1-last;
						if (err != CL_SUCCESS)
						{
							printf("Error: Failed to execute velocity constraint solver kernel!\n");
							exit(1);
						}
						if (noqueue) noqueue = false ; 
					}
					else 
					{
						err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), jointVelocityConstraintSolverKernel[jointType], 1, NULL,
                                           &global, &maxWorkGroupSizeForJointVelocityConstraintSolver[jointType], 1, 
                                           &events[last], &events[curr]);
 						clReleaseEvent(events[last]);
						curr = last;
						last = 1-last;
						if (err != CL_SUCCESS)
						{
							printf("Error: Failed to execute velocity constraint solver kernel!\n");
							exit(1);
						}
					}
			
			}	// for (color index)
		} // for (jointType)
	}// for (Iterations)	
  if (noqueue == false )
    	clReleaseEvent(events[last]);

}

void b2CLSolver::b2CLSolveSplitVelocityConstraint(int numberOfIterations)
{
    if(m_contactCount<=0) return;

	cl_event events[2];
    int curr=0, last=1;
    
    // set some of the arguments of kernel
    int a=0; int err = 0 ; int numBlocks ; size_t global ; 

    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
        exit(1);
    }

	//cl_mem oldVelocitiesBuffer
	unsigned int numContact = m_contactCount ; 
    for(int iter=0;iter<numberOfIterations;iter++) 
	{
		unsigned int lastIter = ((numberOfIterations-1)==iter)?1:0 ; 
		a=0; err = 0 ; 
		err=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
		//err=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2OldVelocitiesBuffer);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++, sizeof(cl_mem), &clb2SplitVelocitiesBuffer);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
	   // err |= clSetKernelArg(splitVelocityConstraintSolverKernel,  a++, sizeof(cl_mem), &testBuffer);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
		err|=clSetKernelArg(splitVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
		err |= clSetKernelArg(splitVelocityConstraintSolverKernel,  a++, sizeof(unsigned int), &numContact);
		//err |= clSetKernelArg(splitVelocityConstraintSolverKernel,  a++, sizeof(unsigned int), &lastIter);

		numBlocks = (this->m_contactCount+maxWorkGroupSizeForVelocityConstraintSolver-1)/maxWorkGroupSizeForVelocityConstraintSolver;
		 global = numBlocks*maxWorkGroupSizeForVelocityConstraintSolver; 

		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), splitVelocityConstraintSolverKernel, 1, NULL, 
			&global, &maxWorkGroupSizeForVelocityConstraintSolver, iter?1:0, iter?&events[last]:NULL, &events[curr]); 
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		if (iter) {
			clReleaseEvent (events[last]);
		}
		curr = last ; 
		last = 1- last; 

		a=0; err = 0 ;

		err=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++, sizeof(cl_mem), &clb2SplitVelocitiesBuffer);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PointsBuffer);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
		err|=clSetKernelArg(mergeVelocityConstraintSolverKernel,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
		err |= clSetKernelArg(mergeVelocityConstraintSolverKernel,  a++, sizeof(cl_int), &this->m_bodyCount);
		numBlocks = (this->m_bodyCount+maxWorkGroupSizeForVelocityConstraintSolver-1)/maxWorkGroupSizeForVelocityConstraintSolver; 
		global = numBlocks * maxWorkGroupSizeForVelocityConstraintSolver ; 
		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), mergeVelocityConstraintSolverKernel, 1, NULL, 
			&global, &maxWorkGroupSizeForVelocityConstraintSolver, 1, &events[last], &events[curr]) ;
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		clReleaseEvent (events[last]);
		curr = last ; 
		last = 1- last; 
	}
	//printf ("funcion is done \n") ; 


}


void b2CLSolver::b2CLIntegratePositions()
{
    if(m_bodyCount<=0) return;
    
//typedef struct{
//    float cx;
//    float cy;
//    float a;
//}clb2Position;	
//typedef struct{
//    float vx;
//    float vy;
//    float w;
//}clb2Velocity;
//	
//	// for debug
//	clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	clb2Velocity * testV = new clb2Velocity[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testV, clb2VelocitiesBuffer, 0, sizeof(clb2Velocity)*m_bodyCount, true);
//	printf("\tInput of b2CLIntegratePositions()\n");
//	for (int i=m_bodyCount-6; i<m_bodyCount; i++)
//	{
//		printf("Pos %d: c=(%f, %f), a=%f\n", i, testPosBefore[i].cx, testPosBefore[i].cy, testPosBefore[i].a);
//		printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//	}
//	delete [] testV;
//	delete [] testPosBefore;

    int a=0;
    int err=clSetKernelArg(integratePositionKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(integratePositionKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=clSetKernelArg(integratePositionKernel,a++,sizeof(const float32),&m_dt);
    err|=clSetKernelArg(integratePositionKernel,a++,sizeof(const int),&m_bodyCount);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set integrate positions arguments! %d\n", err);
        exit(1);
    }
    
    int numBlocks=(m_bodyCount+maxWorkGroupSizeForIntegratingPositions-1)/maxWorkGroupSizeForIntegratingPositions;
    size_t global=numBlocks*maxWorkGroupSizeForIntegratingPositions;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),integratePositionKernel,1,NULL,&global,&maxWorkGroupSizeForIntegratingPositions,0,NULL,NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to integrate position kernel!\n");
        exit(1);
    }
    //wait for commands to finish
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
//	clb2Position * testPos = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	printf("\tResult of b2CLIntegratePositions()\n");
//	for (int i=0; i<m_bodyCount; i++)
//	{
//        if (testPos[i].cy>1)
//        {
//            printf("Pos %d: c=(%f, %f), a=%f\n", i, testPos[i].cx, testPos[i].cy, testPos[i].a);
//            printf("Pos before: c=(%f, %f)\n", testPosBefore[i].cx, testPosBefore[i].cy);
//            printf("V %d: v=(%f, %f), w=%f\n", i, testV[i].vx, testV[i].vy, testV[i].w);
//        }
//	}
//	delete [] testPos;
//    delete [] testPosBefore;
//    delete [] testV;
}
void b2CLSolver::solveStaticDynamicPair () {

}

void b2CLSolver::syncSDPosition ( clb2SDBody* sdBodyArray, int bdSize ) {
	if (bdSize == 0) return ; 
    int a = 0 ; int err ; 
	
	int bodyElmSize = 12 ; 

	err=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyDynamicListBuffer));
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_mem),&sdBodyDataBuffer);
	err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(const float32),&m_dt);
    err|=clSetKernelArg(syncSDBodyKernel,a++,sizeof(cl_int),&bdSize);

	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set syncSDBodyKernel arguments! %d\n", err);
        exit(1);
    }
	int numBlocks=(bdSize +maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
    size_t global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
	err = clEnqueueNDRangeKernel ( b2CLDevice::instance().GetCommandQueue(), syncSDBodyKernel,1, NULL, &global, &maxWorkGroupSizeForPositionConstraintSolver, 0, NULL, NULL ) ;
    if (err != CL_SUCCESS)
    {
         printf("Error: Failed to execute syncSDBodyKernel kernel!\n");
         exit(1);
    }
	b2CLDevice::instance().copyArrayFromDevice(sdBodyArray, sdBodyDataBuffer, 0, sizeof(clb2SDBody)*bdSize, true);

}

void b2CLSolver::collectStaticDynamicPair (clb2SDManifold* sdManifoldArray, clb2SDContact* contactArray, int mfSize, clb2SDBody* sdBodyArray, int bdSize ) {
    if (mfSize == 0 ) return ; 
	
	int mfElmSize = 10 ; int contactElmSize = 4 ; int bodyElmSize = 12 ; 

    if (sdManifoldDataBuffer != NULL) b2CLDevice::instance().freeArray(sdManifoldDataBuffer);
	//sdManifoldDataBuffer = b2CLDevice::instance().allocateArray (sizeof(float)* mfSize* mfElmSize);
	sdManifoldDataBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2SDManifold)* mfSize);
	if (sdBodyDataBuffer != NULL) b2CLDevice::instance().freeArray(sdBodyDataBuffer);
	//sdBodyDataBuffer = b2CLDevice::instance().allocateArray (sizeof(float)* bdSize* bodyElmSize);
	sdBodyDataBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2SDBody)* bdSize);
	if (sdContactDataBuffer != NULL ) b2CLDevice::instance().freeArray(sdContactDataBuffer);
	//sdContactDataBuffer = b2CLDevice::instance().allocateArray (sizeof(int)* mfSize* contactElmSize);
	sdContactDataBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2SDContact)* mfSize);
	if (sdContactBuffer != NULL ) b2CLDevice::instance().freeArray(sdContactBuffer);
	sdContactBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2ContactNode)* mfSize);
	if (sdPointBuffer != NULL ) b2CLDevice::instance().freeArray(sdPointBuffer);
	sdPointBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2PointsNode)* mfSize);
	if (sdImpulseBuffer != NULL ) b2CLDevice::instance().freeArray (sdImpulseBuffer); 
	sdImpulseBuffer = b2CLDevice::instance().allocateArray (sizeof(clb2ImpulseNode)*mfSize); 
    if (sdManifoldBuffer != NULL) b2CLDevice::instance().freeArray(sdManifoldBuffer);
	sdManifoldBuffer = b2CLDevice::instance().allocateArray ( mfSize* sizeof(b2clManifold) );
	

	b2CLDevice::instance().copyArrayToDevice(sdManifoldDataBuffer, sdManifoldArray, 0, sizeof(clb2SDManifold)* mfSize , true ) ; 
	b2CLDevice::instance().copyArrayToDevice(sdBodyDataBuffer, sdBodyArray, 0, sizeof(clb2SDBody)* bdSize, true ) ; 
	b2CLDevice::instance().copyArrayToDevice(sdContactDataBuffer, contactArray, 0, sizeof(clb2SDContact)* mfSize, true ) ; 
    int a = 0 ; int err ; 

	err=clSetKernelArg(this->initSDBodyKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(initSDBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
	err|=clSetKernelArg(initSDBodyKernel,a++,sizeof(cl_mem),&sdBodyDataBuffer);
    err|=clSetKernelArg(initSDBodyKernel,a++,sizeof(int),&bdSize);
	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initSDBodyKernel arguments! %d\n", err);
        exit(1);
    }
	int numBlocks=(bdSize +maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
    size_t global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
	err = clEnqueueNDRangeKernel ( b2CLDevice::instance().GetCommandQueue(), initSDBodyKernel,1, NULL, &global, &maxWorkGroupSizeForPositionConstraintSolver, 0, NULL, NULL ) ;
    if (err != CL_SUCCESS)
    {
         printf("Error: Failed to execute initSDBodyKernel kernel!\n");
         exit(1);
    }




	a = 0 ; 
	err = clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&sdContactBuffer);
	err |= clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&sdPointBuffer);
	err |= clSetKernelArg (this->collectStaticDynamicPairKernel, a++ , sizeof(cl_mem), &sdManifoldBuffer);
	//err |= clSetKernelArg (this->collectStaticDynamicPairKernel, a++ , sizeof(cl_mem), &sdManifoldBuffer);
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&sdImpulseBuffer); 
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().fixtureStaticListBuffer));
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().shapeListBuffer));
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&sdContactDataBuffer);
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_mem),&sdManifoldDataBuffer);
	err|=clSetKernelArg(this->collectStaticDynamicPairKernel,a++,sizeof(cl_int),& mfSize);
	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set collectStaticDynamicPair arguments! %d\n", err);
        exit(1);
    }
	numBlocks=(mfSize +maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
    global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
	err = clEnqueueNDRangeKernel ( b2CLDevice::instance().GetCommandQueue(), collectStaticDynamicPairKernel,1, NULL, &global, &maxWorkGroupSizeForPositionConstraintSolver, 0, NULL, NULL ) ;
    if (err != CL_SUCCESS)
    {
         printf("Error: Failed to execute collectStaticDynamicPair kernel!\n");
         exit(1);
    }



}

void b2CLSolver::solveSDPosition ( int numIteration, int mfSize, unsigned int* colorOffsets, unsigned int* colorLengths, int max_color ) {

	
	cl_event events[2];
	int curr = 0, last =1 ; 

	int a=0;
    int err=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&sdManifoldBuffer);
   // err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&clb2ManifoldsBuffer);
    err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&sdContactBuffer);
	//err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
	err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(solveSDPositionKernel,a++,sizeof(cl_int), &mfSize);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set DynamicStatic position constraint solver arguments! %d\n", err);
        exit(1);
    }
	for (int iter = 0 ; iter < numIteration; iter ++ ) {
		for (int colorIndex = 0 ; colorIndex < max_color; colorIndex ++) {
			err =clSetKernelArg (solveSDPositionKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
			err|=clSetKernelArg(solveSDPositionKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to set DynamicStatic position constraint solver arguments! %d\n", err);
				exit(1);
			}
			int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
			size_t global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
            if(colorIndex==0){
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),solveSDPositionKernel,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,iter?1:0,iter?&events[last]:NULL,&events[curr]);
                if(iter)
                {
                    clReleaseEvent(events[last]);
                }
                curr = last;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute solveSDPosition kernel!\n");
					exit(1);
				}
            }else {
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),solveSDPositionKernel,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,1,&events[last],&events[curr]);
                clReleaseEvent(events[last]);
                curr = last;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute solveSDPosition kernel!\n");
					exit(1);
				}
            }
		}

	}
	clReleaseEvent(events[last]);

}

void b2CLSolver::solveSDVelocity(int numIteration, int sdNum, unsigned int* colorOffsets, unsigned int* colorLengths, int max_color)
{
    cl_event events[2]; 
	int curr = 0, last = 1 ; 

	int a=0;
    int err=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_mem),&clb2VelocitiesBuffer);
    err|=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_mem),&sdContactBuffer);
	err|=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_mem),&sdImpulseBuffer); 
    err|=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_mem),&sdPointBuffer);
	err|=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(solveSDVelocityKernel,a++,sizeof(cl_int), &sdNum);


    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set sovleSDvelocity constraint solver arguments! %d\n", err);
        exit(1);
    }

	for (int iter = 0 ; iter < numIteration; iter ++ ) {
		for (int colorIndex = 0 ; colorIndex < max_color; colorIndex ++) {
			err =clSetKernelArg (solveSDVelocityKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
			err|=clSetKernelArg(solveSDVelocityKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
			if (err != CL_SUCCESS)
			{
				printf("Error: Failed to set DynamicStatic position constraint solver arguments! %d\n", err);
				exit(1);
			}
			int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForVelocityConstraintSolver-1)/maxWorkGroupSizeForVelocityConstraintSolver;
			size_t global=numBlocks*maxWorkGroupSizeForVelocityConstraintSolver;
            if(colorIndex==0){
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),solveSDVelocityKernel,1,NULL,&global,&maxWorkGroupSizeForVelocityConstraintSolver,iter?1:0,iter?&events[last]:NULL,&events[curr]);
                if(iter)
                {
                    clReleaseEvent(events[last]);
                }
                curr = last;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute solveSDPosition kernel!\n");
					exit(1);
				}
            }else {
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),solveSDVelocityKernel,1,NULL,&global,&maxWorkGroupSizeForVelocityConstraintSolver,1,&events[last],&events[curr]);
                clReleaseEvent(events[last]);
                curr = last;
				last = 1-last;
				if (err != CL_SUCCESS)
				{
					printf("Error: Failed to execute solveSDPosition kernel!\n");
					exit(1);
				}
            }
		}

	}
	clReleaseEvent(events[last]);
}



void b2CLSolver::b2CLSolvePositionConstraint(int numberOfIterations)
{
    if(m_contactCount<=0) return;
    
    //printf("number of iterations for positoin: %d\n", numberOfIterations);
//    float rand_n = ((float) rand() / (float)RAND_MAX )*1000000;
//    float sum = 0;
//    for (int i=0; i<rand_n; i++)
//        sum += i;
//    printf("max color: %f\n", sum);
     
//typedef struct{
//    float cx;
//    float cy;
//    float a;
//}clb2Position;	
//	// for debug
//	clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	printf("\tInput of b2CLSolvePositionConstraint()\n");
//	for (int i=m_bodyCount-6; i<m_bodyCount; i++)
//	{
//		printf("Pos %d: c=(%f, %f), a=%f\n", i, testPosBefore[i].cx, testPosBefore[i].cy, testPosBefore[i].a);
//	}
//	delete [] testPosBefore;
//
//

    cl_event events[2];
    int curr=0, last=1;
    
    // set some of the arguments of kernel
    int a=0;
    int err=clSetKernelArg(positionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=clSetKernelArg(positionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ManifoldsBuffer);
    err|=clSetKernelArg(positionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set position constraint solver arguments! %d\n", err);
        exit(1);
    }
    
    //printf("number of interation for position: %d\n", numberOfIterations);
    for(int iter=0;iter<numberOfIterations;iter++){ 
        for(int colorIndex=0; colorIndex<max_color; colorIndex++) {
            // set other arguments of kernel
            err=clSetKernelArg(positionConstraintSolverKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
            err|=clSetKernelArg(positionConstraintSolverKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
            if (err != CL_SUCCESS)
            {
                printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
                exit(1);
            }
            
            //execute the kernel
            int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
            size_t global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
            if(colorIndex==0){
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),positionConstraintSolverKernel,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,iter?1:0,iter?&events[last]:NULL,&events[curr]);
                if(iter)
                {
                    clReleaseEvent(events[last]);
                }
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }else {
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),positionConstraintSolverKernel,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,1,&events[last],&events[curr]);
                clReleaseEvent(events[last]);
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }
        }
    }
	clReleaseEvent(events[last]);
    //wait for commands to finish
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

//	// for debug
//	clb2Position * testPos = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	printf("\tResult of b2CLSolvePositionConstraint()\n");
//	for (int i=0; i<m_bodyCount; i++)
//	{
//        if (testPos[i].cy>1)
//        {
//            printf("Pos %d: c=(%f, %f), a=%f\n", i, testPos[i].cx, testPos[i].cy, testPos[i].a);
//            printf("Pos before: c=(%f, %f)\n", testPosBefore[i].cx, testPosBefore[i].cy);
//        }
//	}
//	delete [] testPos;
}

void b2CLSolver::b2CLSolveSplitPositionConstraint(int numberOfIterations)
{

if(m_contactCount<=0) return;
    

	cl_event events[2];
    int curr=0, last=1;
    
    // set some of the arguments of kernel
    int a=0; int err = 0 ; int numBlocks ; size_t global ; 




    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
        exit(1);
    }

	//cl_mem oldVelocitiesBuffer
	unsigned int numContact = m_contactCount ; 
    for(int iter=0;iter<numberOfIterations;iter++) 
	{
		unsigned int lastIter = ((numberOfIterations-1)==iter)?1:0 ; 
    a=0; err = 0 ; 
    err=clSetKernelArg(splitPositionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=clSetKernelArg(splitPositionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ManifoldsBuffer);
    err|=clSetKernelArg(splitPositionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
	err|=clSetKernelArg(splitPositionConstraintSolverKernel,a++, sizeof(cl_mem), &clb2SplitVelocitiesBuffer);
	err|=clSetKernelArg(splitPositionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2NumContactEachBody);
	err|=clSetKernelArg(splitPositionConstraintSolverKernel,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
	err |= clSetKernelArg(splitPositionConstraintSolverKernel,  a++, sizeof(unsigned int), &numContact) ; 

		numBlocks = (this->m_contactCount+maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
		 global = numBlocks*maxWorkGroupSizeForPositionConstraintSolver; 

		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), splitPositionConstraintSolverKernel, 1, NULL, 
			&global, &maxWorkGroupSizeForVelocityConstraintSolver, iter?1:0, iter?&events[last]:NULL, &events[curr]); 
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		if (iter) {
			clReleaseEvent (events[last]);
		}
		curr = last ; 
		last = 1- last; 

	a=0; err = 0 ;

    err=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++, sizeof(cl_mem), &clb2SplitVelocitiesBuffer);
    err|=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2ContactsBuffer);
	err|=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2NumContactEachBody);
	err|=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2Contact2BodySplitVelocitiesIndex);
	err |= clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,  a++, sizeof(cl_int), &this->m_bodyCount);
		numBlocks = (this->m_bodyCount+maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver; 
		global = numBlocks * maxWorkGroupSizeForPositionConstraintSolver ; 
		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), positionConstraintSolverKernel_MergeSplittedMass, 1, NULL, 
			&global, &maxWorkGroupSizeForPositionConstraintSolver, 1, &events[last], &events[curr]) ;
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		clReleaseEvent (events[last]);
		curr = last ; 
		last = 1- last; 
	}
	//printf ("funcion is done \n") ; 
    if(m_contactCount<=0) return;

}





void b2CLSolver::b2CLSolvePositionConstraint_MergeSplittedMass(int numberOfIterations)
{
    if(m_contactCount<=0) return;
    
    cl_event events[2];
    int curr=0, last=1;
    

    // set some of the arguments of kernel
    int a=0;
	int err = clSetKernelArg (positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2NumContactEachBody);
    err|=     clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=     clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2ManifoldsBuffer);
    err|=     clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set position constraint solver arguments! %d\n", err);
        exit(1);
    }
    
    //printf("number of interation for position: %d\n", numberOfIterations);
    for(int iter=0;iter<numberOfIterations;iter++){ 
        for(int colorIndex=0; colorIndex<max_color; colorIndex++) {
            // set other arguments of kernel
            err=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
            err|=clSetKernelArg(positionConstraintSolverKernel_MergeSplittedMass,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
            if (err != CL_SUCCESS)
            {
                printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
                exit(1);
            }
            
            //execute the kernel
            int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForPositionConstraintSolver-1)/maxWorkGroupSizeForPositionConstraintSolver;
            size_t global=numBlocks*maxWorkGroupSizeForPositionConstraintSolver;
            if(colorIndex==0){
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),positionConstraintSolverKernel_MergeSplittedMass,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,iter?1:0,iter?&events[last]:NULL,&events[curr]);
                if(iter)
                {
                    clReleaseEvent(events[last]);
                }
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }else {
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),positionConstraintSolverKernel_MergeSplittedMass,1,NULL,&global,&maxWorkGroupSizeForPositionConstraintSolver,1,&events[last],&events[curr]);
                clReleaseEvent(events[last]);
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }
        }
    }
	clReleaseEvent(events[last]);
}

void b2CLSolver::b2CLSolveJointPositionConstraint(int numberOfIterations)
{
	cl_event events[2]; int curr = 0, last = 1; 	int a = 0;   bool noqueue = true; int err ; 
	if (b2CLCommonData::instance().numTotalJoints == 0) return ; 
		 
    for (int iter = 0; iter < numberOfIterations; iter++) 
	{
		for (int jointType = 1; jointType < 11; jointType++) 
		{
			if (b2CLCommonData::instance().numJoints[jointType] == 0) 
				continue; 
			int maxcolor = b2CLCommonData::instance().jointMaxColor[jointType];		
			a = 0 ; 
			err = clSetKernelArg(jointPositionConstraintSolverKernel[jointType],a++,sizeof(cl_mem),&clb2PositionsBuffer);						    
			err |= clSetKernelArg(jointPositionConstraintSolverKernel[jointType],a++,sizeof(cl_mem),& (b2CLCommonData::instance().jointListBuffer) );			 			
			for (int colorIndex = 0; colorIndex < maxcolor; colorIndex++) 
			{
						unsigned int length = b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex + 1] - b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex]; 
						unsigned int offset = b2CLCommonData::instance().jointColorOffsets[jointType][colorIndex];
						

						err |= clSetKernelArg(jointPositionConstraintSolverKernel[jointType],a,sizeof(const unsigned int),&offset);	
						err |= clSetKernelArg(jointPositionConstraintSolverKernel[jointType],a+1,sizeof(const unsigned int),&length);
						//err|=clSetKernelArg(jointPositionConstraintSolverKernel[jointType],a++,sizeof(cl_mem),&testBuffer);
						if (err != CL_SUCCESS)
						{
							printf("Error: Failed to set joint position constraint solver arguments! %d\n", err); exit(1);
						}
            
						//execute the kernel
						int numBlocks=(length+maxWorkGroupSizeForJointPositionConstraintSolver[jointType]-1)/maxWorkGroupSizeForJointPositionConstraintSolver[jointType];
						size_t global=numBlocks*maxWorkGroupSizeForJointPositionConstraintSolver[jointType];
						if(colorIndex==0)
						{
							err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), jointPositionConstraintSolverKernel[jointType], 1, NULL, 
												&global, &maxWorkGroupSizeForJointPositionConstraintSolver[jointType], iter?1:0, 
												iter?&events[last]:NULL, &events[curr]);
							if (noqueue == false)
								clReleaseEvent(events[last]);
							curr = last;
							last = 1-last;
							if (err != CL_SUCCESS)
							{
								printf("Error: Failed to execute velocity constraint solver kernel!\n");
								exit(1);
							}
							if (noqueue) noqueue = false ; 
						}
						else 
						{
							err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), jointPositionConstraintSolverKernel[jointType], 1, NULL,
												&global, &maxWorkGroupSizeForJointPositionConstraintSolver[jointType], 1, 
												&events[last], &events[curr]);
 							clReleaseEvent(events[last]);
							curr = last;
							last = 1-last;
							if (err != CL_SUCCESS)
							{
								printf("Error: Failed to execute velocity constraint solver kernel!\n");
								exit(1);
							}
						}
				}// for (colorIndex)
		}// for (jointType)
	}// for (iter)

  if (noqueue == false )
	  clReleaseEvent(events[last]);
}

void b2CLSolver::b2CLCopyForTestKernel()
{
	clEnqueueCopyBuffer(b2CLDevice::instance().GetCommandQueue(), clb2PositionsBuffer, clb2TestPositionsBuffer, 0, 0, sizeof(float32) * 3 * m_bodyCount, 0, 0, 0);
}

void b2CLSolver::b2CLTestKernel(int numberOfIterations)
{
    if(m_contactCount<=0) return;
     
//typedef struct{
//    float cx;
//    float cy;
//    float a;
//}clb2Position;	
//	// for debug
//	clb2Position * testPosBefore = new clb2Position[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPosBefore, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	printf("\tInput of b2CLSolvePositionConstraint()\n");
//	for (int i=m_bodyCount-6; i<m_bodyCount; i++)
//	{
//		printf("Pos %d: c=(%f, %f), a=%f\n", i, testPosBefore[i].cx, testPosBefore[i].cy, testPosBefore[i].a);
//	}
//	delete [] testPosBefore;
//
//

    cl_event events[2];
    int curr=0, last=1;

    // set some of the arguments of kernel
    int a=0;
    int err=clSetKernelArg(testKernel,a++,sizeof(cl_mem),&clb2TestPositionsBuffer);
    err|=clSetKernelArg(testKernel,a++,sizeof(cl_mem),&clb2ManifoldsBuffer);
    err|=clSetKernelArg(testKernel,a++,sizeof(cl_mem),&clb2ContactsBuffer);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set position constraint solver arguments! %d\n", err);
        exit(1);
    }
    
    for(int iter=0;iter<numberOfIterations;iter++){ 
        for(int colorIndex=0; colorIndex<max_color; colorIndex++) {
            // set other arguments of kernel
            err=clSetKernelArg(testKernel,a,sizeof(const unsigned int),&colorOffsets[colorIndex]);
            err|=clSetKernelArg(testKernel,a+1,sizeof(const unsigned int),&colorLengths[colorIndex]);
            if (err != CL_SUCCESS)
            {
                printf("Error: Failed to set velocity constraint solver arguments! %d\n", err);
                exit(1);
            }
            
            //execute the kernel
            int numBlocks=(colorLengths[colorIndex]+maxWorkGroupSizeForTestKernel-1)/maxWorkGroupSizeForTestKernel;
            size_t global=numBlocks*maxWorkGroupSizeForTestKernel;
            if(colorIndex==0){
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),testKernel,1,NULL,&global,&maxWorkGroupSizeForTestKernel,iter?1:0,iter?&events[last]:NULL,&events[curr]);
                if(iter)
                {
                    clReleaseEvent(events[last]);
                }
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }else {
                err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(),testKernel,1,NULL,&global,&maxWorkGroupSizeForTestKernel,1,&events[last],&events[curr]);
                clReleaseEvent(events[last]);
                curr = last;
				last = 1-last;
                if (err != CL_SUCCESS)
                {
                    printf("Error: Failed to execute position constraint solver kernel!\n");
                    exit(1);
                }
            }
        }
    }
	clReleaseEvent(events[last]);
    //wait for commands to finish
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
	//clb2Position * testPos = new clb2Position[m_bodyCount];
	//b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
	//printf("\tResult of b2CLSolvePositionConstraint()\n");
	//for (int i=m_bodyCount-6; i<m_bodyCount; i++)
	//{
	//	printf("Pos %d: c=(%f, %f), a=%f\n", i, testPos[i].cx, testPos[i].cy, testPos[i].a);
	//}
	//delete [] testPos;
}

void b2CLSolver::b2CLSynchronizeXf()
{
    if(m_bodyCount<=0) return;

//typedef struct{
//    float cx;
//    float cy;
//    float a;
//}clb2Position;	// for debug
//	clb2Position * testPos = new clb2Position[m_bodyCount];
//	b2clBodyStatic * testBS = new b2clBodyStatic[m_bodyCount];
//	b2CLDevice::instance().copyArrayFromDevice(testPos, clb2PositionsBuffer, 0, sizeof(clb2Position)*m_bodyCount, true);
//	b2CLDevice::instance().copyArrayFromDevice(testBS, b2CLCommonData::instance().bodyStaticListBuffer, 0, sizeof(b2clBodyStatic)*m_bodyCount, true);
//	printf("\tInput of b2CLSynchronizeXf()\n");
//	for (int i=m_bodyCount-6; i<m_bodyCount; i++)
//	{
//		printf("Pos %d: c=(%f, %f), a=%f\n", i, testPos[i].cx, testPos[i].cy, testPos[i].a);
//		//printf("testBS %d: m_localCenter=(%f, %f)\n", i, testBS[i].m_localCenter[0], testBS[i].m_localCenter[1]);
//	}
//	delete [] testPos;
//	delete [] testBS;
    
    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(synchronizeXfKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().xfListBuffer));
    err|=clSetKernelArg(synchronizeXfKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
	err|=clSetKernelArg(synchronizeXfKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
	err|=clSetKernelArg(synchronizeXfKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyDynamicListBuffer));
    err|=clSetKernelArg(synchronizeXfKernel,a++,sizeof(int),&m_bodyCount);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set synchronize Xf arguments! %d\n", err);
        exit(1);
    }
    
    //execute the kernel
    int numBlocks=(m_bodyCount + maxWorkGroupSizeForSynchronizeXfKernel-1)/maxWorkGroupSizeForSynchronizeXfKernel;
    size_t global=numBlocks*maxWorkGroupSizeForSynchronizeXfKernel;
	err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), synchronizeXfKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForSynchronizeXfKernel, 0, NULL, NULL);
	if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute synchronize Xf kernel!\n");
        exit(1);
    }
    
	// redundant reported by APP Profiler, removed
    //clFinish(b2CLDevice::instance().GetCommandQueue());

	//// for debug
	//clb2Position * testPosAfter = new clb2Position[m_bodyCount];
	//b2clTransform * testXf = new b2clTransform[m_bodyCount];
	//b2CLDevice::instance().copyArrayFromDevice(testPosAfter, clb2PositionsBuffer, 0, sizeof(b2clBodyStatic)*m_bodyCount, true);
	//b2CLDevice::instance().copyArrayFromDevice(testXf, b2CLCommonData::instance().xfListBuffer, 0, sizeof(b2clTransform)*m_bodyCount, true);
	//printf("\tResult of b2CLSynchronizeXf()\n");
	//for (int i=m_bodyCount-6; i<m_bodyCount; i++)
	//{
	//	printf("PosAfter %d: c=(%f, %f), a=%f\n", i, testPosAfter[i].cx, testPosAfter[i].cy, testPosAfter[i].a);
	//	printf("testXf %d: p(%f, %f), q(%f, %f)\n", i, testXf[i].p[0], testXf[i].p[1], testXf[i].q[0], testXf[i].q[1]);
	//}
	//delete [] testPosAfter;
	//delete [] testXf;
}

void b2CLSolver::SortManifoldKeys()
{
	if (m_contactCount<=0) return;
	//if (m_contactCount>1024)
	//	int a = 0;

	//// for debug
	//int *keys = new int[sortCount];
	//for (int i=0; i<sortCount; i++)
	//	keys[i] = i;
	//b2CLDevice::instance().copyArrayToDevice(manifoldKeysBuffers[0], keys, 0, sizeof(unsigned int)*sortCount,true);
	//delete [] keys;
#ifdef _DEBUG
	unsigned int *keysBefore = new unsigned int[sortCount];
	unsigned int *valuesBefore = new unsigned int[sortCount];
	unsigned int *keysAfter = new unsigned int[sortCount];
	unsigned int *valuesAfter = new unsigned int[sortCount];
    b2CLDevice::instance().copyArrayFromDevice(keysBefore, manifoldKeysBuffer, 0, sizeof(unsigned int)*sortCount,true);
    b2CLDevice::instance().copyArrayFromDevice(valuesBefore, globalIndicesBuffer, 0, sizeof(unsigned int)*sortCount,true);
#endif

	//b2CLSort::instance().bitonicSort_NV(
	//	manifoldKeysBuffer,
	//	globalIndicesBuffer,
	//	manifoldKeysBuffer,
	//	globalIndicesBuffer,
	//	1, sortCount, 0);

#if defined(USE_CPU_SORT)
	b2CLSort::instance().stlSort(manifoldKeysBuffer, globalIndicesBuffer, m_contactCount, 0, 1);
#else
	b2CLSort::instance().bitonicSort_Intel(manifoldKeysBuffer, globalIndicesBuffer, sortCount, 0);
#endif

	lastContactCount = m_contactCount;

	//// for debug
	//lastContactCountAll = contactCountAll;

#ifdef _DEBUG
    b2CLDevice::instance().copyArrayFromDevice(keysAfter, manifoldKeysBuffer, 0, sizeof(unsigned int)*sortCount,true);
    b2CLDevice::instance().copyArrayFromDevice(valuesAfter, globalIndicesBuffer, 0, sizeof(unsigned int)*sortCount,true);
	delete [] keysBefore;
	delete [] valuesBefore;
	delete [] keysAfter;
	delete [] valuesAfter;
#endif
}

void b2CLSolver::InitializeSplitBufferParallel (bool bodyNumChanged){
	int maxContactNumPerBody = 20 ;
		if (!clb2SplitVelocitiesBuffer) {
			if (clb2SplitVelocitiesBuffer) b2CLDevice::instance().freeArray(clb2SplitVelocitiesBuffer);
			clb2SplitVelocitiesBuffer = b2CLDevice::instance().allocateArray ( sizeof(float) * 3 * m_bodyCount * maxContactNumPerBody ) ; 
			if (clb2NumContactEachBody) b2CLDevice::instance().freeArray(clb2NumContactEachBody);
	        clb2NumContactEachBody = b2CLDevice::instance().allocateArray (sizeof(unsigned int) * m_bodyCount);
		}
		if (m_contactCount == 0 ) return ; 


		unsigned int* allzero = new unsigned int [m_bodyCount];
		memset ( allzero, 0 , sizeof (unsigned int)* m_bodyCount ) ; 
		b2CLDevice::instance().copyArrayToDevice ( clb2NumContactEachBody, allzero, 0, sizeof (unsigned int)* m_bodyCount, true) ; 
		delete [] allzero;

       
		int a=0; int err = 0 ; unsigned int numContact = m_contactCount ; 
		//err |= clSetKernelArg (countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&this->clb2ContactsBuffer);
		err = clSetKernelArg (countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&this->clb2NumContactEachBody);
		err |= clSetKernelArg (countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&this->clb2Contact2BodySplitVelocitiesIndex);
		err |= clSetKernelArg (countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
		err |= clSetKernelArg (countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&this->coloredContactIndexToContactIndexMapBuffer);
		err|=clSetKernelArg(countContactNum4EachBodyKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().bodyStaticListBuffer));
 	    err |= clSetKernelArg(countContactNum4EachBodyKernel,  a++, sizeof(unsigned int), &numContact);
		
		int numBlocks = (numContact+ maxWorkGroupSizeForCountContactNum4EachBody-1)/maxWorkGroupSizeForCountContactNum4EachBody;
		size_t global = numBlocks*maxWorkGroupSizeForCountContactNum4EachBody; 

		err = clEnqueueNDRangeKernel (b2CLDevice::instance().GetCommandQueue(), countContactNum4EachBodyKernel, 1, NULL, 
			&global, &maxWorkGroupSizeForVelocityConstraintSolver, 0, NULL, NULL ); 
		if (err != CL_SUCCESS) {
			printf ("Error: Failed to set split velocity constraint solver arguments! %d\n", err);
			exit(1); 
		}
		
}

void b2CLSolver::InitializeSplitBufferSequential () {
	if (m_contactCount == 0 ) return ; 
	int maxContactNumPerBody = 20 ; 
	clb2ContactNode* arrayContacts = new clb2ContactNode [m_contactCount];   // ContactList copy back from GPU
	//float32* arrayBodySplitVelocity = new float32[3 * m_bodyCount * maxContactNumPerBody ] ; 
	unsigned int* arrayContact2BodyIndex = new unsigned int [m_contactCount * 2] ;   // For Each Contact's body, we need to know this contact's index in the body Split Velocity List. 
    unsigned int* arrayNumBodyContacts = new unsigned int [m_bodyCount] ;            // Total number of contact for each body
	memset ( arrayNumBodyContacts, 0 , sizeof (unsigned int)* m_bodyCount ) ; 
	

	b2CLDevice::instance().copyArrayFromDevice ( arrayContacts , this->clb2ContactsBuffer, 0 ,  sizeof(struct clb2ContactNode)*m_contactCount, true) ; 

	// for the first time, we copy the velocity from the velocitybuffer to the old buffer. 
	//if (!clb2OldVelocitiesBuffer) {
	//     clb2OldVelocitiesBuffer = b2CLDevice::instance().allocateArray(sizeof(float32) * 3 * m_bodyCount);
	//     b2CLDevice::instance().copyArrayInsideDevice ( this->clb2VelocitiesBuffer, this->clb2OldVelocitiesBuffer, sizeof(float32) * 3 * m_bodyCount ) ; 
	//}
/*
	clb2OldImpulsesBuffer = b2CLDevice::instance().allocateArray( sizeof(struct clb2ImpulseNode)* m_contactCount);
	b2CLDevice::instance().copyArrayInsideDevice ( clb2ImpulsesBuffer, this->clb2OldImpulsesBuffer, sizeof(struct clb2ImpulseNode) * m_contactCount ) ;
*/


	for (int i = 0 ; i < m_contactCount ; i ++ ) {
		clb2ContactNode* pContact = &(arrayContacts[i]) ; 
		int indexA = pContact->indexA ;
		int indexB = pContact->indexB ; 
		arrayContact2BodyIndex[i*2+0] = arrayNumBodyContacts[indexA] ; 
		arrayContact2BodyIndex[i*2+1] = arrayNumBodyContacts[indexB] ;
		if (pContact->invMassA != 0 )
			arrayNumBodyContacts[indexA] += 1 ; 
		if (arrayNumBodyContacts[indexA] >= maxContactNumPerBody ){
				printf ("The number of this Body's contacts is %d, it exceeds the number of maxContactNumPerBody %d \n", arrayNumBodyContacts[indexA], maxContactNumPerBody) ; 
				exit (1) ; 
		}
		if (pContact->invMassB != 0 )
			arrayNumBodyContacts[indexB] += 1 ;
		if (arrayNumBodyContacts[indexB] >= maxContactNumPerBody){
				printf ("The number of this Body's contacts is %d, it exceeds the number of maxContactNumPerBody %d \n", arrayNumBodyContacts[indexB], maxContactNumPerBody) ; 
				exit (1) ; 
		}
	}
	if (clb2SplitVelocitiesBuffer) b2CLDevice::instance().freeArray(clb2SplitVelocitiesBuffer);
	clb2SplitVelocitiesBuffer = b2CLDevice::instance().allocateArray ( sizeof(float) * 3 * m_bodyCount * maxContactNumPerBody ) ;  
	if (clb2Contact2BodySplitVelocitiesIndex) b2CLDevice::instance().freeArray(clb2Contact2BodySplitVelocitiesIndex);
	clb2Contact2BodySplitVelocitiesIndex = b2CLDevice::instance().allocateArray ( sizeof(unsigned int)*m_contactCount * 2 ) ; 
	if (clb2NumContactEachBody) b2CLDevice::instance().freeArray(clb2NumContactEachBody);
	clb2NumContactEachBody = b2CLDevice::instance().allocateArray (sizeof(unsigned int) * m_bodyCount);


	b2CLDevice::instance().copyArrayToDevice ( this->clb2Contact2BodySplitVelocitiesIndex, arrayContact2BodyIndex, 0 , sizeof(unsigned int)*m_contactCount*2, true ) ; 
	b2CLDevice::instance().copyArrayToDevice ( this->clb2NumContactEachBody, arrayNumBodyContacts, 0 , sizeof(unsigned int)*m_bodyCount, true ) ; 
	
	
	delete [] arrayContact2BodyIndex ;
	delete [] arrayNumBodyContacts ; 
	delete [] arrayContacts;
		
}

void b2CLSolver::b2CLStoreImpulses()
{
    if(m_contactCount<=0) return;

#ifdef _DEBUG
	//// for debug
	//int currentbuffer = b2CLCommonData::instance().currentManifoldBuffer;
	//unsigned int *temp = new unsigned int[m_contactCount];
 //   b2CLDevice::instance().copyArrayFromDevice(temp, coloredContactIndexToContactIndexMapBuffer, 0, sizeof(unsigned int)*m_contactCount,true);
	//delete [] temp;
#endif

	CreateBuffersForSortingManifolds();

typedef struct{
    float normalImpulse1;
    float tangentImpulse1;
    
    float normalImpulse2;
    float tangentImpulse2;
}clb2Impulse;

	//// for debug
	//clb2Impulse * testImpulse = new clb2Impulse[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2Impulse)*m_contactCount, true);
	//printf("\tInput of b2CLStoreImpulses()\n");
	//for (int i=m_contactCount-6; i<m_contactCount; i++)
	//{
	//	printf("Impulse %d: nI1=%f, tI1=%f, nI2=%f, tI2=%f\n", i, testImpulse[i].normalImpulse1, testImpulse[i].tangentImpulse1, testImpulse[i].normalImpulse2, testImpulse[i].tangentImpulse2);
	//}
	//delete [] testImpulse;
	////exit(1);

#ifdef BOX2D_OPENCL_ON_CPU
	// Initialize the buffer to 0
	// OCL-GPU will do it automatically
	unsigned int *zeroBuffer = new unsigned int[sortCount];
	memset(zeroBuffer, 0, sizeof(unsigned int)*sortCount);
	b2CLDevice::instance().copyArrayToDevice(manifoldKeysBuffer, zeroBuffer, 0, sizeof(unsigned int)*sortCount, true);
	delete [] zeroBuffer;
#endif

	// set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(storeImpulsesKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
    err|=clSetKernelArg(storeImpulsesKernel,a++, sizeof(cl_mem), &manifoldKeysBuffer);
    err|=clSetKernelArg(storeImpulsesKernel,a++, sizeof(cl_mem), &globalIndicesBuffer);
	err|=clSetKernelArg(storeImpulsesKernel,a++,sizeof(cl_mem),&coloredContactIndexToContactIndexMapBuffer);
	err|=clSetKernelArg(storeImpulsesKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
	err|=clSetKernelArg(storeImpulsesKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
    err|=clSetKernelArg(storeImpulsesKernel,a++,sizeof(const unsigned int),&m_contactCount);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
	//execute the kernel
    int numBlocks = (m_contactCount + maxWorkGroupSizeForStoreImpulsesKernel-1)/maxWorkGroupSizeForStoreImpulsesKernel;
    size_t global = numBlocks * maxWorkGroupSizeForStoreImpulsesKernel;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), storeImpulsesKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForStoreImpulsesKernel, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute read last impulses kernel!\n");
        exit(1);
    }
}

void b2CLSolver::b2CLReadLastImpulses(bool bWarmStarting, float dtRatio)
{
    if(m_contactCount<=0 || lastContactCount<=0) return;

	//printf("contact#: %d, last contact# %d\n", m_contactCount, lastContactCount);

	//// for debug
	//unsigned int *testGlobalIndices = new unsigned int[lastContactCount];
	//b2CLDevice::instance().copyArrayFromDevice(testGlobalIndices, globalIndicesBuffer, 0, sizeof(unsigned int)*lastContactCount,true);
	//for (int i=0; i<lastContactCount; i++)
	//{
	//	unsigned int index = testGlobalIndices[i];
	//	if (index>=lastContactCountAll)
 // 			int a = 0;
	//}
	//delete [] testGlobalIndices;
    
    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&manifoldKeysBuffer);
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&globalIndicesBuffer);
    err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(const unsigned int),&lastContactCount);
    err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(const unsigned int),&m_contactCount);
    
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&coloredContactIndexToContactIndexMapBuffer);
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[1-b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(int),&bWarmStarting);
	err|=clSetKernelArg(readLastImpulsesKernel,a++,sizeof(float),&dtRatio);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
#ifdef _DEBUG
	int currentbuffer = b2CLCommonData::instance().currentManifoldBuffer;
	unsigned int *keysBefore = new unsigned int[sortCount];
	unsigned int *valuesBefore = new unsigned int[sortCount];
	unsigned int *keysAfter = new unsigned int[sortCount];
	unsigned int *valuesAfter = new unsigned int[sortCount];
    b2CLDevice::instance().copyArrayFromDevice(keysAfter, manifoldKeysBuffer, 0, sizeof(unsigned int)*sortCount,true);
    b2CLDevice::instance().copyArrayFromDevice(valuesAfter, globalIndicesBuffer, 0, sizeof(unsigned int)*sortCount,true);
	delete [] keysBefore;
	delete [] valuesBefore;
	delete [] keysAfter;
	delete [] valuesAfter;
#endif
    
	//execute the kernel
    int numBlocks = (m_contactCount + maxWorkGroupSizeForReadLastImpulsesKernel-1)/maxWorkGroupSizeForReadLastImpulsesKernel;
    size_t global = numBlocks * maxWorkGroupSizeForReadLastImpulsesKernel;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), readLastImpulsesKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForReadLastImpulsesKernel, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute read last impulses kernel!\n");
        exit(1);
    }

#ifdef _DEBUG
	//b2clManifold *testManifold = new b2clManifold[m_allContactCount];
	//b2clManifold *testManifoldLast = new b2clManifold[m_allContactCount];
	//clb2ImpulseNode* testImpulse = new clb2ImpulseNode[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(testManifold, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 0, sizeof(b2clManifold)*m_allContactCount,true);
	//b2CLDevice::instance().copyArrayFromDevice(testManifoldLast, b2CLCommonData::instance().manifoldListBuffers[1-b2CLCommonData::instance().currentManifoldBuffer], 0, sizeof(b2clManifold)*m_allContactCount,true);
	//b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2ImpulseNode)*m_contactCount,true);
	//delete [] testManifold;
	//delete [] testManifoldLast;
	//delete [] testImpulse;
#endif

	// redundant reported by APP Profiler, removed 
	// (Debug mode will have problem if removed, need furether investigation)
//#ifdef _DEBUG
	clFinish(b2CLDevice::instance().GetCommandQueue());
//#endif
}

void b2CLSolver::b2CLReadLastImpulsesFirstFrame(bool bWarmStarting, float dtRatio)
{
    if(m_contactCount<=0 || lastContactCount<=0) return;

	//printf("contact#: %d, last contact# %d\n", m_contactCount, lastContactCount);

	//// for debug
	//unsigned int *testGlobalIndices = new unsigned int[lastContactCount];
	//b2CLDevice::instance().copyArrayFromDevice(testGlobalIndices, globalIndicesBuffer, 0, sizeof(unsigned int)*lastContactCount,true);
	//for (int i=0; i<lastContactCount; i++)
	//{
	//	unsigned int index = testGlobalIndices[i];
	//	if (index>=lastContactCountAll)
 // 			int a = 0;
	//}
	//delete [] testGlobalIndices;
    
    // set arguments of kernel
    int a=0;
    int err;
	err=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&clb2ImpulsesBuffer);
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&manifoldKeysBuffer);
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&globalIndicesBuffer);
    //err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&clb2PositionsBuffer);
    err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(const unsigned int),&lastContactCount);
    err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(const unsigned int),&m_contactCount);
    
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().globalIndicesBuffer));
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().fixtureStaticListBuffer));
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&coloredContactIndexToContactIndexMapBuffer);
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(cl_mem),&(b2CLCommonData::instance().manifoldListBuffers[1-b2CLCommonData::instance().currentManifoldBuffer]));
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(int),&bWarmStarting);
	err|=clSetKernelArg(readLastImpulsesFirstFrameKernel,a++,sizeof(float),&dtRatio);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to set initialize velocity constraint solver arguments! %d\n", err);
        exit(1);
    }
    
#ifdef _DEBUG
	//int currentbuffer = b2CLCommonData::instance().currentManifoldBuffer;
	//unsigned int *keysBefore = new unsigned int[sortCount];
	//unsigned int *valuesBefore = new unsigned int[sortCount];
	//unsigned int *keysAfter = new unsigned int[sortCount];
	//unsigned int *valuesAfter = new unsigned int[sortCount];
 //   b2CLDevice::instance().copyArrayFromDevice(keysAfter, manifoldKeysBuffer, 0, sizeof(unsigned int)*sortCount,true);
 //   b2CLDevice::instance().copyArrayFromDevice(valuesAfter, globalIndicesBuffer, 0, sizeof(unsigned int)*sortCount,true);
	//delete [] keysBefore;
	//delete [] valuesBefore;
	//delete [] keysAfter;
	//delete [] valuesAfter;
#endif
    
	//execute the kernel
    int numBlocks = (m_contactCount + maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel-1)/maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel;
    size_t global = numBlocks * maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel;
    err=clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), readLastImpulsesFirstFrameKernel, 1, NULL, 
                               &global, &maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: Failed to execute read last impulses kernel!\n");
        exit(1);
    }

#ifdef _DEBUG
	//b2clManifold *testManifold = new b2clManifold[m_contactCount];
	//b2clManifold *testManifoldLast = new b2clManifold[m_contactCount];
	//clb2ImpulseNode* testImpulse = new clb2ImpulseNode[m_contactCount];
	//b2CLDevice::instance().copyArrayFromDevice(testManifold, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 0, sizeof(b2clManifold)*m_contactCount,true);
	//b2CLDevice::instance().copyArrayFromDevice(testManifoldLast, b2CLCommonData::instance().manifoldListBuffers[1-b2CLCommonData::instance().currentManifoldBuffer], 0, sizeof(b2clManifold)*m_contactCount,true);
	//b2CLDevice::instance().copyArrayFromDevice(testImpulse, clb2ImpulsesBuffer, 0, sizeof(clb2ImpulseNode)*m_contactCount,true);
	//delete [] testManifold;
	//delete [] testManifoldLast;
	//delete [] testImpulse;
#endif

	// redundant reported by APP Profiler, removed 
	// (Debug mode will have problem if removed, need furether investigation)
//#ifdef _DEBUG
	clFinish(b2CLDevice::instance().GetCommandQueue());
//#endif
}

void b2CLSolver::Report(b2ContactManager *m_pContactManager)
{
	if (m_pContactManager->m_contactListener == NULL)
		return;

	int contactCount = m_pContactManager->m_contactCount;
	contactCount = m_contactCount;
	if (contactCount<=0)
		return;

	// Read back data from GPU again, just for the updated impulses
	// (May be able to optimise later!!!)
	b2clManifold *manifoldListData = b2CLCommonData::instance().manifoldListData;
	b2CLDevice::instance().copyArrayFromDevice(manifoldListData, b2CLCommonData::instance().manifoldListBuffers[b2CLCommonData::instance().currentManifoldBuffer], 
		0, sizeof(b2clManifold) * contactCount);
	int *globalIndices = b2CLCommonData::instance().globalIndices;
	b2CLDevice::instance().copyArrayFromDevice(globalIndices, b2CLCommonData::instance().globalIndicesBuffer, 0, sizeof(int)*m_totalContactCount*4, true);

	int *validContactIndices = new int[contactCount];
	b2CLDevice::instance().copyArrayFromDevice(validContactIndices, b2CLCommonData::instance().validContactIndicesBuffer, 0, sizeof(int)*contactCount, true);

	b2Contact* pc = m_pContactManager->m_contactList;

	// Mimic b2ContactManager::AddPair (which is called in b2BroadPhase::UpdatePairs)
	// to find same contact and fill impulses in
	int fixtureIndexA, fixtureIndexB;
	for (int i=0; i<contactCount; i++)
	{
		//if (i==5)
		//	int abc = 1;

		int globalIndex = validContactIndices[i];
		if (manifoldListData[globalIndex].pointCount==0)
			continue;

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
			// fill impulses in
			for (int k=0; k<edge->contact->m_manifold.pointCount; k++)
			{
				edge->contact->m_manifold.points[k].normalImpulse = manifoldListData[i].points[k].normalImpulse;
				edge->contact->m_manifold.points[k].tangentImpulse = manifoldListData[i].points[k].tangentImpulse;
			}
		}
		else
		{
			//assert(0); // we should always found the contact because it is just inserted in the contact list in b2CLNarrowPhase::ReadbackGPUDataForListener
		}
	}

	delete [] validContactIndices;

	int num = 0;
	while (pc)
	{
		b2ContactImpulse impulse;
		impulse.count = pc->m_manifold.pointCount;
		for (int32 j = 0; j < impulse.count; ++j)
		{
			impulse.normalImpulses[j] = pc->m_manifold.points[j].normalImpulse;
			impulse.tangentImpulses[j] = pc->m_manifold.points[j].tangentImpulse;
		}

		m_pContactManager->m_contactListener->PostSolve(pc, &impulse);

		num++;
		pc = pc->GetNext();
	}
}