/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLContactSolver_h
#define Box2D_b2CLContactSolver_h

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Dynamics/b2ContactManager.h>

#ifdef _DEBUG_TIME_SOLVER
#include <Box2D/Common/b2Timer.h>
#endif

struct b2ContactVelocityConstraint;
struct b2ContactPositionConstraint;
struct b2Velocity;
class b2Body;
struct b2Position;

class b2CLSolver
{
	friend class b2CLSolveTOI;

public:
    b2CLSolver();
    ~b2CLSolver();    

    void b2CLInitializeSolver(b2Body** m_bodiesInput, b2Contact** m_contactsInput,
		b2Velocity* m_velocitiesInput, b2Position* m_positionsInput,
		int32 m_bodyCountInput, int32 m_contactCountInput, b2Vec2 m_gravityInput, float32 m_dtInput,
		const b2ContactManager& contactManager, b2Contact** m_islandContacts,
        b2ContactVelocityConstraint* m_velocitiesConstraintsInput=0, b2ContactPositionConstraint* m_positionConstraintsInput=0);
    void b2CLSolveVelocityConstraint(int numberOfIterations);
	void b2CLSolveJointVelocityConstraint (int numberOfIterations); 
    void b2CLSolvePositionConstraint(int numberOfIterations);
	void b2CLSolvePositionConstraint_MergeSplittedMass(int numberOfIterations);
	void b2CLSolveSplitPositionConstraint(int numberOfIterations);
	void b2CLSolveJointPositionConstraint (int numberOfIterations); 
	void b2CLCopyForTestKernel();
    void b2CLTestKernel(int numberOfIterations);
    void b2CLIntegratePositions();
	void b2CLInitializeVelocityConstraint_HasSplit(bool bWarmStarting, float dtRatio);
	void b2CLInitializeVelocityConstraint(bool bWarmStarting, float dtRatio);
	void b2CLInitializeJointVelocityConstraint ( bool bWarmStarting, float dt ) ; 
	void b2CLReadCompactedContacts(int& contactCount);
	void b2CLStoreImpulses();
	void b2CLInitializeBodyState();
	void b2CLInitializeBodyStateFirstFrame();
	void b2CLWarmStart();
	void b2CLWarmStartWithColoring();
	void b2CLWarmStartSplit() ; 
	void b2CLTestWarmStart() ; 
	void b2CLSynchronizeXf();
    void CopyResultsFromGPUForBodies();
    void CopyResultsFromGPUForContacts();
    void StoreImpulses();
	void SortManifoldKeys();
	void b2CLReadLastImpulses(bool bWarmStarting, float dtRatio);
	void b2CLReadLastImpulsesFirstFrame(bool bWarmStarting, float dtRatio);
	void Report(b2ContactManager *m_pContactManager);

    void SetValues(int bodyCount, int contactCount, b2Vec2 gravity, float dt, float dtRatio);
    void CreateBuffersForBodies();
    void CreateBuffersForContacts();
	void CreateBuffersForSortingManifolds();
	int GetLastContactCount() { return lastContactCount; }

	void InitializeSplitBufferSequential () ;
	void InitializeSplitBufferParallel (bool bodyNumChanged);
	void b2CLSolveSplitVelocityConstraint (int numberOfIterations); 

	

		 // for StaticDynamic Pairs
	void collectStaticDynamicPair (clb2SDManifold* sdManifoldArray, clb2SDContact* contactArray, int mfSize, clb2SDBody* sdBodyArray, int bdSize) ; 
	void solveSDPosition(int numIteration, int sdNum, unsigned int* colorOffSets, unsigned int* colorLengths, int max_color);
	void solveSDVelocity(int numIteration, int sdNum, unsigned int* colorOffSets, unsigned int* colorLengths, int max_color); 
	void syncSDPosition ( clb2SDBody* sdBodyArray, int bdSize ); 
	void solveStaticDynamicPair () ; 


	int32 m_validContactCount;
	int* validContactIndices;
	int* globalIndices;
	float colorTime ; 

private:



    cl_program velocityConstraintSolverProgram,
        positionConstraintSolverProgram;
    cl_kernel velocityConstraintSolverKernel, 
		jointVelocityConstraintSolverKernel[11],
		splitVelocityConstraintSolverKernel,
		countContactNum4EachBodyKernel, 
		mergeVelocityConstraintSolverKernel, 
		updateImpulseConstraintSolverKernel,
		velocityConstraintInitializeKernel_HasSplit, 
		velocityConstraintInitializeKernel,
		jointVelocityConstraintInitializeKernel[11],

		bodyStateInitializeKernel,
		bodyStateInitializeFirstFrameKernel,
		warmStartKernel,
		warmStartWithColoringKernel,
		warmStartSplitKernel,
		synchronizeXfKernel,
		readLastImpulsesKernel,
		readLastImpulsesFirstFrameKernel,
		storeImpulsesKernel,
        integratePositionKernel,
        positionConstraintSolverKernel,
		splitPositionConstraintSolverKernel, 
		positionConstraintSolverKernel_MergeSplittedMass,
		jointPositionConstraintSolverKernel[11],
	    testKernel,
	    testWarmstartKernel; 

    
    size_t maxWorkGroupSizeForVelocityConstraintSolver, 
		maxWorkGroupSizeForJointVelocityConstraintSolver[11],
		maxWorkGroupSizeForVelocityConstraintInitialize, 
		maxWorkGroupSizeForCountContactNum4EachBody, 
		maxWorkGroupSizeForBodyStateInitializeKernel,
		maxWorkGroupSizeForBodyStateInitializeFirstFrameKernel,
		maxWorkGroupSizeForWarmStartKernel,
		maxWorkGroupSizeForWarmStartSplitKernel,
		maxWorkGroupSizeForWarmStartWithColoringKernel,
		maxWorkGroupSizeForSynchronizeXfKernel,
		maxWorkGroupSizeForReadLastImpulsesKernel,
		maxWorkGroupSizeForReadLastImpulsesFirstFrameKernel,
		maxWorkGroupSizeForStoreImpulsesKernel,
        maxWorkGroupSizeForIntegratingPositions,
        maxWorkGroupSizeForPositionConstraintSolver,
		maxWorkGroupSizeForJointPositionConstraintSolver[11],
	    maxWorkGroupSizeForTestKernel;
    
    cl_mem clb2VelocitiesBuffer;
	cl_mem clb2TestVelocitiesBuffer;
	cl_mem clb2PositionsBuffer;

	// for SPLIT Velocity Buffer
	cl_mem clb2NumContactEachBody; 
	cl_mem clb2Contact2BodySplitVelocitiesIndex ; 
	cl_mem clb2SplitVelocitiesBuffer ;
	cl_mem clb2HasSplitVelocitiesContactsBuffer ; 
	cl_mem clb2OldVelocitiesBuffer ; 
	cl_mem clb2OldImpulsesBuffer ; 
	
	// for StaticDynamic Pairs
	cl_kernel collectStaticDynamicPairKernel;  
	cl_kernel solveSDVelocityKernel ; 
	cl_kernel solveSDPositionKernel ; 
	cl_kernel initSDBodyKernel ; 
	cl_kernel syncSDBodyKernel ; 
	cl_mem sdManifoldDataBuffer;
	cl_mem sdBodyDataBuffer ; 
	cl_mem sdContactDataBuffer ;
	cl_mem sdManifoldBuffer;
	cl_mem sdContactBuffer; 
	cl_mem sdPointBuffer ; 
	cl_mem sdImpulseBuffer ; 
	int sdIndexBufferSize ; 
	int sdIndexNum ; 



	// for debug
	cl_mem clb2TestPositionsBuffer;

    cl_mem clb2ContactsBuffer;
    cl_mem clb2ImpulsesBuffer;
    cl_mem clb2PointsBuffer;
    cl_mem clb2ManifoldsBuffer;

	cl_mem coloredContactIndexToContactIndexMapBuffer;

	cl_mem manifoldKeysBuffer;
	cl_mem globalIndicesBuffer;
	int sortCount, oldSortCount;
	int lastContactCount;

	// for debug
	int contactCountAll, lastContactCountAll;

	//int32 *islandManifoldIndicesData;
	//cl_mem islandManifoldIndicesBuffer;
    
    b2Contact** m_contacts;
    b2Body** m_bodies;
    b2Velocity* m_velocities;
    b2Position* m_positions;
    b2ContactVelocityConstraint* m_velocitiesConstraints;
    b2ContactPositionConstraint* m_positionsConstraints;
    clb2ContactNode* clb2ContactsList;
    clb2PointsNode* clb2PointsList;
    clb2ImpulseNode* clb2ImpulsesList;
    clb2Manifold* clb2ManifoldsList;
    
    int max_color;
    int* contactIndexToColoredContactIndexMap;
    int* coloredContactIndexToContactIndexMap;
    unsigned int* colorOffsets;
    unsigned int* colorLengths;
    
    int32 m_bodyCount;
    int32 m_contactCount, m_totalContactCount;
	int32 old_body_num;
	int32 old_contact_num;
	int32 old_valid_contact_num, old_global_contact_num;
	float m_gravity[2];
	float m_dt;
	float m_dtRatio;
    
    void ComputeColoringOfContacts(b2Contact** m_islandContacts=NULL);
    void CopyDataToGPUForBodies();
    void CopyDataToGPUForContacts();
    
    ////////////////////////////////////PROFILE INFO
    static float32 totalTimeForComputingColoring;
    static int totalCountForComputingColoring;
    static float32 totalTimeForSolvingVelocityConstraint;
    static int totalCountForSolvingVelocityConstraint;
    static float32 totalTimeForCPUGPUCommunication;
    static int totalCountForCPUGPUCommunication;
    ////////////////////////////////////////////////

	
};

#endif
