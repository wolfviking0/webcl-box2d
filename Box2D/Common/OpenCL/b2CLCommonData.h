/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


#ifndef Box2D_b2CLCommonData_h
#define Box2D_b2CLCommonData_h

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

#ifdef _DEBUG_TIME_NARROWPHASE
#include <Box2D/Common/b2Timer.h>
#endif

class b2World;
class b2Fixture;
class b2Contact;
class b2PolygonShape;
class b2ContactListener;

class b2Joint;
class b2RevoluteJoint;
class b2PrismaticJoint;
class b2DistanceJoint; 
class b2GearJoint;
class b2PulleyJoint; 
class b2RopeJoint; 
class b2WheelJoint;
class b2WeldJoint;
class b2MouseJoint;
class b2FrictionJoint; 

const int maxContactNumPerBody = 20;
const int numJointTypes = 11;
const int32 INVALID_JOINT_ID = 0;

class b2CLCommonData
{
	friend class b2CLBroadPhase;
	friend class b2CLNarrowPhase;
	friend class b2CLSolver;
	friend class b2ContactManager;
	friend class b2CLSolveTOI;

public: 
    b2CLCommonData();
    ~b2CLCommonData();
	static b2CLCommonData& instance();
    
	void CopyShapes(b2World *m_pWorld);
	void CopyStaticFixtureAttributes(b2World *m_pWorld);
	void CopyStaticBodyAttributes(b2World *m_pWorld);
	void CopyDynamicBodyAttributes(b2World *m_pWorld);
	void UpdateFixtureAddressAndChildren(b2World *m_pWorld);

	/// for joint copy
	void DeleteJoints();
	void CopyJoints(b2World *m_pWorld, bool warmStarting, bool isJointChanged, bool isJointUpdated); 
	int32 GetNumJoints() const { return numTotalJoints; }

	void JointAdded(const b2Joint* pJoint);
	void JointRemoved(const b2Joint* pJoint);

	static void CopyJointCommon(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyDistanceJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyRevoluteJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyPrismaticJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyGearJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyPulleyJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyRopeJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyFrictionJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyWheelJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyWeldJoint(const b2Joint* pSrc, b2clJoint* pDest); 
	static void CopyMouseJoint(const b2Joint* pSrc, b2clJoint* pDest); 

	b2Fixture * GetFixture(int id) { return fixture_address[id]; }

private:
	void initReadLastJointImpulses();
	void ComputeJointColors(b2World *m_pWorld); 
	void ReadLastJointImpulses();
	void StoreJointImpulses();

	int old_shape_num, old_fixture_num, old_body_num;
    
	b2clPolygonShape *shapeListData;
	int *shapeToBodyMapData;
	b2clFixtureStatic *fixtureStaticListData;
	b2clBodyStatic *bodyStaticListData;
	b2clTransform *xfListData;
	b2clBodyDynamic *bodyDynamicListData;

	cl_mem manifoldListBuffers[2]; // ping-pong buffers for last and current manifold lists
	int currentManifoldBuffer;
	cl_mem manifoldBinaryBitListBuffer; // binary array for current manifold, 1-valid / 0-invalid
	cl_mem validContactIndicesBuffer;
	cl_mem shapeListBuffer;
	cl_mem shapeToBodyMapBuffer;
	cl_mem xfListBuffer;
	cl_mem fixtureStaticListBuffer;
	cl_mem bodyStaticListBuffer;
	cl_mem bodyDynamicListBuffer;

	cl_mem globalIndicesBuffer;
	cl_mem pairIndicesBuffer;
	cl_mem pairIndicesBinaryBitsBuffer;
	cl_mem pairCountsBuffer;
	cl_mem pairTotalCountBuffer;

	int32 numJoints[numJointTypes];
	int32 numTotalJoints;
	b2clJoint* jointListData;
	int32 jointColorOffsets[numJointTypes][maxContactNumPerBody];
	int32 jointMaxColor[numJointTypes];
	cl_mem jointListBuffer;
	void (*copyJointFunc[numJointTypes])(const b2Joint* pSrc, b2clJoint* pDest);

	cl_mem jointImpulsesBuffer;
	int lastNumTotalJoints;

	static int32 nextJointUniqueIndex;
	std::map<const b2Joint*, int32> jointPointerToUniqueIndexMap;

	int32 lastSortCount;
	cl_mem jointImpulseKeysBuffer;
	cl_mem jointImpulseGlobalIndicesBuffer;

	// for Listener
	b2clManifold *manifoldListData;
	b2Fixture **fixture_address;
	int *fixture_child, old_proxy_count;
	int *globalIndices;

	cl_program commonDataProgram;
    cl_kernel readLastJointImpulseKernel;
    size_t maxWorkGroupSizeForreadLastJointImpulse;

	// QueryAABB, RayCast
	cl_mem intersectionCounts;
	cl_mem intersectionTotalCount;
	cl_mem intersectingShapeIndices;
	cl_mem intersectingShapeTypes;
	int32 oldIntersectionBufferSize;
	unsigned int* intersectingShapeIndicesData;

	cl_mem rayCastOutputBuffer;
	b2clRayCastOutput* rayCastOutputListData;
	int32 oldRayCastOutBufferSize;

	//cl_mem movedBodyIndexBuffer; 

};
#endif
