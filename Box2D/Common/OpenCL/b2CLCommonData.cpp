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
#include <Box2D/Common/OpenCL/b2CLCommonData.h>
#include <Box2D/Common/OpenCL/b2CLSort.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>
#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/Joints/b2DistanceJoint.h>
#include <Box2D/Dynamics/Joints/b2GearJoint.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Joints/b2RopeJoint.h>
#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/Joints/b2WeldJoint.h>
#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>

int32 b2CLCommonData::nextJointUniqueIndex = 1; // 0 is used for empty elements in sort buffer

b2CLCommonData::b2CLCommonData()
{
	old_shape_num = old_fixture_num = old_body_num = 0;
	shapeListData = NULL;
	shapeToBodyMapData = NULL;
	fixtureStaticListData = NULL;
	bodyStaticListData = NULL;
	xfListData = NULL;
	bodyDynamicListData = NULL;

	// joint data
	numTotalJoints = 0;
	jointListData = NULL;
	jointListBuffer = NULL;
	for (int32 i = 0; i < numJointTypes; ++i)
	{
		numJoints[i] = 0;
		jointMaxColor[i] = 0;
		copyJointFunc[i] = NULL;
	}
	copyJointFunc[e_revoluteJoint] = b2CLCommonData::CopyRevoluteJoint;
	copyJointFunc[e_prismaticJoint] = b2CLCommonData::CopyPrismaticJoint;
	copyJointFunc[e_distanceJoint] = b2CLCommonData::CopyDistanceJoint;
	copyJointFunc[e_pulleyJoint] = b2CLCommonData::CopyPulleyJoint;
	copyJointFunc[e_mouseJoint] = b2CLCommonData::CopyMouseJoint;
	copyJointFunc[e_gearJoint] = b2CLCommonData::CopyGearJoint;
	copyJointFunc[e_wheelJoint] = b2CLCommonData::CopyWheelJoint;
	copyJointFunc[e_weldJoint] = b2CLCommonData::CopyWeldJoint;
	copyJointFunc[e_frictionJoint] = b2CLCommonData::CopyFrictionJoint;
	copyJointFunc[e_ropeJoint] = b2CLCommonData::CopyRopeJoint;

	jointImpulsesBuffer = NULL;
	lastNumTotalJoints = 0;
	jointImpulseKeysBuffer = NULL;
	jointImpulseGlobalIndicesBuffer = NULL;
	lastSortCount = 0;
	oldIntersectionBufferSize = 0;
	oldRayCastOutBufferSize = 0;

	manifoldListBuffers[0] = manifoldListBuffers[1] = 
		manifoldBinaryBitListBuffer = 
		validContactIndicesBuffer = 
		shapeListBuffer = 
		shapeToBodyMapBuffer = 
		xfListBuffer =
		fixtureStaticListBuffer =
		bodyStaticListBuffer =
		bodyDynamicListBuffer = NULL;
	    
	// AABBQuery, RayCast callback
	intersectionCounts = NULL;
	intersectionTotalCount = NULL;
	intersectingShapeIndices = NULL;
	intersectingShapeTypes = NULL;
	intersectingShapeIndicesData = NULL;
	rayCastOutputBuffer = NULL;
	rayCastOutputListData = NULL;

	fixture_address = NULL;
	fixture_child = NULL;
	old_proxy_count = 0;
	globalIndices = NULL;

#if defined(BROADPHASE_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		globalIndicesBuffer = pairIndicesBuffer = pairIndicesBinaryBitsBuffer = NULL;
		pairCountsBuffer = b2CLDevice::instance().allocateArray(sizeof(int)* b2Shape::contact_type_num);
		pairTotalCountBuffer = b2CLDevice::instance().allocateArray(sizeof(int));

		intersectionCounts = b2CLDevice::instance().allocateArray(sizeof(int) * b2Shape::e_typeCount);
		intersectionTotalCount = b2CLDevice::instance().allocateArray(sizeof(int));
	}
#endif

	initReadLastJointImpulses();
}

b2CLCommonData::~b2CLCommonData()
{
	DeleteJoints();

	if (jointImpulsesBuffer)
		b2CLDevice::instance().freeArray(jointImpulsesBuffer);

	if (jointImpulseKeysBuffer)
		b2CLDevice::instance().freeArray(jointImpulseKeysBuffer);

	if (jointImpulseGlobalIndicesBuffer)
		b2CLDevice::instance().freeArray(jointImpulseGlobalIndicesBuffer);

	if (intersectionCounts)
		b2CLDevice::instance().freeArray(intersectionCounts);

	if (intersectionTotalCount)
		b2CLDevice::instance().freeArray(intersectionTotalCount);

	if (intersectingShapeIndices)
		b2CLDevice::instance().freeArray(intersectingShapeIndices);

	if (intersectingShapeTypes)
		b2CLDevice::instance().freeArray(intersectingShapeTypes);

	if (rayCastOutputBuffer)
		b2CLDevice::instance().freeArray(rayCastOutputBuffer);

	delete[] intersectingShapeIndicesData;
	delete[] rayCastOutputListData;

}

b2CLCommonData& b2CLCommonData::instance()
{
	static b2CLCommonData inst;
	return inst;
}

void b2CLCommonData::CopyShapes(b2World *m_pWorld)
{
	// Initialize data on CPU
	int body_index, shape_index;
	
	if (old_shape_num < m_pWorld->m_proxyCount)
	{
		if (shapeListData)
			delete [] shapeListData;
		shapeListData = new b2clPolygonShape[m_pWorld->m_proxyCount];

		if (shapeListBuffer)
			b2CLDevice::instance().freeArray(shapeListBuffer);
		shapeListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clPolygonShape) * m_pWorld->m_proxyCount);

		if (shapeToBodyMapData)
			delete [] shapeToBodyMapData;
		shapeToBodyMapData = new int[m_pWorld->m_proxyCount];

		if (shapeToBodyMapBuffer)
			b2CLDevice::instance().freeArray(shapeToBodyMapBuffer);
		shapeToBodyMapBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * m_pWorld->m_proxyCount);

#if defined(BROADPHASE_OPENCL)
		if (b2clGlobal_OpenCLSupported)
		{
			//int contactCount = m_pWorld->m_proxyCount * m_pWorld->m_proxyCount; // most conservative estimation, can be much less
			int maxContactCount = m_pWorld->m_proxyCount * MAX_CONTACT_PER_FIXTURE; // may not be enough!!!

			if (globalIndicesBuffer)
				b2CLDevice::instance().freeArray(globalIndicesBuffer);
			globalIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * 4 * maxContactCount);

			if (pairIndicesBuffer)
				b2CLDevice::instance().freeArray(pairIndicesBuffer);
			pairIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * maxContactCount * b2Shape::contact_type_num);
			if (pairIndicesBinaryBitsBuffer)
				b2CLDevice::instance().freeArray(pairIndicesBinaryBitsBuffer);
			pairIndicesBinaryBitsBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * maxContactCount * b2Shape::contact_type_num);
		}
#endif

		old_shape_num = m_pWorld->m_proxyCount;
		//cout << "New shape list" << endl;
	}

	body_index = shape_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		for (b2Fixture* f = b->m_fixtureList; f; f = f->m_next)
		{
			b2Shape *s = (f->m_shape);

			shapeListData[shape_index].m_filter.categoryBits = f->m_filter.categoryBits;
			shapeListData[shape_index].m_filter.groupIndex = f->m_filter.groupIndex;
			shapeListData[shape_index].m_filter.maskBits = f->m_filter.maskBits;

			switch (s->m_type)
			{
			case b2Shape::e_circle:
				{
					b2CircleShape *sc = (b2CircleShape*)s;

					shapeListData[shape_index].m_centroid[0] = sc->m_p.x;
					shapeListData[shape_index].m_centroid[1] = sc->m_p.y;
					shapeListData[shape_index].m_vertices[0][0] = sc->m_p.x;
					shapeListData[shape_index].m_vertices[0][1] = sc->m_p.y;
					shapeListData[shape_index].m_vertexCount = 1;

					shapeListData[shape_index].m_type = s->m_type;
					shapeListData[shape_index].m_radius = s->m_radius;
					shapeListData[shape_index].m_bIsSensor = f->IsSensor();

					shapeToBodyMapData[shape_index] = body_index;

					shape_index++;

					break;
				}
			case b2Shape::e_edge:
				{
					b2EdgeShape *se = (b2EdgeShape*)s;

					shapeListData[shape_index].m_centroid[0] = se->m_hasVertex0;
					shapeListData[shape_index].m_centroid[1] = se->m_hasVertex3;
					shapeListData[shape_index].m_vertices[0][0] = se->m_vertex1.x;
					shapeListData[shape_index].m_vertices[0][1] = se->m_vertex1.y;
					shapeListData[shape_index].m_vertices[1][0] = se->m_vertex2.x;
					shapeListData[shape_index].m_vertices[1][1] = se->m_vertex2.y;
					if (se->m_hasVertex0)
					{
						shapeListData[shape_index].m_vertices[2][0] = se->m_vertex0.x;
						shapeListData[shape_index].m_vertices[2][1] = se->m_vertex0.y;
					}
					if (se->m_hasVertex3)
					{
						shapeListData[shape_index].m_vertices[3][0] = se->m_vertex3.x;
						shapeListData[shape_index].m_vertices[3][1] = se->m_vertex3.y;
					}

					shapeListData[shape_index].m_type = s->m_type;
					shapeListData[shape_index].m_radius = s->m_radius;
					shapeListData[shape_index].m_bIsSensor = f->IsSensor();

					shapeToBodyMapData[shape_index] = body_index;

					shape_index++;

					break;
				}
			case b2Shape::e_polygon:
				{
					b2PolygonShape *sp = (b2PolygonShape*)s;

					shapeListData[shape_index].m_centroid[0] = sp->m_centroid.x;
					shapeListData[shape_index].m_centroid[1] = sp->m_centroid.y;
					for (int i=0; i<sp->m_vertexCount; i++)
					{
						shapeListData[shape_index].m_vertices[i][0] = sp->m_vertices[i].x;
						shapeListData[shape_index].m_vertices[i][1] = sp->m_vertices[i].y;
						shapeListData[shape_index].m_normals[i][0] = sp->m_normals[i].x;
						shapeListData[shape_index].m_normals[i][1] = sp->m_normals[i].y;
					}
					shapeListData[shape_index].m_vertexCount = sp->m_vertexCount;

					shapeListData[shape_index].m_type = s->m_type;
					shapeListData[shape_index].m_radius = s->m_radius;
					shapeListData[shape_index].m_bIsSensor = f->IsSensor();

					shapeToBodyMapData[shape_index] = body_index;

					shape_index++;

					break;
				}
			case b2Shape::e_chain:
				{
					b2ChainShape *sc = (b2ChainShape*)s;
					b2EdgeShape *se = new b2EdgeShape();

					for (int i=0; i<sc->GetChildCount(); i++)
					{
						sc->GetChildEdge(se, i);

						shapeListData[shape_index].m_centroid[0] = se->m_hasVertex0;
						shapeListData[shape_index].m_centroid[1] = se->m_hasVertex3;
						shapeListData[shape_index].m_vertices[0][0] = se->m_vertex1.x;
						shapeListData[shape_index].m_vertices[0][1] = se->m_vertex1.y;
						shapeListData[shape_index].m_vertices[1][0] = se->m_vertex2.x;
						shapeListData[shape_index].m_vertices[1][1] = se->m_vertex2.y;
						if (se->m_hasVertex0)
						{
							shapeListData[shape_index].m_vertices[2][0] = se->m_vertex0.x;
							shapeListData[shape_index].m_vertices[2][1] = se->m_vertex0.y;
						}
						if (se->m_hasVertex3)
						{
							shapeListData[shape_index].m_vertices[3][0] = se->m_vertex3.x;
							shapeListData[shape_index].m_vertices[3][1] = se->m_vertex3.y;
						}

						shapeListData[shape_index].m_type = b2Shape::e_edge;
						shapeListData[shape_index].m_radius = s->m_radius;
						shapeListData[shape_index].m_bIsSensor = f->IsSensor();

						shapeListData[shape_index].m_filter.categoryBits = f->m_filter.categoryBits;
						shapeListData[shape_index].m_filter.groupIndex = f->m_filter.groupIndex;
						shapeListData[shape_index].m_filter.maskBits = f->m_filter.maskBits;

						shapeToBodyMapData[shape_index] = body_index;

						shape_index++;
					}
					break;
				}
			}
		}
		body_index++;
	}
	//// for debug
	//assert(m_pWorld->m_fixtureCount == shape_index);
	//printf("#Shapes: %d, #Transforms: %d\n", shape_index, xf_index);

	// Copy data from CPU to GPU
	b2CLDevice::instance().copyArrayToDevice(shapeListBuffer, shapeListData, 0, sizeof(b2clPolygonShape) * m_pWorld->m_proxyCount);
	b2CLDevice::instance().copyArrayToDevice(shapeToBodyMapBuffer, shapeToBodyMapData, 0, sizeof(int) * m_pWorld->m_proxyCount);
}

void b2CLCommonData::CopyStaticFixtureAttributes(b2World *m_pWorld)
{
	// Initialize data on CPU
	int fixture_index;
	
	if (old_fixture_num < m_pWorld->m_proxyCount)
	{
		if (fixtureStaticListData)
			delete [] fixtureStaticListData;
		fixtureStaticListData = new b2clFixtureStatic[m_pWorld->m_proxyCount];

		if (fixtureStaticListBuffer)
			b2CLDevice::instance().freeArray(fixtureStaticListBuffer);
		fixtureStaticListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clFixtureStatic) * m_pWorld->m_proxyCount);

		old_fixture_num = m_pWorld->m_proxyCount;
		//cout << "New shape list" << endl;
	}

	fixture_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		for (b2Fixture* f = b->m_fixtureList; f; f = f->m_next)
		{
			b2PolygonShape *s = (b2PolygonShape*)(f->m_shape);

			for (int i=0; i<s->GetChildCount(); i++)
			{
				b2FixtureProxy* p = f->m_proxies + i;

				p->m_last_uid = p->m_uid;
				f->m_last_uid = f->m_uid;

				fixtureStaticListData[fixture_index].m_friction = f->m_friction;
				fixtureStaticListData[fixture_index].m_restitution = f->m_restitution;
				fixtureStaticListData[fixture_index].m_last_uid = p->m_last_uid;

				p->m_uid = fixture_index; // This fixture_index should in fact be called proxy_index. Maybe change it later.
				f->m_uid = fixture_index;

				fixture_index++;
			}
		}
	}
	assert(fixture_index<=m_pWorld->m_proxyCount);

	// Copy data from CPU to GPU
	b2CLDevice::instance().copyArrayToDevice(fixtureStaticListBuffer, fixtureStaticListData, 0, sizeof(b2clFixtureStatic) * m_pWorld->m_proxyCount);
}

// Data structure for jointToBodyMap.
// A joint can be connected up to 4 bodies.
struct Quadruple
{
	Quadruple() { x[0] = x[1] = x[2] = x[3] = -1; }
	int x[4];
};

void b2CLCommonData::CopyStaticBodyAttributes(b2World *m_pWorld)
{
	// Initialize data on CPU
	int body_index;
	
	if (old_body_num < m_pWorld->m_bodyCount)
	{
		if (bodyStaticListData)
			delete [] bodyStaticListData;
		bodyStaticListData = new b2clBodyStatic[m_pWorld->m_bodyCount];

		if (bodyStaticListBuffer)
			b2CLDevice::instance().freeArray(bodyStaticListBuffer);
		bodyStaticListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clBodyStatic) * m_pWorld->m_bodyCount);

		if (xfListData)
			delete [] xfListData;
		xfListData = new b2clTransform[m_pWorld->m_bodyCount];

		if (xfListBuffer)
		    b2CLDevice::instance().freeArray(xfListBuffer);
	    xfListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clTransform) * m_pWorld->m_bodyCount);

		if (bodyDynamicListData)
			delete [] bodyDynamicListData;
		bodyDynamicListData = new b2clBodyDynamic[m_pWorld->m_bodyCount];

		if (bodyDynamicListBuffer)
			b2CLDevice::instance().freeArray(bodyDynamicListBuffer);
		bodyDynamicListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clBodyDynamic) * m_pWorld->m_bodyCount);

		old_body_num = m_pWorld->m_bodyCount;
		//cout << "New body static list" << endl;
	}
	
	std::map<const b2Joint*, Quadruple> jointToBodyMap;
	
	body_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		bodyStaticListData[body_index].m_localCenter[0] = b->m_sweep.localCenter.x;
		bodyStaticListData[body_index].m_localCenter[1] = b->m_sweep.localCenter.y;
		bodyStaticListData[body_index].m_invMass = b->m_invMass;
		bodyStaticListData[body_index].m_invI = b->m_invI;
		bodyStaticListData[body_index].m_linearDamping = b->m_linearDamping;
		bodyStaticListData[body_index].m_angularDamping = b->m_angularDamping;
		bodyStaticListData[body_index].m_gravityScale = b->m_gravityScale;
		bodyStaticListData[body_index].m_type = b->m_type;
		bodyStaticListData[body_index].m_bIsBullet = b->IsBullet() ? 1 : 0;
		
		// Build jointToBodyMap. This data is used to set connected body indices.
		for (b2JointEdge* jn = b->m_jointList; jn; jn = jn->next)
		{
			if (jointToBodyMap.find(jn->joint) == jointToBodyMap.end())
			{
				Quadruple indices;
				indices.x[0] = body_index;
				jointToBodyMap.insert(std::make_pair(jn->joint, indices));
			}
			else
			{
				Quadruple& indices = jointToBodyMap[jn->joint];
				for (int i = 1; i < 4; ++i)
				{
					if (indices.x[i] == -1)
					{
						indices.x[i] = body_index;
						break;
					}
				}
			}
		}

		body_index++;
	}

	// Set connected body indices. Each body has a list of other body indices which are connected by a joint.
	// Connected body indices are used in Broad phase to ignore contacts of bodies which are connected by joints.
	body_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		int connectedArrayPos = 0;
		for (b2JointEdge* jn = b->m_jointList; jn; jn = jn->next)
		{
			if (jn->joint->m_collideConnected)
				continue;

			const Quadruple& indices = jointToBodyMap[jn->joint];
			for (int i = 0; i < 4; ++i)
			{
				if (indices.x[i] == -1)
					break;

				if (indices.x[i] > body_index)
				{
					if (connectedArrayPos == MAX_CONNECTED_BODY_INDICES)
					{
						// TODO: too many joints in one body
                        printf("connectedArrayPos: %d\n", connectedArrayPos);
						assert(false);
						break;
					}

					bodyStaticListData[body_index].m_connectedBodyIndices[connectedArrayPos++] = indices.x[i];
				}
			}
		}
		if (connectedArrayPos < MAX_CONNECTED_BODY_INDICES)
			bodyStaticListData[body_index].m_connectedBodyIndices[connectedArrayPos] = -1;

		body_index++;
	}

	// Copy data from CPU to GPU
	b2CLDevice::instance().copyArrayToDevice(bodyStaticListBuffer, bodyStaticListData, 0, sizeof(b2clBodyStatic) * m_pWorld->m_bodyCount);
}

void b2CLCommonData::CopyDynamicBodyAttributes(b2World *m_pWorld)
{
	// Initialize data on CPU
	int body_index;
	
	body_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		b->m_last_uid = b->m_uid;

		xfListData[body_index].p[0] = b->m_xf.p.x;
		xfListData[body_index].p[1] = b->m_xf.p.y;
		xfListData[body_index].q[0] = b->m_xf.q.s;
		xfListData[body_index].q[1]  = b->m_xf.q.c;

		bodyDynamicListData[body_index].m_sweep.localCenter[0] = b->m_sweep.localCenter.x;
		bodyDynamicListData[body_index].m_sweep.localCenter[1] = b->m_sweep.localCenter.y;
		bodyDynamicListData[body_index].m_sweep.c0[0] = b->m_sweep.c0.x;
		bodyDynamicListData[body_index].m_sweep.c0[1] = b->m_sweep.c0.y;
		bodyDynamicListData[body_index].m_sweep.c[0] = b->m_sweep.c.x;
		bodyDynamicListData[body_index].m_sweep.c[1] = b->m_sweep.c.y;
		bodyDynamicListData[body_index].m_sweep.a0 = b->m_sweep.a0;
		bodyDynamicListData[body_index].m_sweep.a = b->m_sweep.a;
		//bodyDynamicListData[body_index].m_sweep.alpha0 = b->m_sweep.alpha0;
		bodyDynamicListData[body_index].m_linearVelocity[0] = b->m_linearVelocity.x;
		bodyDynamicListData[body_index].m_linearVelocity[1] = b->m_linearVelocity.y;
		bodyDynamicListData[body_index].m_force[0] = b->m_force.x;
		bodyDynamicListData[body_index].m_force[1] = b->m_force.y;
		bodyDynamicListData[body_index].m_angularVelocity = b->m_angularVelocity;
		bodyDynamicListData[body_index].m_torque = b->m_torque;

		bodyDynamicListData[body_index].m_last_uid = b->m_last_uid;

		b->m_uid = body_index;

		body_index++;
	}

	// Copy data from CPU to GPU
	b2CLDevice::instance().copyArrayToDevice(xfListBuffer, xfListData, 0, sizeof(b2clTransform) * m_pWorld->m_bodyCount);
	b2CLDevice::instance().copyArrayToDevice(bodyDynamicListBuffer, bodyDynamicListData, 0, sizeof(b2clBodyDynamic) * m_pWorld->m_bodyCount);
}

void b2CLCommonData::DeleteJoints()
{
	numTotalJoints = 0;

	delete[] jointListData;
	jointListData = NULL;

	if (jointListBuffer) 
	{
		b2CLDevice::instance().freeArray(jointListBuffer);
		jointListBuffer = NULL;
	}

	for (int32 i = 0; i < numJointTypes; ++i)
	{
		numJoints[i] = 0;
		jointMaxColor[i] = 0;
	}
}

void b2CLCommonData::CopyJoints(b2World *m_pWorld, bool warmStarting, bool isJointChanged, bool isJointUpdated) 
{
	if (!isJointChanged && !isJointUpdated)
		return;

	if (isJointChanged)
	{
		if (warmStarting)
			StoreJointImpulses();
		lastNumTotalJoints = numTotalJoints;

		DeleteJoints();

		numTotalJoints = m_pWorld->m_jointCount;
	}	

	if (numTotalJoints == 0)
		return;

	b2Joint* pJoint = NULL;
	if (isJointChanged)
	{
		pJoint = m_pWorld->m_jointList; 
		for (int32 i = 0; i < numTotalJoints; ++i)
		{
			assert(pJoint->GetType() < numJointTypes);
			++numJoints[pJoint->GetType()];
			pJoint = pJoint->GetNext(); 
		}

		jointListData = new b2clJoint [numTotalJoints];
		memset(jointListData, 0, sizeof(b2clJoint) * numTotalJoints);
		jointListBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clJoint) * numTotalJoints);
	}

	int32 jointIndex[numJointTypes];
	int32 currentIndex = 0;
	for (int32 i = 0; i < numJointTypes; ++i)
	{
		jointIndex[i] = currentIndex;
		jointColorOffsets[i][0] = currentIndex;

		currentIndex += numJoints[i];
	}

	pJoint = m_pWorld->m_jointList;
	for (int32 i = 0; i < m_pWorld->m_jointCount; ++i)
	{
		int32 jointType = pJoint->GetType();
		assert(copyJointFunc[jointType]);
		(*copyJointFunc[jointType])(pJoint, &jointListData[jointIndex[jointType]++]);
		pJoint = pJoint->GetNext(); 
	}
	ComputeJointColors(m_pWorld);

	if (isJointChanged)
	{
		b2CLDevice::instance().copyArrayToDevice(this->jointListBuffer, jointListData, 0, sizeof(b2clJoint) * numTotalJoints, true);

		if (warmStarting)
			ReadLastJointImpulses();
	}
	else
	{
		// copy data without indices and impulses
		const size_t origin[] = {sizeof(b2clJointImpulseNode), 0, 0};
		const size_t region[] = {sizeof(b2clJoint) - sizeof(b2clJointImpulseNode), numTotalJoints, 1};
		cl_int ciErrNum = clEnqueueWriteBufferRect(b2CLDevice::instance().GetCommandQueue(), jointListBuffer, CL_TRUE, origin, origin, region,
			sizeof(b2clJoint), 0, sizeof(b2clJoint), 0, jointListData, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);
	}
}

void b2CLCommonData::ComputeJointColors(b2World *m_pWorld)
{
	bool* freezed_body = new bool[m_pWorld->m_bodyCount];
	memset(freezed_body, 0, m_pWorld->m_bodyCount * sizeof(bool));

	int32* jointColors = new int32[numTotalJoints];
	int32* jointIndexToColoredJointIndexMap = new int32[numTotalJoints]; 

	b2clJoint* sortedList = new b2clJoint[numTotalJoints]; 

	// compute colors for each joint type
	for (int jointType = 0; jointType < numJointTypes; ++jointType)
	{
		int32 numCurrentTypeJoints = numJoints[jointType];
		if (numCurrentTypeJoints == 0) 
			continue;

		int32 typeOffset = jointColorOffsets[jointType][0];

		memset(jointColors, 0, numCurrentTypeJoints * sizeof(int32));

		int32 color = 1;
		int32 coloredJointIndex = 0;
		while (coloredJointIndex < numCurrentTypeJoints) 
		{
			for (int32 i = 0; i < numCurrentTypeJoints; ++i) 
			{
				if (jointColors[i]) 
					continue;

				b2clJoint& currentJointData = jointListData[typeOffset + i];

				int32 indexA = currentJointData.indexA; 
				int32 indexB = currentJointData.indexB; 
				int32 indexC = currentJointData.indexC;
				int32 indexD = currentJointData.indexD;

				if (freezed_body[indexA] || freezed_body[indexB])
					continue;
				if (indexC != INVALID_JOINT_ID)
				{
					if (freezed_body[indexC])
						continue;
				}
				if (indexD != INVALID_JOINT_ID)
				{
					if (freezed_body[indexD])
						continue;
				}
            
				currentJointData.color = jointColors[i] = color;
				jointIndexToColoredJointIndexMap[i] = coloredJointIndex;
				coloredJointIndex++;
			
				if (bodyStaticListData[indexA].m_type == b2_dynamicBody)
					freezed_body[indexA] = true;
				if (bodyStaticListData[indexB].m_type == b2_dynamicBody)
					freezed_body[indexB] = true;
				if (indexC != INVALID_JOINT_ID)
				{
					if (bodyStaticListData[indexC].m_type == b2_dynamicBody)
					{
						freezed_body[indexC] = true;
					}
				}
				if (indexD != INVALID_JOINT_ID)
				{
					if (bodyStaticListData[indexD].m_type == b2_dynamicBody)
					{
						freezed_body[indexD] = true;
					}
				}
			}
			++color;
			if (color > maxContactNumPerBody) 
			{
				printf ("joint color value exceeds max contact number per body \n");
				exit(1); 
			}
			memset(freezed_body, 0, m_pWorld->m_bodyCount*sizeof(bool));   
			jointColorOffsets[jointType][color-1] = typeOffset + coloredJointIndex;
		}
		jointMaxColor[jointType] = color-1;  
		jointColorOffsets[jointType][color-1] = typeOffset + numCurrentTypeJoints;
	
		for (int32 i = 0 ; i < numCurrentTypeJoints ; ++i) 
		{
			sortedList[typeOffset + jointIndexToColoredJointIndexMap[i]] = this->jointListData[typeOffset + i];
		}
	}
	delete[] jointColors;
	delete[] jointIndexToColoredJointIndexMap;
	delete[] freezed_body;
	delete[] jointListData;

	jointListData = sortedList; 
}

// Copy joint data which is common in all joint types.
// Used in CopyXXXJoint() functions.
void b2CLCommonData::CopyJointCommon(const b2Joint* pSrc, b2clJoint* pDest)
{
	pDest->color = 0;
	pDest->type = static_cast<int>(pSrc->GetType());
	pDest->index = b2CLCommonData::instance().jointPointerToUniqueIndexMap[pSrc];
	pDest->collideConnected = pSrc->GetCollideConnected() ? 1 : 0;

	pDest->indexA = pSrc->m_bodyA->m_uid;
	pDest->indexB = pSrc->m_bodyB->m_uid;
	pDest->indexC = INVALID_JOINT_ID;
	pDest->indexD = INVALID_JOINT_ID;

	memset(pDest->impulse, 0, sizeof(float) * 4);
}

void b2CLCommonData::CopyDistanceJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2DistanceJoint* pDistanceJoint = static_cast<const b2DistanceJoint*>(pSrc);

	pDest->distanceJointData.frequencyHz = pDistanceJoint->GetFrequency() ;
	pDest->distanceJointData.dampingRatio = pDistanceJoint->GetDampingRatio() ; 
	pDest->distanceJointData.bias = pDistanceJoint->m_bias ; 
	pDest->distanceJointData.length = pDistanceJoint->m_length ; 
	pDest->distanceJointData.localAnchorA[0] = pDistanceJoint->m_localAnchorA.x ; 
	pDest->distanceJointData.localAnchorA[1] = pDistanceJoint->m_localAnchorA.y ;
	pDest->distanceJointData.localAnchorB[0] = pDistanceJoint->m_localAnchorB.x ; 
	pDest->distanceJointData.localAnchorB[1] = pDistanceJoint->m_localAnchorB.y ;
}

void b2CLCommonData::CopyRevoluteJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2RevoluteJoint* pRevoluteJoint = static_cast<const b2RevoluteJoint*>(pSrc);

	pDest->revoluteJointData.localAnchorA[0] = pRevoluteJoint->m_localAnchorA.x; 
	pDest->revoluteJointData.localAnchorA[1] = pRevoluteJoint->m_localAnchorA.y;
	pDest->revoluteJointData.localAnchorB[0] = pRevoluteJoint->m_localAnchorB.x; 
	pDest->revoluteJointData.localAnchorB[1] = pRevoluteJoint->m_localAnchorB.y;
	
	pDest->revoluteJointData.enableMotor = pRevoluteJoint->m_enableMotor ? 1 : 0;
	pDest->revoluteJointData.maxMotorTorque = pRevoluteJoint->m_maxMotorTorque;
	pDest->revoluteJointData.motorSpeed = pRevoluteJoint->m_motorSpeed;

	pDest->revoluteJointData.enableLimit = pRevoluteJoint->m_enableLimit ? 1 : 0;
	pDest->revoluteJointData.referenceAngle = pRevoluteJoint->m_referenceAngle;
	pDest->revoluteJointData.lowerAngle = pRevoluteJoint->m_lowerAngle;
	pDest->revoluteJointData.upperAngle = pRevoluteJoint->m_upperAngle;
} 

void b2CLCommonData::CopyPrismaticJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2PrismaticJoint* pPrismaticJoint = static_cast<const b2PrismaticJoint*>(pSrc);

	pDest->prismaticJointData.localAnchorA[0] = pPrismaticJoint->m_localAnchorA.x; 
	pDest->prismaticJointData.localAnchorA[1] = pPrismaticJoint->m_localAnchorA.y;
	pDest->prismaticJointData.localAnchorB[0] = pPrismaticJoint->m_localAnchorB.x; 
	pDest->prismaticJointData.localAnchorB[1] = pPrismaticJoint->m_localAnchorB.y;
	pDest->prismaticJointData.localXAxisA[0] = pPrismaticJoint->m_localXAxisA.x; 
	pDest->prismaticJointData.localXAxisA[1] = pPrismaticJoint->m_localXAxisA.y;
	pDest->prismaticJointData.localYAxisA[0] = pPrismaticJoint->m_localYAxisA.x; 
	pDest->prismaticJointData.localYAxisA[1] = pPrismaticJoint->m_localYAxisA.y;
	pDest->prismaticJointData.referenceAngle = pPrismaticJoint->m_referenceAngle;

	pDest->prismaticJointData.lowerTranslation = pPrismaticJoint->m_lowerTranslation;
	pDest->prismaticJointData.upperTranslation = pPrismaticJoint->m_upperTranslation;
	pDest->prismaticJointData.maxMotorForce = pPrismaticJoint->m_maxMotorForce;
	pDest->prismaticJointData.motorSpeed = pPrismaticJoint->m_motorSpeed;
	pDest->prismaticJointData.enableLimit = pPrismaticJoint->m_enableLimit ? 1 : 0;
	pDest->prismaticJointData.enableMotor = pPrismaticJoint->m_enableMotor ? 1 : 0;
	pDest->prismaticJointData.limitState = static_cast<int>(pPrismaticJoint->m_limitState);
}

void b2CLCommonData::CopyGearJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2GearJoint* pGearJoint = static_cast<const b2GearJoint*>(pSrc);

	pDest->gearJointData.joint1 = pGearJoint->m_joint1->m_index;
	pDest->gearJointData.joint2 = pGearJoint->m_joint2->m_index;

	pDest->gearJointData.typeA = static_cast<int>(pGearJoint->m_joint1->GetType());
	pDest->gearJointData.typeB = static_cast<int>(pGearJoint->m_joint2->GetType());

	pDest->gearJointData.localAnchorA[0] = pGearJoint->m_localAnchorA.x; 
	pDest->gearJointData.localAnchorA[1] = pGearJoint->m_localAnchorA.y;
	pDest->gearJointData.localAnchorB[0] = pGearJoint->m_localAnchorB.x; 
	pDest->gearJointData.localAnchorB[1] = pGearJoint->m_localAnchorB.y;
	pDest->gearJointData.localAnchorC[0] = pGearJoint->m_localAnchorC.x; 
	pDest->gearJointData.localAnchorC[1] = pGearJoint->m_localAnchorC.y;
	pDest->gearJointData.localAnchorD[0] = pGearJoint->m_localAnchorD.x; 
	pDest->gearJointData.localAnchorD[1] = pGearJoint->m_localAnchorD.y;
	pDest->gearJointData.localAxisC[0] = pGearJoint->m_localAxisC.x; 
	pDest->gearJointData.localAxisC[1] = pGearJoint->m_localAxisC.y;
	pDest->gearJointData.localAxisD[0] = pGearJoint->m_localAxisD.x; 
	pDest->gearJointData.localAxisD[1] = pGearJoint->m_localAxisD.y;
	pDest->gearJointData.referenceAngleA = pGearJoint->m_referenceAngleA;
	pDest->gearJointData.referenceAngleB = pGearJoint->m_referenceAngleB;
	pDest->gearJointData.gearConstant = pGearJoint->m_constant;
	pDest->gearJointData.ratio = pGearJoint->m_ratio;

	// Solver temp
	pDest->indexC = pGearJoint->m_bodyC->m_uid;
	pDest->indexD = pGearJoint->m_bodyD->m_uid;
}

void b2CLCommonData::CopyPulleyJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2PulleyJoint* pPulleyJoint = static_cast<const b2PulleyJoint*>(pSrc);

	pDest->pulleyJointData.groundAnchorA[0] = pPulleyJoint->m_groundAnchorA.x ;
	pDest->pulleyJointData.groundAnchorA[1] = pPulleyJoint->m_groundAnchorA.y ;
	pDest->pulleyJointData.groundAnchorB[0] = pPulleyJoint->m_groundAnchorB.x ;
	pDest->pulleyJointData.groundAnchorB[1] = pPulleyJoint->m_groundAnchorB.y ; 
	pDest->pulleyJointData.lengthA = pPulleyJoint->m_lengthA ; 
	pDest->pulleyJointData.lengthB = pPulleyJoint->m_lengthB ; 
	pDest->pulleyJointData.localAnchorA[0] = pPulleyJoint->m_localAnchorA.x; 
	pDest->pulleyJointData.localAnchorA[1] = pPulleyJoint->m_localAnchorA.y;
	pDest->pulleyJointData.localAnchorB[0] = pPulleyJoint->m_localAnchorB.x; 
	pDest->pulleyJointData.localAnchorB[1] = pPulleyJoint->m_localAnchorB.y;
	pDest->pulleyJointData.pulleyConstant = pPulleyJoint->m_constant ; 
	pDest->pulleyJointData.ratio = pPulleyJoint->m_ratio ; 
}

void b2CLCommonData::CopyRopeJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2RopeJoint* pRopeJoint = static_cast<const b2RopeJoint*>(pSrc);

	pDest->ropeJointData.maxLength = pRopeJoint->m_maxLength ; 
	pDest->ropeJointData.length = pRopeJoint->m_length ; 
	pDest->ropeJointData.limitState = pRopeJoint->m_state ; 
 
	pDest->ropeJointData.localAnchorA[0] = pRopeJoint->m_localAnchorA.x; 
	pDest->ropeJointData.localAnchorA[1] = pRopeJoint->m_localAnchorA.y;
	pDest->ropeJointData.localAnchorB[0] = pRopeJoint->m_localAnchorB.x; 
	pDest->ropeJointData.localAnchorB[1] = pRopeJoint->m_localAnchorB.y;
}

void b2CLCommonData::CopyFrictionJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2FrictionJoint* pFrictionJoint = static_cast<const b2FrictionJoint*>(pSrc);

	// Solver shared
	pDest->frictionJointData.localAnchorA[0] = pFrictionJoint->m_localAnchorA.x;
	pDest->frictionJointData.localAnchorA[1] = pFrictionJoint->m_localAnchorA.y;
	pDest->frictionJointData.localAnchorB[0] = pFrictionJoint->m_localAnchorB.x;
	pDest->frictionJointData.localAnchorB[1] = pFrictionJoint->m_localAnchorB.y;
	pDest->frictionJointData.maxForce = pFrictionJoint->m_maxForce ; 
	pDest->frictionJointData.maxTorque = pFrictionJoint->m_maxTorque ; 
}

void b2CLCommonData::CopyWheelJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2WheelJoint* pWheelJoint = static_cast<const b2WheelJoint*>(pSrc);

	pDest->wheelJointData.frequencyHz = pWheelJoint->m_frequencyHz;
	pDest->wheelJointData.dampingRatio = pWheelJoint->m_dampingRatio;

	// Solver shared
	pDest->wheelJointData.localAnchorA[0] = pWheelJoint->m_localAnchorA.x;
	pDest->wheelJointData.localAnchorA[1] = pWheelJoint->m_localAnchorA.y;
	pDest->wheelJointData.localAnchorB[0] = pWheelJoint->m_localAnchorB.x;
	pDest->wheelJointData.localAnchorB[1] = pWheelJoint->m_localAnchorB.y;
	pDest->wheelJointData.localXAxisA[0] = pWheelJoint->m_localXAxisA.x;
	pDest->wheelJointData.localXAxisA[1] = pWheelJoint->m_localXAxisA.y;
	pDest->wheelJointData.localYAxisA[0] = pWheelJoint->m_localYAxisA.x;
	pDest->wheelJointData.localYAxisA[1] = pWheelJoint->m_localYAxisA.y;
	pDest->wheelJointData.maxMotorTorque = pWheelJoint->m_maxMotorTorque;
	pDest->wheelJointData.motorSpeed = pWheelJoint->m_motorSpeed;
	pDest->wheelJointData.enableMotor = pWheelJoint->m_enableMotor ? 1 : 0;
}

void b2CLCommonData::CopyWeldJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2WeldJoint* pWeldJoint = static_cast<const b2WeldJoint*>(pSrc);

	pDest->weldJointData.frequencyHz = pWeldJoint->m_frequencyHz;
	pDest->weldJointData.dampingRatio = pWeldJoint->m_dampingRatio;
	pDest->weldJointData.bias = pWeldJoint->m_bias;

	// Solver shared
	pDest->weldJointData.localAnchorA[0] = pWeldJoint->m_localAnchorA.x;
	pDest->weldJointData.localAnchorA[1] = pWeldJoint->m_localAnchorA.y;
	pDest->weldJointData.localAnchorB[0] = pWeldJoint->m_localAnchorB.x;
	pDest->weldJointData.localAnchorB[1] = pWeldJoint->m_localAnchorB.y;
	pDest->weldJointData.referenceAngle = pWeldJoint->m_referenceAngle;
	pDest->weldJointData.gamma = pWeldJoint->m_gamma;
}

void b2CLCommonData::CopyMouseJoint(const b2Joint* pSrc, b2clJoint* pDest)
{
	CopyJointCommon(pSrc, pDest);

	const b2MouseJoint* pMouseJoint = static_cast<const b2MouseJoint*>(pSrc);

	pDest->mouseJointData.localAnchorB[0] = pMouseJoint->m_localAnchorB.x;
	pDest->mouseJointData.localAnchorB[1] = pMouseJoint->m_localAnchorB.y;
	pDest->mouseJointData.targetA[0] = pMouseJoint->m_targetA.x;
	pDest->mouseJointData.targetA[1] = pMouseJoint->m_targetA.y;

	pDest->mouseJointData.frequencyHz = pMouseJoint->m_frequencyHz;
	pDest->mouseJointData.dampingRatio = pMouseJoint->m_dampingRatio;
	pDest->mouseJointData.beta = pMouseJoint->m_beta;

	// Solver shared
	pDest->mouseJointData.maxForce = pMouseJoint->m_maxForce;
	pDest->mouseJointData.gamma = pMouseJoint->m_gamma;
}

void b2CLCommonData::ReadLastJointImpulses()
{
	if (lastNumTotalJoints == 0)
		return;

	// execute kernel
	unsigned int a = 0;
        
    int err = CL_SUCCESS;
	err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(cl_mem), &jointListBuffer);
	err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(cl_mem), &jointImpulsesBuffer);
	err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(cl_mem), &jointImpulseKeysBuffer);
	err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(cl_mem), &jointImpulseGlobalIndicesBuffer);
	err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(int), &numTotalJoints);
    err |= clSetKernelArg(readLastJointImpulseKernel,  a++, sizeof(int), &lastNumTotalJoints);
	if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", (char *) readLastJointImpulseKernel);
        return;
    }

	int group_num = (numTotalJoints + maxWorkGroupSizeForreadLastJointImpulse-1)/maxWorkGroupSizeForreadLastJointImpulse;
        
    size_t global = group_num * maxWorkGroupSizeForreadLastJointImpulse;
    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(b2CLDevice::instance().GetCommandQueue(), readLastJointImpulseKernel, 1, NULL, &global, &maxWorkGroupSizeForreadLastJointImpulse, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", (char *) readLastJointImpulseKernel);
        return;
    }
}

void b2CLCommonData::StoreJointImpulses()
{
	assert(numTotalJoints >= 0);

	if (numTotalJoints == 0)
		return;

	int sortCount = numTotalJoints;

#ifndef USE_CPU_SORT
	if (numTotalJoints < BITONIC_SORT_INTEL_MINNUM)
		sortCount = BITONIC_SORT_INTEL_MINNUM;
	else
	{
		// compute the least power-of-2 which >= m_contactCount
		int exp;
		frexp((float)numTotalJoints, &exp);
		sortCount = 1 << (exp-1);
		if (sortCount < numTotalJoints)
			sortCount <<= 1;
	}
#endif

	// Re-allocate impulse buffer if needed.
	if (sortCount > lastSortCount)
	{
		if (jointImpulsesBuffer)
			b2CLDevice::instance().freeArray(jointImpulsesBuffer);
		jointImpulsesBuffer = b2CLDevice::instance().allocateArray(sizeof(b2clJointImpulseNode) * sortCount);

		if (jointImpulseKeysBuffer)
			b2CLDevice::instance().freeArray(jointImpulseKeysBuffer);
		jointImpulseKeysBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * sortCount);

		if (jointImpulseGlobalIndicesBuffer)
			b2CLDevice::instance().freeArray(jointImpulseGlobalIndicesBuffer);
		jointImpulseGlobalIndicesBuffer = b2CLDevice::instance().allocateArray(sizeof(int) * sortCount);
	}

	{
		const size_t origin[] = {0, 0, 0};
		const size_t region[] = {sizeof(b2clJointImpulseNode), numTotalJoints, 1};
		cl_int ciErrNum = clEnqueueCopyBufferRect(b2CLDevice::instance().GetCommandQueue(), jointListBuffer, jointImpulsesBuffer, origin, origin, region,
			sizeof(b2clJoint), 0, 0, 0, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);
	}

	{
		const size_t origin[] = {0, 0, 0};
		const size_t region[] = {sizeof(int), numTotalJoints, 1};
		cl_int ciErrNum = clEnqueueCopyBufferRect(b2CLDevice::instance().GetCommandQueue(), jointListBuffer, jointImpulseKeysBuffer, origin, origin, region,
			sizeof(b2clJoint), 0, 0, 0, 0, NULL, NULL);
		b2clCheckError(ciErrNum, CL_SUCCESS);

#ifndef USE_CPU_SORT
		// Because the buffer size is a power of 2, set unused part of the impulse key buffer to 0.
		unsigned int *zeroBuffer = new unsigned int[sortCount - numTotalJoints];
		memset(zeroBuffer, 0, sizeof(unsigned int) * (sortCount - numTotalJoints));
		b2CLDevice::instance().copyArrayToDevice(jointImpulseKeysBuffer, zeroBuffer, sizeof(int) * numTotalJoints, sizeof(unsigned int) * (sortCount - numTotalJoints), true);
		delete [] zeroBuffer;
#endif
	}

	{
		int* ascendingNumbers = new int[numTotalJoints];
		for (int i = 0; i < numTotalJoints; ++i)
		{
			ascendingNumbers[i] = i;
		}
		b2CLDevice::instance().copyArrayToDevice(jointImpulseGlobalIndicesBuffer, ascendingNumbers, 0, sizeof(int) * numTotalJoints, true);
		delete[] ascendingNumbers;
	}

	// sort by descending order of indices
#if defined(USE_CPU_SORT)
	b2CLSort::instance().stlSort(jointImpulseKeysBuffer, jointImpulseGlobalIndicesBuffer, numTotalJoints, 0, 1);
#else
	b2CLSort::instance().bitonicSort_Intel(jointImpulseKeysBuffer, jointImpulseGlobalIndicesBuffer, sortCount, 0);
#endif

	lastSortCount = sortCount;
}

void b2CLCommonData::JointAdded(const b2Joint* pJoint)
{
	// Assign an unique index for a new joint.
	// The index is used in CopyJointCommon().
	jointPointerToUniqueIndexMap.insert(std::make_pair(pJoint, nextJointUniqueIndex++));
	
}
void b2CLCommonData::JointRemoved(const b2Joint* pJoint)
{
	jointPointerToUniqueIndexMap.erase(pJoint);
}

void b2CLCommonData::initReadLastJointImpulses()
{
	if (b2clGlobal_OpenCLSupported)
	{
		printf("Initializing b2CLCommonData...\n");
    
		int err;
    
		//load opencl programs from files
		char* commonDataSource = 0;
		size_t commonDataSourceLen = 0;

		shrLog("...loading b2CLCommonData.cl\n");
    #ifdef linux
    	commonDataSource = b2clLoadProgSource(shrFindFilePath("/opt/apps/com.samsung.browser/include/Box2D/Common/OpenCL/b2CLCommonData.cl", NULL), "// My comment\n", &commonDataSourceLen);
	#elif defined (_WIN32)
		commonDataSource = b2clLoadProgSource(shrFindFilePath("../../Box2D/Common/OpenCL/b2CLCommonData.cl", NULL), "// My comment\n", &commonDataSourceLen);
    #elif defined (__EMSCRIPTEN__)
        commonDataSource = b2clLoadProgSource(shrFindFilePath("./Common/OpenCL/b2CLCommonData.cl", NULL), "// My comment\n", &commonDataSourceLen);
	#else
		commonDataSource = b2clLoadProgSource(shrFindFilePath("/usr/local/include/Box2D/Common/OpenCL/b2CLCommonData.cl", NULL), "// My comment\n", &commonDataSourceLen);
	#endif
		if(commonDataSource == NULL)
		{
			b2Log("Could not load program source, is path 'b2CLCommonData.cl' correct?");
		}

		//create the compute program from source kernel code
		commonDataProgram = clCreateProgramWithSource(b2CLDevice::instance().GetContext(), 1, (const char**) &commonDataSource, NULL, &err);
		if (!commonDataProgram)
		{
			printf("Error: Failed to create compute program!\n");
			exit(1);
		}
    
		//build the program
		err = clBuildProgram(commonDataProgram, 0, NULL, OPENCL_BUILD_PATH, NULL, NULL);
		if (err != CL_SUCCESS)
		{
			size_t len;
			char buffer[20480];
        
			printf("Error: Failed to build program executable!\n");
			clGetProgramBuildInfo(commonDataProgram, b2CLDevice::instance().GetCurrentDevice(), CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
			printf("%s\n", buffer);
			exit(1);
		}
    
		//create the compute kernel
		readLastJointImpulseKernel = clCreateKernel(commonDataProgram, "ReadLastJointImpulses", &err);
		if (!readLastJointImpulseKernel || err != CL_SUCCESS)
		{
			printf("Error: Failed to create compute kernel!\n");
			exit(1);
		}
		b2CLDevice::instance().getMaximumKernelWorkGroupSize(readLastJointImpulseKernel, maxWorkGroupSizeForreadLastJointImpulse);
	}
}

void b2CLCommonData::UpdateFixtureAddressAndChildren(b2World *m_pWorld)
{
	// alocate space if neccesary
	if (old_proxy_count<m_pWorld->m_proxyCount)
	{
		if (fixture_address)
			delete [] fixture_address;
		fixture_address = new b2Fixture* [m_pWorld->m_proxyCount];

		if (fixture_child)
			delete [] fixture_child;
		fixture_child = new int [m_pWorld->m_proxyCount];

		old_proxy_count = m_pWorld->m_proxyCount;
	}

	int fixture_index = 0;
	for (b2Body* b = m_pWorld->m_bodyList; b; b = b->m_next)
	{
		for (b2Fixture* f = b->m_fixtureList; f; f = f->m_next)
		{
			b2PolygonShape *s = (b2PolygonShape*)(f->m_shape);

			for (int i=0; i<s->GetChildCount(); i++)
			{
				fixture_address[fixture_index] = f;
				fixture_child[fixture_index] = i;
				fixture_index++;
			}
		}
	}
}