/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
// this Box2DOCL file is developed based on Box2D
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2Island.h>
#include <Box2D/Dynamics/Joints/b2PulleyJoint.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Common/b2Draw.h>
#include <Box2D/Common/b2Timer.h>
#include <new>
#include<set>
#include<vector>
//#include <Box2D/Common/OpenCL/b2CLTimeOfImpact.h>
#include <Box2D/Common/OpenCL/b2CLSolveTOI.h>

b2World::b2World(const b2Vec2& gravity)
{
	m_destructionListener = NULL;
	m_debugDraw = NULL;

	m_bodyList = NULL;
	m_jointList = NULL;

	m_bodyCount = 0;
	m_jointCount = 0;
	m_fixtureCount = 0;
	m_proxyCount = 0;
	m_bBodyChanged = false;
	m_bFixtureChanged = false;
	m_bBodyStaticAttributeChanged = false;
	m_bBodyDynamicAttributeChanged = false;
	m_uListenerCallback = 0;

	/// For JointChanged
	m_bJointChanged = true;
	m_bJointUpdated = false;

	m_warmStarting = true;
	m_continuousPhysics = true;
	m_subStepping = false;

	m_stepComplete = true;

	m_allowSleep = true;
	m_gravity = gravity;

	m_flags = e_clearForces;

	m_inv_dt0 = 0.0f;

	m_contactManager.m_allocator = &m_blockAllocator;

	memset(&m_profile, 0, sizeof(b2Profile));

#ifdef BOX2D_OPENCL
	b2CLDevice::instance(); // to init it in case it's the first time a world is created
#endif

	m_contactManager.SetWorldPointer(this);

	frame_num = 0;

#ifdef SOLVER_OPENCL
	if (b2clGlobal_OpenCLSupported)
	{
		bFirstFrame = false;
	}
#endif

#if defined(_DEBUG_TIME)
	numLoops = 0;
	ave_contact_num = ave_valid_contact_num = 0;
	bOutputToFile = false;
#endif

#if defined(_DEBUG_TIME_STEP_TOTAL)
	stepTotalTime = 0;
	stepNarrowPhaseTime = stepSolverTime = stepBroadPhaseTime = 0;
	stepTOIPhaseTime = 0 ; 
#endif

#if defined(_DEBUG_TIME_BROADPHASE)
	updatePairsTime = 0;
	CreateGPUBuffersTime = ComputeAABBsTime = SortAABBsTime = ComputePairsTime = 0;
#endif

#if defined(_DEBUG_TIME_NARROWPHASE)
	updateContactTime = 0;
	InitializeGPUDataTime = UpdateContactPairsTime = ReadbackGPUDataTime = 0;
#endif

#if defined(_DEBUG_TIME_SOLVER)
	solverTotalTime = 0;
	constraintInitCPUTime = constraintInitGPUTime = constraintSolveTime = constraintFinalizeTime = 0;
	testSolveTime = 0;

	testSolveVelocityConstraintTime = testSolvePositionConstraintTime = 0;
#endif
}

b2World::~b2World()
{
	// Some shapes allocate using b2Alloc.
	b2Body* b = m_bodyList;
	while (b)
	{
		b2Body* bNext = b->m_next;

		b2Fixture* f = b->m_fixtureList;
		while (f)
		{
			b2Fixture* fNext = f->m_next;
			f->m_proxyCount = 0;
			f->Destroy(&m_blockAllocator);
			f = fNext;
		}

		b = bNext;
	}
}

void b2World::SetDestructionListener(b2DestructionListener* listener)
{
	m_destructionListener = listener;
}

void b2World::SetContactFilter(b2ContactFilter* filter)
{
	m_contactManager.m_contactFilter = filter;
}

void b2World::SetContactListener(b2ContactListener* listener)
{
	m_contactManager.m_contactListener = listener;
}

void b2World::SetDebugDraw(b2Draw* debugDraw)
{
	m_debugDraw = debugDraw;
}

b2Body* b2World::CreateBody(const b2BodyDef* def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return NULL;
	}

	/*
	// we do not support bullet at this time
	if (def->bullet)
	{
#if defined(_DEBUG) && defined(BOX2D_OPENCL)
		if (b2clGlobal_OpenCLSupported)
			printf("The OpenCL code does not support bullet at this time!\nUse orginal CPU code instead!\n");
#endif
		b2clGlobal_OpenCLSupported = false;
	}
	*/

	void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
	b2Body* b = new (mem) b2Body(def, this);

	// Add to world doubly linked list.
	b->m_prev = NULL;
	b->m_next = m_bodyList;
	if (m_bodyList)
	{
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;
	++m_bodyCount;
	m_bBodyChanged = true;
	m_bJointUpdated = true;

	return b;
}

void b2World::DestroyBody(b2Body* b)
{
	b2Assert(m_bodyCount > 0);
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	// Delete the attached joints.
	b2JointEdge* je = b->m_jointList;
	while (je)
	{
		b2JointEdge* je0 = je;
		je = je->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(je0->joint);
		}

		DestroyJoint(je0->joint);

		b->m_jointList = je;
	}
	b->m_jointList = NULL;

	// Delete the attached contacts.
	b2ContactEdge* ce = b->m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_contactManager.Destroy(ce0->contact);
	}
	b->m_contactList = NULL;

	// Delete the attached fixtures. This destroys broad-phase proxies.
	b2Fixture* f = b->m_fixtureList;
	while (f)
	{
		b2Fixture* f0 = f;
		f = f->m_next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(f0);
		}

		f0->DestroyProxies(&m_contactManager.m_broadPhase);
		f0->Destroy(&m_blockAllocator);
		f0->~b2Fixture();
		m_blockAllocator.Free(f0, sizeof(b2Fixture));

		b->m_fixtureList = f;
		b->m_fixtureCount -= 1;
	}
	b->m_fixtureList = NULL;
	b->m_fixtureCount = 0;

	// Remove world body list.
	if (b->m_prev)
	{
		b->m_prev->m_next = b->m_next;
	}

	if (b->m_next)
	{
		b->m_next->m_prev = b->m_prev;
	}

	if (b == m_bodyList)
	{
		m_bodyList = b->m_next;
	}

	--m_bodyCount;
	m_bBodyChanged = true;
	m_bJointUpdated = true;

	b->~b2Body();
	m_blockAllocator.Free(b, sizeof(b2Body));
}

b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	// for debug
	if (IsLocked())
		int a = 1;

	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return NULL;
	}

	b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = NULL;
	j->m_next = m_jointList;
	if (m_jointList)
	{
		m_jointList->m_prev = j;
	}
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = j->m_bodyA->m_jointList;
	if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
	j->m_bodyA->m_jointList = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = j->m_bodyB->m_jointList;
	if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
	j->m_bodyB->m_jointList = &j->m_edgeB;

	b2Body* bodyA = def->bodyA;
	b2Body* bodyB = def->bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (def->collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}

	// Note: creating a joint doesn't wake the bodies.

#ifdef BOX2D_OPENCL
	b2CLCommonData::instance().JointAdded(j);
#endif
	m_bJointChanged = true; 
	m_bBodyStaticAttributeChanged = true;

//#if defined(_DEBUG) && defined(BOX2D_OPENCL)
//	if (b2clGlobal_OpenCLSupported)
//		printf("The OpenCL code does not support joint at this time!\nUse orginal CPU code instead!\n");
//#endif
//	b2clGlobal_OpenCLSupported = false;

	return j;
}

void b2World::DestroyJoint(b2Joint* j)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
	}

	// Disconnect from island graph.
	b2Body* bodyA = j->m_bodyA;
	b2Body* bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake(true);
	bodyB->SetAwake(true);

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_jointList)
	{
		bodyA->m_jointList = j->m_edgeA.next;
	}

	j->m_edgeA.prev = NULL;
	j->m_edgeA.next = NULL;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_jointList)
	{
		bodyB->m_jointList = j->m_edgeB.next;
	}

	j->m_edgeB.prev = NULL;
	j->m_edgeB.next = NULL;

	b2Joint::Destroy(j, &m_blockAllocator);

	b2Assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}

#ifdef BOX2D_OPENCL
	b2CLCommonData::instance().JointRemoved(j);
#endif
	m_bJointChanged = true;
	m_bBodyStaticAttributeChanged = true;
}

//
void b2World::SetAllowSleeping(bool flag)
{
	if (flag == m_allowSleep)
	{
		return;
	}

	m_allowSleep = flag;
	if (m_allowSleep == false)
	{
		for (b2Body* b = m_bodyList; b; b = b->m_next)
		{
			b->SetAwake(true);
		}
	}
}
void b2World::Solve(const b2TimeStep& step)
{
	// Size the island for the worst case.
	b2Island *pIsland;
#if defined(SOLVER_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
		pIsland = new b2Island(m_bodyCount,
						m_contactManager.m_contactCount,
						m_jointCount,
						&m_stackAllocator,
						m_contactManager.m_contactListener,
						this
						, &m_b2clSolver);
	}
	else
#endif
	{
		pIsland = new b2Island(m_bodyCount,
						m_contactManager.m_contactCount,
						m_jointCount,
						&m_stackAllocator,
						m_contactManager.m_contactListener,
						this
						);
	}

	// Clear all the island flags.
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_flags &= ~b2Body::e_islandFlag;
	}
	for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
	{
		c->m_flags &= ~b2Contact::e_islandFlag;
	}
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_islandFlag = false;
	}

	// Build and simulate all awake islands.
	int32 stackSize = m_bodyCount;
	b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));

	//-- debug for no island
	pIsland->Clear();
	//-- end debug

//int c_num = 0;
#if defined(NOT_USE_ISLAND) && defined(NARROWPHASE_OPENCL)  && defined(SOLVER_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
	#if defined (BROADPHASE_OPENCL)
		pIsland->m_contactCount = m_contactManager.m_contactCount;
	#else
		for (b2Contact *contact = m_contactManager.m_contactList; contact; contact=contact->GetNext())
		{
			//if (contact->IsEnabled() == false ||
			//	contact->IsTouching() == false)
			//{
			//	continue;
			//}
			pIsland->Add(contact);
		}
	#endif
	}
#endif
	for (b2Body* seed = m_bodyList; seed; seed = seed->m_next)
	{
		//-- debug for no island
#ifdef NOT_USE_ISLAND
		{
			//if (seed->IsAwake() == false || seed->IsActive() == false)
			//{
			//	continue;
			//}

			seed->m_flags |= b2Body::e_islandFlag;
			pIsland->Add(seed);
			b2Body* b = seed;
		#if defined(NARROWPHASE_OPENCL) && defined(SOLVER_OPENCL) 
			if (!b2clGlobal_OpenCLSupported)
		#endif
			{
				//int temp = 0;
				for (b2ContactEdge* ce = b->m_contactList; ce; ce = ce->next)
				{
					//temp++;
					b2Contact* contact = ce->contact;

					// Has this contact already been added to an island?
					if (contact->m_flags & b2Contact::e_islandFlag)
					{
						continue;
					}

					// Is this contact solid and touching?
					if (contact->IsEnabled() == false ||
						contact->IsTouching() == false)
					{
						continue;
					}

					// Skip sensors.
 					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					pIsland->Add(contact);
					//c_num++;
					contact->m_flags |= b2Contact::e_islandFlag;

					b2Body* other = ce->other;

					// Was the other body already added to this island?
					if (other->m_flags & b2Body::e_islandFlag)
					{
						continue;
					}

					other->m_flags |= b2Body::e_islandFlag;
				}
			}
			for (b2JointEdge* je = b->m_jointList; je; je = je->next)
			{
				if (je->joint->m_islandFlag == true)
				{
					continue;
				}

				b2Body* other = je->other;

				// Don't simulate joints connected to inactive bodies.
				if (other->IsActive() == false)
				{
					continue;
				}

				pIsland->Add(je->joint);
				je->joint->m_islandFlag = true;

				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				other->m_flags |= b2Body::e_islandFlag;
			}
		}
#else
		if (seed->m_flags & b2Body::e_islandFlag)
		{
			continue;
		}

		if (seed->IsAwake() == false || seed->IsActive() == false)
		{
			continue;
		}

		// The seed can be dynamic or kinematic.
		if (seed->GetType() == b2_staticBody)
		{
			continue;
		}

		// Reset island and stack.
		pIsland->Clear();
		int32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b2Body::e_islandFlag;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			b2Body* b = stack[--stackCount];
			b2Assert(b->IsActive() == true);
			pIsland->Add(b);

			// Make sure the body is awake.
			b->SetAwake(true);

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (b2ContactEdge* ce = b->m_contactList; ce; ce = ce->next)
			{
				b2Contact* contact = ce->contact;

				// Has this contact already been added to an island?
				if (contact->m_flags & b2Contact::e_islandFlag)
				{
					continue;
				}

				// Is this contact solid and touching?
				if (contact->IsEnabled() == false ||
					contact->IsTouching() == false)
				{
					continue;
				}

				// Skip sensors.
				bool sensorA = contact->m_fixtureA->m_isSensor;
				bool sensorB = contact->m_fixtureB->m_isSensor;
				if (sensorA || sensorB)
				{
					continue;
				}

				pIsland->Add(contact);
				contact->m_flags |= b2Contact::e_islandFlag;

				b2Body* other = ce->other;

				// Was the other body already added to this island?
				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}

			// Search all joints connect to this body.
			for (b2JointEdge* je = b->m_jointList; je; je = je->next)
			{
				if (je->joint->m_islandFlag == true)
				{
					continue;
				}

				b2Body* other = je->other;

				// Don't simulate joints connected to inactive bodies.
				if (other->IsActive() == false)
				{
					continue;
				}

				pIsland->Add(je->joint);
				je->joint->m_islandFlag = true;

				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				b2Assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}
		}

		pIsland->Solve(step, m_gravity, m_allowSleep);

		// Post solve cleanup.
		for (int32 i = 0; i < pIsland->m_bodyCount; ++i)
		{
			// Allow static bodies to participate in other islands.
			b2Body* b = pIsland->m_bodies[i];
			if (b->GetType() == b2_staticBody)
			{
				b->m_flags &= ~b2Body::e_islandFlag;
			}
		}
#endif
	}
#ifdef NOT_USE_ISLAND
	//cout << c_num << endl;
	b2Profile profile;
	pIsland->Solve(&profile, step, m_gravity, m_allowSleep);
	m_profile.solveInit += profile.solveInit;
	m_profile.solveVelocity += profile.solveVelocity;
	m_profile.solvePosition += profile.solvePosition;
#endif

	m_stackAllocator.Free(stack);

	delete pIsland;

	int moveCount = m_contactManager.m_broadPhase.m_moveCount;
	//printf("moveCount: %d\n", moveCount);

#if defined(BROADPHASE_OPENCL)
	if (!b2clGlobal_OpenCLSupported || m_continuousPhysics)
#endif
	{
		// Synchronize fixtures, check for out of range bodies.
		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			// If a body was not in an island then it did not move.
			if ((b->m_flags & b2Body::e_islandFlag) == 0)
			{
				continue;
			}

			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			// Update fixtures (for broad-phase).
			b->SynchronizeFixtures();
		}
	}

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2Timer stepBroadPhaseTimer;
#endif

	// Look for new contacts.
	//printf("===FindNewContacts===\n");
#if defined (BOX2D_OPENCL)
	if ( !m_continuousPhysics )
		m_contactManager.FindNewContacts();
#else
	m_contactManager.FindNewContacts();
#endif
	
	//printf("===FindNewContacts finished===\n");

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2CLDevice::instance().finishCommandQueue();
	stepBroadPhaseTime += stepBroadPhaseTimer.GetMilliseconds();
#endif
}

// Find TOI contacts and solve them.
void b2World::SolveTOI(const b2TimeStep& step)
{
	


#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2Timer findTOITimer;
#endif

#if defined (BOX2D_OPENCL)



	m_contactManager.cpuOverLap () ;

#if defined (CCD_SD_OPENCL) && defined (MULTI_SD_OPENCL) 
//The implementation of CCD_OPENCL is not open sourced. 
#else	
	m_contactManager.cpuFindNewContacts() ; 

#endif 


#endif 



	std::set<b2Body*> movedBodySet ; 
b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener, this);
	if (m_stepComplete)
	{
		for (b2Body* b = m_bodyList; b; b = b->m_next)
		{
			b->m_flags &= ~b2Body::e_islandFlag;
			b->m_sweep.alpha0 = 0.0f;
		}

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Invalidate TOI
			c->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			c->m_toiCount = 0;
			c->m_toi = 1.0f;
		}
	}



	bool resetAlpha0 = true;

	int preLoopNum = 0 ; 
	int currentLoopNum = 0 ; 
	for (;;)
	{


	// Find TOI events and solve them

		// Find the first TOI.
		b2Contact* minContact = NULL;
		float32 minAlpha = 1.0f;

		//totalWhileLoop ++ ; 


		preLoopNum = currentLoopNum ; 
		currentLoopNum = 0 ; 

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			currentLoopNum ++ ; 
		//	totalContacts ++ ; 
			// Is this contact disabled?
			if (c->IsEnabled() == false)
			{
				//isCEnabled ++ ; 
				continue;
			}

			// Prevent excessive sub-stepping.
			if (c->m_toiCount > b2_maxSubSteps)
			{
				//isTOICount ++ ; 
				continue;
			}

			float32 alpha = 1.0f;
			if (c->m_flags & b2Contact::e_toiFlag)
			{
				// This contact has a valid cached TOI.
				alpha = c->m_toi;
			}
			else
			{
				b2Fixture* fA = c->GetFixtureA();
				b2Fixture* fB = c->GetFixtureB();

				// Is there a sensor?
				if (fA->IsSensor() || fB->IsSensor())
				{
					//isSensorAB ++ ; 
					continue;
				}

				b2Body* bA = fA->GetBody();
				b2Body* bB = fB->GetBody();

				b2BodyType typeA = bA->m_type;
				b2BodyType typeB = bB->m_type;
				b2Assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

				bool activeA = bA->IsAwake() && typeA != b2_staticBody;
				bool activeB = bB->IsAwake() && typeB != b2_staticBody;


				// Is at least one body active (awake and dynamic or kinematic)?
				if (activeA == false && activeB == false)
				{
					//isActiveAB ++ ; 
					continue;
					
				}

				bool collideA = bA->IsBullet() || typeA != b2_dynamicBody;
				bool collideB = bB->IsBullet() || typeB != b2_dynamicBody;


				// Are these two non-bullet dynamic bodies?
				if (collideA == false && collideB == false)
				{
					//isCollideAB ++ ; 
					continue;
					
				}
#if defined (BOX2D_OPENCL)
#if defined (CCD_SD_OPENCL)		
// The implementation of CCD_OPENCL is not open sourced. 
#endif
#endif

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				float32 alpha0 = bA->m_sweep.alpha0;

				if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0)
				{
					alpha0 = bB->m_sweep.alpha0;
					bA->m_sweep.Advance(alpha0);
				}
				else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0)
				{
					alpha0 = bA->m_sweep.alpha0;
					bB->m_sweep.Advance(alpha0);
				}

				b2Assert(alpha0 < 1.0f);

				int32 indexA = c->GetChildIndexA();
				int32 indexB = c->GetChildIndexB();

				// Compute the time of impact in interval [0, minTOI]
				b2TOIInput input;
				input.proxyA.Set(fA->GetShape(), indexA);
				input.proxyB.Set(fB->GetShape(), indexB);
				input.sweepA = bA->m_sweep;
				input.sweepB = bB->m_sweep;
				input.tMax = 1.0f;
				//b2CLDevice::instance().finishCommandQueue();

				b2TOIOutput output;
				b2TimeOfImpact(&output, &input);
				//totalComputeTOITimes ++ ; 


				// Beta is the fraction of the remaining portion of the .
				float32 beta = output.t;
				if (output.state == b2TOIOutput::e_touching)
				{
					alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
				}
				else
				{
					alpha = 1.0f;
				}

				c->m_toi = alpha;
				c->m_flags |= b2Contact::e_toiFlag;
			}

			if (alpha < minAlpha)
			{
				// This is the minimum TOI found so far.
				minContact = c;
				minAlpha = alpha;
			}
		}

		if (minContact == NULL || 1.0f - 10.0f * b2_epsilon < minAlpha)
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}


		// Advance the bodies to the TOI.

		b2Fixture* fA = minContact->GetFixtureA();
		b2Fixture* fB = minContact->GetFixtureB();
		b2Body* bA = fA->GetBody();
		b2Body* bB = fB->GetBody();

		b2Sweep backup1 = bA->m_sweep;
		b2Sweep backup2 = bB->m_sweep;

		bA->Advance(minAlpha);
		bB->Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		minContact->Update(m_contactManager.m_contactListener);
		minContact->m_flags &= ~b2Contact::e_toiFlag;
		++minContact->m_toiCount;

		// Is the contact solid?
		if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
		{
			// Restore the sweeps.
			minContact->SetEnabled(false);
			bA->m_sweep = backup1;
			bB->m_sweep = backup2;
			bA->SynchronizeTransform();
			bB->SynchronizeTransform();
			continue;
		}

		bA->SetAwake(true);
		bB->SetAwake(true);

		// Build the island
		island.Clear();
		island.Add(bA);
		island.Add(bB);
		island.Add(minContact);

		bA->m_flags |= b2Body::e_islandFlag;
		bB->m_flags |= b2Body::e_islandFlag;
		minContact->m_flags |= b2Contact::e_islandFlag;

		// Get contacts on bodyA and bodyB.
		b2Body* bodies[2] = {bA, bB};
		for (int32 i = 0; i < 2; ++i)
		{
			b2Body* body = bodies[i];
			if (body->m_type == b2_dynamicBody)
			{
				for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
				{
					if (island.m_bodyCount == island.m_bodyCapacity)
					{
						break;
					}

					if (island.m_contactCount == island.m_contactCapacity)
					{
						break;
					}

					b2Contact* contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->m_flags & b2Contact::e_islandFlag)
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					b2Body* other = ce->other;
					if (other->m_type == b2_dynamicBody &&
						body->IsBullet() == false && other->IsBullet() == false)
					{
						continue;
					}

					// Skip sensors.
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Tentatively advance the body to the TOI.
					b2Sweep backup = other->m_sweep;
					if ((other->m_flags & b2Body::e_islandFlag) == 0)
					{
						other->Advance(minAlpha);
					}

					// Update the contact points
					contact->Update(m_contactManager.m_contactListener);

					// Was the contact disabled by the user?
					if (contact->IsEnabled() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Are there contact points?
					if (contact->IsTouching() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Add the contact to the island
					contact->m_flags |= b2Contact::e_islandFlag;
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other->m_flags & b2Body::e_islandFlag)
					{
						continue;
					}
					
					// Add the other body to the island.
					other->m_flags |= b2Body::e_islandFlag;

					if (other->m_type != b2_staticBody)
					{
						other->SetAwake(true);
					}

					island.Add(other);
				}
			}
		}

		b2TimeStep subStep;
		subStep.dt = (1.0f - minAlpha) * step.dt;
		subStep.inv_dt = 1.0f / subStep.dt;
		subStep.dtRatio = 1.0f;
		subStep.positionIterations = 20;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;

		island.SolveTOI(subStep, bA->m_islandIndex, bB->m_islandIndex);

		// Reset island flags and synchronize broad-phase proxies.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			b2Body* body = island.m_bodies[i];
			body->m_flags &= ~b2Body::e_islandFlag;

			if (body->m_type != b2_dynamicBody)
			{
				continue;
			}

			body->SynchronizeFixtures();
			//movedBodyIndexSet.insert (body->m_uid); 
#if defined (BOX2D_OPENCL)
			movedBodySet.insert(body);
#endif
			// Invalidate all contact TOIs on this displaced body.
			for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
			{
				ce->contact->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			}
			//m_b2clSolverTOI.CopyUpdatedBodytoDevice( body ) ; 
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		//m_contactManager.FindNewContacts();
#if defined (BOX2D_OPENCL)
		m_contactManager.cpuFindNewContacts () ; 
#else
		m_contactManager.FindNewContacts();
#endif
		//m_contactManager.cpuOverLap () ;  

		if (m_subStepping)
		{
			m_stepComplete = false;
			break;
		}
	}



#if defined (BOX2D_OPENCL)
	//b2CLCommonData::instance().CopyStaticBodyAttributes (this) ; 
	//m_b2clSolverTOI.CopyUpdatedBodytoDevice( this->GetBodyList() , &(this->m_b2clSolver) );



	m_b2clSolverTOI.syncMovedBodytoDevice( movedBodySet , &(this->m_b2clSolver) ); 
	//b2CLCommonData::instance().CopyDynamicBodyAttributes(this) ; 
	//m_b2clSolverTOI.CopyAllBodytoDevice( this->GetBodyList() , &(this->m_b2clSolver),this->m_bodyCount ); 

	m_contactManager.FindNewContacts();
	//m_contactManager.cpuOverLap () ;
#endif

#if defined(_DEBUG_TIME_STEP_TOTAL)
			b2CLDevice::instance().finishCommandQueue();
			stepTOIPhaseTime += findTOITimer.GetMilliseconds();
#endif

}




void b2World::InitializeGPUData()
{
#if defined(NARROWPHASE_OPENCL)
	// if user use Listener, update fixture_address and fixture_child when needed
	if (m_uListenerCallback)
	{
		if (m_bBodyChanged || m_bFixtureChanged)
		{
			b2CLCommonData::instance().UpdateFixtureAddressAndChildren(this);
		}
	}

	// copy static data (invariant for all steps) here
	// dynamic data (changing in each step) will be copied in each modual saperately
	if (b2clGlobal_OpenCLSupported)
	{
		if (m_bBodyChanged || m_bFixtureChanged)
		{
			b2CLCommonData::instance().CopyShapes(this);
			//cout << "Copy shapes data." << endl;
		}

	#if defined(SOLVER_OPENCL)
		if (m_bFixtureChanged)
		{
			b2CLCommonData::instance().CopyStaticFixtureAttributes(this);
			//cout << "Copy static fixture data." << endl;
		}

		// copy static body info used by constraint solver
		if (m_bBodyChanged)
		{
			b2CLCommonData::instance().CopyStaticBodyAttributes(this);
			//cout << "Copy static body data." << endl;
			bFirstFrame = true;
			//m_bBodyChanged = false;
		}
		else if (m_bBodyStaticAttributeChanged)
		{
			b2CLCommonData::instance().CopyStaticBodyAttributes(this);
			m_bBodyStaticAttributeChanged = false;
			//cout << "Copy static body data." << endl;
		}

		// copy dynamic body info used by constraint solver
		if (bFirstFrame || m_bBodyDynamicAttributeChanged)
		{
            //printf("bFirstFrame: %d, m_bBodyDynamicAttributeChanged: %d\n", bFirstFrame, m_bBodyDynamicAttributeChanged);
			b2CLCommonData::instance().CopyDynamicBodyAttributes(this);
		}

		// for joint
        if (m_bJointChanged || m_bJointUpdated)
		{
			b2CLCommonData::instance().CopyJoints(this, m_warmStarting, m_bJointChanged, m_bJointUpdated);
			m_bJointChanged = false;
			m_bJointUpdated = false;
		}
	#endif
	}
#else
	// set m_uid for debug Listener
	int body_index = 0;
	int fixture_index = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		for (b2Fixture* f = b->m_fixtureList; f; f = f->m_next)
		{
			b2PolygonShape *s = (b2PolygonShape*)(f->m_shape);

			for (int i=0; i<s->GetChildCount(); i++)
			{
				f->m_uid = fixture_index;
				fixture_index++;
			}
		}
		b->m_uid = body_index;
		body_index++;
	}
#endif
}

void b2World::Step(float32 dt, int32 velocityIterations, int32 positionIterations)
{
    //printf("\nBeginning of Step.\n");
#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2Timer stepTotalTimer;
#endif



	InitializeGPUData();

	// If new fixtures were added, we need to find the new contacts.
	//if (m_flags & e_newFixture)
    if (m_bBodyChanged || m_bFixtureChanged)
	{
		//printf("===FindNewContacts (in Step)===\n");
        
		m_contactManager.FindNewContacts();
		m_flags &= ~e_newFixture;
	}

    if (m_bBodyChanged)
    {
        m_bBodyChanged = false;
    }
    if (m_bFixtureChanged)
    {
        m_bFixtureChanged = false;
    }
    
	m_flags |= e_locked;

	b2TimeStep step;
	step.dt = dt;
	step.velocityIterations	= velocityIterations;
	step.positionIterations = positionIterations;
	if (dt > 0.0f)
	{
		step.inv_dt = 1.0f / dt;
	}
	else
	{
		step.inv_dt = 0.0f;
	}

	step.dtRatio = m_inv_dt0 * dt;

	step.warmStarting = m_warmStarting;

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2Timer stepNarrowPhaseTimer;
#endif
	

	// Update contacts. This is where some contacts are destroyed.
	// Original Box2D do Collide unconditionally, i.e. even when step.dt==0.0f.
	// Do not know why. Removed it to make OCL code running correctly for single step case.
	if (step.dt > 0.0f)
	{
 
		//printf("===Collide===\n");

		m_contactManager.Collide();
		//printf("===Collide finished===\n");
	}

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2CLDevice::instance().finishCommandQueue();
	stepNarrowPhaseTime += stepNarrowPhaseTimer.GetMilliseconds();
	b2Timer stepSolverTimer;
#endif

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (step.dt > 0.0f)
	{
		//printf("===Solve===\n");
		Solve(step);
		//printf("===Solve finished===\n");
	}

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2CLDevice::instance().finishCommandQueue();
	stepSolverTime += stepSolverTimer.GetMilliseconds();
#endif

  
	// Handle TOI events.
	if (m_continuousPhysics && step.dt > 0.0f)
	{
		//printf("===SolveTOI===\n");
		SolveTOI(step);
	}

	if (step.dt > 0.0f)
	{
		m_inv_dt0 = step.inv_dt;
	}

	if (m_flags & e_clearForces)
	{
		ClearForces();
	}

	m_flags &= ~e_locked;

#if defined(_DEBUG_TIME_STEP_TOTAL)
	b2CLDevice::instance().finishCommandQueue();
	stepTotalTime += stepTotalTimer.GetMilliseconds();
#endif
	 
#ifdef _DEBUG_TIME
	numLoops++;
	if (numLoops>=TOTAL_NUM_LOOP)
	{
		if (bOutputToFile)
		{
#if defined (_WIN32)
			f_output = fopen("c:\\Timedata.csv", "w");
#else
			f_output = fopen("/Users/g.rong/Timedata.csv", "w");
#endif
		#if defined(_DEBUG_TIME_STEP_TOTAL)
			fprintf(f_output, "Body#, Joint#, , Total step Time, Step NP time, Step Solver time, Step BP Time, ");
		#endif
		#if defined(_DEBUG_TIME_BROADPHASE)
			fprintf(f_output, ", Ave. GPU BP Time, Create GPU Buffer, Compute AABB, Sort AABB, Compute Pair, ");
		#endif
		#if defined(_DEBUG_TIME_NARROWPHASE)
			fprintf(f_output, ", Ave. Contact#, Ave. GPU NP Time, Init. GPU data, Computation, Readback GPU data, ");
		#endif
		#if defined(_DEBUG_TIME_SOLVER)
			fprintf(f_output, ", Ave. Valid Contact#, Ave. GPU Solver Time, Init. CPU, Init GPU, Computation, Finalize, ");
		#endif
			fprintf(f_output, "\n");
		}

		printf("\n");
	#if defined(_DEBUG_TIME_STEP_TOTAL)
		stepTotalTime /= TOTAL_NUM_LOOP;
		stepNarrowPhaseTime /= TOTAL_NUM_LOOP;
		stepSolverTime /= TOTAL_NUM_LOOP;
		stepBroadPhaseTime /= TOTAL_NUM_LOOP;
		stepColorTime /= TOTAL_NUM_LOOP ; 
		stepTOIPhaseTime /= TOTAL_NUM_LOOP ; 

		printf("Total step time is %f ms.\n", stepTotalTime);
		printf("\tStep Narrow Phase is %f ms.\n", stepNarrowPhaseTime);
		printf("\tStep Solver is %f ms.\n", stepSolverTime-stepBroadPhaseTime);
		printf("\tStep Broad Phase is %f ms.\n", stepBroadPhaseTime);
		//printf("\tStep Color Phase is %f ms.\n", stepColorTime);
		printf ("\tStep TOI Phase is %f ms. \n", stepTOIPhaseTime);
		printf("========================================\n");

		if (bOutputToFile)
		{
			fprintf(f_output, "%d, %d, , %f, %f, %f, %f, ", m_bodyCount, m_jointCount, stepTotalTime, stepNarrowPhaseTime, stepSolverTime-stepBroadPhaseTime, stepBroadPhaseTime);
		}

		stepTotalTime = 0;
		stepNarrowPhaseTime = stepSolverTime = stepBroadPhaseTime = stepColorTime = stepTOIPhaseTime = 0;
	#endif

	#if defined(_DEBUG_TIME_BROADPHASE)
		updatePairsTime /= TOTAL_NUM_LOOP;
		CreateGPUBuffersTime /= TOTAL_NUM_LOOP;
		ComputeAABBsTime /= TOTAL_NUM_LOOP;
		SortAABBsTime /= TOTAL_NUM_LOOP;
		ComputePairsTime /= TOTAL_NUM_LOOP;

		printf("Average time of GPU Broad Phase is %f ms.\n", updatePairsTime);
		printf("\tAverage time of CreateGPUBuffers is %f ms.\n", CreateGPUBuffersTime);
		printf("\tAverage time of ComputeAABBs is %f ms.\n", ComputeAABBsTime);
		printf("\tAverage time of SortAABBs is %f ms.\n", SortAABBsTime);
		printf("\tAverage time of ComputePairs is %f ms.\n", ComputePairsTime);
		
		if (bOutputToFile)
		{
			fprintf(f_output, ", %f, %f, %f, %f, %f, ", updatePairsTime, CreateGPUBuffersTime, ComputeAABBsTime,
				SortAABBsTime, ComputePairsTime);
		}

		updatePairsTime = 0;
		CreateGPUBuffersTime = ComputeAABBsTime = 
			SortAABBsTime = ComputePairsTime = 0;
	#endif

	#if defined(_DEBUG_TIME_NARROWPHASE)
		ave_contact_num /= TOTAL_NUM_LOOP;
		updateContactTime /= TOTAL_NUM_LOOP;
		InitializeGPUDataTime /= TOTAL_NUM_LOOP;
		UpdateContactPairsTime /= TOTAL_NUM_LOOP;
		ReadbackGPUDataTime /= TOTAL_NUM_LOOP;

		printf("Average contact#: %f.\n", ave_contact_num);
		printf("Average time of GPU Narrow Phase is %f ms.\n", updateContactTime);
		printf("\tAverage time of InitializeGPUData is %f ms.\n", InitializeGPUDataTime);
		printf("\tAverage time of UpdateContactPairs is %f ms.\n", UpdateContactPairsTime);
		printf("\tAverage time of ReadbackGPUData is %f ms.\n", ReadbackGPUDataTime);
		
		if (bOutputToFile)
		{
			fprintf(f_output, ", %f, %f, %f, %f, %f, ", ave_contact_num, updateContactTime, InitializeGPUDataTime,
				UpdateContactPairsTime, ReadbackGPUDataTime);
		}

		ave_contact_num = 0;
		updateContactTime = 0;
		InitializeGPUDataTime = UpdateContactPairsTime = 
			ReadbackGPUDataTime = 0;
	#endif

	#if defined(_DEBUG_TIME_SOLVER)
		ave_valid_contact_num /= TOTAL_NUM_LOOP;
		solverTotalTime /= TOTAL_NUM_LOOP;
		constraintInitCPUTime /= TOTAL_NUM_LOOP;
		constraintInitGPUTime /= TOTAL_NUM_LOOP;
		constraintSolveTime /= TOTAL_NUM_LOOP;
		constraintFinalizeTime /= TOTAL_NUM_LOOP;

		printf("Average valid contact#: %f.\n", ave_valid_contact_num);
		printf("Average time of GPU Constraints Solver is %f ms.\n", solverTotalTime);
		printf("\tAverage time of initialize constratins on CPU is %f ms.\n", constraintInitCPUTime);
		printf("\tAverage time of initialize constratins on GPU is %f ms.\n", constraintInitGPUTime);
		printf("\tAverage time of solver computation is %f ms.\n", constraintSolveTime);
		printf("\tAverage time of finalize solver is %f ms.\n", constraintFinalizeTime);
		printf("\tAverage time of test solver is %f ms.\n", testSolveTime / TOTAL_NUM_LOOP);
		testSolveTime = 0;

		// for debug
		printf("\t=====Debug Timing Data=======\n");
		testSolveVelocityConstraintTime /= TOTAL_NUM_LOOP;
		testSolvePositionConstraintTime /= TOTAL_NUM_LOOP;
		testSolveTestPositionConstraintTime /= TOTAL_NUM_LOOP;
		printf("\tAverage time of solve velocity contratins is %f ms.\n", testSolveVelocityConstraintTime);
		printf("\tAverage time of test solve position contratins is %f ms.\n", testSolveTestPositionConstraintTime);
		printf("\tAverage time of solve position contratins is %f ms.\n", testSolvePositionConstraintTime);
		testSolveVelocityConstraintTime = testSolvePositionConstraintTime = testSolveTestPositionConstraintTime = 0;

		if (bOutputToFile)
		{
			fprintf(f_output, ", %f, %f, %f, %f, %f, %f, ", ave_valid_contact_num, solverTotalTime,
				constraintInitCPUTime, constraintInitGPUTime, constraintSolveTime, constraintFinalizeTime);
		}

		numLoops = 0;
		ave_valid_contact_num = 0;
		solverTotalTime = 0;
		constraintInitCPUTime = constraintInitGPUTime = 
			constraintSolveTime = constraintFinalizeTime = 0;
	#endif

		numLoops = 0;

		if (bOutputToFile)
		{
			bOutputToFile = false;
			fclose(f_output);
			printf("Data output finished.\n");
			printf("bodies/joints = %d/%d\n", m_bodyCount, m_jointCount);
			//exit(0);
		}
	}
    //printf("======Print time finished======\n");
#endif
}

void b2World::OutputToFile()
{
#if defined(_DEBUG_TIME)
	bOutputToFile = true;
#endif
}

float b2World::getTotalTime () {
#ifdef _DEBUG_TIME
	if (numLoops == 0) return stepTotalTime ; 
	return stepTotalTime/numLoops ; 
#endif
return 0 ; 
}

float b2World::getNarrowPhaseTime () {
#ifdef _DEBUG_TIME
	if (numLoops == 0) return stepNarrowPhaseTime ; 
	return stepNarrowPhaseTime/numLoops ; 
#endif
return 0 ; 
}

float b2World::getSolverTime () {
#ifdef _DEBUG_TIME
	if (numLoops == 0) return stepSolverTime; 
	return stepSolverTime/numLoops ; 
#endif
return 0 ; 
}

void b2World::ClearForces()
{
	for (b2Body* body = m_bodyList; body; body = body->GetNext())
	{
		if (body->m_force.x!=0 || body->m_force.y!=0)
		{
			body->m_force.SetZero();
			m_bBodyDynamicAttributeChanged = true;
		}
		if (body->m_torque!=0)
		{
			body->m_torque = 0.0f;
			m_bBodyDynamicAttributeChanged = true;
		}
	}
}

struct b2WorldQueryWrapper
{
	bool QueryCallback(int32 proxyId)
	{
#ifndef BROADPHASE_OPENCL
		b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
		return callback->ReportFixture(proxy->fixture);
#else
		b2Fixture* fixture = b2CLCommonData::instance().GetFixture(proxyId);
		return callback->ReportFixture(fixture);
#endif
	}

	const b2BroadPhase* broadPhase;
	b2QueryCallback* callback;
};

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
#if defined (BROADPHASE_OPENCL)
	m_contactManager.m_broadPhase.SetWorld(this);
#endif
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

struct b2WorldRayCastWrapper
{
	float32 RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		void* userData = broadPhase->GetUserData(proxyId);
		b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
		b2Fixture* fixture = proxy->fixture;
		int32 index = proxy->childIndex;
		b2RayCastOutput output;
		bool hit = fixture->RayCast(&output, input, index);

		if (hit)
		{
			float32 fraction = output.fraction;
			b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(fixture, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	const b2BroadPhase* broadPhase;
	b2RayCastCallback* callback;
};

void b2World::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.p1 = point1;
	input.p2 = point2;
#if defined (BROADPHASE_OPENCL)
	m_contactManager.m_broadPhase.SetWorld(this);
#endif
	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}

void b2World::DrawShape(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{
	switch (fixture->GetType())
	{
	case b2Shape::e_circle:
		{
			b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

			b2Vec2 center = b2Mul(xf, circle->m_p);
			float32 radius = circle->m_radius;
			b2Vec2 axis = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

			m_debugDraw->DrawSolidCircle(center, radius, axis, color);
		}
		break;

	case b2Shape::e_edge:
		{
			b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();
			b2Vec2 v1 = b2Mul(xf, edge->m_vertex1);
			b2Vec2 v2 = b2Mul(xf, edge->m_vertex2);
			m_debugDraw->DrawSegment(v1, v2, color);
		}
		break;

	case b2Shape::e_chain:
		{
			b2ChainShape* chain = (b2ChainShape*)fixture->GetShape();
			int32 count = chain->m_count;
			const b2Vec2* vertices = chain->m_vertices;

			b2Vec2 v1 = b2Mul(xf, vertices[0]);
			for (int32 i = 1; i < count; ++i)
			{
				b2Vec2 v2 = b2Mul(xf, vertices[i]);
				m_debugDraw->DrawSegment(v1, v2, color);
				m_debugDraw->DrawCircle(v1, 0.05f, color);
				v1 = v2;
			}
		}
		break;

	case b2Shape::e_polygon:
		{
			b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
			int32 vertexCount = poly->m_vertexCount;
			b2Assert(vertexCount <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32 i = 0; i < vertexCount; ++i)
			{
				vertices[i] = b2Mul(xf, poly->m_vertices[i]);
			}

			m_debugDraw->DrawSolidPolygon(vertices, vertexCount, color);
		}
		break;
            
    default:
        break;
	}
}

void b2World::DrawJoint(b2Joint* joint)
{
	b2Body* bodyA = joint->GetBodyA();
	b2Body* bodyB = joint->GetBodyB();
	const b2Transform& xf1 = bodyA->GetTransform();
	const b2Transform& xf2 = bodyB->GetTransform();
	b2Vec2 x1 = xf1.p;
	b2Vec2 x2 = xf2.p;
	b2Vec2 p1 = joint->GetAnchorA();
	b2Vec2 p2 = joint->GetAnchorB();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (joint->GetType())
	{
	case e_distanceJoint:
		m_debugDraw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
		{
			b2PulleyJoint* pulley = (b2PulleyJoint*)joint;
			b2Vec2 s1 = pulley->GetGroundAnchorA();
			b2Vec2 s2 = pulley->GetGroundAnchorB();
			m_debugDraw->DrawSegment(s1, p1, color);
			m_debugDraw->DrawSegment(s2, p2, color);
			m_debugDraw->DrawSegment(s1, s2, color);
		}
		break;

	case e_mouseJoint:
		// don't draw this
		break;

	default:
		m_debugDraw->DrawSegment(x1, p1, color);
		m_debugDraw->DrawSegment(p1, p2, color);
		m_debugDraw->DrawSegment(x2, p2, color);
	}
}

void b2World::DrawDebugData()
{
	if (m_debugDraw == NULL)
	{
		return;
	}

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b2Draw::e_shapeBit)
	{
		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			const b2Transform& xf = b->GetTransform();
			for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
			{
				if (b->colors[0]<0)
				{
					if (b->IsActive() == false)
					{
						DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
					}
					else if (b->GetType() == b2_staticBody)
					{
						DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
						//DrawShape(f, xf, b2Color(0.0f, 1.0f, 0.0f));
					}
					else if (b->GetType() == b2_kinematicBody)
					{
						DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
					}
					else if (b->IsAwake() == false)
					{
						DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
					}
					else
					{
						DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
						//DrawShape(f, xf, b2Color(0.9f, 0.0f, 0.0f));
					}
				}
				else
				{
					DrawShape(f, xf, b2Color(b->colors[0], b->colors[1], b->colors[2]));
				}
			}
		}
	}

	if (flags & b2Draw::e_jointBit)
	{
		for (b2Joint* j = m_jointList; j; j = j->GetNext())
		{
			DrawJoint(j);
		}
	}

	if (flags & b2Draw::e_pairBit)
	{
		b2Color color(0.3f, 0.9f, 0.9f);
		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
		{
			//b2Fixture* fixtureA = c->GetFixtureA();
			//b2Fixture* fixtureB = c->GetFixtureB();

			//b2Vec2 cA = fixtureA->GetAABB().GetCenter();
			//b2Vec2 cB = fixtureB->GetAABB().GetCenter();

			//m_debugDraw->DrawSegment(cA, cB, color);
		}
	}

	if (flags & b2Draw::e_aabbBit)
	{
		b2Color color(0.9f, 0.3f, 0.9f);
		b2BroadPhase* bp = &m_contactManager.m_broadPhase;

		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			if (b->IsActive() == false)
			{
				continue;
			}

			for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext())
			{
				for (int32 i = 0; i < f->m_proxyCount; ++i)
				{
					b2FixtureProxy* proxy = f->m_proxies + i;
					b2AABB aabb = bp->GetFatAABB(proxy->proxyId);
					b2Vec2 vs[4];
					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

					m_debugDraw->DrawPolygon(vs, 4, color);
				}
			}
		}
	}

	if (flags & b2Draw::e_centerOfMassBit)
	{
		for (b2Body* b = m_bodyList; b; b = b->GetNext())
		{
			b2Transform xf = b->GetTransform();
			xf.p = b->GetWorldCenter();
			m_debugDraw->DrawTransform(xf);
		}
	}
}

int32 b2World::GetProxyCount() const
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

int32 b2World::GetTreeHeight() const
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

int32 b2World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

float32 b2World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void b2World::Dump()
{
	if ((m_flags & e_locked) == e_locked)
	{
		return;
	}

	b2Log("b2Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
	b2Log("m_world->SetGravity(g);\n");

	b2Log("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Log("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);
	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_islandIndex = i;
		b->Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Log("{\n");
		j->Dump();
		b2Log("}\n");
	}

	b2Log("b2Free(joints);\n");
	b2Log("b2Free(bodies);\n");
	b2Log("joints = NULL;\n");
	b2Log("bodies = NULL;\n");
}
