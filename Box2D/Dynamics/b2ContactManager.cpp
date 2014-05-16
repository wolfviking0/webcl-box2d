/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>

b2ContactFilter b2_defaultFilter;
b2ContactListener b2_defaultListener;

b2ContactManager::b2ContactManager()
{
	m_contactList = NULL;
	m_contactCount = 0;
	m_pContactCounts = new int[b2Shape::contact_type_num];
	memset(m_pContactCounts, 0, sizeof(int)*b2Shape::contact_type_num);
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = NULL;
}

b2ContactManager::~b2ContactManager()
{
    delete [] m_pContactCounts;
}

void b2ContactManager::Destroy(b2Contact* c)
{
	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	if (m_contactListener && c->IsTouching())
	{
		m_contactListener->EndContact(c);
	}

	// Remove from the world.
	if (c->m_prev)
	{
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next)
	{
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList)
	{
		m_contactList = c->m_next;
	}

	// Remove from body 1
	if (c->m_nodeA.prev)
	{
		c->m_nodeA.prev->next = c->m_nodeA.next;
	}

	if (c->m_nodeA.next)
	{
		c->m_nodeA.next->prev = c->m_nodeA.prev;
	}

	if (&c->m_nodeA == bodyA->m_contactList)
	{
		bodyA->m_contactList = c->m_nodeA.next;
	}

	// Remove from body 2
	if (c->m_nodeB.prev)
	{
		c->m_nodeB.prev->next = c->m_nodeB.next;
	}

	if (c->m_nodeB.next)
	{
		c->m_nodeB.next->prev = c->m_nodeB.prev;
	}

	if (&c->m_nodeB == bodyB->m_contactList)
	{
		bodyB->m_contactList = c->m_nodeB.next;
	}

	// Call the factory.
	b2Contact::Destroy(c, m_allocator);
	--m_contactCount;
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
void b2ContactManager::Collide()
{
#ifdef _DEBUG_TIME_NARROWPHASE
	m_pWorld->ave_contact_num += m_contactCount;
#endif

#if defined(BROADPHASE_OPENCL)
	if (!b2clGlobal_OpenCLSupported)
#endif
	{
		// Update awake contacts.
		b2Contact* c = m_contactList;
		while (c)
		{
			b2Fixture* fixtureA = c->GetFixtureA();
			b2Fixture* fixtureB = c->GetFixtureB();
			int32 indexA = c->GetChildIndexA();
			int32 indexB = c->GetChildIndexB();
			b2Body* bodyA = fixtureA->GetBody();
			b2Body* bodyB = fixtureB->GetBody();

			// Is this contact flagged for filtering?
			if (c->m_flags & b2Contact::e_filterFlag)
			{
				// Should these bodies collide?
				if (bodyB->ShouldCollide(bodyA) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c->m_flags &= ~b2Contact::e_filterFlag;
			}

			bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
			bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false)
			{
				c = c->GetNext();
				continue;
			}

			int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
			int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
			bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false)
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}

	#if defined(NARROWPHASE_OPENCL)
			if (!b2clGlobal_OpenCLSupported)
	#endif
			{
			#ifdef _DEBUG_TIME_NARROWPHASE
				b2Timer narrowPhaseTimer;
			#endif
				// The contact persists.
				c->Update(m_contactListener);

			#ifdef _DEBUG_TIME_NARROWPHASE
				m_pWorld->updateContactTime += narrowPhaseTimer.GetMilliseconds();
			#endif
			}

			c = c->GetNext();
		}
	}

#if defined(NARROWPHASE_OPENCL)
	if (b2clGlobal_OpenCLSupported)
	{
	#ifdef _DEBUG_TIME_NARROWPHASE
		b2Timer narrowPhaseTimer, InitializeGPUDataTimer;
	#endif
	
	//#if defined(BROADPHASE_OPENCL)
	//	m_cl_narrowPhase.CreateXfBuffer(m_pWorld);
	//#endif

		//printf("contact count in NarrowPhase: %d\n", m_pWorld->m_contactManager.m_contactCount);
		if (m_pWorld->m_contactManager.m_contactCount > 0)
		{
			//printf("NP: InitializeGPUData.\n");
			m_cl_narrowPhase.InitializeGPUData(m_pWorld, m_contactList, m_pContactCounts);

		#ifdef _DEBUG_TIME_NARROWPHASE
			b2CLDevice::instance().finishCommandQueue();
			m_pWorld->InitializeGPUDataTime += InitializeGPUDataTimer.GetMilliseconds();
			b2Timer UpdateContactPairsTimer;
		#endif

			//printf("NP: UpdateContactPairs.\n");
		#if defined(BROADPHASE_OPENCL)
			m_cl_narrowPhase.UpdateContactPairs(m_contactCount, m_pContactCounts, m_maxContactCount/*m_contactListener*/);
		#else
			m_cl_narrowPhase.UpdateContactPairs(m_contactCount, m_pContactCounts, m_contactCount/*m_contactListener*/);
		#endif

			if (m_pWorld->m_uListenerCallback)
			{
				int *enableBitArray = NULL;
				if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
				{
					enableBitArray = new int[m_contactCount];
				}
				int validContactCount;
				m_cl_narrowPhase.ReadbackGPUDataForListener(m_pWorld, &m_contactList, m_contactListener, enableBitArray, &validContactCount);
				b2Contact* pc = m_contactList;
				while (pc)
				{
					if ((pc->m_flags & 0x40)) // the bit is still set means it is no longer valid and need to be removed
					{
						b2Contact* cNuke = pc;
						pc = cNuke->GetNext();

						int temp = m_contactCount;
						Destroy(cNuke);
						m_contactCount = temp; // m_contactCount is from b2cl Narrow Phase, and thus should not be affected by Destroy(cNuke)

						continue;
					}
					pc = pc->GetNext();
				}
				//// for debug
				//printf("contact: %d, valid: %d\n", m_contactCount, validContactCount);
				//for (int i=0; i<m_contactCount; i++)
				//{
					//if (i<validContactCount && enableBitArray[i]<=0)
					//	assert(0);
					//printf("%d, ", enableBitArray[i]);
				//}
				//printf("\n");

				if (m_pWorld->m_uListenerCallback & b2ContactListener::l_PreSolve)
				{
					b2CLDevice::instance().copyArrayToDevice(b2CLCommonData::instance().manifoldBinaryBitListBuffer, enableBitArray, 0, sizeof(int) * m_contactCount);

					//// compact enabled contacts
					//m_cl_narrowPhase.CompactEnabledContactPairs(m_contactCount);

					delete [] enableBitArray;
				}
			}

		#if defined(SOLVER_OPENCL)
			//printf("NP: CompactContactPairs.\n");
			m_cl_narrowPhase.CompactContactPairs(m_contactCount);
			//printf("NP: CompactContactPairs finished.\n");
		#endif

		#ifdef _DEBUG_TIME_NARROWPHASE
			b2CLDevice::instance().finishCommandQueue();
			m_pWorld->UpdateContactPairsTime += UpdateContactPairsTimer.GetMilliseconds();
			b2Timer ReadbackGPUDataTimer;
		#endif

		#if !defined(SOLVER_OPENCL)
			// ReadbackGPUDataForListener already read back GPU data.
			// So only call ReadbackGPUData if Listener is NOT used.
			// NOTE: ReadbackGPUData may not work correctly now!!! Check inside!!!
			if (!m_pWorld->m_uListenerCallback)
				m_cl_narrowPhase.ReadbackGPUData(m_pWorld, m_contactList, m_contactListener);
		#endif

		#ifdef _DEBUG_TIME_NARROWPHASE
			b2CLDevice::instance().finishCommandQueue();
			m_pWorld->ReadbackGPUDataTime += ReadbackGPUDataTimer.GetMilliseconds();
			m_pWorld->updateContactTime += narrowPhaseTimer.GetMilliseconds();
		#endif
		}

	}
#endif
}


void b2ContactManager::cpuCollide()
{
		// Update awake contacts.
		b2Contact* c = m_contactList;
		while (c)
		{
			b2Fixture* fixtureA = c->GetFixtureA();
			b2Fixture* fixtureB = c->GetFixtureB();
			int32 indexA = c->GetChildIndexA();
			int32 indexB = c->GetChildIndexB();
			b2Body* bodyA = fixtureA->GetBody();
			b2Body* bodyB = fixtureB->GetBody();

			// Is this contact flagged for filtering?
			if (c->m_flags & b2Contact::e_filterFlag)
			{
				// Should these bodies collide?
				if (bodyB->ShouldCollide(bodyA) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c->m_flags &= ~b2Contact::e_filterFlag;
			}

			bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
			bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false)
			{
				c = c->GetNext();
				continue;
			}

			int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
			int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
			bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false)
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}

			{
			#ifdef _DEBUG_TIME_NARROWPHASE
				b2Timer narrowPhaseTimer;
			#endif
				// The contact persists.
				c->Update(m_contactListener);

			#ifdef _DEBUG_TIME_NARROWPHASE
				m_pWorld->updateContactTime += narrowPhaseTimer.GetMilliseconds();
			#endif
			}

			c = c->GetNext();
		}

}

void b2ContactManager::cpuOverLap()
{
		// Update awake contacts.
		b2Contact* c = m_contactList;
		while (c)
		{
			b2Fixture* fixtureA = c->GetFixtureA();
			b2Fixture* fixtureB = c->GetFixtureB();
			int32 indexA = c->GetChildIndexA();
			int32 indexB = c->GetChildIndexB();
			b2Body* bodyA = fixtureA->GetBody();
			b2Body* bodyB = fixtureB->GetBody();

			// Is this contact flagged for filtering?
			if (c->m_flags & b2Contact::e_filterFlag)
			{
				// Should these bodies collide?
				if (bodyB->ShouldCollide(bodyA) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
				{
					b2Contact* cNuke = c;
					c = cNuke->GetNext();
					Destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c->m_flags &= ~b2Contact::e_filterFlag;
			}

			bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
			bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

			// At least one body must be awake and it must be dynamic or kinematic.
			if (activeA == false && activeB == false)
			{
				c = c->GetNext();
				continue;
			}

			int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
			int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
			bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false)
			{
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}
/*
			{
			#ifdef _DEBUG_TIME_NARROWPHASE
				b2Timer narrowPhaseTimer;
			#endif
				// The contact persists.
				c->Update(m_contactListener);

			#ifdef _DEBUG_TIME_NARROWPHASE
				m_pWorld->updateContactTime += narrowPhaseTimer.GetMilliseconds();
			#endif
			}
*/
			c = c->GetNext();
		}

}

void b2ContactManager::FindNewContacts()
{
#if defined (_DEBUG_TIME_BROADPHASE)
	m_broadPhase.SetTimePointers(&(m_pWorld->updatePairsTime), 
		&(m_pWorld->CreateGPUBuffersTime), 
		&(m_pWorld->ComputeAABBsTime), 
		&(m_pWorld->SortAABBsTime), 
		&(m_pWorld->ComputePairsTime));
#endif

	//m_pWorld->InitializeGPUData();
	m_broadPhase.SetValues(m_pWorld->GetFixtureCount(), &m_contactCount, m_pContactCounts);
	m_maxContactCount = m_pWorld->GetProxyCount() * MAX_CONTACT_PER_FIXTURE;
    //printf("shape number: %d\n", m_pWorld->GetFixtureCount());
	m_broadPhase.UpdatePairs(this);
}


void b2ContactManager::cpuFindNewContacts()
{
#if defined (_DEBUG_TIME_BROADPHASE)
	m_broadPhase.SetTimePointers(&(m_pWorld->updatePairsTime), 
		&(m_pWorld->CreateGPUBuffersTime), 
		&(m_pWorld->ComputeAABBsTime), 
		&(m_pWorld->SortAABBsTime), 
		&(m_pWorld->ComputePairsTime));
#endif

	m_broadPhase.SetValues(m_pWorld->GetFixtureCount(), &m_contactCount, m_pContactCounts);
	m_maxContactCount = m_pWorld->GetProxyCount() * MAX_CONTACT_PER_FIXTURE;
    //printf("shape number: %d\n", m_pWorld->GetFixtureCount());
	m_broadPhase.cpuUpdatePairs(this);
}

void b2ContactManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
{
	b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
	b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;

	b2Fixture* fixtureA = proxyA->fixture;
	b2Fixture* fixtureB = proxyB->fixture;

	int32 indexA = proxyA->childIndex;
	int32 indexB = proxyB->childIndex;

	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	// Are the fixtures on the same body?
	if (bodyA == bodyB)
	{
		return;
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	b2ContactEdge* edge = bodyB->GetContactList();
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
				return;
			}

			if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
			{
				// A contact already exists.
				return;
			}
		}

		edge = edge->next;
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (bodyB->ShouldCollide(bodyA) == false)
	{
		return;
	}

	// Check user filtering.
	if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
	{
		return;
	}

	// Call the factory.
	b2Contact* c = b2Contact::Create(fixtureA, indexA, fixtureB, indexB, m_allocator);
	if (c == NULL)
	{
		return;
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
	c->m_next = m_contactList;
	if (m_contactList != NULL)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

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

	// Wake up the bodies
	bodyA->SetAwake(true);
	bodyB->SetAwake(true);

	++m_contactCount;
}
