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
#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include <Box2D/Collision/b2BroadPhase.h>
#include <Box2D/Common/OpenCL/b2CLNarrowPhase.h>

class b2Contact;
class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;

// Delegate of b2World.
class b2ContactManager
{
public:
	b2ContactManager();
    ~b2ContactManager();

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts();

	void cpuFindNewContacts () ;
	void cpuCollide () ; 
	void cpuOverLap() ; 

	void Destroy(b2Contact* c);

	void Collide();
            
	void SetWorldPointer(b2World *p) { m_pWorld = p; }

	b2BroadPhase m_broadPhase;
	b2Contact* m_contactList;
	int32 *m_pContactCounts, m_contactCount, m_maxContactCount;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;

#if defined(NARROWPHASE_OPENCL)
	b2CLNarrowPhase m_cl_narrowPhase;
#endif

private:
	b2World* m_pWorld;
};

#endif
