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
#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include <Box2D/Common/OpenCL/b2CLDevice.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2DynamicTree.h>
#include <algorithm>

#define MAX_CONTACT_PER_FIXTURE 10

class b2World;
class b2CLBroadPhase;

struct b2Pair
{
	int32 proxyIdA;
	int32 proxyIdB;
	int32 next;
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class b2BroadPhase
{
public:

	enum
	{
		e_nullProxy = -1
	};

	b2BroadPhase();
	~b2BroadPhase();

	/// Create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	int32 CreateProxy(const b2AABB& aabb, void* userData);

	/// Destroy a proxy. It is up to the client to remove any pairs.
	void DestroyProxy(int32 proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	void TouchProxy(int32 proxyId);

	/// Get the fat AABB for a proxy.
	const b2AABB& GetFatAABB(int32 proxyId) const;

	/// Get user data from a proxy. Returns NULL if the id is invalid.
	void* GetUserData(int32 proxyId) const;

	/// Test overlap of fat AABBs.
	bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;

	/// Get the number of proxies.
	int32 GetProxyCount() const;
    
#ifdef BROADPHASE_OPENCL
	bool UseListener() const;
#endif

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
	template <typename T>
	void UpdatePairs(T* callback);
	template <typename T>
	void cpuUpdatePairs (T* callback); 

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const b2AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input) const;

	/// Get the height of the embedded tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the embedded tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the embedded tree.
	float32 GetTreeQuality() const;

#ifdef BROADPHASE_OPENCL
	void SetWorld(const b2World* pWorld) const { m_pWorld = pWorld; }
#endif

	void SetValues(int fixtureCount, int *pTotalContactCount, int *pContactCounts) 
	{ 
#ifdef BROADPHASE_OPENCL
		if (b2clGlobal_OpenCLSupported)
		{
			m_fixtureCount = fixtureCount; 
			m_pTotalContactCount = pTotalContactCount; 
			m_pContactCounts = pContactCounts;
		}
#endif
	}

	void SetTimePointers(float *pUpdatePairsTime, float *pCreateGPUBuffersTime, float *pComputeAABBsTime, float *pSortAABBsTime, float *pComputePairsTime)
	{
#if defined (_DEBUG_TIME_BROADPHASE)
		m_pUpdatePairsTime = pUpdatePairsTime;
		m_pCreateGPUBuffersTime = pCreateGPUBuffersTime;
		m_pComputeAABBsTime = pComputeAABBsTime;
		m_pSortAABBsTime = pSortAABBsTime;
		m_pComputePairsTime = pComputePairsTime;
#endif
	}

private:

	friend class b2DynamicTree;
	friend class b2World;

	void BufferMove(int32 proxyId);
	void UnBufferMove(int32 proxyId);

	bool QueryCallback(int32 proxyId);

	b2DynamicTree m_tree;

	int32 m_proxyCount;

	int32* m_moveBuffer;
	int32 m_moveCapacity;
	int32 m_moveCount;

	b2Pair* m_pairBuffer;
	int32 m_pairCapacity;
	int32 m_pairCount;

	int32 m_queryProxyId;

#ifdef BROADPHASE_OPENCL
    b2CLBroadPhase *m_pCL_broadPhase;
	int m_fixtureCount;
	int *m_pTotalContactCount, *m_pContactCounts;
	mutable const b2World *m_pWorld;
#endif

#if defined (_DEBUG_TIME_BROADPHASE)
	float *m_pUpdatePairsTime;
	float *m_pCreateGPUBuffersTime, *m_pComputeAABBsTime, *m_pSortAABBsTime, *m_pComputePairsTime;
#endif
};

/// This is used to sort pairs.
inline bool b2PairLessThan(const b2Pair& pair1, const b2Pair& pair2)
{
	if (pair1.proxyIdA < pair2.proxyIdA)
	{
		return true;
	}

	if (pair1.proxyIdA == pair2.proxyIdA)
	{
		return pair1.proxyIdB < pair2.proxyIdB;
	}

	return false;
}

inline void* b2BroadPhase::GetUserData(int32 proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

inline bool b2BroadPhase::TestOverlap(int32 proxyIdA, int32 proxyIdB) const
{
	const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return b2TestOverlap(aabbA, aabbB);
}

inline const b2AABB& b2BroadPhase::GetFatAABB(int32 proxyId) const
{
	return m_tree.GetFatAABB(proxyId);
}

inline int32 b2BroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}

inline int32 b2BroadPhase::GetTreeHeight() const
{
	return m_tree.GetHeight();
}

inline int32 b2BroadPhase::GetTreeBalance() const
{
	return m_tree.GetMaxBalance();
}

inline float32 b2BroadPhase::GetTreeQuality() const
{
	return m_tree.GetAreaRatio();
}

#include <Box2D/Common/OpenCL/b2CLBroadPhase.h>


template <typename T>
void b2BroadPhase::cpuUpdatePairs(T* callback)
{
	m_pairCount = 0;

		// Perform tree queries for all moving proxies.
		for (int32 i = 0; i < m_moveCount; ++i)
		{
			m_queryProxyId = m_moveBuffer[i];
			if (m_queryProxyId == e_nullProxy)
			{
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

			// Query tree, create pairs and add them pair buffer.
			m_tree.Query(this, fatAABB);
		}

		// Reset move buffer
		m_moveCount = 0;

		// Sort the pair buffer to expose duplicates.
		std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, b2PairLessThan);

		// Send the pairs back to the client.
		int32 i = 0;
		while (i < m_pairCount)
		{
			b2Pair* primaryPair = m_pairBuffer + i;
			void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
			void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

			callback->AddPair(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs.
			while (i < m_pairCount)
			{
				b2Pair* pair = m_pairBuffer + i;
				if (pair->proxyIdA != primaryPair->proxyIdA || pair->proxyIdB != primaryPair->proxyIdB)
				{
					break;
				}
				++i;
			}
		}
}



template <typename T>
void b2BroadPhase::UpdatePairs(T* callback)
{
#ifdef _DEBUG_TIME_BROADPHASE
	b2Timer updatePairsTimer, CreateGPUBuffersTimer;
#endif

#if defined(BROADPHASE_OPENCL)
	if (!b2clGlobal_OpenCLSupported)
#endif
	{
		// Reset pair buffer
		m_pairCount = 0;

		// Perform tree queries for all moving proxies.
		for (int32 i = 0; i < m_moveCount; ++i)
		{
			m_queryProxyId = m_moveBuffer[i];
			if (m_queryProxyId == e_nullProxy)
			{
				continue;
			}

			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

			// Query tree, create pairs and add them pair buffer.
			m_tree.Query(this, fatAABB);
		}

		// Reset move buffer
		m_moveCount = 0;

		// Sort the pair buffer to expose duplicates.
		std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, b2PairLessThan);

		// Send the pairs back to the client.
		int32 i = 0;
		while (i < m_pairCount)
		{
			b2Pair* primaryPair = m_pairBuffer + i;
			void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
			void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

			callback->AddPair(userDataA, userDataB);
			++i;

			// Skip any duplicate pairs.
			while (i < m_pairCount)
			{
				b2Pair* pair = m_pairBuffer + i;
				if (pair->proxyIdA != primaryPair->proxyIdA || pair->proxyIdB != primaryPair->proxyIdB)
				{
					break;
				}
				++i;
			}
		}

		// Try to keep the tree balanced.
		//m_tree.Rebalance(4);
	}
#if defined(BROADPHASE_OPENCL)
	else
	{
		//if (m_pWorld->frame_num==108)
		//	int a = 1;

		//printf("BP: Create GPU Buffers.\n");
		m_pCL_broadPhase->CreateGPUBuffers(m_proxyCount);

	#ifdef _DEBUG_TIME_BROADPHASE
		b2CLDevice::instance().finishCommandQueue();
		*m_pCreateGPUBuffersTime += CreateGPUBuffersTimer.GetMilliseconds();
		b2Timer ComputeAABBsTimer;
	#endif

		//printf("BP: Compute AABBs.\n");
		m_pCL_broadPhase->ComputeAABBs(m_proxyCount);
		//m_pCL_broadPhase->ComputeAABBsTOI(m_proxyCount);

	#ifdef _DEBUG_TIME_BROADPHASE
		b2CLDevice::instance().finishCommandQueue();
		*m_pComputeAABBsTime += ComputeAABBsTimer.GetMilliseconds();
		b2Timer SortAABBsTimer;
	#endif

		m_pCL_broadPhase->PrepareSumVariance(m_proxyCount);

		m_pCL_broadPhase->InitSortingKeys(m_proxyCount);

		//printf("BP: Sort AABBs.\n");
		m_pCL_broadPhase->SortAABBs(m_proxyCount);

	#ifdef _DEBUG_TIME_BROADPHASE
		b2CLDevice::instance().finishCommandQueue();
		*m_pSortAABBsTime += SortAABBsTimer.GetMilliseconds();
		b2Timer ComputePairsTimer;
	#endif

		//printf("BP: Compute Pairs.\n");
		//printf("BP: fixtureCount: %d.\n", m_fixtureCount);

		m_pCL_broadPhase->ComputePairs(m_proxyCount, m_pTotalContactCount, m_pContactCounts, m_pWorld);
		//m_pCL_broadPhase->ComputePairsNoAtomic(m_proxyCount, m_pTotalContactCount, m_pContactCounts);

		//printf("BP: Compute Pairs finished.\n");
		//printf("BP: contactCount: %d.\n", *m_pContactCount);

	#ifdef _DEBUG_TIME_BROADPHASE
		b2CLDevice::instance().finishCommandQueue();
		*m_pComputePairsTime += ComputePairsTimer.GetMilliseconds();
		*m_pUpdatePairsTime += updatePairsTimer.GetMilliseconds();
	#endif
	}    
#endif
}

template <typename T>
inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb) const
{
#if defined(BROADPHASE_OPENCL)
	if (!b2clGlobal_OpenCLSupported)
#endif
	{
		m_tree.Query(callback, aabb);
	}
#if defined(BROADPHASE_OPENCL)
	else
	{
		m_pCL_broadPhase->Query(callback, aabb, m_proxyCount, UseListener());
	}
#endif
}

template <typename T>
inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
{
#if defined(BROADPHASE_OPENCL)
	if (!b2clGlobal_OpenCLSupported)
#endif
	{
		m_tree.RayCast(callback, input);
	}
#if defined(BROADPHASE_OPENCL)
	else
	{
		m_pCL_broadPhase->RayCast(callback, input, m_proxyCount, UseListener());
	}
#endif
}

#endif
