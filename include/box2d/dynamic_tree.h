// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/constants.h"
#include "box2d/types.h"

#define b2_defaultCategoryBits (0x00000001)
#define b2_defaultMaskBits (0xFFFFFFFF)

/// A node in the dynamic tree. The user does not interact with this directly.
/// 16 + 16 + 8 + pad(8)
typedef struct b2TreeNode
{
	b2AABB aabb; // 16

	// Category bits for collision filtering
	uint32_t categoryBits; // 4

	union
	{
		int32_t parent;
		int32_t next;
	}; // 4

	int32_t child1; // 4
	int32_t child2; // 4

	// todo could be union with child index
	int32_t userData; // 4

	// leaf = 0, free node = -1
	int16_t height; // 2

	bool enlarged; // 1

	char pad[9];
} b2TreeNode;

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as AABB queries and ray casts. Leaf nodes are proxies
/// with an AABB. These are used to hold a user collision object, such as a reference to a b2Shape.
/// Nodes are pooled and relocatable, so I use node indices rather than pointers.
///	The dynamic tree is made available for advanced users that would like to use it to organize
///	spatial game data besides rigid bodies.
typedef struct b2DynamicTree
{
	b2TreeNode* nodes;

	int32_t root;
	int32_t nodeCount;
	int32_t nodeCapacity;
	int32_t freeList;
	int32_t proxyCount;

	int32_t* leafIndices;
	b2AABB* leafBoxes;
	b2Vec2* leafCenters;
	int32_t* binIndices;
	int32_t rebuildCapacity;
} b2DynamicTree;

/// Constructing the tree initializes the node pool.
BOX2D_API b2DynamicTree b2DynamicTree_Create(void);

/// Destroy the tree, freeing the node pool.
BOX2D_API void b2DynamicTree_Destroy(b2DynamicTree* tree);

/// Create a proxy. Provide a tight fitting AABB and a userData value.
BOX2D_API int32_t b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, uint32_t categoryBits, int32_t userData);

/// Destroy a proxy. This asserts if the id is invalid.
BOX2D_API void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int32_t proxyId);

// Clone one tree to another, reusing storage in the outTree if possible
BOX2D_API void b2DynamicTree_Clone(b2DynamicTree* outTree, const b2DynamicTree* inTree);

/// Move a proxy to a new AABB by removing and reinserting into the tree.
BOX2D_API void b2DynamicTree_MoveProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb);

/// Enlarge a proxy and enlarge ancestors as necessary.
BOX2D_API void b2DynamicTree_EnlargeProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb);

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool b2TreeQueryCallbackFcn(int32_t proxyId, int32_t userData, void* context);

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
BOX2D_API void b2DynamicTree_QueryFiltered(const b2DynamicTree* tree, b2AABB aabb, uint32_t maskBits,
										   b2TreeQueryCallbackFcn* callback, void* context);

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
BOX2D_API void b2DynamicTree_Query(const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context);

/// This function receives clipped raycast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeRayCastCallbackFcn(const b2RayCastInput* input, int32_t proxyId, int32_t userData, void* context);

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param callback a callback class that is called for each proxy that is hit by the ray.
BOX2D_API void b2DynamicTree_RayCast(const b2DynamicTree* tree, const b2RayCastInput* input, uint32_t maskBits,
									 b2TreeRayCastCallbackFcn* callback, void* context);

/// This function receives clipped raycast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeShapeCastCallbackFcn(const b2ShapeCastInput* input, int32_t proxyId, int32_t userData, void* context);

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param callback a callback class that is called for each proxy that is hit by the ray.
BOX2D_API void b2DynamicTree_ShapeCast(const b2DynamicTree* tree, const b2ShapeCastInput* input, uint32_t maskBits,
									   b2TreeShapeCastCallbackFcn* callback, void* context);

/// Validate this tree. For testing.
BOX2D_API void b2DynamicTree_Validate(const b2DynamicTree* tree);

/// Compute the height of the binary tree in O(N) time. Should not be
/// called often.
BOX2D_API int32_t b2DynamicTree_GetHeight(const b2DynamicTree* tree);

/// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
BOX2D_API int32_t b2DynamicTree_GetMaxBalance(const b2DynamicTree* tree);

/// Get the ratio of the sum of the node areas to the root area.
BOX2D_API float b2DynamicTree_GetAreaRatio(const b2DynamicTree* tree);

/// Build an optimal tree. Very expensive. For testing.
BOX2D_API void b2DynamicTree_RebuildBottomUp(b2DynamicTree* tree);

/// Get the number of proxies created
BOX2D_API int32_t b2DynamicTree_GetProxyCount(const b2DynamicTree* tree);

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
BOX2D_API int32_t b2DynamicTree_Rebuild(b2DynamicTree* tree, bool fullBuild);

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
BOX2D_API void b2DynamicTree_ShiftOrigin(b2DynamicTree* tree, b2Vec2 newOrigin);

/// Get proxy user data
/// @return the proxy user data or 0 if the id is invalid
static inline int32_t b2DynamicTree_GetUserData(const b2DynamicTree* tree, int32_t proxyId)
{
	return tree->nodes[proxyId].userData;
}

/// Get the AABB of a proxy
static inline b2AABB b2DynamicTree_GetAABB(const b2DynamicTree* tree, int32_t proxyId)
{
	return tree->nodes[proxyId].aabb;
}
