// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "constants.h"
#include "types.h"

#include <assert.h>

#define b2_defaultCategoryBits (0x00000001)
#define b2_defaultMaskBits (0xFFFFFFFF)

#ifdef __cplusplus
extern "C"
{
#endif

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
typedef struct b2DynamicTree
{
	struct b2TreeNode* nodes;
	int32_t root;
	int32_t nodeCount;
	int32_t nodeCapacity;
	int32_t freeList;
	int32_t insertionCount;
} b2DynamicTree;

/// Constructing the tree initializes the node pool.
b2DynamicTree b2DynamicTree_Create();

/// Destroy the tree, freeing the node pool.
void b2DynamicTree_Destroy(b2DynamicTree* tree);

/// Create a proxy. Provide a tight fitting AABB and a userData pointer.
int32_t b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, uint32_t categoryBits, void* userData);

/// Destroy a proxy. This asserts if the id is invalid.
void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int32_t proxyId);

/// Move a proxy with a swepted AABB. If the proxy has moved outside of its
/// fattened AABB, then the proxy is removed from the tree and re-inserted.
/// Otherwise the function returns immediately.
/// @return true if the proxy was re-inserted.
bool b2DynamicTree_MoveProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb1);

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool b2QueryCallbackFcn(int32_t proxyId, void* userData, void* context);

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
void b2DynamicTree_Query(const b2DynamicTree* tree, b2AABB aabb, uint32_t maskBits, b2QueryCallbackFcn* callback,
                         void* context);

/// This function receives clipped raycast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2RayCastCallbackFcn(const b2RayCastInput* input, int32_t proxyId, void* userData, void* context);

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param callback a callback class that is called for each proxy that is hit by the ray.
void b2DynamicTree_RayCast(const b2DynamicTree* tree, const b2RayCastInput* input, uint32_t maskBits,
                           b2RayCastCallbackFcn* callback, void* context);

/// Validate this tree. For testing.
void b2DynamicTree_Validate(const b2DynamicTree* tree);

/// Compute the height of the binary tree in O(N) time. Should not be
/// called often.
int32_t b2DynamicTree_GetHeight(const b2DynamicTree* tree);

/// Get the maximum balance of an node in the tree. The balance is the difference
/// in height of the two children of a node.
int32_t b2DynamicTree_GetMaxBalance(const b2DynamicTree* tree);

/// Get the ratio of the sum of the node areas to the root area.
float b2DynamicTree_GetAreaRatio(const b2DynamicTree* tree);

/// Build an optimal tree. Very expensive. For testing.
void b2DynamicTree_RebuildBottomUp(b2DynamicTree* tree);

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
void b2DynamicTree_ShiftOrigin(b2DynamicTree* tree, b2Vec2 newOrigin);

/// Get proxy user data.
/// @return the proxy user data or 0 if the id is invalid.
void* b2DynamicTree_GetUserData(const b2DynamicTree* tree, int32_t proxyId);

bool b2DynamicTree_WasMoved(const b2DynamicTree* tree, int32_t proxyId);

void b2DynamicTree_ClearMoved(b2DynamicTree* tree, int32_t proxyId);

/// Get the fat AABB for a proxy.
b2AABB b2DynamicTree_GetFatAABB(const b2DynamicTree* tree, int32_t proxyId);

#ifdef __cplusplus
}
#endif
