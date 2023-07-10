// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/dynamic_tree.h"

#include "allocate.h"
#include "box2d/aabb.h"
#include "box2d/constants.h"

#include <float.h>
#include <string.h>

// A node in the dynamic tree. The client does not interact with this directly.
// TODO_ERIN
//union
//{
//	void* userData;
//	struct
//	{
//		int32_t child1;
//		int32_t child2;
//	} Children;
//};
typedef struct b2TreeNode
{
	void* userData;

	// Enlarged AABB
	b2AABB aabb;

	uint32_t categoryBits;

	union
	{
		int32_t parent;
		int32_t next;
	};

	int32_t child1;
	int32_t child2;

	// leaf = 0, free node = -1
	int32_t height;

	bool moved;
} b2TreeNode;

static inline bool b2IsLeaf(const b2TreeNode* node)
{
	return node->height == B2_NULL_INDEX;
}

b2DynamicTree b2DynamicTree_Create()
{
	b2DynamicTree tree;
	tree.root = B2_NULL_INDEX;

	tree.nodeCapacity = 16;
	tree.nodeCount = 0;
	tree.nodes = (b2TreeNode*)b2Alloc(tree.nodeCapacity * sizeof(b2TreeNode));
	memset(tree.nodes, 0, tree.nodeCapacity * sizeof(b2TreeNode));

	// Build a linked list for the free list.
	for (int32_t i = 0; i < tree.nodeCapacity - 1; ++i)
	{
		tree.nodes[i].next = i + 1;
		tree.nodes[i].height = -1;
	}
	tree.nodes[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
	tree.nodes[tree.nodeCapacity - 1].height = -1;
	tree.freeList = 0;

	tree.proxyCount = 0;

	return tree;
}

void b2DynamicTree_Destroy(b2DynamicTree* tree)
{
	b2Free(tree->nodes, tree->nodeCapacity * sizeof(b2TreeNode));
	memset(tree, 0, sizeof(b2DynamicTree));
}

// Allocate a node from the pool. Grow the pool if necessary.
static int32_t b2AllocateNode(b2DynamicTree* tree)
{
	// Expand the node pool as needed.
	if (tree->freeList == B2_NULL_INDEX)
	{
		assert(tree->nodeCount == tree->nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = tree->nodes;
		int32_t oldCapcity = tree->nodeCapacity;
		tree->nodeCapacity += oldCapcity >> 1;
		tree->nodes = (b2TreeNode*)b2Alloc(tree->nodeCapacity * sizeof(b2TreeNode));
		memcpy(tree->nodes, oldNodes, tree->nodeCount * sizeof(b2TreeNode));
		b2Free(oldNodes, oldCapcity * sizeof(b2TreeNode));

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (int32_t i = tree->nodeCount; i < tree->nodeCapacity - 1; ++i)
		{
			tree->nodes[i].next = i + 1;
			tree->nodes[i].height = -1;
		}
		tree->nodes[tree->nodeCapacity - 1].next = B2_NULL_INDEX;
		tree->nodes[tree->nodeCapacity - 1].height = -1;
		tree->freeList = tree->nodeCount;
	}

	// Peel a node off the free list.
	int32_t nodeId = tree->freeList;
	b2TreeNode* node = tree->nodes + nodeId;
	tree->freeList = node->next;
	node->parent = B2_NULL_INDEX;
	node->child1 = B2_NULL_INDEX;
	node->child2 = B2_NULL_INDEX;
	node->categoryBits = 0;
	node->height = 0;
	node->userData = 0;
	node->moved = false;
	++tree->nodeCount;
	return nodeId;
}

// Return a node to the pool.
static void b2FreeNode(b2DynamicTree* tree, int32_t nodeId)
{
	assert(0 <= nodeId && nodeId < tree->nodeCapacity);
	assert(0 < tree->nodeCount);
	tree->nodes[nodeId].next = tree->freeList;
	tree->nodes[nodeId].height = -1;
	tree->freeList = nodeId;
	--tree->nodeCount;
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
int32_t b2BalanceNode(b2DynamicTree* tree, int32_t iA)
{
	assert(iA != B2_NULL_INDEX);

	b2TreeNode* A = tree->nodes + iA;
	if (b2IsLeaf(A) || A->height < 2)
	{
		return iA;
	}

	int32_t iB = A->child1;
	int32_t iC = A->child2;
	assert(0 <= iB && iB < tree->nodeCapacity);
	assert(0 <= iC && iC < tree->nodeCapacity);

	b2TreeNode* B = tree->nodes + iB;
	b2TreeNode* C = tree->nodes + iC;

	int32_t balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int32_t iF = C->child1;
		int32_t iG = C->child2;
		b2TreeNode* F = tree->nodes + iF;
		b2TreeNode* G = tree->nodes + iG;
		assert(0 <= iF && iF < tree->nodeCapacity);
		assert(0 <= iG && iG < tree->nodeCapacity);

		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != B2_NULL_INDEX)
		{
			if (tree->nodes[C->parent].child1 == iA)
			{
				tree->nodes[C->parent].child1 = iC;
			}
			else
			{
				assert(tree->nodes[C->parent].child2 == iA);
				tree->nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			tree->root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb = b2AABB_Union(B->aabb, G->aabb);
			C->aabb = b2AABB_Union(A->aabb, F->aabb);

			A->categoryBits = B->categoryBits | G->categoryBits;
			C->categoryBits = A->categoryBits | F->categoryBits;

			A->height = 1 + B2_MAX(B->height, G->height);
			C->height = 1 + B2_MAX(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb = b2AABB_Union(B->aabb, F->aabb);
			C->aabb = b2AABB_Union(A->aabb, G->aabb);

			A->categoryBits = B->categoryBits | F->categoryBits;
			C->categoryBits = A->categoryBits | G->categoryBits;

			A->height = 1 + B2_MAX(B->height, F->height);
			C->height = 1 + B2_MAX(A->height, G->height);
		}

		return iC;
	}

	// Rotate B up
	if (balance < -1)
	{
		int32_t iD = B->child1;
		int32_t iE = B->child2;
		b2TreeNode* D = tree->nodes + iD;
		b2TreeNode* E = tree->nodes + iE;
		assert(0 <= iD && iD < tree->nodeCapacity);
		assert(0 <= iE && iE < tree->nodeCapacity);

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != B2_NULL_INDEX)
		{
			if (tree->nodes[B->parent].child1 == iA)
			{
				tree->nodes[B->parent].child1 = iB;
			}
			else
			{
				assert(tree->nodes[B->parent].child2 == iA);
				tree->nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			tree->root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb = b2AABB_Union(C->aabb, E->aabb);
			B->aabb = b2AABB_Union(A->aabb, D->aabb);

			A->categoryBits = C->categoryBits | E->categoryBits;
			B->categoryBits = A->categoryBits | D->categoryBits;

			A->height = 1 + B2_MAX(C->height, E->height);
			B->height = 1 + B2_MAX(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb = b2AABB_Union(C->aabb, D->aabb);
			B->aabb = b2AABB_Union(A->aabb, E->aabb);

			A->categoryBits = C->categoryBits | D->categoryBits;
			B->categoryBits = A->categoryBits | E->categoryBits;

			A->height = 1 + B2_MAX(C->height, D->height);
			B->height = 1 + B2_MAX(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

static void b2InsertLeaf(b2DynamicTree* tree, int32_t leaf)
{
	if (tree->root == B2_NULL_INDEX)
	{
		tree->root = leaf;
		tree->nodes[tree->root].parent = B2_NULL_INDEX;
		return;
	}

	// Find the best sibling for this node
	b2AABB leafAABB = tree->nodes[leaf].aabb;
	int32_t index = tree->root;
	while (b2IsLeaf(tree->nodes + index) == false)
	{
		int32_t child1 = tree->nodes[index].child1;
		int32_t child2 = tree->nodes[index].child2;

		float area = b2AABB_Perimeter(tree->nodes[index].aabb);

		b2AABB combinedAABB = b2AABB_Union(tree->nodes[index].aabb, leafAABB);
		float combinedArea = b2AABB_Perimeter(combinedAABB);

		// Cost of creating a new parent for this node and the new leaf
		float cost = 2.0f * combinedArea;

		// Minimum cost of pushing the leaf further down the tree
		float inheritanceCost = 2.0f * (combinedArea - area);

		// Cost of descending into child1
		float cost1;
		if (b2IsLeaf(tree->nodes + child1))
		{
			b2AABB aabb = b2AABB_Union(leafAABB, tree->nodes[child1].aabb);
			cost1 = b2AABB_Perimeter(aabb) + inheritanceCost;
		}
		else
		{
			b2AABB aabb = b2AABB_Union(leafAABB, tree->nodes[child1].aabb);
			float oldArea = b2AABB_Perimeter(tree->nodes[child1].aabb);
			float newArea = b2AABB_Perimeter(aabb);
			cost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending into child2
		float cost2;
		if (b2IsLeaf(tree->nodes + child2))
		{
			b2AABB aabb = b2AABB_Union(leafAABB, tree->nodes[child2].aabb);
			cost2 = b2AABB_Perimeter(aabb) + inheritanceCost;
		}
		else
		{
			b2AABB aabb = b2AABB_Union(leafAABB, tree->nodes[child2].aabb);
			float oldArea = b2AABB_Perimeter(tree->nodes[child2].aabb);
			float newArea = b2AABB_Perimeter(aabb);
			cost2 = newArea - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if (cost < cost1 && cost < cost2)
		{
			break;
		}

		// Descend
		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

	int32_t sibling = index;

	// Create a new parent.
	int32_t oldParent = tree->nodes[sibling].parent;
	int32_t newParent = b2AllocateNode(tree);
	tree->nodes[newParent].parent = oldParent;
	tree->nodes[newParent].userData = 0;
	tree->nodes[newParent].aabb = b2AABB_Union(leafAABB, tree->nodes[sibling].aabb);
	tree->nodes[newParent].categoryBits = tree->nodes[leaf].categoryBits | tree->nodes[sibling].categoryBits;
	tree->nodes[newParent].height = tree->nodes[sibling].height + 1;

	if (oldParent != B2_NULL_INDEX)
	{
		// The sibling was not the root.
		if (tree->nodes[oldParent].child1 == sibling)
		{
			tree->nodes[oldParent].child1 = newParent;
		}
		else
		{
			tree->nodes[oldParent].child2 = newParent;
		}

		tree->nodes[newParent].child1 = sibling;
		tree->nodes[newParent].child2 = leaf;
		tree->nodes[sibling].parent = newParent;
		tree->nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		tree->nodes[newParent].child1 = sibling;
		tree->nodes[newParent].child2 = leaf;
		tree->nodes[sibling].parent = newParent;
		tree->nodes[leaf].parent = newParent;
		tree->root = newParent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = tree->nodes[leaf].parent;
	while (index != B2_NULL_INDEX)
	{
		index = b2BalanceNode(tree, index);

		int32_t child1 = tree->nodes[index].child1;
		int32_t child2 = tree->nodes[index].child2;

		assert(child1 != B2_NULL_INDEX);
		assert(child2 != B2_NULL_INDEX);

		tree->nodes[index].aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);
		tree->nodes[index].categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
		tree->nodes[index].height = 1 + B2_MAX(tree->nodes[child1].height, tree->nodes[child2].height);

		index = tree->nodes[index].parent;
	}

	// Validate();
}

static void b2RemoveLeaf(b2DynamicTree* tree, int32_t leaf)
{
	if (leaf == tree->root)
	{
		tree->root = B2_NULL_INDEX;
		return;
	}

	int32_t parent = tree->nodes[leaf].parent;
	int32_t grandParent = tree->nodes[parent].parent;
	int32_t sibling;
	if (tree->nodes[parent].child1 == leaf)
	{
		sibling = tree->nodes[parent].child2;
	}
	else
	{
		sibling = tree->nodes[parent].child1;
	}

	if (grandParent != B2_NULL_INDEX)
	{
		// Destroy parent and connect sibling to grandParent.
		if (tree->nodes[grandParent].child1 == parent)
		{
			tree->nodes[grandParent].child1 = sibling;
		}
		else
		{
			tree->nodes[grandParent].child2 = sibling;
		}
		tree->nodes[sibling].parent = grandParent;
		b2FreeNode(tree, parent);

		// Adjust ancestor bounds.
		int32_t index = grandParent;
		while (index != B2_NULL_INDEX)
		{
			index = b2BalanceNode(tree, index);

			int32_t child1 = tree->nodes[index].child1;
			int32_t child2 = tree->nodes[index].child2;

			tree->nodes[index].aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);
			tree->nodes[index].categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
			tree->nodes[index].height = 1 + B2_MAX(tree->nodes[child1].height, tree->nodes[child2].height);

			index = tree->nodes[index].parent;
		}
	}
	else
	{
		tree->root = sibling;
		tree->nodes[sibling].parent = B2_NULL_INDEX;
		b2FreeNode(tree, parent);
	}

	// Validate();
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
int32_t b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, uint32_t categoryBits, void* userData)
{
	assert(-b2_huge < aabb.lowerBound.x && aabb.lowerBound.x < b2_huge);
	assert(-b2_huge < aabb.lowerBound.y && aabb.lowerBound.y < b2_huge);
	assert(-b2_huge < aabb.upperBound.x && aabb.upperBound.x < b2_huge);
	assert(-b2_huge < aabb.upperBound.y && aabb.upperBound.y < b2_huge);

	int32_t proxyId = b2AllocateNode(tree);
	b2TreeNode* node = tree->nodes + proxyId;

	// Fatten the aabb.
	b2Vec2 r = {b2_aabbExtension, b2_aabbExtension};
	node->aabb.lowerBound = b2Sub(aabb.lowerBound, r);
	node->aabb.upperBound = b2Add(aabb.upperBound, r);
	node->userData = userData;
	node->categoryBits = categoryBits;
	node->height = 0;
	node->moved = true;

	b2InsertLeaf(tree, proxyId);

	tree->proxyCount += 1;

	return proxyId;
}

void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	assert(b2IsLeaf(tree->nodes + proxyId));

	b2RemoveLeaf(tree, proxyId);
	b2FreeNode(tree, proxyId);

	assert(tree->proxyCount > 0);
	tree->proxyCount -= 1;
}

bool b2DynamicTree_MoveProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb)
{
	assert(-b2_huge < aabb.lowerBound.x && aabb.lowerBound.x < b2_huge);
	assert(-b2_huge < aabb.lowerBound.y && aabb.lowerBound.y < b2_huge);
	assert(-b2_huge < aabb.upperBound.x && aabb.upperBound.x < b2_huge);
	assert(-b2_huge < aabb.upperBound.y && aabb.upperBound.y < b2_huge);

	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	assert(b2IsLeaf(tree->nodes + proxyId));

	// Extend AABB
	b2AABB fatAABB;
	b2Vec2 r = {b2_aabbExtension, b2_aabbExtension};
	fatAABB.lowerBound = b2Sub(aabb.lowerBound, r);
	fatAABB.upperBound = b2Add(aabb.upperBound, r);

	b2AABB treeAABB = tree->nodes[proxyId].aabb;
	if (b2AABB_Contains(treeAABB, aabb))
	{
		// The tree AABB still contains the object, but the tree AABB might be too large.
		// Perhaps the object was moving fast but has since gone to sleep.
		// The huge AABB is larger than the new fat AABB.
		b2AABB hugeAABB;
		hugeAABB.lowerBound = b2MulAdd(fatAABB.lowerBound, -4.0f, r);
		hugeAABB.upperBound = b2MulAdd(fatAABB.upperBound, 4.0f, r);

		if (b2AABB_Contains(hugeAABB, treeAABB))
		{
			// The tree AABB contains the object AABB and the tree AABB is
			// not too large. No tree update needed.
			return false;
		}

		// Otherwise the tree AABB is huge and needs to be shrunk
	}

	b2RemoveLeaf(tree, proxyId);

	tree->nodes[proxyId].aabb = fatAABB;

	b2InsertLeaf(tree, proxyId);

	bool alreadyMoved = tree->nodes[proxyId].moved;
	tree->nodes[proxyId].moved = true;

	if (alreadyMoved)
	{
		return false;
	}

	return true;
}

int32_t b2DynamicTree_GetHeight(const b2DynamicTree* tree)
{
	if (tree->root == B2_NULL_INDEX)
	{
		return 0;
	}

	return tree->nodes[tree->root].height;
}

float b2DynamicTree_GetAreaRatio(const b2DynamicTree* tree)
{
	if (tree->root == B2_NULL_INDEX)
	{
		return 0.0f;
	}

	const b2TreeNode* root = tree->nodes + tree->root;
	float rootArea = b2AABB_Perimeter(root->aabb);

	float totalArea = 0.0f;
	for (int32_t i = 0; i < tree->nodeCapacity; ++i)
	{
		const b2TreeNode* node = tree->nodes + i;
		if (node->height < 0 || b2IsLeaf(node) || i == tree->root)
		{
			// Free node in pool
			continue;
		}

		totalArea += b2AABB_Perimeter(node->aabb);
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
static int32_t b2ComputeHeight(const b2DynamicTree* tree, int32_t nodeId)
{
	assert(0 <= nodeId && nodeId < tree->nodeCapacity);
	b2TreeNode* node = tree->nodes + nodeId;

	if (b2IsLeaf(node))
	{
		return 0;
	}

	int32_t height1 = b2ComputeHeight(tree, node->child1);
	int32_t height2 = b2ComputeHeight(tree, node->child2);
	return 1 + B2_MAX(height1, height2);
}

int32_t b2DynamicTree_ComputeHeight(const b2DynamicTree* tree)
{
	int32_t height = b2ComputeHeight(tree, tree->root);
	return height;
}

#if defined(_DEBUG)
static void b2ValidateStructure(const b2DynamicTree* tree, int32_t index)
{
	if (index == B2_NULL_INDEX)
	{
		return;
	}

	if (index == tree->root)
	{
		assert(tree->nodes[index].parent == B2_NULL_INDEX);
	}

	const b2TreeNode* node = tree->nodes + index;

	int32_t child1 = node->child1;
	int32_t child2 = node->child2;

	if (b2IsLeaf(node))
	{
		assert(child1 == B2_NULL_INDEX);
		assert(child2 == B2_NULL_INDEX);
		assert(node->height == 0);
		return;
	}

	assert(0 <= child1 && child1 < tree->nodeCapacity);
	assert(0 <= child2 && child2 < tree->nodeCapacity);

	assert(tree->nodes[child1].parent == index);
	assert(tree->nodes[child2].parent == index);

	b2ValidateStructure(tree, child1);
	b2ValidateStructure(tree, child2);
}

static void b2ValidateMetrics(const b2DynamicTree* tree, int32_t index)
{
	if (index == B2_NULL_INDEX)
	{
		return;
	}

	const b2TreeNode* node = tree->nodes + index;

	int32_t child1 = node->child1;
	int32_t child2 = node->child2;

	if (b2IsLeaf(node))
	{
		assert(child1 == B2_NULL_INDEX);
		assert(child2 == B2_NULL_INDEX);
		assert(node->height == 0);
		return;
	}

	assert(0 <= child1 && child1 < tree->nodeCapacity);
	assert(0 <= child2 && child2 < tree->nodeCapacity);

	int32_t height1 = tree->nodes[child1].height;
	int32_t height2 = tree->nodes[child2].height;
	int32_t height;
	height = 1 + B2_MAX(height1, height2);
	assert(node->height == height);

	b2AABB aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);

	assert(aabb.lowerBound.x == node->aabb.lowerBound.x);
	assert(aabb.lowerBound.y == node->aabb.lowerBound.y);
	assert(aabb.upperBound.x == node->aabb.upperBound.x);
	assert(aabb.upperBound.y == node->aabb.upperBound.y);

	uint32_t categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
	assert(node->categoryBits == categoryBits);

	b2ValidateMetrics(tree, child1);
	b2ValidateMetrics(tree, child2);
}
#endif

void b2DynamicTree_Validate(const b2DynamicTree* tree)
{
#if defined(_DEBUG)
	b2ValidateStructure(tree, tree->root);
	b2ValidateMetrics(tree, tree->root);

	int32_t freeCount = 0;
	int32_t freeIndex = tree->freeList;
	while (freeIndex != B2_NULL_INDEX)
	{
		assert(0 <= freeIndex && freeIndex < tree->nodeCapacity);
		freeIndex = tree->nodes[freeIndex].next;
		++freeCount;
	}

	int32_t height = b2DynamicTree_GetHeight(tree);
	int32_t computedHeight = b2DynamicTree_ComputeHeight(tree);
	assert(height == computedHeight);

	assert(tree->nodeCount + freeCount == tree->nodeCapacity);
#else
	B2_MAYBE_UNUSED(tree);
#endif
}

int32_t b2DynamicTree_GetMaxBalance(const b2DynamicTree* tree)
{
	int32_t maxBalance = 0;
	for (int32_t i = 0; i < tree->nodeCapacity; ++i)
	{
		const b2TreeNode* node = tree->nodes + i;
		if (node->height <= 1)
		{
			continue;
		}

		assert(b2IsLeaf(node) == false);

		int32_t child1 = node->child1;
		int32_t child2 = node->child2;
		int32_t balance = B2_ABS(tree->nodes[child2].height - tree->nodes[child1].height);
		maxBalance = B2_MAX(maxBalance, balance);
	}

	return maxBalance;
}

void b2DynamicTree_RebuildBottomUp(b2DynamicTree* tree)
{
	int32_t* nodes = (int32_t*)b2Alloc(tree->nodeCount * sizeof(int32_t));
	int32_t count = 0;

	// Build array of leaves. Free the rest.
	for (int32_t i = 0; i < tree->nodeCapacity; ++i)
	{
		if (tree->nodes[i].height < 0)
		{
			// free node in pool
			continue;
		}

		if (b2IsLeaf(tree->nodes + i))
		{
			tree->nodes[i].parent = B2_NULL_INDEX;
			nodes[count] = i;
			++count;
		}
		else
		{
			b2FreeNode(tree, i);
		}
	}

	while (count > 1)
	{
		float minCost = FLT_MAX;
		int32_t iMin = -1, jMin = -1;
		for (int32_t i = 0; i < count; ++i)
		{
			b2AABB aabbi = tree->nodes[nodes[i]].aabb;

			for (int32_t j = i + 1; j < count; ++j)
			{
				b2AABB aabbj = tree->nodes[nodes[j]].aabb;
				b2AABB b = b2AABB_Union(aabbi, aabbj);
				float cost = b2AABB_Perimeter(b);
				if (cost < minCost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		int32_t index1 = nodes[iMin];
		int32_t index2 = nodes[jMin];
		b2TreeNode* child1 = tree->nodes + index1;
		b2TreeNode* child2 = tree->nodes + index2;

		int32_t parentIndex = b2AllocateNode(tree);
		b2TreeNode* parent = tree->nodes + parentIndex;
		parent->child1 = index1;
		parent->child2 = index2;
		parent->aabb = b2AABB_Union(child1->aabb, child2->aabb);
		parent->categoryBits = child1->categoryBits | child2->categoryBits;
		parent->height = 1 + B2_MAX(child1->height, child2->height);
		parent->parent = B2_NULL_INDEX;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count - 1];
		nodes[iMin] = parentIndex;
		--count;
	}

	tree->root = nodes[0];
	b2Free(nodes, tree->nodeCount * sizeof(b2TreeNode));

	b2DynamicTree_Validate(tree);
}

void b2DynamicTree_ShiftOrigin(b2DynamicTree* tree, b2Vec2 newOrigin)
{
	// Build array of leaves. Free the rest.
	for (int32_t i = 0; i < tree->nodeCapacity; ++i)
	{
		b2TreeNode* n = tree->nodes + i;
		n->aabb.lowerBound.x -= newOrigin.x;
		n->aabb.lowerBound.y -= newOrigin.y;
		n->aabb.upperBound.x -= newOrigin.x;
		n->aabb.upperBound.y -= newOrigin.y;
	}
}

#define b2_treeStackSize 256

void b2DynamicTree_QueryFiltered(const b2DynamicTree* tree, b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context)
{
	int32_t stack[b2_treeStackSize];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	while (stackCount > 0)
	{
		int32_t nodeId = stack[--stackCount];
		if (nodeId == B2_NULL_INDEX)
		{
			continue;
		}

		const b2TreeNode* node = tree->nodes + nodeId;

		if (b2AABB_Overlaps(node->aabb, aabb) && (node->categoryBits & maskBits) != 0)
		{
			if (b2IsLeaf(node))
			{
				// callback to user code with proxy id
				bool proceed = callback(nodeId, node->userData, context);
				if (proceed == false)
				{
					return;
				}
			}
			else
			{
				assert(stackCount <= b2_treeStackSize - 2);
				// TODO log this?

				if (stackCount <= b2_treeStackSize - 2)
				{
					stack[stackCount++] = node->child1;
					stack[stackCount++] = node->child2;
				}
			}
		}
	}
}

void b2DynamicTree_Query(const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context)
{
	int32_t stack[b2_treeStackSize];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	while (stackCount > 0)
	{
		int32_t nodeId = stack[--stackCount];
		if (nodeId == B2_NULL_INDEX)
		{
			continue;
		}

		const b2TreeNode* node = tree->nodes + nodeId;

		if (b2AABB_Overlaps(node->aabb, aabb))
		{
			if (b2IsLeaf(node))
			{
				// callback to user code with proxy id
				bool proceed = callback(nodeId, node->userData, context);
				if (proceed == false)
				{
					return;
				}
			}
			else
			{
				assert(stackCount <= b2_treeStackSize - 2);
				// TODO log this?

				if (stackCount <= b2_treeStackSize - 2)
				{
					stack[stackCount++] = node->child1;
					stack[stackCount++] = node->child2;
				}
			}
		}
	}
}

void b2DynamicTree_RayCast(const b2DynamicTree* tree, const b2RayCastInput* input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback,
						   void* context)
{
	b2Vec2 p1 = input->p1;
	b2Vec2 p2 = input->p2;
	b2Vec2 extension = {input->radius, input->radius};

	b2Vec2 r = b2Normalize(b2Sub(p2, p1));

	// v is perpendicular to the segment.
	b2Vec2 v = b2CrossSV(1.0f, r);
	b2Vec2 abs_v = b2Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float maxFraction = input->maxFraction;

	// Build a bounding box for the segment.
	b2AABB segmentAABB;
	{
		// t is the endpoint of the ray
		b2Vec2 t = b2MulAdd(p1, maxFraction, b2Sub(p2, p1));

		// Add radius extension
		segmentAABB.lowerBound = b2Sub(b2Min(p1, t), extension);
		segmentAABB.upperBound = b2Sub(b2Max(p1, t), extension);
	}

	int32_t stack[b2_treeStackSize];
	int32_t stackCount = 0;
	stack[stackCount++] = tree->root;

	while (stackCount > 0)
	{
		int32_t nodeId = stack[--stackCount];
		if (nodeId == B2_NULL_INDEX)
		{
			continue;
		}

		const b2TreeNode* node = tree->nodes + nodeId;
		if (b2AABB_Overlaps(node->aabb, segmentAABB) == false || (node->categoryBits & maskBits) == 0)
		{
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		// radius extension is added to the node in this case
		b2Vec2 c = b2AABB_Center(node->aabb);
		b2Vec2 h = b2Add(b2AABB_Extents(node->aabb), extension);
		float term1 = B2_ABS(b2Dot(v, b2Sub(p1, c)));
		float term2 = b2Dot(abs_v, h);
		if (term2 < term1)
		{
			continue;
		}

		if (b2IsLeaf(node))
		{
			b2RayCastInput subInput;
			subInput.p1 = input->p1;
			subInput.p2 = input->p2;
			subInput.maxFraction = maxFraction;

			float value = callback(&subInput, nodeId, node->userData, context);
			assert(value >= 0.0f);

			if (value == 0.0f)
			{
				// The client has terminated the ray cast.
				return;
			}

			if (value < maxFraction)
			{
				// Update segment bounding box.
				maxFraction = value;
				b2Vec2 t = b2MulAdd(p1, maxFraction, b2Sub(p2, p1));
				segmentAABB.lowerBound = b2Sub(b2Min(p1, t), extension);
				segmentAABB.upperBound = b2Sub(b2Max(p1, t), extension);
			}
		}
		else
		{
			assert(stackCount <= b2_treeStackSize - 2);
			// TODO log this?

			if (stackCount <= b2_treeStackSize - 2)
			{
				// TODO_ERIN just put one node on the stack, continue on a child node
				// TODO_ERIN test ordering children by nearest to ray origin
				stack[stackCount++] = node->child1;
				stack[stackCount++] = node->child2;
			}
		}
	}
}

#define B2_BIN_COUNT 64

typedef struct b2TreeBin
{
	b2AABB aabb;
	int32_t count;
} b2TreeBin;

typedef struct b2TreePlane
{
	b2AABB leftAABB;
	b2AABB rightAABB;
	int32_t leftCount;
	int32_t rightCount;
} b2TreePlane;

// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
static int32_t b2BinSortBoxes(b2DynamicTree* tree, int32_t parentIndex, b2TreeNode* leaves, int32_t count, b2TreeBin* bins, b2TreePlane* planes)
{
	if (count == 1)
	{
		leaves[0].parent = parentIndex;
		return (int32_t)(leaves - tree->nodes);
	}

	b2TreeNode* nodes = tree->nodes;

	b2Vec2 center = b2AABB_Center(leaves[0].aabb);
	b2AABB centroidAABB;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for (int32_t i = 1; i < count; ++i)
	{
		center = b2AABB_Center(leaves[i].aabb);
		centroidAABB.lowerBound = b2Min(centroidAABB.lowerBound, center);
		centroidAABB.upperBound = b2Max(centroidAABB.upperBound, center);
	}

	b2Vec2 d = b2Sub(centroidAABB.upperBound, centroidAABB.lowerBound);

	int32_t axisIndex;
	float invD;
	if (d.x > d.y)
	{
		axisIndex = 0;
		invD = d.x;
	}
	else
	{
		axisIndex = 1;
		invD = d.y;
	}

	invD = invD > 0.0f ? 1.0f / invD : 0.0f;

	for (int32_t i = 0; i < B2_BIN_COUNT; ++i)
	{
		bins[i].aabb.lowerBound = (b2Vec2){FLT_MAX, FLT_MAX};
		bins[i].aabb.upperBound = (b2Vec2){-FLT_MAX, -FLT_MAX};
		bins[i].count = 0;
	}

	float binCount = B2_BIN_COUNT;
	float lowerBoundArray[2] = {centroidAABB.lowerBound.x, centroidAABB.lowerBound.y};
	float minC = lowerBoundArray[axisIndex];
	for (int32_t i = 0; i < count; ++i)
	{
		b2Vec2 c = b2AABB_Center(leaves[i].aabb);
		float cArray[2] = {c.x, c.y};
		int32_t binIndex = (int32_t)(binCount * (cArray[axisIndex] - minC) * invD);
		binIndex = B2_CLAMP(binIndex, 0, B2_BIN_COUNT - 1);
		leaves[i].next = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b2AABB_Union(bins[binIndex].aabb, leaves[i].aabb);
	}

	int32_t planeCount = B2_BIN_COUNT - 1;

	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for (int32_t i = 1; i < planeCount; ++i)
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b2AABB_Union(planes[i - 1].leftAABB, bins[i].aabb);
	}

	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for (int32_t i = planeCount - 2; i >= 0; --i)
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b2AABB_Union(planes[i + 1].rightAABB, bins[i + 1].aabb);
	}

	float minCost = FLT_MAX;
	int32_t bestPlane = 0;
	for (int32_t i = 0; i < planeCount; ++i)
	{
		float leftArea = b2AABB_Perimeter(planes[i].leftAABB);
		float rightArea = b2AABB_Perimeter(planes[i].rightAABB);
		int32_t leftCount = planes[i].leftCount;
		int32_t rightCount = planes[i].rightCount;

		float cost = leftCount * leftArea + rightCount * rightArea;
		if (cost < minCost)
		{
			bestPlane = i;
			minCost = cost;
		}
	}

	assert(tree->nodeCount < tree->nodeCapacity);
	int32_t nodeIndex = tree->nodeCount++;
	b2TreeNode* node = nodes + nodeIndex;
	node->aabb = b2AABB_Union(planes[bestPlane].leftAABB, planes[bestPlane].rightAABB);
	node->parent = parentIndex;

	int32_t i1 = -1;
	for (int32_t i2 = 0; i2 < count; ++i2)
	{
		int32_t binIndex = leaves[i2].next;
		if (binIndex <= bestPlane)
		{
			++i1;
			b2TreeNode temp = leaves[i1];
			leaves[i1] = leaves[i2];
			leaves[i2] = temp;
		}
	}

	int32_t leftCount = i1 + 1;
	int32_t rightCount = count - leftCount;

	if (leftCount == 0)
	{
		leftCount = 1;
		rightCount -= 1;
	}
	else if (rightCount == 0)
	{
		leftCount -= 1;
		rightCount = 1;
	}

	// Recurse
	node->child1 = b2BinSortBoxes(tree, nodeIndex, leaves, leftCount, bins, planes);
	node->child2 = b2BinSortBoxes(tree, nodeIndex, leaves + leftCount, rightCount, bins, planes);

	const b2TreeNode* child1 = nodes + node->child1;
	const b2TreeNode* child2 = nodes + node->child2;

	node->height = 1 + B2_MAX(child1->height, child2->height);

	return nodeIndex;
}

int32_t b2DynamicTree_GetProxyCount(const b2DynamicTree* tree)
{
	return tree->proxyCount;
}

void b2DynamicTree_RebuildTopDownSAH(b2DynamicTree* tree, struct b2ProxyMap* mapArray, int32_t mapCount)
{
	B2_MAYBE_UNUSED(mapCount);
	assert(mapCount == tree->proxyCount);

	// need a way to map proxies
	
	int32_t proxyCount = tree->proxyCount;
	int32_t initialCapacity = tree->nodeCapacity;
	
	// Ensure sufficient capacity
	int32_t requiredCapacity = 2 * proxyCount - 1;
	if (initialCapacity < 2 * proxyCount - 1)
	{
		b2TreeNode* oldNodes = tree->nodes;
		int32_t oldCapcity = tree->nodeCapacity;
		tree->nodeCapacity = requiredCapacity + requiredCapacity / 2;
		tree->nodes = (b2TreeNode*)b2Alloc(tree->nodeCapacity * sizeof(b2TreeNode));
		memcpy(tree->nodes, oldNodes, tree->nodeCount * sizeof(b2TreeNode));
		b2Free(oldNodes, oldCapcity * sizeof(b2TreeNode));

		// TODO_ERIN free list built below
	}

	// Copy all leaf nodes to the beginning of the array
	b2TreeNode* nodes = tree->nodes;
	int32_t nodeCount = 0, k = 0;
	while (nodeCount < proxyCount)
	{
		while (nodes[k].height != 0 && k < initialCapacity)
		{
			k += 1;
		}

		if (k == initialCapacity)
		{
			break;
		}

		assert(nodes[k].height == 0);
		nodes[nodeCount] = nodes[k];
		
		// Store proxy index in child2 to remap user ids
		// TODO_ERIN prefer stack allocator so child2 can be put in union
		nodes[nodeCount].child2 = k;

		nodeCount += 1;
	}

	assert(nodeCount == proxyCount);

	tree->freeList = B2_NULL_INDEX;

	b2TreeBin bins[B2_BIN_COUNT];
	b2TreePlane planes[B2_BIN_COUNT - 1];
	tree->root = b2BinSortBoxes(tree, B2_NULL_INDEX, nodes, nodeCount, bins, planes);

	assert(tree->nodeCount == tree->nodeCapacity);

	for (int32_t i = 0; i < tree->nodeCount; ++i)
	{
		b2TreeNode* n = nodes + i;
		assert(0 <= n->child2 && n->child2 < count);
		proxies[n->child2] = i;
		n->child2 = B2_NULL_INDEX;
	}

	b2DynamicTree_Validate(tree);
}


// TODO_ERIN test this as inlined
void* b2DynamicTree_GetUserData(const b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].userData;
}

// TODO_ERIN test this as inlined
bool b2DynamicTree_WasMoved(const b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].moved;
}

// TODO_ERIN test this as inlined
void b2DynamicTree_ClearMoved(b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	tree->nodes[proxyId].moved = false;
}

// TODO_ERIN test this as inlined
b2AABB b2DynamicTree_GetFatAABB(const b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].aabb;
}

// TODO_ERIN test this as inlined
uint32_t b2DynamicTree_GetCategoryBits(const b2DynamicTree* tree, int32_t proxyId)
{
	assert(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].categoryBits;
}
