// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/dynamic_tree.h"

#include "allocate.h"
#include "array.h"
#include "core.h"

#include "box2d/aabb.h"
#include "box2d/constants.h"

#include <float.h>
#include <string.h>

// TODO_ERIN
// - try incrementally sorting internal nodes by height for better cache efficiency during depth first traversal.

// A node in the dynamic tree. The client does not interact with this directly.
typedef struct b2TreeNode
{
	// Enlarged AABB
	b2AABB aabb; // 16

	// If we put the most common bits in the first 16 bits, this could be 2 bytes and expanded
	// to 0xFFFF0000 | bits. Then we get partial culling in the tree traversal.
	uint32_t categoryBits; // 4

	union
	{
		int32_t parent;
		int32_t next;
	}; // 4

	int32_t child1; // 4
	int32_t child2; // 4

	int32_t userData; // 4

	// leaf = 0, free node = -1
	// If the height is more than 32k we are in big trouble
	int16_t height; // 2

	bool enlarged; // 1
	bool moved;	   // 1
} b2TreeNode;

static b2TreeNode b2_defaultTreeNode = {
	{{0.0f, 0.0f}, {0.0f, 0.0f}}, 0, {B2_NULL_INDEX}, B2_NULL_INDEX, B2_NULL_INDEX, -1, -2, false, false};

static inline bool b2IsLeaf(const b2TreeNode* node)
{
	// TODO_ERIN this should work with height == 0
	return node->child1 == B2_NULL_INDEX;
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

	tree.leafIndices = NULL;
	tree.leafBoxes = NULL;
	tree.leafCenters = NULL;
	tree.binIndices = NULL;
	tree.rebuildCapacity = 0;

	return tree;
}

void b2DynamicTree_Destroy(b2DynamicTree* tree)
{
	b2Free(tree->nodes, tree->nodeCapacity * sizeof(b2TreeNode));
	b2Free(tree->leafIndices, tree->rebuildCapacity * sizeof(int32_t));
	b2Free(tree->leafBoxes, tree->rebuildCapacity * sizeof(b2AABB));
	b2Free(tree->leafCenters, tree->rebuildCapacity * sizeof(b2Vec2));
	b2Free(tree->binIndices, tree->rebuildCapacity * sizeof(int32_t));

	memset(tree, 0, sizeof(b2DynamicTree));
}

void b2DynamicTree_Clone(b2DynamicTree* outTree, const b2DynamicTree* inTree)
{
	if (outTree->nodeCapacity < inTree->nodeCapacity)
	{
		b2Free(outTree->nodes, outTree->nodeCapacity * sizeof(b2TreeNode));
		outTree->nodeCapacity = inTree->nodeCapacity;
		outTree->nodes = (b2TreeNode*)b2Alloc(outTree->nodeCapacity * sizeof(b2TreeNode));
	}

	memcpy(outTree->nodes, inTree->nodes, inTree->nodeCapacity * sizeof(b2TreeNode));
	outTree->root = inTree->root;
	outTree->nodeCount = inTree->nodeCount;
	outTree->freeList = inTree->freeList;
	outTree->proxyCount = inTree->proxyCount;

	// Hook up free list.
	// TODO_ERIN make this optional?
	// TODO_ERIN perhaps find tail of existing free list and append
	int32_t inCapacity = inTree->nodeCapacity;
	int32_t outCapacity = outTree->nodeCapacity;
	if (outCapacity > inCapacity)
	{
		for (int32_t i = inCapacity; i < outCapacity - 1; ++i)
		{
			outTree->nodes[i].next = i + 1;
			outTree->nodes[i].height = -1;
		}
		outTree->nodes[outCapacity - 1].next = outTree->freeList;
		outTree->nodes[outCapacity - 1].height = -1;
		outTree->freeList = inCapacity;
	}
}

// Allocate a node from the pool. Grow the pool if necessary.
static int32_t b2AllocateNode(b2DynamicTree* tree)
{
	// Expand the node pool as needed.
	if (tree->freeList == B2_NULL_INDEX)
	{
		B2_ASSERT(tree->nodeCount == tree->nodeCapacity);

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
	int32_t nodeIndex = tree->freeList;
	b2TreeNode* node = tree->nodes + nodeIndex;
	tree->freeList = node->next;
	*node = b2_defaultTreeNode;
	++tree->nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
static void b2FreeNode(b2DynamicTree* tree, int32_t nodeId)
{
	B2_ASSERT(0 <= nodeId && nodeId < tree->nodeCapacity);
	B2_ASSERT(0 < tree->nodeCount);
	tree->nodes[nodeId].next = tree->freeList;
	tree->nodes[nodeId].height = -1;
	tree->freeList = nodeId;
	--tree->nodeCount;
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
int32_t b2BalanceNode(b2DynamicTree* tree, int32_t iA)
{
	B2_ASSERT(iA != B2_NULL_INDEX);

	b2TreeNode* A = tree->nodes + iA;
	if (b2IsLeaf(A) || A->height < 2)
	{
		return iA;
	}

	int32_t iB = A->child1;
	int32_t iC = A->child2;
	B2_ASSERT(0 <= iB && iB < tree->nodeCapacity);
	B2_ASSERT(0 <= iC && iC < tree->nodeCapacity);

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
		B2_ASSERT(0 <= iF && iF < tree->nodeCapacity);
		B2_ASSERT(0 <= iG && iG < tree->nodeCapacity);

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
				B2_ASSERT(tree->nodes[C->parent].child2 == iA);
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
		B2_ASSERT(0 <= iD && iD < tree->nodeCapacity);
		B2_ASSERT(0 <= iE && iE < tree->nodeCapacity);

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
				B2_ASSERT(tree->nodes[B->parent].child2 == iA);
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
	tree->nodes[newParent].userData = -1;
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

		B2_ASSERT(child1 != B2_NULL_INDEX);
		B2_ASSERT(child2 != B2_NULL_INDEX);

		tree->nodes[index].aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);
		tree->nodes[index].categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
		tree->nodes[index].height = 1 + B2_MAX(tree->nodes[child1].height, tree->nodes[child2].height);

		index = tree->nodes[index].parent;
	}

	b2DynamicTree_Validate(tree);
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

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
int32_t b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, uint32_t categoryBits, int32_t userData, b2AABB* outFatAABB)
{
	B2_ASSERT(-b2_huge < aabb.lowerBound.x && aabb.lowerBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.lowerBound.y && aabb.lowerBound.y < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.x && aabb.upperBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.y && aabb.upperBound.y < b2_huge);

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
	*outFatAABB = node->aabb;

	b2InsertLeaf(tree, proxyId);

	tree->proxyCount += 1;

	return proxyId;
}

void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	B2_ASSERT(b2IsLeaf(tree->nodes + proxyId));

	b2RemoveLeaf(tree, proxyId);
	b2FreeNode(tree, proxyId);

	B2_ASSERT(tree->proxyCount > 0);
	tree->proxyCount -= 1;
}

int32_t b2DynamicTree_GetProxyCount(const b2DynamicTree* tree)
{
	return tree->proxyCount;
}

bool b2DynamicTree_MoveProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb, b2AABB* outFatAABB)
{
	B2_ASSERT(-b2_huge < aabb.lowerBound.x && aabb.lowerBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.lowerBound.y && aabb.lowerBound.y < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.x && aabb.upperBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.y && aabb.upperBound.y < b2_huge);

	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	B2_ASSERT(b2IsLeaf(tree->nodes + proxyId));

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
	*outFatAABB = fatAABB;

	b2InsertLeaf(tree, proxyId);

	bool alreadyMoved = tree->nodes[proxyId].moved;

	if (alreadyMoved)
	{
		return false;
	}

	tree->nodes[proxyId].moved = true;
	return true;
}

bool b2DynamicTree_EnlargeProxy(b2DynamicTree* tree, int32_t proxyId, b2AABB aabb, b2AABB* outFatAABB)
{
	b2TreeNode* nodes = tree->nodes;

	B2_ASSERT(-b2_huge < aabb.lowerBound.x && aabb.lowerBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.lowerBound.y && aabb.lowerBound.y < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.x && aabb.upperBound.x < b2_huge);
	B2_ASSERT(-b2_huge < aabb.upperBound.y && aabb.upperBound.y < b2_huge);

	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	B2_ASSERT(b2IsLeaf(tree->nodes + proxyId));

	if (b2AABB_Contains(nodes[proxyId].aabb, aabb))
	{
		return false;
	}

	b2AABB fatAABB;
	b2Vec2 r = {b2_aabbExtension, b2_aabbExtension};
	fatAABB.lowerBound = b2Sub(aabb.lowerBound, r);
	fatAABB.upperBound = b2Add(aabb.upperBound, r);
	nodes[proxyId].aabb = fatAABB;
	nodes[proxyId].enlarged = true;

	int32_t parentIndex = nodes[proxyId].parent;
	while (parentIndex != B2_NULL_INDEX)
	{
		bool changed = b2AABB_Enlarge(&nodes[parentIndex].aabb, fatAABB);
		nodes[parentIndex].enlarged = true;
		parentIndex = nodes[parentIndex].parent;

		if (changed == false)
		{
			break;
		}
	}

	while (parentIndex != B2_NULL_INDEX)
	{
		nodes[parentIndex].enlarged = true;
		parentIndex = nodes[parentIndex].parent;
	}

	*outFatAABB = fatAABB;

	bool alreadyMoved = nodes[proxyId].moved;
	if (alreadyMoved)
	{
		return false;
	}

	nodes[proxyId].moved = true;
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
	B2_ASSERT(0 <= nodeId && nodeId < tree->nodeCapacity);
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

#if B2_VALIDATE
static void b2ValidateStructure(const b2DynamicTree* tree, int32_t index)
{
	if (index == B2_NULL_INDEX)
	{
		return;
	}

	if (index == tree->root)
	{
		B2_ASSERT(tree->nodes[index].parent == B2_NULL_INDEX);
	}

	const b2TreeNode* node = tree->nodes + index;

	int32_t child1 = node->child1;
	int32_t child2 = node->child2;

	if (b2IsLeaf(node))
	{
		B2_ASSERT(child1 == B2_NULL_INDEX);
		B2_ASSERT(child2 == B2_NULL_INDEX);
		B2_ASSERT(node->height == 0);
		return;
	}

	B2_ASSERT(0 <= child1 && child1 < tree->nodeCapacity);
	B2_ASSERT(0 <= child2 && child2 < tree->nodeCapacity);

	B2_ASSERT(tree->nodes[child1].parent == index);
	B2_ASSERT(tree->nodes[child2].parent == index);

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
		B2_ASSERT(child1 == B2_NULL_INDEX);
		B2_ASSERT(child2 == B2_NULL_INDEX);
		B2_ASSERT(node->height == 0);
		return;
	}

	B2_ASSERT(0 <= child1 && child1 < tree->nodeCapacity);
	B2_ASSERT(0 <= child2 && child2 < tree->nodeCapacity);

	int32_t height1 = tree->nodes[child1].height;
	int32_t height2 = tree->nodes[child2].height;
	int32_t height;
	height = 1 + B2_MAX(height1, height2);
	B2_ASSERT(node->height == height);

	b2AABB aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);

	B2_ASSERT(aabb.lowerBound.x == node->aabb.lowerBound.x);
	B2_ASSERT(aabb.lowerBound.y == node->aabb.lowerBound.y);
	B2_ASSERT(aabb.upperBound.x == node->aabb.upperBound.x);
	B2_ASSERT(aabb.upperBound.y == node->aabb.upperBound.y);

	uint32_t categoryBits = tree->nodes[child1].categoryBits | tree->nodes[child2].categoryBits;
	B2_ASSERT(node->categoryBits == categoryBits);

	b2ValidateMetrics(tree, child1);
	b2ValidateMetrics(tree, child2);
}
#endif

void b2DynamicTree_Validate(const b2DynamicTree* tree)
{
#if B2_VALIDATE
	b2ValidateStructure(tree, tree->root);
	b2ValidateMetrics(tree, tree->root);

	int32_t freeCount = 0;
	int32_t freeIndex = tree->freeList;
	while (freeIndex != B2_NULL_INDEX)
	{
		B2_ASSERT(0 <= freeIndex && freeIndex < tree->nodeCapacity);
		freeIndex = tree->nodes[freeIndex].next;
		++freeCount;
	}

	int32_t height = b2DynamicTree_GetHeight(tree);
	int32_t computedHeight = b2DynamicTree_ComputeHeight(tree);
	B2_ASSERT(height == computedHeight);

	B2_ASSERT(tree->nodeCount + freeCount == tree->nodeCapacity);
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

		B2_ASSERT(b2IsLeaf(node) == false);

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
				B2_ASSERT(stackCount <= b2_treeStackSize - 2);
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
				B2_ASSERT(stackCount <= b2_treeStackSize - 2);
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
			B2_ASSERT(value >= 0.0f);

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
			B2_ASSERT(stackCount <= b2_treeStackSize - 2);
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

// Median split == 0, Surface area heurstic == 1
#define B2_TREE_HEURISTIC 0

#if B2_TREE_HEURISTIC == 0

// Median split heuristic
static int32_t b2PartitionMid(int32_t* indices, b2Vec2* centers, int32_t count)
{
	// Handle trivial case
	if (count <= 2)
	{
		return count / 2;
	}

	// TODO_ERIN SIMD?
	b2Vec2 lowerBound = centers[0];
	b2Vec2 upperBound = centers[0];

	for (int32_t i = 1; i < count; ++i)
	{
		lowerBound = b2Min(lowerBound, centers[i]);
		upperBound = b2Max(upperBound, centers[i]);
	}

	b2Vec2 d = b2Sub(upperBound, lowerBound);
	b2Vec2 c = {0.5f * (lowerBound.x + upperBound.x), 0.5f * (lowerBound.y + upperBound.y)};

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int32_t i1 = 0, i2 = count;
	if (d.x > d.y)
	{
		float pivot = c.x;

		while (i1 < i2)
		{
			while (i1 < i2 && centers[i1].x < pivot)
			{
				i1 += 1;
			};

			while (i1 < i2 && centers[i2 - 1].x >= pivot)
			{
				i2 -= 1;
			};

			if (i1 < i2)
			{
				// Swap indices
				{
					int32_t temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		float pivot = c.y;

		while (i1 < i2)
		{
			while (i1 < i2 && centers[i1].y < pivot)
			{
				i1 += 1;
			};

			while (i1 < i2 && centers[i2 - 1].y >= pivot)
			{
				i2 -= 1;
			};

			if (i1 < i2)
			{
				// Swap indices
				{
					int32_t temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	B2_ASSERT(i1 == i2);

	if (i1 > 0 && i1 < count)
	{
		return i1;
	}
	else
	{
		return count / 2;
	}
}

#else

#define B2_BIN_COUNT 8

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
// Returns the left child count
static int32_t b2PartitionSAH(int32_t* indices, int32_t* binIndices, b2AABB* boxes, int32_t count)
{
	B2_ASSERT(count > 0);

	b2TreeBin bins[B2_BIN_COUNT];
	b2TreePlane planes[B2_BIN_COUNT - 1];

	b2Vec2 center = b2AABB_Center(boxes[0]);
	b2AABB centroidAABB;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for (int32_t i = 1; i < count; ++i)
	{
		center = b2AABB_Center(boxes[i]);
		centroidAABB.lowerBound = b2Min(centroidAABB.lowerBound, center);
		centroidAABB.upperBound = b2Max(centroidAABB.upperBound, center);
	}

	b2Vec2 d = b2Sub(centroidAABB.upperBound, centroidAABB.lowerBound);

	// Find longest axis
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

	// Initialize bin bounds and count
	for (int32_t i = 0; i < B2_BIN_COUNT; ++i)
	{
		bins[i].aabb.lowerBound = (b2Vec2){FLT_MAX, FLT_MAX};
		bins[i].aabb.upperBound = (b2Vec2){-FLT_MAX, -FLT_MAX};
		bins[i].count = 0;
	}

	// Assign boxes to bins and compute bin boxes
	// TODO_ERIN optimize
	float binCount = B2_BIN_COUNT;
	float lowerBoundArray[2] = {centroidAABB.lowerBound.x, centroidAABB.lowerBound.y};
	float minC = lowerBoundArray[axisIndex];
	for (int32_t i = 0; i < count; ++i)
	{
		b2Vec2 c = b2AABB_Center(boxes[i]);
		float cArray[2] = {c.x, c.y};
		int32_t binIndex = (int32_t)(binCount * (cArray[axisIndex] - minC) * invD);
		binIndex = B2_CLAMP(binIndex, 0, B2_BIN_COUNT - 1);
		binIndices[i] = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b2AABB_Union(bins[binIndex].aabb, boxes[i]);
	}

	int32_t planeCount = B2_BIN_COUNT - 1;

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for (int32_t i = 1; i < planeCount; ++i)
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b2AABB_Union(planes[i - 1].leftAABB, bins[i].aabb);
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for (int32_t i = planeCount - 2; i >= 0; --i)
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b2AABB_Union(planes[i + 1].rightAABB, bins[i + 1].aabb);
	}

	// Find best split to minimize SAH
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

	// Partition node indices and boxes using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int32_t i1 = 0, i2 = count;
	while (i1 < i2)
	{
		while (i1 < i2 && binIndices[i1] < bestPlane)
		{
			i1 += 1;
		};

		while (i1 < i2 && binIndices[i2 - 1] >= bestPlane)
		{
			i2 -= 1;
		};

		if (i1 < i2)
		{
			// Swap indices
			{
				int32_t temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
			}

			// Swap boxes
			{
				b2AABB temp = boxes[i1];
				boxes[i1] = boxes[i2 - 1];
				boxes[i2 - 1] = temp;
			}

			i1 += 1;
			i2 -= 1;
		}
	}
	B2_ASSERT(i1 == i2);
	
	if (i1 > 0 && i1 < count)
	{
		return i1;
	}
	else
	{
		return count / 2;
	}
}

#endif

// Temporary data used to track the rebuild of a tree node
struct b2RebuildItem
{
	int32_t nodeIndex;
	int32_t childCount;

	// Leaf indices
	int32_t startIndex;
	int32_t splitIndex;
	int32_t endIndex;
};

// Returns root node index
static int32_t b2BuildTree(b2DynamicTree* tree, int32_t leafCount)
{
	b2TreeNode* nodes = tree->nodes;
	int32_t* leafIndices = tree->leafIndices;

	if (leafCount == 1)
	{
		nodes[leafIndices[0]].parent = B2_NULL_INDEX;
		return leafIndices[0];
	}

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
	int32_t* binIndices = tree->binIndices;
#endif

	struct b2RebuildItem stack[b2_treeStackSize];
	int32_t top = 0;

	stack[0].nodeIndex = b2AllocateNode(tree);
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
#if B2_TREE_HEURISTIC == 0
	stack[0].splitIndex = b2PartitionMid(leafIndices, leafCenters, leafCount);
#else
	stack[0].splitIndex = b2PartitionSAH(leafIndices, binIndices, leafBoxes, leafCount);
#endif

	while (true)
	{
		struct b2RebuildItem* item = stack + top;

		item->childCount += 1;

		if (item->childCount == 2)
		{
			// This internal node has both children established

			if (top == 0)
			{
				// all done
				break;
			}

			struct b2RebuildItem* parentItem = stack + (top - 1);
			b2TreeNode* parentNode = nodes + parentItem->nodeIndex;

			if (parentItem->childCount == 0)
			{
				B2_ASSERT(parentNode->child1 == B2_NULL_INDEX);
				parentNode->child1 = item->nodeIndex;
			}
			else
			{
				B2_ASSERT(parentItem->childCount == 1);
				B2_ASSERT(parentNode->child2 == B2_NULL_INDEX);
				parentNode->child2 = item->nodeIndex;
			}

			b2TreeNode* node = nodes + item->nodeIndex;

			B2_ASSERT(node->parent == B2_NULL_INDEX);
			node->parent = parentItem->nodeIndex;

			B2_ASSERT(node->child1 != B2_NULL_INDEX);
			B2_ASSERT(node->child2 != B2_NULL_INDEX);
			b2TreeNode* child1 = nodes + node->child1;
			b2TreeNode* child2 = nodes + node->child2;

			node->aabb = b2AABB_Union(child1->aabb, child2->aabb);
			node->height = 1 + B2_MAX(child1->height, child2->height);
			node->categoryBits = child1->categoryBits | child2->categoryBits;

			// Pop stack
			top -= 1;
		}
		else
		{
			int32_t startIndex, endIndex;
			if (item->childCount == 0)
			{
				startIndex = item->startIndex;
				endIndex = item->splitIndex;
			}
			else
			{
				B2_ASSERT(item->childCount == 1);
				startIndex = item->splitIndex;
				endIndex = item->endIndex;
			}

			int32_t count = endIndex - startIndex;

			if (count == 1)
			{
				int32_t childIndex = leafIndices[startIndex];
				b2TreeNode* node = nodes + item->nodeIndex;

				if (item->childCount == 0)
				{
					B2_ASSERT(node->child1 == B2_NULL_INDEX);
					node->child1 = childIndex;
				}
				else
				{
					B2_ASSERT(item->childCount == 1);
					B2_ASSERT(node->child2 == B2_NULL_INDEX);
					node->child2 = childIndex;
				}

				b2TreeNode* childNode = nodes + childIndex;
				B2_ASSERT(childNode->parent == B2_NULL_INDEX);
				childNode->parent = item->nodeIndex;
			}
			else
			{
				B2_ASSERT(count > 0);
				B2_ASSERT(top < b2_treeStackSize);

				top += 1;
				struct b2RebuildItem* newItem = stack + top;
				newItem->nodeIndex = b2AllocateNode(tree);
				newItem->childCount = -1;
				newItem->startIndex = startIndex;
				newItem->endIndex = endIndex;
#if B2_TREE_HEURISTIC == 0
				newItem->splitIndex = b2PartitionMid(leafIndices + startIndex, leafCenters + startIndex, count);
#else
				newItem->splitIndex = b2PartitionSAH(leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count);
#endif
				newItem->splitIndex += startIndex;
			}
		}
	}

	b2TreeNode* rootNode = nodes + stack[0].nodeIndex;
	B2_ASSERT(rootNode->parent == B2_NULL_INDEX);
	B2_ASSERT(rootNode->child1 != B2_NULL_INDEX);
	B2_ASSERT(rootNode->child2 != B2_NULL_INDEX);

	b2TreeNode* child1 = nodes + rootNode->child1;
	b2TreeNode* child2 = nodes + rootNode->child2;

	rootNode->aabb = b2AABB_Union(child1->aabb, child2->aabb);
	rootNode->height = 1 + B2_MAX(child1->height, child2->height);
	rootNode->categoryBits = child1->categoryBits | child2->categoryBits;

	return stack[0].nodeIndex;
}

// Not safe to access tree during this operation because it may grow
int32_t b2DynamicTree_Rebuild(b2DynamicTree* tree, bool fullBuild)
{
	int32_t proxyCount = tree->proxyCount;
	if (proxyCount == 0)
	{
		return 0;
	}

	// Ensure capacity for rebuild space
	if (proxyCount > tree->rebuildCapacity)
	{
		int32_t newCapacity = proxyCount + proxyCount / 2;

		b2Free(tree->leafIndices, tree->rebuildCapacity * sizeof(int32_t));
		tree->leafIndices = b2Alloc(newCapacity * sizeof(int32_t));

#if B2_TREE_HEURISTIC == 0
		b2Free(tree->leafCenters, tree->rebuildCapacity * sizeof(b2Vec2));
		tree->leafCenters = b2Alloc(newCapacity * sizeof(b2Vec2));
#else
		b2Free(tree->leafBoxes, tree->rebuildCapacity * sizeof(b2AABB));
		tree->leafBoxes = b2Alloc(newCapacity * sizeof(b2AABB));
		b2Free(tree->binIndices, tree->rebuildCapacity * sizeof(int32_t));
		tree->binIndices = b2Alloc(newCapacity * sizeof(int32_t));
#endif
		tree->rebuildCapacity = newCapacity;
	}

	int32_t leafCount = 0;
	int32_t stack[b2_treeStackSize];
	int32_t stackCount = 0;

	int32_t nodeIndex = tree->root;
	b2TreeNode* nodes = tree->nodes;
	b2TreeNode* node = nodes + nodeIndex;

	// These are the nodes that get sorted to rebuild the tree.
	// I'm using indices because the node pool may grow during the build.
	int32_t* leafIndices = tree->leafIndices;

#if B2_TREE_HEURISTIC == 0
	b2Vec2* leafCenters = tree->leafCenters;
#else
	b2AABB* leafBoxes = tree->leafBoxes;
#endif

	// Gather all proxy nodes that have grown and all internal nodes that haven't grown. Both are
	// considered leaves in the tree rebuild.
	// Free all internal nodes that have grown.
	while (true)
	{
		if (node->height == 0 || (node->enlarged == false && fullBuild == false))
		{
			leafIndices[leafCount] = nodeIndex;
#if B2_TREE_HEURISTIC == 0
			leafCenters[leafCount] = b2AABB_Center(node->aabb);
#else
			leafBoxes[leafCount] = node->aabb;
#endif
			leafCount += 1;

			// Detach
			node->parent = B2_NULL_INDEX;
			node->enlarged = false;
		}
		else
		{
			int32_t doomedNodeIndex = nodeIndex;

			// Handle children
			nodeIndex = node->child1;

			B2_ASSERT(stackCount < b2_treeStackSize);
			if (stackCount < b2_treeStackSize)
			{
				stack[stackCount++] = node->child2;
			}

			node = nodes + nodeIndex;

			// Remove doomed node
			b2FreeNode(tree, doomedNodeIndex);

			continue;
		}

		if (stackCount == 0)
		{
			break;
		}

		nodeIndex = stack[--stackCount];
		node = nodes + nodeIndex;
	}

	B2_ASSERT(leafCount <= proxyCount);

	tree->root = b2BuildTree(tree, leafCount);

	b2DynamicTree_Validate(tree);

	return leafCount;
}

// TODO_ERIN test this as inlined
int32_t b2DynamicTree_GetUserData(const b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].userData;
}

// TODO_ERIN test this as inlined
bool b2DynamicTree_WasMoved(const b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].moved;
}

// TODO_ERIN test this as inlined
void b2DynamicTree_ClearMoved(b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	tree->nodes[proxyId].moved = false;
}

// TODO_ERIN test this as inlined
b2AABB b2DynamicTree_GetFatAABB(const b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].aabb;
}

// TODO_ERIN test this as inlined
uint32_t b2DynamicTree_GetCategoryBits(const b2DynamicTree* tree, int32_t proxyId)
{
	B2_ASSERT(0 <= proxyId && proxyId < tree->nodeCapacity);
	return tree->nodes[proxyId].categoryBits;
}
