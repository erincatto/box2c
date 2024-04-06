// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef struct b2ContactList
{
	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;
} b2ContactList;

typedef struct b2ShapeList
{
	int headShapeId;
	int shapeCount;
} b2ShapeList;

typedef struct b2ChainList
{
	int headChainId;
} b2ChainList;

typedef struct b2JointList
{
	// [31 : contactId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;
} b2JointList;
