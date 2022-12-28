// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// @file
/// Constants used by box2d.
/// box2d uses meters-kilograms-seconds (MKS) units. Angles are always in radians unless
/// degrees are indicated.

/// box2d bases all length units on meters, but you may need different units for your game.
/// You can adjust this value to use different units, normally at application startup.
extern float b2_lengthUnitsPerMeter;

#define b2_pi 3.14159265359f

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
#define b2_maxManifoldPoints 2

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
#define b2_aabbExtension (0.1f * b2_lengthUnitsPerMeter)

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
#define b2_aabbMultiplier 4.0f

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant. In meters.
#define b2_linearSlop (0.005f * b2_lengthUnitsPerMeter)

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
#define b2_angularSlop (2.0f / 180.0f * b2_pi)

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
/// TODO eliminate this
#define b2_polygonRadius (2.0f * b2_linearSlop)

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2_maxPolygonVertices 8

/// Maximum number of sub-steps per contact in continuous physics simulation.
#define b2_maxSubSteps 8

/// Maximum number of contacts to be handled to solve a TOI impact.
#define b2_maxTOIContacts 32

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot. Meters.
#define b2_maxLinearCorrection (0.2f * b2_lengthUnitsPerMeter)

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
#define b2_maxAngularCorrection (8.0f / 180.0f * b2_pi)

/// The maximum linear translation of a body per step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this. Meters.
#define b2_maxTranslation (20.0f * b2_lengthUnitsPerMeter)
#define b2_maxTranslationSquared (b2_maxTranslation * b2_maxTranslation)

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
#define b2_maxRotation (0.5f * b2_pi)
#define b2_maxRotationSquared (b2_maxRotation * b2_maxRotation)

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
#define b2_baumgarte 0.2f
#define b2_toiBaumgarte 0.75f

/// The time that a body must be still before it will go to sleep.
#define b2_timeToSleep 0.5f

/// A body cannot sleep if its linear velocity is above this tolerance.
#define b2_linearSleepTolerance (0.01f * b2_lengthUnitsPerMeter)

/// A body cannot sleep if its angular velocity is above this tolerance.
#define b2_angularSleepTolerance (2.0f / 180.0f * b2_pi)

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
typedef struct b2Version
{
	///< significant changes
	int major;

	///< incremental changes
	int minor;

	///< bug fixes
	int revision;
} b2Version;

/// Current version.
extern b2Version b2_version;
