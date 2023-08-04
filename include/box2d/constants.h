// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/// @file
/// Constants used by box2d.
/// box2d uses meters-kilograms-seconds (MKS) units. Angles are always in radians unless
/// degrees are indicated.
/// Some values can be overridden with a define and some values can be modified at runtime.
/// Other values cannot be modified without causing stability and/or performance problems.

/// box2d bases all length units on meters, but you may need different units for your game.
/// You can adjust this value to use different units, normally at application startup.
extern float b2_lengthUnitsPerMeter;

#define b2_pi 3.14159265359f

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
/// @warning modifying this can have a significant impact on performance
#define b2_aabbMargin (0.1f * b2_lengthUnitsPerMeter)

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant. In meters.
/// @warning modifying this can have a significant impact on stability
#define b2_linearSlop (0.005f * b2_lengthUnitsPerMeter)

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
/// @warning modifying this can have a significant impact on stability
#define b2_angularSlop (2.0f / 180.0f * b2_pi)

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
/// You may define this externally.
#ifndef b2_maxPolygonVertices
#define b2_maxPolygonVertices 8
#endif

/// Maximum number of simultaneous worlds that can be allocated
/// You may define this externally.
#ifndef b2_maxWorlds
#define b2_maxWorlds 32
#endif

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot. Meters.
/// @warning modifying this can have a significant impact on stability
#define b2_maxLinearCorrection (0.2f * b2_lengthUnitsPerMeter)

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
/// @warning modifying this can have a significant impact on stability
#define b2_maxAngularCorrection (8.0f / 180.0f * b2_pi)

/// The maximum linear translation of a body per step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this. Meters.
/// @warning modifying this can have a significant impact on stability
#define b2_maxTranslation (20.0f * b2_lengthUnitsPerMeter)
#define b2_maxTranslationSquared (b2_maxTranslation * b2_maxTranslation)

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
/// @warning modifying this can have a significant impact on stability
#define b2_maxRotation (0.5f * b2_pi)
#define b2_maxRotationSquared (b2_maxRotation * b2_maxRotation)

/// TODO_ERIN make dynamic based on speed?
/// @warning modifying this can have a significant impact on stability
#define b2_speculativeDistance (4.0f * b2_linearSlop)

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
/// @warning modifying this can have a significant impact on stability
#define b2_baumgarte 0.2f

/// The time that a body must be still before it will go to sleep.
extern float b2_timeToSleep;

/// A body cannot sleep if its linear velocity is above this tolerance.
#ifndef b2_linearSleepTolerance
#define b2_linearSleepTolerance (0.01f * b2_lengthUnitsPerMeter)
#endif

/// A body cannot sleep if its angular velocity is above this tolerance.
#ifndef b2_angularSleepTolerance
#define b2_angularSleepTolerance (2.0f / 180.0f * b2_pi)
#endif

/// Used to detect bad values. Positions greater than about 16km will have precision
/// problems, so 100km as a limit should be fine in all cases.
#define b2_huge (100000.0f * b2_lengthUnitsPerMeter)

/// Maximum parallel workers. Used to size some static arrays.
#define b2_maxWorkers 64

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

#ifdef __cplusplus
}
#endif
