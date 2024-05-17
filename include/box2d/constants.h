// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/**
 * @defgroup constants Constants
 * @brief All the constants Box2D uses internally.
 * Constants used by box2d.
 * box2d uses meters-kilograms-seconds (MKS) units. Angles are always in radians unless
 * degrees are indicated.
 * Some values can be overridden by using a compiler definition.
 * Other values cannot be modified without causing stability and/or performance problems.
 * Although most of these are not user configurable, it can be interesting for a user to see
 * these to understand the tuning values Box2D uses.
 * @{
 */

#ifdef BOX2D_USER_CONSTANTS
	#include "user_constants.h"
#endif

/// box2d bases all length units on meters, but you may need different units for your game.
/// You can override this value to use different units.
#ifndef b2_lengthUnitsPerMeter
	#define b2_lengthUnitsPerMeter 1.0f
#endif

/// https://en.wikipedia.org/wiki/Pi
#define b2_pi 3.14159265359f

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant. In meters.
/// @warning modifying this can have a significant impact on stability
#define b2_linearSlop (0.005f * b2_lengthUnitsPerMeter)

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
/// @warning modifying this can have a significant impact on stability
///	todo not used
#define b2_angularSlop (2.0f / 180.0f * b2_pi)

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
///	don't use more vertices.
#ifndef b2_maxPolygonVertices
	#define b2_maxPolygonVertices 8
#endif

/// Maximum number of simultaneous worlds that can be allocated
#define b2_maxWorlds 128

/// The maximum translation of a body per time step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this. Meters.
/// @warning modifying this can have a significant impact on stability
#define b2_maxTranslation (4.0f * b2_lengthUnitsPerMeter)

/// The maximum rotation of a body per time step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
/// @warning modifying this can have a significant impact on stability
#define b2_maxRotation (0.25f * b2_pi)

/// @warning modifying this can have a significant impact on performance and stability
#define b2_speculativeDistance (4.0f * b2_linearSlop)

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
/// @warning modifying this can have a significant impact on performance
#define b2_aabbMargin (0.1f * b2_lengthUnitsPerMeter)

/// The time that a body must be still before it will go to sleep. In seconds.
#define b2_timeToSleep 0.5f

/// Used to detect bad values. Positions greater than about 16km will have precision
/// problems, so 100km as a limit should be fine in all cases.
#define b2_huge (100000.0f * b2_lengthUnitsPerMeter)

/// Maximum parallel workers. Used to size some static arrays.
#define b2_maxWorkers 64

/// Maximum number of colors in the constraint graph. Constraints that cannot
///	find a color are added to the overflow set which are solved single-threaded.
#define b2_graphColorCount 12

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
static const b2Version b2_version = {3, 0, 0};

/**@}*/
