//===-------- Location.h - Operating on Locations for Mapping ---*- C++ -*-===//
//
// Copyright © 2020 Lewis Revill
//
// Use, modification and distribution are subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE.txt or copy at
// https://www.boost.org/LICENSE_1_0.txt).
//
// SPDX-License-Identifier: BSL-1.0
//
//===----------------------------------------------------------------------===//
//
// This file defines functionality for operating on and working with the
// locations of obstacles in 3D space.
//
//===----------------------------------------------------------------------===//

#ifndef WOP_LOCATION_H
#define WOP_LOCATION_H

#include "wdp/Coordinates.h"

#include <cstddef>

// Forward declare types in wdp namespace.
namespace wdp {
struct DepthParameters;
} // namespace wdp

namespace wop {

/// A location of an object relative to the camera. The camera is pointing
/// toward the positive Z axis of a typical right-hand-rule axes system, with
/// the Y axis oriented downwards.
struct Location {
  std::ptrdiff_t X;
  std::ptrdiff_t Y;
  std::ptrdiff_t Z;

  Location(std::ptrdiff_t _X, std::ptrdiff_t _Y, std::ptrdiff_t _Z)
      : X(_X), Y(_Y), Z(_Z) {}
};

/// A location of an object relative to the camera, represented using a depth
/// and angles. The inclination represents the angle to the z axis, and rotation
/// represents the rotation around the x/y plane, from the positive x axis.
struct PolarLocation {
  double Depth;
  double Inclination;
  double Rotation;

  PolarLocation(double _Depth, double _Inclination, double _Rotation)
      : Depth(_Depth), Inclination(_Inclination), Rotation(_Rotation) {}
};

/// Convert a location represented with depth and angles to a location
/// represented with X, Y and Z values.
Location convertToLocation(PolarLocation P);

/// Convert a location represented with X, Y and Z values to a location
/// represented with depth and angles.
PolarLocation convertToPolarLocation(Location Loc);

/// Determine the offset from the centre of an image of the pixel where an
/// obstacle at the given location would appear.
wdp::Offset
determinePixelOffsetOfLocation(PolarLocation P,
                               const wdp::DepthParameters &DepthParams);

/// Determine the polar location of an obstacle at a given depth, detected at a
/// pixel which is offset from the centre of an image by a given amount.
PolarLocation
determineLocationOfPixelOffset(double Depth, wdp::Offset CentreOffset,
                               const wdp::DepthParameters &DepthParams);

} // namespace wop

#endif // WOP_LOCATION_H
