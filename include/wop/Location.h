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

/// Determine the offset from the centre of an image of the pixel where an
/// obstacle at the given location would appear.
wdp::Offset
determinePixelOffsetOfLocation(Location Loc,
                               const wdp::DepthParameters &DepthParams);

/// Determine the location of an obstacle at a given depth, detected at a
/// pixel which is offset from the centre of an image by a given amount.
Location
determineLocationOfPixelOffset(double Depth, wdp::Offset CentreOffset,
                               const wdp::DepthParameters &DepthParams);

} // namespace wop

#endif // WOP_LOCATION_H
