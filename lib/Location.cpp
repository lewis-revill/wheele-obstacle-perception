//===-------- Location.cpp - Operating on Locations for Mapping -*- C++ -*-===//
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
// This file implements functionality for operating on and working with the
// locations of obstacles in 3D space.
//
//===----------------------------------------------------------------------===//

#include "wop/Location.h"

#include "wdp/Depth.h"

#include <cmath>

using namespace wop;

Location wop::convertToLocation(PolarLocation P) {
  std::ptrdiff_t X = round(P.Depth * cos(P.Rotation) * sin(P.Inclination));
  std::ptrdiff_t Y = round(P.Depth * sin(P.Rotation) * sin(P.Inclination));

  std::ptrdiff_t Z = round(P.Depth * cos(P.Inclination));

  return Location(X, Y, Z);
}

PolarLocation wop::convertToPolarLocation(Location Loc) {
  double Depth = sqrt(Loc.X * Loc.X + Loc.Y * Loc.Y + Loc.Z * Loc.Z);

  double Inclination = acos((double)Loc.Z / Depth);
  double Rotation = atan2(Loc.Y, Loc.X);

  return PolarLocation(Depth, Inclination, Rotation);
}

wdp::Offset
wop::determinePixelOffsetOfLocation(PolarLocation P,
                                    const wdp::DepthParameters &DepthParams) {
  // When projected onto the camera plane, DepthParams.FocalLength is the
  // effective z axis value, while the given depth is to be ignored.
  double TrueXCentreOffset =
      DepthParams.FocalLength * cos(P.Rotation) * tan(P.Inclination);
  double TrueYCentreOffset =
      DepthParams.FocalLength * sin(P.Rotation) * tan(P.Inclination);

  std::ptrdiff_t XCentreOffset =
      round(TrueXCentreOffset / DepthParams.PixelScale);
  std::ptrdiff_t YCentreOffset =
      round(TrueYCentreOffset / DepthParams.PixelScale);

  return wdp::Offset(XCentreOffset, YCentreOffset);
}

PolarLocation
wop::determineLocationOfPixelOffset(double Depth, wdp::Offset CentreOffset,
                                    const wdp::DepthParameters &DepthParams) {
  double TrueXCentreOffset = CentreOffset.x * DepthParams.PixelScale;
  double TrueYCentreOffset = CentreOffset.y * DepthParams.PixelScale;

  // Since we don't yet know the real z axis value of the location we must use
  // the length of the opposite side of the triangle and the focal length to
  // calculate the inclination angle, which corresponds to the real inclination
  // from the z axis due to similar triangles..
  double OppositeLength = sqrt(TrueXCentreOffset * TrueXCentreOffset +
                               TrueYCentreOffset * TrueYCentreOffset);

  double Inclination = atan(OppositeLength / DepthParams.FocalLength);
  double Rotation = atan2(TrueYCentreOffset, TrueXCentreOffset);

  return PolarLocation(Depth, Inclination, Rotation);
}
