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

wdp::Offset
wop::determinePixelOffsetOfLocation(Location Loc,
                                    const wdp::DepthParameters &DepthParams) {
  // When projected onto the camera plane, DepthParams.FocalLength is the
  // effective z axis value, so the other axes can be scaled accordingly.
  double Scale = (double)Loc.Z / DepthParams.FocalLength;

  double TrueXCentreOffset = Scale == 0 ? 0 : (double)Loc.X / Scale;
  double TrueYCentreOffset = Scale == 0 ? 0 : (double)Loc.Y / Scale;

  std::ptrdiff_t XCentreOffset =
      round(TrueXCentreOffset / DepthParams.PixelScale);
  std::ptrdiff_t YCentreOffset =
      round(TrueYCentreOffset / DepthParams.PixelScale);

  return wdp::Offset(XCentreOffset, YCentreOffset);
}

Location
wop::determineLocationOfPixelOffset(double Depth, wdp::Offset CentreOffset,
                                    const wdp::DepthParameters &DepthParams) {
  double TrueXCentreOffset = CentreOffset.x * DepthParams.PixelScale;
  double TrueYCentreOffset = CentreOffset.y * DepthParams.PixelScale;

  // Since we don't yet know the real z axis value of the location we must use
  // the hypotenuse of the offset as projected onto the camera plane to compare
  // to the depth, and scale the axes accordingly.
  double Hypotenuse = sqrt(TrueXCentreOffset * TrueXCentreOffset +
                           TrueYCentreOffset * TrueYCentreOffset +
                           DepthParams.FocalLength * DepthParams.FocalLength);

  double Scale = Depth / Hypotenuse;

  std::ptrdiff_t X = round(TrueXCentreOffset * Scale);
  std::ptrdiff_t Y = round(TrueXCentreOffset * Scale);
  std::ptrdiff_t Z = round(DepthParams.FocalLength * Scale);

  return Location(X, Y, Z);
}
