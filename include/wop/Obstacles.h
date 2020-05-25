//===-------- Obstacles.h - Locating Obstacles ------------------*- C++ -*-===//
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
// This file defines functionality to locate obstacles occupying the space
// surrounding the camera using stereo images.
//
//===----------------------------------------------------------------------===//

#ifndef WOP_OCCUPANCY_H
#define WOP_OCCUPANCY_H

#include "Location.h"
#include "OccupancyMap.h"

#include "wdp/Coordinates.h"
#include "wdp/Depth.h"

#include <boost/concept_check.hpp>
#include <boost/gil/image.hpp>

#include <cstddef>

namespace wop {

/// Given the coordinates to search around, determine the location in space of
/// the obstacle at those coordinates.
template <typename LHSImage, typename RHSImage>
Location locateObstacleAtCoordinates(const LHSImage &LHSImg,
                                     const RHSImage &RHSImg, wdp::Coordinates C,
                                     const wdp::SearchParameters &SearchParams,
                                     const wdp::DepthParameters &DepthParams) {
  // LHSImg and RHSImg input arguments must be 2D images.
  boost::function_requires<boost::gil::RandomAccess2DImageConcept<LHSImage>>();
  boost::function_requires<boost::gil::RandomAccess2DImageConcept<RHSImage>>();

  double Depth = wdp::getDepth(LHSImg, RHSImg, C, SearchParams, DepthParams);
  wdp::Offset CentreOffset = wdp::getCentreOffset(LHSImg, C);

  return determineLocationOfPixelOffset(Depth, CentreOffset, DepthParams);
}

/// Fill the occupancy map by tracing out the locations of obstacles visible to
/// the camera. Locations of obstacles are determined such that if all obstacles
/// were at the edge of the visible range then all cells on that face would be
/// covered by the search.
template <std::size_t XMax, std::size_t YMax, std::size_t ZMax,
          std::size_t CellSize, typename LHSImage, typename RHSImage>
bool locateObstacles(OccupancyMap<XMax, YMax, ZMax, CellSize> &OM,
                     const LHSImage &LHSImg, const RHSImage &RHSImg,
                     const wdp::SearchParameters &SearchParams,
                     const wdp::DepthParameters &DepthParams) {
  // LHSImg and RHSImg input arguments must be 2D images.
  boost::function_requires<boost::gil::RandomAccess2DImageConcept<LHSImage>>();
  boost::function_requires<boost::gil::RandomAccess2DImageConcept<RHSImage>>();

  std::ptrdiff_t _XMax = XMax;
  for (std::ptrdiff_t X = -_XMax; X < _XMax; X += CellSize) {
    std::ptrdiff_t _YMax = YMax;
    for (std::ptrdiff_t Y = -_YMax; Y < _YMax; Y += CellSize) {

      Location Loc(X, Y, ZMax);

      // Find which pixel coordinates need to be searched.
      wdp::Offset CentreOffset =
          determinePixelOffsetOfLocation(Loc, DepthParams);
      wdp::Coordinates C = wdp::getCoordinates(LHSImg, CentreOffset);

      // Locate the obstacle at the pixel coordinates and update occupancy map.
      Location ObstacleLoc = locateObstacleAtCoordinates(
          LHSImg, RHSImg, C, SearchParams, DepthParams);

      // Some obstacles are too far away to place on the occupancy map.
      if (!(ObstacleLoc.X < (std::ptrdiff_t)XMax &&
            ObstacleLoc.X >= -((std::ptrdiff_t)XMax)) ||
          !(ObstacleLoc.Y < (std::ptrdiff_t)YMax &&
            ObstacleLoc.Y >= -((std::ptrdiff_t)YMax)) ||
          !(ObstacleLoc.Z >= -((std::ptrdiff_t)ZMax)))
        continue;

      OM[ObstacleLoc] = CellState::Occupied;
    }
  }
  return true;
}

} // namespace wop

#endif // WOP_OCCUPANCY_H
