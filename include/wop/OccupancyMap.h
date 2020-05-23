//===-------- OccupancyMap.h - Represent Occupancy of Space -----*- C++ -*-===//
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
// This file defines OccupancyMap to represent the occupancy of the space
// surrounding the camera.
//
//===----------------------------------------------------------------------===//

#ifndef WOP_OCCUPANCYMAP_H
#define WOP_OCCUPANCYMAP_H

#include "Location.h"

#include <cassert>
#include <cstddef>

namespace wop {

/// State representing occupancy of a given cell.
enum CellState { Unoccupied, Occupied };

/// Class representing the occupancy of the space surrounding the camera.
/// Occupancy state is stored for each cell within the area covered by the axis.
/// The camera looks down the negative z axis so there are no cells for the
/// positive x axis.
template <std::size_t XMax, std::size_t YMax, std::size_t ZMax,
          std::size_t CellSize = 100UL>
class OccupancyMap {
private:
  CellState Cells[2 * ((XMax + CellSize - 1) / CellSize)]
                 [2 * ((YMax + CellSize - 1) / CellSize)]
                 [(ZMax + CellSize - 1) / CellSize];

public:
  CellState &operator[](Location Loc) {
    assert(Loc.X < (std::ptrdiff_t)XMax && Loc.X >= -((std::ptrdiff_t)XMax));
    assert(Loc.Y < (std::ptrdiff_t)YMax && Loc.Y >= -((std::ptrdiff_t)YMax));
    assert(Loc.Z < (std::ptrdiff_t)ZMax && Loc.Z >= 0);

    std::size_t XIndex = (Loc.X + XMax) / CellSize;
    std::size_t YIndex = (Loc.Y + YMax) / CellSize;
    std::size_t ZIndex = Loc.Z / CellSize;

    return Cells[ZIndex][YIndex][XIndex];
  }

  CellState operator[](Location Loc) const {
    assert(Loc.X < (std::ptrdiff_t)XMax && Loc.X >= -((std::ptrdiff_t)XMax));
    assert(Loc.Y < (std::ptrdiff_t)YMax && Loc.Y >= -((std::ptrdiff_t)YMax));
    assert(Loc.Z < (std::ptrdiff_t)ZMax && Loc.Z >= 0);

    std::size_t XIndex = (Loc.X + XMax) / CellSize;
    std::size_t YIndex = (Loc.Y + YMax) / CellSize;
    std::size_t ZIndex = Loc.Z / CellSize;

    return Cells[ZIndex][YIndex][XIndex];
  }
};

} // namespace wop

#endif // WOP_OCCUPANCYMAP_H
