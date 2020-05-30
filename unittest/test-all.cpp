#include "wop/Location.h"
#include "wop/OccupancyMap.h"

#include "wdp/Depth.h"

#include <cassert>
#include <cmath>

// Helper function to determine if two floating point numbers are equal enough
// that the difference is likely to be due to floating point error.
bool approxEqual(double a, double b) {
  return std::abs(a - b) < 0.0000001;
}

void testDeterminePixelOffsetOfLocation() {
  wdp::DepthParameters DepthParams = {24.0, 0.00694, 100};

  wop::Location Loc1(0, 0, 2400);
  wdp::Offset CentreOffset1 = wop::determinePixelOffsetOfLocation(Loc1, DepthParams);
  assert(CentreOffset1.x == 0 && CentreOffset1.y == 0);

  wop::Location Loc2(694, 694, 2400);
  wdp::Offset CentreOffset2 = wop::determinePixelOffsetOfLocation(Loc2, DepthParams);
  assert(CentreOffset2.x == 1000 && CentreOffset2.y == 1000);

  wop::Location Loc3(-694, -694, 2400);
  wdp::Offset CentreOffset3 = wop::determinePixelOffsetOfLocation(Loc3, DepthParams);
  assert(CentreOffset3.x == -1000 && CentreOffset3.y == -1000);

  wop::Location Loc4(-69, -69, 240);
  wdp::Offset CentreOffset4 = wop::determinePixelOffsetOfLocation(Loc4, DepthParams);
  assert(CentreOffset4.x == -994 && CentreOffset4.y == -994);

  wop::Location Loc5(-7, -7, 24);
  wdp::Offset CentreOffset5 = wop::determinePixelOffsetOfLocation(Loc5, DepthParams);
  assert(CentreOffset5.x == -1009 && CentreOffset5.y == -1009);

  wop::Location Loc6(0, 0, 0);
  wdp::Offset CentreOffset6 = wop::determinePixelOffsetOfLocation(Loc6, DepthParams);
  assert(CentreOffset6.x == 0 && CentreOffset6.y == 0);
}

void testDetermineLocationOfPixelOffset() {
  wdp::DepthParameters DepthParams = {24.0, 0.00694, 100};

  wdp::Offset CentreOffset1(0, 0);
  wop::Location Loc1 = wop::determineLocationOfPixelOffset(2400, CentreOffset1, DepthParams);
  assert(Loc1.X == 0 && Loc1.Y == 0 && Loc1.Z == 2400);

  wdp::Offset CentreOffset2(1000, 1000);
  wop::Location Loc2 = wop::determineLocationOfPixelOffset(2593, CentreOffset2, DepthParams);
  assert(Loc2.X == 694 && Loc2.Y == 694 && Loc2.Z == 2400);

  wdp::Offset CentreOffset3(-1000, -1000);
  wop::Location Loc3 = wop::determineLocationOfPixelOffset(2593, CentreOffset3, DepthParams);
  assert(Loc3.X == -694 && Loc3.Y == -694 && Loc3.Z == 2400);

  wdp::Offset CentreOffset4(-994, -994);
  wop::Location Loc4 = wop::determineLocationOfPixelOffset(259, CentreOffset4, DepthParams);
  assert(Loc4.X == -69 && Loc4.Y == -69 && Loc4.Z == 240);

  wdp::Offset CentreOffset5(-1009, -1009);
  wop::Location Loc5 = wop::determineLocationOfPixelOffset(26, CentreOffset5, DepthParams);
  assert(Loc5.X == -7 && Loc5.Y == -7 && Loc5.Z == 24);
}

void testLocation() {
  testDeterminePixelOffsetOfLocation();
  testDetermineLocationOfPixelOffset();
}

void testOccupancyMapSubscriptOperator() {
  wop::OccupancyMap<5000, 100, 10000> OM;

  // All cells should initially be unoccupied.
  assert(OM[wop::Location(0, 0, 1000)] == wop::CellState::Unoccupied);

  // Set cell to occupied.
  OM[wop::Location(0, 0, 1000)] = wop::CellState::Occupied;
  assert(OM[wop::Location(0, 0, 1000)] == wop::CellState::Occupied);

  // These indexes should be within the range of the occupancy map.

  OM[wop::Location(-5000, -100, 0)] = wop::CellState::Occupied;
  assert(OM[wop::Location(-5000, -100, 0)] == wop::CellState::Occupied);

  OM[wop::Location(4999, 99, 9999)] = wop::CellState::Occupied;
  assert(OM[wop::Location(4999, 99, 9999)] == wop::CellState::Occupied);
}

void testOccupancyMap() {
  testOccupancyMapSubscriptOperator();
}

int main() {
  testLocation();
  testOccupancyMap();

  return 0;
}