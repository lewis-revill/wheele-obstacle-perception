#include "WOPConfig.h"

#include "wop/Location.h"
#include "wop/Obstacles.h"
#include "wop/OccupancyMap.h"

#include "wdp/Coordinates.h"
#include "wdp/Depth.h"

#include <boost/gil/extension/io/jpeg_io.hpp>
#include <boost/gil/typedefs.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <iostream>
#include <string>

#define STR(X) #X
#define XSTR(X) STR(X)

namespace bpo = boost::program_options;
namespace bg = boost::gil;

int main(int argc, char **argv) {
  std::string ImageDir;
  double CameraDisplacement;

  // Declare program options.
  bpo::options_description Desc("Obstacle perception application options");
  Desc.add_options()
      ("help", "produce help message")
      ("imagedir", bpo::value<std::string>(&ImageDir),
       "name of the directory within 'data/' containing stereo images, named "
       "lhs.jpg and rhs.jpg")
      ("displacement",
       bpo::value<double>(&CameraDisplacement)->default_value(100.0),
       "Displacement between the two images");

  bpo::variables_map VM;
  bpo::store(bpo::command_line_parser(argc, argv).options(Desc).run(), VM);
  bpo::notify(VM);

  if (VM.count("help")) {
    std::cout << Desc << std::endl;
    return 1;
  }

  if (!VM.count("imagedir")) {
    std::cout << "Image directory not set - see help for details" << std::endl;
    return 1;
  }

  std::string LHSFilename =
      std::string(XSTR(WOP_DATA_DIR) "/").append(ImageDir).append("/lhs.jpg");
  std::string RHSFilename =
      std::string(XSTR(WOP_DATA_DIR) "/").append(ImageDir).append("/rhs.jpg");
  bg::rgb8_image_t LHSImg;
  bg::rgb8_image_t RHSImg;
  bg::jpeg_read_image(LHSFilename, LHSImg);
  bg::jpeg_read_image(RHSFilename, RHSImg);

  wdp::Radii NeighbourhoodRadii(15, 15);
  wdp::Offset SearchOffset(-250, 0);
  wdp::Radii SearchRadii(250, 100);

  wdp::SearchParameters SearchParams = {NeighbourhoodRadii, SearchOffset,
                                        SearchRadii, 1000000UL};

  wdp::DepthParameters DepthParams = {24.0, 0.00694, CameraDisplacement};

  wop::OccupancyMap<4800, 500, 10000> OM;
  wop::locateObstacles(OM, LHSImg, RHSImg, SearchParams, DepthParams);

  std::cout << OM[wop::Location(3, 0, 3)] << std::endl;
  return 0;
}
