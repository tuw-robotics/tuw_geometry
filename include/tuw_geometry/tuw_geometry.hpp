#ifndef TUW_GEOMETRY__TUW_GEOMETRY_HPP
#define TUW_GEOMETRY__TUW_GEOMETRY_HPP

#include <tuw_geometry/command.hpp>
#include <tuw_geometry/figure.hpp>
#include <tuw_geometry/line2d.hpp>
#include <tuw_geometry/linesegment2d.hpp>
#include <tuw_geometry/point2d.hpp>
#include <tuw_geometry/polar2d.hpp>
#include <tuw_geometry/pose2d.hpp>
#include <tuw_geometry/sample2d.hpp>

#include <map>
#include <string>

namespace tuw
{
enum DistributionType { NORMAL_DISTRIBUTION = 0, UNIFORM_DISTRIBUTION = 1,
  GRID_DISTRIBUTION = 2 };
static std::map<DistributionType, std::string> DistributionTypeName;

}  // namespace tuw

#endif  // TUW_GEOMETRY__TUW_GEOMETRY_HPP
