#include <opencv2/highgui.hpp>
#include <tuw_geometry/figure.hpp>
#include <tuw_geometry/geo_handler.hpp>

int main(int, char **)
{
  std::string filename_jgw("ws02/src/tuw_geo/tuw_geo_map/config/tuw_geo_map/straden/mapimage.jgw");
  std::string filename_jpg("ws02/src/tuw_geo/tuw_geo_map/config/tuw_geo_map/straden/mapimage.jpg");
  tuw::WorldFile worldfile;
  worldfile.read_jgw(filename_jgw);
  cv::Mat view = cv::imread(filename_jpg);

  tuw::GeoHdl geo_hdl;
  geo_hdl.init(
    view.size(), worldfile.resolution_x, geo_hdl.TOP_LEFT,
    cv::Vec3d(worldfile.coordinate_x, worldfile.coordinate_y, 0), 33, true);

  cv::imshow("map", view);
  cv::waitKey(2000);
}
