#include <tgmath.h>

#include <cfloat>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/utils.hpp>
#include <tuw_geometry/map_handler.hpp>

using namespace tuw;

MapHdl::MapHdl()
: width_pixel_(-1), height_pixel_(-1), min_x_(0), max_x_(0), min_y_(0), max_y_(0)
{
}

bool MapHdl::initialized() {return (width_pixel_ != -1) && (height_pixel_ != -1);}

void MapHdl::init()
{
  dx_ = max_x_ - min_x_;
  dy_ = max_y_ - min_y_;
  sx_ = width_pixel_ / dx_;
  sy_ = height_pixel_ / dy_;
  ox_ = width_pixel_ / 2.0;
  oy_ = height_pixel_ / 2.0;
  double ca = cos(rotation_), sa = sin(rotation_);
  mx_ = min_x_ + dx_ / 2.;
  my_ = min_y_ + dy_ / 2.;
  cv::Matx<double, 3, 3> Tw(1, 0, -mx_, 0, 1, -my_, 0, 0, 1);  // translation
  cv::Matx<double, 3, 3> Sc(sx_, 0, 0, 0, sy_, 0, 0, 0, 1);    // scaling
  cv::Matx<double, 3, 3> Sp(-1, 0, 0, 0, 1, 0, 0, 0, 1);       // mirroring
  cv::Matx<double, 3, 3> R(ca, -sa, 0, sa, ca, 0, 0, 0, 1);    // rotation
  cv::Matx<double, 3, 3> Tm(1, 0, ox_, 0, 1, oy_, 0, 0, 1);    // translation
  Mw2m_ = Tm * R * Sp * Sc * Tw;

  Mm2w_ = Mw2m_.inv();
}
std::string MapHdl::info_map() const
{
  char txt[0x1FF];
  sprintf(
    txt,
    "MapInfo: [%4dpix, %4dpix] * [%6.5f m/pix,  %6.5f m/pix] = [%6.2fm, %6.2fm]; origin: [%6.2fm, %6.2fm, "
    "%4.3frad]",
    width_pixel_, height_pixel_, 1./sx_, 1./sy_, dx_, dy_, mx_, my_, rotation_);
  return txt;
}

void MapHdl::init(cv::Size canvas_size, double resolution, Origin origin){
  switch (origin)
  {
  case TOP_LEFT:
      init(canvas_size, resolution, cv::Point2d(0,0));
    break;
  case BOTTOM_LEFT:
      init(canvas_size, resolution, cv::Point2d(0, -canvas_size.height * resolution));
    break;
  case CENTER:
      init(canvas_size, resolution, cv::Point2d(canvas_size.width * resolution, -canvas_size.height * resolution)/2.);
  default:
    break;
  }
}

void MapHdl::init(cv::Size cavas_size, double resolution, cv::Point2d origin){
  sx_ = 1.0 / resolution;
  sy_ = 1.0 / resolution;
  rotation_ = 0;
  dx_ = cavas_size.width  * resolution;
  dy_ = cavas_size.height * resolution;
  ox_ = 0;
  oy_ = 0;  
  mx_ =  origin.x;
  my_ =  origin.y;
  cv::Matx<double, 3, 3> Tw(  1,   0, mx_, 
                              0,   1, my_, 
                              0,   0,   1);    // translation
  cv::Matx<double, 3, 3> Sc(sx_,   0,   0, 
                              0, sy_,   0, 
                              0,   0,   1);    // scaling
  cv::Matx<double, 3, 3> Sp(  1,   0,   0, 
                              0,  -1,   0, 
                              0,   0,   1);    // mirroring
  Mw2m_ = Sp * Sc * Tw;
  Mm2w_ = Mw2m_.inv();

  cv::Vec3d min = Mm2w_ * cv::Vec3d(0.0,0.0,1.);
  min_x_ = min[0];
  min_y_ = min[1];
  max_x_ = min_x_ + dx_;
  max_y_ = min_y_ + dy_;
}

void MapHdl::init(
  int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y,
  double rotation)
{
  width_pixel_ = width_pixel, height_pixel_ = height_pixel;
  min_y_ = std::min(min_y, max_y);
  max_y_ = std::max(min_y, max_y);
  min_x_ = std::min(min_x, max_x);
  max_x_ = std::max(min_x, max_x);
  rotation_ = rotation;

  init();
}

const cv::Matx33d & MapHdl::Mw2m() const {return Mw2m_;}
const cv::Matx33d & MapHdl::Mm2w() const {return Mm2w_;}
Point2D MapHdl::w2m(const Point2D & src) const {return Mw2m_ * src;}
Point2D MapHdl::w2m(double x, double y) const {return w2m(Point2D(x, y));}
Point2D & MapHdl::w2m(const Point2D & src, Point2D & des) const
{
  des = Mw2m_ * src;
  return des;
}
Point2D MapHdl::m2w(const Point2D & src) const {return Mm2w_ * src;}
Point2D MapHdl::m2w(double x, double y) const {return m2w(Point2D(x, y));}
Point2D & MapHdl::m2w(const Point2D & src, Point2D & des) const
{
  des = Mm2w_ * src;
  return des;
}
double MapHdl::max_x() const {return max_x_;}
double MapHdl::min_x() const {return min_x_;}
double MapHdl::scale_x() const {return sx_;}
double MapHdl::resolution_x() const {return 1./sx_;}
double MapHdl::max_y() const {return max_y_;}
double MapHdl::min_y() const {return min_y_;}
double MapHdl::scale_y() const {return sy_;}
double MapHdl::resolution_y() const {return 1./sy_;}
int MapHdl::width() const {return width_pixel_;}
int MapHdl::height() const {return height_pixel_;}
int MapHdl::origin_x() const {return mx_;}
int MapHdl::origin_y() const {return my_;}

double MapHdl::scale_w2m(double v) const {return v * sx_;}
std::string MapHdl::infoHeader() const
{
  char buffer[0x1FF];
  Point2D p0 = m2w(0, 0);
  Point2D p1 = m2w(width_pixel_ / 2, height_pixel_ / 2);
  sprintf(
    buffer,
    "%4i,%4i [px];  %6.2f, %6.2f [m] => %6.2f, %6.2f [px/m]; 0, 0 [px] = %6.2f, %6.2f [m] @ %3.2f "
    "[rad]; %4i, %4i [px] = %6.2f, %6.2f [m] @ %3.2f [rad]",
    width_pixel_, height_pixel_, dx_, dy_, sx_, sy_, p0.x(), p0.y(), rotation_, width_pixel_ / 2,
    height_pixel_ / 2, p1.x(), p1.y(), rotation_);
  return std::string(buffer);
}
