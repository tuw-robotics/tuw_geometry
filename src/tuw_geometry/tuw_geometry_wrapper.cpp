#include <boost/python.hpp>
#include "tuw_geometry/point2d.h"
#include "tuw_geometry/pose2d.h"

using namespace boost::python;


BOOST_PYTHON_MODULE ( tuw_geometry_wrapper ) {

    tuw::Point2D &(tuw::Point2D::*Point2Dset2)(double, double)                          = &tuw::Point2D::set;
    tuw::Point2D &(tuw::Point2D::*Point2Dset3)(double, double, double)                  = &tuw::Point2D::set;
    
    class_<tuw::Point2D> ( "Point2D" )
    .def ( init<double, double>() )
    .def ( init<double, double, double>())
    .def( "angle", &tuw::Point2D::angle)
    .def( "inside",&tuw::Point2D::inside)
    .def( "set", Point2Dset2, return_value_policy<copy_non_const_reference>() )
    .def( "set", Point2Dset3, return_value_policy<copy_non_const_reference>() )
    .def( "__str__", &tuw::Point2D::str);
    
    
    tuw::Pose2D &(tuw::Pose2D::*Pose2Dset3)(double, double, double)                  = &tuw::Pose2D::set;
    
    class_<tuw::Pose2D> ( "Pose2D" )
    .def ( init<double, double, double>())
    .def( "set", Pose2Dset3, return_value_policy<copy_non_const_reference>() )
    .def( "__str__", &tuw::Pose2D::str);
}
