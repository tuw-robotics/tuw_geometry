#include <boost/python.hpp>
#include "tuw_geometry/point2d.h"

using namespace boost::python;


BOOST_PYTHON_MODULE ( tuw_geometry_wrapper ) {

    tuw::Point2D &(tuw::Point2D::*set2)(double, double)                          = &tuw::Point2D::set;
    tuw::Point2D &(tuw::Point2D::*set3)(double, double, double)                  = &tuw::Point2D::set;
    bool         &(tuw::Point2D::*inside)(double, double, double, double)        = &tuw::Point2D::inside;
    
    class_<tuw::Point2D> ( "Point2D" )
    .def ( init<double, double>() )
    .def ( init<double, double, double>())
    .def("angle", &tuw::Point2D::angle)
    .def("inside", inside)
//    .def ( "set", set2, return_value_policy<manage_new_object>() )
//    .def ( "set", set3, return_value_policy<manage_new_object>() )
    .def ( init<double, double, double>())
    .def ( "__str__", &tuw::Point2D::str);
}
