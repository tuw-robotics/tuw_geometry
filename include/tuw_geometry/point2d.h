#ifndef POINT2D_H
#define POINT2D_H

#include <memory>
#include <opencv2/core/core.hpp>

namespace tuw {
typedef cv::Matx<double, 3, 3> Tf2D;

/**
 * creates a new vector by adding a single component
 * usable to homogeneous vectors
 * @param src to normalize
 * @param value
 * @return extended vector
 **/
inline cv::Vec<double, 3>  append (const cv::Vec<double, 2> &src, double value = 1.0) {
    return cv::Vec<double, 3>(src.val[0], src.val[1], value);
}

/**
 * creates a new vector by adding a single component
 * usable to homogeneous vectors
 * @param src to normalize
 * @param value
 * @return extended vector
 **/
inline cv::Vec<double, 4>  append (const cv::Vec<double, 3> &src, double value = 1.0) {
    return cv::Vec<double, 4>(src.val[0], src.val[1], src.val[2], value);
}

/**
 * normalizes an angle between min_angle and max_angle but max_angle - min_angle >= 2PI
 * @param angle to normalize
 * @param min_angle
 * @param max_angle
 * @return normalize angle
 **/
inline double angle_normalize (double angle, double min_angle = -M_PI, double max_angle = +M_PI) {
    while ( angle > max_angle ) angle -= ( 2.*M_PI );
    while ( angle < min_angle ) angle += ( 2.*M_PI );
    return angle;
}
/**
 * computes the angle difference between two angles by taking into account the circular space
 * @param alpha0
 * @param angle1
 * @return difference
 **/
inline double angle_difference(double alpha0, double angle1){
  return atan2(sin(alpha0-angle1), cos(alpha0-angle1));
}


class Point2D;  /// Prototype
class Polar2D;
using Point2DPtr = std::shared_ptr< Point2D >;
using Point2DConstPtr = std::shared_ptr< Point2D const>;

/**
 * class to represent a point using homogeneous coordinates [x, y, 1]
 **/
class Point2D : public cv::Vec<double, 3 > {
public:
    /**
     * constructor
     **/
    Point2D(); 
    /**
     * copy constructor
     * @param p source
     **/
    Point2D ( const Point2D &p );
    /**
     * constructor
     * @param p source
     **/
    Point2D ( const cv::Point &p );
    /**
     * constructor
     * @param x
     * @param y
     **/
    Point2D ( double x, double y );
    /**
     * constructor
     * @param x
     * @param y
     * @param h
     **/
    Point2D ( double x, double y, double h );
    /**
     * constructor
     * @param p
     **/
    Point2D ( const Polar2D& p );
    template<typename T> Point2D ( const cv::Vec<T,3> &p ) : cv::Vec<double,3> ( p ) {};
    template<typename T> Point2D ( const cv::Vec<T,2> &p ) : cv::Vec<double,3> ( p ) {};
    template<typename T> Point2D ( const cv::Point_<T> &p ) : cv::Vec<double,3> ( p.x,p.y,1. ) {};

    /**
     * sets values
     * @param x
     * @param y
     * @return this reference
     **/
    Point2D &set ( double x, double y );
    /**
     * sets values
     * @param x
     * @param y
     * @return this reference
     **/
    Point2D &set ( double x, double y, double h );
    /**
     * translational x component
     * @return x component
     **/
    const double &x () const;
    /**
     * translational x component
     * @return x component
     **/
    double &x ();
    /**
     * translational y component
     * @return y component
     **/
    const double &y () const;
    /**
     * translational y component
     * @return y component
     **/
    double &y ();
    /**
     * homogeneous component
     * @return rotation
     **/
    const double &h () const;
    /**
     * homogeneous component
     * @return rotation
     **/
    double &h ();
    /**
     * set funktion for x
     * @param x component
     **/
    void set_x (double v);
    /**
     * get function for x
     * @return x component
     **/
    double get_x () const;
    /**
     * set funktion for y
     * @param y component
     **/
    void set_y (double v);
    /**
     * get function for y
     * @return y component
     **/
    double get_y () const;
    /**
     * set funktion for h
     * @param h component
     **/
    void set_h (double v);
    /**
     * get function for h
     * @return h component
     **/
    double get_h () const;
    /**
     * angle form origin to point (alpha in polar space) 
     * @see radius
     * @see Polar2D
     * @return angle between -PI and +PI
     **/
    double angle () const;
    /**
     * distance to origin (rho in polar space) 
     * @see angle
     * @see Polar2D
     * @return distance
     **/
    double radius () const;
    /** 
     * vector without homogeneous component
     * @return state vector
     **/
    cv::Vec<double, 2> vector () const;
    /**
     * returns the distance to an other point
     * @return disance
     **/
    double  distanceTo ( const Point2D &p ) const;
    /**
     * returns a cv::Point_<double> reference
     * @return cv
     **/
    const cv::Point_<double>  &cv () const;
    /**
     * returns a cv::Point_<double> reference
     * @return cv
     **/
    cv::Point_<double>  &cv () ;
    /**
     * checks if a point is within a rectangle
     * @param x0 top left x
     * @param y0 top left y
     * @param x1 bottom right x
     * @param y1 bottom right y
     * @return true if inside
     **/
    bool inside ( double x0, double y0, double x1, double y1 ) const;
    /**
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/  
    friend std::ostream &operator << ( std::ostream &os, const Point2D &o ) {
        os << "[" << o.x() <<  ", " << o.y() << "]";
        return os;
    }
    /**
     * returns x and y as formated string
     * @param format using printf format
     * @return string
     **/  
    std::string str(const char* format = "[%6.4lf, %6.4lf, %6.5lf]") const;
};


};

/**
 * overloads the * operator to allow the mutlipication fo the homogeneous point class with an opencv matrix 
 * @param a
 * @param b
 * @return point
 **/
namespace cv {
template<typename _Tp> static inline tuw::Point2D operator * ( const Matx<_Tp, 3, 3>& a, const tuw::Point2D& b ) {
    Matx<_Tp, 3, 1> c ( a, b, Matx_MatMulOp() );
    return reinterpret_cast<const tuw::Point2D&> ( c );
}
}
#endif //POINT2D_H

