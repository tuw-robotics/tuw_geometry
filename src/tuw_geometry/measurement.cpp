#include <tuw_geometry/measurement.h>

using namespace tuw;

Measurement::Measurement ( Measurement::Type type )
    : type_ ( type ) {};

Measurement::Measurement ( const Measurement &o )
    : type_ ( o.type_ ), stamp_ ( o.stamp_ ), tf_ ( o.tf_ ) {
}
Measurement::Type Measurement::getType() const {
    return type_;
}
const std::string Measurement::getTypeName() const {
    switch ( type_ ) {
    case Type::LASER:
        return "LASER";
    case Type::LINE:
        return "LINE";
    }
    return "NA";
}
const cv::Matx33d& Measurement::tf() const {
    return tf_;
}
cv::Matx33d& Measurement::tf() {
    return tf_;
}

const boost::posix_time::ptime& Measurement::stamp() const {
    return stamp_;
}
boost::posix_time::ptime& Measurement::stamp() {
    return stamp_;
}
