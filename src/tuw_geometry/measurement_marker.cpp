#include <tuw_geometry/measurement_marker.h>
using namespace tuw;

MeasurementMarker::MeasurementMarker()
    :Measurement ( Type::MARKER ) {
}
MeasurementMarker::MeasurementMarker ( const MeasurementMarker& o )
    :Measurement ( o ), range_max_id_ ( o.range_max_id_ ), range_max_ ( o.range_max_ ), range_min_ ( o.range_min_ ), markers_ ( o.markers_ ) {
}
void MeasurementMarker::resize ( size_t n ) {
    markers_.resize ( n );
}
double &MeasurementMarker::angle_min() {
    return angle_min_;
}
double &MeasurementMarker::angle_max() {
    return angle_max_;
}
double &MeasurementMarker::range_min() {
    return range_min_;
}
const double &MeasurementMarker::range_min() const {
    return range_min_;
}
double &MeasurementMarker::range_max() {
    return range_max_;
}
const double &MeasurementMarker::range_max() const {
    return range_max_;
}
double &MeasurementMarker::range_max_id() {
    return range_max_id_;
}
const double &MeasurementMarker::range_max_id() const {
    return range_max_id_;
}
double &MeasurementMarker::sigma_radial() {
    return sigma_radial_;
}
const double &MeasurementMarker::sigma_radial() const {
    return sigma_radial_;
}
double &MeasurementMarker::sigma_polar() {
    return sigma_polar_;
}
const double &MeasurementMarker::sigma_polar() const {
    return sigma_polar_;
}
double &MeasurementMarker::sigma_azimuthal() {
    return sigma_azimuthal_;
}
const double &MeasurementMarker::sigma_azimuthal() const {
    return sigma_azimuthal_;
}
double &MeasurementMarker::sigma_roll() {
    return sigma_roll_;
}
const double &MeasurementMarker::sigma_roll() const {
    return sigma_roll_;
}
double &MeasurementMarker::sigma_pitch() {
    return sigma_pitch_;
}
const double &MeasurementMarker::sigma_pitch() const {
    return sigma_pitch_;
}
double &MeasurementMarker::sigma_yaw() {
    return sigma_yaw_;
}
const double &MeasurementMarker::sigma_yaw() const {
    return sigma_yaw_;
}
bool MeasurementMarker::empty() const {
    return markers_.empty();
}
size_t MeasurementMarker::size() const {
    return markers_.size();
}
/** Array operation
 * @param i entry to return
 * @return reference to the element
 **/
MeasurementMarker::Marker& MeasurementMarker::operator[] ( int i ) {
    return markers_[i];
}
/** Array operation const version
 * @param i entry to return
 * @return reference to the element
 **/
const MeasurementMarker::Marker& MeasurementMarker::operator[] ( int i ) const {
    return markers_[i];
}

