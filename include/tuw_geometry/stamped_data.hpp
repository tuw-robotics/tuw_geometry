#ifndef TUW_GEOMETRY__STAMP_DATA_HPP
#define TUW_GEOMETRY__STAMP_DATA_HPP

namespace tuw
{
template<class T>
class StampedData
{
public:
  StampedData() {}
  StampedData(const StampedData<T> & o)
  : stamp(o.stamp), tf(o.tf), info(o.info), data(o.data) {}
  StampedData(
    const std::chrono::steady_clock::time_point & stamp, const tuw::Tf2D & tf = tuw::Tf2D(),
    const std::string & info = std::string())
  : stamp(stamp), tf(tf), info(info)
  {
  }
  StampedData(
    const T & d, const std::chrono::steady_clock::time_point & stamp,
    const tuw::Tf2D & tf = tuw::Tf2D(), const std::string & info = std::string())
  : stamp(stamp), tf(tf), data(d), info(info)
  {
  }
  StampedData(const T & d, const std::string & info)
  : stamp(), tf(), data(d), info(info) {}
  StampedData(
    const T & d, const std::string & info, const std::chrono::steady_clock::time_point & stamp,
    const tuw::Tf2D & tf = tuw::Tf2D())
  : stamp(stamp), tf(tf), data(d), info(info)
  {
  }
  StampedData(const T & d)
  : data(d) {}
  std::chrono::steady_clock::time_point stamp;
  tuw::Tf2D tf;
  std::string info;
  T data;
  template<class T2>
  friend bool operator<(const StampedData<T> & l, const StampedData<T2> & r)
  {
    return l.stamp < r.stamp;
  }
  template<class T2>
  friend bool operator>(const StampedData<T> & l, const StampedData<T2> & r)
  {
    return l.stamp > r.stamp;
  }
  template<class T2>
  friend bool operator==(const StampedData<T> & l, const StampedData<T2> & r)
  {
    return l.stamp == r.stamp;
  }
  template<class T2>
  friend bool operator!=(const StampedData<T> & l, const StampedData<T2> & r)
  {
    return l.stamp != r.stamp;
  }
};
template<class T>
using StampedDataPtr = std::shared_ptr<StampedData<T>>;
template<class T>
using StampedDataConstPtr = std::shared_ptr<StampedData<T> const>;
}  // namespace tuw
#endif  // TUW_GEOMETRY__STAMP_DATA_HPP
