#ifndef TUW_GEOMETRY__STAMP_HPP
#define TUW_GEOMETRY__STAMP_HPP

namespace tuw
{
template<class T>
class Stamp
{
public:
  std::chrono::microseconds stamp;
  tuw::Tf2D tf;
  T data;
  friend bool operator<(const Stamp<T> & l, const Stamp<T> & r)
  {
    return l.stamp < r.stamp;
  }
  friend bool operator>(const Stamp<T> & l, const Stamp<T> & r)
  {
    return l.stamp > r.stamp;
  }
  friend bool operator==(const Stamp<T> & l, const Stamp<T> & r)
  {
    return l.stamp == r.stamp;
  }
};
template<class T>
using StampPtr = std::shared_ptr<Stamp<T>>;
template<class T>
using StampConstPtr = std::shared_ptr<Stamp<T> const>;
}
#endif // TUW_GEOMETRY__STAMP_HPP
