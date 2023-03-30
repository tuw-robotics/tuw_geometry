#ifndef TUW_GEOMETRY__SAMPLE_HPP
#define TUW_GEOMETRY__SAMPLE_HPP

#include <tuw_geometry/pose2d.hpp>

namespace tuw
{
/**
 * Class to describe a particle used in the particle filter localization
 */
template<class T> class Sample;
template<class T> using SamplePtr      = std::shared_ptr<Sample<T>>;
template<class T> using SampleConstPtr = std::shared_ptr<Sample<T> const>;

template<class T>
class Sample : public T
{
  double weight_;      /// weight
  unsigned int idx_;   /// index (for debugging)

public:
  Sample()
  :T(), weight_(0), idx_(0) {}
  /**  copy constructor
   * @param s
   **/
  Sample(const Sample<T> & s)
  : T(s), weight_(s.weight_) {} 

  /**  constructor
   * @param o
   * @param weight
   **/
  Sample(const T & o, double weight = 0)
  : T(o), weight_(weight_) {} 

  /**  set sample
   * @param p
   * @param weight
   **/
  void set(const T & o, double weight = 0)
  {
    T::set(o), weight_ = weight;
  }

  /**  set sample
   * @param s
   **/
  void set(const T & s)
  {
    set(s, s.weight_);
  }

  /**  set sample
   * @param s
   **/
  
  void set(const SamplePtr<T> & s)
  {
    set(*s, s->weight_);
  }
  
  /** @return idx  **/
  const unsigned int & idx() const{
    return idx_;
  }
  /** @return idx  **/
  unsigned int & idx()
{
  return idx_;
}
  /** @return weight  **/
  const double & weight() const
{
  return weight_;
}
  /** @return weight  **/
  double & weight()
{
  return weight_;
}
  /** Stream extraction
   * @param os outputstream
   * @param o object
   * @return stream
   **/
  friend std::ostream & operator<<(std::ostream & os, const Sample<T> & o)
  {
    os << "[" << (Pose2D &) o << ", " << o.weight() << "]";
    return os;
  }

  static bool greater(const SamplePtr<T> & a, const SamplePtr<T> & b)
  {
    return a->weight() > b->weight();
  }

  static bool smaller(const SamplePtr<T> & a, const SamplePtr<T> & b)
  {
    return a->weight() < b->weight();
  }
};

}
#endif // TUW_GEOMETRY__SAMPLE_HPP
