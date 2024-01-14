// Copyright 2022 Markus Bader
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Markus Bader nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tuw_geometry/tuw_geometry.hpp"

TEST(Plane3D, intersectionLine)
{
  tuw::Plane3D planeA;
  planeA.create(cv::Vec3d(0, 0, 0), cv::Vec3d(1, 0, 0), cv::Vec3d(0, 1, 0));

  cv::Vec3d pi;
  planeA.intersectionLine(cv::Vec3d(1, 1, 1), cv::Vec3d(-1, -1, -1), pi);

  ASSERT_TRUE(pi[0] == 0);
  ASSERT_TRUE(pi[1] == 0);
  ASSERT_TRUE(pi[2] == 0);


  tuw::Plane3D planeB;
  planeB.create(cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1));
  planeB.intersectionLine(cv::Vec3d(4, 5, 1), cv::Vec3d(2, 3, -1), pi);

  ASSERT_TRUE(pi[0] == 3);
  ASSERT_TRUE(pi[1] == 4);
  ASSERT_TRUE(pi[2] == 0);

}

TEST(VectorCV, Carsting1)
{
  cv::Vec<double, 4> a(1, 2, 3, 4);
  cv::Vec<double, 3> b(5, 6, 7);
  // std::cout << "a: " << a << ", b: " << b << std::endl;
  cv::Vec<double, 3> *c = reinterpret_cast<cv::Vec<double, 3>*>(&a);
  //cv::Vec<double, 3> & c = (cv::Vec<double, 3> &)a;
  *c = b;
  // std::cout << "col: " << c.cols << ", rows: " << c.rows << std::endl;
  // std::cout << "a: " << a << ", b: " << b << std::endl;

  ASSERT_TRUE(a[0] == b[0]);
  ASSERT_TRUE(a[1] == b[1]);
  ASSERT_TRUE(a[2] == b[2]);
}

TEST(Point2D, TestDistanceTo)
{
  tuw::Point2D p0(0.0, 0.0);
  tuw::Point2D p1(1.0, 0.0);
  tuw::Point2D p2(1.0, 1.0);
  ASSERT_EQ(1, p0.distanceTo(p1));
  EXPECT_NEAR(1.41421, p0.distanceTo(p2), 0.001);

  cv::Point_<double> & p0_cv = p0.cv();
  cv::Point_<double> & p1_cv = p1.cv();

  double d = p1_cv.x - p0_cv.x;
  ASSERT_EQ(1.0, d);
}
TEST(Point2D, CastToCVPoint)
{
  tuw::Point2D p0(0.0, 0.0);
  tuw::Point2D p1(1.0, 0.0);

  cv::Point_<double> & p0_cv = p0.cv();
  cv::Point_<double> & p1_cv = p1.cv();

  double dx = p1_cv.x - p0_cv.x;
  double dy = p1_cv.y - p0_cv.y;
  ASSERT_EQ(1.0, dx);
  ASSERT_EQ(0.0, dy);
}

TEST(StampedData, TestCompare)
{
  using namespace std::chrono_literals;
  auto t = std::chrono::steady_clock::now();
  tuw::StampedData<tuw::Point2D> p0(tuw::Point2D(0.0, 0.0));
  p0.stamp = t;
  tuw::StampedData<tuw::Point2D> p1(tuw::Point2D(1.0, 0.0));
  p1.stamp = t + 1ms;
  tuw::StampedData<tuw::Point2D> p2(tuw::Point2D(1.0, 1.0));
  p2.stamp = t;
  ASSERT_NE(p0, p1);
  ASSERT_EQ(p0, p2);
  ASSERT_TRUE(p0 < p1);
  ASSERT_TRUE(p1 > p0);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
