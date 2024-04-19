#include <opencv2/highgui.hpp>
#include <tuw_geometry/figure.hpp>

int main(int, char **)
{
  tuw::Figure figure_local_("example");
  figure_local_.init(600, 400, -3, 3, -2, 2, 0, 1, 1);
  cv::imshow(figure_local_.title(), figure_local_.view());
  cv::waitKey(1000);
}
