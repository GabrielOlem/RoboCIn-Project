#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv ) {
  
  Mat image, yuvimage, cinzado;
  image = imread("d.jpg");
  cvtColor(image, yuvimage, COLOR_BGR2YUV);
  medianBlur(yuvimage, cinzado, 15);
  imshow("yuv", yuvimage);
  imshow("cinzado", cinzado);
  waitKey(0);
  return 0;
}
