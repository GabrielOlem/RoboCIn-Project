#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv ) {
  
  Mat image;
  image = imread("d.jpg");
  Mat out = Mat::zeros(image.size(), image.type());
  for(int y = 0; y < image.rows; y++){
    for(int x = 0; x < image.cols; x++){
      for(int c = 0; c < image.channels(); c++){
        out.at<Vec3b>(y,x)[c] = image.at<Vec3b>(y,x)[2] - image.at<Vec3b>(y,x)[1] - image.at<Vec3b>(y,x)[0];
      }
    }
  }
  
  imshow( "Display window", image);
  imshow("Changed", out);
  waitKey(0);
  return 0;
}
