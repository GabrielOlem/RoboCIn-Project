#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace cv;
using namespace Eigen;
using namespace std;

MatrixXd X(4, 1);
MatrixXd A(4, 4);
MatrixXd B(4, 1);
MatrixXd C(2, 4);
MatrixXd Ez(2, 2);
MatrixXd Ex(4, 4);
MatrixXd K(4, 4);

int H_MIN = 54; //5 54
int H_MAX = 184;//19 184 
int S_MIN = 70;//138 70
int S_MAX = 88;//204 88
int V_MIN = 187;//251 187
int V_MAX = 226;//256 226

int main( int argc, char** argv ) {
  A << 1,0,1,0,
       0,1,0,1,
       0,0,1,0,
       0,0,0,1;
  B << 0.5,0.5,1,1;
  C << 1,0,0,0,
       0,1,0,0;
  Ez << 0.0025, 0,
        0, 0.0025;
  Ex << 1/4,0,1/2,0,
        0,1/4,0,1/2,
        1/2,0,1,0,
        0,1/2,0,1;
  X << 0,0,0,0;
  MatrixXd lido(4,1);
  MatrixXd P(4,4);
  lido << 0,0,0,0;
  Mat image, yuvimage, cinzado, frame;
  VideoCapture print(0);
  print.read(image);
  imshow("a", image);
  while(1){
    print.read(image);
    cvtColor(image, yuvimage, COLOR_BGR2YUV);
    inRange(yuvimage, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), cinzado);
    imshow("cinz", cinzado);
    
    Mat temp;
    cinzado.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    double refArea = 0;
    bool objectFound = false;
    int numObjects = 0;
    X = A*X;
    P = A*P*A.transpose() + Ex;
    K = P*C.transpose()*(C*P*C.transpose() + Ez).inverse();
    if(hierarchy.size() > 0){
      numObjects = hierarchy.size();
    }
    if(numObjects < 50){
      for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
        if(area>152 && area<1600 && area>=refArea){
					double a, b;
          a = lido(0);
          b = lido(1);
          lido(0) = moment.m10/area;
					lido(1) = moment.m01/area;
          lido(2) = lido(0) - a;
          lido(3) = lido(1) - b;
					objectFound = true;
					refArea = area;
				}
        else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
        MatrixXd teste(2,1);
        teste(0) = lido(0);
        teste(1) = lido(1);
        X = X + K*(teste - C*X);
				circle(image,Point(lido(0),lido(1)),10,Scalar(0,255,0),5);
      }
      circle(image, Point(X(0), X(1)), 10, Scalar(0, 0, 255), 2);
		}
    else{
      cout << "TOO MUCH NOISE! ADJUST FILTER" << endl;
    }
    MatrixXd I(4,4);
    I << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;
    P = (I - K*C)*P;
    imshow("a", image);
    waitKey(30);
  }
  waitKey(0);
  return 0;
}
