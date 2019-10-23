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

int H_MIN = 208; //5 52
int H_MAX = 236;//19 75 
int S_MIN = 0;//138 84
int S_MAX = 255;//204 120
int V_MIN = 0;//251 188
int V_MAX = 255;//256 210
//bola laranja 52 to 75 84 to 120 and 188 to 210
//bola amarela 208 to 236
//bola vermelha 52 to 75 84 to 120 and 188 to 210
void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}
void createTrackbars(){
	//create window for trackbars


    namedWindow("trackbarWindowName",0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", "trackbarWindowName", &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", "trackbarWindowName", &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", "trackbarWindowName", &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", "trackbarWindowName", &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", "trackbarWindowName", &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", "trackbarWindowName", &V_MAX, V_MAX, on_trackbar );


}
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
  VideoCapture print("untitled.webm");
  print.read(image);
  imshow("a", image);
  //createTrackbars();
  Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3));
  Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8));
  while(1){
    cvtColor(image, yuvimage, COLOR_BGR2YUV);
    inRange(yuvimage, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), cinzado);
    erode(cinzado, cinzado, erodeElement);
    erode(cinzado, cinzado, erodeElement);
    
    dilate(cinzado, cinzado, dilateElement);
    dilate(cinzado, cinzado, dilateElement);
    
    imshow("cinz", cinzado);
    Mat temp;
    cinzado.copyTo(temp);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    bool objectFound = false;
    int numObjects = 0;
    X = A*X;
    P = A*P*A.transpose() + Ex;
    K = P*C.transpose()*(C*P*C.transpose() + Ez).inverse();
    if(hierarchy.size() > 0){
      numObjects = hierarchy.size();
    }
    printf("Numero de objetos %i\n", numObjects);
    if(numObjects < 5 && numObjects > 0){
      for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

        printf("Area %lf\n", area);
        if(area>3300 && area<4000){
					double a, b;
          a = lido(0);
          b = lido(1);
          lido(0) = moment.m10/area;
					lido(1) = moment.m01/area;
          lido(2) = lido(0) - a;
          lido(3) = lido(1) - b;
					objectFound = true;
				}
        else objectFound = false;


			}
			//let user know you found an object
			if(objectFound == true){
        MatrixXd teste(2,1);
        teste(0) = lido(0);
        teste(1) = lido(1);
        X = X + K*(teste - C*X);
				circle(image,Point(lido(0),lido(1)),30,Scalar(0,255,0),2);
      }
      
		}
    else{
      cout << "TOO MUCH NOISE! ADJUST FILTER" << endl;
    }
    circle(image, Point(X(0), X(1)), 20, Scalar(0, 0, 255), 5);
    MatrixXd I(4,4);
    I << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;
    P = (I - K*C)*P;
    imshow("a", image);
    if(waitKey(0) == 27){
      print.read(image);
    }
  }
  waitKey(0);
  return 0;
}
