#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;
int H_MIN = 0;
int S_MIN = 0;
int V_MIN = 0;
int H_MAX = 256;
int S_MAX = 256;
int V_MAX = 256;
void on_trackbar(int, void*){

}
void morphismo(Mat& thresh){
  Mat erodeelement = getStructuringElement(MORPH_RECT, Size(3,3));
  Mat dilateElemen = getStructuringElement(MORPH_RECT, Size(8,8));
  erode(thresh, thresh, erodeelement);
  erode(thresh, thresh, erodeelement);
  erode(thresh, thresh, erodeelement);

  dilate(thresh, thresh, erodeelement);
}
void createTrackbars(){
	//create window for trackbars


    namedWindow("trackbar",0);
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
    createTrackbar( "H_MIN", "trackbar", &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", "trackbar", &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", "trackbar", &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", "trackbar", &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", "trackbar", &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", "trackbar", &V_MAX, V_MAX, on_trackbar );


}
int main( int argc, char** argv ) {
  
  Mat image, yuvimage, cinzado;
  VideoCapture frame;
  frame.open(0);
  frame.set(CAP_PROP_FRAME_WIDTH, 640);
  frame.set(CAP_PROP_FRAME_HEIGHT, 480);
  createTrackbars();
  while(1){
    frame.read(image);
    cvtColor(image, yuvimage, COLOR_BGR2HSV);
    inRange(yuvimage, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), cinzado);
    morphismo(cinzado);
    imshow("yuv", yuvimage);
    imshow("cinzado", cinzado);
    waitKey(30);
  }
  waitKey(0);
  return 0;
}
