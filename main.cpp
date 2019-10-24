#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <pthread.h>
#include <bits/stdc++.h>

#define NUM_OBJECTS 2

using namespace cv;
using namespace Eigen;
using namespace std;
struct object{
  Matrix<double, 4, 1> teste;
  MatrixXd X, A, B, C, Ez, Ex, K, lido, P;
  int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;
  double aream, areama;
};
Mat yuvimage;

object objetos[NUM_OBJECTS];


//bola laranja 110 to 169  31 to 87 and 0 to 255
//bola amarela 208 to 236
//bola vermelha 52 to 75 84 to 120 and 188 to 210

Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3));
Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8,8));

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}
void createTrackbars(int i){
	//create window for trackbars


    namedWindow("trackbarWindowName",0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", objetos[i].H_MIN);
	sprintf( TrackbarName, "H_MAX", objetos[i].H_MAX);
	sprintf( TrackbarName, "S_MIN", objetos[i].S_MIN);
	sprintf( TrackbarName, "S_MAX", objetos[i].S_MAX);
	sprintf( TrackbarName, "V_MIN", objetos[i].V_MIN);
	sprintf( TrackbarName, "V_MAX", objetos[i].V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", "trackbarWindowName", &objetos[i].H_MIN, objetos[i].H_MAX, on_trackbar );
    createTrackbar( "H_MAX", "trackbarWindowName", &objetos[i].H_MAX, objetos[i].H_MAX, on_trackbar );
    createTrackbar( "S_MIN", "trackbarWindowName", &objetos[i].S_MIN, objetos[i].S_MAX, on_trackbar );
    createTrackbar( "S_MAX", "trackbarWindowName", &objetos[i].S_MAX, objetos[i].S_MAX, on_trackbar );
    createTrackbar( "V_MIN", "trackbarWindowName", &objetos[i].V_MIN, objetos[i].V_MAX, on_trackbar );
    createTrackbar( "V_MAX", "trackbarWindowName", &objetos[i].V_MAX, objetos[i].V_MAX, on_trackbar );


}
void init(int i){
  objetos[i].X.resize(4, 1);
  objetos[i].A.resize(4, 4);
  objetos[i].B.resize(4, 1);
  objetos[i].C.resize(2, 4);
  objetos[i].Ez.resize(2, 2);
  objetos[i].Ex.resize(4, 4);
  objetos[i].K.resize(4, 4);
  objetos[i].lido.resize(4, 1);
  objetos[i].P.resize(4, 4);

  objetos[i].A << 1,0,1,0,
       0,1,0,1,
       0,0,1,0,
       0,0,0,1;
  objetos[i].B << 0.5,0.5,1,1;
  objetos[i].C << 1,0,0,0,
       0,1,0,0;
  objetos[i].Ez << 0.0025, 0,
        0, 0.0025;
  objetos[i].Ex << 1/4,0,1/2,0,
        0,1/4,0,1/2,
        1/2,0,1,0,
        0,1/2,0,1;
  objetos[i].X << 0,0,0,0;
  objetos[i].lido << 0,0,0,0;
  if(i == 0){
    objetos[i].H_MIN = 109; //5 52
    objetos[i].H_MAX = 138;//19 75
    objetos[i].S_MIN = 86;//138 84
    objetos[i].S_MAX = 101;//204 120
    objetos[i].V_MIN = 0;//251 188
    objetos[i].V_MAX = 255;//256 210
    objetos[i].aream = 900;
    objetos[i].areama = 1150;
  }
  if(i == 1){
    objetos[i].H_MIN = 110; //5 52
    objetos[i].H_MAX = 169;//19 75
    objetos[i].S_MIN = 31;//138 84
    objetos[i].S_MAX = 87;//204 120
    objetos[i].V_MIN = 0;//251 188
    objetos[i].V_MAX = 255;//256 210
    objetos[i].aream = 2500;
    objetos[i].areama = 4500;
  }
}
void *op(void *n){
  int numero = *((int*) n);
  Mat cinzado;
  inRange(yuvimage, Scalar(objetos[numero].H_MIN, objetos[numero].S_MIN, objetos[numero].V_MIN), Scalar(objetos[numero].H_MAX, objetos[numero].S_MAX, objetos[numero].V_MAX), cinzado);
  
  erode(cinzado, cinzado, erodeElement);
  erode(cinzado, cinzado, erodeElement);
    
  dilate(cinzado, cinzado, dilateElement);
  dilate(cinzado, cinzado, dilateElement);
  Mat temp;
  cinzado.copyTo(temp);
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
  bool objectFound = false;
  int numObjects = 0;
  objetos[numero].X = objetos[numero].A*objetos[numero].X;
  objetos[numero].P = objetos[numero].A*objetos[numero].P*objetos[numero].A.transpose() + objetos[numero].Ex;
  objetos[numero].K = objetos[numero].P*objetos[numero].C.transpose()*(objetos[numero].C*objetos[numero].P*objetos[numero].C.transpose() + objetos[numero].Ez).inverse();
  if(hierarchy.size() > 0){
    numObjects = hierarchy.size();
  }
  printf("Numero de objetos %i\n", numObjects);
  if(numObjects < 5 && numObjects > 0){
    for (int index = 0; index >= 0; index = hierarchy[index][0]) {

      Moments moment = moments((cv::Mat)contours[index]);
      double area = moment.m00;

      printf("Area %lf\n", area);
      if(area>objetos[numero].aream && area<objetos[numero].areama){
        double a, b;
        a = objetos[numero].lido(0);
        b = objetos[numero].lido(1);
        objetos[numero].lido(0) = moment.m10/area;
        objetos[numero].lido(1) = moment.m01/area;
        objetos[numero].lido(2) = objetos[numero].lido(0) - a;
        objetos[numero].lido(3) = objetos[numero].lido(1) - b;
        objectFound = true;
      }
      else objectFound = false;


    }
    //let user know you found an object
    if(objectFound == true){
      MatrixXd teste(2,1);
      teste(0) = objetos[numero].lido(0);
      teste(1) = objetos[numero].lido(1);
      objetos[numero].X = objetos[numero].X + objetos[numero].K*(teste - objetos[numero].C*objetos[numero].X);
    }
    else{
      objetos[numero].lido(0) = -1;
    }
    
  }
  else{
    cout << "TOO MUCH NOISE! ADJUST FILTER" << endl;
  }
  MatrixXd I(4,4);
  I << 1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;
  objetos[numero].P = (I - objetos[numero].K*objetos[numero].C)*objetos[numero].P;
  pthread_exit(NULL);
}
int main( int argc, char** argv ) {
  for(int i=0; i<NUM_OBJECTS; i++){
    init(i);
  }
  Mat image, frame;
  VideoCapture print("robocin.webm");
  //createTrackbars();
  print.read(image);
  imshow("a", image);
  while(1){
    cvtColor(image, yuvimage, COLOR_BGR2YUV);
    pthread_t threads[NUM_OBJECTS];
    
    for(int i=0; i<NUM_OBJECTS; i++){
      int *n = new int;
      *n = i;
      pthread_create(&threads[i], NULL, op, (void*)n);
    }
    for(int i=0; i<NUM_OBJECTS; i++){
      pthread_join(threads[i], NULL);
    }
    for(int i=0; i<NUM_OBJECTS; i++){
      if(objetos[i].lido(0) != -1){
        circle(image,Point(objetos[i].lido(0),objetos[i].lido(1)),30,Scalar(0,255,0),2);
      }
      circle(image, Point(objetos[i].X(0), objetos[i].X(1)), 20, Scalar(0, 0, 255), 5);
    }
    
    imshow("a", image);
    if(waitKey(0) == 27){
      print.read(image);
    }
  }
  waitKey(0);
  return 0;
}
