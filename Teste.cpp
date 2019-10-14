#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

//Predição
//X = [x, y, dx, dy]
//A = [1, 0, T, 0,
//     0, 1, 0, T,
//     0, 0, 1, 0
//     0, 0, 0, 1]
//B = [T^2/2, T^2/2, T, T]
//C = [1, 0, 0, 0,
//     0, 1, 0, 0]
//Ez = [alpha^2, 0,
//      0, beta^2]
//alpha variância no x
//beta variância no y
//Ex = [T^4/4, 0, T^3/2, 0,
//      0, T^4/4, 0, T^3/2,
//      T^3/2, 0, T^2, 0,
//      0, T^3/2, 0, T^2]
//Xk = AX(k-1) + Bu(k-1) + Ex
//Z = C*Xk + Ez
//Ordem das operações
//Xk = A*Xk-1
//P = A*P*A.transpose() + Ex;
//K = P*C.transpose()*(C*p*C.transpose() + Ez).inverse();
//se tiver lido ai, Xk += K*(lido - C*Xk);
//P = (I(4) - K*C)*P;

int main(){

  return 0;
}
