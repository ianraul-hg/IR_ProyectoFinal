#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>

float K_pro=0, T_der=0, T_int=0, T_muestra=0.02;

//Variables control eje X
float paraX_K0=0, paraX_K1=0, paraX_K2=0;
float error_X=0, error_X1=0, error_X2=0, control_X1=0, control_X=0;

//Variables control eje Y
float paraY_K0=0, paraY_K1=0, paraY_K2=0;
float error_Y=0, error_Y1=0, error_Y2=0, control_Y1=0, control_Y=0;

//Ganancias controlador
K_pro = 0.015;
T_der = 5;    
T_int = 0;

int main (int argc, char** argv)
{
cv::Mat imagen_entrada;
cv::Mat imagen_salida;
cv::Mat img_hsv;
cv::Mat img_ent;

cv::namedWindow("Imagen", cv::WINDOW_AUTOSIZE);

int columnas = 0;
int filas = 0;

int lowH = 0;
int lowS = 0;
int lowV = 0;

int highH = 179;
int highS = 255;
int highV = 255;

if(argc !=2)
{
    std::cout << "comando correcto: ./Proyecto image.jpg\n";
    return -1;
}

imagen_entrada = cv::imread(argv[1], cv::IMREAD_COLOR);

cv::resize(imagen_entrada,img_ent, cv::Size(1100,200));

cv::createTrackbar("LowH", "Imagen", &lowH, 179);
cv::createTrackbar("LowS", "Imagen", &lowS, 255);
cv::createTrackbar("LowV", "Imagen", &lowV, 255);

cv::createTrackbar("HighH", "Imagen", &highH, 179);
cv::createTrackbar("HighS", "Imagen", &highS, 255);
cv::createTrackbar("HighV", "Imagen", &highV, 255);

while(1)
{
//Centroide Frame
int columnas = img_ent.cols;
int filas = img_ent.rows;


//Segmentacion
cvtColor(img_ent, img_hsv, cv::COLOR_BGR2HSV);
inRange(img_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imagen_salida);

erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));


//Centroide Objeto

cv::Moments moment = cv::moments(imagen_salida, true);

int cx = int(moment.m10/moment.m00);
int cy = int(moment.m01/moment.m00);

cv::circle(imagen_salida, cv::Point (cx, cy), 6, (0,255,0),-1);


//Controlador de eje X
paraX_K0 = K_pro + (K_pro * T_der/T_muestra) + (K_pro * T_muestra/T_int);
paraX_K1 = K_pro - (2*K_pro*T_der/T_muestra);
paraX_K2 = K_pro * T_der/T_muestra;

error_X = cx - (columnas/2);
control_X = control_X1 + paraX_K0*error_X + paraX_K1*error_X1 + paraX_K2*error_X2;

control_X1 = control_X;
error_X2 = error_X1;
error_X1 = error_X;
//////////////////////////////////////////

//Controlador de eje Y
paraY_K0 = K_pro + (K_pro * T_der/T_muestra) + (K_pro * T_muestra/T_int);
paraY_K1 = K_pro - (2*K_pro*T_der/T_muestra);
paraY_K2 = K_pro * T_der/T_muestra;

error_Y = cy - (filas/2);
control_Y = control_Y1 + paraY_K0*error_Y + paraY_K1*error_Y1 + paraY_K2*error_Y2;

control_Y1 = control_Y;
error_Y2 = error_Y1;
error_Y1 = error_Y;
///////////////////////////////////////////


imshow("Imagen", imagen_salida);
cv::waitKey(40);
}

return 0;
}

/// Correr programa ./imagen /home/student/Downloads/images/0000000208.png