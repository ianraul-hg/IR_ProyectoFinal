#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void imgCallback(const sensor_msgs::ImageConstPtr& msg);


float T_muestra=0.02;

//Variables control eje X
float paraX_K0=0, paraX_K1=0, paraX_K2=0;
float error_X=0, error_X1=0, error_X2=0, control_X1=0, control_X=0;

//Variables control eje Y
float paraY_K0=0, paraY_K1=0, paraY_K2=0;
float error_Y=0, error_Y1=0, error_Y2=0, control_Y1=0, control_Y=0;

int columnas = 0;
int filas = 0;

int cx = 0;
int cy = 0;


int main(int argc, char** argv)
{ 
    cv::Mat imagen_entrada;
    cv::Mat imagen_salida;
    cv::Mat img_hsv;

    ros::init(argc,argv, "proyecto");
    ros::NodeHandle node_handle;


    image_transport::ImageTransport imt(node_handle);
    //Simulacion
    //image_transport::Subscriber img_sub = imt.subscribe("/robotis_op3/camera/image_raw", 1, imgCallback);

    //Robot
    image_transport::Subscriber img_sub = imt.subscribe("/usb_cam_node/image_raw", 1, imgCallback);

    ros::Publisher datos = node_handle.advertise<geometry_msgs::Point>("/signal", 1);


    while(ros::ok())
    {

    geometry_msgs::Point datosXY;
    ros::spin();

    //Segmentacion
    cvtColor(imagen_entrada, img_hsv, cv::COLOR_BGR2HSV);
    inRange(img_hsv, cv::Scalar(0, 173, 85), cv::Scalar(159, 255, 255), imagen_salida);

    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

    //Centroide Frame
    int columnas = imagen_entrada.cols;
    int filas = imagen_entrada.rows;

    //Centroide Objeto
    cv::Moments moment = cv::moments(imagen_salida, true);

    cx = int(moment.m10/moment.m00);
    cy = int(moment.m01/moment.m00);


    ///Controlador X

    float K_proX = 1;
    float T_derX = 0;    
    float  T_intX = 0;

    paraX_K0 = K_proX + (K_proX * T_derX/T_muestra) + (K_proX * T_muestra/T_intX);
    paraX_K1 = K_proX - (2*K_proX*T_derX/T_muestra);
    paraX_K2 = K_proX * T_derX/T_muestra;

    error_X = cx - (columnas/2);
    control_X = control_X1 + paraX_K0*error_X + paraX_K1*error_X1 + paraX_K2*error_X2;

    datosXY.x = 0.017 * control_X; 
     
    control_X1 = control_X;
    error_X2 = error_X1;
    error_X1 = error_X;

    ///Controlador Y

    float K_proY = 1;
    float T_derY = 0;    
    float  T_intY = 0;


    paraY_K0 = K_proY + (K_proY * T_derY/T_muestra) + (K_proY * T_muestra/T_intY);
    paraY_K1 = K_proY - (2*K_proY*T_derY/T_muestra);
    paraY_K2 = K_proY * T_derY/T_muestra;

    error_Y = cy - (filas/2);
    control_Y = control_Y1 + paraY_K0*error_Y + paraY_K1*error_Y1 + paraY_K2*error_Y2;

    datosXY.y = 0.017 * control_Y;   

    datos.publish(datosXY);

    std::cout << cx << "pos \n";

    //Reasignacion variables X
    control_X1 = control_X;
    error_X2 = error_X1;
    error_X1 = error_X;
    //Reasignacion variables X
    control_Y1 = control_Y;
    error_Y2 = error_Y1;
    error_Y1 = error_Y;


    }

    return 0;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat imagen_entrada = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("image wrong");
    }
}